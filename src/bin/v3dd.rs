// SPDX-License-Identifier: GPL-2.0-only
//
// v3dd: the V3D daemon on the Pi 4 and 400 (BCM2711, V3D 4.2), step 3 of
// redox-rpi#598. It owns the 3D block: it powers it up as v3dprobe does
// (step 0), keeps one V3D MMU page table, hands out buffer objects (BOs)
// mapped into it, and runs compute (CSD) jobs for its clients.
//
// The scheme is /scheme/v3d. A client opens it and talks in lines: it
// writes one command, then reads the answer.
//
//   bo BYTES                  -> ID VA         a new zeroed BO, mapped at VA
//   put ID WORD HEX...        -> ok            writes 32-bit words from word WORD
//   get ID WORD COUNT         -> HEX...        reads COUNT words from word WORD
//   fill ID WORD COUNT HEX    -> ok            sets COUNT words to HEX
//   free ID                   -> ok
//   csd CFG0 CFG1 ... CFG6    -> done US       runs one CSD job, waits for it
//   cl BCL END RCL END QMA QMS QTS
//                             -> done US bin US render US
//                                              one bin job, then one render
//                                              job (all V3D addresses: the
//                                              control lists, the tile
//                                              allocation memory and its size,
//                                              the tile state)
//   stats                     -> one line of counters
//
// Numbers are decimal or 0x hex. A failed command answers "error ...". BOs
// belong to the open file and are freed when it is closed. Jobs run one at
// a time, synchronously, with the caches handled as Linux's v3d driver does
// (v3d_gem.c v3d_invalidate_caches(), v3d_clean_caches(); v3d_sched.c
// v3d_csd_job_run()).
//
// Errors (step 3c): an MMU fault during a job (an address with no valid
// page table entry, a write to a read-only page) comes back as "error
// mmu-fault ..." with the faulting address and the unit, as Linux's
// v3d_hub_irq() logs it. A job that does not end within 1 s is stopped by
// a reset of V3D (Linux v3d_reset(): on the BCM2711 the PM reset line
// with the bridges closed, then the MMU set up again) and comes back as
// "error timeout ...". The daemon goes on in both cases.
//
// Not yet (later steps of #598): mmap of BOs, the V3D interrupt, one
// address space per client.
//
// The binner's overflow memory belongs to v3dd: 512 KiB at the start of
// every bin job, and another 512 KiB chunk each time the binner reports
// it is out of memory (OUTOMEM), as Linux's v3d_overflow_mem_work().
//
// The SoC watchdog (bcm2835_wdt) is armed around the power-up and around
// each job, as in v3dprobe: the Pi 400 in the lab has no remote power
// switch, and a GPU access that hangs the bus resets the SoC.

use std::collections::BTreeMap;
use std::fs;
use std::io::Write as _;
use std::process;
use std::thread;
use std::time::{Duration, Instant};

use common::dma::Dma;
use redox_scheme::scheme::SchemeSync;
use redox_scheme::{CallerCtx, OpenResult, Socket};
use scheme_utils::{Blocking, FpathWriter, HandleMap};
use syscall::error::{EACCES, EBADF, EINVAL, Error, Result};
use syscall::schemev2::NewFdFlags;

#[path = "v3d/client.rs"]
#[allow(dead_code)]
mod client;
#[path = "v3d/csd.rs"]
#[allow(dead_code)]
mod csd;

// Addresses as in v3dprobe (the Pi 400's devicetree).
const HUB_PHYS: usize = 0xfec0_0000;
const CORE0_PHYS: usize = 0xfec0_4000;
const PM_PHYS: usize = 0xfe10_0000;
const RPIVID_ASB_PHYS: usize = 0xfec1_1000;

const PM_GRAFX: usize = 0x10c;
const PM_V3DRSTN: u32 = 1 << 6;
const PM_POWOK: u32 = 1 << 1;
const PM_PASSWORD: u32 = 0x5a00_0000;
const PM_RSTC: usize = 0x1c;
const PM_WDOG: usize = 0x24;
const PM_WDOG_TIME_SET: u32 = 0x000f_ffff;
const PM_RSTC_WRCFG_CLR: u32 = 0xffff_ffcf;
const PM_RSTC_WRCFG_FULL_RESET: u32 = 0x0000_0020;
const PM_RSTC_RESET: u32 = 0x0000_0102;
const WATCHDOG_SECS: u32 = 10;

const ASB_V3D_S_CTRL: usize = 0x08;
const ASB_V3D_M_CTRL: usize = 0x0c;
const ASB_AXI_BRDG_ID: usize = 0x20;
const ASB_REQ_STOP: u32 = 1 << 0;
const ASB_ACK: u32 = 1 << 1;
const BCM2835_BRDG_ID: u32 = 0x6272_6467;

const V3D_HUB_IDENT1: usize = 0x0c;
const V3D_MMUC_CONTROL: usize = 0x1000;
const V3D_MMUC_CONTROL_ENABLE: u32 = 1 << 0;
const V3D_MMU_CTL: usize = 0x1200;
const V3D_MMU_CTL_ENABLES: u32 =
    (1 << 0) | (1 << 16) | (1 << 19) | (1 << 18) | (1 << 11) | (1 << 10) | (1 << 26) | (1 << 25);
const V3D_MMU_PT_PA_BASE: usize = 0x1204;
const V3D_MMU_ILLEGAL_ADDR: usize = 0x1230;
const V3D_MMU_ILLEGAL_ADDR_ENABLE: u32 = 1 << 31;
const V3D_MMU_CTL_CAP_EXCEEDED: u32 = 1 << 27;
const V3D_MMU_CTL_PT_INVALID: u32 = 1 << 20;
const V3D_MMU_CTL_WRITE_VIOLATION: u32 = 1 << 12;
const V3D_MMU_VIO_ID: usize = 0x122c;
const V3D_MMU_VIO_ADDR: usize = 0x1234;
const V3D_MMU_DEBUG_INFO: usize = 0x1238;
const V3D_HUB_INT_STS: usize = 0x0050;
const V3D_HUB_INT_CLR: usize = 0x0058;
const V3D_HUB_INT_MMU_WRV: u32 = 1 << 5;
const V3D_HUB_INT_MMU_PTI: u32 = 1 << 4;
const V3D_HUB_INT_MMU_CAP: u32 = 1 << 3;
const V3D_HUB_INT_MMU: u32 = V3D_HUB_INT_MMU_WRV | V3D_HUB_INT_MMU_PTI | V3D_HUB_INT_MMU_CAP;
const V3D_ERR_STAT: usize = 0x0f20;

const V3D_CTL_SLCACTL: usize = 0x0024;
const V3D_CTL_L2TCACTL: usize = 0x0030;
const V3D_L2TCACTL_TMUWCF: u32 = 1 << 8;
const V3D_L2TCACTL_FLM_CLEAN: u32 = 2 << 1;
const V3D_L2TCACTL_L2TFLS: u32 = 1 << 0;
const V3D_CTL_L2TFLSTA: usize = 0x0034;
const V3D_CTL_L2TFLEND: usize = 0x0038;
const V3D_CTL_INT_STS: usize = 0x0050;
const V3D_CTL_INT_CLR: usize = 0x0058;
const V3D_INT_OUTOMEM: u32 = 1 << 2;
const V3D_CLE_CT0CS: usize = 0x0100;
const V3D_CLE_CT1CS: usize = 0x0104;
const V3D_CLE_CT0CA: usize = 0x0110;
const V3D_CLE_CT1CA: usize = 0x0114;
const V3D_CLE_BFC: usize = 0x0134;
const V3D_CLE_RFC: usize = 0x0138;
const V3D_CLE_CT0QTS: usize = 0x015c;
const V3D_CLE_CT0QTS_ENABLE: u32 = 1 << 1;
const V3D_CLE_CT0QBA: usize = 0x0160;
const V3D_CLE_CT1QBA: usize = 0x0164;
const V3D_CLE_CT0QEA: usize = 0x0168;
const V3D_CLE_CT1QEA: usize = 0x016c;
const V3D_CLE_CT0QMA: usize = 0x0170;
const V3D_CLE_CT0QMS: usize = 0x0174;
const V3D_PTB_BPOA: usize = 0x0308;
const V3D_PTB_BPOS: usize = 0x030c;
const V3D_CSD_STATUS: usize = 0x0900;
const V3D_CSD_QUEUED_CFG0: usize = 0x0904;

const V3D_PTE_WRITEABLE: u32 = 1 << 29;
const V3D_PTE_VALID: u32 = 1 << 28;
const PAGE: usize = 4096;
const PT_ENTRIES: usize = 1 << 20;
/// BOs start above the first MiB (page 0 stays unmapped, as in Linux).
const VA_START: u32 = 0x10_0000;
const VA_END: u32 = 0x4000_0000;
const BO_MAX: usize = 16 << 20;
/// Binner overflow chunks, as Linux (V3D_OVERFLOW_SIZE is 512 KiB there).
const OVERFLOW_SIZE: usize = 512 * 1024;
const OVERFLOW_CHUNKS_MAX: usize = 16;

const V3D_POWER: &str = "/scheme/rpi-firmware/v3d_power";

pub struct Regs {
    base: *mut u32,
}

impl Regs {
    fn map(phys: usize, size: usize, name: &str) -> Regs {
        let base = unsafe { common::physmap(phys, size, common::Prot::RW, common::MemoryType::DeviceMemory) }
            .unwrap_or_else(|err| fail(&format!("cannot map {name} at {phys:#x}: {err:?}")));
        Regs { base: base as *mut u32 }
    }
    pub fn read(&self, offset: usize) -> u32 {
        unsafe { self.base.add(offset / 4).read_volatile() }
    }
    pub fn write(&self, offset: usize, value: u32) {
        unsafe { self.base.add(offset / 4).write_volatile(value) }
    }
}

pub fn say(line: &str) {
    println!("v3dd: {line}");
    let _ = std::io::stdout().flush();
}

fn fail(why: &str) -> ! {
    say(&format!("FAIL: {why}"));
    process::exit(1);
}

struct Watchdog<'a>(&'a Regs);

impl Watchdog<'_> {
    /// bcm2835_wdt_start()
    fn arm(&self) {
        let pm = self.0;
        pm.write(PM_WDOG, PM_PASSWORD | ((WATCHDOG_SECS << 16) & PM_WDOG_TIME_SET));
        let cur = pm.read(PM_RSTC);
        pm.write(PM_RSTC, PM_PASSWORD | (cur & PM_RSTC_WRCFG_CLR) | PM_RSTC_WRCFG_FULL_RESET);
    }
    /// bcm2835_wdt_stop()
    fn stop(&self) {
        self.0.write(PM_RSTC, PM_PASSWORD | PM_RSTC_RESET);
    }
}

/// Powers V3D up as v3dprobe does (Tests 552-557): the firmware's V3D
/// domain, the reset released, the async bridges open, master first.
fn power_up(pm: &Regs, asb: &Regs) {
    let id = asb.read(ASB_AXI_BRDG_ID);
    if id != BCM2835_BRDG_ID {
        fail(&format!("bridge id {id:#x}, not \"brdg\": not a BCM2711?"));
    }
    let mut grafx = pm.read(PM_GRAFX);
    if grafx & PM_POWOK == 0 {
        if let Err(err) = fs::write(V3D_POWER, "on") {
            fail(&format!("{V3D_POWER}: {err} (rpi-firmwared without v3d_power?)"));
        }
        thread::sleep(Duration::from_millis(5));
        grafx = pm.read(PM_GRAFX);
    }
    if grafx & PM_V3DRSTN == 0 {
        pm.write(PM_GRAFX, PM_PASSWORD | grafx | PM_V3DRSTN);
        thread::sleep(Duration::from_millis(1));
    }
    for (reg, name) in [(ASB_V3D_M_CTRL, "master"), (ASB_V3D_S_CTRL, "slave")] {
        asb.write(reg, PM_PASSWORD | (asb.read(reg) & !ASB_REQ_STOP));
        let start = Instant::now();
        while asb.read(reg) & ASB_ACK != 0 {
            if start.elapsed() > Duration::from_millis(10) {
                for r in [ASB_V3D_S_CTRL, ASB_V3D_M_CTRL] {
                    asb.write(r, PM_PASSWORD | asb.read(r) | ASB_REQ_STOP);
                }
                pm.write(PM_GRAFX, PM_PASSWORD | (pm.read(PM_GRAFX) & !PM_V3DRSTN));
                fail(&format!("ASB {name} bridge: no ack within 10 ms: bridges closed, reset asserted"));
            }
        }
    }
}

/// Gives a V3D address range back: sorted, merged with its neighbours, and
/// handed back to the top (`next_va`) when it ends there, so freed ranges
/// do not pile up (Test 600: 3 -> 23 ranges).
fn release_va(free: &mut Vec<(u32, usize)>, next_va: &mut u32, va: u32, pages: usize) {
    let end = |(va, pages): (u32, usize)| va + (pages * PAGE) as u32;
    let i = free.partition_point(|&(v, _)| v < va);
    free.insert(i, (va, pages));
    if i + 1 < free.len() && end(free[i]) == free[i + 1].0 {
        free[i].1 += free[i + 1].1;
        free.remove(i + 1);
    }
    if i > 0 && end(free[i - 1]) == free[i].0 {
        free[i - 1].1 += free[i].1;
        free.remove(i);
    }
    if let Some(&last) = free.last()
        && end(last) == *next_va
    {
        *next_va = last.0;
        free.pop();
    }
}

struct Bo {
    mem: Dma<[u32]>,
    va: u32,
    pages: usize,
}

#[derive(Default)]
struct Stats {
    bos: u64,
    frees: u64,
    jobs: u64,
    failed_jobs: u64,
    job_us_max: u128,
    cl_jobs: u64,
    outomem: u64,
    mmu_faults: u64,
    resets: u64,
}

struct V3d {
    hub: Regs,
    core: Regs,
    pm: Regs,
    asb: Regs,
    pt: Dma<[u32]>,
    scratch_phys: usize,
    /// Free V3D address ranges (VA, pages), first fit.
    free_va: Vec<(u32, usize)>,
    next_va: u32,
    stats: Stats,
    /// The binner's overflow memory: chunk 0 at the start of each bin job,
    /// more on OUTOMEM. Kept for the next jobs.
    overflow: Vec<Bo>,
}

impl V3d {
    fn new(hub: Regs, core: Regs, pm: Regs, asb: Regs) -> V3d {
        let pt = match Dma::<[u32]>::zeroed_slice(PT_ENTRIES) {
            Ok(dma) => unsafe { dma.assume_init() },
            Err(err) => fail(&format!("no 4 MiB of contiguous memory for the page table: {err:?}")),
        };
        // The MMU sends illegal accesses here.
        let scratch = match Dma::<[u32]>::zeroed_slice(PAGE / 4) {
            Ok(dma) => unsafe { dma.assume_init() },
            Err(err) => fail(&format!("no scratch page: {err:?}")),
        };
        let pt_phys = pt.physical();
        let scratch_phys = scratch.physical();
        // The scratch page lives as long as the daemon.
        std::mem::forget(scratch);
        let mut v3d = V3d {
            hub,
            core,
            pm,
            asb,
            pt,
            scratch_phys,
            free_va: Vec::new(),
            next_va: VA_START,
            stats: Stats::default(),
            overflow: Vec::new(),
        };
        if !v3d.hw_setup() {
            fail("MMU flush did not finish");
        }
        say(&format!("MMU on: page table at {pt_phys:#x}, scratch page at {scratch_phys:#x}"));
        match v3d.new_bo(OVERFLOW_SIZE) {
            Ok(bo) => v3d.overflow.push(bo),
            Err(err) => fail(&format!("no binner overflow memory: {err}")),
        }
        v3d
    }

    fn alloc_va(&mut self, pages: usize) -> Option<u32> {
        if let Some(i) = self.free_va.iter().position(|&(_, n)| n >= pages) {
            let (va, n) = self.free_va[i];
            if n == pages {
                self.free_va.remove(i);
            } else {
                self.free_va[i] = (va + (pages * PAGE) as u32, n - pages);
            }
            return Some(va);
        }
        let va = self.next_va;
        let end = va.checked_add((pages * PAGE) as u32)?;
        if end > VA_END {
            return None;
        }
        self.next_va = end;
        Some(va)
    }

    fn new_bo(&mut self, bytes: usize) -> std::result::Result<Bo, String> {
        if bytes == 0 || bytes > BO_MAX {
            return Err(format!("size {bytes} not in 1..={BO_MAX}"));
        }
        let pages = bytes.div_ceil(PAGE);
        let mem = match Dma::<[u32]>::zeroed_slice(pages * PAGE / 4) {
            Ok(dma) => unsafe { dma.assume_init() },
            Err(err) => return Err(format!("no contiguous memory: {err:?}")),
        };
        let va = self.alloc_va(pages).ok_or("V3D address space full")?;
        let phys = mem.physical();
        for page in 0..pages {
            self.pt[va as usize / PAGE + page] = V3D_PTE_WRITEABLE | V3D_PTE_VALID | ((phys / PAGE + page) as u32);
        }
        if !csd::mmu_flush(&self.hub) {
            return Err("MMU flush did not finish".into());
        }
        self.stats.bos += 1;
        Ok(Bo { mem, va, pages })
    }

    fn free_bo(&mut self, bo: Bo) {
        for page in 0..bo.pages {
            self.pt[bo.va as usize / PAGE + page] = 0;
        }
        csd::mmu_flush(&self.hub);
        release_va(&mut self.free_va, &mut self.next_va, bo.va, bo.pages);
        self.stats.frees += 1;
        // bo.mem is freed here, after the MMU no longer maps it.
    }

    /// v3d_init_core() and v3d_mmu_set_page_table(): after power-up and
    /// after every reset.
    fn hw_setup(&self) -> bool {
        self.core.write(V3D_CTL_L2TFLSTA, 0);
        self.core.write(V3D_CTL_L2TFLEND, !0);
        self.hub.write(V3D_MMU_PT_PA_BASE, (self.pt.physical() / PAGE) as u32);
        self.hub.write(V3D_MMU_CTL, V3D_MMU_CTL_ENABLES);
        self.hub.write(V3D_MMU_ILLEGAL_ADDR, (self.scratch_phys / PAGE) as u32 | V3D_MMU_ILLEGAL_ADDR_ENABLE);
        self.hub.write(V3D_MMUC_CONTROL, V3D_MMUC_CONTROL_ENABLE);
        csd::mmu_flush(&self.hub)
    }

    /// v3d_reset() on the BCM2711: the reset controller is the PM's
    /// (bcm2835_reset_reset(): bridges closed, slave first, PM_V3DRSTN
    /// asserted, then released and the bridges opened, master first), then
    /// the core and the MMU set up again. The jobs in flight are lost.
    fn reset(&mut self) {
        let (pm, asb) = (&self.pm, &self.asb);
        say(&format!("reset: V3D_ERR_STAT {:#x}", self.core.read(V3D_ERR_STAT)));
        let bridge = |reg: usize, stop: bool| -> bool {
            let v = asb.read(reg);
            asb.write(reg, PM_PASSWORD | if stop { v | ASB_REQ_STOP } else { v & !ASB_REQ_STOP });
            let start = Instant::now();
            while (asb.read(reg) & ASB_ACK != 0) != stop {
                if start.elapsed() > Duration::from_millis(10) {
                    return false;
                }
            }
            true
        };
        // A bridge that does not ack its stop (an access stuck in it) does
        // not stop the reset: Linux gives up there, v3dd goes on, so the
        // core is reset either way.
        for (reg, name) in [(ASB_V3D_S_CTRL, "slave"), (ASB_V3D_M_CTRL, "master")] {
            if !bridge(reg, true) {
                say(&format!("reset: ASB {name} bridge: no ack for the stop within 10 ms"));
            }
        }
        pm.write(PM_GRAFX, PM_PASSWORD | (pm.read(PM_GRAFX) & !PM_V3DRSTN));
        thread::sleep(Duration::from_millis(1));
        pm.write(PM_GRAFX, PM_PASSWORD | pm.read(PM_GRAFX) | PM_V3DRSTN);
        thread::sleep(Duration::from_millis(1));
        let mut open = true;
        for (reg, name) in [(ASB_V3D_M_CTRL, "master"), (ASB_V3D_S_CTRL, "slave")] {
            if !bridge(reg, false) {
                say(&format!("reset: ASB {name} bridge: no ack for the start within 10 ms"));
                open = false;
            }
        }
        let setup = open && self.hw_setup();
        self.stats.resets += 1;
        say(&format!(
            "reset: {} (resets so far: {})",
            if setup { "done, MMU set up again" } else { "FAILED: V3D is not usable" },
            self.stats.resets
        ));
    }

    /// An MMU fault since the last call, cleared, as v3d_hub_irq() reports
    /// it: the address (MMU_VIO_ADDR << (va_width - 32)) and the unit.
    fn mmu_fault(&mut self) -> Option<String> {
        let hub = &self.hub;
        let ints = hub.read(V3D_HUB_INT_STS) & V3D_HUB_INT_MMU;
        let ctl = hub.read(V3D_MMU_CTL);
        let flags = ctl & (V3D_MMU_CTL_CAP_EXCEEDED | V3D_MMU_CTL_PT_INVALID | V3D_MMU_CTL_WRITE_VIOLATION);
        if ints == 0 && flags == 0 {
            return None;
        }
        let va_width = 30 + ((hub.read(V3D_MMU_DEBUG_INFO) >> 4) & 0xf);
        let addr = (hub.read(V3D_MMU_VIO_ADDR) as u64) << va_width.saturating_sub(32);
        let id = hub.read(V3D_MMU_VIO_ID) >> 5;
        let unit = ["L2T", "PTB", "PSE", "TLB", "CLE", "TFU", "MMU", "GMP"].get(id as usize).copied().unwrap_or("?");
        let mut kinds = Vec::new();
        if ints & V3D_HUB_INT_MMU_PTI != 0 || flags & V3D_MMU_CTL_PT_INVALID != 0 {
            kinds.push("no-pte");
        }
        if ints & V3D_HUB_INT_MMU_WRV != 0 || flags & V3D_MMU_CTL_WRITE_VIOLATION != 0 {
            kinds.push("write-violation");
        }
        if ints & V3D_HUB_INT_MMU_CAP != 0 || flags & V3D_MMU_CTL_CAP_EXCEEDED != 0 {
            kinds.push("cap-exceeded");
        }
        // The status bits in MMU_CTL are write-1-to-clear (v3d_hub_irq()).
        hub.write(V3D_MMU_CTL, ctl);
        hub.write(V3D_HUB_INT_CLR, V3D_HUB_INT_MMU);
        self.stats.mmu_faults += 1;
        Some(format!("{} va {addr:#x} unit {unit}", kinds.join(",")))
    }

    /// Runs one job with the watchdog armed, then looks for an MMU fault;
    /// a job that did not end (any Err of `run`) resets V3D.
    fn job<T>(&mut self, run: impl FnOnce(&mut V3d) -> std::result::Result<T, String>) -> std::result::Result<T, String> {
        let pm = Regs { base: self.pm.base };
        let watchdog = Watchdog(&pm);
        watchdog.arm();
        if let Some(old) = self.mmu_fault() {
            say(&format!("stale MMU fault before a job, cleared: {old}"));
            self.stats.mmu_faults -= 1;
        }
        let result = run(self);
        // The register dump before mmu_fault() clears the fault state.
        if result.is_err() {
            self.state();
        }
        let fault = self.mmu_fault();
        if result.is_err() {
            self.reset();
        }
        watchdog.stop();
        self.stats.jobs += 1;
        match (result, fault) {
            (Ok(value), None) => Ok(value),
            (Ok(_), Some(fault)) => {
                self.stats.failed_jobs += 1;
                say(&format!("job ended with an MMU fault: {fault}"));
                Err(format!("mmu-fault {fault}"))
            }
            (Err(err), None) => {
                self.stats.failed_jobs += 1;
                say(&format!("job failed: {err}; V3D reset"));
                Err(format!("{err}; V3D reset"))
            }
            (Err(err), Some(fault)) => {
                self.stats.failed_jobs += 1;
                say(&format!("job failed: {err}, MMU fault {fault}; V3D reset"));
                Err(format!("mmu-fault {fault}; {err}; V3D reset"))
            }
        }
    }

    fn state(&self) {
        let core = &self.core;
        say(&format!(
            "state: CT0CS {:#x} CT0CA {:#x}, CT1CS {:#x} CT1CA {:#x}, BFC {:#x}, RFC {:#x}, CTL_INT_STS {:#x}",
            core.read(V3D_CLE_CT0CS),
            core.read(V3D_CLE_CT0CA),
            core.read(V3D_CLE_CT1CS),
            core.read(V3D_CLE_CT1CA),
            core.read(V3D_CLE_BFC),
            core.read(V3D_CLE_RFC),
            core.read(V3D_CTL_INT_STS)
        ));
        csd::state(&self.hub, &self.core);
    }

    fn invalidate_caches(&self) {
        self.core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_L2TFLS);
        self.core.write(V3D_CTL_SLCACTL, 0x0f0f_0f0f);
    }

    /// v3d_clean_caches(): the TMU write combiner, then L2T, so the CPU
    /// sees what the job wrote.
    fn clean_caches(&self) {
        let core = &self.core;
        core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_TMUWCF);
        csd::wait("TMU write combiner flush", 100, || core.read(V3D_CTL_L2TCACTL) & V3D_L2TCACTL_TMUWCF == 0);
        core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_L2TFLS | V3D_L2TCACTL_FLM_CLEAN);
        csd::wait("L2T clean", 100, || core.read(V3D_CTL_L2TCACTL) & V3D_L2TCACTL_L2TFLS == 0);
    }

    /// One CSD job (v3d_csd_job_run()). Time in microseconds.
    fn csd(&mut self, cfg: [u32; 7]) -> std::result::Result<u128, String> {
        let us = self.job(|v3d| {
            v3d.invalidate_caches();
            let core = &v3d.core;
            let completed_before = (core.read(V3D_CSD_STATUS) >> 4) & 0xff;
            for i in 1..=6 {
                core.write(V3D_CSD_QUEUED_CFG0 + 4 * i, cfg[i]);
            }
            let start = Instant::now();
            core.write(V3D_CSD_QUEUED_CFG0, cfg[0]);
            let finished = csd::wait("job", 1000, || {
                let s = core.read(V3D_CSD_STATUS);
                (s >> 4) & 0xff != completed_before && s & 0x3 == 0
            });
            let us = start.elapsed().as_micros();
            if !finished {
                return Err(format!("timeout: CSD job not done after {us} us"));
            }
            v3d.clean_caches();
            Ok(us)
        })?;
        self.stats.job_us_max = self.stats.job_us_max.max(us);
        Ok(us)
    }

    /// One bin job (v3d_bin_job_run()), the binner's OUTOMEM answered with
    /// more overflow memory (v3d_overflow_mem_work()), then one render job
    /// (v3d_render_job_run()). Times in microseconds.
    fn bin_render(&mut self, bcl: (u32, u32), rcl: (u32, u32), qma: u32, qms: u32, qts: u32)
        -> std::result::Result<(u128, u128), String> {
        self.stats.cl_jobs += 1;
        let (bin, render) = self.job(|v3d| v3d.bin_render_armed(bcl, rcl, qma, qms, qts))?;
        self.stats.job_us_max = self.stats.job_us_max.max(bin + render);
        Ok((bin, render))
    }

    fn bin_render_armed(&mut self, bcl: (u32, u32), rcl: (u32, u32), qma: u32, qms: u32, qts: u32)
        -> std::result::Result<(u128, u128), String> {
        self.invalidate_caches();
        self.core.write(V3D_CTL_INT_CLR, V3D_INT_OUTOMEM);
        let bfc = self.core.read(V3D_CLE_BFC) & 0xff;
        self.core.write(V3D_PTB_BPOA, self.overflow[0].va);
        self.core.write(V3D_PTB_BPOS, OVERFLOW_SIZE as u32);
        if qma != 0 {
            self.core.write(V3D_CLE_CT0QMA, qma);
            self.core.write(V3D_CLE_CT0QMS, qms);
        }
        if qts != 0 {
            self.core.write(V3D_CLE_CT0QTS, qts | V3D_CLE_CT0QTS_ENABLE);
        }
        self.core.write(V3D_CLE_CT0QBA, bcl.0);
        let start = Instant::now();
        self.core.write(V3D_CLE_CT0QEA, bcl.1);
        let mut next = 1;
        loop {
            if self.core.read(V3D_CLE_BFC) & 0xff != bfc {
                break;
            }
            if self.core.read(V3D_CTL_INT_STS) & V3D_INT_OUTOMEM != 0 {
                // Edge signalled: cleared first, set again only after the
                // binner has used the new memory up too.
                self.core.write(V3D_CTL_INT_CLR, V3D_INT_OUTOMEM);
                if next == self.overflow.len() {
                    if next == OVERFLOW_CHUNKS_MAX {
                        return Err(format!("bin job: out of memory after {next} overflow chunks"));
                    }
                    let bo = self.new_bo(OVERFLOW_SIZE).map_err(|err| format!("bin job: overflow memory: {err}"))?;
                    self.overflow.push(bo);
                }
                self.core.write(V3D_PTB_BPOA, self.overflow[next].va);
                self.core.write(V3D_PTB_BPOS, OVERFLOW_SIZE as u32);
                next += 1;
                self.stats.outomem += 1;
            }
            if start.elapsed() > Duration::from_secs(1) {
                return Err(format!("timeout: bin job not done after {} us", start.elapsed().as_micros()));
            }
            thread::yield_now();
        }
        let bin_us = start.elapsed().as_micros();

        self.invalidate_caches();
        let core = &self.core;
        let rfc = core.read(V3D_CLE_RFC) & 0xff;
        core.write(V3D_CLE_CT1QBA, rcl.0);
        let start = Instant::now();
        core.write(V3D_CLE_CT1QEA, rcl.1);
        if !csd::wait("render job", 1000, || core.read(V3D_CLE_RFC) & 0xff != rfc) {
            return Err(format!("timeout: render job not done after {} us", start.elapsed().as_micros()));
        }
        let render_us = start.elapsed().as_micros();
        // As after a CSD job, so the CPU sees what the TLB stored.
        self.clean_caches();
        Ok((bin_us, render_us))
    }
}

enum Handle {
    Root,
    Client { reply: Vec<u8>, bos: BTreeMap<u32, Bo>, next_id: u32 },
}

struct V3dScheme {
    v3d: V3d,
    handles: HandleMap<Handle>,
}

fn number(s: &str) -> std::result::Result<u64, String> {
    let r = match s.strip_prefix("0x") {
        Some(hex) => u64::from_str_radix(hex, 16),
        None => s.parse(),
    };
    r.map_err(|_| format!("not a number: {s}"))
}

impl V3dScheme {
    fn command(&mut self, id: usize, line: &str) -> std::result::Result<String, String> {
        let Ok(Handle::Client { bos, next_id, .. }) = self.handles.get_mut(id) else {
            return Err("not a client handle".into());
        };
        let words: Vec<&str> = line.split_whitespace().collect();
        let arg = |i: usize| -> std::result::Result<u64, String> {
            number(words.get(i).ok_or_else(|| format!("{}: missing argument {i}", words[0]))?)
        };
        match words.first().copied() {
            Some("bo") => {
                let bo = self.v3d.new_bo(arg(1)? as usize)?;
                let va = bo.va;
                let bo_id = *next_id;
                *next_id += 1;
                bos.insert(bo_id, bo);
                Ok(format!("{bo_id} {va:#x}"))
            }
            Some("put") => {
                let bo = bos.get_mut(&(arg(1)? as u32)).ok_or("no such BO")?;
                let first = arg(2)? as usize;
                let values: Vec<u32> =
                    words[3..].iter().map(|w| number(w).map(|v| v as u32)).collect::<std::result::Result<_, _>>()?;
                if first + values.len() > bo.mem.len() {
                    return Err("past the end of the BO".into());
                }
                bo.mem[first..first + values.len()].copy_from_slice(&values);
                Ok("ok".into())
            }
            Some("get") => {
                let bo = bos.get(&(arg(1)? as u32)).ok_or("no such BO")?;
                let (first, count) = (arg(2)? as usize, arg(3)? as usize);
                if first + count > bo.mem.len() {
                    return Err("past the end of the BO".into());
                }
                Ok(bo.mem[first..first + count].iter().map(|w| format!("{w:#x}")).collect::<Vec<_>>().join(" "))
            }
            Some("fill") => {
                let bo = bos.get_mut(&(arg(1)? as u32)).ok_or("no such BO")?;
                let (first, count, value) = (arg(2)? as usize, arg(3)? as usize, arg(4)? as u32);
                if first + count > bo.mem.len() {
                    return Err("past the end of the BO".into());
                }
                bo.mem[first..first + count].fill(value);
                Ok("ok".into())
            }
            Some("cl") => {
                let mut a = [0u32; 7];
                for (i, v) in a.iter_mut().enumerate() {
                    *v = arg(i + 1)? as u32;
                }
                let (bin, render) = self.v3d.bin_render((a[0], a[1]), (a[2], a[3]), a[4], a[5], a[6])?;
                Ok(format!("done {} bin {bin} render {render}", bin + render))
            }
            Some("free") => {
                let bo = bos.remove(&(arg(1)? as u32)).ok_or("no such BO")?;
                self.v3d.free_bo(bo);
                Ok("ok".into())
            }
            Some("csd") => {
                let mut cfg = [0u32; 7];
                for (i, c) in cfg.iter_mut().enumerate() {
                    *c = arg(i + 1)? as u32;
                }
                let us = self.v3d.csd(cfg)?;
                Ok(format!("done {us}"))
            }
            Some("stats") => {
                let s = &self.v3d.stats;
                Ok(format!(
                    "bos {} frees {} jobs {} cl_jobs {} failed {} mmu_faults {} resets {} job_us_max {} outomem {} overflow_chunks {} va_next {:#x} va_free_ranges {}",
                    s.bos,
                    s.frees,
                    s.jobs,
                    s.cl_jobs,
                    s.failed_jobs,
                    s.mmu_faults,
                    s.resets,
                    s.job_us_max,
                    s.outomem,
                    self.v3d.overflow.len(),
                    self.v3d.next_va,
                    self.v3d.free_va.len()
                ))
            }
            _ => Err(format!("unknown command: {line}")),
        }
    }
}

impl SchemeSync for V3dScheme {
    fn scheme_root(&mut self) -> Result<usize> {
        Ok(self.handles.insert(Handle::Root))
    }

    fn openat(
        &mut self,
        dirfd: usize,
        path: &str,
        _flags: usize,
        _fcntl_flags: u32,
        ctx: &CallerCtx,
    ) -> Result<OpenResult> {
        if !matches!(self.handles.get(dirfd)?, Handle::Root) {
            return Err(Error::new(EACCES));
        }
        // GPU jobs can read and write any memory mapped for them: root only
        // for now.
        if ctx.uid != 0 || !path.trim_matches('/').is_empty() {
            return Err(Error::new(EACCES));
        }
        let handle = Handle::Client { reply: Vec::new(), bos: BTreeMap::new(), next_id: 1 };
        Ok(OpenResult::ThisScheme { number: self.handles.insert(handle), flags: NewFdFlags::empty() })
    }

    fn read(&mut self, id: usize, buf: &mut [u8], _offset: u64, _fcntl_flags: u32, _ctx: &CallerCtx) -> Result<usize> {
        let Handle::Client { reply, .. } = self.handles.get_mut(id)? else {
            return Err(Error::new(EBADF));
        };
        let len = buf.len().min(reply.len());
        buf[..len].copy_from_slice(&reply[..len]);
        reply.drain(..len);
        Ok(len)
    }

    fn write(&mut self, id: usize, buf: &[u8], _offset: u64, _fcntl_flags: u32, _ctx: &CallerCtx) -> Result<usize> {
        if !matches!(self.handles.get(id)?, Handle::Client { .. }) {
            return Err(Error::new(EBADF));
        }
        let line = std::str::from_utf8(buf).map_err(|_| Error::new(EINVAL))?.trim();
        let answer = match self.command(id, line) {
            Ok(answer) => answer,
            Err(err) => format!("error {err}"),
        };
        if let Ok(Handle::Client { reply, .. }) = self.handles.get_mut(id) {
            *reply = format!("{answer}\n").into_bytes();
        }
        Ok(buf.len())
    }

    fn fpath(&mut self, id: usize, buf: &mut [u8], _ctx: &CallerCtx) -> Result<usize> {
        self.handles.get(id).map_err(|_| Error::new(EBADF))?;
        FpathWriter::with(buf, "v3d", |_| Ok(()))
    }

    fn on_close(&mut self, id: usize) {
        if let Some(Handle::Client { bos, .. }) = self.handles.remove(id) {
            for bo in bos.into_values() {
                self.v3d.free_bo(bo);
            }
        }
    }
}

fn main() {
    daemon::SchemeDaemon::new(daemon);
}

fn daemon(daemon: daemon::SchemeDaemon) -> ! {
    common::init();
    let dtb = fs::read("/scheme/kernel.dtb").unwrap_or_default();
    let bcm2711 = fdt::Fdt::new(&dtb).is_ok_and(|f| {
        f.find_compatible(&["brcm,2711-v3d"]).is_some() || f.root().compatible().all().any(|c| c == "brcm,bcm2711")
    });
    if !bcm2711 {
        fail("not a BCM2711 (no brcm,2711-v3d, no brcm,bcm2711 in the devicetree): V3D left alone");
    }
    let pm = Regs::map(PM_PHYS, 0x1000, "pm");
    let asb = Regs::map(RPIVID_ASB_PHYS, 0x1000, "rpivid_asb");
    let watchdog = Watchdog(&pm);
    watchdog.arm();
    power_up(&pm, &asb);
    let hub = Regs::map(HUB_PHYS, 0x4000, "hub");
    let core = Regs::map(CORE0_PHYS, 0x4000, "core0");
    let h1 = hub.read(V3D_HUB_IDENT1);
    watchdog.stop();
    let ver = (h1 & 0xf) * 10 + ((h1 >> 4) & 0xf);
    if ver < 41 {
        fail(&format!("V3D version {ver} (HUB_IDENT1 {h1:#x})"));
    }
    say(&format!("V3D {ver} powered up"));
    let v3d = V3d::new(hub, core, pm, asb);
    let mut scheme = V3dScheme { v3d, handles: HandleMap::new() };

    let socket = Socket::create().expect("v3dd: failed to create the v3d scheme");
    let handler = Blocking::new(Box::new(socket), 16);
    let _ = daemon.ready_sync_scheme(handler.socket(), &mut scheme);
    say("ready: /scheme/v3d");
    handler.process_requests_blocking(scheme).expect("v3dd: failed to process requests");
}
