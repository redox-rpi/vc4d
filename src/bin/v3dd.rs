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
//   free ID                   -> ok
//   csd CFG0 CFG1 ... CFG6    -> done US       runs one CSD job, waits for it
//   stats                     -> one line of counters
//
// Numbers are decimal or 0x hex. A failed command answers "error ...". BOs
// belong to the open file and are freed when it is closed. Jobs run one at
// a time, synchronously, with the caches handled as Linux's v3d driver does
// (v3d_gem.c v3d_invalidate_caches(), v3d_clean_caches(); v3d_sched.c
// v3d_csd_job_run()).
//
// Not yet (later steps of #598): bin/render jobs, mmap of BOs, the V3D
// interrupt, recovery after a hung job, one address space per client.
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

const V3D_CTL_SLCACTL: usize = 0x0024;
const V3D_CTL_L2TCACTL: usize = 0x0030;
const V3D_L2TCACTL_TMUWCF: u32 = 1 << 8;
const V3D_L2TCACTL_FLM_CLEAN: u32 = 2 << 1;
const V3D_L2TCACTL_L2TFLS: u32 = 1 << 0;
const V3D_CTL_L2TFLSTA: usize = 0x0034;
const V3D_CTL_L2TFLEND: usize = 0x0038;
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
}

struct V3d {
    hub: Regs,
    core: Regs,
    pm: Regs,
    pt: Dma<[u32]>,
    /// Free V3D address ranges (VA, pages), first fit.
    free_va: Vec<(u32, usize)>,
    next_va: u32,
    stats: Stats,
}

impl V3d {
    fn new(hub: Regs, core: Regs, pm: Regs) -> V3d {
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
        core.write(V3D_CTL_L2TFLSTA, 0);
        core.write(V3D_CTL_L2TFLEND, !0);
        hub.write(V3D_MMU_PT_PA_BASE, (pt_phys / PAGE) as u32);
        hub.write(V3D_MMU_CTL, V3D_MMU_CTL_ENABLES);
        hub.write(V3D_MMU_ILLEGAL_ADDR, (scratch_phys / PAGE) as u32 | V3D_MMU_ILLEGAL_ADDR_ENABLE);
        hub.write(V3D_MMUC_CONTROL, V3D_MMUC_CONTROL_ENABLE);
        if !csd::mmu_flush(&hub) {
            fail("MMU flush did not finish");
        }
        say(&format!("MMU on: page table at {pt_phys:#x}, scratch page at {scratch_phys:#x}"));
        V3d { hub, core, pm, pt, free_va: Vec::new(), next_va: VA_START, stats: Stats::default() }
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
        self.free_va.push((bo.va, bo.pages));
        self.stats.frees += 1;
        // bo.mem is freed here, after the MMU no longer maps it.
    }

    /// One CSD job (v3d_csd_job_run()), caches as v3d_invalidate_caches()
    /// before and v3d_clean_caches() after.
    fn csd(&mut self, cfg: [u32; 7]) -> std::result::Result<u128, String> {
        let core = &self.core;
        let watchdog = Watchdog(&self.pm);
        watchdog.arm();
        core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_L2TFLS);
        core.write(V3D_CTL_SLCACTL, 0x0f0f_0f0f);
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
        if finished {
            core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_TMUWCF);
            csd::wait("TMU write combiner flush", 100, || core.read(V3D_CTL_L2TCACTL) & V3D_L2TCACTL_TMUWCF == 0);
            core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_L2TFLS | V3D_L2TCACTL_FLM_CLEAN);
            csd::wait("L2T clean", 100, || core.read(V3D_CTL_L2TCACTL) & V3D_L2TCACTL_L2TFLS == 0);
        }
        watchdog.stop();
        self.stats.jobs += 1;
        if finished {
            self.stats.job_us_max = self.stats.job_us_max.max(us);
            Ok(us)
        } else {
            self.stats.failed_jobs += 1;
            csd::state(&self.hub, &self.core);
            Err(format!("job not done after {us} us"))
        }
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
                    "bos {} frees {} jobs {} failed {} job_us_max {} va_next {:#x} va_free_ranges {}",
                    s.bos,
                    s.frees,
                    s.jobs,
                    s.failed_jobs,
                    s.job_us_max,
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
    let v3d = V3d::new(hub, core, pm);
    let mut scheme = V3dScheme { v3d, handles: HandleMap::new() };

    let socket = Socket::create().expect("v3dd: failed to create the v3d scheme");
    let handler = Blocking::new(Box::new(socket), 16);
    let _ = daemon.ready_sync_scheme(handler.socket(), &mut scheme);
    say("ready: /scheme/v3d");
    handler.process_requests_blocking(scheme).expect("v3dd: failed to process requests");
}
