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
//   free ID                   -> ok            (V3D keeps it until the
//                                              client's pending jobs are done)
//   submit csd CFG0 ... CFG6  -> job N         queues one CSD job
//   submit cl BCL END RCL END QMA QMS QTS
//                             -> job N         queues one bin job, then one
//                                              render job (all V3D addresses:
//                                              the control lists, the tile
//                                              allocation memory and its size,
//                                              the tile state)
//   wait N                    -> done US       the read waits until job N is
//                                (csd), or     done; a job's answer is given
//                                done US bin US render US (cl)
//                                              once, to wait or poll
//   poll N                    -> queued, running, or the answer as wait
//   csd CFG0 ... CFG6         -> as submit csd, then wait
//   cl BCL END RCL END QMA QMS QTS -> as submit cl, then wait
//   stats                     -> one line of counters
//
// Jobs (G2, redox-rpi#668) go into one queue for all clients and run in
// order on the job thread, one at a time; the request loop goes on
// meanwhile, so other clients' commands and BOs do not wait for a job.
// A read that waits for a job is held and answered by the job thread when
// the job is done. If that read is cancelled (a signal), it answers EINTR
// and the next read waits again. If the client writes a new command
// instead, the wait ends and the job's answer stays for wait or poll.

// A client maps a BO into its own memory with fmap on the same handle
// (step 3e, redox-rpi#666): offset (ID << 32) + BYTE, BYTE a multiple of
// the page size, MAP_SHARED. The mapping is the BO's own memory, with the
// memory type v3dd has (uncached, as all DMA memory on aarch64), so
// neither side needs a cache flush; the client issues a DSB before it
// writes the job command. A BO freed while mapped stays the client's
// memory until it unmaps it (the kernel counts references to the pages),
// but V3D no longer sees it.
//
// Numbers are decimal or 0x hex. A failed command answers "error ...". BOs
// belong to the open file and are freed when it is closed; the client's
// queued jobs go then, and a running one ends first. Jobs run one at a
// time, with the caches handled as Linux's v3d driver does
// (v3d_gem.c v3d_invalidate_caches(), v3d_clean_caches(); v3d_sched.c
// v3d_csd_job_run()).
//
// Errors (step 3c): an MMU fault during a job (an address with no valid
// page table entry, a write to a read-only page) comes back as "error
// mmu-fault ..." with the faulting address and the unit, as Linux's
// v3d_hub_irq() logs it. A job that does not end within 1 s is stopped by
// a reset of V3D (Linux v3d_reset(): on the BCM2711 the PM reset line
// with the bridges closed, then the MMU set up again) and comes back as
// "error timeout ...". The daemon goes on in both cases, with the next job
// in the queue: only the job that ran fails, the queued ones never reached
// V3D.
//
// Interrupt (step 3d): v3dd waits for a job on the V3D interrupt (the
// devicetree's v3d node, GIC SPI 74 on the BCM2711, through /scheme/irq),
// as Linux's v3d_irq.c: FLDONE, FRDONE, CSDDONE and OUTOMEM from the core,
// the MMU faults from the hub. A 5 ms timer stays as a fallback, and
// without an interrupt v3dd polls as before.
//
// Address spaces (G2 step 2): every client gets its own V3D page table
// with its first BO, and the job thread loads the job's table into the MMU
// before the job runs (V3D has one page table base; jobs run one at a
// time). A job reaches only its own client's BOs: any other address is an
// MMU fault. Each table covers all 32-bit V3D addresses, 4 MiB of
// contiguous memory per client. The binner's overflow memory is mapped
// in every space at the same address, above the clients' range. A closed
// client's space goes with its BOs, after its running job.
//
// Binary interface (G2 step 3): the V3D ioctls of Linux as scheme calls
// (see v3d/uapi.rs for the wire form): CREATE_BO, MMAP_BO (the fmap offset
// above), GET_BO_OFFSET, GEM_CLOSE, GET_PARAM, SUBMIT_CL and SUBMIT_CSD
// with their BO lists, and WAIT_BO, which waits for the last job that named
// the BO; it answers EIO when that job failed, and ETIME for timeout 0
// while the job runs (another timeout is not kept: jobs end within 1 s).
// Not yet: syncobjs (in_sync is covered by the queue order, out_sync is
// not signalled), TFU jobs, a render job without a bin job, perfmon. Both
// interfaces work on the same handle.
//
// The binner's overflow memory belongs to v3dd: 512 KiB at the start of
// every bin job, and another 512 KiB chunk each time the binner reports
// it is out of memory (OUTOMEM), as Linux's v3d_overflow_mem_work().
//
// The SoC watchdog (bcm2835_wdt) is armed around the power-up and around
// each job, as in v3dprobe: the Pi 400 in the lab has no remote power
// switch, and a GPU access that hangs the bus resets the SoC.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write as _};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::process;
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::thread;
use std::time::{Duration, Instant};

use common::dma::Dma;
use event::{EventQueue, user_data};
use redox_scheme::scheme::{IntoTag as _, Op, SchemeState, SchemeSync};
use redox_scheme::{CallerCtx, OpenResult, RequestKind, Response, SignalBehavior, Socket};
use scheme_utils::{FpathWriter, HandleMap};
use syscall::error::{EACCES, EBADF, EINTR, EINVAL, EIO, ENOENT, ENOMEM, ENOSYS, ETIME, Error, Result};
use syscall::flag::{MapFlags, MunmapFlags};
use syscall::schemev2::NewFdFlags;

#[path = "v3d/client.rs"]
#[allow(dead_code)]
mod client;
#[path = "v3d/csd.rs"]
#[allow(dead_code)]
mod csd;
#[path = "v3d/uapi.rs"]
#[allow(dead_code)]
mod uapi;

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
const V3D_HUB_INT_MSK_SET: usize = 0x0060;
const V3D_HUB_INT_MSK_CLR: usize = 0x0064;
const V3D_CTL_INT_MSK_SET: usize = 0x0060;
const V3D_CTL_INT_MSK_CLR: usize = 0x0064;
const V3D_INT_CSDDONE: u32 = 1 << 7;
const V3D_INT_FLDONE: u32 = 1 << 1;
const V3D_INT_FRDONE: u32 = 1 << 0;
/// v3d_irq.c V3D_CORE_IRQS and V3D_HUB_IRQS for V3D 4.2 (no TFU here).
const V3D_CORE_IRQS: u32 = V3D_INT_OUTOMEM | V3D_INT_FLDONE | V3D_INT_FRDONE | V3D_INT_CSDDONE;
const V3D_HUB_IRQS: u32 = V3D_HUB_INT_MMU;
/// The fallback check while v3dd waits for the interrupt.
const IRQ_FALLBACK: Duration = Duration::from_millis(5);
/// The Pi 400's v3d node: GIC SPI 74, level high.
const V3D_IRQ_DEFAULT: [u32; 3] = [0, 74, 4];

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

// Device memory, mapped for the life of the daemon; the threads that share
// a Regs take turns through the locks around V3d and Mmu.
unsafe impl Send for Regs {}

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

user_data! {
    enum IrqSource {
        Irq,
        Time,
    }
}

/// The V3D interrupt from /scheme/irq and the fallback timer.
struct Irq {
    path: String,
    file: File,
    timer: File,
    queue: EventQueue<IrqSource>,
    /// A fallback timeout is pending: every write to /scheme/time adds one
    /// more (Test 603: one per loop piled up to a wakeup every 0.12 ms).
    armed: bool,
}

/// /scheme/irq/phandle-P/A,B,C for the v3d node: `interrupts` of the node
/// (GIC SPI 74 when the devicetree has no v3d node), the controller from
/// the first `interrupt-parent` of the node, its bus or the root.
fn irq_path(fdt: &fdt::Fdt) -> Option<String> {
    let cells = |raw: &[u8]| -> Vec<u32> {
        raw.chunks_exact(4).map(|c| u32::from_be_bytes([c[0], c[1], c[2], c[3]])).collect()
    };
    let node = fdt.find_compatible(&["brcm,2711-v3d"]);
    let parent = [node, fdt.find_node("/v3dbus"), fdt.find_node("/scb"), fdt.find_node("/")]
        .into_iter()
        .flatten()
        .find_map(|n| cells(n.property("interrupt-parent")?.value).first().copied())?;
    let gic = fdt.find_phandle(parent)?;
    let count = *cells(gic.property("#interrupt-cells")?.value).first()? as usize;
    let spec = node
        .and_then(|n| n.property("interrupts"))
        .map(|p| cells(p.value))
        .unwrap_or_else(|| V3D_IRQ_DEFAULT.to_vec());
    let spec: Vec<String> = spec.get(..count)?.iter().map(u32::to_string).collect();
    Some(format!("/scheme/irq/phandle-{parent}/{}", spec.join(",")))
}

impl Irq {
    fn open(path: String) -> std::result::Result<Irq, String> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .open(&path)
            .map_err(|err| format!("{path}: {err}"))?;
        let timer = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(syscall::O_NONBLOCK as i32)
            .open(format!("/scheme/time/{}", syscall::CLOCK_MONOTONIC))
            .map_err(|err| format!("/scheme/time: {err}"))?;
        let queue = EventQueue::<IrqSource>::new().map_err(|err| format!("event queue: {err}"))?;
        queue
            .subscribe(file.as_raw_fd() as usize, IrqSource::Irq, event::EventFlags::READ)
            .map_err(|err| format!("event queue: {err}"))?;
        queue
            .subscribe(timer.as_raw_fd() as usize, IrqSource::Time, event::EventFlags::READ)
            .map_err(|err| format!("event queue: {err}"))?;
        Ok(Irq { path, file, timer, queue, armed: false })
    }

    /// The fallback timer fires `after` from now, unless one is pending.
    fn arm(&mut self, after: Duration) {
        if self.armed {
            return;
        }
        self.armed = true;
        let mut time = syscall::TimeSpec::default();
        if self.timer.read(&mut time).is_err() {
            return;
        }
        let nsec = time.tv_nsec as u64 + u64::from(after.subsec_nanos());
        time.tv_sec += after.as_secs() as i64 + (nsec / 1_000_000_000) as i64;
        time.tv_nsec = (nsec % 1_000_000_000) as i32;
        let _ = self.timer.write(&time);
    }
}

/// fmap offsets: the BO ID above this bit, the byte in the BO below.
const MAP_ID_SHIFT: u32 = 32;

/// A BO's memory: v3dd's own contiguous pages, or another driver's buffer
/// imported through its PRIME fd (G4 step 2, redox-rpi#670).
enum Mem {
    Own(Dma<[u32]>),
    Imported(Imported),
}

/// Another driver's buffer (vc4d's scanout buffer), imported through the
/// PRIME fd a client sent (sendfd): v3dd keeps the fd and its own mapping
/// of it, so the buffer stays while the BO exists. The physical pages come
/// from the kernel for that mapping, never from a client.
struct Imported {
    /// Held open: the PRIME fd keeps the other driver's buffer alive.
    _file: File,
    addr: usize,
    bytes: usize,
}

impl Drop for Imported {
    fn drop(&mut self) {
        // The MMU no longer maps the pages (free_bo); the file closes after.
        let _ = unsafe { syscall::funmap(self.addr, self.bytes) };
    }
}

impl std::ops::Deref for Mem {
    type Target = [u32];
    fn deref(&self) -> &[u32] {
        match self {
            Mem::Own(dma) => dma,
            Mem::Imported(imp) => unsafe { std::slice::from_raw_parts(imp.addr as *const u32, imp.bytes / 4) },
        }
    }
}

impl std::ops::DerefMut for Mem {
    fn deref_mut(&mut self) -> &mut [u32] {
        match self {
            Mem::Own(dma) => dma,
            Mem::Imported(imp) => unsafe { std::slice::from_raw_parts_mut(imp.addr as *mut u32, imp.bytes / 4) },
        }
    }
}

/// The largest buffer v3dd imports: a 3840x2160 XRGB8888 scanout buffer.
const IMPORT_MAX: usize = 64 << 20;

struct Bo {
    mem: Mem,
    va: u32,
    pages: usize,
    /// The address space it is mapped in (OVERFLOW_SPACES: all of them).
    space: u32,
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
    irqs: u64,
    irq_wakeups: u64,
    timer_wakeups: u64,
    maps: u64,
    unmaps: u64,
    overflow_chunks: usize,
    /// Address spaces made for clients, and removed again.
    spaces: u64,
    spaces_removed: u64,
    /// The interrupt path, or "off" while v3dd polls.
    irq: String,
}

/// Locks a mutex; a thread that panicked with it held does not stop v3dd.
fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// One V3D address space (G2 step 2, redox-rpi#668): a page table over all
/// of V3D's 32-bit addresses (4 MiB), and its allocator. Every client gets
/// its own on its first BO, so a job reaches only its own client's BOs.
struct Space {
    pt: Dma<[u32]>,
    /// Free V3D address ranges (VA, pages), first fit.
    free_va: Vec<(u32, usize)>,
    next_va: u32,
}

/// v3dd's own space: the MMU uses it when no client's is loaded. It holds
/// only the overflow memory.
const DAEMON_SPACE: u32 = 0;
/// The space of the binner's overflow chunks: mapped in every space.
const OVERFLOW_SPACES: u32 = u32::MAX;
/// The overflow chunks lie above every client's addresses (VA_END), at
/// the same address in every space.
const OVERFLOW_VA: u32 = VA_END;

/// The address spaces and the MMU. A lock of its own: the request loop maps
/// and unmaps BOs while a job runs on the job thread, as Linux's v3d_mmu.c
/// inserts PTEs while jobs run. The job thread loads the job's space
/// before the job; a reset holds the lock throughout, so no MMU flush meets
/// V3D in reset.
struct Mmu {
    hub: Regs,
    scratch_phys: usize,
    spaces: BTreeMap<u32, Space>,
    next_space: u32,
    /// The space whose page table the MMU walks.
    loaded: u32,
    /// The overflow chunks' physical addresses, chunk i at
    /// OVERFLOW_VA + i * OVERFLOW_SIZE in every space.
    overflow: Vec<usize>,
    stats: Arc<Mutex<Stats>>,
}

struct V3d {
    hub: Regs,
    core: Regs,
    pm: Regs,
    asb: Regs,
    mmu: Arc<Mutex<Mmu>>,
    stats: Arc<Mutex<Stats>>,
    /// None: v3dd polls.
    irq: Option<Irq>,
    /// Interrupt bits the handler took and cleared, for the waiting code.
    core_seen: u32,
    hub_seen: u32,
    /// The binner's overflow memory: chunk 0 at the start of each bin job,
    /// more on OUTOMEM. Kept for the next jobs.
    overflow: Vec<Bo>,
}

impl Space {
    fn new() -> std::result::Result<Space, String> {
        match Dma::<[u32]>::zeroed_slice(PT_ENTRIES) {
            Ok(dma) => Ok(Space { pt: unsafe { dma.assume_init() }, free_va: Vec::new(), next_va: VA_START }),
            Err(err) => Err(format!("no 4 MiB of contiguous memory for a page table: {err:?}")),
        }
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

    fn map(&mut self, va: u32, phys: usize, pages: usize) {
        for page in 0..pages {
            self.pt[va as usize / PAGE + page] = V3D_PTE_WRITEABLE | V3D_PTE_VALID | ((phys / PAGE + page) as u32);
        }
    }

    /// Maps pages that need not be contiguous (an imported buffer).
    fn map_pages(&mut self, va: u32, phys: &[usize]) {
        for (page, &addr) in phys.iter().enumerate() {
            self.pt[va as usize / PAGE + page] = V3D_PTE_WRITEABLE | V3D_PTE_VALID | ((addr / PAGE) as u32);
        }
    }

    fn unmap(&mut self, va: u32, pages: usize) {
        self.pt[va as usize / PAGE..va as usize / PAGE + pages].fill(0);
    }
}

impl Mmu {
    fn pt_phys(&self) -> usize {
        self.spaces[&self.loaded].pt.physical()
    }

    /// A new space for a client, with the overflow chunks mapped.
    fn new_space(&mut self) -> std::result::Result<u32, String> {
        let mut space = Space::new()?;
        for (i, &phys) in self.overflow.iter().enumerate() {
            space.map(OVERFLOW_VA + (i * OVERFLOW_SIZE) as u32, phys, OVERFLOW_SIZE / PAGE);
        }
        self.next_space += 1;
        let id = self.next_space;
        self.spaces.insert(id, space);
        lock(&self.stats).spaces += 1;
        Ok(id)
    }

    /// Drops a client's space once its BOs and jobs are gone.
    fn remove_space(&mut self, id: u32) {
        if id == DAEMON_SPACE {
            return;
        }
        if self.loaded == id
            && let Err(err) = self.load(DAEMON_SPACE)
        {
            say(&format!("removing space {id}: {err}"));
        }
        if self.spaces.remove(&id).is_some() {
            lock(&self.stats).spaces_removed += 1;
        }
    }

    /// Points the MMU at a space's page table, then flushes it.
    fn load(&mut self, id: u32) -> std::result::Result<(), String> {
        if self.loaded == id {
            return Ok(());
        }
        let phys = self.spaces.get(&id).ok_or_else(|| format!("no address space {id}"))?.pt.physical();
        self.hub.write(V3D_MMU_PT_PA_BASE, (phys / PAGE) as u32);
        self.loaded = id;
        if !csd::mmu_flush(&self.hub) {
            return Err("MMU flush did not finish".into());
        }
        Ok(())
    }

    fn flush_if_loaded(&self, space: u32) -> bool {
        (space != self.loaded && space != OVERFLOW_SPACES) || csd::mmu_flush(&self.hub)
    }

    fn new_bo(&mut self, space: u32, bytes: usize) -> std::result::Result<Bo, String> {
        if bytes == 0 || bytes > BO_MAX {
            return Err(format!("size {bytes} not in 1..={BO_MAX}"));
        }
        let pages = bytes.div_ceil(PAGE);
        let mem = match Dma::<[u32]>::zeroed_slice(pages * PAGE / 4) {
            Ok(dma) => unsafe { dma.assume_init() },
            Err(err) => return Err(format!("no contiguous memory: {err:?}")),
        };
        let table = self.spaces.get_mut(&space).ok_or_else(|| format!("no address space {space}"))?;
        let va = table.alloc_va(pages).ok_or("V3D address space full")?;
        table.map(va, mem.physical(), pages);
        lock(&self.stats).bos += 1;
        let bo = Bo { mem: Mem::Own(mem), va, pages, space };
        if !self.flush_if_loaded(space) {
            self.free_bo(bo);
            return Err("MMU flush did not finish".into());
        }
        Ok(bo)
    }

    /// A BO over an imported buffer's pages, in a client's space.
    fn import_bo(&mut self, space: u32, imported: Imported, phys: &[usize]) -> std::result::Result<Bo, String> {
        let pages = phys.len();
        let table = self.spaces.get_mut(&space).ok_or_else(|| format!("no address space {space}"))?;
        let va = table.alloc_va(pages).ok_or("V3D address space full")?;
        table.map_pages(va, phys);
        lock(&self.stats).bos += 1;
        let bo = Bo { mem: Mem::Imported(imported), va, pages, space };
        if !self.flush_if_loaded(space) {
            self.free_bo(bo);
            return Err("MMU flush did not finish".into());
        }
        Ok(bo)
    }

    fn free_bo(&mut self, bo: Bo) {
        if let Some(table) = self.spaces.get_mut(&bo.space) {
            table.unmap(bo.va, bo.pages);
            release_va(&mut table.free_va, &mut table.next_va, bo.va, bo.pages);
        }
        self.flush_if_loaded(bo.space);
        lock(&self.stats).frees += 1;
        // bo.mem is freed here, after the MMU no longer maps it.
    }

    /// One more chunk of the binner's overflow memory, in every space.
    fn new_overflow(&mut self) -> std::result::Result<Bo, String> {
        let pages = OVERFLOW_SIZE / PAGE;
        let mem = match Dma::<[u32]>::zeroed_slice(OVERFLOW_SIZE / 4) {
            Ok(dma) => unsafe { dma.assume_init() },
            Err(err) => return Err(format!("no contiguous memory: {err:?}")),
        };
        let va = OVERFLOW_VA + (self.overflow.len() * OVERFLOW_SIZE) as u32;
        for table in self.spaces.values_mut() {
            table.map(va, mem.physical(), pages);
        }
        self.overflow.push(mem.physical());
        lock(&self.stats).bos += 1;
        if !csd::mmu_flush(&self.hub) {
            return Err("MMU flush did not finish".into());
        }
        Ok(Bo { mem: Mem::Own(mem), va, pages, space: OVERFLOW_SPACES })
    }
}

impl V3d {
    fn new(hub: Regs, core: Regs, pm: Regs, asb: Regs, irq: Option<Irq>) -> V3d {
        let daemon_space = Space::new().unwrap_or_else(|err| fail(&err));
        let pt_phys = daemon_space.pt.physical();
        // The MMU sends illegal accesses here.
        let scratch = match Dma::<[u32]>::zeroed_slice(PAGE / 4) {
            Ok(dma) => unsafe { dma.assume_init() },
            Err(err) => fail(&format!("no scratch page: {err:?}")),
        };
        let scratch_phys = scratch.physical();
        // The scratch page lives as long as the daemon.
        std::mem::forget(scratch);
        let stats = Arc::new(Mutex::new(Stats {
            irq: irq.as_ref().map(|irq| irq.path.clone()).unwrap_or_else(|| "off".into()),
            ..Stats::default()
        }));
        let mmu = Mmu {
            hub: Regs { base: hub.base },
            scratch_phys,
            spaces: BTreeMap::from([(DAEMON_SPACE, daemon_space)]),
            next_space: DAEMON_SPACE,
            loaded: DAEMON_SPACE,
            overflow: Vec::new(),
            stats: stats.clone(),
        };
        let mut v3d = V3d {
            hub,
            core,
            pm,
            asb,
            mmu: Arc::new(Mutex::new(mmu)),
            stats,
            irq,
            core_seen: 0,
            hub_seen: 0,
            overflow: Vec::new(),
        };
        if !v3d.hw_setup(&lock(&v3d.mmu)) {
            fail("MMU flush did not finish");
        }
        say(&format!("MMU on: v3dd's page table at {pt_phys:#x}, scratch page at {scratch_phys:#x}"));
        let overflow = lock(&v3d.mmu).new_overflow();
        match overflow {
            Ok(bo) => v3d.overflow.push(bo),
            Err(err) => fail(&format!("no binner overflow memory: {err}")),
        }
        lock(&v3d.stats).overflow_chunks = v3d.overflow.len();
        v3d
    }

    /// v3d_init_core() and v3d_mmu_set_page_table(): after power-up and
    /// after every reset.
    fn hw_setup(&self, mmu: &Mmu) -> bool {
        self.core.write(V3D_CTL_L2TFLSTA, 0);
        self.core.write(V3D_CTL_L2TFLEND, !0);
        self.hub.write(V3D_MMU_PT_PA_BASE, (mmu.pt_phys() / PAGE) as u32);
        self.hub.write(V3D_MMU_CTL, V3D_MMU_CTL_ENABLES);
        self.hub.write(V3D_MMU_ILLEGAL_ADDR, (mmu.scratch_phys / PAGE) as u32 | V3D_MMU_ILLEGAL_ADDR_ENABLE);
        self.hub.write(V3D_MMUC_CONTROL, V3D_MMUC_CONTROL_ENABLE);
        if self.irq.is_some() {
            // v3d_irq_enable(): ours unmasked, all others masked.
            self.core.write(V3D_CTL_INT_CLR, V3D_CORE_IRQS);
            self.hub.write(V3D_HUB_INT_CLR, V3D_HUB_IRQS);
            self.core.write(V3D_CTL_INT_MSK_SET, !V3D_CORE_IRQS);
            self.core.write(V3D_CTL_INT_MSK_CLR, V3D_CORE_IRQS);
            self.hub.write(V3D_HUB_INT_MSK_SET, !V3D_HUB_IRQS);
            self.hub.write(V3D_HUB_INT_MSK_CLR, V3D_HUB_IRQS);
        }
        csd::mmu_flush(&self.hub)
    }

    /// v3d_irq() and v3d_hub_irq(): takes the status bits, clears them in
    /// V3D (the line is level triggered), keeps them for the waiting code,
    /// then lets the interrupt controller deliver the next one.
    /// The read returns nothing when no interrupt is pending: then there is
    /// nothing to ack either. Returns whether one was handled.
    fn handle_irq(&mut self) -> bool {
        let mut token = [0u8; 8];
        let Some(irq) = self.irq.as_mut() else { return false };
        if !matches!(irq.file.read(&mut token), Ok(8)) {
            return false;
        }
        let core = self.core.read(V3D_CTL_INT_STS) & V3D_CORE_IRQS;
        let hub = self.hub.read(V3D_HUB_INT_STS) & V3D_HUB_IRQS;
        self.core.write(V3D_CTL_INT_CLR, core);
        self.hub.write(V3D_HUB_INT_CLR, hub);
        self.core_seen |= core;
        self.hub_seen |= hub;
        lock(&self.stats).irqs += 1;
        let _ = irq.file.write(&token);
        true
    }

    /// Waits until `done`, at most `ms`: on the interrupt with the fallback
    /// timer, or by polling when there is no interrupt.
    fn wait_for(&mut self, what: &str, ms: u64, done: impl Fn(&V3d) -> bool) -> bool {
        let start = Instant::now();
        loop {
            if done(self) {
                return true;
            }
            if start.elapsed() > Duration::from_millis(ms) {
                say(&format!("timeout: {what} ({ms} ms)"));
                return false;
            }
            let Some(irq) = self.irq.as_mut() else {
                thread::yield_now();
                continue;
            };
            irq.arm(IRQ_FALLBACK);
            match irq.queue.next() {
                Some(Ok(event)) => match event.user_data {
                    IrqSource::Irq => {
                        lock(&self.stats).irq_wakeups += 1;
                        self.handle_irq();
                    }
                    IrqSource::Time => {
                        irq.armed = false;
                        let mut time = syscall::TimeSpec::default();
                        let _ = irq.timer.read(&mut time);
                        lock(&self.stats).timer_wakeups += 1;
                    }
                },
                Some(Err(err)) if err.is_interrupt() => {}
                Some(Err(err)) => {
                    say(&format!("interrupt: event queue failed ({err}): polling from now on"));
                    self.irq = None;
                    lock(&self.stats).irq = "off".into();
                }
                None => {
                    self.irq = None;
                    lock(&self.stats).irq = "off".into();
                }
            }
        }
    }

    /// v3d_reset() on the BCM2711: the reset controller is the PM's
    /// (bcm2835_reset_reset(): bridges closed, slave first, PM_V3DRSTN
    /// asserted, then released and the bridges opened, master first), then
    /// the core and the MMU set up again. The jobs in flight are lost.
    fn reset(&mut self) {
        let mmu = self.mmu.clone();
        let mmu = lock(&mmu);
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
        let setup = open && self.hw_setup(&mmu);
        lock(&self.stats).resets += 1;
        self.core_seen = 0;
        self.hub_seen = 0;
        say(&format!(
            "reset: {} (resets so far: {})",
            if setup { "done, MMU set up again" } else { "FAILED: V3D is not usable" },
            lock(&self.stats).resets
        ));
    }

    /// An MMU fault since the last call, cleared, as v3d_hub_irq() reports
    /// it: the address (MMU_VIO_ADDR << (va_width - 32)) and the unit.
    fn mmu_fault(&mut self) -> Option<String> {
        let hub = &self.hub;
        let ints = (hub.read(V3D_HUB_INT_STS) | self.hub_seen) & V3D_HUB_INT_MMU;
        self.hub_seen = 0;
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
        lock(&self.stats).mmu_faults += 1;
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
            lock(&self.stats).mmu_faults -= 1;
        }
        let result = run(self);
        // A job that was done before v3dd waited leaves its interrupt
        // pending; ack it now, or the line stays blocked for the next job.
        self.handle_irq();
        // The register dump before mmu_fault() clears the fault state.
        if result.is_err() {
            self.state();
        }
        let fault = self.mmu_fault();
        if result.is_err() {
            self.reset();
        }
        watchdog.stop();
        lock(&self.stats).jobs += 1;
        match (result, fault) {
            (Ok(value), None) => Ok(value),
            (Ok(_), Some(fault)) => {
                lock(&self.stats).failed_jobs += 1;
                say(&format!("job ended with an MMU fault: {fault}"));
                Err(format!("mmu-fault {fault}"))
            }
            (Err(err), None) => {
                lock(&self.stats).failed_jobs += 1;
                say(&format!("job failed: {err}; V3D reset"));
                Err(format!("{err}; V3D reset"))
            }
            (Err(err), Some(fault)) => {
                lock(&self.stats).failed_jobs += 1;
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
            let finished = v3d.wait_for("CSD job", 1000, |v3d| {
                let s = v3d.core.read(V3D_CSD_STATUS);
                (s >> 4) & 0xff != completed_before && s & 0x3 == 0
            });
            v3d.core_seen &= !V3D_INT_CSDDONE;
            let us = start.elapsed().as_micros();
            if !finished {
                return Err(format!("timeout: CSD job not done after {us} us"));
            }
            v3d.clean_caches();
            Ok(us)
        })?;
        {
            let mut stats = lock(&self.stats);
            stats.job_us_max = stats.job_us_max.max(us);
        }
        Ok(us)
    }

    /// One bin job (v3d_bin_job_run()), the binner's OUTOMEM answered with
    /// more overflow memory (v3d_overflow_mem_work()), then one render job
    /// (v3d_render_job_run()). Times in microseconds.
    fn bin_render(&mut self, bcl: (u32, u32), rcl: (u32, u32), qma: u32, qms: u32, qts: u32)
        -> std::result::Result<(u128, u128), String> {
        lock(&self.stats).cl_jobs += 1;
        let (bin, render) = self.job(|v3d| v3d.bin_render_armed(bcl, rcl, qma, qms, qts))?;
        {
            let mut stats = lock(&self.stats);
            stats.job_us_max = stats.job_us_max.max(bin + render);
        }
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
        let binned = |v3d: &V3d| v3d.core.read(V3D_CLE_BFC) & 0xff != bfc;
        let outomem = |v3d: &V3d| (v3d.core.read(V3D_CTL_INT_STS) | v3d.core_seen) & V3D_INT_OUTOMEM != 0;
        loop {
            let left = 1000u64.saturating_sub(start.elapsed().as_millis() as u64);
            if !self.wait_for("bin job", left, |v3d| binned(v3d) || outomem(v3d)) {
                return Err(format!("timeout: bin job not done after {} us", start.elapsed().as_micros()));
            }
            if binned(self) {
                break;
            }
            {
                // Edge signalled: cleared first, set again only after the
                // binner has used the new memory up too.
                self.core_seen &= !V3D_INT_OUTOMEM;
                self.core.write(V3D_CTL_INT_CLR, V3D_INT_OUTOMEM);
                if next == self.overflow.len() {
                    if next == OVERFLOW_CHUNKS_MAX {
                        return Err(format!("bin job: out of memory after {next} overflow chunks"));
                    }
                    let bo = lock(&self.mmu).new_overflow();
                    let bo = bo.map_err(|err| format!("bin job: overflow memory: {err}"))?;
                    self.overflow.push(bo);
                    lock(&self.stats).overflow_chunks = self.overflow.len();
                }
                self.core.write(V3D_PTB_BPOA, self.overflow[next].va);
                self.core.write(V3D_PTB_BPOS, OVERFLOW_SIZE as u32);
                next += 1;
                lock(&self.stats).outomem += 1;
            }
        }
        self.core_seen &= !V3D_INT_FLDONE;
        let bin_us = start.elapsed().as_micros();

        self.invalidate_caches();
        let rfc = self.core.read(V3D_CLE_RFC) & 0xff;
        self.core.write(V3D_CLE_CT1QBA, rcl.0);
        let start = Instant::now();
        self.core.write(V3D_CLE_CT1QEA, rcl.1);
        let rendered = self.wait_for("render job", 1000, |v3d| v3d.core.read(V3D_CLE_RFC) & 0xff != rfc);
        self.core_seen &= !V3D_INT_FRDONE;
        if !rendered {
            return Err(format!("timeout: render job not done after {} us", start.elapsed().as_micros()));
        }
        let render_us = start.elapsed().as_micros();
        // As after a CSD job, so the CPU sees what the TLB stored.
        self.clean_caches();
        Ok((bin_us, render_us))
    }
}

/// A job in the queue. Job numbers count up from 1 over all clients; the
/// queue runs in that order, so job N is done once `done_through` >= N.
struct Job {
    id: u64,
    client: usize,
    /// The client's address space, loaded before the job runs.
    space: u32,
    kind: JobKind,
}

enum JobKind {
    Csd([u32; 7]),
    Cl([u32; 7]),
}

/// A read the request loop held back until a job is done. The Op points
/// into the client's buffer, which the kernel keeps until the answer.
struct HeldRead {
    caller: CallerCtx,
    op: Op,
}

// The job thread answers the read; the Op is not used anywhere else.
unsafe impl Send for HeldRead {}

/// Results kept for a client that has not asked for them yet; older ones go.
const RESULTS_MAX: usize = 256;

struct Client {
    reply: Vec<u8>,
    bos: BTreeMap<u32, Bo>,
    next_id: u32,
    /// The client's jobs not yet done (queued or running).
    pending: BTreeSet<u64>,
    /// Answers of done jobs nobody asked for yet.
    results: BTreeMap<u64, String>,
    /// The job the next read waits for (wait, or the old csd/cl).
    waiting: Option<u64>,
    held: Option<HeldRead>,
    /// The client's address space, made with its first BO.
    space: Option<u32>,
    /// The last job that named each BO in its submit (binary interface).
    bo_jobs: BTreeMap<u32, u64>,
    /// WAIT_BO and SYNCOBJ_WAIT calls held until a job is done.
    held_calls: Vec<(u64, HeldCall)>,
    /// DRM sync objects: None has no fence yet (made unsignalled, or
    /// reset); Some(job) is signalled once that job is done (job 0: at
    /// once).
    syncobjs: BTreeMap<u32, Option<u64>>,
    next_syncobj: u32,
    /// The BO of the last imported PRIME fd, until PRIME_FD_TO_HANDLE
    /// takes it: the kernel answers an fd-passing call with the number of
    /// fds, not with the scheme's answer, so relibc asks for the handle
    /// with a second call.
    prime_import: Option<u32>,
}

/// A held binary-interface call and how to answer it.
struct HeldCall {
    held: HeldRead,
    /// WAIT_BO: EIO if the job it waits for failed.
    wait_bo: bool,
    /// SYNCOBJ_WAIT for any: the index of the handle that is signalled
    /// when the job it waits for is done.
    first_signaled: Option<u32>,
}

/// Where a SYNCOBJ_WAIT stands.
enum SyncWait {
    /// Done; the first signalled handle (for a wait on any).
    Done(u32),
    /// Waits for this job; the index of its handle.
    Job(u64, u32),
    /// Waits for a submit that has not come (WAIT_FOR_SUBMIT): v3dd does
    /// not hold such a wait, it answers ETIME.
    Submit,
}

fn sync_wait(client: &Client, handles: &[u32], flags: u32, done: u64) -> Result<SyncWait> {
    if flags & !(uapi::SYNCOBJ_WAIT_ALL | uapi::SYNCOBJ_WAIT_FOR_SUBMIT | uapi::SYNCOBJ_WAIT_AVAILABLE) != 0 || handles.is_empty() {
        return Err(Error::new(EINVAL));
    }
    let all = flags & uapi::SYNCOBJ_WAIT_ALL != 0;
    // The latest job (all) or the earliest (any) not done yet.
    let mut wait: Option<(u64, u32)> = None;
    let mut unsubmitted = false;
    for (i, h) in handles.iter().enumerate() {
        let i = i as u32;
        match *client.syncobjs.get(h).ok_or(Error::new(ENOENT))? {
            None if flags & uapi::SYNCOBJ_WAIT_FOR_SUBMIT == 0 => return Err(Error::new(EINVAL)),
            None => unsubmitted = true,
            Some(job) if job <= done => {
                if !all {
                    return Ok(SyncWait::Done(i));
                }
            }
            Some(job) => {
                let later = |(w, _): (u64, u32)| if all { job > w } else { job < w };
                if wait.is_none_or(later) {
                    wait = Some((job, i));
                }
            }
        }
    }
    Ok(match (unsubmitted, wait) {
        (true, _) if all => SyncWait::Submit,
        (_, Some((job, i))) => SyncWait::Job(job, i),
        (true, None) => SyncWait::Submit,
        (false, None) => SyncWait::Done(0),
    })
}

/// CLOCK_MONOTONIC in ns, the clock of SYNCOBJ_WAIT's timeout.
fn monotonic_ns() -> i64 {
    let mut time = syscall::TimeSpec::default();
    match File::open(format!("/scheme/time/{}", syscall::CLOCK_MONOTONIC)).and_then(|mut f| f.read(&mut time)) {
        Ok(_) => time.tv_sec * 1_000_000_000 + time.tv_nsec as i64,
        Err(_) => 0,
    }
}

/// A new BO in the client's own address space (made with its first BO):
/// its ID and V3D address.
fn client_bo(shared: &Shared, client: &mut Client, bytes: usize) -> std::result::Result<(u32, u32), String> {
    let bo = {
        let mut mmu = lock(&shared.mmu);
        let space = match client.space {
            Some(space) => space,
            None => {
                let space = mmu.new_space()?;
                client.space = Some(space);
                space
            }
        };
        mmu.new_bo(space, bytes)?
    };
    let (bo_id, va) = (client.next_id, bo.va);
    client.next_id += 1;
    client.bos.insert(bo_id, bo);
    Ok((bo_id, va))
}

/// Imports another driver's buffer from the PRIME fd a client sent
/// (PRIME_FD_TO_HANDLE across drivers, see relibc): maps the fd into
/// v3dd, looks up the physical pages of that mapping and maps them into the
/// client's V3D address space. The new BO's handle.
fn client_import(shared: &Shared, client: &mut Client, file: File) -> std::result::Result<(u32, usize), String> {
    use std::io::{Seek as _, SeekFrom};
    let mut file = file;
    let bytes = file.seek(SeekFrom::End(0)).map_err(|err| format!("size of the PRIME fd: {err}"))? as usize;
    if bytes == 0 || bytes > IMPORT_MAX || bytes % PAGE != 0 {
        return Err(format!("buffer of {bytes} bytes (1..={IMPORT_MAX}, whole pages)"));
    }
    let map = syscall::Map {
        offset: 0,
        size: bytes,
        flags: MapFlags::PROT_READ | MapFlags::PROT_WRITE | MapFlags::MAP_SHARED,
        ..Default::default()
    };
    let addr = unsafe { syscall::fmap(file.as_raw_fd() as usize, &map) }.map_err(|err| format!("fmap: {err}"))?;
    let imported = Imported { _file: file, addr, bytes };
    let translate = common::VirtaddrTranslationHandle::new().map_err(|err| format!("translation handle: {err}"))?;
    let phys = (0..bytes / PAGE)
        .map(|page| translate.translate(addr + page * PAGE).map_err(|err| format!("page {page}: {err}")))
        .collect::<std::result::Result<Vec<usize>, String>>()?;
    let contiguous = phys.windows(2).all(|w| w[1] == w[0] + PAGE);
    let bo = {
        let mut mmu = lock(&shared.mmu);
        let space = match client.space {
            Some(space) => space,
            None => {
                let space = mmu.new_space()?;
                client.space = Some(space);
                space
            }
        };
        mmu.import_bo(space, imported, &phys)?
    };
    let bo_id = client.next_id;
    client.next_id += 1;
    say(&format!(
        "imported a buffer of {bytes} bytes as BO {bo_id} at V3D {:#x} ({} pages, {})",
        bo.va,
        phys.len(),
        if contiguous { "contiguous" } else { "scattered" }
    ));
    client.bos.insert(bo_id, bo);
    Ok((bo_id, bytes))
}

/// Frees a client's BO now, or after the client's last pending job.
fn client_free(shared: &Shared, zombies: &mut Vec<(u64, Zombie)>, client: &mut Client, bo_id: u32) -> bool {
    let Some(bo) = client.bos.remove(&bo_id) else { return false };
    client.bo_jobs.remove(&bo_id);
    match client.pending.last().copied() {
        Some(job) => zombies.push((job, Zombie::Bo(bo))),
        None => lock(&shared.mmu).free_bo(bo),
    }
    true
}

/// Queues a job; its number.
fn submit_job(shared: &Shared, queue: &mut VecDeque<Job>, next_job: &mut u64, id: usize, client: &mut Client, kind: JobKind) -> u64 {
    *next_job += 1;
    let job = *next_job;
    client.pending.insert(job);
    let space = client.space.unwrap_or(DAEMON_SPACE);
    queue.push_back(Job { id: job, client: id, space, kind });
    shared.work.notify_one();
    job
}

/// Freed once the job it waits for is done, in the order they came.
enum Zombie {
    Bo(Bo),
    Space(u32),
}

enum Handle {
    Root,
    Client(Client),
}

struct State {
    handles: HandleMap<Handle>,
    queue: VecDeque<Job>,
    next_job: u64,
    running: Option<u64>,
    done_through: u64,
    /// BOs freed (or closed) while jobs of their client were pending, and
    /// the spaces of clients closed meanwhile, with the last such job.
    zombies: Vec<(u64, Zombie)>,
}

struct Shared {
    /// GET_PARAM's identification registers, read at start.
    params: [u64; 7],
    v3d: Mutex<V3d>,
    mmu: Arc<Mutex<Mmu>>,
    stats: Arc<Mutex<Stats>>,
    state: Mutex<State>,
    work: Condvar,
    socket: Socket,
}

impl Shared {
    /// Frees the zombies whose jobs are done, in order: a client's BOs
    /// before its space. Lock order: state, then mmu.
    fn reap(&self, state: &mut State) {
        let done = state.done_through;
        let (due, keep): (Vec<_>, Vec<_>) = std::mem::take(&mut state.zombies).into_iter().partition(|(job, _)| *job <= done);
        state.zombies = keep;
        if !due.is_empty() {
            let mut mmu = lock(&self.mmu);
            for (_, zombie) in due {
                bury(&mut mmu, zombie);
            }
        }
    }

    /// Frees a BO or a space now, or after the client's last pending job.
    fn release(&self, state: &mut State, last_pending: Option<u64>, zombie: Zombie) {
        match last_pending {
            Some(job) => state.zombies.push((job, zombie)),
            None => bury(&mut lock(&self.mmu), zombie),
        }
    }

    /// The job thread: runs the queue in order, one job at a time.
    fn run_jobs(self: Arc<Shared>) -> ! {
        let mut scheme = V3dScheme { shared: self.clone() };
        let mut scheme_state = SchemeState::new();
        loop {
            let job = {
                let mut state = lock(&self.state);
                loop {
                    if let Some(job) = state.queue.pop_front() {
                        state.running = Some(job.id);
                        break job;
                    }
                    state = self.work.wait(state).unwrap_or_else(|poisoned| poisoned.into_inner());
                }
            };
            let answer = {
                let mut v3d = lock(&self.v3d);
                let loaded = lock(&self.mmu).load(job.space);
                if let Err(err) = loaded {
                    Err(format!("address space: {err}"))
                } else {
                    match job.kind {
                    JobKind::Csd(cfg) => v3d.csd(cfg).map(|us| format!("done {us}")),
                    JobKind::Cl(a) => v3d
                        .bin_render((a[0], a[1]), (a[2], a[3]), a[4], a[5], a[6])
                        .map(|(bin, render)| format!("done {} bin {bin} render {render}", bin + render)),
                    }
                }
            };
            let answer = answer.unwrap_or_else(|err| format!("error {err}"));
            let failed = answer.starts_with("error");
            let mut calls = Vec::new();
            let held = {
                let mut state = lock(&self.state);
                state.running = None;
                state.done_through = job.id;
                let held = match state.handles.get_mut(job.client) {
                    Ok(Handle::Client(client)) => {
                        client.pending.remove(&job.id);
                        // WAIT_BOs on this job (jobs end in order: also on
                        // earlier ones, if any were left).
                        let (due, keep): (Vec<_>, Vec<_>) =
                            std::mem::take(&mut client.held_calls).into_iter().partition(|(waits, _)| *waits <= job.id);
                        client.held_calls = keep;
                        calls = due;
                        if client.waiting == Some(job.id) {
                            client.waiting = None;
                            client.reply = format!("{answer}\n").into_bytes();
                            client.held.take()
                        } else {
                            client.results.insert(job.id, answer);
                            while client.results.len() > RESULTS_MAX {
                                client.results.pop_first();
                            }
                            None
                        }
                    }
                    _ => None,
                };
                self.reap(&mut state);
                held
            };
            for (waits, call) in calls {
                let HeldCall { held: HeldRead { mut op, .. }, wait_bo, first_signaled } = call;
                let result = if wait_bo && failed && waits == job.id { Err(Error::new(EIO)) } else { Ok(0) };
                if let (Some(i), Op::Call(call)) = (first_signaled, &mut op) {
                    let (payload, _) = call.payload_and_metadata();
                    let at = uapi::SYNCOBJ_WAIT_FIRST_SIGNALED;
                    if let Some(field) = payload.get_mut(at..at + 4) {
                        field.copy_from_slice(&i.to_ne_bytes());
                    }
                }
                if let Err(err) = self.socket.write_response(Response::new(result, op), SignalBehavior::Restart) {
                    say(&format!("answer to a held wait failed: {err}"));
                }
            }
            if let Some(HeldRead { caller, op }) = held {
                let response = op.handle_sync(caller, &mut scheme, &mut scheme_state);
                if let Err(err) = self.socket.write_response(response, SignalBehavior::Restart) {
                    say(&format!("answer to a held read failed: {err}"));
                }
            }
        }
    }
}

fn bury(mmu: &mut Mmu, zombie: Zombie) {
    match zombie {
        Zombie::Bo(bo) => mmu.free_bo(bo),
        Zombie::Space(id) => mmu.remove_space(id),
    }
}

struct V3dScheme {
    shared: Arc<Shared>,
}

fn number(s: &str) -> std::result::Result<u64, String> {
    let r = match s.strip_prefix("0x") {
        Some(hex) => u64::from_str_radix(hex, 16),
        None => s.parse(),
    };
    r.map_err(|_| format!("not a number: {s}"))
}

/// The answer of a write: a line now, or none yet (the read waits for a job).
enum Answer {
    Now(String),
    Later,
}

impl V3dScheme {
    fn command(&mut self, state: &mut State, id: usize, line: &str) -> std::result::Result<Answer, String> {
        let shared = self.shared.clone();
        let State { handles, queue, next_job, running, done_through, zombies } = state;
        let Ok(Handle::Client(client)) = handles.get_mut(id) else {
            return Err("not a client handle".into());
        };
        let words: Vec<&str> = line.split_whitespace().collect();
        let arg = |i: usize| -> std::result::Result<u64, String> {
            number(words.get(i).ok_or_else(|| format!("{}: missing argument {i}", words[0]))?)
        };
        let args7 = |from: usize| -> std::result::Result<[u32; 7], String> {
            let mut a = [0u32; 7];
            for (i, v) in a.iter_mut().enumerate() {
                *v = arg(from + i)? as u32;
            }
            Ok(a)
        };
        let mut submit = |client: &mut Client, kind: JobKind| -> u64 { submit_job(&shared, queue, next_job, id, client, kind) };
        match words.first().copied() {
            Some("bo") => {
                let (bo_id, va) = client_bo(&shared, client, arg(1)? as usize)?;
                Ok(Answer::Now(format!("{bo_id} {va:#x}")))
            }
            Some("put") => {
                let bo = client.bos.get_mut(&(arg(1)? as u32)).ok_or("no such BO")?;
                let first = arg(2)? as usize;
                let values: Vec<u32> =
                    words[3..].iter().map(|w| number(w).map(|v| v as u32)).collect::<std::result::Result<_, _>>()?;
                if first + values.len() > bo.mem.len() {
                    return Err("past the end of the BO".into());
                }
                bo.mem[first..first + values.len()].copy_from_slice(&values);
                Ok(Answer::Now("ok".into()))
            }
            Some("get") => {
                let bo = client.bos.get(&(arg(1)? as u32)).ok_or("no such BO")?;
                let (first, count) = (arg(2)? as usize, arg(3)? as usize);
                if first + count > bo.mem.len() {
                    return Err("past the end of the BO".into());
                }
                Ok(Answer::Now(bo.mem[first..first + count].iter().map(|w| format!("{w:#x}")).collect::<Vec<_>>().join(" ")))
            }
            Some("fill") => {
                let bo = client.bos.get_mut(&(arg(1)? as u32)).ok_or("no such BO")?;
                let (first, count, value) = (arg(2)? as usize, arg(3)? as usize, arg(4)? as u32);
                if first + count > bo.mem.len() {
                    return Err("past the end of the BO".into());
                }
                bo.mem[first..first + count].fill(value);
                Ok(Answer::Now("ok".into()))
            }
            Some("free") => {
                if !client_free(&shared, zombies, client, arg(1)? as u32) {
                    return Err("no such BO".into());
                }
                Ok(Answer::Now("ok".into()))
            }
            // The old blocking forms: submit, then the read waits.
            Some("csd") => {
                let job = submit(client, JobKind::Csd(args7(1)?));
                client.waiting = Some(job);
                Ok(Answer::Later)
            }
            Some("cl") => {
                let job = submit(client, JobKind::Cl(args7(1)?));
                client.waiting = Some(job);
                Ok(Answer::Later)
            }
            Some("submit") => {
                let kind = match words.get(1).copied() {
                    Some("csd") => JobKind::Csd(args7(2)?),
                    Some("cl") => JobKind::Cl(args7(2)?),
                    _ => return Err("submit: csd or cl".into()),
                };
                Ok(Answer::Now(format!("job {}", submit(client, kind))))
            }
            Some(cmd @ ("wait" | "poll")) => {
                let job = arg(1)?;
                if let Some(answer) = client.results.remove(&job) {
                    Ok(Answer::Now(answer))
                } else if !client.pending.contains(&job) {
                    Err(format!("no job {job} of this client (done ones answer once)"))
                } else if cmd == "poll" {
                    Ok(Answer::Now(if *running == Some(job) { "running".into() } else { "queued".into() }))
                } else {
                    client.waiting = Some(job);
                    Ok(Answer::Later)
                }
            }
            Some("stats") => {
                let (spaces, loaded) = {
                    let mmu = lock(&shared.mmu);
                    (mmu.spaces.len() - 1, mmu.loaded)
                };
                let s = lock(&shared.stats);
                Ok(Answer::Now(format!(
                    "bos {} frees {} jobs {} cl_jobs {} failed {} mmu_faults {} resets {} job_us_max {} outomem {} overflow_chunks {} spaces {} spaces_made {} space_loaded {} irq {} irqs {} irq_wakeups {} timer_wakeups {} maps {} unmaps {} live_bos {} submitted {} done_through {} queued {} running {} zombies {}",
                    s.bos,
                    s.frees,
                    s.jobs,
                    s.cl_jobs,
                    s.failed_jobs,
                    s.mmu_faults,
                    s.resets,
                    s.job_us_max,
                    s.outomem,
                    s.overflow_chunks,
                    spaces,
                    s.spaces,
                    loaded,
                    s.irq,
                    s.irqs,
                    s.irq_wakeups,
                    s.timer_wakeups,
                    s.maps,
                    s.unmaps,
                    // Client BOs, held or zombie: the overflow chunks are v3dd's.
                    s.bos - s.frees - s.overflow_chunks as u64,
                    *next_job,
                    *done_through,
                    queue.len(),
                    running.map_or("-".into(), |job| job.to_string()),
                    zombies.len()
                )))
            }
            _ => Err(format!("unknown command: {line}")),
        }
    }
}

impl V3dScheme {
    /// The binary interface (see v3d/uapi.rs): one V3D ioctl.
    fn ioctl(&mut self, state: &mut State, id: usize, func: u64, payload: &mut [u8]) -> Result<usize> {
        let shared = self.shared.clone();
        let State { handles, queue, next_job, done_through, zombies, .. } = state;
        let Ok(Handle::Client(client)) = handles.get_mut(id) else {
            return Err(Error::new(EBADF));
        };
        let einval = || Error::new(EINVAL);
        match func {
            uapi::VERSION => {
                let (mut arg, _) = uapi::read::<uapi::Version>(payload).ok_or_else(einval)?;
                let fixed = size_of::<uapi::Version>();
                let texts = [uapi::DRIVER_NAME, uapi::DRIVER_DATE, uapi::DRIVER_DESC];
                uapi::put_strings(&mut payload[fixed..], &[arg.name_len, arg.date_len, arg.desc_len], &texts)
                    .ok_or_else(einval)?;
                (arg.version_major, arg.version_minor, arg.version_patchlevel) = (1, 0, 0);
                (arg.name_len, arg.date_len, arg.desc_len) = (texts[0].len() as u64, texts[1].len() as u64, texts[2].len() as u64);
                uapi::write(payload, &arg);
                Ok(0)
            }
            uapi::GET_UNIQUE => {
                let (mut arg, _) = uapi::read::<uapi::Unique>(payload).ok_or_else(einval)?;
                let fixed = size_of::<uapi::Unique>();
                uapi::put_strings(&mut payload[fixed..], &[arg.unique_len], &[uapi::DRIVER_NAME]).ok_or_else(einval)?;
                arg.unique_len = uapi::DRIVER_NAME.len() as u64;
                uapi::write(payload, &arg);
                Ok(0)
            }
            uapi::SET_VERSION => {
                // Interface 1.4 as Linux answers; the driver version is
                // only checked, it does not change anything.
                let (mut arg, _) = uapi::read::<uapi::SetVersion>(payload).ok_or_else(einval)?;
                if arg.di_major > 1 || (arg.di_major == 1 && arg.di_minor > 4) || arg.dd_major > 1 {
                    return Err(einval());
                }
                arg = uapi::SetVersion { di_major: 1, di_minor: 4, dd_major: 1, dd_minor: 0 };
                uapi::write(payload, &arg);
                Ok(0)
            }
            // After an fd-passing call (on_sendfd): the handle of the BO
            // it imported.
            uapi::PRIME_FD_TO_HANDLE if payload.is_empty() => {
                client.prime_import.take().map(|handle| handle as usize).ok_or(Error::new(ENOENT))
            }
            uapi::GET_CAP => {
                let (mut arg, _) = uapi::read::<uapi::GetCap>(payload).ok_or_else(einval)?;
                arg.value = match arg.capability {
                    uapi::CAP_SYNCOBJ => 1,
                    // PRIME: import of other drivers' buffers (G4 step 2),
                    // no export.
                    uapi::CAP_PRIME => uapi::PRIME_CAP_IMPORT,
                    // No dumb buffers, timelines or the rest.
                    1..=uapi::CAP_SYNCOBJ_TIMELINE => 0,
                    _ => return Err(einval()),
                };
                uapi::write(payload, &arg);
                Ok(0)
            }
            uapi::CREATE_BO => {
                let (mut arg, _) = uapi::read::<uapi::CreateBo>(payload).ok_or_else(einval)?;
                if arg.flags != 0 || arg.size == 0 || arg.size as usize > BO_MAX {
                    return Err(einval());
                }
                let (handle, va) = client_bo(&shared, client, arg.size as usize).map_err(|err| {
                    say(&format!("client {id}: CREATE_BO: {err}"));
                    Error::new(ENOMEM)
                })?;
                arg.handle = handle;
                arg.offset = va;
                uapi::write(payload, &arg);
                Ok(0)
            }
            uapi::MMAP_BO => {
                let (mut arg, _) = uapi::read::<uapi::MmapBo>(payload).ok_or_else(einval)?;
                if arg.flags != 0 {
                    return Err(einval());
                }
                client.bos.get(&arg.handle).ok_or(Error::new(ENOENT))?;
                arg.offset = (arg.handle as u64) << MAP_ID_SHIFT;
                uapi::write(payload, &arg);
                Ok(0)
            }
            uapi::GET_BO_OFFSET => {
                let (mut arg, _) = uapi::read::<uapi::GetBoOffset>(payload).ok_or_else(einval)?;
                arg.offset = client.bos.get(&arg.handle).ok_or(Error::new(ENOENT))?.va;
                uapi::write(payload, &arg);
                Ok(0)
            }
            uapi::GEM_CLOSE => {
                let (arg, _) = uapi::read::<uapi::GemClose>(payload).ok_or_else(einval)?;
                if !client_free(&shared, zombies, client, arg.handle) {
                    return Err(Error::new(ENOENT));
                }
                Ok(0)
            }
            uapi::GET_PARAM => {
                let (mut arg, _) = uapi::read::<uapi::GetParam>(payload).ok_or_else(einval)?;
                arg.value = match arg.param {
                    p @ uapi::PARAM_UIFCFG..=uapi::PARAM_CORE0_IDENT2 => shared.params[p as usize],
                    uapi::PARAM_SUPPORTS_CSD => 1,
                    // No TFU, cache flush, perfmon, multisync, CPU queue,
                    // performance counters or super pages.
                    uapi::PARAM_SUPPORTS_TFU..=uapi::PARAM_SUPPORTS_SUPER_PAGES => 0,
                    _ => return Err(einval()),
                };
                uapi::write(payload, &arg);
                Ok(0)
            }
            uapi::SUBMIT_CL | uapi::SUBMIT_CSD => {
                let (kind, count, rest, extensions, flags_ok, in_syncs, out_sync) = if func == uapi::SUBMIT_CL {
                    let (arg, rest) = uapi::read::<uapi::SubmitCl>(payload).ok_or_else(einval)?;
                    // A render job without a bin job is not supported yet.
                    if arg.bcl_start == arg.bcl_end {
                        return Err(einval());
                    }
                    let a = [arg.bcl_start, arg.bcl_end, arg.rcl_start, arg.rcl_end, arg.qma, arg.qms, arg.qts];
                    let flags_ok = arg.flags & !uapi::SUBMIT_CL_FLUSH_CACHE == 0;
                    (JobKind::Cl(a), arg.bo_handle_count, rest, arg.extensions, flags_ok, vec![arg.in_sync_bcl, arg.in_sync_rcl], arg.out_sync)
                } else {
                    let (arg, rest) = uapi::read::<uapi::SubmitCsd>(payload).ok_or_else(einval)?;
                    (JobKind::Csd(arg.cfg), arg.bo_handle_count, rest, arg.extensions, arg.flags == 0, vec![arg.in_sync], arg.out_sync)
                };
                // Jobs run in the order they came, so an in_sync of the
                // client's own jobs is met already; it only has to exist.
                // out_sync gets this job's fence.
                if extensions != 0 || !flags_ok {
                    return Err(einval());
                }
                if in_syncs.iter().chain([&out_sync]).any(|&s| s != 0 && !client.syncobjs.contains_key(&s)) {
                    return Err(Error::new(ENOENT));
                }
                let bos = uapi::handles(rest, count).ok_or_else(einval)?;
                if bos.iter().any(|h| !client.bos.contains_key(h)) {
                    return Err(Error::new(ENOENT));
                }
                let job = submit_job(&shared, queue, next_job, id, client, kind);
                for h in bos {
                    client.bo_jobs.insert(h, job);
                }
                if out_sync != 0 {
                    client.syncobjs.insert(out_sync, Some(job));
                }
                Ok(0)
            }
            uapi::SYNCOBJ_CREATE => {
                let (mut arg, _) = uapi::read::<uapi::SyncobjCreate>(payload).ok_or_else(einval)?;
                if arg.flags & !uapi::SYNCOBJ_CREATE_SIGNALED != 0 {
                    return Err(einval());
                }
                client.next_syncobj += 1;
                arg.handle = client.next_syncobj;
                let fence = (arg.flags & uapi::SYNCOBJ_CREATE_SIGNALED != 0).then_some(0);
                client.syncobjs.insert(arg.handle, fence);
                uapi::write(payload, &arg);
                Ok(0)
            }
            uapi::SYNCOBJ_DESTROY => {
                let (arg, _) = uapi::read::<uapi::SyncobjDestroy>(payload).ok_or_else(einval)?;
                client.syncobjs.remove(&arg.handle).ok_or(Error::new(ENOENT))?;
                Ok(0)
            }
            uapi::SYNCOBJ_RESET | uapi::SYNCOBJ_SIGNAL => {
                let (arg, rest) = uapi::read::<uapi::SyncobjArray>(payload).ok_or_else(einval)?;
                let syncs = uapi::handles(rest, arg.count_handles).ok_or_else(einval)?;
                if syncs.iter().any(|s| !client.syncobjs.contains_key(s)) {
                    return Err(Error::new(ENOENT));
                }
                let fence = (func == uapi::SYNCOBJ_SIGNAL).then_some(0);
                for s in syncs {
                    client.syncobjs.insert(s, fence);
                }
                Ok(0)
            }
            uapi::SYNCOBJ_WAIT => {
                // A wait that can still end before its timeout was held
                // (hold_op); here it is done, or it has timed out.
                let (mut arg, rest) = uapi::read::<uapi::SyncobjWait>(payload).ok_or_else(einval)?;
                let syncs = uapi::handles(rest, arg.count_handles).ok_or_else(einval)?;
                match sync_wait(client, &syncs, arg.flags, *done_through)? {
                    SyncWait::Done(first) => {
                        arg.first_signaled = first;
                        uapi::write(payload, &arg);
                        Ok(0)
                    }
                    SyncWait::Job(..) | SyncWait::Submit => Err(Error::new(ETIME)),
                }
            }
            uapi::WAIT_BO => {
                let (arg, _) = uapi::read::<uapi::WaitBo>(payload).ok_or_else(einval)?;
                client.bos.get(&arg.handle).ok_or(Error::new(ENOENT))?;
                match client.bo_jobs.get(&arg.handle) {
                    // A job of this BO not done: a wait with a timeout was
                    // held (hold_op), so only timeout 0 comes here.
                    Some(&job) if job > *done_through => Err(Error::new(ETIME)),
                    Some(&job) if client.results.get(&job).is_some_and(|r| r.starts_with("error")) => Err(Error::new(EIO)),
                    _ => Ok(0),
                }
            }
            _ => Err(Error::new(ENOSYS)),
        }
    }
}

impl SchemeSync for V3dScheme {
    fn call(&mut self, id: usize, payload: &mut [u8], metadata: &[u64], _ctx: &CallerCtx) -> Result<usize> {
        let shared = self.shared.clone();
        let mut state = lock(&shared.state);
        let func = *metadata.first().ok_or(Error::new(EINVAL))?;
        self.ioctl(&mut state, id, func, payload)
    }

    fn scheme_root(&mut self) -> Result<usize> {
        Ok(lock(&self.shared.state).handles.insert(Handle::Root))
    }

    fn openat(
        &mut self,
        dirfd: usize,
        path: &str,
        _flags: usize,
        _fcntl_flags: u32,
        ctx: &CallerCtx,
    ) -> Result<OpenResult> {
        let mut state = lock(&self.shared.state);
        if !matches!(state.handles.get(dirfd)?, Handle::Root) {
            return Err(Error::new(EACCES));
        }
        // GPU jobs can read and write any memory mapped for them: root only
        // for now.
        if ctx.uid != 0 || !path.trim_matches('/').is_empty() {
            return Err(Error::new(EACCES));
        }
        let handle = Handle::Client(Client {
            reply: Vec::new(),
            bos: BTreeMap::new(),
            next_id: 1,
            pending: BTreeSet::new(),
            results: BTreeMap::new(),
            waiting: None,
            held: None,
            space: None,
            bo_jobs: BTreeMap::new(),
            held_calls: Vec::new(),
            syncobjs: BTreeMap::new(),
            next_syncobj: 0,
            prime_import: None,
        });
        Ok(OpenResult::ThisScheme { number: state.handles.insert(handle), flags: NewFdFlags::empty() })
    }

    fn read(&mut self, id: usize, buf: &mut [u8], _offset: u64, _fcntl_flags: u32, _ctx: &CallerCtx) -> Result<usize> {
        let mut state = lock(&self.shared.state);
        let Handle::Client(Client { reply, .. }) = state.handles.get_mut(id)? else {
            return Err(Error::new(EBADF));
        };
        let len = buf.len().min(reply.len());
        buf[..len].copy_from_slice(&reply[..len]);
        reply.drain(..len);
        Ok(len)
    }

    fn write(&mut self, id: usize, buf: &[u8], _offset: u64, _fcntl_flags: u32, _ctx: &CallerCtx) -> Result<usize> {
        let shared = self.shared.clone();
        let mut state = lock(&shared.state);
        match state.handles.get_mut(id)? {
            // A new command while a wait is open (its read was cancelled,
            // or never came): the wait ends, and the job's answer stays for
            // wait or poll.
            Handle::Client(client) => {
                if let Some(job) = client.waiting.take() {
                    say(&format!("client {id}: new command while waiting for job {job}: the answer stays for wait {job}"));
                }
            }
            Handle::Root => return Err(Error::new(EBADF)),
        }
        let line = std::str::from_utf8(buf).map_err(|_| Error::new(EINVAL))?.trim();
        let reply = match self.command(&mut state, id, line) {
            Ok(Answer::Now(answer)) => Some(answer),
            Ok(Answer::Later) => None,
            Err(err) => Some(format!("error {err}")),
        };
        if let Ok(Handle::Client(client)) = state.handles.get_mut(id) {
            client.reply = reply.map(|answer| format!("{answer}\n").into_bytes()).unwrap_or_default();
        }
        Ok(buf.len())
    }

    /// fmap of a BO (see the top of the file). Returns where the BO lies in
    /// v3dd: the kernel maps the same pages into the client.
    fn mmap_prep(&mut self, id: usize, offset: u64, size: usize, flags: MapFlags, _ctx: &CallerCtx) -> Result<usize> {
        let state = lock(&self.shared.state);
        let Handle::Client(Client { bos, .. }) = state.handles.get(id)? else {
            return Err(Error::new(EBADF));
        };
        // A private (copy-on-write) mapping would also make v3dd's own
        // pages read-only, and V3D would never see the client's writes.
        if !flags.contains(MapFlags::MAP_SHARED) {
            return Err(Error::new(EINVAL));
        }
        let bo = bos.get(&((offset >> MAP_ID_SHIFT) as u32)).ok_or(Error::new(ENOENT))?;
        let at = (offset & ((1 << MAP_ID_SHIFT) - 1)) as usize;
        if at % PAGE != 0 || size == 0 || at.checked_add(size).is_none_or(|end| end > bo.pages * PAGE) {
            return Err(Error::new(EINVAL));
        }
        lock(&self.shared.stats).maps += 1;
        Ok(bo.mem.as_ptr() as usize + at)
    }

    /// The client unmapped (part of) a BO. The kernel keeps the pages
    /// until the last mapping goes, so there is nothing to free here.
    fn munmap(&mut self, _id: usize, _offset: u64, _size: usize, _flags: MunmapFlags, _ctx: &CallerCtx) -> Result<()> {
        lock(&self.shared.stats).unmaps += 1;
        Ok(())
    }

    /// A character device, so stat() of the render node
    /// /dev/dri/renderD128 (a link here) works: libdrm's drmGetDevices2
    /// skips a node it cannot stat, and Mesa then finds no device.
    fn fstat(&mut self, id: usize, stat: &mut syscall::Stat, _ctx: &CallerCtx) -> Result<()> {
        lock(&self.shared.state).handles.get(id).map_err(|_| Error::new(EBADF))?;
        stat.st_mode = syscall::MODE_CHR | 0o666;
        stat.st_nlink = 1;
        Ok(())
    }

    fn fpath(&mut self, id: usize, buf: &mut [u8], _ctx: &CallerCtx) -> Result<usize> {
        lock(&self.shared.state).handles.get(id).map_err(|_| Error::new(EBADF))?;
        FpathWriter::with(buf, "v3d", |_| Ok(()))
    }

    /// A client sent a PRIME fd of another driver (relibc's
    /// PRIME_FD_TO_HANDLE when the fd is not v3dd's): import it as a BO and
    /// answer its handle. The fd is taken in any case, so a refused import
    /// leaves nothing behind.
    fn on_sendfd(&mut self, req: &redox_scheme::SendFdRequest) -> Result<usize> {
        if req.num_fds() != 1 {
            return Err(Error::new(EINVAL));
        }
        let id = req.id();
        let mut fds = [usize::MAX];
        req.obtain_fd(&self.shared.socket, syscall::flag::FobtainFdFlags::empty(), &mut fds).map_err(|err| {
            say(&format!("client {id}: PRIME fd: obtaining it failed: {err}"));
            err
        })?;
        let file = unsafe { <File as std::os::unix::io::FromRawFd>::from_raw_fd(fds[0] as i32) };
        let shared = self.shared.clone();
        let mut state = lock(&shared.state);
        let Ok(Handle::Client(client)) = state.handles.get_mut(id) else {
            say(&format!("client {id}: PRIME fd sent to a handle that is not a client"));
            return Err(Error::new(EBADF));
        };
        let (handle, _) = client_import(&shared, client, file).map_err(|err| {
            say(&format!("client {id}: PRIME import: {err}"));
            Error::new(EINVAL)
        })?;
        client.prime_import = Some(handle);
        Ok(handle as usize)
    }

    /// The client's queued jobs go; a running one ends first, and its BOs
    /// stay until then.
    fn on_close(&mut self, id: usize) {
        let shared = self.shared.clone();
        let mut state = lock(&shared.state);
        let Some(Handle::Client(client)) = state.handles.remove(id) else {
            return;
        };
        state.queue.retain(|job| job.client != id);
        let last = state.running.filter(|job| client.pending.contains(job));
        for bo in client.bos.into_values() {
            shared.release(&mut state, last, Zombie::Bo(bo));
        }
        if let Some(space) = client.space {
            shared.release(&mut state, last, Zombie::Space(space));
        }
        // A held read goes with the file: the kernel cancelled it.
        drop(client.held);
    }
}

fn main() {
    daemon::SchemeDaemon::new(daemon);
}

/// What a held call waits on.
enum CallWait {
    Bo(u32),
    /// The handles and flags of a SYNCOBJ_WAIT.
    Syncobjs(Vec<u32>, u32),
}

/// Hands a request that must wait for a job to the job thread: a read on
/// a client that waits for a job, a WAIT_BO with a timeout on a BO whose
/// last job is not done, or a SYNCOBJ_WAIT whose timeout lies ahead (the
/// timeouts are not kept: the job ends within its own 1 s limit). Returns
/// the Op back if it can be answered now.
fn hold_op(shared: &Shared, caller: CallerCtx, mut op: Op) -> Option<(CallerCtx, Op)> {
    let call_wait = match &mut op {
        Op::Read(_) => None,
        Op::Call(call) => {
            let fd = call.fd();
            let (payload, metadata) = call.payload_and_metadata();
            match metadata.first() {
                Some(&uapi::WAIT_BO) => match uapi::read::<uapi::WaitBo>(payload) {
                    Some((arg, _)) if arg.timeout_ns != 0 => Some((fd, CallWait::Bo(arg.handle))),
                    _ => return Some((caller, op)),
                },
                Some(&uapi::SYNCOBJ_WAIT) => match uapi::read::<uapi::SyncobjWait>(payload) {
                    Some((arg, rest)) if arg.timeout_nsec > monotonic_ns() => match uapi::handles(rest, arg.count_handles) {
                        Some(syncs) => Some((fd, CallWait::Syncobjs(syncs, arg.flags))),
                        None => return Some((caller, op)),
                    },
                    _ => return Some((caller, op)),
                },
                _ => return Some((caller, op)),
            }
        }
        _ => return Some((caller, op)),
    };
    let mut state = lock(&shared.state);
    let done = state.done_through;
    if let Some((fd, wait)) = call_wait {
        let Ok(Handle::Client(client)) = state.handles.get_mut(fd) else { return Some((caller, op)) };
        let (job, wait_bo, first_signaled) = match wait {
            CallWait::Bo(handle) => match client.bo_jobs.get(&handle) {
                Some(&job) if job > done => (job, true, None),
                _ => return Some((caller, op)),
            },
            CallWait::Syncobjs(syncs, flags) => match sync_wait(client, &syncs, flags, done) {
                Ok(SyncWait::Job(job, i)) => (job, false, (flags & uapi::SYNCOBJ_WAIT_ALL == 0).then_some(i)),
                _ => return Some((caller, op)),
            },
        };
        client.held_calls.push((job, HeldCall { held: HeldRead { caller, op }, wait_bo, first_signaled }));
        return None;
    }
    let Op::Read(read) = &op else { return Some((caller, op)) };
    let fd = read.fd;
    match state.handles.get_mut(fd) {
        Ok(Handle::Client(client)) if client.waiting.is_some() && client.held.is_none() => {
            client.held = Some(HeldRead { caller, op });
            None
        }
        Ok(Handle::Client(client)) if client.waiting.is_some() => {
            say(&format!("client {fd}: a second read while one waits: answered now, empty"));
            Some((caller, op))
        }
        _ => Some((caller, op)),
    }
}

/// The request loop. A read that waits for a job is held, and the job
/// thread answers it; every other request is answered here at once.
fn serve(shared: Arc<Shared>) -> ! {
    let mut scheme = V3dScheme { shared: shared.clone() };
    let mut scheme_state = SchemeState::new();
    loop {
        let request = match shared.socket.next_request(SignalBehavior::Restart) {
            Ok(Some(request)) => request,
            Ok(None) => {
                say("scheme socket closed");
                process::exit(0);
            }
            Err(err) => fail(&format!("reading requests: {err}")),
        };
        let response = match request.kind() {
            RequestKind::Call(call) => {
                let caller = call.caller();
                match call.op() {
                    Ok(op) => match hold_op(&shared, caller, op) {
                        Some((caller, op)) => op.handle_sync(caller, &mut scheme, &mut scheme_state),
                        None => continue,
                    },
                    Err(call) => Response::new(Err(Error::new(ENOSYS)), call),
                }
            }
            RequestKind::Cancellation(cancel) => {
                // A held read whose caller gave up (a signal): answer EINTR.
                // The wait stays open: a read again is held again, a new
                // command ends it.
                let held = {
                    let mut state = lock(&shared.state);
                    let mut found = None;
                    for (&id, handle) in state.handles.iter_mut() {
                        if let Handle::Client(client) = handle
                            && let Some(i) = client.held_calls.iter().position(|(_, call)| call.held.op.req_id() == cancel.id)
                        {
                            say(&format!("client {id}: a held wait was cancelled"));
                            found = Some(client.held_calls.remove(i).1.held);
                            break;
                        }
                        if let Handle::Client(client) = handle
                            && client.held.as_ref().is_some_and(|held| held.op.req_id() == cancel.id)
                        {
                            let job = client.waiting.unwrap_or(0);
                            say(&format!("client {id}: read cancelled while waiting for job {job}"));
                            found = client.held.take();
                            break;
                        }
                    }
                    found
                };
                match held {
                    Some(held) => Response::new(Err(Error::new(EINTR)), held.op),
                    None => continue,
                }
            }
            RequestKind::OnClose { id } => {
                scheme.on_close(id);
                continue;
            }
            RequestKind::SendFd(req) => {
                let result = scheme.on_sendfd(&req);
                Response::new(result, req)
            }
            RequestKind::RecvFd(req) => {
                let result = scheme.on_recvfd(&req);
                Response::open_dup_like(result, req)
            }
            _ => continue,
        };
        if let Err(err) = shared.socket.write_response(response, SignalBehavior::Restart) {
            fail(&format!("writing a response: {err}"));
        }
    }
}

fn daemon(daemon: daemon::SchemeDaemon) -> ! {
    common::init();
    let dtb = fs::read("/scheme/kernel.dtb").unwrap_or_default();
    let fdt = fdt::Fdt::new(&dtb).ok();
    let bcm2711 = fdt.as_ref().is_some_and(|f| {
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
    let irq = match fdt.as_ref().and_then(irq_path) {
        Some(path) => match Irq::open(path) {
            Ok(irq) => {
                say(&format!("interrupt: {}", irq.path));
                Some(irq)
            }
            Err(err) => {
                say(&format!("interrupt: {err}: polling"));
                None
            }
        },
        None => {
            say("interrupt: none in the devicetree: polling");
            None
        }
    };
    let v3d = V3d::new(hub, core, pm, asb, irq);
    let socket = Socket::create().expect("v3dd: failed to create the v3d scheme");
    // UIFCFG, HUB_IDENT1..3, CORE0 IDENT0..2 (Linux v3d_get_param_ioctl()).
    let params = [
        v3d.hub.read(0x0004),
        v3d.hub.read(V3D_HUB_IDENT1),
        v3d.hub.read(0x0010),
        v3d.hub.read(0x0014),
        v3d.core.read(0x0000),
        v3d.core.read(0x0004),
        v3d.core.read(0x0008),
    ]
    .map(u64::from);
    let shared = Arc::new(Shared {
        params,
        mmu: v3d.mmu.clone(),
        stats: v3d.stats.clone(),
        v3d: Mutex::new(v3d),
        state: Mutex::new(State {
            handles: HandleMap::new(),
            queue: VecDeque::new(),
            next_job: 0,
            running: None,
            done_through: 0,
            zombies: Vec::new(),
        }),
        work: Condvar::new(),
        socket,
    });
    let mut scheme = V3dScheme { shared: shared.clone() };
    let _ = daemon.ready_sync_scheme(&shared.socket, &mut scheme);
    let jobs = shared.clone();
    thread::Builder::new()
        .name("v3d-jobs".into())
        .spawn(move || jobs.run_jobs())
        .unwrap_or_else(|err| fail(&format!("no job thread: {err}")));
    say("ready: /scheme/v3d");
    serve(shared)
}
