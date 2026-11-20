// SPDX-License-Identifier: GPL-2.0-only
//
// V3D step 1 (redox-rpi#595): one compute job through the CSD (compute
// shader dispatch) with memory through the V3D MMU, checked by the CPU.
//
// Translated from Linux v6.12 drivers/gpu/drm/v3d:
// - v3d_mmu.c: page table (one 32-bit entry per 4 KiB page), the MMU
//   registers in v3d_mmu_set_page_table(), the flush in v3d_mmu_flush_all();
// - v3d_gem.c: v3d_init_core(), v3d_invalidate_caches() before a job,
//   v3d_clean_caches() after a CSD job (TMU write combiner, L2T clean);
// - v3d_sched.c: v3d_csd_job_run(), CFG1..6, then CFG0 starts the job.
// The CSD configuration words are what Mesa's v3d driver and py-videocore6
// (GPL-2.0) submit for one workgroup of 16 invocations.
//
// The shader is hand-assembled V3D 4.2 QPU code (field layout from Mesa's
// qpu_pack.c, checked against the words in Mesa's qpu_disasm test). Each of
// the 16 lanes stores lane + CONSTANT at result[lane].

use std::thread;
use std::time::{Duration, Instant};

use common::dma::Dma;

use super::{say, Regs};

// Hub registers (v3d_regs.h)
const V3D_HUB_INT_STS: usize = 0x0050;
const V3D_MMUC_CONTROL: usize = 0x1000;
const V3D_MMUC_CONTROL_FLUSHING: u32 = 1 << 2;
const V3D_MMUC_CONTROL_FLUSH: u32 = 1 << 1;
const V3D_MMUC_CONTROL_ENABLE: u32 = 1 << 0;
const V3D_MMU_CTL: usize = 0x1200;
const V3D_MMU_CTL_CAP_EXCEEDED_ABORT: u32 = 1 << 26;
const V3D_MMU_CTL_CAP_EXCEEDED_INT: u32 = 1 << 25;
const V3D_MMU_CTL_PT_INVALID_ABORT: u32 = 1 << 19;
const V3D_MMU_CTL_PT_INVALID_INT: u32 = 1 << 18;
const V3D_MMU_CTL_PT_INVALID_ENABLE: u32 = 1 << 16;
const V3D_MMU_CTL_WRITE_VIOLATION_ABORT: u32 = 1 << 11;
const V3D_MMU_CTL_WRITE_VIOLATION_INT: u32 = 1 << 10;
const V3D_MMU_CTL_TLB_CLEARING: u32 = 1 << 7;
const V3D_MMU_CTL_TLB_CLEAR: u32 = 1 << 2;
const V3D_MMU_CTL_ENABLE: u32 = 1 << 0;
const V3D_MMU_PT_PA_BASE: usize = 0x1204;
const V3D_MMU_VIO_ID: usize = 0x122c;
const V3D_MMU_ILLEGAL_ADDR: usize = 0x1230;
const V3D_MMU_ILLEGAL_ADDR_ENABLE: u32 = 1 << 31;
const V3D_MMU_VIO_ADDR: usize = 0x1234;
const V3D_MMU_DEBUG_INFO: usize = 0x1238;

// Core registers
const V3D_CTL_SLCACTL: usize = 0x0024;
const V3D_CTL_L2TCACTL: usize = 0x0030;
const V3D_L2TCACTL_TMUWCF: u32 = 1 << 8;
const V3D_L2TCACTL_FLM_FLUSH: u32 = 0 << 1;
const V3D_L2TCACTL_FLM_CLEAN: u32 = 2 << 1;
const V3D_L2TCACTL_L2TFLS: u32 = 1 << 0;
const V3D_CTL_L2TFLSTA: usize = 0x0034;
const V3D_CTL_L2TFLEND: usize = 0x0038;
const V3D_CTL_INT_STS: usize = 0x0050;
const V3D_INT_CSDDONE: u32 = 1 << 7;
const V3D_GMP_STATUS: usize = 0x0800;
const V3D_CSD_STATUS: usize = 0x0900;
const V3D_CSD_QUEUED_CFG0: usize = 0x0904;
const V3D_CSD_CURRENT_CFG0: usize = 0x0920;

const V3D_PTE_WRITEABLE: u32 = 1 << 29;
const V3D_PTE_VALID: u32 = 1 << 28;
const PAGE: usize = 4096;
/// 4 GiB of V3D address space, 4 KiB pages: 1 Mi entries, as Linux.
const PT_ENTRIES: usize = 1 << 20;

// V3D virtual addresses (page 0 stays unmapped, as drm_mm starts at 1 in
// Linux).
const CODE_VA: u32 = 0x1000;
const UNIF_VA: u32 = 0x2000;
const RESULT_VA: u32 = 0x3000;
const CONSTANT: u32 = 0x5a5a_0000;
const LANES: usize = 16;

#[rustfmt::skip]
const SHADER: [u64; 17] = [
    0x3d803186bb800000, // nop; ldunifrf rf0 (uniform 0: result buffer, V3D address)
    0x3d807186bb800000, // nop; ldunifrf rf1 (uniform 1: the constant)
    0x3c003180bb802000, // eidx r0 (lane 0..15)
    0x3c00318138800000, // add r1, r0, r0 (2 * lane)
    0x3c00318138809000, // add r1, r1, r1 (4 * lane)
    0x3c0021803880e000, // add rf0, rf0, r1 (buffer + 4 * lane)
    0x3c00318238830040, // add r2, r0, rf1 (lane + constant)
    0x3c0032c6bbe80000, // mov tmud, r2 (TMU write data)
    0x3c003306bbf80000, // mov tmua, rf0 (TMU write address: the store)
    0x3c203186bb800000, // nop; thrsw (program end: thrsw, thrsw, 2 nops, thrsw, 3 nops)
    0x3c203186bb800000, // nop; thrsw
    0x3c003186bb800000, // nop
    0x3c003186bb800000, // nop
    0x3c203186bb800000, // nop; thrsw
    0x3c003186bb800000, // nop
    0x3c003186bb800000, // nop
    0x3c003186bb800000, // nop
];

pub(crate) fn wait(what: &str, ms: u64, mut done: impl FnMut() -> bool) -> bool {
    let start = Instant::now();
    while !done() {
        if start.elapsed() > Duration::from_millis(ms) {
            say(&format!("csd: timeout: {what} ({ms} ms)"));
            return false;
        }
        thread::yield_now();
    }
    true
}

/// v3d_mmu_flush_all()
pub(crate) fn mmu_flush(hub: &Regs) -> bool {
    wait("TLB clear idle before", 100, || hub.read(V3D_MMU_CTL) & V3D_MMU_CTL_TLB_CLEARING == 0);
    hub.write(V3D_MMU_CTL, hub.read(V3D_MMU_CTL) | V3D_MMU_CTL_TLB_CLEAR);
    hub.write(V3D_MMUC_CONTROL, V3D_MMUC_CONTROL_FLUSH | V3D_MMUC_CONTROL_ENABLE);
    wait("TLB clear", 100, || hub.read(V3D_MMU_CTL) & V3D_MMU_CTL_TLB_CLEARING == 0)
        && wait("MMUC flush", 100, || hub.read(V3D_MMUC_CONTROL) & V3D_MMUC_CONTROL_FLUSHING == 0)
}

pub(crate) fn state(hub: &Regs, core: &Regs) {
    // v3d_irq.c: the faulting address is MMU_VIO_ADDR << (va_width - 32).
    let va_width = 30 + ((hub.read(V3D_MMU_DEBUG_INFO) >> 4) & 0xf);
    let vio = (hub.read(V3D_MMU_VIO_ADDR) as u64) << va_width.saturating_sub(32);
    say(&format!(
        "csd: CSD_STATUS {:#x}, CTL_INT_STS {:#x}, HUB_INT_STS {:#x}, GMP_STATUS {:#x}",
        core.read(V3D_CSD_STATUS),
        core.read(V3D_CTL_INT_STS),
        hub.read(V3D_HUB_INT_STS),
        core.read(V3D_GMP_STATUS)
    ));
    say(&format!(
        "csd: MMU_CTL {:#x}, MMU_VIO_ADDR {:#x} (address {vio:#x}), MMU_VIO_ID {:#x}",
        hub.read(V3D_MMU_CTL),
        hub.read(V3D_MMU_VIO_ADDR),
        hub.read(V3D_MMU_VIO_ID)
    ));
}

/// Runs the job; true when the 16 results are right.
pub fn run(hub: &Regs, core: &Regs) -> bool {
    let debug = hub.read(V3D_MMU_DEBUG_INFO);
    say(&format!(
        "csd: MMU version {}, VA width {}, PA width {} (MMU_DEBUG_INFO {debug:#x})",
        debug & 0xf,
        (debug >> 4) & 0xf,
        (debug >> 8) & 0xf
    ));

    // Uncached, contiguous memory (common::dma maps DMA memory uncached on
    // aarch64): the page table, and four pages for code, uniforms, result
    // and the MMU's scratch page for illegal accesses.
    let mut pt = match Dma::<[u32]>::zeroed_slice(PT_ENTRIES) {
        Ok(dma) => unsafe { dma.assume_init() },
        Err(err) => {
            say(&format!("csd: FAIL: no 4 MiB contiguous memory for the page table: {err:?}"));
            return false;
        }
    };
    let mut mem = match Dma::<[u32]>::zeroed_slice(4 * PAGE / 4) {
        Ok(dma) => unsafe { dma.assume_init() },
        Err(err) => {
            say(&format!("csd: FAIL: no contiguous memory for the job: {err:?}"));
            return false;
        }
    };
    let pt_phys = pt.physical();
    let mem_phys = mem.physical();
    // The V3D on the BCM2711 sees RAM at its CPU physical address (the
    // devicetree's /scb dma-ranges are 1:1), unlike the HVS (0xc0000000 up).
    say(&format!("csd: page table at {pt_phys:#x}, job memory at {mem_phys:#x}"));

    // Pages: 0 code, 1 uniforms, 2 result, 3 scratch.
    for (i, word) in SHADER.iter().enumerate() {
        mem[2 * i] = *word as u32;
        mem[2 * i + 1] = (*word >> 32) as u32;
    }
    let unif = PAGE / 4;
    mem[unif] = RESULT_VA;
    mem[unif + 1] = CONSTANT;
    let result = 2 * PAGE / 4;
    for lane in 0..LANES {
        mem[result + lane] = 0xdead_0000 | lane as u32;
    }
    let scratch_phys = mem_phys + 3 * PAGE;
    for (va, page) in [(CODE_VA, 0), (UNIF_VA, 1), (RESULT_VA, 2)] {
        let phys = mem_phys + page * PAGE;
        pt[va as usize / PAGE] = V3D_PTE_WRITEABLE | V3D_PTE_VALID | (phys / PAGE) as u32;
    }
    say(&format!(
        "csd: PTEs: code {:#x}, uniforms {:#x}, result {:#x}",
        pt[CODE_VA as usize / PAGE],
        pt[UNIF_VA as usize / PAGE],
        pt[RESULT_VA as usize / PAGE]
    ));

    // v3d_init_core(): flush all of L2T on every flush.
    core.write(V3D_CTL_L2TFLSTA, 0);
    core.write(V3D_CTL_L2TFLEND, !0);

    // v3d_mmu_set_page_table()
    hub.write(V3D_MMU_PT_PA_BASE, (pt_phys / PAGE) as u32);
    hub.write(
        V3D_MMU_CTL,
        V3D_MMU_CTL_ENABLE
            | V3D_MMU_CTL_PT_INVALID_ENABLE
            | V3D_MMU_CTL_PT_INVALID_ABORT
            | V3D_MMU_CTL_PT_INVALID_INT
            | V3D_MMU_CTL_WRITE_VIOLATION_ABORT
            | V3D_MMU_CTL_WRITE_VIOLATION_INT
            | V3D_MMU_CTL_CAP_EXCEEDED_ABORT
            | V3D_MMU_CTL_CAP_EXCEEDED_INT,
    );
    hub.write(V3D_MMU_ILLEGAL_ADDR, (scratch_phys / PAGE) as u32 | V3D_MMU_ILLEGAL_ADDR_ENABLE);
    hub.write(V3D_MMUC_CONTROL, V3D_MMUC_CONTROL_ENABLE);
    if !mmu_flush(hub) {
        say("csd: FAIL: MMU flush did not finish");
        state(hub, core);
        return false;
    }
    say(&format!("csd: MMU on: MMU_CTL {:#x}, PT_PA_BASE {:#x}", hub.read(V3D_MMU_CTL), hub.read(V3D_MMU_PT_PA_BASE)));
    state(hub, core);

    // v3d_invalidate_caches(): L2T flush, slice caches (V3D 4.2 has no L3
    // flush and no L2C).
    core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_L2TFLS | V3D_L2TCACTL_FLM_FLUSH);
    core.write(V3D_CTL_SLCACTL, 0x0f0f_0f0f);

    // One workgroup (1, 1, 1) of 16 invocations, 16 workgroups per
    // supergroup, one batch: as py-videocore6's dispatch() with its
    // defaults. CFG5: shader address, no NaN propagation, not single
    // segment, no threading. CFG6: uniforms.
    let wg_size: u32 = 16;
    let wgs_per_sg: u32 = 16;
    let cfg = [
        1 << 16,
        1 << 16,
        1 << 16,
        ((wgs_per_sg * wg_size).div_ceil(16) - 1) << 12 | wgs_per_sg << 8 | (wg_size & 0xff),
        0,
        CODE_VA,
        UNIF_VA,
    ];
    let completed_before = (core.read(V3D_CSD_STATUS) >> 4) & 0xff;
    say(&format!("csd: dispatch, cfg {cfg:#x?}"));
    for i in 1..=6 {
        core.write(V3D_CSD_QUEUED_CFG0 + 4 * i, cfg[i]);
    }
    let start = Instant::now();
    core.write(V3D_CSD_QUEUED_CFG0, cfg[0]);
    let finished = wait("job", 1000, || {
        let s = core.read(V3D_CSD_STATUS);
        (s >> 4) & 0xff != completed_before && s & 0x3 == 0
    });
    let took = start.elapsed();
    state(hub, core);
    if !finished {
        say(&format!(
            "csd: current dispatch CFG0..6 {:#x?}",
            (0..7).map(|i| core.read(V3D_CSD_CURRENT_CFG0 + 4 * i)).collect::<Vec<_>>()
        ));
        say("csd: FAIL: the job did not finish");
        return false;
    }
    say(&format!(
        "csd: job done after {} us, CSD done interrupt bit {}",
        took.as_micros(),
        core.read(V3D_CTL_INT_STS) & V3D_INT_CSDDONE != 0
    ));

    // v3d_clean_caches(): TMU write combiner, then L2T clean.
    core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_TMUWCF);
    wait("TMU write combiner flush", 100, || core.read(V3D_CTL_L2TCACTL) & V3D_L2TCACTL_TMUWCF == 0);
    core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_L2TFLS | V3D_L2TCACTL_FLM_CLEAN);
    wait("L2T clean", 100, || core.read(V3D_CTL_L2TCACTL) & V3D_L2TCACTL_L2TFLS == 0);

    let got: Vec<u32> = (0..LANES).map(|lane| mem[result + lane]).collect();
    say(&format!("csd: result {got:#x?}"));
    let wrong: Vec<usize> = (0..LANES).filter(|&lane| got[lane] != CONSTANT + lane as u32).collect();
    let scratch: Vec<u32> = (0..8).map(|i| mem[3 * PAGE / 4 + i]).collect();
    if scratch.iter().any(|&w| w != 0) {
        say(&format!("csd: the MMU's scratch page was written: {scratch:#x?}"));
    }
    if wrong.is_empty() {
        say(&format!("csd: PASS: 16 lanes wrote lane + {CONSTANT:#x} through the MMU"));
        true
    } else {
        say(&format!("csd: FAIL: {} of 16 lanes wrong: {wrong:?}", wrong.len()));
        false
    }
}
