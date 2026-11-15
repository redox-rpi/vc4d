// SPDX-License-Identifier: GPL-2.0-only
//
// Stages 2c and 3a of redox-rpi#188: the mode registers vc4d computes for
// the monitor's preferred mode (crate vc4-mode), next to what the firmware
// wrote, and then, with VC4D_MODE=write, written by vc4d itself.
//
// 2c, read only: pixel valve 2 and, once the HSM clock is seen running, the
// HDMI core's timing, each register as "ours" and "firmware".
// 3a, VC4D_MODE=write: when pixel valve 2 agrees, vc4d programs it the way
// Linux's vc4_crtc does (video off, valve off, FIFO emptied, timing, valve
// and video on) and reads it back. Same mode, so the monitor keeps its
// mode; the screen is blank for a few frames. The HDMI core is not
// written yet.

use std::thread;
use std::time::Duration;

use fdt::Fdt;
use vc4_mode::{Pv, Step};

/// Bus address of pixel valve 2 (HDMI) on the BCM2835/6/7.
const PV2_BUS_ADDR: usize = 0x7e80_7000;
const PV_SIZE: usize = 0x1000;
/// The HDMI core and the clock manager (BCM2835 offsets).
const HDMI_BUS_ADDR: usize = 0x7e90_2000;
const HDMI_SIZE: usize = 0x1000;
const CM_BUS_ADDR: usize = 0x7e10_1000;
const CM_SIZE: usize = 0x1000;
/// CM_HSMCTL, the HDMI state machine clock (Linux clk-bcm2835.c): BUSY
/// while the clock runs, ENAB when it is switched on.
const CM_HSMCTL: usize = 0x088;
const CM_BUSY: u32 = 1 << 7;
const CM_ENAB: u32 = 1 << 4;

fn phys(fdt: Option<&Fdt>, bus: usize) -> usize {
    fdt.map_or(bus, |fdt| crate::soc_phys(fdt, bus))
}

fn map(fdt: Option<&Fdt>, bus: usize, size: usize, prot: common::Prot, what: &str) -> Option<*mut u32> {
    let phys = phys(fdt, bus);
    match unsafe { common::physmap(phys, size, prot, common::MemoryType::DeviceMemory) } {
        Ok(regs) => Some(regs as *mut u32),
        Err(err) => {
            println!("vc4d: mode: failed to map {what} at {phys:#x}: {err:?}");
            None
        }
    }
}

fn log_regs(block: &str, regs: &[(usize, &'static str, u32)], read: impl Fn(usize) -> u32) -> bool {
    let mut same = true;
    for &(offset, name, value) in regs {
        let firmware = read(offset);
        same &= value == firmware;
        println!(
            "vc4d: mode: {block} {name:<9} ours {value:#010x} firmware {firmware:#010x}{}",
            if value == firmware { "" } else { "  DIFFERENT" }
        );
    }
    same
}

/// Compares; returns vc4d's pixel valve 2 values when they equal the
/// firmware's.
pub fn compare(fdt: Option<&Fdt>, edid: &vc4_edid::Edid) -> Option<(Pv, vc4_edid::Timing)> {
    let Some(t) = edid.preferred() else {
        println!("vc4d: mode: no preferred mode in the EDID");
        return None;
    };
    let pv = match vc4_mode::pv2(t) {
        Ok(pv) => pv,
        Err(err) => {
            println!("vc4d: mode: preferred mode not supported: {err:?}");
            return None;
        }
    };
    let mode = format!("{}x{}@{}", t.hactive, t.vactive, t.refresh_mhz() / 1000);
    let regs = map(fdt, PV2_BUS_ADDR, PV_SIZE, common::Prot::RO, "pixel valve 2")? as *const u32;
    let read = |offset: usize| unsafe { regs.add(offset / 4).read_volatile() };
    let pv_same = log_regs("pv2", &pv.regs(), read);
    if pv_same {
        println!("vc4d: mode: PASS: pixel valve 2 values for {mode} equal the firmware's");
    } else {
        let names: Vec<&str> = vc4_mode::differences(&pv.regs(), read).iter().map(|d| d.0).collect();
        println!("vc4d: mode: DIFFERENT: pixel valve 2 for {mode}: {} (the firmware may have picked another mode)", names.join(" "));
    }
    compare_hdmi(fdt, t, &mode);
    pv_same.then_some((pv, *t))
}

/// The HDMI core's timing, read only once its clock is seen running:
/// reading the block without it can hang the bus.
fn compare_hdmi(fdt: Option<&Fdt>, t: &vc4_edid::Timing, mode: &str) {
    let Ok(hdmi) = vc4_mode::hdmi_timing(t) else { return };
    let Some(cm) = map(fdt, CM_BUS_ADDR, CM_SIZE, common::Prot::RO, "the clock manager") else { return };
    let hsm = unsafe { (cm as *const u32).add(CM_HSMCTL / 4).read_volatile() };
    if hsm & (CM_BUSY | CM_ENAB) != (CM_BUSY | CM_ENAB) {
        println!("vc4d: mode: HSM clock not running (CM_HSMCTL {hsm:#010x}), HDMI core not read");
        return;
    }
    let Some(regs) = map(fdt, HDMI_BUS_ADDR, HDMI_SIZE, common::Prot::RO, "the HDMI core") else { return };
    let regs = regs as *const u32;
    let read = |offset: usize| unsafe { regs.add(offset / 4).read_volatile() };
    if log_regs("hdmi", &hdmi.regs(), read) {
        println!("vc4d: mode: PASS: HDMI timing for {mode} equals the firmware's (CM_HSMCTL {hsm:#010x})");
    } else {
        let names: Vec<&str> = vc4_mode::differences(&hdmi.regs(), read).iter().map(|d| d.0).collect();
        println!("vc4d: mode: DIFFERENT: HDMI timing for {mode}: {}", names.join(" "));
    }
}

/// Stage 3a: programs pixel valve 2 with `pv` and reads it back. Returns
/// whether every register reads back as written.
pub fn write_pv2(fdt: Option<&Fdt>, pv: &Pv, t: &vc4_edid::Timing) -> bool {
    let Some(regs) = map(fdt, PV2_BUS_ADDR, PV_SIZE, common::Prot::RW, "pixel valve 2") else { return false };
    let frame_ms = 1_000_000 / u64::from(t.refresh_mhz().max(1000)) + 1;
    println!("vc4d: mode: write: programming pixel valve 2 (screen blank for a moment)");
    for step in vc4_mode::pv_program(pv) {
        match step {
            Step::Write(offset, _, value) => unsafe { regs.add(offset / 4).write_volatile(value) },
            Step::WaitFrames(n) => thread::sleep(Duration::from_millis(frame_ms * u64::from(n))),
        }
    }
    let read = |offset: usize| unsafe { regs.add(offset / 4).read_volatile() };
    let mut ok = true;
    for (offset, name, value) in pv.regs() {
        let got = read(offset);
        let want = value & vc4_mode::readback_mask(offset);
        ok &= got == want;
        println!(
            "vc4d: mode: write: pv2 {name:<9} wrote {value:#010x} reads {got:#010x}{}",
            if got == want { "" } else { "  DIFFERENT" }
        );
    }
    println!("vc4d: mode: write: {}", if ok { "PASS: pixel valve 2 reads back as written" } else { "FAILED: pixel valve 2 reads back differently" });
    ok
}
