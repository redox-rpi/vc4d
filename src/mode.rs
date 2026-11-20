// SPDX-License-Identifier: GPL-2.0-only
//
// Stages 2c, 3a and 3b of redox-rpi#188: the mode registers vc4d computes for
// the monitor's preferred mode (crate vc4-mode), next to what the firmware
// wrote, and then, with VC4D_MODE=write, written by vc4d itself.
//
// 2c, read only: pixel valve 2 and, once the HSM clock is seen running, the
// HDMI core's timing, each register as "ours" and "firmware".
// 3a, VC4D_MODE=write: when pixel valve 2 agrees, vc4d programs it the way
// Linux's vc4_crtc does (video off, valve off, FIFO emptied, timing, valve
// and video on) and reads it back. Same mode, so the monitor keeps its
// mode; the screen is blank for a few frames.
// 3b: when the HDMI timing agrees too (and the HSM clock runs), vc4d also
// writes the HDMI core's timing registers while the valve is off, as
// Linux's vc4_hdmi_set_timings() does before the valve is configured.

use std::thread;
use std::time::Duration;

use fdt::Fdt;
use vc4_mode::{HdmiTiming, Pv, Step};

/// What vc4d computed and found equal to the firmware's.
pub struct Same {
    pub pv: Pv,
    /// The HDMI timing, when the HSM clock runs and it agrees.
    pub hdmi: Option<HdmiTiming>,
    pub timing: vc4_edid::Timing,
}

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
pub fn compare(fdt: Option<&Fdt>, edid: &vc4_edid::Edid) -> Option<Same> {
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
    let hdmi = compare_hdmi(fdt, t, &mode);
    pv_same.then_some(Same { pv, hdmi, timing: *t })
}

/// The HDMI core's timing, read only once its clock is seen running:
/// reading the block without it can hang the bus.
fn compare_hdmi(fdt: Option<&Fdt>, t: &vc4_edid::Timing, mode: &str) -> Option<HdmiTiming> {
    let hdmi = vc4_mode::hdmi_timing(t).ok()?;
    let cm = map(fdt, CM_BUS_ADDR, CM_SIZE, common::Prot::RO, "the clock manager")?;
    let hsm = unsafe { (cm as *const u32).add(CM_HSMCTL / 4).read_volatile() };
    if hsm & (CM_BUSY | CM_ENAB) != (CM_BUSY | CM_ENAB) {
        println!("vc4d: mode: HSM clock not running (CM_HSMCTL {hsm:#010x}), HDMI core not read");
        return None;
    }
    let regs = map(fdt, HDMI_BUS_ADDR, HDMI_SIZE, common::Prot::RO, "the HDMI core")? as *const u32;
    let read = |offset: usize| unsafe { regs.add(offset / 4).read_volatile() };
    if log_regs("hdmi", &hdmi.regs(), read) {
        println!("vc4d: mode: PASS: HDMI timing for {mode} equals the firmware's (CM_HSMCTL {hsm:#010x})");
        Some(hdmi)
    } else {
        let names: Vec<&str> = vc4_mode::differences(&hdmi.regs(), read).iter().map(|d| d.0).collect();
        println!("vc4d: mode: DIFFERENT: HDMI timing for {mode}: {}", names.join(" "));
        None
    }
}

/// Stages 3a and 3b: programs pixel valve 2 (and, with `same.hdmi`, the
/// HDMI core's timing) and reads them back. Returns whether every register
/// reads back as written.
pub fn write_mode(fdt: Option<&Fdt>, same: &Same) -> bool {
    let pv = &same.pv;
    let Some(regs) = map(fdt, PV2_BUS_ADDR, PV_SIZE, common::Prot::RW, "pixel valve 2") else { return false };
    let hdmi = match same.hdmi {
        Some(h) => match map(fdt, HDMI_BUS_ADDR, HDMI_SIZE, common::Prot::RW, "the HDMI core") {
            Some(hregs) => Some((h, hregs)),
            None => return false,
        },
        None => None,
    };
    let frame_ms = 1_000_000 / u64::from(same.timing.refresh_mhz().max(1000)) + 1;
    println!(
        "vc4d: mode: write: programming pixel valve 2{} (screen blank for a moment)",
        if hdmi.is_some() { " and the HDMI timing" } else { "" }
    );
    for step in vc4_mode::mode_program(pv, hdmi.as_ref().map(|(h, _)| h)) {
        match step {
            Step::Write(offset, _, value) => unsafe { regs.add(offset / 4).write_volatile(value) },
            Step::Hdmi(offset, _, value) => {
                if let Some((_, hregs)) = hdmi {
                    unsafe { hregs.add(offset / 4).write_volatile(value) }
                }
            }
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
    if let Some((h, hregs)) = hdmi {
        for (offset, name, value) in h.regs() {
            let got = unsafe { hregs.add(offset / 4).read_volatile() };
            ok &= got == value;
            println!(
                "vc4d: mode: write: hdmi {name:<9} wrote {value:#010x} reads {got:#010x}{}",
                if got == value { "" } else { "  DIFFERENT" }
            );
        }
    } else {
        println!("vc4d: mode: write: HDMI timing not written (not compared equal, or no HSM clock)");
    }
    let what = if hdmi.is_some() { "pixel valve 2 and the HDMI timing" } else { "pixel valve 2" };
    if ok {
        println!("vc4d: mode: write: PASS: {what} read back as written");
    } else {
        println!("vc4d: mode: write: FAILED: {what} read back differently");
    }
    ok
}


/// CPRMAN: the PLLH channel divider for the pixel clock (Linux
/// clk-bcm2835.c, pllh_pix) and its load bit.
const CPRMAN_SIZE: usize = 0x2000;
const A2W_PLLH_PIX: usize = 0x1560;
const CM_PLLH: usize = 0x110;
const CM_PLLH_LOADPIX: u32 = 1 << 0;
const CM_PASSWORD: u32 = 0x5a00_0000;

/// Switches from the firmware's mode (`from`, equal to vc4d's values) to
/// `target`, whose pixel clock is the running one divided by `divisor`: the
/// valve off, the PLLH_PIX divider multiplied, the HDMI and valve timing,
/// the valve on. The monitor sees a mode change.
pub fn switch(fdt: Option<&Fdt>, from: &Same, target: &vc4_edid::Timing, divisor: u32) -> bool {
    let name = format!("{}x{}@{}", target.hactive, target.vactive, target.refresh_mhz() / 1000);
    let (Ok(pv), Ok(hdmi)) = (vc4_mode::pv2(target), vc4_mode::hdmi_timing(target)) else {
        println!("vc4d: mode: switch: {name} not supported");
        return false;
    };
    if from.hdmi.is_none() {
        println!("vc4d: mode: switch: HDMI timing not known equal to the firmware's, not switching");
        return false;
    }
    let Some(cm) = map(fdt, CM_BUS_ADDR, CPRMAN_SIZE, common::Prot::RW, "the clock manager") else { return false };
    let Some(regs) = map(fdt, PV2_BUS_ADDR, PV_SIZE, common::Prot::RW, "pixel valve 2") else { return false };
    let Some(hregs) = map(fdt, HDMI_BUS_ADDR, HDMI_SIZE, common::Prot::RW, "the HDMI core") else { return false };
    let cm_read = |offset: usize| unsafe { cm.add(offset / 4).read_volatile() };
    let cm_write = |offset: usize, value: u32| unsafe { cm.add(offset / 4).write_volatile(CM_PASSWORD | value) };
    let div = cm_read(A2W_PLLH_PIX) & 0xff;
    let new_div = div * divisor;
    if div == 0 || new_div > 255 {
        println!("vc4d: mode: switch: PLLH_PIX divider {div} cannot be multiplied by {divisor}");
        return false;
    }
    println!("vc4d: mode: switch: to {name}, PLLH_PIX divider {div} -> {new_div} (screen blank, monitor resyncs)");
    let frame_ms = 1_000_000 / u64::from(from.timing.refresh_mhz().max(1000)) + 1;
    for (i, step) in vc4_mode::mode_program(&pv, Some(&hdmi)).into_iter().enumerate() {
        match step {
            Step::Write(offset, _, value) => unsafe { regs.add(offset / 4).write_volatile(value) },
            Step::Hdmi(offset, _, value) => unsafe { hregs.add(offset / 4).write_volatile(value) },
            Step::WaitFrames(n) => thread::sleep(Duration::from_millis(frame_ms * u64::from(n))),
        }
        // Steps 0-3 stop the valve and empty its FIFO: change the clock
        // while it is off, as Linux sets the pixel clock before the valve.
        if i == 3 {
            cm_write(A2W_PLLH_PIX, new_div);
            let pllh = cm_read(CM_PLLH) & 0x00ff_ffff;
            cm_write(CM_PLLH, pllh | CM_PLLH_LOADPIX);
            cm_write(CM_PLLH, pllh & !CM_PLLH_LOADPIX);
        }
    }
    let got = cm_read(A2W_PLLH_PIX) & 0xff;
    let read = |offset: usize| unsafe { regs.add(offset / 4).read_volatile() };
    let mut ok = got == new_div;
    for (offset, name, value) in pv.regs() {
        ok &= read(offset) == value & vc4_mode::readback_mask(offset);
    }
    println!("vc4d: mode: switch: {} (divider reads {got})", if ok { "PASS" } else { "FAIL: registers read back different" });
    ok
}
