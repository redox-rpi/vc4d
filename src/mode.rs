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

/// Pixel valve 2 (HDMI0) on the BCM2711, if the devicetree has none.
const PV2_BCM2711_BUS_ADDR: usize = 0x7e20_a000;

fn node_addr(fdt: &Fdt, compatible: &str, index: usize) -> Option<usize> {
    let node = fdt.find_compatible(&[compatible])?;
    Some(node.reg()?.nth(index)?.starting_address as usize)
}

/// redox-rpi#671 step 2, read only: pixel valve 2 (HDMI0) and the HDMI0
/// core's timing as vc4d computes them for the EDID's preferred mode, next
/// to the firmware's, and a dump of the HDMI0 core's "hdmi" registers with
/// the words that hold the active width and height. Called only
/// while the firmware's display runs (an HVS channel is enabled), so the
/// HDMI block is clocked.
pub fn compare_bcm2711(fdt: &Fdt, edid: &vc4_edid::Edid) {
    let Some(t) = edid.preferred() else {
        println!("vc4d: mode5: no preferred mode in the EDID");
        return;
    };
    let mode = format!("{}x{}@{}", t.hactive, t.vactive, t.refresh_mhz() / 1000);
    let pv_bus = node_addr(fdt, "brcm,bcm2711-pixelvalve2", 0).unwrap_or(PV2_BCM2711_BUS_ADDR);
    let mut pv_same = false;
    if let Some(regs) = map(Some(fdt), pv_bus, PV_SIZE, common::Prot::RO, "pixel valve 2") {
        let read = |offset: usize| unsafe { (regs as *const u32).add(offset / 4).read_volatile() };
        match vc4_mode::pv_bcm2711(t) {
            Ok(pv) => {
                pv_same = log_regs("pv2(2711)", &pv.regs(), read);
                println!(
                    "vc4d: mode5: {}: pixel valve 2 for {mode} equals the firmware's",
                    if pv_same { "PASS" } else { "DIFFERENT" }
                );
            }
            Err(err) => println!("vc4d: mode5: preferred mode not supported: {err:?}"),
        }
        let words: Vec<String> = (0..0x40).step_by(4).map(|o| format!("{:08x}", read(o))).collect();
        println!("vc4d: mode5: pv2 raw 00-3c: {}", words.join(" "));
    }
    let Some(hdmi_bus) = node_addr(fdt, "brcm,bcm2711-hdmi0", 0) else {
        println!("vc4d: mode5: no brcm,bcm2711-hdmi0 node");
        return;
    };
    let phys = phys(Some(fdt), hdmi_bus);
    let base = phys & !0xfff;
    let Some(page) = map(None, base, 0x1000, common::Prot::RO, "the HDMI0 core") else { return };
    let regs = unsafe { (page as *const u8).add(phys - base) } as *const u32;
    let read = |offset: usize| unsafe { regs.add(offset / 4).read_volatile() };
    if let Ok(h) = vc4_mode::hdmi_timing_bcm2711(t) {
        let same = log_regs("hdmi(2711)", &vc4_mode::hdmi_bcm2711_regs(&h), read);
        println!(
            "vc4d: mode5: {}: HDMI0 timing for {mode} equals the firmware's",
            if same { "PASS" } else { "DIFFERENT" }
        );
        if same && pv_same {
            println!("vc4d: mode5: PASS: pixel valve 2 and HDMI0 timing for {mode} equal the firmware's");
        }
    }
    for row in (0..0x300).step_by(0x20) {
        let words: Vec<String> = (row..row + 0x20).step_by(4).map(|o| format!("{:08x}", read(o))).collect();
        println!("vc4d: mode5: hdmi {row:03x}: {}", words.join(" "));
    }
    let mut hits = Vec::new();
    for offset in (0..0x300).step_by(4) {
        let v = read(offset);
        if v & 0x1fff == t.hactive {
            hits.push(format!("{offset:#05x}={v:#010x}(hactive)"));
        }
        if v & 0x1fff == t.vactive {
            hits.push(format!("{offset:#05x}={v:#010x}(vactive)"));
        }
    }
    println!("vc4d: mode5: hdmi words holding the active size in bits 12:0: {}", hits.join(" "));
}
