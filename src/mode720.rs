// SPDX-License-Identifier: GPL-2.0-only
//
// redox-rpi#671 step 4: a real mode change on the BCM2711 (Pi 4, Pi 400),
// from the firmware's mode to 1280x720 at 60 Hz (CEA-861 VIC 4), with the
// way back built in. Opt-in: VC4D_MODE=720p or /etc/vc4d-mode-720p.
//
// The firmware keeps the encoder set up (colour space converter, packet
// scheduler, video enable); vc4d changes what depends on the mode, in the
// order of Linux's vc4_crtc.c and vc4_hdmi.c:
//   pixel valve 2 off and its FIFO emptied,
//   HVS channel size and a display list for the new size,
//   HDMI PHY for the new TMDS rate (vc5_hdmi_phy_init(), mode crate),
//   HDMI0 timing, pixel valve 2 timing, valve and video on,
//   HDMI FIFO recentred (vc4_hdmi_recenter_fifo()).
// The firmware's HDMI state machine and BVB clocks are read through the
// firmware (firmware.rs) and raised only if they are too slow for the new
// rate; at 720p the firmware's 1080p rates are enough.
//
// No person needed to see a black screen: after the change vc4d checks
// that every register reads back as written, that the HDMI encoder's
// frame counter and the HVS channel count the new mode's frames per
// second (the frame rate follows the PHY's pixel clock and the timing:
// 74.25 MHz / (1650 x 750) = 60 Hz), that the HVS reports no underrun, that
// the packet scheduler is active and the FIFO recentres, and that the
// monitor still answers on DDC (EDID, and SCDC status on HDMI 2.0
// monitors). Any check failing restores the firmware's values at once.
// Otherwise vc4d keeps 720p for a few seconds and then restores the
// firmware's mode and checks it again ("restore N" in the file, default
// 15), unless the file says "keep": then the desktop starts at 720p.

use std::thread;
use std::time::{Duration, Instant};

use fdt::Fdt;
use vc4_edid::Timing;
use vc4_mode::{Pv, Region, Step};

use crate::firmware::{Firmware, CLOCK_BVB, CLOCK_HDMI};
use crate::hvs::{self, Hvs, Plane};
use crate::mode::Same;

/// CEA-861 VIC 4: 1280x720 at 60 Hz, sync pulses positive.
pub const P720: Timing = Timing {
    pixel_khz: 74_250,
    hactive: 1280,
    hfp: 110,
    hsync: 40,
    hbp: 220,
    vactive: 720,
    vfp: 5,
    vsync: 5,
    vbp: 20,
    hsync_positive: true,
    vsync_positive: true,
    interlaced: false,
};

/// Where the display list for the switch goes: after vc4d's own three
/// slots (adapter.rs), before the filter kernel.
const SWITCH_LIST: usize = 0x980;
const PV_SIZE: usize = 0x100;

// HDMI0 "hdmi" region.
const HDMI_FIFO_CTL: usize = 0x074;
const HDMI_SCHEDULER_CONTROL: usize = 0x0e0;
const HDMI_MISC_CONTROL: usize = 0x100;
const FIFO_CTL_RECENTER: u32 = 1 << 6;
const FIFO_CTL_RECENTER_DONE: u32 = 1 << 14;
const FIFO_VALID_WRITE_MASK: u32 = 0xefff;
const SCHEDULER_MANUAL_FORMAT: u32 = 1 << 15;
const SCHEDULER_IGNORE_VSYNC_PREDICTS: u32 = 1 << 5;
const SCHEDULER_HDMI_ACTIVE: u32 = 1 << 1;
// HDMI0 in the shared "hd" region.
const HD_VID_CTL: usize = 0x044;
const HD_FRAME_COUNT: usize = 0x060;
/// SCDC of HDMI 2.0 monitors: sink version (0x01), status flags 0 (0x40:
/// clock detected, channels 0-2 locked).
const SCDC_ADDR: u8 = 0x54;
const SCDC_SINK_VERSION: u8 = 0x01;
const SCDC_STATUS_FLAGS_0: u8 = 0x40;

/// How the 720p switch ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum After {
    /// Back to the firmware's mode after this many seconds.
    Restore(u64),
    /// Stay at 720p; the desktop starts at 720p.
    Keep,
}

/// The opt-in: VC4D_MODE=720p or /etc/vc4d-mode-720p ("keep" or
/// "restore N").
pub fn wanted() -> Option<After> {
    let file = std::fs::read_to_string("/etc/vc4d-mode-720p").ok();
    let env = std::env::var("VC4D_MODE").is_ok_and(|v| v == "720p");
    if file.is_none() && !env {
        return None;
    }
    let text = file.unwrap_or_default();
    let mut words = text.split_whitespace();
    Some(match (words.next(), words.next().and_then(|n| n.parse().ok())) {
        (Some("keep"), _) => After::Keep,
        (Some("restore"), Some(n)) => After::Restore(n),
        _ => After::Restore(15),
    })
}

fn map(phys: usize, size: usize, what: &str) -> Option<*mut u32> {
    let base = phys & !0xfff;
    let len = (phys - base + size + 0xfff) & !0xfff;
    match unsafe { common::physmap(base, len, common::Prot::RW, common::MemoryType::DeviceMemory) } {
        Ok(regs) => Some(unsafe { (regs as *mut u8).add(phys - base) } as *mut u32),
        Err(err) => {
            println!("vc4d: mode5: 720p: failed to map {what} at {phys:#x}: {err:?}");
            None
        }
    }
}

/// HDMI0's region `name` (its reg-names entry), mapped.
fn hdmi0_region(fdt: &Fdt, name: &str) -> Option<*mut u32> {
    let node = fdt.find_compatible(&["brcm,bcm2711-hdmi0"])?;
    let names = node.property("reg-names")?.value;
    let index = names.split(|&b| b == 0).position(|n| n == name.as_bytes())?;
    let region = node.reg()?.nth(index)?;
    let phys = crate::soc_phys(fdt, region.starting_address as usize);
    map(phys, region.size.unwrap_or(0x100).max(4), name)
}

#[derive(Clone, Copy)]
struct Regs(*mut u32);

impl Regs {
    fn rd(self, offset: usize) -> u32 {
        unsafe { self.0.add(offset / 4).read_volatile() }
    }
    fn wr(self, offset: usize, value: u32) {
        unsafe { self.0.add(offset / 4).write_volatile(value) }
    }
}

struct Blocks {
    pv: Regs,
    hdmi: Regs,
    phy: Regs,
    rm: Regs,
    hd: Regs,
}

impl Blocks {
    fn open(fdt: &Fdt) -> Option<Blocks> {
        let pv_bus = fdt
            .find_compatible(&["brcm,bcm2711-pixelvalve2"])
            .and_then(|n| n.reg()?.next())
            .map(|r| r.starting_address as usize)
            .unwrap_or(0x7e20_a000);
        Some(Blocks {
            pv: Regs(map(crate::soc_phys(fdt, pv_bus), PV_SIZE, "pixel valve 2")?),
            hdmi: Regs(hdmi0_region(fdt, "hdmi")?),
            phy: Regs(hdmi0_region(fdt, "phy")?),
            rm: Regs(hdmi0_region(fdt, "rm")?),
            hd: Regs(hdmi0_region(fdt, "hd")?),
        })
    }

    fn region(&self, region: Region) -> Regs {
        match region {
            Region::Phy => self.phy,
            Region::Rm => self.rm,
        }
    }
}

/// Everything the switch changes, as the firmware left it.
struct Saved {
    pv: Pv,
    hdmi: [(usize, &'static str, u32); 6],
    phy: Vec<(Region, usize, &'static str, u32)>,
    hvs_ctrl: u32,
    hvs_list: usize,
}

fn save(b: &Blocks, hvs: &Hvs, ch: usize) -> Saved {
    let pv = Pv {
        control: b.pv.rd(0x00),
        v_control: b.pv.rd(0x04),
        horza: b.pv.rd(0x0c),
        horzb: b.pv.rd(0x10),
        verta: b.pv.rd(0x14),
        vertb: b.pv.rd(0x18),
    };
    let mut hdmi = [(0, "", 0); 6];
    for (i, &(offset, name)) in
        [(0x0e4, "HORZA"), (0x0e8, "HORZB"), (0x0ec, "VERTA0"), (0x0f0, "VERTB0"), (0x0f4, "VERTA1"), (0x0f8, "VERTB1")]
            .iter()
            .enumerate()
    {
        hdmi[i] = (offset, name, b.hdmi.rd(offset));
    }
    let phy = vc4_mode::PHY5_REGS.iter().map(|&(r, o, n)| (r, o, n, b.region(r).rd(o))).collect();
    Saved { pv, hdmi, phy, hvs_ctrl: hvs.channel_ctrl(ch), hvs_list: hvs.list_start(ch) }
}

/// The target of one change: a computed mode, or the saved firmware state.
enum Target<'a> {
    Mode { t: &'a Timing, pv: Pv, hdmi: [(usize, &'static str, u32); 6], phy: vc4_mode::Phy5 },
    Firmware(&'a Saved),
}

fn frame_ms(t: &Timing) -> u64 {
    1_000_000 / u64::from(t.refresh_mhz().max(1000)) + 1
}

/// Runs the change: valve off, then (where the HDMI timing would go) the
/// HVS, the PHY and the HDMI timing, then the valve's timing and on.
fn apply(b: &Blocks, hvs: &Hvs, ch: usize, target: &Target, now: &Timing) {
    let (pv, hdmi) = match target {
        Target::Mode { pv, hdmi, .. } => (*pv, *hdmi),
        Target::Firmware(s) => (s.pv, s.hdmi),
    };
    let wait = |t: &Timing, n: u32| thread::sleep(Duration::from_millis(frame_ms(t) * u64::from(n)));
    let steps = vc4_mode::mode_program_regs(&pv, Some(&hdmi));
    let split = steps.iter().position(|s| matches!(s, Step::Hdmi(..))).unwrap_or(steps.len());
    let new_timing = match target {
        Target::Mode { t, .. } => **t,
        Target::Firmware(_) => *now,
    };
    let run = |step: &Step, t: &Timing| match *step {
        Step::Write(offset, _, value) => b.pv.wr(offset, value),
        Step::Hdmi(offset, _, value) => b.hdmi.wr(offset, value),
        Step::WaitFrames(n) => wait(t, n),
    };
    for step in &steps[..split] {
        run(step, now);
    }
    // The valve is off: nothing pulls from the HVS channel or feeds the
    // encoder.
    match target {
        Target::Mode { t, phy, .. } => {
            hvs.set_list_start(ch, SWITCH_LIST);
            hvs.set_channel_size(ch, t.hactive, t.vactive);
            for step in vc4_mode::phy5_program(phy) {
                let regs = b.region(step.region);
                regs.wr(step.offset, step.apply(regs.rd(step.offset)));
            }
        }
        Target::Firmware(s) => {
            hvs.set_list_start(ch, s.hvs_list);
            hvs.set_channel_ctrl(ch, s.hvs_ctrl);
            // The firmware's PHY values, the PLL held in reset until the
            // rest is written.
            b.phy.wr(vc4_mode::PHY5_RESET_CTL, 0x0f);
            for &(r, o, _, v) in &s.phy {
                if (r, o) != (Region::Phy, vc4_mode::PHY5_RESET_CTL) {
                    b.region(r).wr(o, v);
                }
            }
            let reset = s.phy.iter().find(|p| p.0 == Region::Phy && p.1 == vc4_mode::PHY5_RESET_CTL).map_or(0x30, |p| p.3);
            b.phy.wr(vc4_mode::PHY5_RESET_CTL, reset & !0x30);
            b.phy.wr(vc4_mode::PHY5_RESET_CTL, reset);
        }
    }
    let sched = b.hdmi.rd(HDMI_SCHEDULER_CONTROL);
    b.hdmi.wr(HDMI_SCHEDULER_CONTROL, sched | SCHEDULER_MANUAL_FORMAT | SCHEDULER_IGNORE_VSYNC_PREDICTS);
    for step in &steps[split..] {
        run(step, &new_timing);
    }
    recenter(b);
}

/// vc4_hdmi_recenter_fifo(); true when the encoder reports it done.
fn recenter(b: &Blocks) -> bool {
    let drift = b.hdmi.rd(HDMI_FIFO_CTL) & FIFO_VALID_WRITE_MASK;
    for _ in 0..2 {
        b.hdmi.wr(HDMI_FIFO_CTL, drift & !FIFO_CTL_RECENTER);
        b.hdmi.wr(HDMI_FIFO_CTL, drift | FIFO_CTL_RECENTER);
        thread::sleep(Duration::from_millis(1));
    }
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(20) {
        if b.hdmi.rd(HDMI_FIFO_CTL) & FIFO_CTL_RECENTER_DONE != 0 {
            return true;
        }
        thread::sleep(Duration::from_millis(1));
    }
    false
}

/// Frames per second over one second: the encoder's frame counter and the
/// HVS channel's (6 bits, sampled often enough not to wrap).
fn measure(b: &Blocks, hvs: &Hvs, ch: usize) -> (u32, u32) {
    let hd0 = b.hd.rd(HD_FRAME_COUNT);
    let mut last = hvs.frame_count(ch);
    let mut frames = 0;
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(1) {
        thread::sleep(Duration::from_millis(5));
        let now = hvs.frame_count(ch);
        frames += now.wrapping_sub(last) & 0x3f;
        last = now;
    }
    (b.hd.rd(HD_FRAME_COUNT).wrapping_sub(hd0), frames)
}

/// The checks without a person; true when all pass.
fn verify(b: &Blocks, hvs: &Hvs, ch: usize, target: &Target, t: &Timing, hd_counts: bool, fdt: &Fdt, what: &str) -> bool {
    let mut ok = true;
    let mut fail = |why: String| {
        println!("vc4d: mode5: {what}: check FAILED: {why}");
        ok = false;
    };
    // 1. Read-back.
    let (pv, hdmi) = match target {
        Target::Mode { pv, hdmi, .. } => (*pv, *hdmi),
        Target::Firmware(s) => (s.pv, s.hdmi),
    };
    for (offset, name, value) in pv.regs() {
        let got = b.pv.rd(offset);
        if got != value & vc4_mode::readback_mask(offset) {
            fail(format!("pv2 {name} wrote {value:#010x} reads {got:#010x}"));
        }
    }
    for (offset, name, value) in hdmi {
        let got = b.hdmi.rd(offset);
        if got != value {
            fail(format!("hdmi {name} wrote {value:#010x} reads {got:#010x}"));
        }
    }
    let size = hvs.channel(ch).map(|c| (c.width, c.height));
    if size != Some((t.hactive, t.vactive)) {
        fail(format!("HVS channel {ch} is {size:?}, not {}x{}", t.hactive, t.vactive));
    }
    let phy_now: Vec<String> =
        vc4_mode::PHY5_REGS.iter().map(|&(r, o, n)| format!("{n}={:08x}", b.region(r).rd(o))).collect();
    println!("vc4d: mode5: {what}: phy {}", phy_now.join(" "));
    // 2. Frames per second: they follow the pixel clock and the timing.
    thread::sleep(Duration::from_millis(300));
    hvs.clear_errors(ch);
    let (hd_fps, hvs_fps) = measure(b, hvs, ch);
    let want = t.refresh_mhz().div_ceil(1000);
    let near = |fps: u32| fps + 2 >= want && fps <= want + 2;
    println!(
        "vc4d: mode5: {what}: {hvs_fps} HVS frames/s, {hd_fps} encoder frames/s (frame count {:#010x}), {want} expected",
        b.hd.rd(HD_FRAME_COUNT)
    );
    if !near(hvs_fps) {
        fail(format!("HVS channel counts {hvs_fps} frames/s, not {want}"));
    }
    if hd_counts && !near(hd_fps) {
        fail(format!("encoder frame counter counts {hd_fps} frames/s, not {want}"));
    }
    let errors = hvs.errors(ch);
    if !errors.is_empty() {
        fail(format!("HVS errors on channel {ch}: {}", errors.join(" ")));
    }
    // 3. Encoder status.
    let sched = b.hdmi.rd(HDMI_SCHEDULER_CONTROL);
    if sched & SCHEDULER_HDMI_ACTIVE == 0 {
        fail(format!("packet scheduler not active (SCHEDULER_CONTROL {sched:#010x})"));
    }
    let fifo = b.hdmi.rd(HDMI_FIFO_CTL);
    if fifo & FIFO_CTL_RECENTER_DONE == 0 {
        fail(format!("HDMI FIFO not recentred (FIFO_CTL {fifo:#010x})"));
    }
    println!(
        "vc4d: mode5: {what}: SCHEDULER_CONTROL {sched:#010x} FIFO_CTL {fifo:#010x} MISC_CONTROL {:#010x} VID_CTL {:#010x}",
        b.hdmi.rd(HDMI_MISC_CONTROL),
        b.hd.rd(HD_VID_CTL)
    );
    // 4. The monitor on DDC: EDID, and SCDC lock status where it has one.
    match crate::ddc_stb::StbDdc::open(fdt) {
        Ok(ddc) => {
            let mut block = [0u8; 128];
            match ddc.read_edid(0, &mut block) {
                Ok(()) if vc4_edid::checksum(&block) == 0 => println!("vc4d: mode5: {what}: EDID still answers, checksum good"),
                Ok(()) => fail("EDID checksum bad after the change".into()),
                Err(err) => fail(format!("EDID no longer answers: {err}")),
            }
            let mut version = [0u8; 1];
            match ddc.read_reg(SCDC_ADDR, SCDC_SINK_VERSION, &mut version) {
                Ok(()) => {
                    let mut flags = [0u8; 1];
                    match ddc.read_reg(SCDC_ADDR, SCDC_STATUS_FLAGS_0, &mut flags) {
                        Ok(()) => {
                            let f = flags[0];
                            println!(
                                "vc4d: mode5: {what}: SCDC version {} status {f:#04x}: clock {}, channels locked {}{}{}",
                                version[0],
                                if f & 1 != 0 { "detected" } else { "NOT detected" },
                                f >> 1 & 1,
                                f >> 2 & 1,
                                f >> 3 & 1
                            );
                            if f & 0x0f != 0x0f {
                                fail(format!("the monitor's SCDC reports no lock (status {f:#04x})"));
                            }
                        }
                        Err(err) => println!("vc4d: mode5: {what}: SCDC status not read: {err}"),
                    }
                }
                Err(_) => println!("vc4d: mode5: {what}: no SCDC (monitor without HDMI 2.0 status), lock not readable"),
            }
        }
        Err(err) => println!("vc4d: mode5: {what}: DDC not opened: {err}"),
    }
    println!("vc4d: mode5: {what}: {}", if ok { "PASS" } else { "FAILED" });
    ok
}

/// The firmware's HDMI clocks, raised where 720p needs more (it never
/// does after 1080p). False if a clock is too slow and cannot be raised.
fn clocks(tmds_hz: u64) -> bool {
    let need_hsm = (tmds_hz / 100 * 101) as u32;
    let need_bvb: u32 = if tmds_hz > 297_000_000 {
        300_000_000
    } else if tmds_hz > 148_500_000 {
        150_000_000
    } else {
        75_000_000
    };
    let fw = match Firmware::open() {
        Ok(fw) => fw,
        Err(err) => {
            println!("vc4d: mode5: 720p: clocks not read ({err}), left as the firmware set them for its mode");
            return true;
        }
    };
    let mut ok = true;
    for (id, name, need) in [(CLOCK_HDMI, "hdmi (HSM)", need_hsm), (CLOCK_BVB, "bvb", need_bvb)] {
        match fw.clock_rate(id) {
            Ok(rate) if rate >= need => {
                println!("vc4d: mode5: 720p: clock {name} (firmware id {id}) {rate} Hz, needs {need}: kept")
            }
            Ok(rate) => match fw.set_clock_rate(id, need) {
                Ok(new) => {
                    println!("vc4d: mode5: 720p: clock {name} (firmware id {id}) {rate} Hz, needs {need}: set, now {new}");
                    ok &= new >= need;
                }
                Err(err) => {
                    println!("vc4d: mode5: 720p: clock {name} {rate} Hz, needs {need}: setting failed: {err}");
                    ok = false;
                }
            },
            Err(err) => println!("vc4d: mode5: 720p: clock {name} (firmware id {id}) not read: {err}"),
        }
    }
    ok
}

/// Step 4. `same` is the firmware's mode as vc4d computed it (step 2
/// compared it equal). Returns true when the screen stays at 720p.
pub fn switch(fdt: &Fdt, hvs: &Hvs, ch: usize, same: &Same, after: After) -> bool {
    let fw_t = same.timing;
    let t = P720;
    let tmds = u64::from(t.pixel_khz) * 1000;
    println!(
        "vc4d: mode5: 720p: from the firmware's {}x{}@{} to 1280x720@60 on channel {ch} ({after:?})",
        fw_t.hactive,
        fw_t.vactive,
        fw_t.refresh_mhz() / 1000
    );
    let (Ok(pv), Ok(h)) = (vc4_mode::pv_bcm2711(&t), vc4_mode::hdmi_timing_bcm2711(&t)) else {
        println!("vc4d: mode5: 720p: skipped, the mode does not fit the registers");
        return false;
    };
    let Some(b) = Blocks::open(fdt) else {
        println!("vc4d: mode5: 720p: skipped, HDMI0 regions not mapped");
        return false;
    };
    let saved = save(&b, hvs, ch);
    // Read only: the PHY values vc4d would write for the firmware's own
    // rate, next to the firmware's.
    let fw_phy = vc4_mode::phy5(u64::from(fw_t.pixel_khz) * 1000);
    let ours = vc4_mode::phy5_result(&fw_phy, |r, o| {
        saved.phy.iter().find(|p| p.0 == r && p.1 == o).map_or(0, |p| p.3)
    });
    let mut phy_same = 0;
    for (&(_, _, name, firmware), &(_, _, _, value)) in saved.phy.iter().zip(ours.iter()) {
        phy_same += usize::from(value == firmware);
        println!(
            "vc4d: mode5: phy({}) {name:<9} ours {value:#010x} firmware {firmware:#010x}{}",
            fw_t.vactive,
            if value == firmware { "" } else { "  DIFFERENT" }
        );
    }
    println!("vc4d: mode5: phy({}): {phy_same} of {} registers equal the firmware's", fw_t.vactive, ours.len());
    if !clocks(tmds) {
        println!("vc4d: mode5: 720p: skipped, the HDMI clocks are too slow");
        return false;
    }
    let (hd_fps, hvs_fps) = measure(&b, hvs, ch);
    println!(
        "vc4d: mode5: before: {hvs_fps} HVS frames/s, {hd_fps} encoder frames/s (frame count {:#010x}, VID_CTL {:#010x})",
        b.hd.rd(HD_FRAME_COUNT),
        b.hd.rd(HD_VID_CTL)
    );
    let hd_counts = hd_fps > 0;
    // The firmware's picture, its top left 1280x720, while the switch is
    // checked; vc4d's own list replaces it when it takes over.
    let first = hvs::decode_list(hvs.generation(), &hvs.read_list(saved.hvs_list, 32)).into_iter().next();
    let list = match first {
        Some(p) => hvs.list(&[Plane {
            bus: p.bus,
            pitch: p.pitch,
            x: 0,
            y: 0,
            width: t.hactive.min(p.width),
            height: t.vactive.min(p.height),
            alpha: false,
            scale: None,
        }]),
        None => hvs.list(&[]),
    };
    hvs.write_dlist(SWITCH_LIST, &list);
    let target = Target::Mode { t: &t, pv, hdmi: vc4_mode::hdmi_bcm2711_regs(&h), phy: vc4_mode::phy5(tmds) };
    println!("vc4d: mode5: 720p: switching (screen blank for a moment)");
    apply(&b, hvs, ch, &target, &fw_t);
    let good = verify(&b, hvs, ch, &target, &t, hd_counts, fdt, "720p");
    let restore = |why: &str| {
        println!("vc4d: mode5: restore: {why}, back to the firmware's mode");
        let fw = Target::Firmware(&saved);
        apply(&b, hvs, ch, &fw, &t);
        verify(&b, hvs, ch, &fw, &fw_t, hd_counts, fdt, "restore")
    };
    if !good {
        restore("720p failed its checks");
        return false;
    }
    match after {
        After::Keep => {
            println!("vc4d: mode5: 720p: kept (keep), the desktop starts at 1280x720");
            true
        }
        After::Restore(secs) => {
            println!("vc4d: mode5: 720p: holding for {secs} s");
            thread::sleep(Duration::from_secs(secs));
            restore(&format!("{secs} s are over"));
            false
        }
    }
}

