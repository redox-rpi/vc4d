// SPDX-License-Identifier: GPL-2.0-only
//
// Register values for a video mode on the BCM283x display path: pixel
// valve 2 and the HDMI core (stage 2c of redox-rpi#188). Pure arithmetic,
// no register access, so it tests on any host (cargo test here).
//
// Field layouts from Linux drivers/gpu/drm/vc4/vc4_regs.h and
// vc4_hdmi_regs.h (BCM2835 table), as used by vc4_crtc.c and vc4_hdmi.c;
// the code is our own. The tests check the result against what Broadcom's
// firmware programmed for 1080p60 (redox-rpi Pi test row 337, socdump
// reference pi3bp-firmware-1080p-display.regs).

pub use vc4_edid::Timing;

/// Bytes in pixel valve 2's FIFO (HDMI) and the HVS latency Linux leaves
/// free in it, in pixels.
const PV2_FIFO_BYTES: u32 = 64;
const HVS_FIFO_LATENCY_PIX: u32 = 6;

/// Pixel valve register offsets.
pub const PV_CONTROL: usize = 0x00;
pub const PV_V_CONTROL: usize = 0x04;

// PV_CONTROL fields.
const PV_CONTROL_FORMAT_24: u32 = 0 << 21;
const PV_CONTROL_FIFO_LEVEL_SHIFT: u32 = 15;
const PV_CONTROL_CLR_AT_START: u32 = 1 << 14;
const PV_CONTROL_TRIGGER_UNDERFLOW: u32 = 1 << 13;
const PV_CONTROL_WAIT_HSTART: u32 = 1 << 12;
const PV_CONTROL_CLK_SELECT_DPI_SMI_HDMI: u32 = 1 << 2;
/// Empties the valve's FIFO; reads back as 0.
pub const PV_CONTROL_FIFO_CLR: u32 = 1 << 1;
const PV_CONTROL_EN: u32 = 1 << 0;

// PV_V_CONTROL fields.
const PV_VCONTROL_VIDEN: u32 = 1 << 0;
const PV_VCONTROL_CONTINUOUS: u32 = 1 << 1;
const PV_VCONTROL_INTERLACE: u32 = 1 << 4;

// HDMI_HORZA fields.
const HDMI_HORZA_VPOS: u32 = 1 << 14;
const HDMI_HORZA_HPOS: u32 = 1 << 13;

/// Pixel valve registers, at their offsets from the block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Pv {
    pub control: u32,   // 0x00
    pub v_control: u32, // 0x04
    pub horza: u32,     // 0x0c
    pub horzb: u32,     // 0x10
    pub verta: u32,     // 0x14
    pub vertb: u32,     // 0x18
}

impl Pv {
    /// (offset, name, value), in the order they are written.
    pub fn regs(&self) -> [(usize, &'static str, u32); 6] {
        [
            (0x0c, "HORZA", self.horza),
            (0x10, "HORZB", self.horzb),
            (0x14, "VERTA", self.verta),
            (0x18, "VERTB", self.vertb),
            (0x00, "CONTROL", self.control),
            (0x04, "V_CONTROL", self.v_control),
        ]
    }
}

/// HDMI core timing registers (BCM2835 offsets).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HdmiTiming {
    pub horza: u32,  // 0x0c4
    pub horzb: u32,  // 0x0c8
    pub verta0: u32, // 0x0cc
    pub vertb0: u32, // 0x0d0
    pub verta1: u32, // 0x0d4
    pub vertb1: u32, // 0x0d8
}

impl HdmiTiming {
    pub fn regs(&self) -> [(usize, &'static str, u32); 6] {
        [
            (0x0c4, "HORZA", self.horza),
            (0x0c8, "HORZB", self.horzb),
            (0x0cc, "VERTA0", self.verta0),
            (0x0d0, "VERTB0", self.vertb0),
            (0x0d4, "VERTA1", self.verta1),
            (0x0d8, "VERTB1", self.vertb1),
        ]
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// A field is wider than its register allows.
    TooWide(&'static str, u32),
    /// Interlaced modes are not done yet.
    Interlaced,
}

fn field(name: &'static str, value: u32, bits: u32) -> Result<u32, Error> {
    if value >> bits != 0 {
        Err(Error::TooWide(name, value))
    } else {
        Ok(value)
    }
}

/// Pixel valve 2 (HDMI) for a progressive mode, as Linux's vc4_crtc sets
/// it: 24-bit pixels, the FIFO level for HDMI, HDMI clock, enabled, video
/// on and continuous.
pub fn pv2(t: &Timing) -> Result<Pv, Error> {
    if t.interlaced {
        return Err(Error::Interlaced);
    }
    let fifo_level = PV2_FIFO_BYTES - 3 * HVS_FIFO_LATENCY_PIX;
    let h = |name, v| field(name, v, 16);
    Ok(Pv {
        control: PV_CONTROL_FORMAT_24
            | fifo_level << PV_CONTROL_FIFO_LEVEL_SHIFT
            | PV_CONTROL_CLR_AT_START
            | PV_CONTROL_TRIGGER_UNDERFLOW
            | PV_CONTROL_WAIT_HSTART
            | PV_CONTROL_CLK_SELECT_DPI_SMI_HDMI
            | PV_CONTROL_EN,
        v_control: PV_VCONTROL_VIDEN | PV_VCONTROL_CONTINUOUS,
        horza: h("hbp", t.hbp)? << 16 | h("hsync", t.hsync)?,
        horzb: h("hfp", t.hfp)? << 16 | h("hactive", t.hactive)?,
        verta: h("vbp", t.vbp)? << 16 | h("vsync", t.vsync)?,
        vertb: h("vfp", t.vfp)? << 16 | h("vactive", t.vactive)?,
    })
}

// BCM2711 (Pi 4, Pi 400), redox-rpi#671. The layouts below are read from
// what Broadcom's firmware programs at 1080p60 (pi-tests Test 701,
// HP 27 FHD on HDMI0) and agree with Linux vc4_crtc.c and vc4_hdmi.c's
// BCM2711 paths; the code is our own.

/// Bytes in the BCM2711's pixel valve 2 FIFO (HDMI0): the firmware's FIFO
/// level is 238 = 256 - 3 * 6.
const PV2_BCM2711_FIFO_BYTES: u32 = 256;
/// The FIFO level's two high bits (BCM2711 pixel valves).
const PV5_CONTROL_FIFO_LEVEL_HIGH_SHIFT: u32 = 25;
/// Set by the firmware on HDMI0's valve; its meaning is not confirmed.
const PV5_CONTROL_BIT24: u32 = 1 << 24;

/// Pixel valve 2 (HDMI0) on the BCM2711. The valve takes two pixels per
/// clock, so every horizontal value is halved (Linux divides by
/// `pixels_per_clock`). CONTROL: 24-bit pixels, the FIFO level split over
/// bits 20:15 and 26:25, clear at start, bit 24 as the firmware sets it,
/// enabled; no WAIT_HSTART or TRIGGER_UNDERFLOW and clock select 0, as the
/// firmware leaves them.
pub fn pv_bcm2711(t: &Timing) -> Result<Pv, Error> {
    let mut pv = pv2(t)?;
    let half = |name, v: u32| -> Result<u32, Error> {
        if v % 2 != 0 {
            Err(Error::TooWide(name, v))
        } else {
            field(name, v / 2, 16)
        }
    };
    pv.horza = half("hbp", t.hbp)? << 16 | half("hsync", t.hsync)?;
    pv.horzb = half("hfp", t.hfp)? << 16 | half("hactive", t.hactive)?;
    let level = PV2_BCM2711_FIFO_BYTES - 3 * HVS_FIFO_LATENCY_PIX;
    pv.control = PV_CONTROL_FORMAT_24
        | (level & 0x3f) << PV_CONTROL_FIFO_LEVEL_SHIFT
        | (level >> 6) << PV5_CONTROL_FIFO_LEVEL_HIGH_SHIFT
        | PV5_CONTROL_BIT24
        | PV_CONTROL_CLR_AT_START
        | PV_CONTROL_EN;
    Ok(pv)
}

/// HDMI0 core timing registers on the BCM2711 (offsets in its "hdmi"
/// region). Full pixel values, not halved.
pub fn hdmi_timing_bcm2711(t: &Timing) -> Result<HdmiTiming, Error> {
    if t.interlaced {
        return Err(Error::Interlaced);
    }
    let horza = field("hfp", t.hfp, 16)? << 16
        | if t.hsync_positive { 1 << 15 } else { 0 }
        | if t.vsync_positive { 1 << 14 } else { 0 }
        | field("hactive", t.hactive, 13)?;
    let horzb = field("hbp", t.hbp, 16)? << 16 | field("hsync", t.hsync, 16)?;
    let verta = field("vsync", t.vsync, 8)? << 24 | field("vfp", t.vfp, 8)? << 16 | field("vactive", t.vactive, 13)?;
    let vertb = field("vbp", t.vbp, 16)?;
    Ok(HdmiTiming { horza, horzb, verta0: verta, vertb0: vertb, verta1: verta, vertb1: vertb })
}

/// The BCM2711 HDMI0 timing registers: (offset, name, value).
pub fn hdmi_bcm2711_regs(h: &HdmiTiming) -> [(usize, &'static str, u32); 6] {
    [
        (0x0e4, "HORZA", h.horza),
        (0x0e8, "HORZB", h.horzb),
        (0x0ec, "VERTA0", h.verta0),
        (0x0f0, "VERTB0", h.vertb0),
        (0x0f4, "VERTA1", h.verta1),
        (0x0f8, "VERTB1", h.vertb1),
    ]
}

/// The interlace bit of PV_V_CONTROL, for readers that decode it.
pub const fn pv_interlaced(v_control: u32) -> bool {
    v_control & PV_VCONTROL_INTERLACE != 0
}

/// The HDMI core's timing for a progressive mode, as Linux's vc4_hdmi sets
/// it on the BCM2835: sync polarities, active width, porches and syncs.
/// Field 1 equals field 0 for progressive modes.
pub fn hdmi_timing(t: &Timing) -> Result<HdmiTiming, Error> {
    if t.interlaced {
        return Err(Error::Interlaced);
    }
    let horza = if t.vsync_positive { HDMI_HORZA_VPOS } else { 0 }
        | if t.hsync_positive { HDMI_HORZA_HPOS } else { 0 }
        | field("hactive", t.hactive, 13)?;
    let horzb = field("hbp", t.hbp, 10)? << 20 | field("hsync", t.hsync, 10)? << 10 | field("hfp", t.hfp, 10)?;
    let verta = field("vsync", t.vsync, 5)? << 20 | field("vfp", t.vfp, 7)? << 13 | field("vactive", t.vactive, 13)?;
    let vertb = field("vbp", t.vbp, 9)?;
    Ok(HdmiTiming { horza, horzb, verta0: verta, vertb0: vertb, verta1: verta, vertb1: vertb })
}

/// One step of programming a pixel valve and the HDMI core.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// Write the value to the pixel valve register at this offset.
    Write(usize, &'static str, u32),
    /// Write the value to the HDMI core register at this offset.
    Hdmi(usize, &'static str, u32),
    /// Let this many frames pass (the valve finishes the frame it is in).
    WaitFrames(u32),
}

/// The steps that program a running pixel valve with `pv`, in the order of
/// Linux's vc4_crtc.c (vc4_crtc_disable(), vc4_crtc_pixelvalve_reset(),
/// vc4_crtc_config_pv(), vc4_crtc_atomic_enable()): video off at the end
/// of the frame, the valve off, its FIFO emptied, the timing, then the
/// valve and the video on again. The HDMI encoder and the HVS keep
/// running; the screen is blank for a few frames.
pub fn pv_program(pv: &Pv) -> Vec<Step> {
    mode_program(pv, None)
}

/// As `pv_program`, and with `hdmi` the HDMI core's timing too, written
/// while the valve is off, before the valve's own timing: Linux sets the
/// encoder's timing in vc4_hdmi_encoder_pre_crtc_configure() (through
/// vc4_hdmi_set_timings()), which runs before vc4_crtc_config_pv(). The
/// encoder itself stays on; only its timing registers are written.
pub fn mode_program(pv: &Pv, hdmi: Option<&HdmiTiming>) -> Vec<Step> {
    mode_program_regs(pv, hdmi.map(|h| h.regs()).as_ref().map(|r| &r[..]))
}

/// As `mode_program`, with the HDMI timing registers given as (offset,
/// name, value), for the BCM2711's offsets (`hdmi_bcm2711_regs`).
pub fn mode_program_regs(pv: &Pv, hdmi: Option<&[(usize, &'static str, u32)]>) -> Vec<Step> {
    let off = pv.control & !PV_CONTROL_EN;
    let mut steps = vec![
        Step::Write(PV_V_CONTROL, "V_CONTROL", pv.v_control & !PV_VCONTROL_VIDEN),
        Step::WaitFrames(2),
        Step::Write(PV_CONTROL, "CONTROL", off),
        Step::Write(PV_CONTROL, "CONTROL", off | PV_CONTROL_FIFO_CLR),
    ];
    if let Some(hdmi) = hdmi {
        steps.extend(hdmi.iter().map(|&(offset, name, value)| Step::Hdmi(offset, name, value)));
    }
    for (offset, name, value) in pv.regs() {
        if offset != PV_CONTROL && offset != PV_V_CONTROL {
            steps.push(Step::Write(offset, name, value));
        }
    }
    steps.extend([
        Step::Write(PV_CONTROL, "CONTROL", pv.control),
        Step::Write(PV_V_CONTROL, "V_CONTROL", pv.v_control),
        Step::WaitFrames(2),
    ]);
    steps
}

/// What a register reads back after `pv_program`: FIFO_CLR is not kept.
pub fn readback_mask(offset: usize) -> u32 {
    if offset == PV_CONTROL {
        !PV_CONTROL_FIFO_CLR
    } else {
        !0
    }
}

/// Registers that differ between two sets: (name, ours, theirs).
pub fn differences(
    ours: &[(usize, &'static str, u32)],
    theirs: impl Fn(usize) -> u32,
) -> Vec<(&'static str, u32, u32)> {
    ours.iter()
        .filter_map(|&(offset, name, value)| {
            let other = theirs(offset);
            (other != value).then_some((name, value, other))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 1080p60 as the test monitor's EDID gives it (Pi test row 387).
    const P1080: Timing = Timing {
        pixel_khz: 148_500, hactive: 1920, hfp: 88, hsync: 44, hbp: 148,
        vactive: 1080, vfp: 4, vsync: 5, vbp: 36,
        hsync_positive: true, vsync_positive: true, interlaced: false,
    };

    /// What Broadcom's firmware left in pixel valve 2 and the HDMI core at
    /// 1080p60 (Pi test row 337).
    fn firmware_pv2(offset: usize) -> u32 {
        match offset {
            0x00 => 0x0017_7005,
            0x04 => 0x0000_0003,
            0x0c => 0x0094_002c,
            0x10 => 0x0058_0780,
            0x14 => 0x0024_0005,
            0x18 => 0x0004_0438,
            _ => unreachable!(),
        }
    }

    /// What the firmware left in the BCM2711's pixel valve 2 and HDMI0
    /// core at 1080p60 (pi-tests Test 701).
    #[test]
    fn bcm2711_equals_firmware_1080p() {
        let pv = pv_bcm2711(&P1080).unwrap();
        assert_eq!(pv.control, 0x0717_4001);
        assert_eq!(pv.v_control, 0x0000_0003);
        assert_eq!(pv.horza, 0x004a_0016);
        assert_eq!(pv.horzb, 0x002c_03c0);
        assert_eq!(pv.verta, 0x0024_0005);
        assert_eq!(pv.vertb, 0x0004_0438);
        let h = hdmi_timing_bcm2711(&P1080).unwrap();
        let firmware = |o| match o {
            0x0e4 => 0x0058_c780,
            0x0e8 => 0x0094_002c,
            0x0ec | 0x0f4 => 0x0504_0438,
            0x0f0 | 0x0f8 => 0x0000_0024,
            _ => unreachable!(),
        };
        assert!(differences(&hdmi_bcm2711_regs(&h), firmware).is_empty());
    }

    #[test]
    fn bcm2711_program_writes_hdmi0_offsets() {
        let pv = pv_bcm2711(&P1080).unwrap();
        let h = hdmi_bcm2711_regs(&hdmi_timing_bcm2711(&P1080).unwrap());
        let steps = mode_program_regs(&pv, Some(&h));
        let hdmi: Vec<usize> = steps.iter().filter_map(|s| match s { Step::Hdmi(o, _, _) => Some(*o), _ => None }).collect();
        assert_eq!(hdmi, vec![0x0e4, 0x0e8, 0x0ec, 0x0f0, 0x0f4, 0x0f8]);
        assert_eq!(steps.last(), Some(&Step::WaitFrames(2)));
        assert!(steps.contains(&Step::Write(PV_CONTROL, "CONTROL", 0x0717_4001)));
    }

    fn firmware_hdmi(offset: usize) -> u32 {
        match offset {
            0x0c4 => 0x0000_6780,
            0x0c8 => 0x0940_b058,
            0x0cc | 0x0d4 => 0x0050_8438,
            0x0d0 | 0x0d8 => 0x0000_0024,
            _ => unreachable!(),
        }
    }

    #[test]
    fn pv2_matches_the_firmware_at_1080p() {
        let pv = pv2(&P1080).unwrap();
        assert_eq!(differences(&pv.regs(), firmware_pv2), vec![]);
    }

    #[test]
    fn hdmi_matches_the_firmware_at_1080p() {
        let h = hdmi_timing(&P1080).unwrap();
        assert_eq!(differences(&h.regs(), firmware_hdmi), vec![]);
    }

    #[test]
    fn the_monitors_own_edid_gives_the_firmwares_registers() {
        // The EDID vc4d read from the test monitor (row 387), parsed.
        let edid = vc4_edid::Edid::parse(&NANOKVM).unwrap();
        let t = edid.preferred().unwrap();
        assert_eq!(differences(&pv2(t).unwrap().regs(), firmware_pv2), vec![]);
        assert_eq!(differences(&hdmi_timing(t).unwrap().regs(), firmware_hdmi), vec![]);
    }

    #[test]
    fn p720_and_polarity() {
        let t = Timing {
            pixel_khz: 74_250, hactive: 1280, hfp: 110, hsync: 40, hbp: 220,
            vactive: 720, vfp: 5, vsync: 5, vbp: 20,
            hsync_positive: false, vsync_positive: true, interlaced: false,
        };
        let h = hdmi_timing(&t).unwrap();
        assert_eq!(h.horza, HDMI_HORZA_VPOS | 1280);
        assert_eq!(h.horzb, 220 << 20 | 40 << 10 | 110);
        assert_eq!(h.verta0, 5 << 20 | 5 << 13 | 720);
        assert_eq!(h.vertb0, 20);
        let pv = pv2(&t).unwrap();
        assert_eq!((pv.horza, pv.horzb, pv.verta, pv.vertb), (220 << 16 | 40, 110 << 16 | 1280, 20 << 16 | 5, 5 << 16 | 720));
    }

    #[test]
    fn limits() {
        let mut t = P1080;
        t.interlaced = true;
        assert_eq!(pv2(&t), Err(Error::Interlaced));
        let mut t = P1080;
        t.hfp = 1024;
        assert_eq!(hdmi_timing(&t), Err(Error::TooWide("hfp", 1024)));
        let mut t = P1080;
        t.vfp = 128;
        assert_eq!(hdmi_timing(&t), Err(Error::TooWide("vfp", 128)));
        assert!(!pv_interlaced(pv2(&P1080).unwrap().v_control));
    }

    /// A pixel valve as registers in memory: FIFO_CLR reads back as 0.
    fn run(steps: &[Step], regs: &mut std::collections::BTreeMap<usize, u32>) -> Vec<String> {
        let mut log = Vec::new();
        for step in steps {
            match *step {
                Step::Write(offset, name, value) => {
                    regs.insert(offset, value & readback_mask(offset));
                    log.push(format!("{name} {value:#x}"));
                }
                Step::Hdmi(_, name, value) => log.push(format!("hdmi {name} {value:#x}")),
                Step::WaitFrames(n) => log.push(format!("wait {n}")),
            }
        }
        log
    }

    #[test]
    fn hdmi_timing_while_the_valve_is_off() {
        let pv = pv2(&P1080).unwrap();
        let hdmi = hdmi_timing(&P1080).unwrap();
        let steps = mode_program(&pv, Some(&hdmi));
        let fifo_clear = steps.iter().position(|s| *s == Step::Write(PV_CONTROL, "CONTROL", 0x0017_7006)).unwrap();
        let pv_timing = steps.iter().position(|s| *s == Step::Write(0x0c, "HORZA", 0x0094_002c)).unwrap();
        let hdmi_at: Vec<usize> =
            steps.iter().enumerate().filter(|(_, s)| matches!(s, Step::Hdmi(..))).map(|(i, _)| i).collect();
        assert_eq!(hdmi_at.len(), 6);
        assert!(hdmi_at.iter().all(|&i| fifo_clear < i && i < pv_timing), "{steps:?}");
        // The written values are the firmware's (Pi test 337 and 444).
        for step in &steps {
            if let Step::Hdmi(offset, name, value) = *step {
                assert_eq!(value, firmware_hdmi(offset), "{name}");
            }
        }
        // Without HDMI, the same as pv_program, and the pixel valve part
        // is unchanged by it.
        assert_eq!(mode_program(&pv, None), pv_program(&pv));
        let pv_only: Vec<Step> = steps.into_iter().filter(|s| !matches!(s, Step::Hdmi(..))).collect();
        assert_eq!(pv_only, pv_program(&pv));
    }

    #[test]
    fn program_ends_with_the_firmwares_registers() {
        let pv = pv2(&P1080).unwrap();
        let mut regs: std::collections::BTreeMap<usize, u32> =
            [0x00, 0x04, 0x0c, 0x10, 0x14, 0x18].iter().map(|&o| (o, 0)).collect();
        run(&pv_program(&pv), &mut regs);
        for (offset, value) in &regs {
            assert_eq!(*value, firmware_pv2(*offset), "offset {offset:#x}");
        }
    }

    #[test]
    fn program_order_follows_linux() {
        let pv = pv2(&P1080).unwrap();
        let steps = pv_program(&pv);
        let pos = |want: Step| steps.iter().position(|s| *s == want).unwrap();
        let video_off = pos(Step::Write(PV_V_CONTROL, "V_CONTROL", PV_VCONTROL_CONTINUOUS));
        let valve_off = pos(Step::Write(PV_CONTROL, "CONTROL", 0x0017_7004));
        let fifo_clear = pos(Step::Write(PV_CONTROL, "CONTROL", 0x0017_7006));
        let timing = pos(Step::Write(0x0c, "HORZA", 0x0094_002c));
        let valve_on = pos(Step::Write(PV_CONTROL, "CONTROL", 0x0017_7005));
        let video_on = pos(Step::Write(PV_V_CONTROL, "V_CONTROL", 3));
        assert_eq!(video_off, 0);
        assert_eq!(steps[1], Step::WaitFrames(2), "the frame ends before the valve stops");
        assert!(video_off < valve_off && valve_off < fifo_clear && fifo_clear < timing);
        assert!(timing < valve_on && valve_on < video_on);
        assert_eq!(steps.last(), Some(&Step::WaitFrames(2)));
        // Every timing register once, while the valve is off.
        let timing_writes: Vec<_> =
            steps[fifo_clear..valve_on].iter().filter(|s| matches!(s, Step::Write(o, ..) if *o >= 0x0c)).collect();
        assert_eq!(timing_writes.len(), 4);
    }

    /// The test monitor's EDID (Pi test row 387), the same bytes as the
    /// vc4-edid test.
    const NANOKVM: [u8; 256] = include!("../../edid/tests/nanokvm.in");
}
