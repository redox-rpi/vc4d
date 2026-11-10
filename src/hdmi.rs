// SPDX-License-Identifier: GPL-2.0-only
//
// The HDMI encoder of the BCM2835/BCM2837, read only: the HDMI core block
// and the HD block (video control, colour conversion, frame counter), as the
// firmware set them up. For the noise bands of Pi test #275 and as the
// start of stage 2b (#188), where vc4d will program them itself.
//
// Register offsets translated from Linux v6.12,
// drivers/gpu/drm/vc4/vc4_hdmi_regs.h (vc4_hdmi_fields, the BCM2835
// table), and fields from vc4_regs.h (VC4_HDMI_*, VC4_HD_*). The decoding
// for the log is our own code.

use std::ptr;

/// Bus addresses of the two blocks if the devicetree has none.
pub const HDMI_BUS_ADDR: usize = 0x7e90_2000;
pub const HD_BUS_ADDR: usize = 0x7e80_8000;
pub const HDMI_SIZE: usize = 0x1000;
pub const HD_SIZE: usize = 0x1000;

// HDMI core block.
const HDMI_CORE_REV: usize = 0x0000;
const HDMI_HOTPLUG: usize = 0x000c;
const HDMI_FIFO_CTL: usize = 0x005c;
const HDMI_SCHEDULER_CONTROL: usize = 0x00c0;
const HDMI_HORZA: usize = 0x00c4;
const HDMI_HORZB: usize = 0x00c8;
const HDMI_VERTA0: usize = 0x00cc;
const HDMI_VERTB0: usize = 0x00d0;
const HDMI_MISC_CONTROL: usize = 0x00e4;
const HDMI_TX_PHY_CTL_0: usize = 0x02c4;

// HD block.
const HD_M_CTL: usize = 0x000c;
const HD_VID_CTL: usize = 0x0038;
const HD_CSC_CTL: usize = 0x0040;
const HD_FRAME_COUNT: usize = 0x0068;

const HD_VID_CTL_ENABLE: u32 = 1 << 31;
const HD_VID_CTL_UNDERFLOW_ENABLE: u32 = 1 << 30;
const HDMI_HOTPLUG_CONNECTED: u32 = 1 << 0;
const HDMI_FIFO_CTL_RECENTER_DONE: u32 = 1 << 14;
const HDMI_SCHEDULER_CONTROL_HDMI_ACTIVE: u32 = 1 << 1;
const HDMI_SCHEDULER_CONTROL_MODE_HDMI: u32 = 1 << 0;

fn bits(reg: u32, high: u32, low: u32) -> u32 {
    (reg >> low) & (u32::MAX >> (31 - (high - low)))
}

/// GPIO block registers for the hotplug line (BCM2835 GPLEV0/1).
pub const GPIO_SIZE: usize = 0x1000;
const GPIO_GPLEV0: usize = 0x0034;

/// The hotplug line when the board wires it to a SoC GPIO (devicetree
/// `hpd-gpios`, as on the Pi 3B+: GPIO 28 "HDMI_HPD_N", active low)
/// instead of the HDMI core's HOTPLUG bit.
pub struct HpdGpio {
    pub gpio: *mut u32,
    pub pin: u32,
    pub active_low: bool,
}

impl HpdGpio {
    /// Whether the line says a monitor is there, and the raw level.
    pub fn read(&self) -> (bool, u32) {
        let reg = unsafe { ptr::read_volatile(self.gpio.add((GPIO_GPLEV0 + 4 * (self.pin / 32) as usize) / 4)) };
        let level = (reg >> (self.pin % 32)) & 1;
        ((level == 1) != self.active_low, level)
    }
}

pub struct Hdmi {
    hdmi: *mut u32,
    hd: *mut u32,
    hpd: Option<HpdGpio>,
}

// Device registers, only read.
unsafe impl Send for Hdmi {}

impl Hdmi {
    /// # Safety
    /// `hdmi` and `hd` map the two blocks (HDMI_SIZE and HD_SIZE bytes).
    pub unsafe fn new(hdmi: *mut u32, hd: *mut u32) -> Self {
        Hdmi { hdmi, hd, hpd: None }
    }

    /// Read hotplug from this GPIO too.
    pub fn set_hpd(&mut self, hpd: HpdGpio) {
        self.hpd = Some(hpd);
    }

    fn core(&self, offset: usize) -> u32 {
        unsafe { ptr::read_volatile(self.hdmi.add(offset / 4)) }
    }

    fn hd(&self, offset: usize) -> u32 {
        unsafe { ptr::read_volatile(self.hd.add(offset / 4)) }
    }

    /// HD_FRAME_COUNT: frames the encoder sent.
    pub fn frame_count(&self) -> u32 {
        self.hd(HD_FRAME_COUNT)
    }

    /// The state that can change while running, for the minute lines.
    pub fn status(&self) -> String {
        let hotplug = self.core(HDMI_HOTPLUG);
        let fifo = self.core(HDMI_FIFO_CTL);
        let vid = self.hd(HD_VID_CTL);
        let gpio = self.hpd.as_ref().map_or(String::new(), |hpd| {
            let (connected, level) = hpd.read();
            format!(
                ", hpd gpio {} level {level} ({})",
                hpd.pin,
                if connected { "connected" } else { "not connected" }
            )
        });
        format!(
            "hotplug {hotplug:#x} ({}){gpio}, fifo_ctl {fifo:#06x} (recenter done {}), vid_ctl {vid:#010x}",
            if hotplug & HDMI_HOTPLUG_CONNECTED != 0 { "connected" } else { "not connected" },
            u32::from(fifo & HDMI_FIFO_CTL_RECENTER_DONE != 0),
        )
    }

    /// All registers read here and the mode they give, for the start.
    pub fn describe(&self) -> String {
        let (horza, horzb) = (self.core(HDMI_HORZA), self.core(HDMI_HORZB));
        let (verta, vertb) = (self.core(HDMI_VERTA0), self.core(HDMI_VERTB0));
        let sched = self.core(HDMI_SCHEDULER_CONTROL);
        let vid = self.hd(HD_VID_CTL);
        format!(
            "core rev {:#010x}, {}, scheduler {sched:#010x} (hdmi mode {}, active {}), \
             h active {} fp {} sync {} bp {}, v active {} fp {} sync {} bp {}, misc {:#010x}, phy {:#010x}, \
             m_ctl {:#010x}, vid_ctl enable {} underflow enable {}, csc_ctl {:#010x}, frames {}",
            self.core(HDMI_CORE_REV),
            self.status(),
            u32::from(sched & HDMI_SCHEDULER_CONTROL_MODE_HDMI != 0),
            u32::from(sched & HDMI_SCHEDULER_CONTROL_HDMI_ACTIVE != 0),
            bits(horza, 12, 0),
            bits(horzb, 9, 0),
            bits(horzb, 19, 10),
            bits(horzb, 29, 20),
            bits(verta, 12, 0),
            bits(verta, 19, 13),
            bits(verta, 24, 20),
            bits(vertb, 8, 0),
            self.core(HDMI_MISC_CONTROL),
            self.core(HDMI_TX_PHY_CTL_0),
            self.hd(HD_M_CTL),
            u32::from(vid & HD_VID_CTL_ENABLE != 0),
            u32::from(vid & HD_VID_CTL_UNDERFLOW_ENABLE != 0),
            self.hd(HD_CSC_CTL),
            self.frame_count()
        )
    }
}
