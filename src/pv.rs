// SPDX-License-Identifier: GPL-2.0-only
//
// The pixel valve that feeds HDMI (PV2 on the BCM2835 and BCM2837), read
// only: the timing the firmware set, its interrupt status, and how many
// pixels the last line really got from the HVS. For the noise bands of Pi
// test #275: whether full lines leave the HVS.
//
// Register offsets and fields translated from Linux v6.12,
// drivers/gpu/drm/vc4/vc4_regs.h (PV_*). The sampling is our own code.

use std::ptr;

/// Bus address of PV2 if the devicetree has none.
pub const PV2_BUS_ADDR: usize = 0x7e80_7000;
pub const PV_SIZE: usize = 0x100;

const PV_CONTROL: usize = 0x00;
const PV_V_CONTROL: usize = 0x04;
const PV_HORZA: usize = 0x0c;
const PV_HORZB: usize = 0x10;
const PV_VERTA: usize = 0x14;
const PV_VERTB: usize = 0x18;
const PV_INTEN: usize = 0x24;
const PV_INTSTAT: usize = 0x28;
const PV_STAT: usize = 0x2c;
const PV_HACT_ACT: usize = 0x30;

fn hi(reg: u32) -> u32 {
    reg >> 16
}

fn lo(reg: u32) -> u32 {
    reg & 0xffff
}

pub struct PixelValve {
    regs: *mut u32,
}

// Device registers, only read.
unsafe impl Send for PixelValve {}

impl PixelValve {
    /// # Safety
    /// `regs` maps the pixel valve's PV_SIZE bytes of registers.
    pub unsafe fn new(regs: *mut u32) -> Self {
        PixelValve { regs }
    }

    fn reg(&self, offset: usize) -> u32 {
        unsafe { ptr::read_volatile(self.regs.add(offset / 4)) }
    }

    /// Active pixels per line as programmed.
    pub fn hactive(&self) -> u32 {
        lo(self.reg(PV_HORZB))
    }

    /// PV_HACT_ACT: Linux defines it but never reads it; most likely the
    /// pixels the last line really got.
    pub fn hact_act(&self) -> u32 {
        self.reg(PV_HACT_ACT)
    }

    pub fn stat(&self) -> u32 {
        self.reg(PV_STAT)
    }

    pub fn intstat(&self) -> u32 {
        self.reg(PV_INTSTAT)
    }

    /// The registers and the mode they give, for the log.
    pub fn describe(&self) -> String {
        let control = self.reg(PV_CONTROL);
        let (horza, horzb) = (self.reg(PV_HORZA), self.reg(PV_HORZB));
        let (verta, vertb) = (self.reg(PV_VERTA), self.reg(PV_VERTB));
        let htotal = lo(horza) + hi(horza) + lo(horzb) + hi(horzb);
        let vtotal = lo(verta) + hi(verta) + lo(vertb) + hi(vertb);
        format!(
            "control {control:#010x} (en {}, fifo level {}, underflow trigger {}, clock {}) vcontrol {:#010x} \
             h sync {} bp {} active {} fp {} (total {htotal}) v sync {} bp {} active {} fp {} (total {vtotal}) \
             inten {:#06x} intstat {:#06x} stat {:#010x} hact_act {}",
            control & 1,
            (control >> 15) & 0x3f,
            (control >> 13) & 1,
            (control >> 2) & 3,
            self.reg(PV_V_CONTROL),
            lo(horza),
            hi(horza),
            lo(horzb),
            hi(horzb),
            lo(verta),
            hi(verta),
            lo(vertb),
            hi(vertb),
            self.reg(PV_INTEN),
            self.intstat(),
            self.stat(),
            self.hact_act()
        )
    }
}
