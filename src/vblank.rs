// SPDX-License-Identifier: GPL-2.0-only
//
// G1 (redox-rpi#667): the vertical blank of HDMI 0 on the BCM2711 as an
// interrupt, so a waiting flip is shown at the next frame instead of on a
// 4 ms timer. The source is pixel valve 2's "vertical front porch start"
// (PV_INT_VFP_START), as Linux v6.12 vc4_crtc.c enables it for vblank
// (vc4_enable_vblank(), vc4_crtc_irq_handler()). The firmware leaves pixel
// valve 2's interrupts off (PV_INTEN 0, Pi test 606), so vc4d owns them.
// The HVS interrupts stay the firmware's.
//
// The interrupt is enabled only while a flip waits (Linux enables vblank
// on demand too), so an idle desktop takes none.

use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;

use fdt::Fdt;

const PV_INTEN: usize = 0x24;
const PV_INTSTAT: usize = 0x28;
const PV_INT_VFP_START: u32 = 1 << 7;

pub struct Vblank {
    regs: *mut u32,
    file: File,
    pub path: String,
    enabled: bool,
    pub count: u64,
}

impl Vblank {
    /// Pixel valve 2's registers and interrupt, from the devicetree.
    pub fn open(fdt: &Fdt) -> Result<Vblank, String> {
        let (phys, path) = crate::irqprobe::node(fdt, "brcm,bcm2711-pixelvalve2")
            .ok_or("no brcm,bcm2711-pixelvalve2 node with interrupts")?;
        let regs = unsafe { common::physmap(phys, 0x1000, common::Prot::RW, common::MemoryType::DeviceMemory) }
            .map_err(|err| format!("failed to map pixel valve 2 at {phys:#x}: {err:?}"))? as *mut u32;
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .custom_flags(syscall::O_NONBLOCK as i32)
            .open(&path)
            .map_err(|err| format!("{path}: {err}"))?;
        let vblank = Vblank { regs, file, path, enabled: false, count: 0 };
        vblank.write(PV_INTSTAT, PV_INT_VFP_START);
        Ok(vblank)
    }

    pub fn fd(&self) -> usize {
        self.file.as_raw_fd() as usize
    }

    fn read(&self, reg: usize) -> u32 {
        unsafe { self.regs.add(reg / 4).read_volatile() }
    }

    fn write(&self, reg: usize, value: u32) {
        unsafe { self.regs.add(reg / 4).write_volatile(value) }
    }

    /// Turns the interrupt on while a flip waits, off otherwise.
    pub fn want(&mut self, on: bool) {
        if on == self.enabled {
            return;
        }
        let inten = self.read(PV_INTEN);
        if on {
            self.write(PV_INTSTAT, PV_INT_VFP_START);
            self.write(PV_INTEN, inten | PV_INT_VFP_START);
        } else {
            self.write(PV_INTEN, inten & !PV_INT_VFP_START);
        }
        self.enabled = on;
    }

    /// One interrupt: clears it in the pixel valve (the line is level
    /// triggered), then acks it at the GIC. False when none was pending.
    pub fn handle(&mut self) -> bool {
        let mut token = [0u8; 8];
        if !matches!(self.file.read(&mut token), Ok(8)) {
            return false;
        }
        self.write(PV_INTSTAT, PV_INT_VFP_START);
        let _ = self.file.write(&token);
        self.count += 1;
        true
    }
}
