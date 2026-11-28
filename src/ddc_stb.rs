// SPDX-License-Identifier: GPL-2.0-only
//
// EDID over HDMI DDC on the BCM2711 (Pi 4, Pi 400; redox-rpi#671 step 1).
// Each HDMI block of the BCM2711 has an I2C controller of its own
// ("brcm,bcm2711-hdmi-i2c", the set-top-box BSC that Linux's i2c-brcmstb
// drives), not the BCM283x's BSC2. Next to it sits the firmware's
// "auto-i2c" engine, which owns the bus until it is told to release it.
//
// A transfer: chip address, byte count, data format in CTL, then ENABLE in
// IIC_ENABLE. The controller sets INTRP when the transfer is done and
// NOACK when the device did not answer. With CTLHI.DATA_REG_SIZE set, each
// of the eight data registers holds four bytes (least significant first),
// so one transfer moves up to 32 bytes. As on the BCM283x, the offset is
// written in one transfer and the bytes are read in the next ones: EDID
// EEPROMs keep counting from the last offset.

use std::time::{Duration, Instant};

use fdt::Fdt;

const EDID_ADDR: u32 = 0x50;

// Registers of the "bsc" region (byte offsets).
const CHIP_ADDRESS: usize = 0x00;
const DATA_IN: usize = 0x04; // 8 words, bytes for the device
const CNT_REG: usize = 0x24;
const CTL_REG: usize = 0x28;
const IIC_ENABLE: usize = 0x2c;
const DATA_OUT: usize = 0x30; // 8 words, bytes from the device
const CTLHI_REG: usize = 0x50;

// CTL: data transfer format (bits 1:0), SCL select (bits 5:4), SCL / 4.
const CTL_DTF_WRITE: u32 = 0;
const CTL_DTF_READ: u32 = 1;
const CTL_SCL_390K: u32 = 1 << 4;
const CTL_DIV_CLK: u32 = 1 << 7;
// IIC_ENABLE bits.
const IIC_ENABLE_BIT: u32 = 1 << 0;
const IIC_INTRP: u32 = 1 << 1;
const IIC_NOACK: u32 = 1 << 2;
// CTLHI: four bytes per data register.
const CTLHI_DATA_REG_SIZE: u32 = 1 << 6;

// The "auto-i2c" region: writing RELEASE_BSC hands the bus to software.
const AUTOI2C_CTRL0: usize = 0x26c;
const AUTOI2C_RELEASE_BSC: u32 = 1 << 1;

const MAX_XFER: usize = 32;
const TIMEOUT: Duration = Duration::from_millis(200);

/// Maps `size` bytes at a physical address that need not be page aligned.
fn map(phys: usize, size: usize, what: &str) -> Result<*mut u32, String> {
    let base = phys & !0xfff;
    let len = (phys - base + size + 0xfff) & !0xfff;
    let regs = unsafe { common::physmap(base, len, common::Prot::RW, common::MemoryType::DeviceMemory) }
        .map_err(|err| format!("failed to map the {what} at {phys:#x}: {err:?}"))?;
    Ok(unsafe { (regs as *mut u8).add(phys - base) } as *mut u32)
}

pub struct StbDdc {
    regs: *mut u32,
    pub phys: usize,
    pub auto_phys: Option<usize>,
}

impl StbDdc {
    /// Maps HDMI0's DDC controller (the hdmi0 node's `ddc` phandle) and
    /// releases it from the firmware's auto-i2c engine.
    pub fn open(fdt: &Fdt) -> Result<StbDdc, String> {
        let hdmi = fdt
            .find_compatible(&["brcm,bcm2711-hdmi0"])
            .ok_or("no brcm,bcm2711-hdmi0 node in the devicetree")?;
        let phandle = hdmi.property("ddc").and_then(|p| p.as_usize()).ok_or("hdmi0 has no ddc property")? as u32;
        let node = fdt.find_phandle(phandle).ok_or("hdmi0's ddc phandle names no node")?;
        let mut reg = node.reg().ok_or("the DDC node has no reg")?;
        let bsc = reg.next().ok_or("the DDC node has no bsc region")?.starting_address as usize;
        let auto = reg.next().map(|r| r.starting_address as usize);

        let phys = crate::soc_phys(fdt, bsc);
        let regs = map(phys, 0x100, "DDC controller")?;
        let auto_phys = auto.map(|a| crate::soc_phys(fdt, a));
        if let Some(auto_phys) = auto_phys {
            let auto_regs = map(auto_phys, 0x300, "auto-i2c engine")?;
            unsafe { auto_regs.add(AUTOI2C_CTRL0 / 4).write_volatile(AUTOI2C_RELEASE_BSC) };
        }
        let ddc = StbDdc { regs, phys, auto_phys };
        ddc.write(IIC_ENABLE, 0);
        ddc.write(CTLHI_REG, CTLHI_DATA_REG_SIZE);
        Ok(ddc)
    }

    fn read(&self, reg: usize) -> u32 {
        unsafe { self.regs.add(reg / 4).read_volatile() }
    }

    fn write(&self, reg: usize, value: u32) {
        unsafe { self.regs.add(reg / 4).write_volatile(value) }
    }

    /// One transfer: `tx` to the device, or `rx.len()` bytes from it.
    fn transfer(&self, tx: &[u8], rx: Option<&mut [u8]>) -> Result<(), String> {
        let (len, read) = match &rx {
            Some(buf) => (buf.len(), true),
            None => (tx.len(), false),
        };
        assert!(len > 0 && len <= MAX_XFER);
        self.write(IIC_ENABLE, 0);
        self.write(CHIP_ADDRESS, (EDID_ADDR << 1) | u32::from(read));
        for (i, chunk) in tx.chunks(4).enumerate() {
            let word = chunk.iter().enumerate().fold(0u32, |w, (j, &b)| w | (u32::from(b) << (8 * j)));
            self.write(DATA_IN + 4 * i, word);
        }
        self.write(CNT_REG, len as u32);
        let dtf = if read { CTL_DTF_READ } else { CTL_DTF_WRITE };
        self.write(CTL_REG, CTL_SCL_390K | CTL_DIV_CLK | dtf);
        self.write(IIC_ENABLE, IIC_ENABLE_BIT);

        let start = Instant::now();
        let status = loop {
            let s = self.read(IIC_ENABLE);
            if s & IIC_INTRP != 0 {
                break s;
            }
            if start.elapsed() > TIMEOUT {
                self.write(IIC_ENABLE, 0);
                return Err(format!("timeout, IIC_ENABLE {s:#06x}"));
            }
            std::hint::spin_loop();
        };
        self.write(IIC_ENABLE, 0);
        if status & IIC_NOACK != 0 {
            return Err("no acknowledge (no monitor, or it does not answer at 0x50)".into());
        }
        if let Some(buf) = rx {
            for (i, chunk) in buf.chunks_mut(4).enumerate() {
                let word = self.read(DATA_OUT + 4 * i);
                for (j, b) in chunk.iter_mut().enumerate() {
                    *b = (word >> (8 * j)) as u8;
                }
            }
        }
        Ok(())
    }

    /// Reads `buf.len()` bytes of the EDID from `offset` (0 or 128).
    pub fn read_edid(&self, offset: u8, buf: &mut [u8]) -> Result<(), String> {
        self.transfer(&[offset], None).map_err(|e| format!("offset {offset}: {e}"))?;
        let mut at = usize::from(offset);
        for chunk in buf.chunks_mut(MAX_XFER) {
            self.transfer(&[], Some(chunk)).map_err(|e| format!("read at {at}: {e}"))?;
            at += chunk.len();
        }
        Ok(())
    }
}
