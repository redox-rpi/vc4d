// SPDX-License-Identifier: GPL-2.0-only
//
// Reading the monitor's EDID over HDMI DDC (stage 2b of redox-rpi#188).
// DDC is I2C: the EDID sits at address 0x50, one byte of offset, then
// 128-byte blocks. On the BCM283x the HDMI DDC lines go to a BSC (Broadcom
// Serial Controller) of their own, BSC2 at bus address 0x7e805000; the
// devicetree's hdmi node names it in its `ddc` property.
//
// BSC registers from the BCM2835 ARM peripherals manual (section 3); the
// code is our own. Two plain transfers (write the offset, then read) instead
// of a repeated start: the BSC has none, and EDID EEPROMs keep the offset
// between transfers.

use std::time::{Duration, Instant};

use fdt::Fdt;

/// Bus address of BSC2, the HDMI DDC controller, if the devicetree has none.
pub const DDC_BUS_ADDR: usize = 0x7e80_5000;
const MAP_SIZE: usize = 0x1000;
const EDID_ADDR: u32 = 0x50;

// Registers (byte offsets).
const C: usize = 0x00;
const S: usize = 0x04;
const DLEN: usize = 0x08;
const A: usize = 0x0c;
const FIFO: usize = 0x10;
const DIV: usize = 0x14;
const CLKT: usize = 0x1c;

// C bits.
const C_I2CEN: u32 = 1 << 15;
const C_ST: u32 = 1 << 7;
const C_CLEAR: u32 = 3 << 4;
const C_READ: u32 = 1 << 0;

// S bits.
const S_TA: u32 = 1 << 0;
const S_DONE: u32 = 1 << 1;
const S_TXD: u32 = 1 << 4;
const S_RXD: u32 = 1 << 5;
const S_ERR: u32 = 1 << 8;
const S_CLKT: u32 = 1 << 9;

/// SCL = core clock / DIV. 4000 gives 62.5 kHz at the 250 MHz core clock
/// and 100 kHz at 400 MHz, inside DDC's 100 kHz either way.
const DIVIDER: u32 = 4000;
const TIMEOUT: Duration = Duration::from_millis(200);

pub struct Ddc {
    regs: *mut u32,
    pub phys: usize,
}

impl Ddc {
    /// Maps the DDC controller the devicetree's hdmi node points at.
    pub fn open(fdt: Option<&Fdt>) -> Result<Ddc, String> {
        let bus = fdt.and_then(ddc_bus_addr).unwrap_or(DDC_BUS_ADDR);
        let phys = fdt.map_or(bus, |fdt| crate::soc_phys(fdt, bus));
        let regs = unsafe { common::physmap(phys, MAP_SIZE, common::Prot::RW, common::MemoryType::DeviceMemory) }
            .map_err(|err| format!("failed to map the DDC controller at {phys:#x}: {err:?}"))?;
        Ok(Ddc { regs: regs as *mut u32, phys })
    }

    fn read(&self, reg: usize) -> u32 {
        unsafe { self.regs.add(reg / 4).read_volatile() }
    }

    fn write(&self, reg: usize, value: u32) {
        unsafe { self.regs.add(reg / 4).write_volatile(value) }
    }

    /// Clears the FIFO and the sticky status bits, sets the clock.
    fn reset(&self) {
        self.write(C, C_I2CEN | C_CLEAR);
        self.write(S, S_CLKT | S_ERR | S_DONE);
        self.write(DIV, DIVIDER);
        // Clock stretching timeout in SCL cycles (reset value 0x40).
        self.write(CLKT, 0x40);
    }

    /// Waits for the transfer to end; reads bytes into `rx` as they come.
    fn finish(&self, mut rx: Option<&mut [u8]>) -> Result<usize, String> {
        let start = Instant::now();
        let mut got = 0;
        loop {
            let s = self.read(S);
            if let Some(buf) = rx.as_deref_mut() {
                while self.read(S) & S_RXD != 0 {
                    let byte = self.read(FIFO) as u8;
                    if got < buf.len() {
                        buf[got] = byte;
                        got += 1;
                    }
                }
            }
            if s & S_ERR != 0 {
                self.reset();
                return Err("no acknowledge (no monitor, or it does not answer at 0x50)".into());
            }
            if s & S_CLKT != 0 {
                self.reset();
                return Err("clock stretching timeout".into());
            }
            if s & S_DONE != 0 && s & S_TA == 0 {
                if let Some(buf) = rx {
                    while self.read(S) & S_RXD != 0 && got < buf.len() {
                        buf[got] = self.read(FIFO) as u8;
                        got += 1;
                    }
                }
                self.write(S, S_DONE);
                return Ok(got);
            }
            if start.elapsed() > TIMEOUT {
                let s = self.read(S);
                self.reset();
                return Err(format!("timeout, status {s:#06x}"));
            }
            std::hint::spin_loop();
        }
    }

    /// Reads `buf.len()` bytes of the EDID from `offset` (0 or 128: blocks
    /// beyond the second need the E-DDC segment pointer, not done yet).
    pub fn read_edid(&self, offset: u8, buf: &mut [u8]) -> Result<(), String> {
        self.reset();
        self.write(A, EDID_ADDR);
        // Write the offset.
        self.write(DLEN, 1);
        if self.read(S) & S_TXD != 0 {
            self.write(FIFO, u32::from(offset));
        }
        self.write(C, C_I2CEN | C_ST);
        self.finish(None).map_err(|e| format!("offset {offset}: {e}"))?;
        // Read the bytes.
        self.write(DLEN, buf.len() as u32);
        self.write(C, C_I2CEN | C_ST | C_CLEAR | C_READ);
        let got = self.finish(Some(buf)).map_err(|e| format!("read at {offset}: {e}"))?;
        if got != buf.len() {
            return Err(format!("read at {offset}: {got} of {} bytes", buf.len()));
        }
        Ok(())
    }
}

/// The bus address of the controller in the hdmi node's `ddc` property.
fn ddc_bus_addr(fdt: &Fdt) -> Option<usize> {
    let hdmi = fdt.find_compatible(&["brcm,bcm2835-hdmi"])?;
    let phandle = hdmi.property("ddc")?.as_usize()? as u32;
    let node = fdt.find_phandle(phandle)?;
    Some(node.reg()?.next()?.starting_address as usize)
}

/// Reads the EDID (base block and the first extension, if there is one)
/// and logs what the monitor says it can show, with the raw bytes.
pub fn log_edid(fdt: Option<&Fdt>) {
    let ddc = match Ddc::open(fdt) {
        Ok(ddc) => ddc,
        Err(err) => {
            println!("vc4d: edid: {err}");
            return;
        }
    };
    println!("vc4d: edid: DDC controller at {:#x}", ddc.phys);
    let mut data = vec![0u8; vc4_edid::BLOCK];
    if let Err(err) = ddc.read_edid(0, &mut data) {
        println!("vc4d: edid: FAIL: {err}");
        return;
    }
    if data[126] > 0 {
        let mut ext = vec![0u8; vc4_edid::BLOCK];
        match ddc.read_edid(128, &mut ext) {
            Ok(()) => data.extend_from_slice(&ext),
            Err(err) => println!("vc4d: edid: extension block: {err}"),
        }
    }
    for (i, line) in data.chunks(16).enumerate() {
        let hex: Vec<String> = line.iter().map(|b| format!("{b:02x}")).collect();
        println!("vc4d: edid: raw {:03x}: {}", i * 16, hex.join(" "));
    }
    let edid = match vc4_edid::Edid::parse(&data) {
        Ok(edid) => edid,
        Err(err) => {
            println!("vc4d: edid: FAIL: {err}");
            return;
        }
    };
    println!("vc4d: edid: PASS: {edid}");
    match edid.preferred() {
        Some(t) => println!("vc4d: edid: preferred {t}"),
        None => println!("vc4d: edid: no preferred mode"),
    }
    for t in edid.timings.iter().skip(1) {
        println!("vc4d: edid: detailed {t}");
    }
    if let Some(r) = edid.range {
        println!(
            "vc4d: edid: range {}-{} Hz vertical, {}-{} kHz horizontal, pixel clock up to {} MHz",
            r.vmin_hz, r.vmax_hz, r.hmin_khz, r.hmax_khz, r.max_pixel_mhz
        );
    }
    let list = |modes: &[vc4_edid::Mode]| modes.iter().map(|m| m.to_string()).collect::<Vec<_>>().join(" ");
    if !edid.established.is_empty() {
        println!("vc4d: edid: established {}", list(&edid.established));
    }
    if !edid.standard.is_empty() {
        println!("vc4d: edid: standard {}", list(&edid.standard));
    }
    if let Some(cea) = &edid.cea {
        let vics: Vec<String> = cea
            .vics
            .iter()
            .map(|&(vic, native)| {
                let mode = vc4_edid::vic_mode(vic).map_or(String::new(), |m| format!("={m}"));
                format!("{vic}{mode}{}", if native { "*" } else { "" })
            })
            .collect();
        println!(
            "vc4d: edid: CEA-861 rev {}, {}{}{}, video codes {}",
            cea.revision,
            if cea.hdmi { "HDMI" } else { "DVI" },
            if cea.audio { ", audio" } else { "" },
            if cea.underscan { ", underscan" } else { "" },
            vics.join(" ")
        );
        for t in &cea.timings {
            println!("vc4d: edid: CEA detailed {t}");
        }
    }
}
