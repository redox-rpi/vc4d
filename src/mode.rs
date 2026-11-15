// SPDX-License-Identifier: GPL-2.0-only
//
// Stage 2c of redox-rpi#188, read only: the pixel valve 2 registers vc4d
// would write for the monitor's preferred mode (crate vc4-mode), next to
// what the firmware wrote. Nothing is written yet; when they agree, vc4d's
// mode setting can start from the same values.

use fdt::Fdt;

/// Bus address of pixel valve 2 (HDMI) on the BCM2835/6/7.
const PV2_BUS_ADDR: usize = 0x7e80_7000;
const PV_SIZE: usize = 0x1000;

pub fn compare(fdt: Option<&Fdt>, edid: &vc4_edid::Edid) {
    let Some(t) = edid.preferred() else {
        println!("vc4d: mode: no preferred mode in the EDID");
        return;
    };
    let pv = match vc4_mode::pv2(t) {
        Ok(pv) => pv,
        Err(err) => {
            println!("vc4d: mode: preferred mode not supported: {err:?}");
            return;
        }
    };
    let phys = fdt.map_or(PV2_BUS_ADDR, |fdt| crate::soc_phys(fdt, PV2_BUS_ADDR));
    let regs = match unsafe { common::physmap(phys, PV_SIZE, common::Prot::RO, common::MemoryType::DeviceMemory) } {
        Ok(regs) => regs as *const u32,
        Err(err) => {
            println!("vc4d: mode: failed to map pixel valve 2 at {phys:#x}: {err:?}");
            return;
        }
    };
    let read = |offset: usize| unsafe { regs.add(offset / 4).read_volatile() };
    for (offset, name, value) in pv.regs() {
        let firmware = read(offset);
        println!(
            "vc4d: mode: pv2 {name:<9} ours {value:#010x} firmware {firmware:#010x}{}",
            if value == firmware { "" } else { "  DIFFERENT" }
        );
    }
    let diff = vc4_mode::differences(&pv.regs(), read);
    let mode = format!("{}x{}@{}", t.hactive, t.vactive, t.refresh_mhz() / 1000);
    if diff.is_empty() {
        println!("vc4d: mode: PASS: pixel valve 2 values for {mode} equal the firmware's");
    } else {
        let names: Vec<&str> = diff.iter().map(|d| d.0).collect();
        println!("vc4d: mode: DIFFERENT: pixel valve 2 for {mode}: {} (the firmware may have picked another mode)", names.join(" "));
    }
}
