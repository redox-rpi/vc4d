// SPDX-License-Identifier: GPL-2.0-only
//
// G1 (redox-rpi#667): before vc4d waits for the vertical blank on an
// interrupt on the BCM2711, find out whether the firmware already uses the
// interrupts of the HVS and of pixel valve 2 (HDMI 0), and whether their
// lines reach the GIC. Read only: no enable bit is set, and every interrupt
// seen is acked at the GIC only.
//
// The registers are Linux v6.12 drivers/gpu/drm/vc4/vc4_regs.h:
// SCALER_DISPCTRL (enables, SCALER5_DISPCTRL_DSPEIEOF etc.),
// SCALER_DISPSTAT (status), PV_INTEN and PV_INTSTAT (PV_INT_VFP_START etc.).

use std::fs::OpenOptions;
use std::io::{Read, Write};
use std::os::unix::fs::OpenOptionsExt;
use std::thread;
use std::time::{Duration, Instant};

use fdt::Fdt;

const SCALER_DISPCTRL: usize = 0x00;
const SCALER_DISPSTAT: usize = 0x04;
const PV_CONTROL: usize = 0x00;
const PV_INTEN: usize = 0x24;
const PV_INTSTAT: usize = 0x28;

/// How long each line is watched.
const WATCH: Duration = Duration::from_secs(1);

fn cells(raw: &[u8]) -> Vec<u32> {
    raw.chunks_exact(4).map(|c| u32::from_be_bytes([c[0], c[1], c[2], c[3]])).collect()
}

/// The register base (CPU physical) and the /scheme/irq path of the first
/// node with `compatible`. The firmware's devicetree has these nodes, with
/// status "disabled".
pub fn node(fdt: &Fdt, compatible: &str) -> Option<(usize, String)> {
    let node = fdt.find_compatible(&[compatible])?;
    let bus = node.reg()?.next()?.starting_address as usize;
    let parent = [Some(node), fdt.find_node("/soc"), fdt.find_node("/")]
        .into_iter()
        .flatten()
        .find_map(|n| cells(n.property("interrupt-parent")?.value).first().copied())?;
    let gic = fdt.find_phandle(parent)?;
    let count = *cells(gic.property("#interrupt-cells")?.value).first()? as usize;
    let spec = cells(node.property("interrupts")?.value);
    let spec: Vec<String> = spec.get(..count)?.iter().map(u32::to_string).collect();
    Some((crate::soc_phys(fdt, bus), format!("/scheme/irq/phandle-{parent}/{}", spec.join(","))))
}

/// Reads 32-bit registers of a block, mapped for this call only.
fn read_regs(phys: usize, offsets: &[usize]) -> Option<Vec<u32>> {
    let regs = unsafe { common::physmap(phys, 0x1000, common::Prot::RO, common::MemoryType::DeviceMemory) }.ok()?
        as *const u32;
    let values = offsets.iter().map(|&o| unsafe { regs.add(o / 4).read_volatile() }).collect();
    let _ = unsafe { syscall::funmap(regs as usize, 0x1000) };
    Some(values)
}

/// Counts the interrupts on one line for WATCH, acking each at the GIC.
fn count(path: &str) -> Result<u32, String> {
    let mut file = OpenOptions::new()
        .read(true)
        .write(true)
        // /scheme/irq/phandle-... opens only with O_CREAT (Pi test 606).
        .create(true)
        .custom_flags(syscall::O_NONBLOCK as i32)
        .open(path)
        .map_err(|err| format!("{path}: {err}"))?;
    let start = Instant::now();
    let mut seen = 0;
    while start.elapsed() < WATCH {
        let mut token = [0u8; 8];
        if let Ok(8) = file.read(&mut token) {
            seen += 1;
            let _ = file.write(&token);
        }
        thread::sleep(Duration::from_millis(2));
    }
    Ok(seen)
}

pub fn run(fdt: &Fdt) {
    for (name, compatible, offsets, what) in [
        ("hvs", "brcm,bcm2711-hvs", [SCALER_DISPCTRL, SCALER_DISPSTAT, SCALER_DISPSTAT], "DISPCTRL DISPSTAT DISPSTAT"),
        ("pv2", "brcm,bcm2711-pixelvalve2", [PV_CONTROL, PV_INTEN, PV_INTSTAT], "CONTROL INTEN INTSTAT"),
    ] {
        let Some((phys, path)) = node(fdt, compatible) else {
            println!("vc4d: irq probe: {name}: no {compatible} node with interrupts");
            continue;
        };
        let before = read_regs(phys, &offsets);
        let seen = count(&path);
        let after = read_regs(phys, &offsets);
        println!(
            "vc4d: irq probe: {name} at {phys:#x}, {what} before {:08x?} after {:08x?}, {path}: {}",
            before.unwrap_or_default(),
            after.unwrap_or_default(),
            match seen {
                Ok(n) => format!("{n} interrupts in {} ms", WATCH.as_millis()),
                Err(err) => err,
            }
        );
    }
}
