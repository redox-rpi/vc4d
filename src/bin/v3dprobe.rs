// SPDX-License-Identifier: GPL-2.0-only
//
// v3dprobe: step 0 of V3D on the Pi 4 and 400 (BCM2711, V3D 4.2). It powers
// the 3D block up as Linux does and reads its identification registers.
// Nothing is rendered yet.
//
// Translated from Linux v6.12:
// - drivers/pmdomain/bcm/bcm2835-power.c: bcm2835_asb_power_on() for
//   BCM2835_POWER_DOMAIN_GRAFX_V3D. It releases the V3D reset in PM_GRAFX,
//   then opens the async AXI bridges, master first, at the BCM2711's
//   "rpivid_asb" registers. The firmware has already powered GRAFX
//   (Linux skips power_on on the BCM2711).
// - drivers/gpu/drm/v3d/v3d_regs.h, v3d_drv.c: V3D_HUB_IDENT0..3,
//   V3D_CTL_IDENT0..2 and what v3d_platform_drm_probe() logs.
//
// v3dprobe [--off]   --off: close the bridges and assert the reset afterwards

use std::env;
use std::fs;
use std::io::Write;
use std::thread;
use std::time::{Duration, Instant};

use fdt::Fdt;

// The Pi 400's devicetree (bcm2711-rpi-400.dtb): /v3dbus/v3d@7ec04000 with
// reg-names hub and core0, and /v3dbus ranges 0x7c500000 -> 0xfc500000. The
// power manager (watchdog@7e100000, reg-names pm, asb, rpivid_asb) is in
// /soc (0x7e000000 -> 0xfe000000).
const HUB_PHYS: usize = 0xfec0_0000;
const CORE0_PHYS: usize = 0xfec0_4000;
const PM_PHYS: usize = 0xfe10_0000;
const RPIVID_ASB_PHYS: usize = 0xfec1_1000;

const PM_GRAFX: usize = 0x10c;
const PM_V3DRSTN: u32 = 1 << 6;
const PM_POWOK: u32 = 1 << 1;
const PM_POWUP: u32 = 1 << 0;
const PM_PASSWORD: u32 = 0x5a00_0000;

const ASB_V3D_S_CTRL: usize = 0x08;
const ASB_V3D_M_CTRL: usize = 0x0c;
const ASB_AXI_BRDG_ID: usize = 0x20;
const ASB_REQ_STOP: u32 = 1 << 0;
const ASB_ACK: u32 = 1 << 1;
const BCM2835_BRDG_ID: u32 = 0x6272_6467;

const V3D_HUB_IDENT0: usize = 0x08;
const V3D_HUB_IDENT1: usize = 0x0c;
const V3D_HUB_IDENT2: usize = 0x10;
const V3D_HUB_IDENT3: usize = 0x14;
const V3D_CTL_IDENT0: usize = 0x00;
const V3D_CTL_IDENT1: usize = 0x04;
const V3D_CTL_IDENT2: usize = 0x08;

struct Regs {
    base: *mut u32,
    name: &'static str,
}

impl Regs {
    fn map(phys: usize, size: usize, name: &'static str) -> Regs {
        say(&format!("map {name} at {phys:#x}"));
        let base = unsafe { common::physmap(phys, size, common::Prot::RW, common::MemoryType::DeviceMemory) }
            .unwrap_or_else(|err| fail(&format!("cannot map {name} at {phys:#x}: {err:?}")));
        Regs { base: base as *mut u32, name }
    }

    fn read(&self, offset: usize) -> u32 {
        unsafe { self.base.add(offset / 4).read_volatile() }
    }

    fn write(&self, offset: usize, value: u32) {
        unsafe { self.base.add(offset / 4).write_volatile(value) }
    }

    fn show(&self, offset: usize, what: &str) -> u32 {
        // Printed before the read too: if the read aborts (a block without
        // power or clock), the log says which one.
        say(&format!("read {} {offset:#05x} ({what})", self.name));
        let v = self.read(offset);
        say(&format!("  = {v:#010x}"));
        v
    }
}

fn main() {
    common::init();
    let off = env::args().any(|a| a == "--off");

    let dtb = fs::read("/scheme/kernel.dtb").unwrap_or_default();
    let fdt = Fdt::new(&dtb).ok();
    let v3d = fdt.as_ref().and_then(|f| f.find_compatible(&["brcm,2711-v3d", "brcm,7278-v3d"]));
    match &v3d {
        Some(node) => say(&format!(
            "devicetree: {} ({}), status {}",
            node.name,
            node.compatible().map(|c| c.first()).unwrap_or("?"),
            node.property("status").and_then(|p| p.as_str()).unwrap_or("okay")
        )),
        None if fdt.as_ref().is_some_and(|f| f.root().compatible().all().any(|c| c == "brcm,bcm2711")) => {
            say("devicetree: no v3d node (an upstream devicetree), but brcm,bcm2711: going on with the Pi 400 addresses")
        }
        None => fail("not a BCM2711 (no brcm,2711-v3d, no brcm,bcm2711 in the devicetree)"),
    }
    // The firmware's clocks (rpi-firmwared): V3D should run.
    match fs::read_to_string("/scheme/rpi-firmware/clocks") {
        Ok(clocks) => {
            for line in clocks.lines() {
                say(&format!("clock: {line}"));
            }
        }
        Err(err) => say(&format!("clocks: /scheme/rpi-firmware/clocks: {err}")),
    }

    let pm = Regs::map(PM_PHYS, 0x1000, "pm");
    let asb = Regs::map(RPIVID_ASB_PHYS, 0x1000, "rpivid_asb");
    let grafx = pm.show(PM_GRAFX, "PM_GRAFX");
    say(&format!(
        "PM_GRAFX: power up {}, power ok {}, V3D reset {}",
        grafx & PM_POWUP != 0,
        grafx & PM_POWOK != 0,
        if grafx & PM_V3DRSTN != 0 { "released" } else { "asserted" }
    ));
    let id = asb.show(ASB_AXI_BRDG_ID, "ASB_AXI_BRDG_ID");
    if id != BCM2835_BRDG_ID {
        fail(&format!("bridge id {id:#x}, not \"brdg\" ({BCM2835_BRDG_ID:#x}): wrong address"));
    }
    asb.show(ASB_V3D_M_CTRL, "ASB_V3D_M_CTRL");
    asb.show(ASB_V3D_S_CTRL, "ASB_V3D_S_CTRL");
    if grafx & PM_POWOK == 0 {
        fail("GRAFX has no power (the firmware did not power it): not touching V3D");
    }

    // bcm2835_asb_power_on(): reset released, then the bridges, master first.
    if grafx & PM_V3DRSTN == 0 {
        say("releasing the V3D reset (PM_GRAFX |= V3DRSTN)");
        pm.write(PM_GRAFX, PM_PASSWORD | grafx | PM_V3DRSTN);
        thread::sleep(Duration::from_millis(1));
    }
    for (reg, name) in [(ASB_V3D_M_CTRL, "master"), (ASB_V3D_S_CTRL, "slave")] {
        let v = asb.read(reg);
        say(&format!("opening the ASB {name} bridge ({v:#x})"));
        asb.write(reg, PM_PASSWORD | (v & !ASB_REQ_STOP));
        let start = Instant::now();
        while asb.read(reg) & ASB_ACK != 0 {
            if start.elapsed() > Duration::from_millis(10) {
                fail(&format!("ASB {name} bridge: no ack within 10 ms ({:#x})", asb.read(reg)));
            }
        }
        say(&format!("  ASB {name}: {:#x}", asb.read(reg)));
    }

    let hub = Regs::map(HUB_PHYS, 0x4000, "hub");
    let core = Regs::map(CORE0_PHYS, 0x4000, "core0");
    let h0 = hub.show(V3D_HUB_IDENT0, "V3D_HUB_IDENT0");
    let h1 = hub.show(V3D_HUB_IDENT1, "V3D_HUB_IDENT1");
    let h2 = hub.show(V3D_HUB_IDENT2, "V3D_HUB_IDENT2");
    let h3 = hub.show(V3D_HUB_IDENT3, "V3D_HUB_IDENT3");
    let c0 = core.show(V3D_CTL_IDENT0, "V3D_CTL_IDENT0");
    let c1 = core.show(V3D_CTL_IDENT1, "V3D_CTL_IDENT1");
    let c2 = core.show(V3D_CTL_IDENT2, "V3D_CTL_IDENT2");

    // As v3d_platform_drm_probe() and v3d_debugfs_ident() decode them.
    let tver = h1 & 0xf;
    let rev = (h1 >> 4) & 0xf;
    let ver = tver * 10 + rev;
    let ncores = (h1 >> 8) & 0xf;
    let magic = [(c0 & 0xff) as u8, ((c0 >> 8) & 0xff) as u8, ((c0 >> 16) & 0xff) as u8];
    say(&format!("hub: ident0 {h0:#x}, V3D {}.{}, cores {ncores}, MMU {}, L3C {} KiB, TFU {}, IP rev {} idx {}",
        tver, rev, h2 & (1 << 8) != 0, h2 & 0xff, h1 & (1 << 17) != 0, (h3 >> 8) & 0xff, h3 & 0xff));
    say(&format!(
        "core0: \"{}\" version {}, VPM {} KiB, slices {}, QPUs per slice {}, TMUs per slice {}",
        String::from_utf8_lossy(&magic),
        c0 >> 24,
        (c1 >> 28) & 0xf,
        (c1 >> 4) & 0xf,
        (c1 >> 8) & 0xf,
        (c1 >> 12) & 0xf,
    ));
    let _ = c2;
    // Linux checks only the version and the cores; the "V3D" in ident0's
    // low bytes is logged for comparison.
    let ok = ver >= 41 && ncores >= 1;
    say(&format!("{}: V3D {ver} on the BCM2711", if ok { "PASS" } else { "FAIL" }));

    if off {
        for (reg, name) in [(ASB_V3D_S_CTRL, "slave"), (ASB_V3D_M_CTRL, "master")] {
            asb.write(reg, PM_PASSWORD | asb.read(reg) | ASB_REQ_STOP);
            say(&format!("closed the ASB {name} bridge ({:#x})", asb.read(reg)));
        }
        pm.write(PM_GRAFX, PM_PASSWORD | (pm.read(PM_GRAFX) & !PM_V3DRSTN));
        say("V3D reset asserted again");
    }
    std::process::exit(if ok { 0 } else { 1 });
}

fn say(line: &str) {
    println!("v3dprobe: {line}");
    let _ = std::io::stdout().flush();
}

fn fail(why: &str) -> ! {
    say(&format!("FAIL: {why}"));
    std::process::exit(1);
}
