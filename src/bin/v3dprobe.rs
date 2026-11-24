// SPDX-License-Identifier: GPL-2.0-only
//
// v3dprobe: step 0 of V3D on the Pi 4 and 400 (BCM2711, V3D 4.2). It powers
// the 3D block up as Linux does and reads its identification registers.
// With --csd and --render it runs the step 1 and step 2 jobs.
//
// Translated from Linux v6.12:
// - drivers/pmdomain/bcm/bcm2835-power.c: bcm2835_asb_power_on() for
//   BCM2835_POWER_DOMAIN_GRAFX_V3D. It releases the V3D reset in PM_GRAFX,
//   then opens the async AXI bridges, master first, at the BCM2711's
//   "rpivid_asb" registers. Linux skips power_on on the BCM2711.
// - drivers/pmdomain/bcm/raspberrypi-power.c: on the BCM2711 the firmware
//   powers V3D (RPI_POWER_DOMAIN_V3D, SET_DOMAIN_STATE). The firmware
//   leaves it off at boot (Test 547), so v3dprobe asks rpi-firmwared
//   (/scheme/rpi-firmware/v3d_power) when GRAFX has no power.
// - drivers/gpu/drm/v3d/v3d_regs.h, v3d_drv.c: V3D_HUB_IDENT0..3,
//   V3D_CTL_IDENT0..2 and what v3d_platform_drm_probe() logs.
//
// v3dprobe [--csd] [--render [--triangle | --triangle-math] [--show SECONDS]]
//          [--off] [--no-watchdog]
// v3dprobe --via-v3dd [--loops N] [--render [--triangle | --triangle-math]]
//   --show: the rendered picture in an Orbital window for SECONDS (step 2c)
//   --render: step 2 (redox-rpi#596): bin and render control lists, see
//     v3d/render.rs; clear only, or with --triangle a red triangle whose
//     coordinates the CPU precomputes, with --triangle-math one whose
//     shaders compute the screen coordinates
//   --csd: step 1 (redox-rpi#595): MMU page table and one compute job, see
//     v3d/csd.rs
//   --off: close the bridges, assert the reset and power V3D off afterwards
//   --via-v3dd: step 3 (redox-rpi#598): the --csd job N times (default
//     100) through v3dd and /scheme/v3d; touches no registers itself.
//     With --render, the --render frame N times instead (step 3b).
//   --no-watchdog: do not arm the SoC watchdog around the V3D accesses
//
// The first V3D accesses can hang the bus (a block without power or clock).
// The Pi 400 in the lab has no remote power switch, so v3dprobe arms the
// PM watchdog (Linux's bcm2835_wdt) for WATCHDOG_SECS before it releases
// the reset and stops it after the identification: a hang resets the SoC
// instead of needing a person.

use std::env;
use std::fs;
use std::io::Write;
use std::thread;
use std::time::{Duration, Instant};

use fdt::Fdt;

#[path = "v3d/client.rs"]
mod client;
#[path = "v3d/csd.rs"]
mod csd;
#[path = "v3d/render.rs"]
mod render;

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
// drivers/watchdog/bcm2835_wdt.c
const PM_RSTC: usize = 0x1c;
const PM_RSTS: usize = 0x20;
const PM_WDOG: usize = 0x24;
const PM_WDOG_TIME_SET: u32 = 0x000f_ffff;
const PM_RSTC_WRCFG_CLR: u32 = 0xffff_ffcf;
const PM_RSTC_WRCFG_FULL_RESET: u32 = 0x0000_0020;
const PM_RSTC_RESET: u32 = 0x0000_0102;
const WATCHDOG_SECS: u32 = 10;

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
    let args: Vec<String> = env::args().collect();
    // Step 3 (redox-rpi#598): the CSD job through v3dd, no register access.
    if args.iter().any(|a| a == "--via-v3dd") {
        let loops = args.iter().position(|a| a == "--loops").and_then(|i| args.get(i + 1)?.parse().ok()).unwrap_or(100);
        let ok = if args.iter().any(|a| a == "--render") {
            let draw = if args.iter().any(|a| a == "--triangle-math") {
                render::Draw::MathShaders
            } else if args.iter().any(|a| a == "--triangle") {
                render::Draw::CopyShaders
            } else {
                render::Draw::Clear
            };
            render::via_v3dd(draw, loops)
        } else {
            csd::via_v3dd(loops)
        };
        std::process::exit(if ok { 0 } else { 1 });
    }
    common::init();
    let off = env::args().any(|a| a == "--off");
    let watchdog = !env::args().any(|a| a == "--no-watchdog");
    let run_csd = env::args().any(|a| a == "--csd");
    let run_render = env::args().any(|a| a == "--render");
    let draw = if env::args().any(|a| a == "--triangle-math") {
        render::Draw::MathShaders
    } else if env::args().any(|a| a == "--triangle") {
        render::Draw::CopyShaders
    } else {
        render::Draw::Clear
    };
    let show_secs: Option<u64> =
        args.iter().position(|a| a == "--show").map(|i| args.get(i + 1).and_then(|s| s.parse().ok()).unwrap_or(30));
    let mut picture = Vec::new();

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
    pm.show(PM_RSTS, "PM_RSTS");
    pm.show(PM_RSTC, "PM_RSTC");
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
    // Test 547: GRAFX shows no power. Test 550: the firmware's V3D domain
    // (raspberrypi-power, RPI_POWER_DOMAIN_V3D) switches on, but PM_GRAFX
    // stays 0x1000. Mainline Linux does not look at PM_GRAFX on the BCM2711
    // either: bcm2835_power_power_on() returns early there, and
    // bcm2835_asb_power_on() only releases the reset and opens the
    // bridges. So: firmware domain on, then the same as Linux.
    let mut powered = false;
    let mut grafx = grafx;
    if grafx & PM_POWOK == 0 {
        say(&format!("v3d_power: {}", read_v3d_power()));
        say("asking the firmware for V3D power (/scheme/rpi-firmware/v3d_power = on)");
        if let Err(err) = fs::write(V3D_POWER, "on") {
            fail(&format!("{V3D_POWER}: {err} (rpi-firmwared without v3d_power?)"));
        }
        powered = true;
        thread::sleep(Duration::from_millis(5));
        grafx = pm.show(PM_GRAFX, "PM_GRAFX");
        say(&format!(
            "after v3d_power on: v3d_power {}, PM_GRAFX power ok {} (not checked on the BCM2711, as in Linux)",
            read_v3d_power(),
            grafx & PM_POWOK != 0
        ));
    }

    if watchdog {
        // bcm2835_wdt_start()
        pm.write(PM_WDOG, PM_PASSWORD | ((WATCHDOG_SECS << 16) & PM_WDOG_TIME_SET));
        let cur = pm.read(PM_RSTC);
        pm.write(PM_RSTC, PM_PASSWORD | (cur & PM_RSTC_WRCFG_CLR) | PM_RSTC_WRCFG_FULL_RESET);
        say(&format!("watchdog armed: {WATCHDOG_SECS} s, then a full SoC reset if v3dprobe hangs"));
    }

    // bcm2835_asb_power_on(): reset released, then the bridges, master first.
    if grafx & PM_V3DRSTN == 0 {
        say("releasing the V3D reset (PM_GRAFX |= V3DRSTN)");
        pm.write(PM_GRAFX, PM_PASSWORD | grafx | PM_V3DRSTN);
        thread::sleep(Duration::from_millis(1));
        pm.show(PM_GRAFX, "PM_GRAFX");
    }
    for (reg, name) in [(ASB_V3D_M_CTRL, "master"), (ASB_V3D_S_CTRL, "slave")] {
        let v = asb.read(reg);
        say(&format!("opening the ASB {name} bridge ({v:#x})"));
        asb.write(reg, PM_PASSWORD | (v & !ASB_REQ_STOP));
        let start = Instant::now();
        while asb.read(reg) & ASB_ACK != 0 {
            if start.elapsed() > Duration::from_millis(10) {
                let now = asb.read(reg);
                // Undo, as Linux's error path: bridges closed, reset
                // asserted, domain off. V3D itself is not read.
                for r in [ASB_V3D_S_CTRL, ASB_V3D_M_CTRL] {
                    asb.write(r, PM_PASSWORD | asb.read(r) | ASB_REQ_STOP);
                }
                pm.write(PM_GRAFX, PM_PASSWORD | (pm.read(PM_GRAFX) & !PM_V3DRSTN));
                if powered {
                    let _ = fs::write(V3D_POWER, "off");
                }
                if watchdog {
                    pm.write(PM_RSTC, PM_PASSWORD | PM_RSTC_RESET);
                    say("watchdog stopped");
                }
                fail(&format!("ASB {name} bridge: no ack within 10 ms ({now:#x}): bridges closed, reset asserted, V3D power off; V3D not read"));
            }
        }
        say(&format!("  ASB {name}: {:#x} after {} us", asb.read(reg), start.elapsed().as_micros()));
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
    let stop_watchdog = || {
        if watchdog {
            // bcm2835_wdt_stop()
            pm.write(PM_RSTC, PM_PASSWORD | PM_RSTC_RESET);
            say(&format!("watchdog stopped (PM_RSTC {:#x})", pm.read(PM_RSTC)));
        }
    };
    if !run_csd && !run_render {
        stop_watchdog();
    }

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
    let mut ok = ver >= 41 && ncores >= 1;
    say(&format!("{}: V3D {ver} on the BCM2711", if ok { "PASS" } else { "FAIL" }));
    if run_csd || run_render {
        // The watchdog stays armed through the jobs: a GPU job that hangs
        // the bus resets the SoC.
        if ok && run_csd {
            ok = csd::run(&hub, &core);
        }
        if ok && run_render {
            (ok, picture) = render::run(&hub, &core, draw);
        }
        stop_watchdog();
    }
    if let (Some(seconds), false) = (show_secs, picture.is_empty()) {
        render::show(&picture, seconds);
    }

    if off {
        for (reg, name) in [(ASB_V3D_S_CTRL, "slave"), (ASB_V3D_M_CTRL, "master")] {
            asb.write(reg, PM_PASSWORD | asb.read(reg) | ASB_REQ_STOP);
            say(&format!("closed the ASB {name} bridge ({:#x})", asb.read(reg)));
        }
        pm.write(PM_GRAFX, PM_PASSWORD | (pm.read(PM_GRAFX) & !PM_V3DRSTN));
        say("V3D reset asserted again");
        if powered {
            match fs::write(V3D_POWER, "off") {
                Ok(()) => say(&format!("v3d_power off: {}, PM_GRAFX {:#x}", read_v3d_power(), pm.read(PM_GRAFX))),
                Err(err) => say(&format!("v3d_power off: {err}")),
            }
        }
    }
    std::process::exit(if ok { 0 } else { 1 });
}

const V3D_POWER: &str = "/scheme/rpi-firmware/v3d_power";

fn read_v3d_power() -> String {
    match fs::read_to_string(V3D_POWER) {
        Ok(state) => state.trim().to_owned(),
        Err(err) => format!("({err})"),
    }
}

fn say(line: &str) {
    println!("v3dprobe: {line}");
    let _ = std::io::stdout().flush();
}

fn fail(why: &str) -> ! {
    say(&format!("FAIL: {why}"));
    std::process::exit(1);
}
