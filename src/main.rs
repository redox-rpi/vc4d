// SPDX-License-Identifier: GPL-2.0-only
//
// vc4d: display driver for the VideoCore IV (Raspberry Pi 3, Zero 2 W),
// stage 1 of redox-rpi#188: HVS planes.
//
// The firmware still sets up HDMI, the pixel valve and the mode at
// power-on, and leaves one HVS channel running with its own display list
// (its framebuffer). vc4d takes that channel over: it writes its own
// display list with a primary plane that scans out the client's dumb
// buffer directly, and a 64x64 cursor plane, so moving the pointer only
// changes one word of the list. It registers as a display like rpi-fbd, so
// fbcond and Orbital use it unchanged.
//
// Translated from Linux v6.12 drivers/gpu/drm/vc4/ (see hvs.rs and
// adapter.rs for the files). soc_phys() and ram_bus_offset() come from
// redox-rpi's rpi-fbd (MIT).
//
// Environment: VC4D=off leaves the display to vesad or rpi-fbd.
// VC4D_FAKE=WIDTHxHEIGHT runs against an HVS in memory (QEMU has none):
// everything but the picture. VC4D_LOG=1 logs every new display list (the
// first ones are always logged). VC4D_CHECK=off skips the check without a
// screen (check.rs) that runs before the handover and watches after it.

use std::env;
use std::thread;
use std::time::Duration;

use driver_graphics::GraphicsScheme;
use event::{user_data, EventQueue};
use fdt::Fdt;

use crate::adapter::Vc4Adapter;
use crate::hvs::{Hvs, CHANNELS, HVS_BUS_ADDR, HVS_SIZE};

mod adapter;
mod check;
#[allow(dead_code)]
mod hvs;
mod hdmi;
mod pv;
mod shot;

/// Bus address minus physical address of RAM when the devicetree has no
/// dma-ranges: the BCM283x's uncached alias.
const RAM_BUS_OFFSET: usize = 0xc000_0000;

fn main() {
    common::init();
    daemon::Daemon::new(daemon);
}

/// Ends the daemon without a display; the console stays where it is.
fn give_up(daemon: daemon::Daemon, why: &str) -> ! {
    println!("vc4d: {why}, display left as it is");
    daemon.ready();
    std::process::exit(0);
}

fn daemon(daemon: daemon::Daemon) -> ! {
    if env::var("VC4D").is_ok_and(|v| v == "off") {
        give_up(daemon, "off (VC4D=off)");
    }

    let dtb = std::fs::read("/scheme/kernel.dtb").unwrap_or_default();
    let fdt = Fdt::new(&dtb).ok();
    let bus_addr = fdt
        .as_ref()
        .and_then(|fdt| fdt.find_compatible(&["brcm,bcm2835-hvs"]))
        .and_then(|node| node.reg())
        .and_then(|mut reg| reg.next())
        .map(|reg| reg.starting_address as usize)
        .unwrap_or(HVS_BUS_ADDR);
    let phys = fdt.as_ref().map_or(bus_addr, |fdt| soc_phys(fdt, bus_addr));
    let ram_bus_offset = fdt.as_ref().map_or(RAM_BUS_OFFSET, ram_bus_offset);

    let fake = env::var("VC4D_FAKE").ok().and_then(|v| parse_size(&v));
    let hvs = match fake {
        Some((width, height)) => {
            println!("vc4d: VC4D_FAKE: an HVS in memory at {width}x{height}, nothing on the screen");
            Hvs::fake(width, height)
        }
        None => {
            let regs = match unsafe {
                common::physmap(phys, HVS_SIZE, common::Prot::RW, common::MemoryType::DeviceMemory)
            } {
                Ok(regs) => regs as *mut u32,
                Err(err) => give_up(daemon, &format!("failed to map the HVS at {phys:#x}: {err:?}")),
            };
            unsafe { Hvs::new(regs) }
        }
    };
    println!(
        "vc4d: HVS at {phys:#x}, id {:#010x}, {}",
        hvs.id(),
        if hvs.enabled() { "enabled" } else { "disabled" }
    );
    if !hvs.enabled() {
        give_up(daemon, "the firmware left the HVS off (or this is QEMU, which has none)");
    }

    // The pixel valve behind HVS channel 1 (HDMI), read only: what the
    // HVS hands on (Pi test #275).
    let pv = match (&fake, fdt.as_ref()) {
        (None, Some(fdt)) => {
            let bus = fdt
                .find_compatible(&["brcm,bcm2835-pixelvalve2"])
                .and_then(|node| node.reg())
                .and_then(|mut reg| reg.next())
                .map_or(pv::PV2_BUS_ADDR, |reg| reg.starting_address as usize);
            let pv_phys = soc_phys(fdt, bus);
            match unsafe { common::physmap(pv_phys, pv::PV_SIZE, common::Prot::RW, common::MemoryType::DeviceMemory) } {
                Ok(regs) => {
                    let pv = unsafe { pv::PixelValve::new(regs as *mut u32) };
                    println!("vc4d: pv2 at {pv_phys:#x}: {}", pv.describe());
                    Some(pv)
                }
                Err(err) => {
                    println!("vc4d: pv2: cannot map {pv_phys:#x}: {err:?}");
                    None
                }
            }
        }
        _ => None,
    };
    // The HDMI encoder, read only (stage 2b, #188): its two register blocks.
    let hdmi = match (&fake, fdt.as_ref()) {
        (None, Some(fdt)) => {
            let mut regs = fdt
                .find_compatible(&["brcm,bcm2835-hdmi"])
                .and_then(|node| node.reg())
                .map(|regs| regs.map(|r| r.starting_address as usize).collect::<Vec<_>>())
                .unwrap_or_default()
                .into_iter();
            let core_bus = regs.next().unwrap_or(hdmi::HDMI_BUS_ADDR);
            let hd_bus = regs.next().unwrap_or(hdmi::HD_BUS_ADDR);
            let (core_phys, hd_phys) = (soc_phys(fdt, core_bus), soc_phys(fdt, hd_bus));
            let map = |phys, size| unsafe { common::physmap(phys, size, common::Prot::RW, common::MemoryType::DeviceMemory) };
            match (map(core_phys, hdmi::HDMI_SIZE), map(hd_phys, hdmi::HD_SIZE)) {
                (Ok(core), Ok(hd)) => {
                    let hdmi = unsafe { hdmi::Hdmi::new(core as *mut u32, hd as *mut u32) };
                    println!("vc4d: hdmi at {core_phys:#x} and {hd_phys:#x}: {}", hdmi.describe());
                    Some(hdmi)
                }
                (core, hd) => {
                    println!("vc4d: hdmi: cannot map {core_phys:#x} ({:?}) or {hd_phys:#x} ({:?})", core.err(), hd.err());
                    None
                }
            }
        }
        _ => None,
    };

    // The firmware's channels and lists, for the log: what the Pi tests
    // compare against.
    let mut found = None;
    println!("vc4d: {}", hvs.describe_global());
    for index in 0..CHANNELS {
        println!("vc4d: channel {index}: {}", hvs.describe(index));
        if let Some(channel) = hvs.channel(index) {
            let list = hvs.read_list(hvs.list_start(index), 32);
            println!("vc4d: channel {index}: {}x{}, list {list:08x?}", channel.width, channel.height);
            found.get_or_insert(channel);
        }
    }
    let Some(channel) = found else {
        give_up(daemon, "no HVS channel enabled");
    };
    // A channel the firmware set up but does not scan out would show
    // nothing; its frame count moves only when it runs.
    let before = hvs.frame_count(channel.index);
    thread::sleep(Duration::from_millis(50));
    if fake.is_none() && hvs.frame_count(channel.index) == before {
        give_up(daemon, &format!("channel {} does not run", channel.index));
    }
    println!(
        "vc4d: taking over channel {} at {}x{}",
        channel.index, channel.width, channel.height
    );

    let checking = !env::var("VC4D_CHECK").is_ok_and(|v| v == "off");
    if checking {
        let passed = check::self_check(&hvs, channel, ram_bus_offset, fake.is_some());
        println!("vc4d: check: {}", if passed { "PASSED" } else { "FAILED" });
    }
    let watched = hvs.share();
    let shown = shot::Shown::default();

    let adapter = Vc4Adapter {
        firmware_list: hvs.list_start(channel.index),
        hvs,
        channel,
        ram_bus_offset,
        next_slot: 0,
        primary: None,
        cursor: None,
        blanked: false,
        log_lists: fake.is_some() || env::var("VC4D_LOG").is_ok(),
        lists: 0,
        shown: shown.clone(),
    };
    let mut scheme = GraphicsScheme::new(adapter, "display.vc4".to_owned(), false);

    user_data! {
        enum Source {
            Scheme,
        }
    }
    let event_queue: EventQueue<Source> =
        EventQueue::new().expect("vc4d: failed to create event queue");
    event_queue
        .subscribe(scheme.event_handle().raw(), Source::Scheme, event::EventFlags::READ)
        .unwrap();

    daemon.ready();
    check::watch(watched, pv, hdmi, channel, fake.is_some(), checking, shown);

    let all = [Source::Scheme];
    for event in all.into_iter().chain(event_queue.filter_map(|e| match e {
        Ok(e) => Some(e.user_data),
        Err(err) if err.is_interrupt() => None,
        Err(err) => panic!("vc4d: failed to get next event: {err}"),
    })) {
        match event {
            Source::Scheme => scheme.tick().expect("vc4d: failed to handle scheme events"),
        }
    }

    panic!();
}

fn parse_size(value: &str) -> Option<(u32, u32)> {
    let (w, h) = value.split_once('x')?;
    let (w, h) = (w.parse().ok()?, h.parse().ok()?);
    ((1..=4096).contains(&w) && (1..=4096).contains(&h)).then_some((w, h))
}

/// The CPU physical address of a peripheral's bus address, through `/soc`'s
/// ranges.
fn soc_phys(fdt: &Fdt, bus_addr: usize) -> usize {
    fdt.find_node("/soc")
        .and_then(|soc| soc.ranges())
        .and_then(|mut ranges| {
            ranges.find(|r| {
                r.child_bus_address <= bus_addr && bus_addr - r.child_bus_address < r.size
            })
        })
        .map(|r| r.parent_bus_address + (bus_addr - r.child_bus_address))
        .unwrap_or(bus_addr)
}

/// Bus address minus physical address of RAM, from the first entry of
/// `/soc`'s dma-ranges (one cell each for bus address, physical address and
/// size on the BCM283x).
fn ram_bus_offset(fdt: &Fdt) -> usize {
    fdt.find_node("/soc")
        .and_then(|soc| soc.property("dma-ranges"))
        .filter(|prop| prop.value.len() >= 12)
        .map(|prop| {
            let cell = |i: usize| u32::from_be_bytes(prop.value[i * 4..i * 4 + 4].try_into().unwrap());
            (cell(0) as usize).wrapping_sub(cell(1) as usize)
        })
        .unwrap_or(RAM_BUS_OFFSET)
}
