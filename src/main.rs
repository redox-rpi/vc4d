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
// VC4D_EDID=off skips reading the monitor's EDID over DDC (ddc.rs), which
// runs first, also without an HVS (QEMU with an i2c-ddc device).
// VC4D_MODE=write lets vc4d program pixel valve 2 and the HDMI timing
// itself when its values equal the firmware's (mode.rs, stages 3a and 3b);
// without it vc4d only compares. VC4D_FLIP=off scans the client's buffer
// out directly, as before (adapter.rs, Flip): without it the primary
// plane is double buffered.

use std::env;
use std::thread;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::time::Duration;

use driver_graphics::GraphicsScheme;
use event::{user_data, EventQueue};
use fdt::Fdt;

use crate::adapter::Vc4Adapter;
use crate::hvs::{Hvs, CHANNELS, HVS_BUS_ADDR, HVS_SIZE};

mod adapter;
mod check;
mod ddc;
mod mode;
use vc4_hvs as hvs;
mod shot;
mod trace;

/// Bus address minus physical address of RAM when the devicetree has no
/// dma-ranges: the BCM283x's uncached alias.
const RAM_BUS_OFFSET: usize = 0xc000_0000;

/// How often a frame waiting for the HVS is looked at (a quarter frame).
const FLUSH_TICK: Duration = Duration::from_millis(4);

/// Arms the timer `after` from now; false if the clock cannot be read.
fn arm(time: &mut std::fs::File, after: Duration) -> bool {
    use std::io::{Read, Write};
    let mut now = syscall::TimeSpec::default();
    if time.read(&mut now).is_err() {
        return false;
    }
    let nanos = now.tv_nsec as i64 + after.as_nanos() as i64;
    now.tv_sec += nanos / 1_000_000_000;
    now.tv_nsec = (nanos % 1_000_000_000) as i32;
    time.write(&now).is_ok()
}

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
    let edid = if env::var("VC4D_EDID").is_ok_and(|v| v == "off") {
        None
    } else {
        ddc::log_edid(fdt.as_ref())
    };
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
    // The firmware's display runs: compare its pixel valve 2 with what
    // vc4d would write for the EDID's preferred mode (read only).
    if let (Some(edid), None) = (&edid, fake) {
        let same = mode::compare(fdt.as_ref(), edid);
        if env::var("VC4D_MODE").is_ok_and(|v| v == "write") {
            match same {
                Some(same) => {
                    mode::write_mode(fdt.as_ref(), &same);
                }
                None => println!("vc4d: mode: write: skipped, pixel valve 2 differs from the firmware's"),
            }
        }
    }

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
    if let Some(path) = trace::path() {
        trace::start(&path, hvs.share(), hvs.share(), channel.index);
    }
    let shown = shot::Shown::default();

    let flip_off = env::var("VC4D_FLIP").is_ok_and(|v| v == "off")
        || std::path::Path::new("/etc/vc4d-flip-off").exists();
    let flip = if flip_off {
        println!("vc4d: flip: off (VC4D_FLIP=off or /etc/vc4d-flip-off), clients' buffers scanned out directly");
        None
    } else {
        let flip = adapter::Flip::new(channel.width, channel.height);
        println!(
            "vc4d: flip: {}",
            if flip.is_some() { "two scanout buffers, damage copied" } else { "no contiguous memory for two buffers, off" }
        );
        flip
    };
    let adapter = Vc4Adapter {
        firmware_list: hvs.list_start(channel.index),
        hvs,
        channel,
        ram_bus_offset,
        next_slot: 0,
        list_frame: None,
        list_pending: false,
        primary: None,
        cursor: None,
        overlays: Vec::new(),
        blanked: false,
        log_lists: fake.is_some() || env::var("VC4D_LOG").is_ok(),
        lists: 0,
        shown: shown.clone(),
        flip,
        fake: fake.is_some(),
    };
    let mut scheme = GraphicsScheme::new(adapter, "display.vc4".to_owned(), false);

    user_data! {
        enum Source {
            Scheme,
            Time,
        }
    }
    let event_queue: EventQueue<Source> =
        EventQueue::new().expect("vc4d: failed to create event queue");
    event_queue
        .subscribe(scheme.event_handle().raw(), Source::Scheme, event::EventFlags::READ)
        .unwrap();
    // A frame that waits for the HVS (flip) is shown from this timer.
    let mut time = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(syscall::O_NONBLOCK as i32)
        .open(format!("/scheme/time/{}", syscall::CLOCK_MONOTONIC))
        .expect("vc4d: failed to open /scheme/time");
    event_queue
        .subscribe(time.as_raw_fd() as usize, Source::Time, event::EventFlags::READ)
        .unwrap();
    let mut armed = false;

    daemon.ready();
    check::watch(watched, channel, fake.is_some(), checking, shown);

    let all = [Source::Scheme];
    for event in all.into_iter().chain(event_queue.filter_map(|e| match e {
        Ok(e) => Some(e.user_data),
        Err(err) if err.is_interrupt() => None,
        Err(err) => panic!("vc4d: failed to get next event: {err}"),
    })) {
        match event {
            Source::Scheme => scheme.tick().expect("vc4d: failed to handle scheme events"),
            Source::Time => {
                armed = false;
                scheme.adapter_mut().flush();
            }
        }
        if !armed && scheme.adapter_mut().flip_pending() {
            armed = arm(&mut time, FLUSH_TICK);
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
