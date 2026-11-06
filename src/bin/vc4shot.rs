// SPDX-License-Identifier: GPL-2.0-only
//
// vc4shot: a screenshot of what the HVS scans out, for tests without a
// person. It lists the planes of the enabled channel's display list
// (read-only, nothing is written to the HVS), then asks vc4d for the
// picture of the buffers it shows (shot.rs), cursor alpha included.
//
// vc4shot [file.ppm] [channel]   (default /tmp/vc4shot.ppm, the first
// enabled channel)

use std::env;
use std::fs;
use std::thread;
use std::time::{Duration, Instant};

use fdt::Fdt;

#[allow(dead_code)]
#[path = "../hvs.rs"]
mod hvs;

/// As in shot.rs (vc4d answers the request).
mod shot {
    pub const REQUEST: &str = "/tmp/vc4shot.request";
    pub const PICTURE: &str = "/tmp/vc4shot.ppm";
}

use hvs::{Hvs, CHANNELS, HVS_BUS_ADDR, HVS_SIZE};


fn main() {
    common::init();
    let args: Vec<String> = env::args().skip(1).collect();
    let out = args.first().cloned().unwrap_or_else(|| "/tmp/vc4shot.ppm".to_owned());
    let wanted: Option<usize> = args.get(1).and_then(|a| a.parse().ok());

    let dtb = fs::read("/scheme/kernel.dtb").unwrap_or_default();
    let fdt = Fdt::new(&dtb).ok();
    let bus_addr = fdt
        .as_ref()
        .and_then(|fdt| fdt.find_compatible(&["brcm,bcm2835-hvs"]))
        .and_then(|node| node.reg())
        .and_then(|mut reg| reg.next())
        .map(|reg| reg.starting_address as usize)
        .unwrap_or(HVS_BUS_ADDR);
    let phys = fdt.as_ref().map_or(bus_addr, |fdt| soc_phys(fdt, bus_addr));

    let regs = unsafe { common::physmap(phys, HVS_SIZE, common::Prot::RO, common::MemoryType::DeviceMemory) }
        .unwrap_or_else(|err| fail(&format!("cannot map the HVS at {phys:#x}: {err:?}")));
    // Only read from: the mapping is read-only.
    let hvs = unsafe { Hvs::new(regs as *mut u32) };
    // The list is only for the log: without an HVS (VC4D_FAKE in QEMU)
    // vc4d still writes the picture.
    let channel = (0..CHANNELS)
        .filter(|&i| wanted.is_none_or(|w| w == i))
        .find_map(|i| hvs.channel(i))
        .filter(|_| hvs.enabled());
    match channel {
        None => println!("vc4shot: no enabled HVS channel, asking vc4d anyway"),
        Some(channel) => {
            let start = hvs.list_start(channel.index);
            let planes = hvs::decode_list(&hvs.read_list(start, 256));
            println!(
                "vc4shot: channel {} {}x{}, list at {start}, {} planes, {}",
                channel.index,
                channel.width,
                channel.height,
                planes.len(),
                hvs.describe(channel.index)
            );
            for (n, plane) in planes.iter().enumerate() {
                println!("vc4shot: plane {n}: {plane:?}");
            }
        }
    }

    // Redox lets no program map RAM by its physical address, so vc4d,
    // which has the buffers mapped, writes the picture (see shot.rs).
    let _ = fs::remove_file(shot::PICTURE);
    fs::write(shot::REQUEST, b"").unwrap_or_else(|err| fail(&format!("cannot write {}: {err}", shot::REQUEST)));
    let started = Instant::now();
    while !fs::metadata(shot::PICTURE).is_ok_and(|m| m.len() > 0) {
        if started.elapsed() > Duration::from_secs(15) {
            let _ = fs::remove_file(shot::REQUEST);
            fail("vc4d wrote no picture within 15 s (is vc4d running?)");
        }
        thread::sleep(Duration::from_millis(200));
    }
    // vc4d may still be writing: wait until the size stops changing.
    let mut size = 0;
    while let Ok(m) = fs::metadata(shot::PICTURE) {
        if m.len() == size {
            break;
        }
        size = m.len();
        thread::sleep(Duration::from_millis(300));
    }
    if out != shot::PICTURE {
        fs::copy(shot::PICTURE, &out).unwrap_or_else(|err| fail(&format!("cannot write {out}: {err}")));
    }
    println!("vc4shot: wrote {out} ({size} bytes)");
}

fn fail(why: &str) -> ! {
    eprintln!("vc4shot: {why}");
    std::process::exit(1);
}

/// As in vc4d's main.rs.
fn soc_phys(fdt: &Fdt, bus_addr: usize) -> usize {
    fdt.find_node("/soc")
        .and_then(|soc| soc.ranges())
        .and_then(|mut ranges| {
            ranges.find(|r| r.child_bus_address <= bus_addr && bus_addr - r.child_bus_address < r.size)
        })
        .map(|r| r.parent_bus_address + (bus_addr - r.child_bus_address))
        .unwrap_or(bus_addr)
}
