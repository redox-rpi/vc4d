// SPDX-License-Identifier: GPL-2.0-only
//
// vc4shot: a screenshot of what the HVS scans out, for tests without a
// person. It reads the enabled channel's display list (read-only, nothing
// is written to the HVS), maps the buffers its unscaled planes point at and
// composes them into a PPM picture, cursor (per-pixel alpha) included.
// Scaled planes (the firmware's own) are only listed.
//
// vc4shot [file.ppm] [channel]   (default /tmp/vc4shot.ppm, the first
// enabled channel)

use std::env;
use std::fs;
use std::io::Write;

use fdt::Fdt;

#[allow(dead_code)]
#[path = "../hvs.rs"]
mod hvs;

use hvs::{Hvs, CHANNELS, HVS_BUS_ADDR, HVS_SIZE};

/// Bus address minus physical address of RAM, as in vc4d.
const RAM_BUS_OFFSET: usize = 0xc000_0000;
/// The HVS's 32-bit RGBA formats (vc4_regs.h HVS_PIXEL_FORMAT_RGBA8888).
const HVS_PIXEL_FORMAT_RGBA8888: u32 = 7;

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
    let ram_bus_offset = fdt.as_ref().map_or(RAM_BUS_OFFSET, ram_bus_offset);

    let regs = unsafe { common::physmap(phys, HVS_SIZE, common::Prot::RO, common::MemoryType::DeviceMemory) }
        .unwrap_or_else(|err| fail(&format!("cannot map the HVS at {phys:#x}: {err:?}")));
    // Only read from: the mapping is read-only.
    let hvs = unsafe { Hvs::new(regs as *mut u32) };
    if !hvs.enabled() {
        fail("the HVS is off");
    }
    let channel = (0..CHANNELS)
        .filter(|&i| wanted.is_none_or(|w| w == i))
        .find_map(|i| hvs.channel(i))
        .unwrap_or_else(|| fail("no enabled HVS channel"));
    let start = hvs.list_start(channel.index);
    let words = hvs.read_list(start, 256);
    let planes = hvs::decode_list(&words);
    println!(
        "vc4shot: channel {} {}x{}, list at {start}, {} planes, {}",
        channel.index,
        channel.width,
        channel.height,
        planes.len(),
        hvs.describe(channel.index)
    );

    let (width, height) = (channel.width as usize, channel.height as usize);
    // The background (DISPBKGND fill is black here) under the planes.
    let mut rgb = vec![0u8; width * height * 3];
    for (n, plane) in planes.iter().enumerate() {
        println!("vc4shot: plane {n}: {plane:?}");
        if !plane.unity || plane.format != HVS_PIXEL_FORMAT_RGBA8888 {
            println!("vc4shot: plane {n}: scaled or not RGBA8888, not drawn");
            continue;
        }
        let len = plane.pitch as usize * plane.height as usize;
        let buffer_phys = (plane.bus as usize).wrapping_sub(ram_bus_offset);
        let page = buffer_phys & !0xfff;
        let map_len = (buffer_phys - page + len + 0xfff) & !0xfff;
        let mem = match unsafe { common::physmap(page, map_len, common::Prot::RO, common::MemoryType::Uncacheable) } {
            Ok(mem) => mem as *const u8,
            Err(err) => {
                println!("vc4shot: plane {n}: cannot map {buffer_phys:#x}: {err:?}");
                continue;
            }
        };
        let pixels = unsafe { mem.add(buffer_phys - page) };
        for row in 0..plane.height as usize {
            let y = plane.y as usize + row;
            if y >= height {
                break;
            }
            for col in 0..plane.width as usize {
                let x = plane.x as usize + col;
                if x >= width {
                    break;
                }
                let at = row * plane.pitch as usize + col * 4;
                let p = u32::from_le_bytes(unsafe { [*pixels.add(at), *pixels.add(at + 1), *pixels.add(at + 2), *pixels.add(at + 3)] });
                let alpha = if plane.alpha { p >> 24 } else { 255 };
                let src = [(p >> 16) as u8, (p >> 8) as u8, p as u8];
                let dst = &mut rgb[(y * width + x) * 3..][..3];
                for c in 0..3 {
                    dst[c] = ((u32::from(src[c]) * alpha + u32::from(dst[c]) * (255 - alpha)) / 255) as u8;
                }
            }
        }
    }

    let mut file = fs::File::create(&out).unwrap_or_else(|err| fail(&format!("cannot create {out}: {err}")));
    write!(file, "P6\n{width} {height}\n255\n").and_then(|()| file.write_all(&rgb))
        .unwrap_or_else(|err| fail(&format!("cannot write {out}: {err}")));
    println!("vc4shot: wrote {out} ({width}x{height})");
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

/// As in vc4d's main.rs.
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
