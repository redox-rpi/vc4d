// SPDX-License-Identifier: GPL-2.0-only
//
// Screenshots of what vc4d shows, for checks without a person. Redox lets
// no program map RAM by its physical address (vc4shot got "Operation not
// permitted" on the Pi, row 256), so vc4d itself writes the picture from
// the buffers it shows: `vc4shot` (or `touch /tmp/vc4shot.request`) asks,
// and the watch thread writes /tmp/vc4shot.ppm within a few seconds.
//
// Our own code, not translated from Linux.

use std::fs;
use std::io::Write;
use std::sync::{Arc, Mutex};

use crate::adapter::Vc4Buffer;
use crate::hvs::Plane;

pub const REQUEST: &str = "/tmp/vc4shot.request";
pub const PICTURE: &str = "/tmp/vc4shot.ppm";

/// A plane vc4d shows, with the address of its first visible pixel in
/// vc4d's memory.
#[derive(Clone, Debug)]
pub struct ShotPlane {
    pub plane: Plane,
    pub pixels: *const u8,
    /// Keeps the buffer alive while the picture is written.
    pub _buffer: Arc<Vc4Buffer>,
}

// The pixels belong to the buffer that `_buffer` keeps alive.
unsafe impl Send for ShotPlane {}

/// The planes of the current display list, bottom first.
pub type Shown = Arc<Mutex<Vec<ShotPlane>>>;

/// Writes the planes, composed (per-pixel alpha where the plane has it),
/// as a PPM picture of the screen's size.
pub fn write(path: &str, width: usize, height: usize, planes: &[ShotPlane]) -> std::io::Result<()> {
    let mut rgb = vec![0u8; width * height * 3];
    for shot in planes {
        let p = shot.plane;
        for row in 0..(p.height as usize).min(height.saturating_sub(p.y as usize)) {
            let y = p.y as usize + row;
            for col in 0..(p.width as usize).min(width.saturating_sub(p.x as usize)) {
                let x = p.x as usize + col;
                let at = row * p.pitch as usize + col * 4;
                let pixel = unsafe { shot.pixels.add(at).cast::<u32>().read_unaligned() };
                let alpha = if p.alpha { pixel >> 24 } else { 255 };
                let src = [(pixel >> 16) as u8, (pixel >> 8) as u8, pixel as u8];
                let dst = &mut rgb[(y * width + x) * 3..][..3];
                for c in 0..3 {
                    dst[c] = ((u32::from(src[c]) * alpha + u32::from(dst[c]) * (255 - alpha)) / 255) as u8;
                }
            }
        }
    }
    let mut file = fs::File::create(path)?;
    write!(file, "P6\n{width} {height}\n255\n")?;
    file.write_all(&rgb)
}

/// Answers a request if there is one; called by the watch thread.
pub fn serve(shown: &Shown, width: usize, height: usize) {
    if fs::remove_file(REQUEST).is_err() {
        return;
    }
    let planes = shown.lock().unwrap().clone();
    match write(PICTURE, width, height, &planes) {
        Ok(()) => println!("vc4d: shot: wrote {PICTURE} ({width}x{height}, {} planes)", planes.len()),
        Err(err) => println!("vc4d: shot: cannot write {PICTURE}: {err}"),
    }
}
