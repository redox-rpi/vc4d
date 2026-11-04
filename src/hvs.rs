// SPDX-License-Identifier: GPL-2.0-only
//
// The HVS (hardware video scaler) of the VideoCore IV: its registers and
// the display list that tells it which planes to compose.
//
// Translated from Linux v6.12, drivers/gpu/drm/vc4/:
// - vc4_regs.h: the SCALER_* registers and display list fields
// - vc4_plane.c: vc4_plane_mode_set(), the words of an unscaled RGB plane,
//   and the format table (pixel format and order for XRGB/ARGB8888)
// - vc4_hvs.c: HVS_BOOTLOADER_DLIST_END, the end word after the last plane
//
// Only what an unscaled, linear 32-bit plane needs: no scaling, no YUV, no
// tiling, no HVS5 (BCM2711).

use std::fmt;

/// Bus address of the HVS on all BCM283x, if the devicetree has none.
pub const HVS_BUS_ADDR: usize = 0x7e40_0000;
/// Registers and display list memory.
pub const HVS_SIZE: usize = 0x6000;

const SCALER_DISPCTRL: usize = 0x00;
const SCALER_DISPCTRL_ENABLE: u32 = 1 << 31;
const SCALER_DISPID: usize = 0x08;
/// Start of channel `x`'s display list, in words of display list memory.
const fn scaler_displist(x: usize) -> usize {
    0x20 + x * 4
}
/// The list the HVS works on now.
const fn scaler_displact(x: usize) -> usize {
    0x30 + x * 4
}
const fn scaler_dispctrlx(x: usize) -> usize {
    0x40 + x * 0x10
}
const fn scaler_dispstatx(x: usize) -> usize {
    0x48 + x * 0x10
}
const SCALER_DISPCTRLX_ENABLE: u32 = 1 << 31;
const SCALER_DISPCTRLX_WIDTH: Field = Field(23, 12);
const SCALER_DISPCTRLX_HEIGHT: Field = Field(11, 0);
const SCALER_DISPSTATX_FRAME_COUNT: Field = Field(17, 12);

const SCALER_DLIST_START: usize = 0x2000;
/// Words of display list memory.
pub const SCALER_DLIST_WORDS: usize = 0x4000 / 4;
/// The firmware's lists are below this word (Linux leaves them alone too).
pub const HVS_BOOTLOADER_DLIST_END: usize = 32;

pub const CHANNELS: usize = 3;

const SCALER_CTL0_END: u32 = 1 << 31;
const SCALER_CTL0_VALID: u32 = 1 << 30;
const SCALER_CTL0_SIZE: Field = Field(29, 24);
const SCALER_CTL0_ORDER: Field = Field(14, 13);
const SCALER_CTL0_UNITY: u32 = 1 << 4;
const SCALER_CTL0_PIXEL_FORMAT: Field = Field(3, 0);

const SCALER_POS0_FIXED_ALPHA: Field = Field(31, 24);
const SCALER_POS0_START_Y: Field = Field(23, 12);
const SCALER_POS0_START_X: Field = Field(11, 0);

const SCALER_POS2_ALPHA_MODE: Field = Field(31, 30);
const SCALER_POS2_ALPHA_MODE_PIPELINE: u32 = 0;
const SCALER_POS2_ALPHA_MODE_FIXED: u32 = 1;
const SCALER_POS2_HEIGHT: Field = Field(27, 16);
const SCALER_POS2_WIDTH: Field = Field(11, 0);

const SCALER_SRC_PITCH: Field = Field(15, 0);

const HVS_PIXEL_FORMAT_RGBA8888: u32 = 7;
const HVS_PIXEL_ORDER_ABGR: u32 = 3;

/// Written where the HVS keeps its own state while it scans a plane out.
const CONTEXT: u32 = 0xc0c0_c0c0;

/// Bits `.0` down to `.1` of a register.
#[derive(Clone, Copy)]
struct Field(u32, u32);

impl Field {
    fn mask(self) -> u32 {
        (u32::MAX >> (31 - self.0)) & (u32::MAX << self.1)
    }

    fn set(self, value: u32) -> u32 {
        (value << self.1) & self.mask()
    }

    fn get(self, reg: u32) -> u32 {
        (reg & self.mask()) >> self.1
    }
}

/// One unscaled plane of 32-bit pixels, already clipped to the screen.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plane {
    /// Bus address of the first visible pixel.
    pub bus: u32,
    /// Bytes per row of the buffer.
    pub pitch: u32,
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
    /// ARGB with per-pixel alpha (not premultiplied); else XRGB, opaque.
    pub alpha: bool,
}

impl Plane {
    /// The plane's words of a display list, as vc4_plane_mode_set() writes
    /// them for an unscaled linear RGB plane.
    pub fn words(&self) -> [u32; 7] {
        let alpha_mode = if self.alpha {
            // DRM_MODE_BLEND_COVERAGE: per-pixel alpha, not premultiplied.
            SCALER_POS2_ALPHA_MODE_PIPELINE
        } else {
            SCALER_POS2_ALPHA_MODE_FIXED
        };
        [
            // Control word 0: an unscaled RGBA8888 plane of 7 words.
            SCALER_CTL0_VALID
                | SCALER_CTL0_ORDER.set(HVS_PIXEL_ORDER_ABGR)
                | SCALER_CTL0_PIXEL_FORMAT.set(HVS_PIXEL_FORMAT_RGBA8888)
                | SCALER_CTL0_UNITY
                | SCALER_CTL0_SIZE.set(7),
            // Position word 0: start and the plane's own alpha.
            SCALER_POS0_FIXED_ALPHA.set(0xff)
                | SCALER_POS0_START_X.set(self.x)
                | SCALER_POS0_START_Y.set(self.y),
            // Position word 2 (no word 1 without scaling): source size.
            SCALER_POS2_ALPHA_MODE.set(alpha_mode)
                | SCALER_POS2_WIDTH.set(self.width)
                | SCALER_POS2_HEIGHT.set(self.height),
            // Position word 3: context.
            CONTEXT,
            // Pointer word 0.
            self.bus,
            // Pointer context word 0.
            CONTEXT,
            // Pitch word 0.
            SCALER_SRC_PITCH.set(self.pitch),
        ]
    }
}

/// A whole display list: the planes from bottom to top, then the end word.
pub fn display_list(planes: &[Plane]) -> Vec<u32> {
    let mut list = Vec::new();
    for plane in planes {
        list.extend_from_slice(&plane.words());
    }
    list.push(SCALER_CTL0_END);
    list
}

/// An enabled output channel of the HVS.
#[derive(Clone, Copy, Debug)]
pub struct Channel {
    pub index: usize,
    pub width: u32,
    pub height: u32,
}

/// The mapped HVS.
pub struct Hvs {
    regs: *mut u32,
}

impl fmt::Debug for Hvs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Hvs").finish()
    }
}

impl Hvs {
    /// # Safety
    /// `regs` maps `HVS_SIZE` bytes of the HVS as device memory, for as long
    /// as this value lives.
    pub unsafe fn new(regs: *mut u32) -> Self {
        Hvs { regs }
    }

    fn reg(&self, offset: usize) -> u32 {
        unsafe { self.regs.add(offset / 4).read_volatile() }
    }

    fn set_reg(&self, offset: usize, value: u32) {
        unsafe { self.regs.add(offset / 4).write_volatile(value) }
    }

    /// Whether the firmware left the HVS running.
    pub fn enabled(&self) -> bool {
        self.reg(SCALER_DISPCTRL) & SCALER_DISPCTRL_ENABLE != 0
    }

    pub fn id(&self) -> u32 {
        self.reg(SCALER_DISPID)
    }

    /// The channel's size if it is enabled.
    pub fn channel(&self, index: usize) -> Option<Channel> {
        let ctrl = self.reg(scaler_dispctrlx(index));
        let (width, height) = (
            SCALER_DISPCTRLX_WIDTH.get(ctrl),
            SCALER_DISPCTRLX_HEIGHT.get(ctrl),
        );
        (ctrl & SCALER_DISPCTRLX_ENABLE != 0 && width > 0 && height > 0)
            .then_some(Channel { index, width, height })
    }

    /// One line about the channel's registers, for the log.
    pub fn describe(&self, index: usize) -> String {
        let stat = self.reg(scaler_dispstatx(index));
        format!(
            "ctrl {:#010x} list {} active {} stat {:#010x} (frame {})",
            self.reg(scaler_dispctrlx(index)),
            self.list_start(index),
            self.reg(scaler_displact(index)),
            stat,
            SCALER_DISPSTATX_FRAME_COUNT.get(stat)
        )
    }

    pub fn frame_count(&self, index: usize) -> u32 {
        SCALER_DISPSTATX_FRAME_COUNT.get(self.reg(scaler_dispstatx(index)))
    }

    /// Where the channel's display list starts, in words.
    pub fn list_start(&self, index: usize) -> usize {
        self.reg(scaler_displist(index)) as usize
    }

    /// The HVS takes the new list at the start of the next frame.
    pub fn set_list_start(&self, index: usize, start: usize) {
        self.set_reg(scaler_displist(index), start as u32);
    }

    pub fn dlist_word(&self, word: usize) -> u32 {
        self.reg(SCALER_DLIST_START + word * 4)
    }

    pub fn write_dlist(&self, start: usize, words: &[u32]) {
        assert!(start >= HVS_BOOTLOADER_DLIST_END && start + words.len() <= SCALER_DLIST_WORDS);
        for (i, &word) in words.iter().enumerate() {
            self.set_reg(SCALER_DLIST_START + (start + i) * 4, word);
        }
    }

    /// The words of the list at `start` up to its end word (at most `max`).
    pub fn read_list(&self, start: usize, max: usize) -> Vec<u32> {
        let mut words = Vec::new();
        for word in start..(start + max).min(SCALER_DLIST_WORDS) {
            let value = self.dlist_word(word);
            words.push(value);
            if value & SCALER_CTL0_END != 0 {
                break;
            }
        }
        words
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fields() {
        assert_eq!(SCALER_CTL0_SIZE.mask(), 0x3f00_0000);
        assert_eq!(SCALER_DISPCTRLX_WIDTH.get(0x8000_0000 | 1824 << 12 | 984), 1824);
        assert_eq!(SCALER_DISPCTRLX_HEIGHT.get(0x8000_0000 | 1824 << 12 | 984), 984);
        assert_eq!(SCALER_POS0_FIXED_ALPHA.set(0xff), 0xff00_0000);
    }

    #[test]
    fn opaque_plane() {
        let plane = Plane {
            bus: 0xc010_0000,
            pitch: 1824 * 4,
            x: 0,
            y: 0,
            width: 1824,
            height: 984,
            alpha: false,
        };
        let list = display_list(&[plane]);
        assert_eq!(
            list,
            [
                // valid, size 7, order ABGR, RGBA8888, unity
                0x4700_6017,
                0xff00_0000,
                // fixed alpha, height 984, width 1824
                0x4000_0000 | 984 << 16 | 1824,
                CONTEXT,
                0xc010_0000,
                CONTEXT,
                1824 * 4,
                SCALER_CTL0_END,
            ]
        );
    }

    #[test]
    fn cursor_plane() {
        let plane = Plane {
            bus: 0xc020_0000,
            pitch: 256,
            x: 100,
            y: 50,
            width: 64,
            height: 64,
            alpha: true,
        };
        let words = plane.words();
        assert_eq!(words[1], 0xff00_0000 | 50 << 12 | 100);
        assert_eq!(words[2], 64 << 16 | 64);
    }
}
