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
const SCALER_DISPSTAT: usize = 0x04;
/// The channel's error bits in SCALER_DISPSTAT (write 1 to clear): COBLOW,
/// EOLN, ESFRAME, ESLINE, EUFLOW (underrun). EOF (bit 8) is not an error.
const fn scaler_dispstat_errors(x: usize) -> u32 {
    0x3e << (8 + x * 8)
}
const SCALER_DISPSTAT_DMA_ERROR: u32 = 1 << 7;
const SCALER_DISPSTAT_RESP: Field = Field(15, 14);
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
const fn scaler_dispbkgndx(x: usize) -> usize {
    0x44 + 0x10 * x
}
const SCALER_DISPCTRLX_ENABLE: u32 = 1 << 31;
const SCALER_DISPCTRLX_WIDTH: Field = Field(23, 12);
const SCALER_DISPCTRLX_HEIGHT: Field = Field(11, 0);
const SCALER_DISPSTATX_FRAME_COUNT: Field = Field(17, 12);
const SCALER_DISPSTATX_MODE: Field = Field(31, 30);

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
pub const CONTEXT: u32 = 0xc0c0_c0c0;

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

/// A plane as read back from a display list, for vc4shot.
#[derive(Clone, Copy, Debug)]
pub struct ListPlane {
    /// Unscaled (UNITY); scaled planes have a different word layout and
    /// only their position is decoded.
    pub unity: bool,
    pub format: u32,
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
    pub bus: u32,
    pub pitch: u32,
    /// Per-pixel alpha (SCALER_POS2_ALPHA_MODE_PIPELINE).
    pub alpha: bool,
}

/// The planes of a display list as `Hvs::read_list` returns it. An
/// unscaled plane's words are those `Plane::words` writes; a scaled
/// plane (the firmware's) has position word 1 (its size on the screen)
/// after position word 0, then the source size.
pub fn decode_list(words: &[u32]) -> Vec<ListPlane> {
    let mut planes = Vec::new();
    let mut i = 0;
    while i < words.len() && words[i] & SCALER_CTL0_END == 0 {
        let ctl0 = words[i];
        let size = (SCALER_CTL0_SIZE.get(ctl0) as usize).max(1);
        let w = &words[i..(i + size).min(words.len())];
        let unity = ctl0 & SCALER_CTL0_UNITY != 0;
        let (pos2, rest) = if unity { (2, 3) } else { (3, 4) };
        if w.len() >= rest + 4 {
            let (width, height) = if unity {
                (SCALER_POS2_WIDTH.get(w[pos2]), SCALER_POS2_HEIGHT.get(w[pos2]))
            } else {
                // Position word 1: the size on the screen.
                (w[2] & 0xfff, (w[2] >> 16) & 0xfff)
            };
            planes.push(ListPlane {
                unity,
                format: SCALER_CTL0_PIXEL_FORMAT.get(ctl0),
                x: SCALER_POS0_START_X.get(w[1]),
                y: SCALER_POS0_START_Y.get(w[1]),
                width,
                height,
                bus: w[rest + 1],
                pitch: SCALER_SRC_PITCH.get(w[rest + 3]),
                alpha: SCALER_POS2_ALPHA_MODE.get(w[pos2]) == SCALER_POS2_ALPHA_MODE_PIPELINE,
            });
        }
        i += size;
    }
    planes
}

/// The start (x, y) in a plane's position word 0 (its second word).
pub fn plane_start(pos0: u32) -> (u32, u32) {
    (SCALER_POS0_START_X.get(pos0), SCALER_POS0_START_Y.get(pos0))
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
    /// Normal memory, not an HVS (VC4D_FAKE): nothing clears on write.
    fake: bool,
}

// The registers are device memory that stays mapped for the whole run; the
// watch thread only reads them.
unsafe impl Send for Hvs {}

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
        Hvs { regs, fake: false }
    }

    /// An HVS in normal memory with channel 1 enabled at this size, for
    /// testing everything but the picture where there is no HVS (QEMU).
    pub fn fake(width: u32, height: u32) -> Self {
        let regs = Box::leak(vec![0u32; HVS_SIZE / 4].into_boxed_slice()).as_mut_ptr();
        let hvs = Hvs { regs, fake: true };
        hvs.set_reg(SCALER_DISPCTRL, SCALER_DISPCTRL_ENABLE);
        hvs.set_reg(
            scaler_dispctrlx(1),
            SCALER_DISPCTRLX_ENABLE
                | SCALER_DISPCTRLX_WIDTH.set(width)
                | SCALER_DISPCTRLX_HEIGHT.set(height),
        );
        hvs.set_reg(SCALER_DLIST_START, SCALER_CTL0_END);
        hvs
    }

    fn reg(&self, offset: usize) -> u32 {
        unsafe { self.regs.add(offset / 4).read_volatile() }
    }

    fn set_reg(&self, offset: usize, value: u32) {
        unsafe { self.regs.add(offset / 4).write_volatile(value) }
    }

    /// A second handle on the same registers, for the watch thread.
    pub fn share(&self) -> Hvs {
        Hvs { regs: self.regs, fake: self.fake }
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
            "ctrl {:#010x} bkgnd {:#010x} list {} active {} stat {:#010x} (mode {}, frame {})",
            self.reg(scaler_dispctrlx(index)),
            self.reg(scaler_dispbkgndx(index)),
            self.list_start(index),
            self.active(index),
            stat,
            SCALER_DISPSTATX_MODE.get(stat),
            SCALER_DISPSTATX_FRAME_COUNT.get(stat)
        )
    }

    /// SCALER_DISPCTRL and SCALER_DISPSTAT, for the log.
    pub fn describe_global(&self) -> String {
        format!(
            "dispctrl {:#010x} dispstat {:#010x}",
            self.reg(SCALER_DISPCTRL),
            self.reg(SCALER_DISPSTAT)
        )
    }

    /// The word the channel reads in the display list memory now.
    pub fn active(&self, index: usize) -> usize {
        self.reg(scaler_displact(index)) as usize
    }

    /// The errors seen on the channel since they were last cleared, and the
    /// HVS-wide ones (DMA error, bus response); empty when there are none.
    pub fn errors(&self, index: usize) -> Vec<&'static str> {
        let stat = self.reg(SCALER_DISPSTAT);
        let mut errors = Vec::new();
        let channel = stat >> (8 + index * 8);
        for (bit, name) in [(5, "COBLOW"), (4, "EOLN"), (3, "ESFRAME"), (2, "ESLINE"), (1, "EUFLOW")] {
            if channel & (1 << bit) != 0 {
                errors.push(name);
            }
        }
        if stat & SCALER_DISPSTAT_DMA_ERROR != 0 {
            errors.push("DMA_ERROR");
        }
        if SCALER_DISPSTAT_RESP.get(stat) >= 2 {
            errors.push("RESP_ERROR");
        }
        errors
    }

    /// Clears the channel's error bits and the DMA error.
    pub fn clear_errors(&self, index: usize) {
        if self.fake {
            return;
        }
        self.set_reg(SCALER_DISPSTAT, scaler_dispstat_errors(index) | SCALER_DISPSTAT_DMA_ERROR);
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
    /// Each plane's control word gives its size; only a control word can be
    /// the end word (other words, like an alpha of 0xff, may have bit 31).
    pub fn read_list(&self, start: usize, max: usize) -> Vec<u32> {
        let end = (start + max).min(SCALER_DLIST_WORDS);
        let mut words = Vec::new();
        let mut word = start;
        while word < end {
            let ctl0 = self.dlist_word(word);
            let size = if ctl0 & SCALER_CTL0_END != 0 {
                1
            } else {
                (SCALER_CTL0_SIZE.get(ctl0) as usize).max(1)
            };
            for i in word..(word + size).min(end) {
                words.push(self.dlist_word(i));
            }
            if ctl0 & SCALER_CTL0_END != 0 {
                break;
            }
            word += size;
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

    #[test]
    fn read_list_past_alpha() {
        // Position word 0 has the alpha 0xff in its top byte, bit 31.
        let hvs = Hvs::fake(640, 480);
        let plane = Plane { bus: 0xc000_0000, pitch: 2560, x: 0, y: 0, width: 640, height: 480, alpha: false };
        let list = display_list(&[plane, plane]);
        hvs.write_dlist(0x800, &list);
        assert_eq!(hvs.read_list(0x800, 32), list);
        assert_eq!(hvs.read_list(0x800, 3), list[..3]);
    }

    #[test]
    fn decode_own_list() {
        let cursor = Plane { bus: 0xc100_0000, pitch: 256, x: 5, y: 7, width: 64, height: 64, alpha: true };
        let screen = Plane { bus: 0xc000_0000, pitch: 7680, x: 0, y: 0, width: 1920, height: 1080, alpha: false };
        let planes = decode_list(&display_list(&[screen, cursor]));
        assert_eq!(planes.len(), 2);
        let c = planes[1];
        assert!(c.unity && c.alpha);
        assert_eq!((c.x, c.y, c.width, c.height, c.bus, c.pitch), (5, 7, 64, 64, 0xc100_0000, 256));
        assert!(!planes[0].alpha);
    }

    #[test]
    fn decode_firmware_list() {
        // Pi 3B+ firmware list from row 220a: 1824x984 scaled to 1920x1035 at y 22.
        let words = [
            0x4e005807, 0xff016000, 0x040b0780, 0x43d80720, 0x03d70000, 0xfe914000, 0xfefeaf80,
            0x00001c80, 0x00008300, 0x40f33261, 0x40f36260, 0x00017d36, 0x00000ff4, 0x00000ff4,
            0x80000000,
        ];
        let planes = decode_list(&words);
        assert_eq!(planes.len(), 1);
        let p = planes[0];
        assert!(!p.unity);
        assert_eq!((p.x, p.y, p.width, p.height), (0, 22, 1920, 1035));
    }
}
