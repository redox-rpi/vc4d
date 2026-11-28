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
// Linear 32-bit RGB planes, unscaled or scaled; no YUV, no tiling. Both
// the HVS of the BCM2835/6/7 and the HVS5 of the BCM2711 (`Gen`), whose
// registers and lists differ in a few fields (vc4->is_vc5 in Linux).
//
// A crate of its own with no dependencies, so its tests run on any host
// (redox-rpi#271); the driver in ../src uses it as `hvs`.

use std::fmt;

/// Bus address of the HVS on all BCM283x, if the devicetree has none.
pub const HVS_BUS_ADDR: usize = 0x7e40_0000;
/// Registers and display list memory (BCM283x; `Gen::size` for both).
pub const HVS_SIZE: usize = 0x6000;

/// Which HVS: the BCM2835/6/7's or the BCM2711's HVS5.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gen {
    Vc4,
    Vc5,
}

impl Gen {
    /// Bytes of registers and display list memory to map.
    pub fn size(self) -> usize {
        match self {
            Gen::Vc4 => 0x6000,
            Gen::Vc5 => 0x8000,
        }
    }

    fn dlist_start(self) -> usize {
        match self {
            Gen::Vc4 => SCALER_DLIST_START,
            Gen::Vc5 => SCALER5_DLIST_START,
        }
    }

    fn dispctrlx_size(self) -> (Field, Field) {
        match self {
            Gen::Vc4 => (SCALER_DISPCTRLX_WIDTH, SCALER_DISPCTRLX_HEIGHT),
            Gen::Vc5 => (SCALER5_DISPCTRLX_WIDTH, SCALER5_DISPCTRLX_HEIGHT),
        }
    }

    /// LBM words of a scaled plane are rounded to this and allocated at
    /// this alignment (vc4_lbm_size(), vc4_plane_allocate_lbm()).
    pub fn lbm_align(self) -> u32 {
        match self {
            Gen::Vc4 => 32,
            Gen::Vc5 => 64,
        }
    }
}

const SCALER_DISPCTRL: usize = 0x00;
const SCALER_DISPCTRL_ENABLE: u32 = 1 << 31;
const SCALER_DISPSTAT: usize = 0x04;
/// The channel's error bits in SCALER_DISPSTAT (write 1 to clear): ESFRAME,
/// ESLINE, EUFLOW (underrun). The other bits of the channel are events, not
/// errors (vc4_regs.h): EOF (8), EOLN (12, the DISPEOLN line is done) and
/// COBLOW (13, the output buffer is low; on the HVS5 nearly every frame).
pub const fn dispstat_errors(x: usize) -> u32 {
    0x0e << (8 + x * 8) | SCALER_DISPSTAT_DMA_ERROR
}
/// The channel's event bits that vc4d clears with the errors: EOLN, COBLOW.
const fn dispstat_events(x: usize) -> u32 {
    0x30 << (8 + x * 8)
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
const SCALER5_DISPCTRLX_WIDTH: Field = Field(28, 16);
const SCALER5_DISPCTRLX_HEIGHT: Field = Field(12, 0);
/// Frame counts: channel 0 and 1 in SCALER_DISPSTAT1, channel 2 in
/// SCALER_DISPSTAT2 (vc4_hvs_get_fifo_frame_count()).
const SCALER_DISPSTAT1_FRCNT0: Field = Field(23, 18);
const SCALER_DISPSTAT1_FRCNT1: Field = Field(17, 12);
const SCALER_DISPSTAT2_FRCNT2: Field = Field(17, 12);
const SCALER_DISPSTATX_MODE: Field = Field(31, 30);

const SCALER_DLIST_START: usize = 0x2000;
const SCALER5_DLIST_START: usize = 0x4000;
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

// HVS5 (vc4_plane_mode_set(), the is_vc5 branch): unity and the expand
// bits move in control word 0, the alpha moves from position words 0 and 2
// to control word 2 after position word 0, and the sizes get 13 bits.
const SCALER5_CTL0_UNITY: u32 = 1 << 15;
const SCALER5_CTL0_ALPHA_EXPAND: u32 = 1 << 12;
const SCALER5_CTL0_RGB_EXPAND: u32 = 1 << 11;
const SCALER5_CTL0_PIXEL_FORMAT: Field = Field(4, 0);
const SCALER5_POS0_START_Y: Field = Field(27, 16);
const SCALER5_POS0_START_X: Field = Field(13, 0);
const SCALER5_CTL2_ALPHA_MODE: Field = Field(31, 30);
const SCALER5_CTL2_ALPHA: Field = Field(15, 4);
const SCALER5_POS1_SCL_HEIGHT: Field = Field(28, 16);
const SCALER5_POS1_SCL_WIDTH: Field = Field(12, 0);
const SCALER5_POS2_HEIGHT: Field = Field(28, 16);
const SCALER5_POS2_WIDTH: Field = Field(12, 0);

const SCALER_CTL0_SCL1: Field = Field(10, 8);
const SCALER_CTL0_SCL0: Field = Field(7, 5);
const SCALER_POS1_SCL_HEIGHT: Field = Field(27, 16);
const SCALER_POS1_SCL_WIDTH: Field = Field(11, 0);
const SCALER_PPF_AGC: u32 = 1 << 30;
const SCALER_PPF_SCALE: Field = Field(24, 8);
const SCALER_TPZ0_SCALE: Field = Field(28, 8);
const SCALER_TPZ1_RECIP: Field = Field(15, 0);
const SCALER_PPF_KERNEL_OFFSET: Field = Field(13, 0);

/// Where vc4d puts the PPF filter kernel in display list memory (words),
/// above its list slots.
pub const KERNEL_START: usize = 0xa00;

/// The Mitchell/Netravali (B = C = 1/3) filter kernel for the PPF scaler,
/// as vc4_hvs_upload_linear_kernel() writes it: the first six words, then
/// the first five again in reverse order.
pub fn ppf_kernel() -> [u32; 11] {
    const fn word(c0: i32, c1: i32, c2: i32) -> u32 {
        ((c0 as u32) & 0x1ff) | (((c1 as u32) & 0x1ff) << 9) | (((c2 as u32) & 0x1ff) << 18)
    }
    let half = [
        word(0, -2, -6),
        word(-8, -10, -8),
        word(-3, 2, 18),
        word(50, 82, 119),
        word(155, 187, 213),
        word(227, 227, 0),
    ];
    let mut k = [0; 11];
    for i in 0..11 {
        k[i] = if i < 6 { half[i] } else { half[10 - i] };
    }
    k
}

/// How one axis of a plane is scaled (vc4_get_scaling_mode()).
#[derive(Clone, Copy, Debug, PartialEq)]
enum Scaling {
    None,
    /// Polyphase filter: up, or down to two thirds.
    Ppf,
    /// Trapezoidal: further down.
    Tpz,
}

fn scaling(src: u32, dst: u32) -> Scaling {
    if src == dst {
        Scaling::None
    } else if 3 * dst >= 2 * src {
        Scaling::Ppf
    } else {
        Scaling::Tpz
    }
}

/// The SCL0/SCL1 field for an RGB plane (vc4_get_scl_field()).
fn scl_field(x: Scaling, y: Scaling) -> u32 {
    use Scaling::*;
    match (x, y) {
        (Ppf, Ppf) => 0,
        (Tpz, Ppf) => 1,
        (Ppf, Tpz) => 2,
        (Tpz, Tpz) => 3,
        (Ppf, None) => 4,
        (None, Ppf) => 5,
        (None, Tpz) => 6,
        (Tpz, None) => 7,
        (None, None) => 0,
    }
}

/// Line buffer memory a scaled plane needs, in LBM words (vc4_lbm_size()):
/// none without vertical scaling.
pub fn lbm_words(generation: Gen, src_w: u32, src_h: u32, dst_w: u32, dst_h: u32) -> u32 {
    let y = scaling(src_h, dst_h);
    if y == Scaling::None {
        return 0;
    }
    let pix_per_line = if scaling(src_w, dst_w) == Scaling::Tpz { dst_w } else { src_w };
    let bytes = pix_per_line * if y == Scaling::Tpz { 8 } else { 16 };
    match generation {
        Gen::Vc4 => bytes.next_multiple_of(64) / 2,
        Gen::Vc5 => bytes.next_multiple_of(128) / 4,
    }
}

/// A plane shown at another size than its source: the size on the screen
/// and where its line buffer starts in the HVS's LBM.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Scale {
    pub width: u32,
    pub height: u32,
    pub lbm: u32,
}

const HVS_PIXEL_FORMAT_RGBA8888: u32 = 7;
const HVS_PIXEL_ORDER_ABGR: u32 = 3;
/// XRGB/ARGB8888 on the HVS5 (pixel_order_hvs5).
const HVS_PIXEL_ORDER_ARGB: u32 = 2;

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
    /// Shown scaled to another size; None: at its own size.
    pub scale: Option<Scale>,
}

impl Plane {
    /// The plane's words of a display list, as vc4_plane_mode_set() writes
    /// them for an unscaled linear RGB plane.
    pub fn words(&self, generation: Gen) -> Vec<u32> {
        let scale = self.scale.filter(|s| (s.width, s.height) != (self.width, self.height));
        match (generation, scale) {
            (Gen::Vc4, Some(scale)) => self.scaled_words(scale),
            (Gen::Vc4, None) => self.unity_words().to_vec(),
            (Gen::Vc5, scale) => self.words5(scale),
        }
    }

    /// The HVS5's words for a linear RGB plane, unscaled or scaled.
    fn words5(&self, scale: Option<Scale>) -> Vec<u32> {
        let mut w = vec![
            0, // control word 0, below, once the size is known
            SCALER5_POS0_START_X.set(self.x) | SCALER5_POS0_START_Y.set(self.y),
            // Control word 2: the plane's alpha (opaque) and the alpha mode.
            SCALER5_CTL2_ALPHA.set(0xfff) | SCALER5_CTL2_ALPHA_MODE.set(self.alpha_mode()),
        ];
        if let Some(scale) = scale {
            // Position word 1: the size on the screen.
            w.push(SCALER5_POS1_SCL_WIDTH.set(scale.width) | SCALER5_POS1_SCL_HEIGHT.set(scale.height));
        }
        w.extend_from_slice(&[
            // Position word 2: the source size.
            SCALER5_POS2_WIDTH.set(self.width) | SCALER5_POS2_HEIGHT.set(self.height),
            CONTEXT,
            self.bus,
            CONTEXT,
            SCALER_SRC_PITCH.set(self.pitch),
        ]);
        let mut ctl0 = SCALER_CTL0_VALID
            | SCALER_CTL0_ORDER.set(HVS_PIXEL_ORDER_ARGB)
            | SCALER5_CTL0_PIXEL_FORMAT.set(HVS_PIXEL_FORMAT_RGBA8888)
            | SCALER5_CTL0_ALPHA_EXPAND
            | SCALER5_CTL0_RGB_EXPAND;
        match scale {
            None => ctl0 |= SCALER5_CTL0_UNITY,
            Some(scale) => {
                let scl = self.push_scaling(&mut w, scale);
                ctl0 |= SCALER_CTL0_SCL0.set(scl) | SCALER_CTL0_SCL1.set(scl);
            }
        }
        w[0] = ctl0 | SCALER_CTL0_SIZE.set(w.len() as u32);
        w
    }

    fn alpha_mode(&self) -> u32 {
        if self.alpha { SCALER_POS2_ALPHA_MODE_PIPELINE } else { SCALER_POS2_ALPHA_MODE_FIXED }
    }

    /// The words vc4_plane_mode_set() writes for a scaled linear RGB plane.
    fn scaled_words(&self, scale: Scale) -> Vec<u32> {
        let mut w = vec![
            0, // control word 0, below, once the size is known
            SCALER_POS0_FIXED_ALPHA.set(0xff)
                | SCALER_POS0_START_X.set(self.x)
                | SCALER_POS0_START_Y.set(self.y),
            // Position word 1: the size on the screen.
            SCALER_POS1_SCL_WIDTH.set(scale.width) | SCALER_POS1_SCL_HEIGHT.set(scale.height),
            // Position word 2: the source size.
            SCALER_POS2_ALPHA_MODE.set(self.alpha_mode())
                | SCALER_POS2_WIDTH.set(self.width)
                | SCALER_POS2_HEIGHT.set(self.height),
            CONTEXT,
            self.bus,
            CONTEXT,
            SCALER_SRC_PITCH.set(self.pitch),
        ];
        let scl = self.push_scaling(&mut w, scale);
        w[0] = SCALER_CTL0_VALID
            | SCALER_CTL0_ORDER.set(HVS_PIXEL_ORDER_ABGR)
            | SCALER_CTL0_PIXEL_FORMAT.set(HVS_PIXEL_FORMAT_RGBA8888)
            | SCALER_CTL0_SCL0.set(scl)
            | SCALER_CTL0_SCL1.set(scl)
            | SCALER_CTL0_SIZE.set(w.len() as u32);
        w
    }

    /// The words after the pitch word of a scaled plane (LBM address, PPF
    /// or TPZ parameters, kernel pointers), the same on both HVS; returns
    /// the SCL field.
    fn push_scaling(&self, w: &mut Vec<u32>, scale: Scale) -> u32 {
        let x = scaling(self.width, scale.width);
        let y = scaling(self.height, scale.height);
        if y != Scaling::None {
            // LBM base address.
            w.push(scale.lbm);
        }
        let factor = |src: u32, dst: u32| ((1u64 << 16) * u64::from(src) / u64::from(dst)) as u32;
        if x == Scaling::Ppf {
            w.push(SCALER_PPF_AGC | SCALER_PPF_SCALE.set(factor(self.width, scale.width)));
        }
        if y == Scaling::Ppf {
            w.push(SCALER_PPF_AGC | SCALER_PPF_SCALE.set(factor(self.height, scale.height)));
            w.push(CONTEXT);
        }
        if x == Scaling::Tpz {
            let f = factor(self.width, scale.width);
            w.push(SCALER_TPZ0_SCALE.set(f));
            w.push(SCALER_TPZ1_RECIP.set(u32::MAX / f));
        }
        if y == Scaling::Tpz {
            let f = factor(self.height, scale.height);
            w.push(SCALER_TPZ0_SCALE.set(f));
            w.push(SCALER_TPZ1_RECIP.set(u32::MAX / f));
            w.push(CONTEXT);
        }
        if x == Scaling::Ppf || y == Scaling::Ppf {
            // Kernel pointers: H and V PPF of planes 0 and 1.
            let kernel = SCALER_PPF_KERNEL_OFFSET.set(KERNEL_START as u32);
            w.extend_from_slice(&[kernel; 4]);
        }
        scl_field(x, y)
    }

    fn unity_words(&self) -> [u32; 7] {
        // DRM_MODE_BLEND_COVERAGE (per-pixel alpha, not premultiplied), or fixed.
        let alpha_mode = self.alpha_mode();
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
pub fn display_list(generation: Gen, planes: &[Plane]) -> Vec<u32> {
    let mut list = Vec::new();
    for plane in planes {
        list.extend_from_slice(&plane.words(generation));
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
pub fn decode_list(generation: Gen, words: &[u32]) -> Vec<ListPlane> {
    if generation == Gen::Vc5 {
        return decode_list5(words);
    }
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

/// The HVS5 list's planes: control word 0, position word 0, control word
/// 2, position word 1 if scaled, position word 2, context, pointer.
fn decode_list5(words: &[u32]) -> Vec<ListPlane> {
    let mut planes = Vec::new();
    let mut i = 0;
    while i < words.len() && words[i] & SCALER_CTL0_END == 0 {
        let ctl0 = words[i];
        let size = (SCALER_CTL0_SIZE.get(ctl0) as usize).max(1);
        let w = &words[i..(i + size).min(words.len())];
        let unity = ctl0 & SCALER5_CTL0_UNITY != 0;
        let pos2 = if unity { 3 } else { 4 };
        if w.len() >= pos2 + 5 {
            // Position word 2 (unity) or 1 (scaled): the size on the screen.
            let size_word = w[3];
            let (width, height) = if unity {
                (SCALER5_POS2_WIDTH.get(size_word), SCALER5_POS2_HEIGHT.get(size_word))
            } else {
                (SCALER5_POS1_SCL_WIDTH.get(size_word), SCALER5_POS1_SCL_HEIGHT.get(size_word))
            };
            planes.push(ListPlane {
                unity,
                format: SCALER5_CTL0_PIXEL_FORMAT.get(ctl0),
                x: SCALER5_POS0_START_X.get(w[1]),
                y: SCALER5_POS0_START_Y.get(w[1]),
                width,
                height,
                bus: w[pos2 + 2],
                pitch: SCALER_SRC_PITCH.get(w[pos2 + 4]),
                alpha: SCALER5_CTL2_ALPHA_MODE.get(w[2]) == SCALER_POS2_ALPHA_MODE_PIPELINE,
            });
        }
        i += size;
    }
    planes
}

/// The start (x, y) in a plane's position word 0 (its second word).
pub fn plane_start(generation: Gen, pos0: u32) -> (u32, u32) {
    match generation {
        Gen::Vc4 => (SCALER_POS0_START_X.get(pos0), SCALER_POS0_START_Y.get(pos0)),
        Gen::Vc5 => (SCALER5_POS0_START_X.get(pos0), SCALER5_POS0_START_Y.get(pos0)),
    }
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
    generation: Gen,
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
    pub unsafe fn new(regs: *mut u32, generation: Gen) -> Self {
        Hvs { regs, generation, fake: false }
    }

    pub fn generation(&self) -> Gen {
        self.generation
    }

    /// An HVS in normal memory with channel 1 enabled at this size, for
    /// testing everything but the picture where there is no HVS (QEMU).
    pub fn fake(width: u32, height: u32, generation: Gen) -> Self {
        let regs = Box::leak(vec![0u32; generation.size() / 4].into_boxed_slice()).as_mut_ptr();
        let hvs = Hvs { regs, generation, fake: true };
        let (w, h) = generation.dispctrlx_size();
        hvs.set_reg(SCALER_DISPCTRL, SCALER_DISPCTRL_ENABLE);
        hvs.set_reg(scaler_dispctrlx(1), SCALER_DISPCTRLX_ENABLE | w.set(width) | h.set(height));
        hvs.set_reg(generation.dlist_start(), SCALER_CTL0_END);
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
        Hvs { regs: self.regs, generation: self.generation, fake: self.fake }
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
        let (w, h) = self.generation.dispctrlx_size();
        let (width, height) = (w.get(ctrl), h.get(ctrl));
        (ctrl & SCALER_DISPCTRLX_ENABLE != 0 && width > 0 && height > 0)
            .then_some(Channel { index, width, height })
    }

    /// SCALER_DISPCTRLX of the channel, raw.
    pub fn channel_ctrl(&self, index: usize) -> u32 {
        self.reg(scaler_dispctrlx(index))
    }

    /// Writes SCALER_DISPCTRLX of the channel as it is (restoring a saved
    /// value).
    pub fn set_channel_ctrl(&self, index: usize, ctrl: u32) {
        self.set_reg(scaler_dispctrlx(index), ctrl);
    }

    /// Changes the channel's output size and keeps its other bits: for a
    /// new mode, while the pixel valve that pulls from it is stopped.
    pub fn set_channel_size(&self, index: usize, width: u32, height: u32) {
        let (w, h) = self.generation.dispctrlx_size();
        let ctrl = self.reg(scaler_dispctrlx(index)) & !(w.mask() | h.mask());
        self.set_reg(scaler_dispctrlx(index), ctrl | w.set(width) | h.set(height));
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
            self.frame_count(index)
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
        for (bit, name) in [(3, "ESFRAME"), (2, "ESLINE"), (1, "EUFLOW")] {
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
        self.set_reg(SCALER_DISPSTAT, dispstat_errors(index) | dispstat_events(index));
    }

    /// SCALER_DISPSTATX of the channel, raw: mode (31:30), FIFO full and
    /// empty (29, 28), frame count (17:12), line (11:0).
    pub fn channel_stat(&self, index: usize) -> u32 {
        self.reg(scaler_dispstatx(index))
    }

    /// SCALER_DISPSTAT, raw: the error bits of all channels.
    pub fn dispstat(&self) -> u32 {
        self.reg(SCALER_DISPSTAT)
    }

    pub fn frame_count(&self, index: usize) -> u32 {
        match index {
            0 => SCALER_DISPSTAT1_FRCNT0.get(self.reg(scaler_dispstatx(1))),
            1 => SCALER_DISPSTAT1_FRCNT1.get(self.reg(scaler_dispstatx(1))),
            _ => SCALER_DISPSTAT2_FRCNT2.get(self.reg(scaler_dispstatx(2))),
        }
    }

    /// The display list of `planes` for this HVS.
    pub fn list(&self, planes: &[Plane]) -> Vec<u32> {
        display_list(self.generation, planes)
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
        self.reg(self.generation.dlist_start() + word * 4)
    }

    /// Put the PPF filter kernel at KERNEL_START, for scaled planes.
    pub fn upload_kernel(&self) {
        self.write_dlist(KERNEL_START, &ppf_kernel());
    }

    pub fn write_dlist(&self, start: usize, words: &[u32]) {
        assert!(start >= HVS_BOOTLOADER_DLIST_END && start + words.len() <= SCALER_DLIST_WORDS);
        for (i, &word) in words.iter().enumerate() {
            self.set_reg(self.generation.dlist_start() + (start + i) * 4, word);
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
            scale: None,
        };
        let list = display_list(Gen::Vc4, &[plane]);
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
            scale: None,
        };
        let words = plane.words(Gen::Vc4);
        assert_eq!(words[1], 0xff00_0000 | 50 << 12 | 100);
        assert_eq!(words[2], 64 << 16 | 64);
    }

    #[test]
    fn read_list_past_alpha() {
        // Position word 0 has the alpha 0xff in its top byte, bit 31.
        let hvs = Hvs::fake(640, 480, Gen::Vc4);
        let plane = Plane { bus: 0xc000_0000, pitch: 2560, x: 0, y: 0, width: 640, height: 480, alpha: false, scale: None };
        let list = display_list(Gen::Vc4, &[plane, plane]);
        hvs.write_dlist(0x800, &list);
        assert_eq!(hvs.read_list(0x800, 32), list);
        assert_eq!(hvs.read_list(0x800, 3), list[..3]);
    }

    #[test]
    fn decode_own_list() {
        let cursor = Plane { bus: 0xc100_0000, pitch: 256, x: 5, y: 7, width: 64, height: 64, alpha: true, scale: None };
        let screen = Plane { bus: 0xc000_0000, pitch: 7680, x: 0, y: 0, width: 1920, height: 1080, alpha: false, scale: None };
        let planes = decode_list(Gen::Vc4, &display_list(Gen::Vc4, &[screen, cursor]));
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
        let planes = decode_list(Gen::Vc4, &words);
        assert_eq!(planes.len(), 1);
        let p = planes[0];
        assert!(!p.unity);
        assert_eq!((p.x, p.y, p.width, p.height), (0, 22, 1920, 1035));
    }

    #[test]
    fn scaled_up_plane() {
        // 1280x720 shown at 1920x1080: PPF on both axes.
        let plane = Plane {
            bus: 0xc000_0000,
            pitch: 1280 * 4,
            x: 0,
            y: 0,
            width: 1280,
            height: 720,
            alpha: false,
            scale: Some(Scale { width: 1920, height: 1080, lbm: 0 }),
        };
        let w = plane.words(Gen::Vc4);
        assert_eq!(w.len(), 16);
        assert_eq!(w[0] & 0x3f00_0000, 16 << 24);
        assert_eq!(w[0] & SCALER_CTL0_UNITY, 0);
        assert_eq!(SCALER_CTL0_SCL0.get(w[0]), 0);
        assert_eq!(w[2], 1080 << 16 | 1920);
        assert_eq!(w[3], 0x4000_0000 | 720 << 16 | 1280);
        assert_eq!(w[8], 0); // LBM
        assert_eq!(w[9], SCALER_PPF_AGC | SCALER_PPF_SCALE.set(43690)); // 2/3
        assert_eq!(w[10], SCALER_PPF_AGC | SCALER_PPF_SCALE.set(43690));
        assert_eq!(w[11], CONTEXT);
        assert_eq!(&w[12..], &[KERNEL_START as u32; 4]);
        assert_eq!(lbm_words(Gen::Vc4, 1280, 720, 1920, 1080), 1280 * 8);
    }

    #[test]
    fn same_size_scale_is_unity() {
        let plane = Plane {
            bus: 0, pitch: 256, x: 0, y: 0, width: 64, height: 64, alpha: true,
            scale: Some(Scale { width: 64, height: 64, lbm: 0 }),
        };
        assert_eq!(plane.words(Gen::Vc4).len(), 7);
        assert_ne!(plane.words(Gen::Vc4)[0] & SCALER_CTL0_UNITY, 0);
    }

    #[test]
    fn kernel_is_symmetric() {
        let k = ppf_kernel();
        for i in 0..5 {
            assert_eq!(k[i], k[10 - i]);
        }
        assert_eq!(k[5] & 0x1ff, 227);
    }

    fn screen(width: u32, height: u32) -> Plane {
        Plane { bus: 0xc010_0000, pitch: width * 4, x: 0, y: 0, width, height, alpha: false, scale: None }
    }

    #[test]
    fn hvs5_opaque_plane() {
        // vc4_plane_mode_set() with is_vc5 for an XRGB8888 plane at 1920x1080.
        let list = display_list(Gen::Vc5, &[screen(1920, 1080)]);
        assert_eq!(
            list,
            [
                // valid, size 8, order ARGB (2), unity (15), alpha and RGB expand, RGBA8888
                0x4800_0000 | 2 << 13 | 1 << 15 | 1 << 12 | 1 << 11 | 7,
                0,
                // control word 2: alpha 0xfff, fixed alpha mode
                0x4000_fff0,
                1080 << 16 | 1920,
                CONTEXT,
                0xc010_0000,
                CONTEXT,
                1920 * 4,
                SCALER_CTL0_END,
            ]
        );
    }

    #[test]
    fn hvs5_cursor_position_and_alpha() {
        let cursor = Plane { x: 3000, y: 2000, width: 64, height: 64, alpha: true, ..screen(64, 64) };
        let w = cursor.words(Gen::Vc5);
        assert_eq!(w.len(), 8);
        assert_eq!(plane_start(Gen::Vc5, w[1]), (3000, 2000));
        assert_eq!(w[2] >> 30, SCALER_POS2_ALPHA_MODE_PIPELINE);
    }

    #[test]
    fn hvs5_decode_own_list() {
        let cursor = Plane { x: 100, y: 200, width: 64, height: 64, alpha: true, ..screen(64, 64) };
        let planes = decode_list(Gen::Vc5, &display_list(Gen::Vc5, &[screen(1920, 1080), cursor]));
        assert_eq!(planes.len(), 2);
        assert!(planes[0].unity && !planes[0].alpha);
        assert_eq!((planes[0].width, planes[0].height, planes[0].pitch), (1920, 1080, 1920 * 4));
        assert_eq!((planes[1].x, planes[1].y, planes[1].bus), (100, 200, 0xc010_0000));
        assert!(planes[1].alpha);
    }

    #[test]
    fn hvs5_scaled_plane() {
        let plane = Plane { scale: Some(Scale { width: 1920, height: 1080, lbm: 0x40 }), ..screen(1280, 720) };
        let w = plane.words(Gen::Vc5);
        // ctl0, pos0, ctl2, pos1, pos2, context, ptr, ptr context, pitch, LBM,
        // H PPF, V PPF and context, 4 kernel pointers.
        assert_eq!(w.len(), 17);
        assert_eq!(w[0] & SCALER5_CTL0_UNITY, 0);
        assert_eq!(SCALER_CTL0_SIZE.get(w[0]), 17);
        assert_eq!(w[3], 1080 << 16 | 1920);
        assert_eq!(w[4], 720 << 16 | 1280);
        assert_eq!(w[9], 0x40);
        let planes = decode_list(Gen::Vc5, &w);
        assert_eq!((planes[0].width, planes[0].height, planes[0].bus), (1920, 1080, 0xc010_0000));
        assert_eq!(lbm_words(Gen::Vc5, 1280, 720, 1920, 1080), 1280 * 16 / 4);
    }

    #[test]
    fn hvs5_channel_and_dlist() {
        let hvs = Hvs::fake(1920, 1080, Gen::Vc5);
        let c = hvs.channel(1).unwrap();
        assert_eq!((c.width, c.height), (1920, 1080));
        hvs.write_dlist(0x800, &hvs.list(&[screen(1920, 1080)]));
        assert_eq!(hvs.read_list(0x800, 64).len(), 9);
        // The list memory starts at 0x4000 on the HVS5.
        assert_eq!(hvs.reg(0x4000 + 0x800 * 4), hvs.dlist_word(0x800));
    }

    #[test]
    fn eoln_and_coblow_are_not_errors() {
        let hvs = Hvs::fake(640, 480, Gen::Vc5);
        // Test 544: dispstat 0x00200000 (COBLOW 1), 0x00100004 (EOLN 1).
        hvs.set_reg(SCALER_DISPSTAT, 0x0030_0004);
        assert!(hvs.errors(1).is_empty());
        hvs.set_reg(SCALER_DISPSTAT, 1 << 17);
        assert_eq!(hvs.errors(1), ["EUFLOW"]);
        assert_eq!(dispstat_errors(1), 0x0e00 << 8 | 0x80);
    }
}
