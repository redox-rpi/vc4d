// SPDX-License-Identifier: GPL-2.0-only
//
// EDID parsing for vc4d (stage 2b, redox-rpi#188): what the monitor says it
// can show, read over HDMI DDC. Base block (EDID 1.3/1.4) and the CEA-861
// extension (HDMI). No dependencies: builds and tests on any host
// (cargo test in this directory, redox-rpi#271).
//
// Layout from the VESA E-EDID standard and CEA-861 as summarised in public
// documentation; the code is our own.

use std::fmt;

pub const BLOCK: usize = 128;
const HEADER: [u8; 8] = [0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00];
const CEA_TAG: u8 = 0x02;
const HDMI_OUI: u32 = 0x000c03;

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    Short(usize),
    Header,
    Checksum { block: usize, sum: u8 },
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Short(len) => write!(f, "{len} bytes, a block has {BLOCK}"),
            Error::Header => write!(f, "no EDID header (00 ff ff ff ff ff ff 00)"),
            Error::Checksum { block, sum } => write!(f, "block {block}: checksum off by {sum:#04x}"),
        }
    }
}

/// A block's bytes add up to 0 modulo 256.
pub fn checksum(block: &[u8]) -> u8 {
    block.iter().fold(0u8, |a, b| a.wrapping_add(*b))
}

/// A video mode's timing, as in a detailed timing descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    pub pixel_khz: u32,
    pub hactive: u32,
    pub hfp: u32,
    pub hsync: u32,
    pub hbp: u32,
    pub vactive: u32,
    pub vfp: u32,
    pub vsync: u32,
    pub vbp: u32,
    pub hsync_positive: bool,
    pub vsync_positive: bool,
    pub interlaced: bool,
}

impl Timing {
    pub fn htotal(&self) -> u32 {
        self.hactive + self.hfp + self.hsync + self.hbp
    }

    pub fn vtotal(&self) -> u32 {
        self.vactive + self.vfp + self.vsync + self.vbp
    }

    /// Refresh rate in millihertz (fields per second for interlaced modes).
    pub fn refresh_mhz(&self) -> u32 {
        let total = u64::from(self.htotal()) * u64::from(self.vtotal());
        if total == 0 {
            return 0;
        }
        let fields = if self.interlaced { 2 } else { 1 };
        (u64::from(self.pixel_khz) * 1_000_000 * fields / total) as u32
    }

    /// From an 18-byte detailed timing descriptor; None for a display
    /// descriptor (pixel clock 0).
    pub fn from_descriptor(d: &[u8]) -> Option<Timing> {
        let clock = u32::from(u16::from_le_bytes([d[0], d[1]]));
        if clock == 0 {
            return None;
        }
        let hi = |b: u8, shift: u32| u32::from((b >> shift) & 0xf);
        let hactive = u32::from(d[2]) | hi(d[4], 4) << 8;
        let hblank = u32::from(d[3]) | hi(d[4], 0) << 8;
        let vactive = u32::from(d[5]) | hi(d[7], 4) << 8;
        let vblank = u32::from(d[6]) | hi(d[7], 0) << 8;
        let hfp = u32::from(d[8]) | u32::from(d[11] >> 6 & 3) << 8;
        let hsync = u32::from(d[9]) | u32::from(d[11] >> 4 & 3) << 8;
        let vfp = u32::from(d[10] >> 4) | u32::from(d[11] >> 2 & 3) << 4;
        let vsync = u32::from(d[10] & 0xf) | u32::from(d[11] & 3) << 4;
        let flags = d[17];
        // Digital separate sync (bits 4:3 = 11) carries both polarities.
        let separate = flags >> 3 & 3 == 3;
        Some(Timing {
            pixel_khz: clock * 10,
            hactive,
            hfp,
            hsync,
            hbp: hblank.saturating_sub(hfp + hsync),
            vactive,
            vfp,
            vsync,
            vbp: vblank.saturating_sub(vfp + vsync),
            hsync_positive: separate && flags & 0x02 != 0,
            vsync_positive: separate && flags & 0x04 != 0,
            interlaced: flags & 0x80 != 0,
        })
    }
}

impl fmt::Display for Timing {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mhz = self.refresh_mhz();
        write!(
            f,
            "{}x{}{}@{}.{:03} Hz, pixel clock {}.{:03} MHz, h {} fp {} sync {} bp {} (total {}), v {} fp {} sync {} bp {} (total {}), sync {}{}",
            self.hactive,
            self.vactive,
            if self.interlaced { "i" } else { "" },
            mhz / 1000,
            mhz % 1000,
            self.pixel_khz / 1000,
            self.pixel_khz % 1000,
            self.hactive,
            self.hfp,
            self.hsync,
            self.hbp,
            self.htotal(),
            self.vactive,
            self.vfp,
            self.vsync,
            self.vbp,
            self.vtotal(),
            if self.hsync_positive { "+" } else { "-" },
            if self.vsync_positive { "+" } else { "-" },
        )
    }
}

/// A mode named only by size and refresh rate (established and standard
/// timings, CEA video codes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mode {
    pub width: u32,
    pub height: u32,
    pub hz: u32,
    pub interlaced: bool,
}

impl fmt::Display for Mode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}x{}{}@{}", self.width, self.height, if self.interlaced { "i" } else { "" }, self.hz)
    }
}

const fn m(width: u32, height: u32, hz: u32) -> Mode {
    Mode { width, height, hz, interlaced: false }
}
const fn mi(width: u32, height: u32, hz: u32) -> Mode {
    Mode { width, height, hz, interlaced: true }
}

/// Established timings I and II (bytes 35-36, most significant bit first)
/// and the manufacturer's byte 37 bit 7.
const ESTABLISHED: [Mode; 17] = [
    m(720, 400, 70), m(720, 400, 88), m(640, 480, 60), m(640, 480, 67),
    m(640, 480, 72), m(640, 480, 75), m(800, 600, 56), m(800, 600, 60),
    m(800, 600, 72), m(800, 600, 75), m(832, 624, 75), mi(1024, 768, 87),
    m(1024, 768, 60), m(1024, 768, 70), m(1024, 768, 75), m(1280, 1024, 75),
    m(1152, 870, 75),
];

/// The CEA-861 video codes (VIC) a Pi 3 can drive, with their mode. Others
/// show as their number.
pub fn vic_mode(vic: u8) -> Option<Mode> {
    Some(match vic {
        1 => m(640, 480, 60),
        2 | 3 => m(720, 480, 60),
        4 => m(1280, 720, 60),
        5 => mi(1920, 1080, 60),
        6 | 7 => mi(1440, 480, 60),
        16 => m(1920, 1080, 60),
        17 | 18 => m(720, 576, 50),
        19 => m(1280, 720, 50),
        20 => mi(1920, 1080, 50),
        21 | 22 => mi(1440, 576, 50),
        31 => m(1920, 1080, 50),
        32 => m(1920, 1080, 24),
        33 => m(1920, 1080, 25),
        34 => m(1920, 1080, 30),
        60 => m(1280, 720, 24),
        61 => m(1280, 720, 25),
        62 => m(1280, 720, 30),
        63 => m(1920, 1080, 120),
        64 => m(1920, 1080, 100),
        93 => m(3840, 2160, 24),
        94 => m(3840, 2160, 25),
        95 => m(3840, 2160, 30),
        96 => m(3840, 2160, 50),
        97 => m(3840, 2160, 60),
        _ => return None,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RangeLimits {
    pub vmin_hz: u32,
    pub vmax_hz: u32,
    pub hmin_khz: u32,
    pub hmax_khz: u32,
    /// Highest pixel clock, 0 if not given.
    pub max_pixel_mhz: u32,
}

/// What a CEA-861 extension block adds.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct Cea {
    pub revision: u8,
    pub underscan: bool,
    pub audio: bool,
    pub ycbcr444: bool,
    pub ycbcr422: bool,
    /// Video codes in the order listed, with the native flag.
    pub vics: Vec<(u8, bool)>,
    /// A vendor block with the HDMI OUI 00-0C-03: the sink is HDMI, not DVI.
    pub hdmi: bool,
    pub timings: Vec<Timing>,
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct Edid {
    /// Three letters from the PNP ID, e.g. "SAM".
    pub manufacturer: String,
    pub product: u16,
    pub serial: u32,
    pub week: u8,
    pub year: u32,
    pub version: (u8, u8),
    pub digital: bool,
    /// Physical size in cm (0 if unknown).
    pub width_cm: u8,
    pub height_cm: u8,
    pub name: Option<String>,
    pub serial_text: Option<String>,
    pub range: Option<RangeLimits>,
    /// Detailed timings of the base block; the first is the preferred mode.
    pub timings: Vec<Timing>,
    pub established: Vec<Mode>,
    pub standard: Vec<Mode>,
    /// Extension blocks the base block announces.
    pub extensions: u8,
    pub cea: Option<Cea>,
}

fn text(d: &[u8]) -> String {
    d[5..18]
        .iter()
        .take_while(|&&b| b != 0x0a)
        .map(|&b| if b.is_ascii_graphic() || b == b' ' { b as char } else { '?' })
        .collect::<String>()
        .trim_end()
        .to_string()
}

impl Edid {
    /// Parses the base block (the first 128 bytes) and any extension
    /// blocks after it in `data`. Extensions with a bad checksum are left
    /// out; the base block must be valid.
    pub fn parse(data: &[u8]) -> Result<Edid, Error> {
        if data.len() < BLOCK {
            return Err(Error::Short(data.len()));
        }
        let b = &data[..BLOCK];
        if b[..8] != HEADER {
            return Err(Error::Header);
        }
        let sum = checksum(b);
        if sum != 0 {
            return Err(Error::Checksum { block: 0, sum });
        }
        let id = u16::from_be_bytes([b[8], b[9]]);
        let letter = |shift: u32| (b'A' - 1 + ((id >> shift) & 0x1f) as u8) as char;
        let mut edid = Edid {
            manufacturer: [letter(10), letter(5), letter(0)].iter().collect(),
            product: u16::from_le_bytes([b[10], b[11]]),
            serial: u32::from_le_bytes([b[12], b[13], b[14], b[15]]),
            week: b[16],
            year: 1990 + u32::from(b[17]),
            version: (b[18], b[19]),
            digital: b[20] & 0x80 != 0,
            width_cm: b[21],
            height_cm: b[22],
            extensions: b[126],
            ..Edid::default()
        };
        let bits = u32::from(b[35]) << 9 | u32::from(b[36]) << 1 | u32::from(b[37] >> 7);
        edid.established = ESTABLISHED
            .iter()
            .enumerate()
            .filter(|(i, _)| bits & (1 << (16 - i)) != 0)
            .map(|(_, mode)| *mode)
            .collect();
        for pair in b[38..54].chunks_exact(2) {
            if pair == [0x01, 0x01] || pair[0] == 0 {
                continue;
            }
            let width = (u32::from(pair[0]) + 31) * 8;
            let height = match pair[1] >> 6 {
                // 16:10 in EDID 1.3 and later (1:1 before).
                0 if edid.version >= (1, 3) => width * 10 / 16,
                0 => width,
                1 => width * 3 / 4,
                2 => width * 4 / 5,
                _ => width * 9 / 16,
            };
            edid.standard.push(m(width, height, 60 + u32::from(pair[1] & 0x3f)));
        }
        for d in b[54..126].chunks_exact(18) {
            if let Some(t) = Timing::from_descriptor(d) {
                edid.timings.push(t);
                continue;
            }
            match d[3] {
                0xfc => edid.name = Some(text(d)),
                0xff => edid.serial_text = Some(text(d)),
                0xfd => {
                    // Offset flags in byte 4 add 255 to max/min (EDID 1.4).
                    let add = |bit: u8| if d[4] & bit != 0 { 255 } else { 0 };
                    edid.range = Some(RangeLimits {
                        vmin_hz: u32::from(d[5]) + add(0x01),
                        vmax_hz: u32::from(d[6]) + add(0x02),
                        hmin_khz: u32::from(d[7]) + add(0x04),
                        hmax_khz: u32::from(d[8]) + add(0x08),
                        max_pixel_mhz: u32::from(d[9]) * 10,
                    });
                }
                _ => {}
            }
        }
        for block in data[BLOCK..].chunks_exact(BLOCK) {
            if checksum(block) == 0 && block[0] == CEA_TAG && edid.cea.is_none() {
                edid.cea = Some(parse_cea(block));
            }
        }
        Ok(edid)
    }

    /// The mode the monitor prefers: the first detailed timing.
    pub fn preferred(&self) -> Option<&Timing> {
        self.timings.first().or_else(|| self.cea.as_ref().and_then(|c| c.timings.first()))
    }
}

fn parse_cea(b: &[u8]) -> Cea {
    let mut cea = Cea {
        revision: b[1],
        underscan: b[3] & 0x80 != 0,
        audio: b[3] & 0x40 != 0,
        ycbcr444: b[3] & 0x20 != 0,
        ycbcr422: b[3] & 0x10 != 0,
        ..Cea::default()
    };
    let dtd_start = usize::from(b[2]).min(BLOCK - 1);
    let mut at = 4;
    while at < dtd_start {
        let tag = b[at] >> 5;
        let len = usize::from(b[at] & 0x1f);
        let payload = &b[(at + 1).min(dtd_start)..(at + 1 + len).min(dtd_start)];
        match tag {
            2 => {
                for &svd in payload {
                    // CEA-861-F: 129-192 are codes 1-64 with the native flag.
                    cea.vics.push(if (129..=192).contains(&svd) { (svd - 128, true) } else { (svd, false) });
                }
            }
            3 if payload.len() >= 3 => {
                let oui = u32::from(payload[0]) | u32::from(payload[1]) << 8 | u32::from(payload[2]) << 16;
                cea.hdmi |= oui == HDMI_OUI;
            }
            _ => {}
        }
        at += 1 + len;
    }
    if dtd_start >= 4 {
        let mut at = dtd_start;
        while at + 18 <= BLOCK - 1 {
            match Timing::from_descriptor(&b[at..at + 18]) {
                Some(t) => cea.timings.push(t),
                None => break,
            }
            at += 18;
        }
    }
    cea
}

impl fmt::Display for Edid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} product {:#06x} serial {} \"{}\", EDID {}.{}, made week {} of {}, {}, {}x{} cm, {} extension(s)",
            self.manufacturer,
            self.product,
            self.serial,
            self.name.as_deref().unwrap_or(""),
            self.version.0,
            self.version.1,
            self.week,
            self.year,
            if self.digital { "digital" } else { "analog" },
            self.width_cm,
            self.height_cm,
            self.extensions
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An 18-byte descriptor for a timing (the reverse of from_descriptor).
    fn dtd(t: &Timing) -> [u8; 18] {
        let mut d = [0u8; 18];
        let clock = (t.pixel_khz / 10) as u16;
        d[..2].copy_from_slice(&clock.to_le_bytes());
        let hblank = t.hfp + t.hsync + t.hbp;
        let vblank = t.vfp + t.vsync + t.vbp;
        d[2] = t.hactive as u8;
        d[3] = hblank as u8;
        d[4] = ((t.hactive >> 8) << 4 | (hblank >> 8)) as u8;
        d[5] = t.vactive as u8;
        d[6] = vblank as u8;
        d[7] = ((t.vactive >> 8) << 4 | (vblank >> 8)) as u8;
        d[8] = t.hfp as u8;
        d[9] = t.hsync as u8;
        d[10] = ((t.vfp & 0xf) << 4 | (t.vsync & 0xf)) as u8;
        d[11] = ((t.hfp >> 8) << 6 | (t.hsync >> 8) << 4 | (t.vfp >> 4) << 2 | (t.vsync >> 4)) as u8;
        d[17] = 0x18 | if t.vsync_positive { 4 } else { 0 } | if t.hsync_positive { 2 } else { 0 }
            | if t.interlaced { 0x80 } else { 0 };
        d
    }

    fn descriptor(tag: u8, text: &str) -> [u8; 18] {
        let mut d = [0u8; 18];
        d[3] = tag;
        let mut body = [0x20u8; 13];
        body[..text.len()].copy_from_slice(text.as_bytes());
        if text.len() < 13 {
            body[text.len()] = 0x0a;
        }
        d[5..].copy_from_slice(&body);
        d
    }

    fn fix_checksum(block: &mut [u8]) {
        block[127] = 0;
        block[127] = 0u8.wrapping_sub(checksum(block));
    }

    pub const P1080: Timing = Timing {
        pixel_khz: 148_500, hactive: 1920, hfp: 88, hsync: 44, hbp: 148,
        vactive: 1080, vfp: 4, vsync: 5, vbp: 36,
        hsync_positive: true, vsync_positive: true, interlaced: false,
    };
    const P720: Timing = Timing {
        pixel_khz: 74_250, hactive: 1280, hfp: 110, hsync: 40, hbp: 220,
        vactive: 720, vfp: 5, vsync: 5, vbp: 20,
        hsync_positive: true, vsync_positive: true, interlaced: false,
    };

    /// A 1080p HDMI monitor: base block plus a CEA extension.
    fn monitor() -> Vec<u8> {
        let mut b = vec![0u8; 256];
        b[..8].copy_from_slice(&HEADER);
        // "RPI": R=18, P=16, I=9.
        let id: u16 = 18 << 10 | 16 << 5 | 9;
        b[8..10].copy_from_slice(&id.to_be_bytes());
        b[10..12].copy_from_slice(&0x1234u16.to_le_bytes());
        b[12..16].copy_from_slice(&42u32.to_le_bytes());
        b[16] = 10;
        b[17] = 34; // 2024
        b[18] = 1;
        b[19] = 3;
        b[20] = 0x80;
        b[21] = 53;
        b[22] = 30;
        b[35] = 0x21; // 640x480@60, 800x600@60
        b[36] = 0x08; // 1024x768@60
        b[38..54].copy_from_slice(&[0x01; 16]);
        b[38] = (1280 / 8 - 31) as u8;
        b[39] = 0xc0; // 16:9 @60 -> 1280x720@60
        b[54..72].copy_from_slice(&dtd(&P1080));
        b[72..90].copy_from_slice(&descriptor(0xfc, "Test 1080p"));
        let mut range = [0u8; 18];
        range[3] = 0xfd;
        range[5..10].copy_from_slice(&[24, 75, 15, 83, 17]);
        b[90..108].copy_from_slice(&range);
        b[108..126].copy_from_slice(&descriptor(0xff, "SN0042"));
        b[126] = 1;
        fix_checksum(&mut b[..128]);

        let e = &mut b[128..];
        e[0] = CEA_TAG;
        e[1] = 3;
        e[3] = 0x80 | 0x40 | 0x01;
        // Video block: VIC 16 native, 4, 31, 97; vendor block with the HDMI OUI.
        let blocks = [0x44, 16 | 0x80, 4, 31, 97, 0x65, 0x03, 0x0c, 0x00, 0x10, 0x00];
        e[4..4 + blocks.len()].copy_from_slice(&blocks);
        e[2] = (4 + blocks.len()) as u8;
        let at = usize::from(e[2]);
        e[at..at + 18].copy_from_slice(&dtd(&P720));
        fix_checksum(e);
        b
    }

    #[test]
    fn base_block() {
        let e = Edid::parse(&monitor()).unwrap();
        assert_eq!(e.manufacturer, "RPI");
        assert_eq!((e.product, e.serial, e.week, e.year), (0x1234, 42, 10, 2024));
        assert_eq!(e.version, (1, 3));
        assert!(e.digital);
        assert_eq!(e.name.as_deref(), Some("Test 1080p"));
        assert_eq!(e.serial_text.as_deref(), Some("SN0042"));
        assert_eq!(e.range, Some(RangeLimits { vmin_hz: 24, vmax_hz: 75, hmin_khz: 15, hmax_khz: 83, max_pixel_mhz: 170 }));
        assert_eq!(e.established, vec![m(640, 480, 60), m(800, 600, 60), m(1024, 768, 60)]);
        assert_eq!(e.standard, vec![m(1280, 720, 60)]);
        assert_eq!(e.extensions, 1);
    }

    #[test]
    fn preferred_mode_is_1080p60() {
        let e = Edid::parse(&monitor()).unwrap();
        let p = e.preferred().unwrap();
        assert_eq!(*p, P1080);
        assert_eq!((p.htotal(), p.vtotal()), (2200, 1125));
        assert_eq!(p.refresh_mhz(), 60_000);
        assert!(p.to_string().starts_with("1920x1080@60.000 Hz, pixel clock 148.500 MHz"));
    }

    #[test]
    fn cea_extension() {
        let cea = Edid::parse(&monitor()).unwrap().cea.unwrap();
        assert_eq!(cea.revision, 3);
        assert!(cea.underscan && cea.audio && cea.hdmi);
        assert_eq!(cea.vics, vec![(16, true), (4, false), (31, false), (97, false)]);
        assert_eq!(cea.timings, vec![P720]);
        assert_eq!(vic_mode(16), Some(m(1920, 1080, 60)));
        assert_eq!(vic_mode(5), Some(mi(1920, 1080, 60)));
        assert_eq!(vic_mode(200), None);
    }

    #[test]
    fn bad_input() {
        assert_eq!(Edid::parse(&[0u8; 10]), Err(Error::Short(10)));
        assert_eq!(Edid::parse(&[0u8; 128]), Err(Error::Header));
        let mut b = monitor();
        b[20] ^= 1;
        assert!(matches!(Edid::parse(&b), Err(Error::Checksum { block: 0, .. })));
        // A broken extension is left out, the base block still parses.
        let mut b = monitor();
        b[200] ^= 1;
        let e = Edid::parse(&b).unwrap();
        assert!(e.cea.is_none());
        assert_eq!(e.timings.len(), 1);
    }

    /// What QEMU's i2c-ddc device sends (QEMU 10.2, defaults), as vc4d
    /// read it over BSC2 in QEMU raspi3b: an EDID made by someone else.
    const QEMU: [u8; 128] = [
        0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x49, 0x14, 0x34, 0x12,
        0x00, 0x00, 0x00, 0x00, 0x2a, 0x18, 0x01, 0x04, 0xa5, 0x20, 0x14, 0x78,
        0x06, 0xee, 0x91, 0xa3, 0x54, 0x4c, 0x99, 0x26, 0x0f, 0x50, 0x54, 0x21,
        0x08, 0x00, 0xe1, 0xc0, 0xd1, 0xc0, 0xd1, 0x00, 0xa9, 0x40, 0xb3, 0x00,
        0x95, 0x00, 0x81, 0x80, 0x81, 0x40, 0xea, 0x29, 0x00, 0xc0, 0x51, 0x20,
        0x1c, 0x30, 0x40, 0x26, 0x44, 0x40, 0x45, 0xcb, 0x10, 0x00, 0x00, 0x18,
        0x00, 0x00, 0x00, 0xf7, 0x00, 0x0a, 0x00, 0x40, 0x82, 0x00, 0x28, 0x20,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x32,
        0x7d, 0x1e, 0xa0, 0xff, 0x01, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
        0x00, 0x00, 0x00, 0xfc, 0x00, 0x51, 0x45, 0x4d, 0x55, 0x20, 0x4d, 0x6f,
        0x6e, 0x69, 0x74, 0x6f, 0x72, 0x0a, 0x00, 0x3b,
    ];

    #[test]
    fn qemu_monitor() {
        let e = Edid::parse(&QEMU).unwrap();
        assert_eq!(e.manufacturer, "RHT");
        assert_eq!(e.name.as_deref(), Some("QEMU Monitor"));
        assert_eq!((e.version, e.week, e.year, e.width_cm, e.height_cm), ((1, 4), 42, 2014, 32, 20));
        let p = e.preferred().unwrap();
        assert_eq!((p.hactive, p.vactive, p.pixel_khz), (1280, 800, 107_300));
        assert_eq!((p.hfp, p.hsync, p.hbp, p.vfp, p.vsync, p.vbp), (320, 38, 90, 4, 4, 20));
        assert_eq!(p.refresh_mhz(), 74_993);
        assert_eq!(e.range, Some(RangeLimits { vmin_hz: 50, vmax_hz: 125, hmin_khz: 30, hmax_khz: 160, max_pixel_mhz: 2550 }));
        assert_eq!(e.established, vec![m(640, 480, 60), m(800, 600, 60), m(1024, 768, 60)]);
        assert_eq!(e.standard[..3], [m(2048, 1152, 60), m(1920, 1080, 60), m(1920, 1200, 60)]);
        assert_eq!(e.standard.len(), 8);
        assert!(e.cea.is_none());
    }

    /// The NanoKVM on the test Pi 3B+, as vc4d read it over BSC2 (redox-rpi
    /// Pi test row 387): base block and CEA-861 extension.
    const NANOKVM: [u8; 256] = [
        0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x4e, 0x04, 0x01, 0x33,
        0x15, 0x00, 0x00, 0x00, 0x1e, 0x23, 0x01, 0x03, 0x80, 0x30, 0x1b, 0x78,
        0x2a, 0x69, 0x25, 0xa3, 0x5b, 0x50, 0xa3, 0x27, 0x11, 0x50, 0x54, 0xa5,
        0x4b, 0x00, 0xd1, 0xc0, 0x81, 0x80, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
        0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38,
        0x2d, 0x40, 0x58, 0x2c, 0x45, 0x00, 0xdc, 0x0c, 0x11, 0x00, 0x00, 0x1e,
        0x00, 0x00, 0x00, 0xff, 0x00, 0x4e, 0x61, 0x6e, 0x6f, 0x4b, 0x56, 0x4d,
        0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x4e,
        0x61, 0x6e, 0x6f, 0x4b, 0x56, 0x4d, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20,
        0x00, 0x00, 0x00, 0xfd, 0x00, 0x38, 0x4c, 0x1e, 0x53, 0x11, 0x00, 0x0a,
        0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0xf5, 0x02, 0x03, 0x11, 0x30,
        0x46, 0x04, 0x1f, 0x14, 0x13, 0x01, 0x10, 0x65, 0x03, 0x0c, 0x00, 0x10,
        0x00, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c, 0x45,
        0x00, 0xdc, 0x0c, 0x11, 0x00, 0x00, 0x1e, 0x01, 0x1d, 0x00, 0x72, 0x51,
        0xd0, 0x1e, 0x20, 0x6e, 0x28, 0x55, 0x00, 0xdc, 0x0c, 0x11, 0x00, 0x00,
        0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0xda,
    ];

    #[test]
    fn nanokvm() {
        let e = Edid::parse(&NANOKVM).unwrap();
        assert_eq!((e.manufacturer.as_str(), e.product, e.serial), ("SPD", 0x3301, 21));
        assert_eq!(e.name.as_deref(), Some("NanoKVM"));
        assert_eq!(e.version, (1, 3));
        // The timing the closed firmware programmed (Pi test row 337).
        assert_eq!(*e.preferred().unwrap(), P1080);
        assert_eq!(e.range, Some(RangeLimits { vmin_hz: 56, vmax_hz: 76, hmin_khz: 30, hmax_khz: 83, max_pixel_mhz: 170 }));
        assert_eq!(e.established.len(), 8);
        assert_eq!(e.standard, vec![m(1920, 1080, 60), m(1280, 1024, 60)]);
        let cea = e.cea.unwrap();
        assert!(cea.hdmi);
        assert_eq!(cea.vics.iter().map(|v| v.0).collect::<Vec<_>>(), vec![4, 31, 20, 19, 1, 16]);
        assert_eq!(cea.timings, vec![P1080, P720]);
    }

    #[test]
    fn descriptor_round_trip_with_large_fields() {
        // 3840x2160@30: fields above 8 bits in every place.
        let t = Timing {
            pixel_khz: 297_000, hactive: 3840, hfp: 176, hsync: 88, hbp: 296,
            vactive: 2160, vfp: 8, vsync: 10, vbp: 72,
            hsync_positive: true, vsync_positive: false, interlaced: false,
        };
        assert_eq!(Timing::from_descriptor(&dtd(&t)), Some(t));
        assert_eq!(t.refresh_mhz(), 30_000);
    }
}
