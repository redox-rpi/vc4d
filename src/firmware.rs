// SPDX-License-Identifier: GPL-2.0-only
//
// Clock rates through the Raspberry Pi firmware's property interface.
// rpi-firmwared owns the mailbox; other drivers send whole property
// messages through its root-only file /scheme/rpi-firmware/property
// (redox-rpi experiment rpi-fb, base patch 0060). Without that file vc4d
// leaves the clocks as the firmware set them.
//
// Tags: https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface

use std::fs::{File, OpenOptions};
use std::os::unix::fs::FileExt;

const PROPERTY: &str = "/scheme/rpi-firmware/property";
const TAG_GET_CLOCK_RATE: u32 = 0x0003_0002;
const TAG_SET_CLOCK_RATE: u32 = 0x0003_8002;
const RESPONSE_OK: u32 = 0x8000_0000;
const TAG_RESPONSE: u32 = 1 << 31;

/// Firmware clock ids of HDMI0 in the BCM2711 devicetree: the HDMI state
/// machine clock ("hdmi") and the pixel BVB clock ("bvb").
pub const CLOCK_HDMI: u32 = 13;
pub const CLOCK_BVB: u32 = 14;

pub struct Firmware {
    file: File,
}

impl Firmware {
    pub fn open() -> Result<Firmware, String> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(PROPERTY)
            .map_err(|err| format!("{PROPERTY}: {err}"))?;
        Ok(Firmware { file })
    }

    /// One tag with `value` words in and `words` words of room for the
    /// answer.
    fn call(&self, tag: u32, value: &[u32], words: usize) -> Result<Vec<u32>, String> {
        let room = words.max(value.len());
        let mut msg = vec![0u32; 5 + room + 1];
        msg[0] = (msg.len() * 4) as u32;
        msg[2] = tag;
        msg[3] = (room * 4) as u32;
        msg[5..5 + value.len()].copy_from_slice(value);
        let bytes: Vec<u8> = msg.iter().flat_map(|w| w.to_le_bytes()).collect();
        self.file.write_at(&bytes, 0).map_err(|err| format!("tag {tag:#010x}: {err}"))?;
        let mut answer = vec![0u8; bytes.len()];
        self.file.read_at(&mut answer, 0).map_err(|err| format!("tag {tag:#010x}: {err}"))?;
        let msg: Vec<u32> =
            answer.chunks_exact(4).map(|w| u32::from_le_bytes([w[0], w[1], w[2], w[3]])).collect();
        if msg[1] != RESPONSE_OK {
            return Err(format!("tag {tag:#010x}: firmware answered {:#010x}", msg[1]));
        }
        if msg[4] & TAG_RESPONSE == 0 {
            return Err(format!("tag {tag:#010x}: not answered"));
        }
        Ok(msg[5..5 + room].to_vec())
    }

    /// The clock's rate in Hz.
    pub fn clock_rate(&self, id: u32) -> Result<u32, String> {
        Ok(self.call(TAG_GET_CLOCK_RATE, &[id], 2)?[1])
    }

    /// Sets the clock's rate; returns the rate the firmware chose.
    pub fn set_clock_rate(&self, id: u32, hz: u32) -> Result<u32, String> {
        Ok(self.call(TAG_SET_CLOCK_RATE, &[id, hz, 0], 3)?[1])
    }
}
