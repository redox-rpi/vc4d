// SPDX-License-Identifier: GPL-2.0-only
//
// A client of v3dd (redox-rpi#598): the line protocol of /scheme/v3d, see
// src/bin/v3dd.rs. One command per write, one answer per read.
//
// With MAPPED set (v3dprobe --mapped, redox-rpi#666), every BO is also
// mapped into this process (fmap on the same handle), and put, get and
// fill go through that mapping instead of command lines.

use std::collections::BTreeMap;
use std::fs::{File, OpenOptions};
use std::io::{Read, Write};
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};

/// put/get/fill through the BO mappings instead of command lines.
pub static MAPPED: AtomicBool = AtomicBool::new(false);

/// ioctl() through relibc with Linux's request numbers and structs
/// (v3dprobe --libc, G3 of redox-rpi#669) instead of the scheme call:
/// the path Mesa and libdrm take, on the render node /dev/dri/renderD128
/// (a link to /scheme/v3d).
pub static LIBC: AtomicBool = AtomicBool::new(false);

unsafe extern "C" {
    fn ioctl(fd: i32, request: u64, arg: *mut u8) -> i32;
}

/// `_IOW`/`_IOWR('d', nr, size)` of Linux's drm.h and v3d_drm.h.
fn drm_request(nr: u64, size: usize, read_back: bool) -> u64 {
    let dir: u64 = if read_back { 3 } else { 1 };
    dir << 30 | (size as u64) << 16 | (b'd' as u64) << 8 | nr
}

/// One ioctl through relibc. `payload` is the wire form (v3d/uapi.rs);
/// Linux's struct differs from it only where a submit carries the
/// pointer to its BO handles, which come after the fixed part.
fn libc_ioctl(fd: i32, func: u64, payload: &mut [u8]) -> Result<usize, syscall::Error> {
    // GEM_CLOSE 0x09, SUBMIT_CL 0x40, SUBMIT_CSD 0x47, SYNCOBJ_DESTROY
    // 0xC0, SYNCOBJ_WAIT 0xC3, SYNCOBJ_RESET/SIGNAL 0xC4/0xC5.
    let (fixed, at, gap, read_back) = match func {
        0x40 => (64, 40, 0, true),
        0xC3 => (32, 0, 0, true),
        0xC4 | 0xC5 => (8, 0, 0, true),
        // The wire form has 4 bytes of padding at 60 that Linux's struct
        // does not have there; it has 4 at 44 instead.
        0x47 => (80, 44, 4, false),
        0x09 => (payload.len(), 0, 0, false),
        // VERSION: three length/pointer pairs; GET_UNIQUE: one.
        0x00 => (40, 0, 0, true),
        0x01 => (8, 0, 0, true),
        _ => (payload.len(), 0, 0, true),
    };
    let handles = payload.get(fixed..).unwrap_or(&[]).as_ptr() as u64;
    let mut linux: Vec<u8> = Vec::with_capacity(fixed + 12);
    match func {
        0x40 => {
            linux.extend_from_slice(&payload[..at]);
            linux.extend_from_slice(&handles.to_ne_bytes());
            linux.extend_from_slice(&payload[at..fixed]);
        }
        0x47 => {
            linux.extend_from_slice(&payload[..at]);
            linux.extend_from_slice(&[0; 4][..gap]);
            linux.extend_from_slice(&handles.to_ne_bytes());
            linux.extend_from_slice(&payload[at..60]);
            linux.extend_from_slice(&payload[64..fixed]);
        }
        0x00 => {
            let len = |at: usize| u64::from_ne_bytes(payload[at..at + 8].try_into().unwrap());
            let (name, date, desc) = (len(16), len(24), len(32));
            let base = payload[fixed..].as_ptr() as u64;
            linux.extend_from_slice(&payload[..16]);
            linux.extend_from_slice(&name.to_ne_bytes());
            linux.extend_from_slice(&base.to_ne_bytes());
            linux.extend_from_slice(&date.to_ne_bytes());
            linux.extend_from_slice(&(base + name).to_ne_bytes());
            linux.extend_from_slice(&desc.to_ne_bytes());
            linux.extend_from_slice(&(base + name + date).to_ne_bytes());
        }
        0x01 => {
            linux.extend_from_slice(&payload[..8]);
            linux.extend_from_slice(&handles.to_ne_bytes());
        }
        // The handles pointer comes first.
        0xC3 | 0xC4 | 0xC5 => {
            linux.extend_from_slice(&handles.to_ne_bytes());
            linux.extend_from_slice(&payload[..fixed]);
        }
        _ => linux.extend_from_slice(payload),
    }
    let request = drm_request(func, linux.len(), read_back);
    if unsafe { ioctl(fd, request, linux.as_mut_ptr()) } < 0 {
        let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(syscall::EIO);
        return Err(syscall::Error::new(errno));
    }
    match func {
        0x40 | 0x47 => {}
        0x00 => {
            payload[..16].copy_from_slice(&linux[..16]);
            for (wire, linux_at) in [(16, 16), (24, 32), (32, 48)] {
                payload[wire..wire + 8].copy_from_slice(&linux[linux_at..linux_at + 8]);
            }
        }
        0x01 => payload[..8].copy_from_slice(&linux[..8]),
        0xC3 | 0xC4 | 0xC5 => payload[..fixed].copy_from_slice(&linux[8..]),
        _ => payload.copy_from_slice(&linux),
    }
    Ok(0)
}

/// v3dd's fmap offsets: the BO ID above bit 32.
const MAP_ID_SHIFT: u32 = 32;
const PAGE: usize = 4096;

/// Words per put/get line, so a line stays a few KiB.
const CHUNK: usize = 512;

pub struct Client {
    file: File,
    /// BO ID -> (address, words) of its mapping, when MAPPED.
    maps: BTreeMap<u32, (usize, usize)>,
}

pub struct Bo {
    pub id: u32,
    pub va: u32,
}

fn hex(s: &str) -> Option<u32> {
    u32::from_str_radix(s.trim_start_matches("0x"), 16).ok()
}

impl Client {
    pub fn open() -> Result<Client, String> {
        OpenOptions::new()
            .read(true)
            .write(true)
            .open(if LIBC.load(Ordering::Relaxed) { "/dev/dri/renderD128" } else { "/scheme/v3d" })
            .map(|file| Client { file, maps: BTreeMap::new() })
            .map_err(|err| format!("/scheme/v3d: {err} (v3dd not running?)"))
    }

    /// One command; the answer, or Err with v3dd's "error ..." line.
    pub fn ask(&mut self, line: &str) -> Result<String, String> {
        let what = line.split(' ').next().unwrap_or(line);
        self.file.write_all(line.as_bytes()).map_err(|err| format!("{what}: write: {err}"))?;
        let mut reply = Vec::new();
        let mut buf = [0u8; 8192];
        while !reply.ends_with(b"\n") {
            // A signal ends v3dd's held read with EINTR; v3dd keeps the
            // wait open, so reading again waits for the same answer.
            let n = match self.file.read(&mut buf) {
                Ok(n) => n,
                Err(err) if err.kind() == std::io::ErrorKind::Interrupted => {
                    eprintln!("v3dprobe: {what}: read interrupted, reading again");
                    continue;
                }
                Err(err) => return Err(format!("{what}: read: {err}")),
            };
            if n == 0 {
                break;
            }
            reply.extend_from_slice(&buf[..n]);
        }
        let reply = String::from_utf8_lossy(&reply).trim().to_string();
        if reply.starts_with("error") || reply.is_empty() {
            Err(format!("{what}: {reply}"))
        } else {
            Ok(reply)
        }
    }

    pub fn bo(&mut self, bytes: usize) -> Result<Bo, String> {
        let reply = self.ask(&format!("bo {bytes}"))?;
        let mut it = reply.split(' ');
        match (it.next().and_then(|s| s.parse().ok()), it.next().and_then(hex)) {
            (Some(id), Some(va)) => {
                let bo = Bo { id, va };
                if MAPPED.load(Ordering::Relaxed) {
                    self.map(&bo, bytes)?;
                }
                Ok(bo)
            }
            _ => Err(format!("bo: bad reply {reply}")),
        }
    }

    /// Maps the whole BO (`bytes` long) into this process.
    pub fn map(&mut self, bo: &Bo, bytes: usize) -> Result<(), String> {
        let size = bytes.div_ceil(PAGE) * PAGE;
        let map = syscall::Map {
            offset: (bo.id as usize) << MAP_ID_SHIFT,
            size,
            flags: syscall::MapFlags::PROT_READ | syscall::MapFlags::PROT_WRITE | syscall::MapFlags::MAP_SHARED,
            address: 0,
        };
        let addr = unsafe { syscall::fmap(self.file.as_raw_fd() as usize, &map) }
            .map_err(|err| format!("fmap of BO {}: {err}", bo.id))?;
        self.maps.insert(bo.id, (addr, size / 4));
        Ok(())
    }

    /// One ioctl of the binary interface (v3d/uapi.rs): a scheme call with
    /// the function number as metadata.
    pub fn ioctl(&mut self, func: u64, payload: &mut [u8]) -> Result<usize, syscall::Error> {
        if LIBC.load(Ordering::Relaxed) {
            return libc_ioctl(self.file.as_raw_fd(), func, payload);
        }
        syscall::call_rw(self.file.as_raw_fd() as usize, payload, syscall::CallFlags::empty(), &[func])
    }

    pub fn unmap(&mut self, id: u32) {
        if let Some((addr, words)) = self.maps.remove(&id) {
            let _ = unsafe { syscall::funmap(addr, words * 4) };
        }
    }

    /// The mapped words `first..first + count` of a BO, if it is mapped.
    fn mapped(&self, bo: &Bo, first: usize, count: usize) -> Option<Result<*mut u32, String>> {
        let &(addr, words) = self.maps.get(&bo.id)?;
        if first + count > words {
            return Some(Err(format!("BO {}: words {first}..{} past the end ({words})", bo.id, first + count)));
        }
        Some(Ok((addr as *mut u32).wrapping_add(first)))
    }

    pub fn free(&mut self, bo: &Bo) -> Result<(), String> {
        self.unmap(bo.id);
        self.ask(&format!("free {}", bo.id)).map(|_| ())
    }

    /// Writes `words` from word `first` of the BO on.
    pub fn put(&mut self, bo: &Bo, first: usize, words: &[u32]) -> Result<(), String> {
        if let Some(ptr) = self.mapped(bo, first, words.len()) {
            let ptr = ptr?;
            for (i, &w) in words.iter().enumerate() {
                unsafe { ptr.add(i).write_volatile(w) };
            }
            barrier();
            return Ok(());
        }
        for (i, chunk) in words.chunks(CHUNK).enumerate() {
            let list: Vec<String> = chunk.iter().map(|w| format!("{w:#x}")).collect();
            self.ask(&format!("put {} {} {}", bo.id, first + i * CHUNK, list.join(" ")))?;
        }
        Ok(())
    }

    /// Writes bytes from byte `at` (a multiple of 4) on, padded with zeros
    /// to whole words.
    pub fn put_bytes(&mut self, bo: &Bo, at: usize, bytes: &[u8]) -> Result<(), String> {
        let words: Vec<u32> = bytes
            .chunks(4)
            .map(|c| {
                let mut w = [0u8; 4];
                w[..c.len()].copy_from_slice(c);
                u32::from_le_bytes(w)
            })
            .collect();
        self.put(bo, at / 4, &words)
    }

    /// Sets `count` words from word `first` on to `value`.
    pub fn fill(&mut self, bo: &Bo, first: usize, count: usize, value: u32) -> Result<(), String> {
        if let Some(ptr) = self.mapped(bo, first, count) {
            let ptr = ptr?;
            for i in 0..count {
                unsafe { ptr.add(i).write_volatile(value) };
            }
            barrier();
            return Ok(());
        }
        self.ask(&format!("fill {} {first} {count} {value:#x}", bo.id)).map(|_| ())
    }

    pub fn get(&mut self, bo: &Bo, first: usize, count: usize) -> Result<Vec<u32>, String> {
        if let Some(ptr) = self.mapped(bo, first, count) {
            let ptr = ptr?;
            return Ok((0..count).map(|i| unsafe { ptr.add(i).read_volatile() }).collect());
        }
        self.get_by_command(bo, first, count)
    }

    /// `get` through v3dd's command, also when the BO is mapped.
    pub fn get_by_command(&mut self, bo: &Bo, first: usize, count: usize) -> Result<Vec<u32>, String> {
        let mut out = Vec::with_capacity(count);
        while out.len() < count {
            let n = CHUNK.min(count - out.len());
            let reply = self.ask(&format!("get {} {} {n}", bo.id, first + out.len()))?;
            out.extend(reply.split(' ').filter_map(hex));
        }
        if out.len() != count {
            return Err(format!("get: {} words, not {count}", out.len()));
        }
        Ok(out)
    }

    /// `put` through v3dd's command, also when the BO is mapped.
    pub fn put_by_command(&mut self, bo: &Bo, first: usize, words: &[u32]) -> Result<(), String> {
        let maps = std::mem::take(&mut self.maps);
        let result = self.put(bo, first, words);
        self.maps = maps;
        result
    }

    /// "submit csd ..." or "submit cl ...": the job number, answered at once
    /// (G2, redox-rpi#668).
    pub fn submit(&mut self, job: &str) -> Result<u64, String> {
        let reply = self.ask(&format!("submit {job}"))?;
        reply.strip_prefix("job ").and_then(|n| n.parse().ok()).ok_or_else(|| format!("submit: bad reply {reply}"))
    }

    /// Waits for a submitted job: its "done ..." line, or Err with v3dd's
    /// "error ..." line.
    pub fn wait(&mut self, job: u64) -> Result<String, String> {
        self.ask(&format!("wait {job}"))
    }

    /// "done US": the job's time in microseconds.
    pub fn job(&mut self, line: &str) -> Result<u128, String> {
        let done = self.ask(line)?;
        Ok(done.split(' ').nth(1).and_then(|s| s.parse().ok()).unwrap_or(0))
    }
}

impl Drop for Client {
    fn drop(&mut self) {
        let ids: Vec<u32> = self.maps.keys().copied().collect();
        for id in ids {
            self.unmap(id);
        }
    }
}

/// The mappings are uncached (v3dd's DMA memory type): a DSB makes the
/// stores reach memory before the next command starts a job on them.
fn barrier() {
    #[cfg(target_arch = "aarch64")]
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}
