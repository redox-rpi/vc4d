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
            .open("/scheme/v3d")
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

    fn unmap(&mut self, id: u32) {
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
