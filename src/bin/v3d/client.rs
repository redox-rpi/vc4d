// SPDX-License-Identifier: GPL-2.0-only
//
// A client of v3dd (redox-rpi#598): the line protocol of /scheme/v3d, see
// src/bin/v3dd.rs. One command per write, one answer per read.

use std::fs::{File, OpenOptions};
use std::io::{Read, Write};

/// Words per put/get line, so a line stays a few KiB.
const CHUNK: usize = 512;

pub struct Client {
    file: File,
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
            .map(|file| Client { file })
            .map_err(|err| format!("/scheme/v3d: {err} (v3dd not running?)"))
    }

    /// One command; the answer, or Err with v3dd's "error ..." line.
    pub fn ask(&mut self, line: &str) -> Result<String, String> {
        let what = line.split(' ').next().unwrap_or(line);
        self.file.write_all(line.as_bytes()).map_err(|err| format!("{what}: write: {err}"))?;
        let mut reply = Vec::new();
        let mut buf = [0u8; 8192];
        while !reply.ends_with(b"\n") {
            let n = self.file.read(&mut buf).map_err(|err| format!("{what}: read: {err}"))?;
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
            (Some(id), Some(va)) => Ok(Bo { id, va }),
            _ => Err(format!("bo: bad reply {reply}")),
        }
    }

    pub fn free(&mut self, bo: &Bo) -> Result<(), String> {
        self.ask(&format!("free {}", bo.id)).map(|_| ())
    }

    /// Writes `words` from word `first` of the BO on.
    pub fn put(&mut self, bo: &Bo, first: usize, words: &[u32]) -> Result<(), String> {
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
        self.ask(&format!("fill {} {first} {count} {value:#x}", bo.id)).map(|_| ())
    }

    pub fn get(&mut self, bo: &Bo, first: usize, count: usize) -> Result<Vec<u32>, String> {
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

    /// "done US": the job's time in microseconds.
    pub fn job(&mut self, line: &str) -> Result<u128, String> {
        let done = self.ask(line)?;
        Ok(done.split(' ').nth(1).and_then(|s| s.parse().ok()).unwrap_or(0))
    }
}
