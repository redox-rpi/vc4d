// SPDX-License-Identifier: GPL-2.0-only
//
// A check that needs no screen and no person: it shows a test picture with
// a cursor plane on the HVS channel, moves the cursor, and reads back from
// the HVS that it took the lists, keeps scanning out and reports no errors.
// Then a thread watches the channel for two minutes while fbcond and
// Orbital use it. Everything goes to the log (serial), lines start with
// "vc4d: check:".
//
// The register meanings follow Linux v6.12 drivers/gpu/drm/vc4/vc4_regs.h
// (SCALER_DISPSTAT, SCALER_DISPSTATX, SCALER_DISPLACTX) and the underrun
// handling of vc4_hvs.c (vc4_hvs_irq_handler()).

use std::fs;
use std::thread;
use std::time::{Duration, Instant};

use common::dma::Dma;

use crate::hvs::{self, Channel, Hvs, Plane};
use crate::shot;

/// Where the check's lists go: apart from the adapter's.
const CHECK_SLOTS: [usize; 2] = [0x880, 0x8c0];

/// When the watch thread looks, after the handover.
const WATCH_SECONDS: [u64; 6] = [2, 5, 10, 30, 60, 120];

fn dma_buffer(len: usize) -> Option<Dma<[u32]>> {
    match Dma::<[u32]>::zeroed_slice(len) {
        Ok(dma) => Some(unsafe { dma.assume_init() }),
        Err(err) => {
            println!("vc4d: check: no contiguous memory for {len} pixels: {err}");
            None
        }
    }
}

/// Waits for `frames` new frames of the channel; false on timeout.
fn wait_frames(hvs: &Hvs, index: usize, frames: u32) -> bool {
    let start = Instant::now();
    let mut last = hvs.frame_count(index);
    let mut seen = 0;
    while seen < frames {
        if start.elapsed() > Duration::from_millis(100 * u64::from(frames) + 200) {
            return false;
        }
        thread::sleep(Duration::from_millis(1));
        let now = hvs.frame_count(index);
        if now != last {
            seen += 1;
            last = now;
        }
    }
    true
}

struct Results {
    passed: u32,
    failed: u32,
}

impl Results {
    fn report(&mut self, ok: bool, what: &str) {
        println!("vc4d: check: {} {what}", if ok { "PASS" } else { "FAIL" });
        if ok {
            self.passed += 1;
        } else {
            self.failed += 1;
        }
    }
}

/// Shows the list on the channel and checks that the HVS took it: the
/// channel points at it, the words read back as written, the active word
/// is inside it and frames go on.
fn show(hvs: &Hvs, channel: Channel, slot: usize, list: &[u32], fake: bool, r: &mut Results) {
    let index = channel.index;
    hvs.write_dlist(slot, list);
    hvs.set_list_start(index, slot);
    let frames_go_on = fake || wait_frames(hvs, index, 2);
    let back = hvs.read_list(slot, list.len());
    r.report(hvs.list_start(index) == slot, &format!("channel {index} list start is {slot}"));
    // The HVS keeps its own state in the context words while it scans the
    // plane out (seen on the Pi, row 220a): a changed context word shows it
    // read the list. All other words must read back as written.
    let same = back.len() == list.len()
        && back.iter().zip(list).all(|(&b, &w)| w == hvs::CONTEXT || b == w);
    let used = back.iter().zip(list).filter(|&(&b, &w)| w == hvs::CONTEXT && b != w).count();
    r.report(
        same,
        &format!("list at {slot} reads back as written, {used} context words filled in by the HVS: {back:08x?}"),
    );
    if !fake {
        r.report(frames_go_on, "frame count advances (2 frames)");
        let active = hvs.active(index);
        r.report(
            (slot..=slot + list.len()).contains(&active),
            &format!("active word {active} is in the list {slot}..={}", slot + list.len()),
        );
    }
}

/// Runs the check; true if every part passed. With `fake`, there is no
/// hardware behind the registers: frames and the active word are skipped.
pub fn self_check(hvs: &Hvs, channel: Channel, ram_bus_offset: usize, fake: bool) -> bool {
    let index = channel.index;
    let mut r = Results { passed: 0, failed: 0 };
    let firmware_list = hvs.list_start(index);
    println!("vc4d: check: start on channel {index}, {}", hvs.describe(index));
    println!("vc4d: check: before: errors {:?}, {}", hvs.errors(index), hvs.describe_global());

    let (width, height) = (channel.width as usize, channel.height as usize);
    let (Some(mut screen), Some(mut cursor)) = (dma_buffer(width * height), dma_buffer(64 * 64)) else {
        r.report(false, "buffers in contiguous memory");
        return false;
    };
    // Eight colour bars, and a white cursor square with a black edge.
    const BARS: [u32; 8] = [
        0xffffffff, 0xffffff00, 0xff00ffff, 0xff00ff00, 0xffff00ff, 0xffff0000, 0xff0000ff, 0xff000000,
    ];
    for (i, pixel) in screen.iter_mut().enumerate() {
        *pixel = BARS[(i % width) * 8 / width];
    }
    for (i, pixel) in cursor.iter_mut().enumerate() {
        let (x, y) = (i % 64, i / 64);
        let edge = x < 2 || y < 2 || x >= 62 || y >= 62;
        *pixel = if edge { 0xff000000 } else { 0xffffffff };
    }
    let primary = Plane {
        bus: (screen.physical() + ram_bus_offset) as u32,
        pitch: (width * 4) as u32,
        x: 0,
        y: 0,
        width: width as u32,
        height: height as u32,
        alpha: false,
    };
    let at = |x: u32, y: u32| Plane {
        bus: (cursor.physical() + ram_bus_offset) as u32,
        pitch: 64 * 4,
        x,
        y,
        width: 64.min(channel.width - x),
        height: 64.min(channel.height - y),
        alpha: true,
    };
    let moves = [(16, 16), (channel.width / 2, channel.height / 2)];

    hvs.clear_errors(index);
    for (n, &(x, y)) in moves.iter().enumerate() {
        let list = hvs::display_list(&[primary, at(x, y)]);
        let slot = CHECK_SLOTS[n % 2];
        show(hvs, channel, slot, &list, fake, &mut r);
        // Position word 0 of the cursor plane, the second plane's second word.
        let start = hvs::plane_start(hvs.dlist_word(slot + 8));
        r.report(start == (x, y), &format!("cursor at {start:?} in the HVS, wanted ({x}, {y})"));
    }
    if !fake {
        // A few more frames on the test picture, for underruns to show.
        wait_frames(hvs, index, 10);
    }
    let errors = hvs.errors(index);
    r.report(errors.is_empty(), &format!("no HVS errors on channel {index}: {errors:?}"));

    hvs.set_list_start(index, firmware_list);
    if !fake {
        wait_frames(hvs, index, 2);
    }
    println!(
        "vc4d: check: back on the firmware list {firmware_list}; {} passed, {} failed",
        r.passed, r.failed
    );
    r.failed == 0
}

/// How often the watch thread looks for a screenshot request.
const SHOT_POLL: Duration = Duration::from_secs(3);

/// After the checks, how often the watch thread reads (and clears) the
/// HVS error bits, and how often it logs a summary.
const ERROR_POLL: Duration = Duration::from_secs(1);
const ERROR_SUMMARY: Duration = Duration::from_secs(60);

/// Every second the watch thread reads the channel's live FIFO state this
/// long, as fast as it can (about 2 % of a core).
const FIFO_SAMPLE: Duration = Duration::from_millis(20);

/// `echo BASE TOP > /tmp/vc4d.cob` gives our channel pixels BASE..=TOP of
/// the COB, to see whether a larger FIFO stops underruns.
const COB_REQUEST: &str = "/tmp/vc4d.cob";

/// FIFO reads while the channel ran, and those that found it empty, by
/// eighth of the screen; the last bucket is lines after the last one.
#[derive(Default)]
struct FifoStats {
    reads: u64,
    empty: u64,
    by_part: [u64; 9],
}

impl FifoStats {
    /// Reads the FIFO state for FIFO_SAMPLE; returns this sample's numbers.
    fn sample(&mut self, hvs: &Hvs, index: usize, height: u32) -> FifoStats {
        let mut now = FifoStats::default();
        let start = Instant::now();
        while start.elapsed() < FIFO_SAMPLE {
            let fifo = hvs.fifo(index);
            if fifo.mode != hvs::MODE_RUN {
                continue;
            }
            now.reads += 1;
            if fifo.empty && !fifo.full {
                now.empty += 1;
                let part = if fifo.line >= height { 8 } else { (fifo.line * 8 / height.max(1)) as usize };
                now.by_part[part] += 1;
            }
        }
        self.reads += now.reads;
        self.empty += now.empty;
        for (sum, n) in self.by_part.iter_mut().zip(now.by_part) {
            *sum += n;
        }
        now
    }
}

/// Applies a COB request if there is one.
fn serve_cob(hvs: &Hvs, index: usize) {
    let Ok(text) = fs::read_to_string(COB_REQUEST) else {
        return;
    };
    let _ = fs::remove_file(COB_REQUEST);
    let numbers: Vec<u32> = text.split_whitespace().filter_map(|w| w.parse().ok()).collect();
    match numbers[..] {
        [base, top] if base < top && top < hvs::COB_SIZE => {
            let old = hvs.cob(index);
            hvs.set_cob(index, base, top);
            println!("vc4d: cob: channel {index} {}..={} -> {base}..={top}: {}", old.0, old.1, hvs.describe_cob());
        }
        _ => println!("vc4d: cob: want \"BASE TOP\" below {} in {COB_REQUEST}, got {text:?}", hvs::COB_SIZE),
    }
}

/// Logs the channel's state a few times over two minutes after the
/// handover, while fbcond and Orbital draw: the list it points at, whether
/// frames go on, and new errors. Then it keeps reading the error bits every
/// second, logging the first underruns of each minute with their time, a
/// summary every minute, and changes to its display list by others
/// (Pi test #275: bands on HDMI while Orbital drew, vc4shot clean), and
/// answers screenshot requests. Every second it also reads the live FIFO
/// state for 20 ms: the latched underrun bit (EUFLOW) may never show,
/// because the firmware still takes the HVS interrupts and clears it.
pub fn watch(hvs: Hvs, channel: Channel, fake: bool, checking: bool, shown: shot::Shown) {
    let index = channel.index;
    let (width, height) = (channel.width as usize, channel.height as usize);
    thread::spawn(move || {
        println!("vc4d: cob: {}", hvs.describe_cob());
        hvs.clear_errors(index);
        let start = Instant::now();
        let mut frame = hvs.frame_count(index);
        for seconds in WATCH_SECONDS.iter().copied().filter(|_| checking) {
            while start.elapsed() < Duration::from_secs(seconds) {
                let wait = Duration::from_secs(seconds).saturating_sub(start.elapsed());
                thread::sleep(wait.min(SHOT_POLL));
                shot::serve(&shown, width, height);
            }
            let now = hvs.frame_count(index);
            let errors = hvs.errors(index);
            let moving = fake || now != frame || wait_frames(&hvs, index, 1);
            println!(
                "vc4d: check: after {seconds} s: {} frames {}, errors {errors:?}, {}",
                if errors.is_empty() && moving { "PASS" } else { "FAIL" },
                if fake { "(fake)" } else if moving { "advance" } else { "STOPPED" },
                hvs.describe(index)
            );
            frame = now;
        }
        hvs.clear_errors(index);
        let (mut shot_at, mut summary_at) = (Instant::now(), Instant::now());
        let (mut polls, mut with_errors) = (0u32, 0u32);
        let mut was_elsewhere = false;
        // List findings per minute; only the first few are logged.
        let mut list_findings = 0u32;
        let mut fifo = FifoStats::default();
        let mut fifo_logged = 0u32;
        loop {
            thread::sleep(ERROR_POLL);
            polls += 1;
            serve_cob(&hvs, index);
            if !fake {
                let now = fifo.sample(&hvs, index, height as u32);
                if now.empty > 0 {
                    fifo_logged += 1;
                    if fifo_logged <= 5 {
                        println!(
                            "vc4d: fifo: {} s: empty in {} of {} reads, by eighth {:?}",
                            start.elapsed().as_secs(),
                            now.empty,
                            now.reads,
                            now.by_part
                        );
                    }
                }
            }
            // Something else (the firmware) writing the list memory or
            // pointing the channel elsewhere would show garbage planes.
            if let Some((word, written, now)) = hvs.changed_list_word() {
                list_findings += 1;
                if list_findings <= 3 {
                    println!("vc4d: scanout: {} s: list word {word} is {now:#010x}, written {written:#010x}", start.elapsed().as_secs());
                }
            }
            // Twice in a row: the adapter points the channel right after
            // writing a list, so one read may fall in between.
            let list = hvs.list_start(index);
            let elsewhere = hvs.written_start().is_some_and(|written| written != list);
            if elsewhere && was_elsewhere {
                list_findings += 1;
                if list_findings <= 3 {
                    println!("vc4d: scanout: {} s: channel reads the list at {list}, not ours", start.elapsed().as_secs());
                }
            }
            was_elsewhere = elsewhere;
            let errors = hvs.errors(index);
            if !errors.is_empty() {
                with_errors += 1;
                if with_errors <= 5 {
                    println!(
                        "vc4d: scanout: {} s: errors {errors:?}, {}, {}",
                        start.elapsed().as_secs(),
                        hvs.describe(index),
                        hvs.describe_global()
                    );
                }
                hvs.clear_errors(index);
            }
            if shot_at.elapsed() >= SHOT_POLL {
                shot::serve(&shown, width, height);
                shot_at = Instant::now();
            }
            if summary_at.elapsed() >= ERROR_SUMMARY {
                println!(
                    "vc4d: scanout: {} s: errors in {with_errors} of {polls} reads, {list_findings} list findings in the last minute",
                    start.elapsed().as_secs()
                );
                if !fake {
                    println!(
                        "vc4d: fifo: {} s: empty in {} of {} reads in the last minute, by eighth {:?}",
                        start.elapsed().as_secs(),
                        fifo.empty,
                        fifo.reads,
                        fifo.by_part
                    );
                }
                (polls, with_errors, list_findings, summary_at) = (0, 0, 0, Instant::now());
                (fifo, fifo_logged) = (FifoStats::default(), 0);
            }
        }
    });
}
