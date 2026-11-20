//! A time series of what vc4d and the HVS do, for display stalls
//! (redox-rpi#563). On with `VC4D_TRACE=<file>` or when `/etc/vc4d-trace`
//! exists (its first line names the file; empty: /var/log/vc4d-trace.csv).
//!
//! One CSV line per event:
//!
//!     t_us,event,frame,line,stat,dispstat,x,y,w,h,us
//!
//! - `t_us`: microseconds since vc4d started tracing
//! - `event`: `damage` (a client changed the primary plane: the rectangle),
//!   `defer` (the HVS still shows the last list: the frame waits),
//!   `copy` (copied into the hidden scanout buffer: the rectangle, `us`
//!   the copy time), `list` (a new display list, `us` the time to write
//!   it), `cursor` (the cursor plane moved: x, y), `frame` (the HVS frame
//!   count changed, from a sampler thread every millisecond: `us` the time
//!   since the last change, about 16667 at 60 Hz; more means frames were
//!   missed or the sampler was late), `errors` (new HVS error bits)
//! - `frame`, `line`, `stat`: the channel's frame count, scanout line and
//!   raw SCALER_DISPSTATX when the event happened
//! - `dispstat`: raw SCALER_DISPSTAT (error bits of all channels)
//!
//! A stall shows as a gap in `damage` lines (the client was blocked or
//! idle), long `copy` times, or `frame` lines far above 16667.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::sync::{Mutex, OnceLock};
use std::thread;
use std::time::{Duration, Instant};

use crate::hvs::Hvs;

const DEFAULT_PATH: &str = "/var/log/vc4d-trace.csv";
const SAMPLE: Duration = Duration::from_millis(1);
const FLUSH: Duration = Duration::from_millis(500);

struct Trace {
    out: Mutex<BufWriter<File>>,
    start: Instant,
    /// Hvs is Send but not Sync (a raw register pointer).
    hvs: Mutex<Hvs>,
    index: usize,
}

static TRACE: OnceLock<Trace> = OnceLock::new();

/// The file to trace into, if tracing is on.
pub fn path() -> Option<String> {
    if let Ok(path) = std::env::var("VC4D_TRACE") {
        return Some(path);
    }
    let first = std::fs::read_to_string("/etc/vc4d-trace").ok()?;
    let first = first.lines().next().unwrap_or("").trim();
    Some(if first.is_empty() { DEFAULT_PATH.to_owned() } else { first.to_owned() })
}

/// Starts tracing into `path` and the sampler thread.
pub fn start(path: &str, hvs: Hvs, sampler_hvs: Hvs, index: usize) {
    let file = match File::create(path) {
        Ok(file) => file,
        Err(err) => {
            println!("vc4d: trace: cannot create {path}: {err}");
            return;
        }
    };
    let mut out = BufWriter::with_capacity(1 << 16, file);
    let _ = writeln!(out, "t_us,event,frame,line,stat,dispstat,x,y,w,h,us");
    let trace = Trace { out: Mutex::new(out), start: Instant::now(), hvs: Mutex::new(hvs), index };
    if TRACE.set(trace).is_err() {
        return;
    }
    println!("vc4d: trace: into {path}");
    thread::spawn(move || sampler(sampler_hvs, index));
}

/// Writes one event line; does nothing when tracing is off.
pub fn event(event: &str, rect: (u32, u32, u32, u32), us: u64) {
    let Some(t) = TRACE.get() else { return };
    let (stat, dispstat) = {
        let hvs = t.hvs.lock().unwrap();
        (hvs.channel_stat(t.index), hvs.dispstat())
    };
    line(t, event, stat, dispstat, rect, us);
}

fn line(t: &Trace, event: &str, stat: u32, dispstat: u32, rect: (u32, u32, u32, u32), us: u64) {
    let now = t.start.elapsed().as_micros();
    let mut out = t.out.lock().unwrap();
    let _ = writeln!(
        out,
        "{now},{event},{},{},{stat:#010x},{dispstat:#010x},{},{},{},{},{us}",
        (stat >> 12) & 0x3f,
        stat & 0xfff,
        rect.0,
        rect.1,
        rect.2,
        rect.3
    );
}

/// Every millisecond: a `frame` line when the frame count changed, an
/// `errors` line when error bits appeared; flushes the file twice a second.
fn sampler(hvs: Hvs, index: usize) {
    let Some(t) = TRACE.get() else { return };
    let error_bits = crate::hvs::dispstat_errors(index);
    let mut frame = hvs.frame_count(index);
    let mut errors = hvs.dispstat() & error_bits;
    let mut changed = Instant::now();
    let mut flushed = Instant::now();
    loop {
        thread::sleep(SAMPLE);
        let stat = hvs.channel_stat(index);
        let dispstat = hvs.dispstat();
        let now_frame = hvs.frame_count(index);
        if now_frame != frame {
            let us = changed.elapsed().as_micros() as u64;
            changed = Instant::now();
            frame = now_frame;
            line(t, "frame", stat, dispstat, (0, 0, 0, 0), us);
        }
        if dispstat & error_bits != errors {
            errors = dispstat & error_bits;
            line(t, "errors", stat, dispstat, (0, 0, 0, 0), 0);
        }
        if flushed.elapsed() >= FLUSH {
            flushed = Instant::now();
            let _ = t.out.lock().unwrap().flush();
        }
    }
}
