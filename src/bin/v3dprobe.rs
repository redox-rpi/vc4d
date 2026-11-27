// SPDX-License-Identifier: GPL-2.0-only
//
// v3dprobe: step 0 of V3D on the Pi 4 and 400 (BCM2711, V3D 4.2). It powers
// the 3D block up as Linux does and reads its identification registers.
// With --csd and --render it runs the step 1 and step 2 jobs.
//
// Translated from Linux v6.12:
// - drivers/pmdomain/bcm/bcm2835-power.c: bcm2835_asb_power_on() for
//   BCM2835_POWER_DOMAIN_GRAFX_V3D. It releases the V3D reset in PM_GRAFX,
//   then opens the async AXI bridges, master first, at the BCM2711's
//   "rpivid_asb" registers. Linux skips power_on on the BCM2711.
// - drivers/pmdomain/bcm/raspberrypi-power.c: on the BCM2711 the firmware
//   powers V3D (RPI_POWER_DOMAIN_V3D, SET_DOMAIN_STATE). The firmware
//   leaves it off at boot (Test 547), so v3dprobe asks rpi-firmwared
//   (/scheme/rpi-firmware/v3d_power) when GRAFX has no power.
// - drivers/gpu/drm/v3d/v3d_regs.h, v3d_drv.c: V3D_HUB_IDENT0..3,
//   V3D_CTL_IDENT0..2 and what v3d_platform_drm_probe() logs.
//
// v3dprobe [--csd] [--render [--triangle | --triangle-math] [--show SECONDS]]
//          [--off] [--no-watchdog]
// v3dprobe --via-v3dd [--loops N] [--render [--triangle | --triangle-math] | --faults
//          | --queue [--clients C] | --ioctl] [--mapped]
//   --ioctl: v3dd's binary interface (G2 step 3, redox-rpi#668), Linux's
//     V3D ioctls as scheme calls: N compute jobs by SUBMIT_CSD and WAIT_BO,
//     wrong arguments, an endless job (WAIT_BO: ETIME, then EIO)
//   --queue: v3dd's job queue (G2, redox-rpi#668): C clients (4) at once,
//     each with N jobs (at most 64) in flight; an endless job in the middle
//     of a queue; a client that closes with jobs in flight; a client's job
//     cannot reach another client's BO; no BOs and no address spaces left
//   --show: the rendered picture in an Orbital window for SECONDS (step 2c)
//   --render: step 2 (redox-rpi#596): bin and render control lists, see
//     v3d/render.rs; clear only, or with --triangle a red triangle whose
//     coordinates the CPU precomputes, with --triangle-math one whose
//     shaders compute the screen coordinates
//   --csd: step 1 (redox-rpi#595): MMU page table and one compute job, see
//     v3d/csd.rs
//   --off: close the bridges, assert the reset and power V3D off afterwards
//   --via-v3dd: step 3 (redox-rpi#598): the --csd job N times (default
//     100) through v3dd and /scheme/v3d; touches no registers itself.
//     With --render, the --render frame N times instead (step 3b).
//     With --faults, a job with a bad address and a job that never ends,
//     then jobs that must still pass (step 3c).
//     With --mapped, the BOs are mapped into v3dprobe and written and read
//     there instead of through put/get (step 3e, redox-rpi#666); a check of
//     the mappings runs first and a check for leaked BOs last.
//   --no-watchdog: do not arm the SoC watchdog around the V3D accesses
//
// The first V3D accesses can hang the bus (a block without power or clock).
// The Pi 400 in the lab has no remote power switch, so v3dprobe arms the
// PM watchdog (Linux's bcm2835_wdt) for WATCHDOG_SECS before it releases
// the reset and stops it after the identification: a hang resets the SoC
// instead of needing a person.

use std::env;
use std::fs;
use std::io::Write;
use std::thread;
use std::time::{Duration, Instant};

use fdt::Fdt;

#[path = "v3d/client.rs"]
mod client;
#[path = "v3d/csd.rs"]
mod csd;
#[path = "v3d/render.rs"]
mod render;
#[path = "v3d/uapi.rs"]
#[allow(dead_code)]
mod uapi;

// The Pi 400's devicetree (bcm2711-rpi-400.dtb): /v3dbus/v3d@7ec04000 with
// reg-names hub and core0, and /v3dbus ranges 0x7c500000 -> 0xfc500000. The
// power manager (watchdog@7e100000, reg-names pm, asb, rpivid_asb) is in
// /soc (0x7e000000 -> 0xfe000000).
const HUB_PHYS: usize = 0xfec0_0000;
const CORE0_PHYS: usize = 0xfec0_4000;
const PM_PHYS: usize = 0xfe10_0000;
const RPIVID_ASB_PHYS: usize = 0xfec1_1000;

const PM_GRAFX: usize = 0x10c;
const PM_V3DRSTN: u32 = 1 << 6;
const PM_POWOK: u32 = 1 << 1;
const PM_POWUP: u32 = 1 << 0;
const PM_PASSWORD: u32 = 0x5a00_0000;
// drivers/watchdog/bcm2835_wdt.c
const PM_RSTC: usize = 0x1c;
const PM_RSTS: usize = 0x20;
const PM_WDOG: usize = 0x24;
const PM_WDOG_TIME_SET: u32 = 0x000f_ffff;
const PM_RSTC_WRCFG_CLR: u32 = 0xffff_ffcf;
const PM_RSTC_WRCFG_FULL_RESET: u32 = 0x0000_0020;
const PM_RSTC_RESET: u32 = 0x0000_0102;
const WATCHDOG_SECS: u32 = 10;

const ASB_V3D_S_CTRL: usize = 0x08;
const ASB_V3D_M_CTRL: usize = 0x0c;
const ASB_AXI_BRDG_ID: usize = 0x20;
const ASB_REQ_STOP: u32 = 1 << 0;
const ASB_ACK: u32 = 1 << 1;
const BCM2835_BRDG_ID: u32 = 0x6272_6467;

const V3D_HUB_IDENT0: usize = 0x08;
const V3D_HUB_IDENT1: usize = 0x0c;
const V3D_HUB_IDENT2: usize = 0x10;
const V3D_HUB_IDENT3: usize = 0x14;
const V3D_CTL_IDENT0: usize = 0x00;
const V3D_CTL_IDENT1: usize = 0x04;
const V3D_CTL_IDENT2: usize = 0x08;

struct Regs {
    base: *mut u32,
    name: &'static str,
}

impl Regs {
    fn map(phys: usize, size: usize, name: &'static str) -> Regs {
        say(&format!("map {name} at {phys:#x}"));
        let base = unsafe { common::physmap(phys, size, common::Prot::RW, common::MemoryType::DeviceMemory) }
            .unwrap_or_else(|err| fail(&format!("cannot map {name} at {phys:#x}: {err:?}")));
        Regs { base: base as *mut u32, name }
    }

    fn read(&self, offset: usize) -> u32 {
        unsafe { self.base.add(offset / 4).read_volatile() }
    }

    fn write(&self, offset: usize, value: u32) {
        unsafe { self.base.add(offset / 4).write_volatile(value) }
    }

    fn show(&self, offset: usize, what: &str) -> u32 {
        // Printed before the read too: if the read aborts (a block without
        // power or clock), the log says which one.
        say(&format!("read {} {offset:#05x} ({what})", self.name));
        let v = self.read(offset);
        say(&format!("  = {v:#010x}"));
        v
    }
}

/// Step 3c (redox-rpi#598): a job with a bad address and a job that never
/// ends; v3dd must report both and run the next jobs right.
fn faults_via_v3dd() -> bool {
    let mut v3d = match client::Client::open() {
        Ok(v3d) => v3d,
        Err(err) => {
            say(&format!("faults: FAIL: {err}"));
            return false;
        }
    };
    let mut ok = true;
    let mut check = |name: &str, pass: bool, what: String| {
        say(&format!("faults: {name}: {} {what}", if pass { "PASS" } else { "FAIL" }));
        ok &= pass;
    };
    match csd::bad_address_via_v3dd(&mut v3d) {
        Ok(answer) => check("bad address", answer.contains("mmu-fault"), format!("(v3dd: {answer})")),
        Err(err) => check("bad address", false, err),
    }
    check("CSD after the fault", csd::via_v3dd(10), String::new());
    match render::endless_via_v3dd(&mut v3d) {
        Ok(answer) => check(
            "endless job",
            answer.contains("timeout") && answer.contains("reset"),
            format!("(v3dd: {answer})"),
        ),
        Err(err) => check("endless job", false, err),
    }
    check("CSD after the reset", csd::via_v3dd(10), String::new());
    check("render after the reset", render::via_v3dd(render::Draw::Clear, 10), String::new());
    if let Ok(stats) = v3d.ask("stats") {
        say(&format!("faults: v3dd stats: {stats}"));
    }
    say(if ok {
        "faults: PASS: the MMU fault came back, the endless job was reset, the next jobs ran right"
    } else {
        "faults: FAIL"
    });
    ok
}

/// Step 3e (redox-rpi#666): what is written through a mapping is what v3dd
/// reads, and the other way round; a BO freed while mapped stays usable
/// memory for the client.
fn mapped_check() -> Result<String, String> {
    let mut v3d = client::Client::open()?;
    let bo = v3d.bo(2 * 4096)?;
    let words = 2 * 4096 / 4;
    let a: Vec<u32> = (0..words as u32).map(|i| 0x5a00_0000 ^ i.wrapping_mul(0x9e37_79b9)).collect();
    v3d.put(&bo, 0, &a)?;
    if v3d.get_by_command(&bo, 0, words)? != a {
        return Err("v3dd does not read what was written through the mapping".into());
    }
    let b: Vec<u32> = a.iter().map(|w| !w).collect();
    v3d.put_by_command(&bo, 0, &b)?;
    if v3d.get(&bo, 0, words)? != b {
        return Err("the mapping does not show what v3dd wrote".into());
    }
    // Freed by command while still mapped: V3D loses it, the client keeps
    // its pages until it unmaps them (when v3d is dropped).
    v3d.ask(&format!("free {}", bo.id))?;
    v3d.put(&bo, 0, &a)?;
    if v3d.get(&bo, 0, words)? != a {
        return Err("a BO freed while mapped changed under the client".into());
    }
    v3d.ask("stats")
}

/// BOs v3dd still holds besides its overflow memory: none once every
/// client has closed.
fn leak_check() -> Result<String, String> {
    let stats = client::Client::open()?.ask("stats")?;
    let field = |name: &str| -> u64 {
        let mut it = stats.split(' ');
        while let Some(key) = it.next() {
            if key == name {
                return it.next().and_then(|v| v.parse().ok()).unwrap_or(0);
            }
        }
        0
    };
    let held = field("bos").saturating_sub(field("frees"));
    if held == field("overflow_chunks") {
        Ok(stats)
    } else {
        Err(format!("{held} BOs held, {} of them overflow memory: {stats}", field("overflow_chunks")))
    }
}

/// G2 (redox-rpi#668): one client of the queue test. All its jobs go in
/// before it waits for the first; its code BO is freed right after the
/// submits, and v3dd must keep it until the jobs are done.
fn queue_client(n: u32, jobs: u32) -> Result<(), String> {
    let mut v3d = client::Client::open()?;
    let code = v3d.bo(4096)?;
    v3d.put(&code, 0, &csd::shader_words())?;
    let mut work = Vec::new();
    for j in 0..jobs {
        let unif = v3d.bo(4096)?;
        let result = v3d.bo(4096)?;
        let constant = csd::constant(n << 12 | j << 5);
        v3d.fill(&result, 0, 16, 0xdead_0000)?;
        v3d.put(&unif, 0, &[result.va, constant])?;
        let job = v3d.submit(&csd::job_line(code.va, unif.va))?;
        work.push((job, unif, result, constant));
    }
    v3d.free(&code)?;
    for (job, unif, result, constant) in work {
        v3d.wait(job).map_err(|err| format!("client {n}: job {job}: {err}"))?;
        let got = v3d.get(&result, 0, 16)?;
        if got != csd::want(constant) {
            return Err(format!("client {n}: job {job}: result {got:#x?}, want {:#x?}", csd::want(constant)));
        }
        v3d.free(&unif)?;
        v3d.free(&result)?;
    }
    Ok(())
}

/// G2: a good job, an endless one, a good one, submitted back to back:
/// only the endless one fails (timeout, reset), the one after it runs.
fn queue_hung() -> Result<String, String> {
    let mut v3d = client::Client::open()?;
    let code = v3d.bo(4096)?;
    v3d.put(&code, 0, &csd::shader_words())?;
    let mut good = Vec::new();
    for k in 0..2u32 {
        let unif = v3d.bo(4096)?;
        let result = v3d.bo(4096)?;
        let constant = csd::constant(0xf000 | k << 5);
        v3d.fill(&result, 0, 16, 0xdead_0000)?;
        v3d.put(&unif, 0, &[result.va, constant])?;
        good.push((unif, result, constant));
    }
    let (endless_bo, endless) = render::endless_job(&mut v3d)?;
    let a = v3d.submit(&csd::job_line(code.va, good[0].0.va))?;
    let hung = v3d.submit(&endless)?;
    let b = v3d.submit(&csd::job_line(code.va, good[1].0.va))?;
    v3d.wait(a).map_err(|err| format!("the job before the endless one: {err}"))?;
    let answer = match v3d.wait(hung) {
        Ok(done) => return Err(format!("the endless job ended: {done}")),
        Err(err) if err.contains("timeout") && err.contains("reset") => err,
        Err(err) => return Err(format!("the endless job: not a timeout: {err}")),
    };
    v3d.wait(b).map_err(|err| format!("the job after the endless one: {err}"))?;
    for (unif, result, constant) in &good {
        if v3d.get(result, 0, 16)? != csd::want(*constant) {
            return Err(format!("a good job next to the endless one: wrong result at {:#x}", result.va));
        }
        v3d.free(unif)?;
        v3d.free(result)?;
    }
    v3d.free(&endless_bo)?;
    v3d.free(&code)?;
    Ok(answer)
}

/// G2: a client that closes with jobs in flight. v3dd drops the queued
/// ones and frees the BOs once the running one is done.
fn queue_close() -> Result<(), String> {
    let mut v3d = client::Client::open()?;
    let code = v3d.bo(4096)?;
    v3d.put(&code, 0, &csd::shader_words())?;
    let unif = v3d.bo(4096)?;
    let result = v3d.bo(4096)?;
    v3d.put(&unif, 0, &[result.va, csd::constant(0xe000)])?;
    for _ in 0..4 {
        v3d.submit(&csd::job_line(code.va, unif.va))?;
    }
    Ok(())
}

/// G2 step 2: each client has its own address space. Client A's third BO
/// lies at an address where client B has none: B's job storing there must
/// end in an MMU fault, and A's BO must stay as it was.
fn queue_isolation() -> Result<String, String> {
    let mut a = client::Client::open()?;
    let mut b = client::Client::open()?;
    let a_bos = [a.bo(4096)?, a.bo(4096)?, a.bo(4096)?];
    a.fill(&a_bos[2], 0, 16, 0x1111_1111)?;
    let code = b.bo(4096)?;
    let unif = b.bo(4096)?;
    if code.va != a_bos[0].va {
        return Err(format!("B's first BO at {:#x}, A's at {:#x}: one shared address space", code.va, a_bos[0].va));
    }
    b.put(&code, 0, &csd::shader_words())?;
    b.put(&unif, 0, &[a_bos[2].va, csd::constant(0xd000)])?;
    let answer = b.job(&csd::job_line(code.va, unif.va));
    let left = a.get(&a_bos[2], 0, 16)?;
    let result = match answer {
        Ok(us) => Err(format!("B's job stored to {:#x} and passed after {us} us", a_bos[2].va)),
        Err(err) if !err.contains("mmu-fault") => Err(format!("B's job: not an MMU fault: {err}")),
        Err(_) if left.iter().any(|&w| w != 0x1111_1111) => Err(format!("A's BO changed: {left:#x?}")),
        Err(err) => Ok(format!("(both start at {:#x}; v3dd: {err})", code.va)),
    };
    for bo in &a_bos {
        a.free(bo)?;
    }
    b.free(&code)?;
    b.free(&unif)?;
    result
}

/// A number from a stats line.
fn stat(stats: &str, name: &str) -> Option<u64> {
    let mut it = stats.split(' ');
    while let Some(key) = it.next() {
        if key == name {
            return it.next()?.parse().ok();
        }
    }
    None
}

/// G2: the queue test (see queue_client, queue_hung, queue_close), then
/// no client BO left in v3dd.
fn queue_via_v3dd(clients: u32, jobs: u32) -> bool {
    let mut ok = true;
    let mut check = |name: &str, result: Result<String, String>| {
        match &result {
            Ok(what) => say(&format!("queue: {name}: PASS {what}")),
            Err(err) => say(&format!("queue: {name}: FAIL {err}")),
        }
        ok &= result.is_ok();
    };
    let start = Instant::now();
    let (results, hung) = thread::scope(|scope| {
        let workers: Vec<_> = (0..clients).map(|n| scope.spawn(move || queue_client(n, jobs))).collect();
        let hung = queue_hung();
        let results: Vec<Result<(), String>> =
            workers.into_iter().map(|w| w.join().unwrap_or_else(|_| Err("client thread panicked".into()))).collect();
        (results, hung)
    });
    let failed: Vec<String> = results.into_iter().filter_map(Result::err).collect();
    check(
        &format!("{clients} clients with {jobs} jobs in flight each"),
        if failed.is_empty() {
            Ok(format!("({} jobs right in {} ms)", clients * jobs, start.elapsed().as_millis()))
        } else {
            Err(failed.join("; "))
        },
    );
    check("an endless job in the middle of a queue", hung.map(|answer| format!("(v3dd: {answer})")));
    check("a client closed with jobs in flight", queue_close().map(|()| String::new()));
    check("another client's BO is out of reach", queue_isolation());
    check("no BOs left", nothing_left());
    say(if ok { "queue: PASS" } else { "queue: FAIL" });
    ok
}

/// Once v3dd is idle (a closed client's running job ends within its
/// timeout): no client BO, zombie or address space left.
fn nothing_left() -> Result<String, String> {
    let deadline = Instant::now() + Duration::from_secs(3);
    let stats = loop {
        let stats = client::Client::open().and_then(|mut v3d| v3d.ask("stats"));
        let idle = stats.as_ref().is_ok_and(|s| {
            stat(s, "queued") == Some(0) && stat(s, "zombies") == Some(0) && s.contains(" running - ")
        });
        if idle || Instant::now() > deadline {
            break stats;
        }
        thread::sleep(Duration::from_millis(50));
    };
    stats.and_then(|s| match (stat(&s, "live_bos"), stat(&s, "zombies"), stat(&s, "queued"), stat(&s, "spaces")) {
        // The stats client itself has no BO, so no space.
        (Some(0), Some(0), Some(0), Some(0)) => Ok(format!("({s})")),
        _ => Err(format!("({s})")),
    })
}

/// G2 step 3 (redox-rpi#668): the binary interface, Linux's V3D ioctls as
/// scheme calls. GET_PARAM; BOs made, mapped, located and closed by ioctl;
/// `loops` compute jobs by SUBMIT_CSD and WAIT_BO; wrong arguments; then an
/// endless bin job by SUBMIT_CL: WAIT_BO answers ETIME at timeout 0 while
/// it runs, and EIO when it has failed.
fn ioctl_check(loops: u32) -> Result<String, String> {
    use uapi::*;
    let mut v3d = client::Client::open()?;
    fn call<T: Copy>(v3d: &mut client::Client, func: u64, arg: &T, handles: &[u32]) -> Result<T, syscall::Error> {
        let mut buf = bytes(arg, handles);
        v3d.ioctl(func, &mut buf)?;
        Ok(read::<T>(&buf).expect("short reply").0)
    }
    let expect_err = |what: &str, r: Result<(), syscall::Error>, errno: i32| -> Result<(), String> {
        match r {
            Err(e) if e.errno == errno => Ok(()),
            Err(e) => Err(format!("{what}: {e}, want errno {errno}")),
            Ok(()) => Err(format!("{what}: passed, want errno {errno}")),
        }
    };
    let param = |v3d: &mut client::Client, param: u32| {
        call(v3d, GET_PARAM, &GetParam { param, ..Default::default() }, &[]).map(|p| p.value)
    };
    let ident1 = param(&mut v3d, PARAM_HUB_IDENT1).map_err(|e| format!("GET_PARAM: {e}"))?;
    if ident1 == 0 || param(&mut v3d, PARAM_SUPPORTS_CSD).ok() != Some(1) {
        return Err(format!("GET_PARAM: HUB_IDENT1 {ident1:#x}, SUPPORTS_CSD not 1"));
    }
    expect_err("GET_PARAM 99", param(&mut v3d, 99).map(|_| ()), syscall::EINVAL)?;
    let create = |v3d: &mut client::Client| -> Result<client::Bo, String> {
        let bo = call(v3d, CREATE_BO, &CreateBo { size: 4096, ..Default::default() }, &[])
            .map_err(|e| format!("CREATE_BO: {e}"))?;
        let mapped = call(v3d, MMAP_BO, &MmapBo { handle: bo.handle, ..Default::default() }, &[])
            .map_err(|e| format!("MMAP_BO: {e}"))?;
        if mapped.offset != (bo.handle as u64) << 32 {
            return Err(format!("MMAP_BO: offset {:#x}", mapped.offset));
        }
        let at = call(v3d, GET_BO_OFFSET, &GetBoOffset { handle: bo.handle, ..Default::default() }, &[])
            .map_err(|e| format!("GET_BO_OFFSET: {e}"))?;
        if at.offset != bo.offset {
            return Err(format!("GET_BO_OFFSET {:#x}, CREATE_BO said {:#x}", at.offset, bo.offset));
        }
        let bo = client::Bo { id: bo.handle, va: bo.offset };
        v3d.map(&bo, 4096)?;
        Ok(bo)
    };
    expect_err(
        "CREATE_BO with flags",
        call(&mut v3d, CREATE_BO, &CreateBo { size: 4096, flags: 1, ..Default::default() }, &[]).map(|_| ()),
        syscall::EINVAL,
    )?;
    let (code, unif, result) = (create(&mut v3d)?, create(&mut v3d)?, create(&mut v3d)?);
    v3d.put(&code, 0, &csd::shader_words())?;
    let wait = |v3d: &mut client::Client, bo: &client::Bo, timeout_ns: u64| {
        call(v3d, WAIT_BO, &WaitBo { handle: bo.id, timeout_ns, ..Default::default() }, &[]).map(|_| ())
    };
    let start = Instant::now();
    for i in 0..loops {
        let constant = csd::constant(0xc000 | i << 5);
        v3d.fill(&result, 0, 16, 0xdead_0000)?;
        v3d.put(&unif, 0, &[result.va, constant])?;
        let arg = SubmitCsd { cfg: csd::job_cfg(code.va, unif.va), bo_handle_count: 3, ..Default::default() };
        call(&mut v3d, SUBMIT_CSD, &arg, &[code.id, unif.id, result.id]).map_err(|e| format!("SUBMIT_CSD: {e}"))?;
        wait(&mut v3d, &result, u64::MAX).map_err(|e| format!("loop {i}: WAIT_BO: {e}"))?;
        let got = v3d.get(&result, 0, 16)?;
        if got != csd::want(constant) {
            return Err(format!("loop {i}: result {got:#x?}, want {:#x?}", csd::want(constant)));
        }
    }
    let us = start.elapsed().as_micros() / loops.max(1) as u128;
    wait(&mut v3d, &result, 0).map_err(|e| format!("WAIT_BO on an idle BO with timeout 0: {e}"))?;
    expect_err("WAIT_BO on handle 999", wait(&mut v3d, &client::Bo { id: 999, va: 0 }, 0), syscall::ENOENT)?;
    let arg = SubmitCsd { cfg: csd::job_cfg(code.va, unif.va), bo_handle_count: 1, ..Default::default() };
    expect_err("SUBMIT_CSD naming handle 999", call(&mut v3d, SUBMIT_CSD, &arg, &[999]).map(|_| ()), syscall::ENOENT)?;
    let arg = SubmitCsd { extensions: 1, ..arg };
    expect_err("SUBMIT_CSD with extensions", call(&mut v3d, SUBMIT_CSD, &arg, &[code.id]).map(|_| ()), syscall::EINVAL)?;
    // The endless bin job: busy, then failed.
    let (endless, _) = render::endless_job(&mut v3d)?;
    let end = endless.va + 0x100;
    let arg = SubmitCl {
        bcl_start: endless.va,
        bcl_end: end,
        rcl_start: endless.va,
        rcl_end: end,
        bo_handle_count: 1,
        ..Default::default()
    };
    call(&mut v3d, SUBMIT_CL, &arg, &[endless.id]).map_err(|e| format!("SUBMIT_CL: {e}"))?;
    expect_err("WAIT_BO with timeout 0 on the endless job", wait(&mut v3d, &endless, 0), syscall::ETIME)?;
    let start = Instant::now();
    expect_err("WAIT_BO on the endless job", wait(&mut v3d, &endless, u64::MAX), syscall::EIO)?;
    let waited = start.elapsed().as_millis();
    for bo in [&code, &unif, &result, &endless] {
        v3d.unmap(bo.id);
        call(&mut v3d, GEM_CLOSE, &GemClose { handle: bo.id, ..Default::default() }, &[])
            .map_err(|e| format!("GEM_CLOSE {}: {e}", bo.id))?;
    }
    expect_err(
        "GEM_CLOSE twice",
        call(&mut v3d, GEM_CLOSE, &GemClose { handle: code.id, ..Default::default() }, &[]).map(|_| ()),
        syscall::ENOENT,
    )?;
    Ok(format!(
        "HUB_IDENT1 {ident1:#x}; {loops} of {loops} jobs right, {us} us per job round trip; wrong arguments refused; endless job: ETIME, then EIO after {waited} ms"
    ))
}

fn main() {
    let args: Vec<String> = env::args().collect();
    // Step 3 (redox-rpi#598): the CSD job through v3dd, no register access.
    if args.iter().any(|a| a == "--via-v3dd") {
        let mapped = args.iter().any(|a| a == "--mapped");
        if mapped {
            client::MAPPED.store(true, std::sync::atomic::Ordering::Relaxed);
            match mapped_check() {
                Ok(stats) => say(&format!("mapped: PASS: both ways, and after a free while mapped ({stats})")),
                Err(err) => {
                    say(&format!("mapped: FAIL: {err}"));
                    std::process::exit(1);
                }
            }
        }
        let loops = args.iter().position(|a| a == "--loops").and_then(|i| args.get(i + 1)?.parse().ok()).unwrap_or(100);
        let ok = if args.iter().any(|a| a == "--ioctl") {
            let result = ioctl_check(loops).and_then(|done| nothing_left().map(|left| format!("{done}; no BOs left {left}")));
            match &result {
                Ok(what) => say(&format!("ioctl: PASS: {what}")),
                Err(err) => say(&format!("ioctl: FAIL: {err}")),
            }
            result.is_ok()
        } else if args.iter().any(|a| a == "--queue") {
            let clients =
                args.iter().position(|a| a == "--clients").and_then(|i| args.get(i + 1)?.parse().ok()).unwrap_or(4);
            queue_via_v3dd(clients, loops.min(64))
        } else if args.iter().any(|a| a == "--faults") {
            faults_via_v3dd()
        } else if args.iter().any(|a| a == "--render") {
            let draw = if args.iter().any(|a| a == "--triangle-math") {
                render::Draw::MathShaders
            } else if args.iter().any(|a| a == "--triangle") {
                render::Draw::CopyShaders
            } else {
                render::Draw::Clear
            };
            render::via_v3dd(draw, loops)
        } else {
            csd::via_v3dd(loops)
        };
        let ok = ok
            && (!mapped
                || match leak_check() {
                    Ok(stats) => {
                        say(&format!("mapped: no BOs leaked ({stats})"));
                        true
                    }
                    Err(err) => {
                        say(&format!("mapped: FAIL: {err}"));
                        false
                    }
                });
        std::process::exit(if ok { 0 } else { 1 });
    }
    common::init();
    let off = env::args().any(|a| a == "--off");
    let watchdog = !env::args().any(|a| a == "--no-watchdog");
    let run_csd = env::args().any(|a| a == "--csd");
    let run_render = env::args().any(|a| a == "--render");
    let draw = if env::args().any(|a| a == "--triangle-math") {
        render::Draw::MathShaders
    } else if env::args().any(|a| a == "--triangle") {
        render::Draw::CopyShaders
    } else {
        render::Draw::Clear
    };
    let show_secs: Option<u64> =
        args.iter().position(|a| a == "--show").map(|i| args.get(i + 1).and_then(|s| s.parse().ok()).unwrap_or(30));
    let mut picture = Vec::new();

    let dtb = fs::read("/scheme/kernel.dtb").unwrap_or_default();
    let fdt = Fdt::new(&dtb).ok();
    let v3d = fdt.as_ref().and_then(|f| f.find_compatible(&["brcm,2711-v3d", "brcm,7278-v3d"]));
    match &v3d {
        Some(node) => say(&format!(
            "devicetree: {} ({}), status {}",
            node.name,
            node.compatible().map(|c| c.first()).unwrap_or("?"),
            node.property("status").and_then(|p| p.as_str()).unwrap_or("okay")
        )),
        None if fdt.as_ref().is_some_and(|f| f.root().compatible().all().any(|c| c == "brcm,bcm2711")) => {
            say("devicetree: no v3d node (an upstream devicetree), but brcm,bcm2711: going on with the Pi 400 addresses")
        }
        None => fail("not a BCM2711 (no brcm,2711-v3d, no brcm,bcm2711 in the devicetree)"),
    }
    // The firmware's clocks (rpi-firmwared): V3D should run.
    match fs::read_to_string("/scheme/rpi-firmware/clocks") {
        Ok(clocks) => {
            for line in clocks.lines() {
                say(&format!("clock: {line}"));
            }
        }
        Err(err) => say(&format!("clocks: /scheme/rpi-firmware/clocks: {err}")),
    }

    let pm = Regs::map(PM_PHYS, 0x1000, "pm");
    let asb = Regs::map(RPIVID_ASB_PHYS, 0x1000, "rpivid_asb");
    pm.show(PM_RSTS, "PM_RSTS");
    pm.show(PM_RSTC, "PM_RSTC");
    let grafx = pm.show(PM_GRAFX, "PM_GRAFX");
    say(&format!(
        "PM_GRAFX: power up {}, power ok {}, V3D reset {}",
        grafx & PM_POWUP != 0,
        grafx & PM_POWOK != 0,
        if grafx & PM_V3DRSTN != 0 { "released" } else { "asserted" }
    ));
    let id = asb.show(ASB_AXI_BRDG_ID, "ASB_AXI_BRDG_ID");
    if id != BCM2835_BRDG_ID {
        fail(&format!("bridge id {id:#x}, not \"brdg\" ({BCM2835_BRDG_ID:#x}): wrong address"));
    }
    asb.show(ASB_V3D_M_CTRL, "ASB_V3D_M_CTRL");
    asb.show(ASB_V3D_S_CTRL, "ASB_V3D_S_CTRL");
    // Test 547: GRAFX shows no power. Test 550: the firmware's V3D domain
    // (raspberrypi-power, RPI_POWER_DOMAIN_V3D) switches on, but PM_GRAFX
    // stays 0x1000. Mainline Linux does not look at PM_GRAFX on the BCM2711
    // either: bcm2835_power_power_on() returns early there, and
    // bcm2835_asb_power_on() only releases the reset and opens the
    // bridges. So: firmware domain on, then the same as Linux.
    let mut powered = false;
    let mut grafx = grafx;
    if grafx & PM_POWOK == 0 {
        say(&format!("v3d_power: {}", read_v3d_power()));
        say("asking the firmware for V3D power (/scheme/rpi-firmware/v3d_power = on)");
        if let Err(err) = fs::write(V3D_POWER, "on") {
            fail(&format!("{V3D_POWER}: {err} (rpi-firmwared without v3d_power?)"));
        }
        powered = true;
        thread::sleep(Duration::from_millis(5));
        grafx = pm.show(PM_GRAFX, "PM_GRAFX");
        say(&format!(
            "after v3d_power on: v3d_power {}, PM_GRAFX power ok {} (not checked on the BCM2711, as in Linux)",
            read_v3d_power(),
            grafx & PM_POWOK != 0
        ));
    }

    if watchdog {
        // bcm2835_wdt_start()
        pm.write(PM_WDOG, PM_PASSWORD | ((WATCHDOG_SECS << 16) & PM_WDOG_TIME_SET));
        let cur = pm.read(PM_RSTC);
        pm.write(PM_RSTC, PM_PASSWORD | (cur & PM_RSTC_WRCFG_CLR) | PM_RSTC_WRCFG_FULL_RESET);
        say(&format!("watchdog armed: {WATCHDOG_SECS} s, then a full SoC reset if v3dprobe hangs"));
    }

    // bcm2835_asb_power_on(): reset released, then the bridges, master first.
    if grafx & PM_V3DRSTN == 0 {
        say("releasing the V3D reset (PM_GRAFX |= V3DRSTN)");
        pm.write(PM_GRAFX, PM_PASSWORD | grafx | PM_V3DRSTN);
        thread::sleep(Duration::from_millis(1));
        pm.show(PM_GRAFX, "PM_GRAFX");
    }
    for (reg, name) in [(ASB_V3D_M_CTRL, "master"), (ASB_V3D_S_CTRL, "slave")] {
        let v = asb.read(reg);
        say(&format!("opening the ASB {name} bridge ({v:#x})"));
        asb.write(reg, PM_PASSWORD | (v & !ASB_REQ_STOP));
        let start = Instant::now();
        while asb.read(reg) & ASB_ACK != 0 {
            if start.elapsed() > Duration::from_millis(10) {
                let now = asb.read(reg);
                // Undo, as Linux's error path: bridges closed, reset
                // asserted, domain off. V3D itself is not read.
                for r in [ASB_V3D_S_CTRL, ASB_V3D_M_CTRL] {
                    asb.write(r, PM_PASSWORD | asb.read(r) | ASB_REQ_STOP);
                }
                pm.write(PM_GRAFX, PM_PASSWORD | (pm.read(PM_GRAFX) & !PM_V3DRSTN));
                if powered {
                    let _ = fs::write(V3D_POWER, "off");
                }
                if watchdog {
                    pm.write(PM_RSTC, PM_PASSWORD | PM_RSTC_RESET);
                    say("watchdog stopped");
                }
                fail(&format!("ASB {name} bridge: no ack within 10 ms ({now:#x}): bridges closed, reset asserted, V3D power off; V3D not read"));
            }
        }
        say(&format!("  ASB {name}: {:#x} after {} us", asb.read(reg), start.elapsed().as_micros()));
    }

    let hub = Regs::map(HUB_PHYS, 0x4000, "hub");
    let core = Regs::map(CORE0_PHYS, 0x4000, "core0");
    let h0 = hub.show(V3D_HUB_IDENT0, "V3D_HUB_IDENT0");
    let h1 = hub.show(V3D_HUB_IDENT1, "V3D_HUB_IDENT1");
    let h2 = hub.show(V3D_HUB_IDENT2, "V3D_HUB_IDENT2");
    let h3 = hub.show(V3D_HUB_IDENT3, "V3D_HUB_IDENT3");
    let c0 = core.show(V3D_CTL_IDENT0, "V3D_CTL_IDENT0");
    let c1 = core.show(V3D_CTL_IDENT1, "V3D_CTL_IDENT1");
    let c2 = core.show(V3D_CTL_IDENT2, "V3D_CTL_IDENT2");
    let stop_watchdog = || {
        if watchdog {
            // bcm2835_wdt_stop()
            pm.write(PM_RSTC, PM_PASSWORD | PM_RSTC_RESET);
            say(&format!("watchdog stopped (PM_RSTC {:#x})", pm.read(PM_RSTC)));
        }
    };
    if !run_csd && !run_render {
        stop_watchdog();
    }

    // As v3d_platform_drm_probe() and v3d_debugfs_ident() decode them.
    let tver = h1 & 0xf;
    let rev = (h1 >> 4) & 0xf;
    let ver = tver * 10 + rev;
    let ncores = (h1 >> 8) & 0xf;
    let magic = [(c0 & 0xff) as u8, ((c0 >> 8) & 0xff) as u8, ((c0 >> 16) & 0xff) as u8];
    say(&format!("hub: ident0 {h0:#x}, V3D {}.{}, cores {ncores}, MMU {}, L3C {} KiB, TFU {}, IP rev {} idx {}",
        tver, rev, h2 & (1 << 8) != 0, h2 & 0xff, h1 & (1 << 17) != 0, (h3 >> 8) & 0xff, h3 & 0xff));
    say(&format!(
        "core0: \"{}\" version {}, VPM {} KiB, slices {}, QPUs per slice {}, TMUs per slice {}",
        String::from_utf8_lossy(&magic),
        c0 >> 24,
        (c1 >> 28) & 0xf,
        (c1 >> 4) & 0xf,
        (c1 >> 8) & 0xf,
        (c1 >> 12) & 0xf,
    ));
    let _ = c2;
    // Linux checks only the version and the cores; the "V3D" in ident0's
    // low bytes is logged for comparison.
    let mut ok = ver >= 41 && ncores >= 1;
    say(&format!("{}: V3D {ver} on the BCM2711", if ok { "PASS" } else { "FAIL" }));
    if run_csd || run_render {
        // The watchdog stays armed through the jobs: a GPU job that hangs
        // the bus resets the SoC.
        if ok && run_csd {
            ok = csd::run(&hub, &core);
        }
        if ok && run_render {
            (ok, picture) = render::run(&hub, &core, draw);
        }
        stop_watchdog();
    }
    if let (Some(seconds), false) = (show_secs, picture.is_empty()) {
        render::show(&picture, seconds);
    }

    if off {
        for (reg, name) in [(ASB_V3D_S_CTRL, "slave"), (ASB_V3D_M_CTRL, "master")] {
            asb.write(reg, PM_PASSWORD | asb.read(reg) | ASB_REQ_STOP);
            say(&format!("closed the ASB {name} bridge ({:#x})", asb.read(reg)));
        }
        pm.write(PM_GRAFX, PM_PASSWORD | (pm.read(PM_GRAFX) & !PM_V3DRSTN));
        say("V3D reset asserted again");
        if powered {
            match fs::write(V3D_POWER, "off") {
                Ok(()) => say(&format!("v3d_power off: {}, PM_GRAFX {:#x}", read_v3d_power(), pm.read(PM_GRAFX))),
                Err(err) => say(&format!("v3d_power off: {err}")),
            }
        }
    }
    std::process::exit(if ok { 0 } else { 1 });
}

const V3D_POWER: &str = "/scheme/rpi-firmware/v3d_power";

fn read_v3d_power() -> String {
    match fs::read_to_string(V3D_POWER) {
        Ok(state) => state.trim().to_owned(),
        Err(err) => format!("({err})"),
    }
}

fn say(line: &str) {
    println!("v3dprobe: {line}");
    let _ = std::io::stdout().flush();
}

fn fail(why: &str) -> ! {
    say(&format!("FAIL: {why}"));
    std::process::exit(1);
}
