# vc4d

Display and 3D drivers for the Broadcom VideoCore GPUs of the Raspberry Pi on
[Redox OS](https://www.redox-os.org/), for the
[redox-rpi](https://github.com/redox-rpi/redox-rpi) project. The plan is
[redox-rpi#188](https://github.com/redox-rpi/redox-rpi/issues/188).

| Program | What it does | Boards |
|---|---|---|
| `vc4d` | Display: HVS planes, modes, flips, screenshots | Pi 3, 3B+, Zero 2 W (HVS4); Pi 4 and 400 (HVS5) |
| `vc4shot` | Saves the picture on the screen as a PPM file | all of the above |
| `v3dprobe` | V3D bring-up probe: power, MMU, compute and render jobs | Pi 4 and 400 (V3D 4.2) |
| `v3dd` | V3D daemon: buffer objects, compute and bin/render jobs behind `/scheme/v3d` | Pi 4 and 400 (V3D 4.2) |

redox-rpi builds a pinned commit of this repository (experiments `vc4d` and
`vc4d-flip`). Each image manifest names that commit.

## Display: vc4d

The firmware sets up HDMI, the clocks and the mode at power-on and leaves
one HVS channel running. vc4d takes that channel over:

- **Planes.** vc4d writes its own display lists and points the channel at
  them. It registers as a display (`display.vc4`) through Redox's
  `driver-graphics` and gives clients dumb buffers in contiguous DMA
  memory. Planes from bottom to top: the primary plane, three overlay
  planes and a 64x64 ARGB cursor plane. Orbital uses the cursor plane on
  its own. A compositor can show a window on an overlay plane, so moving the
  pointer or the window only rewrites the display list.
- **Display lists.** There are three list slots. A slot is reused only two
  lists later, and the HVS gets at most one new list per frame, so it never
  reads a list that is being overwritten.
- **Flips.** Clients draw into normal memory and report what changed. vc4d
  copies the damaged area into the scanout buffer the HVS is not reading
  and switches the list at the next frame: no tearing, and clients never
  read uncached memory. Flips are asynchronous: a client never waits for a
  frame, newer damage is merged into the waiting frame, and a 4 ms timer
  shows it once the HVS has taken the last list. `VC4D_FLIP=off` or
  `/etc/vc4d-flip-off` scans the client buffer out directly instead.
- **Vertical blank** (Pi 4 and 400). A waiting flip is shown on pixel
  valve 2's vertical front porch interrupt (HDMI 0), as Linux
  `vc4_crtc.c` does for vblank, instead of the 4 ms timer. The interrupt is
  on only while a flip waits. The firmware leaves pixel valve 2's
  interrupts alone, but it takes the HVS interrupts, so those stay its
  own. A 20 ms timer catches a missed frame. The log counts flips shown on
  the interrupt and on the timer. At start, `vc4d: irq probe:` lines show
  the HVS and pixel valve 2 interrupt registers and how many interrupts
  reach the GIC in one second.
- **Scaled modes.** With `/etc/vc4d-mode` (for example `1280x720`), clients
  see only that mode, and the HVS scales every plane to the firmware's mode
  (the PPF and TPZ scalers and the Mitchell/Netravali kernel of Linux
  `vc4_plane.c`).
- **EDID** (HVS4 boards). At start vc4d reads the monitor's EDID over HDMI
  DDC (BSC2 on the BCM2837) and logs it decoded: maker, name, the preferred
  mode with its timing, the other modes, and the CEA-861 extension.
  `VC4D_EDID=off` skips the read. On the Pi 4 and 400 the read is opt-in
  (`VC4D_EDID=on` or `/etc/vc4d-edid-on`, redox-rpi#671): HDMI0's own I2C
  controller (`ddc_stb.rs`), taken over from the firmware's auto-i2c
  engine. vc4d then compares pixel valve 2 and the HDMI0 timing it
  computes for the preferred mode with the firmware's (two pixels per
  clock, BCM2711 offsets; read only), and with `VC4D_MODE=write` or
  `/etc/vc4d-mode-write` writes them itself when they match.
- **Mode registers** (HVS4 boards). vc4d computes pixel valve 2 and the
  HDMI core's timing from the EDID, laid out as Linux `vc4_crtc.c` and
  `vc4_hdmi.c` do, and compares them with the firmware's. With
  `VC4D_MODE=write`, if they match, it writes both itself in one blank
  moment and reads them back. The mode is still the firmware's: the PLL,
  the HDMI PHY and the encoder stay as the firmware set them.
- **HVS5** (Pi 4 and 400). vc4d picks the HVS generation from the
  devicetree. On the BCM2711 it uses the HVS5 list format (Linux
  `SCALER5_*`) and keeps the firmware's mode. It reads no EDID there,
  because HDMI and DDC are other blocks on that SoC.
- **Check without a screen.** Before the handover vc4d shows colour bars
  with a cursor, moves the cursor and reads back from the HVS: list address,
  list words, frame counter, cursor position and error bits. A thread logs
  the channel for two minutes afterwards. Lines start with `vc4d: check:`.
  `VC4D_CHECK=off` skips it.
- **Trace.** `VC4D_TRACE=<file>` or `/etc/vc4d-trace` writes a CSV time
  series of damage, copies, lists, cursor moves and every HVS frame, to
  find where the time of a display stall goes.

Environment (set by the bootloader) and files:

| Setting | Effect |
|---|---|
| `VC4D=off` | vc4d does not start; `vesad` or `rpi-fbd` keeps the display |
| `VC4D_FAKE=WxH` | runs against an HVS in normal memory, for QEMU (no HVS there) |
| `VC4D_LOG=1` | logs every display list and overlay placement (the first four lists always) |
| `VC4D_FLIP=off`, `/etc/vc4d-flip-off` | no flips, direct scanout of the client buffer |
| `VC4D_MODE=write` | writes pixel valve 2 and the HDMI timing itself (HVS4; on the Pi 4 and 400 with the EDID read on) |
| `/etc/vc4d-mode-write` | the same as `VC4D_MODE=write` (Pi 4 and 400) |
| `VC4D_EDID=off` | no EDID read |
| `VC4D_EDID=on`, `/etc/vc4d-edid-on` | EDID read on the Pi 4 and 400 (log only) |
| `VC4D_CHECK=off` | no check before the handover |
| `VC4D_TRACE=<file>`, `/etc/vc4d-trace` | CSV trace |
| `VC4D_VBLANK=off` | flips on the 4 ms timer instead of the vertical blank (Pi 4 and 400) |
| `VC4D_IRQ_PROBE=off` | no interrupt probe at start (Pi 4 and 400) |
| `/etc/vc4d-mode` | the clients' mode, scaled by the HVS |

### vc4shot

`vc4shot [file.ppm] [channel]` saves the picture on the screen, for
checks without a person (default `/tmp/vc4shot.ppm`). It lists the planes of
the channel's display list. Redox lets no program map RAM by its physical
address, so vc4d composes the picture itself: when `/tmp/vc4shot.request`
appears, vc4d writes `/tmp/vc4shot.ppm` from its mapped buffers, cursor alpha
included. vc4d checks for the request every 3 s.

## 3D: V3D 4.2 on the Pi 4 and 400

The steps are redox-rpi issues
[#595](https://github.com/redox-rpi/redox-rpi/issues/595),
[#596](https://github.com/redox-rpi/redox-rpi/issues/596) and
[#598](https://github.com/redox-rpi/redox-rpi/issues/598). The code follows
Linux v6.12 `drivers/gpu/drm/v3d` and Mesa's v3d driver; the file headers say
which parts.

**Power.** The firmware leaves the V3D power domain off. Both programs turn
it on through `/scheme/rpi-firmware/v3d_power` (redox-rpi experiment
`v3d-power`). They then release the reset in `PM_GRAFX` and open the async AXI
bridges, master first, as Linux `bcm2835-power.c` does. The SoC watchdog is
armed around the first accesses and around each job, so a bus hang resets
the SoC instead of needing a person.

**v3dprobe.** Each step adds an option:

- step 0: power, `HUB_IDENT` and `CTL_IDENT`;
- step 1, `--csd`: a 4 MiB MMU page table, and one compute (CSD) job with a
  hand-assembled QPU shader; the CPU checks its 16 results;
- step 2, `--render [--triangle | --triangle-math] [--show N]`: bin and
  render control lists for a 256x256 RGBA8 picture, cleared or with a red
  triangle from hand-encoded coordinate, vertex and fragment shaders. The CPU
  checks every pixel. `--show N` puts the picture in an Orbital window for N
  seconds.

**v3dd.** v3dd is started by init (redox-rpi experiment `v3dd`) and owns
V3D. It powers it up, keeps one MMU page table, hands out zeroed buffer
objects (BOs) mapped into it, and runs CSD jobs and bin/render jobs one at
a time, with the cache handling of Linux v3d. Clients open `/scheme/v3d`
(root only), write one command line and read the answer:

| Command | Answer |
|---|---|
| `bo BYTES` | `ID VA`: a new zeroed BO, mapped at V3D address VA |
| `put ID WORD HEX...` | `ok`: writes 32-bit words from word WORD on |
| `get ID WORD COUNT` | COUNT words in hex |
| `fill ID WORD COUNT HEX` | `ok`: sets COUNT words to HEX |
| `free ID` | `ok` |
| `csd CFG0 ... CFG6` | `done US`: runs one CSD job and waits for it |
| `cl BCL END RCL END QMA QMS QTS` | `done US bin US render US`: one bin job, then one render job |
| `stats` | counters: BOs, frees, jobs, bin/render jobs, failed jobs, MMU faults, resets, longest job, overflow memory, free address ranges, interrupt (on/off, interrupts, waits ended by the interrupt or the timer) |

`cl` takes V3D addresses only: the bin and render control lists (start
and end), the tile allocation memory and its size, and the tile state.
The client builds them in its BOs. The binner's overflow memory belongs
to v3dd: 512 KiB at the start of every bin job, and another 512 KiB chunk
(at most 16) each time the binner runs out (`OUTOMEM`), as Linux
`v3d_overflow_mem_work()`.

BOs belong to the open file and are freed when it closes. Freed address
ranges are reused. On other boards v3dd leaves the hardware alone.

**Mapping BOs.** A client maps a BO into its own memory with `fmap` on
the same handle: offset `(ID << 32) + BYTE` (BYTE a multiple of 4096),
`MAP_SHARED`. The kernel maps the BO's own pages, with the memory type
v3dd has (uncached, as all DMA memory on aarch64), so the client writes
control lists and shader data and reads results directly, with no cache
flush on either side; it issues a DSB before the command that starts the
job. Private mappings are refused. A BO freed while mapped stays the
client's memory until it unmaps it, but V3D no longer sees it. `put`,
`get` and `fill` stay for tests.

**Interrupt.** v3dd waits for a job on the V3D interrupt, which it finds
in the devicetree and opens through `/scheme/irq`. The core interrupts
(bin done, render done, CSD done, out of overflow memory) and the hub
interrupts (MMU faults) are those of Linux `v3d_irq.c`. A 5 ms timer is
the fallback if an interrupt is lost. Without the interrupt v3dd polls.
Jobs still run one at a time, and the client waits for `done`.

**Errors.** A job that makes the V3D MMU fault (an address with no page
table entry, a write to a read-only page) answers
`error mmu-fault KIND va ADDR unit UNIT`, as Linux `v3d_mmu_irq()` reports
it. A job that is not done after 1 s answers `error timeout ...`. In both
cases v3dd logs the core registers, resets V3D through the PM (as Linux
`bcm2835_reset_reset()`), sets up the MMU and the caches again, and the
next job runs. The SoC watchdog is armed only while a job runs, longer
than the job timeout.

`v3dprobe --via-v3dd [--loops N]` runs the step 1 job N times through
v3dd; with `--render [--triangle | --triangle-math]` it runs the step 2
frame N times instead and checks that every frame equals the first.
`--faults` runs a job that stores to an unmapped address and one that
never ends, and checks that both come back and the next jobs run right.
With `--mapped`, every BO is mapped into v3dprobe and written and read
there instead of through `put`/`get`; a check of the mappings runs first
and a check for leaked BOs last.

Do not run `v3dprobe --csd` or `--render` while v3dd runs: they set up
their own page table.

## Open

- Display: modes the firmware did not pick (the PLLH divider and the HDMI
  PHY; branch `mode-switch` has a first try), EDID and mode registers on
  the BCM2711 (redox-rpi#671), the vertical blank on the BCM2837.
- Orbital: windows on overlay planes (redox-rpi experiment
  `orbital-planes`).
- V3D: asynchronous jobs, one address space per client, a directory
  listing of `/scheme/v3d`. Then a Mesa winsys for Redox.
- The 3D block of the Pi 3, 3B+ and Zero 2 W (VideoCore IV, Linux `vc4`
  V3D) is a different GPU and has no code yet.

## Build

The drivers build only for Redox (`aarch64-unknown-redox`), through the
redox-rpi cookbook. They need large contiguous buffers (redox-rpi kernel
patches `0049` and `0050`).

The logic without Redox dependencies is split into crates that build and
test on any host with stable Rust:

- `hvs/` (`vc4-hvs`): HVS registers, display lists for HVS4 and HVS5, and
  the scaler;
- `edid/` (`vc4-edid`): the EDID parser, with test EDIDs from QEMU and a
  real monitor;
- `mode/` (`vc4-mode`): pixel valve and HDMI timing registers, and the order
  of the writes.

```
cd hvs && cargo test
```

CI runs these tests and clippy (report only) on every push
(`.github/workflows/host-tests.yml`).

In QEMU (`raspi3b`), `VC4D_FAKE` stands in for the HVS, and
`-device i2c-ddc,bus=i2c-bus.2` puts QEMU's EDID generator on the DDC bus.

## License

GPL-2.0-only, see [LICENSE](LICENSE).

The drivers are ports of the Linux `vc4` and `v3d` DRM drivers
(`drivers/gpu/drm/vc4`, `drivers/gpu/drm/v3d`), which are GPL-2.0-only. Each
source file carries an `SPDX-License-Identifier: GPL-2.0-only` header and
names the Linux files and kernel version it was translated from.

The programs run on their own on Redox and talk to the kernel and to Orbital
through schemes. The rest of redox-rpi stays MIT.
