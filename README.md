# vc4d

A display and 3D driver for the Broadcom VideoCore IV GPU (Raspberry Pi 3,
3B+ and Zero 2 W) on [Redox OS](https://www.redox-os.org/), for the
[redox-rpi](https://github.com/redox-rpi/redox-rpi) project.

Status: stage 1 (display: HVS planes) has code, on branch `stage1-hvs`
([#1](https://github.com/redox-rpi/vc4d/pull/1)), and runs on the Pi 3B+:
the desktop with a hardware cursor plane at 1920x1080 (redox-rpi Pi tests
220a, 256, 259 and 276). redox-rpi builds a pinned commit of it (experiment
`vc4d`, on its branch `vc4d-fifo-diag` for now). The firmware still sets up
HDMI and the mode. Stage 2 has started read only: branch `underrun-diag`
logs the HVS FIFO, pixel valve 2, the HDMI encoder and the hotplug GPIO
(Pi tests 316 to 328). Host tests: the HVS logic as the crate `vc4-hvs`
([#2](https://github.com/redox-rpi/vc4d/pull/2)). Stage 2b reads the monitor's EDID over HDMI DDC
([#4](https://github.com/redox-rpi/vc4d/pull/4)); on the Pi 3B+ the
preferred mode is 1080p60, as the firmware set it (redox-rpi Pi test 387).
Stage 3 has no code yet.
The plan is
[redox-rpi#188](https://github.com/redox-rpi/redox-rpi/issues/188).

## Plan

1. **Display:** HVS planes and HDMI, as a replacement for `rpi-fbd`. The
   mouse pointer and windows become hardware planes, so moving a window
   needs almost no redrawing.
2. **3D:** V3D job submission, plus Mesa's `vc4` Gallium driver (MIT) with a
   small Redox winsys. This gives OpenGL ES 2.0 in hardware.
3. **Compositing:** Orbital composes windows on the GPU.

## Stage 1: HVS planes

The firmware still sets up HDMI, the pixel valve and the mode at power-on,
and leaves one HVS channel running with its own display list. `vc4d`:

- maps the HVS (`brcm,bcm2835-hvs` in the devicetree), finds the enabled
  channel and its size, and logs the firmware's registers and list;
- registers as a display (`display.vc4`) through Redox's `driver-graphics`,
  with one mode, the firmware's;
- gives clients dumb buffers in contiguous DMA memory and scans the primary
  plane's buffer out directly: no copy into a firmware framebuffer;
- offers a 64x64 ARGB cursor plane, which Orbital uses on its own, so the
  pointer moves by changing the display list, without redrawing;
- writes each new display list into a free place above the firmware's
  lists, then points the channel at it.

Not yet: other modes, vertical blank interrupts, windows as planes (needs
Orbital changes), the pixel valve and HDMI.

## Stage 2b: EDID over DDC

At start, before it touches the HVS, `vc4d` reads the monitor's EDID over
HDMI DDC: I2C address 0x50 on the BSC the devicetree's hdmi node names in
`ddc` (BSC2, bus address 0x7e805000, on the BCM2837), base block and the
first extension. It logs the raw bytes and what they say: maker, name,
preferred mode with its full timing, the other detailed timings, the range
limits, the established and standard modes, and the CEA-861 extension
(HDMI or DVI, audio, video codes, its detailed timings). All lines start
with `vc4d: edid:`; `PASS` when an EDID was read and parsed, `FAIL` with
the reason otherwise (no monitor answers with `no acknowledge`). Nothing
changes on the screen yet: the mode stays the firmware's. `VC4D_EDID=off`
skips the read.

The parser is the crate `vc4-edid` in `edid/`, without dependencies:
`cargo test` there runs on any host. In QEMU (`raspi3b`, which models the
BSCs) `-device i2c-ddc,bus=i2c-bus.2` puts QEMU's EDID generator (`QEMU Monitor`, 1280x800 at 75 Hz preferred; it ignores `xres`/`yres`)
on the DDC bus. Blocks after the second (E-DDC segments) are not read yet.

## Stage 2c: mode registers, read only

The crate `vc4-mode` (`mode/`, host tests) turns a timing from the EDID
into the register values for pixel valve 2 and the HDMI core's timing, the
way Linux's vc4 driver lays them out. The tests check them against what
Broadcom's firmware programmed at 1080p60 (redox-rpi Pi test 337): for the
test monitor's own EDID (Pi test 387) every one of them is the same.

On the Pi, after the EDID, vc4d reads pixel valve 2 and logs its values
next to the firmware's (`vc4d: mode: pv2 ...`), then `vc4d: mode: PASS`
or `DIFFERENT`. It writes nothing. The HDMI core is not read here: it
needs its clocks running, which vc4d does not check yet. Next: write these
values, then set up PLLH and the HDMI PHY for modes the firmware did not
pick.

## Stage 3a: vc4d programs pixel valve 2

With `VC4D_MODE=write`, when pixel valve 2's values equal the firmware's,
vc4d writes them itself, in the order of Linux's `vc4_crtc.c`:

1. video off at the end of the frame;
2. the valve off, its FIFO emptied;
3. the four timing registers;
4. the valve on, the video on.

The mode is the firmware's, so the monitor keeps it; the screen is blank
for a few frames. vc4d then reads every register back (`vc4d: mode:
write: PASS`). The usual checks follow: the HVS channel must run on, and
the check without a screen must pass.

`vc4_mode::pv_program()` gives the steps, and host tests check their order
and the end state.

Also in this stage, read only: the HDMI core's timing is compared with
the firmware's. It is read only after `CM_HSMCTL` shows the HSM clock
running, since reading the block without its clock can hang the bus.

Without `VC4D_MODE=write`, vc4d only compares, as in stage 2c. Next:
writing the HDMI core's timing, then PLLH and the PHY for a mode the
firmware did not pick.

Environment (the bootloader's): `VC4D=off` leaves the display to `vesad` or
`rpi-fbd`. `VC4D_FAKE=WIDTHxHEIGHT` runs against an HVS in normal memory, for
QEMU, which has none: everything but the picture. `VC4D_LOG=1` logs every
new display list (the first four always are). `VC4D_CHECK=off` skips the
check below. `VC4D_EDID=off` skips the EDID read.

## Check without a screen

Before it registers as a display, `vc4d` shows colour bars with a cursor
square for a few frames, moves the cursor once, and reads back from the
HVS: the channel points at its list, the list reads back as written, the
active word is inside it, the frame count advances, the cursor's position
word is the new one, and no error bits (underrun and the like) are set.
Then it goes back to the firmware's list. A thread logs the channel 2, 5,
10, 30, 60 and 120 s after the handover. All lines start with
`vc4d: check:` and say `PASS` or `FAIL`. This needs no monitor, only an
enabled HVS channel.

## vc4shot

`vc4shot [file.ppm] [channel]` saves what vc4d shows as a PPM picture
(default `/tmp/vc4shot.ppm`), for checks without a person. It lists the
planes of the enabled channel's display list (read-only), then asks vc4d
for the picture: Redox lets no program map RAM by its physical address,
so vc4d, which has the buffers mapped, composes them (cursor alpha
included) when `/tmp/vc4shot.request` appears and writes
`/tmp/vc4shot.ppm`. The watch thread looks every 3 s.

## Build

redox-rpi's experiment `vc4d` builds a pinned commit of this repository
with the cookbook. It needs large contiguous buffers (redox-rpi kernel
patches `0049` and `0050`). The driver only builds for Redox
(`aarch64-unknown-redox`).

The logic without Redox dependencies is its own crate, `vc4-hvs` in
`hvs/`: HVS registers, display list encoding and decoding, register
fields. It builds and tests on any host with stable Rust:

```
cd hvs && cargo test
```

CI runs these tests and clippy (report only) on every push
(`.github/workflows/host-tests.yml`).

## License

GPL-2.0-only, see [LICENSE](LICENSE).

The driver is a port of the Linux `vc4` DRM driver
(`drivers/gpu/drm/vc4`), which is GPL-2.0-only. Each source file carries an
`SPDX-License-Identifier: GPL-2.0-only` header and names the Linux files
and kernel version it was translated from.

`vc4d` runs as its own program on Redox and talks to the kernel and to
Orbital through schemes. The rest of redox-rpi stays MIT. redox-rpi builds
`vc4d` from a fixed commit of this repository, and each image manifest names
that commit.
