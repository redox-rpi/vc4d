# vc4d

A display and 3D driver for the Broadcom VideoCore IV GPU (Raspberry Pi 3,
3B+ and Zero 2 W) on [Redox OS](https://www.redox-os.org/), for the
[redox-rpi](https://github.com/redox-rpi/redox-rpi) project.

Status: stage 1 (HVS planes) runs on a Pi 3B+ with HDMI (redox-rpi row
220a, 2026-11-05): the HVS takes vc4d's lists, scans them out without
errors, and moves the cursor plane. Nobody has looked at the picture yet.
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
