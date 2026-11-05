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

Not yet: EDID and other modes, vertical blank interrupts, windows as
planes (needs Orbital changes), the pixel valve and HDMI.

Environment (the bootloader's): `VC4D=off` leaves the display to `vesad` or
`rpi-fbd`. `VC4D_FAKE=WIDTHxHEIGHT` runs against an HVS in normal memory, for
QEMU, which has none: everything but the picture. `VC4D_LOG=1` logs every
new display list (the first four always are). `VC4D_CHECK=off` skips the
check below.

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

## Build

redox-rpi's experiment `vc4d` builds a pinned commit of this repository
with the cookbook. It needs large contiguous buffers (redox-rpi kernel
patches `0049` and `0050`). The driver only builds for Redox
(`aarch64-unknown-redox`); the tests of the display list encoding in
`src/hvs.rs` run on any host when that file is built as a library.

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
