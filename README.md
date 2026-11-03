# vc4d

A display and 3D driver for the Broadcom VideoCore IV GPU (Raspberry Pi 3,
3B+ and Zero 2 W) on [Redox OS](https://www.redox-os.org/), for the
[redox-rpi](https://github.com/redox-rpi/redox-rpi) project.

Status: planning. No code yet. The plan is
[redox-rpi#188](https://github.com/redox-rpi/redox-rpi/issues/188).

## Plan

1. **Display:** HVS planes and HDMI, as a replacement for `rpi-fbd`. The
   mouse pointer and windows become hardware planes, so moving a window
   needs almost no redrawing.
2. **3D:** V3D job submission, plus Mesa's `vc4` Gallium driver (MIT) with a
   small Redox winsys. This gives OpenGL ES 2.0 in hardware.
3. **Compositing:** Orbital composes windows on the GPU.

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
