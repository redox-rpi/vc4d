// SPDX-License-Identifier: GPL-2.0-only
//
// V3D step 2 (redox-rpi#596): a bin job and a render job from hand-built
// control lists. Step 2a: the binner gets no primitives; the render control
// list clears every tile to one colour and stores it to a raster buffer,
// which the CPU checks. Step 2b: one red triangle on that background, with
// hand-encoded coordinate, vertex and fragment shaders (QPU code), checked
// pixel by pixel.
//
// The control lists follow what Mesa's v3d driver emits for V3D 4.2
// (src/gallium/drivers/v3d/v3dx_draw.c v3d_start_binning(), v3dx_rcl.c
// v3dX(emit_rcl)(), packet layouts from src/broadcom/cle/v3d_packet.xml);
// the job start as Linux v3d_sched.c v3d_bin_job_run() and
// v3d_render_job_run(). Addresses are V3D virtual addresses through the MMU,
// set up as in step 1 (csd.rs).

use common::dma::Dma;

use super::csd::{mmu_flush, state, wait};
use super::{say, Regs};

const V3D_MMUC_CONTROL: usize = 0x1000;
const V3D_MMUC_CONTROL_ENABLE: u32 = 1 << 0;
const V3D_MMU_CTL: usize = 0x1200;
const V3D_MMU_CTL_ENABLES: u32 = (1 << 0) | (1 << 16) | (1 << 19) | (1 << 18) | (1 << 11) | (1 << 10) | (1 << 26) | (1 << 25);
const V3D_MMU_PT_PA_BASE: usize = 0x1204;
const V3D_MMU_ILLEGAL_ADDR: usize = 0x1230;
const V3D_MMU_ILLEGAL_ADDR_ENABLE: u32 = 1 << 31;

const V3D_CTL_SLCACTL: usize = 0x0024;
const V3D_CTL_L2TCACTL: usize = 0x0030;
const V3D_L2TCACTL_L2TFLS: u32 = 1 << 0;
const V3D_L2TCACTL_FLM_CLEAN: u32 = 2 << 1;
const V3D_L2TCACTL_TMUWCF: u32 = 1 << 8;
const V3D_CTL_L2TFLSTA: usize = 0x0034;
const V3D_CTL_L2TFLEND: usize = 0x0038;
const V3D_CTL_INT_STS: usize = 0x0050;
const V3D_CLE_CT0CS: usize = 0x0100;
const V3D_CLE_CT1CS: usize = 0x0104;
const V3D_CLE_CT0CA: usize = 0x0110;
const V3D_CLE_CT1CA: usize = 0x0114;
const V3D_CLE_BFC: usize = 0x0134;
const V3D_CLE_RFC: usize = 0x0138;
const V3D_CLE_CT0QTS: usize = 0x015c;
const V3D_CLE_CT0QTS_ENABLE: u32 = 1 << 1;
const V3D_CLE_CT0QBA: usize = 0x0160;
const V3D_CLE_CT1QBA: usize = 0x0164;
const V3D_CLE_CT0QEA: usize = 0x0168;
const V3D_CLE_CT1QEA: usize = 0x016c;
const V3D_CLE_CT0QMA: usize = 0x0170;
const V3D_CLE_CT0QMS: usize = 0x0174;
const V3D_PTB_BPOA: usize = 0x0308;
const V3D_PTB_BPOS: usize = 0x030c;

const V3D_PTE_WRITEABLE: u32 = 1 << 29;
const V3D_PTE_VALID: u32 = 1 << 28;
const PAGE: usize = 4096;
const PT_ENTRIES: usize = 1 << 20;

// The picture: 256x256 RGBA8, one render target, 32 bpp, so 64x64 tiles.
const WIDTH: u32 = 256;
const HEIGHT: u32 = 256;
const TILE: u32 = 64;
const TILES_X: u32 = WIDTH.div_ceil(TILE);
const TILES_Y: u32 = HEIGHT.div_ceil(TILE);
/// Bytes 0..3 in memory are R, G, B, A (output format rgba8).
pub const CLEAR_COLOR: u32 = 0xff_ff_80_20;

// One arena, mapped at ARENA_VA: offsets of the parts.
const ARENA_VA: u32 = 0x10_0000;
const ARENA: usize = 0x20_0000;
const BCL: usize = 0x0000;
const RCL: usize = 0x1000;
const GTL: usize = 0x2000;
const TILE_STATE: usize = 0x3000;
const FB: usize = 0x4000;
const FB_SIZE: usize = (WIDTH * HEIGHT * 4) as usize;
const TILE_ALLOC: usize = FB + FB_SIZE;
const TILE_ALLOC_SIZE: usize = ((TILES_X * TILES_Y * 64) as usize).next_multiple_of(PAGE) + 8192 + 512 * 1024;
const OVERFLOW: usize = TILE_ALLOC + TILE_ALLOC_SIZE;
const OVERFLOW_SIZE: usize = 256 * 1024;
const SCRATCH: usize = OVERFLOW + OVERFLOW_SIZE;
// Step 2b: shader record, attributes, uniforms and QPU code, one page.
const GEOM: usize = SCRATCH + PAGE;
const SHADER_REC: usize = GEOM; // 32-byte aligned, attribute records at +36
const DEFAULT_ATTRS: usize = GEOM + 0x100; // 16 x vec4, 16-byte aligned
const VERTS0: usize = GEOM + 0x200;
const VERTS1: usize = GEOM + 0x240;
const CS_UNIFS: usize = GEOM + 0x300;
const VS_UNIFS: usize = GEOM + 0x320;
const FS_UNIFS: usize = GEOM + 0x340;
const CS_CODE: usize = GEOM + 0x400;
const VS_CODE: usize = GEOM + 0x500;
const FS_CODE: usize = GEOM + 0x600;
const _: () = assert!(GEOM + PAGE <= ARENA);
/// The triangle's colour: opaque red (bytes R, G, B, A in memory).
pub const TRIANGLE_COLOR: u32 = 0xff_00_00_ff;

/// Where the arena is in V3D memory: ARENA_VA when v3dprobe sets the MMU
/// up itself, the arena BO's address when v3dd does (via_v3dd).
static ARENA_BASE: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(ARENA_VA);

fn va(offset: usize) -> u32 {
    ARENA_BASE.load(std::sync::atomic::Ordering::Relaxed) + offset as u32
}

/// A control list being built: opcode byte, then the fields packed little
/// endian, as Mesa's generated cl_emit() packing.
#[derive(Default)]
struct Cl(Vec<u8>);

/// `value` in `size` bits from bit `start` (after the opcode).
fn f(value: u64, start: u32, size: u32) -> u128 {
    assert!(size == 64 || value < 1 << size, "field {value:#x} wider than {size} bits");
    (value as u128) << start
}

impl Cl {
    fn emit(&mut self, opcode: u8, bytes: usize, fields: u128) {
        self.0.push(opcode);
        self.0.extend_from_slice(&fields.to_le_bytes()[..bytes - 1]);
    }
    fn op(&mut self, opcode: u8) {
        self.emit(opcode, 1, 0);
    }
    fn len(&self) -> usize {
        self.0.len()
    }
}

// Packets (v3d_packet.xml, V3D 4.2)
const FLUSH: u8 = 4;
const START_TILE_BINNING: u8 = 6;
const END_OF_RENDERING: u8 = 13;
const RETURN_FROM_SUB_LIST: u8 = 18;
const FLUSH_VCD_CACHE: u8 = 19;
const START_ADDRESS_OF_GENERIC_TILE_LIST: u8 = 20;
const BRANCH_TO_IMPLICIT_TILE_LIST: u8 = 21;
const SUPERTILE_COORDINATES: u8 = 23;
const CLEAR_TILE_BUFFERS: u8 = 25;
const END_OF_LOADS: u8 = 26;
const END_OF_TILE_MARKER: u8 = 27;
const STORE_TILE_BUFFER_GENERAL: u8 = 29;
const SET_INSTANCEID: u8 = 54;
const PRIM_LIST_FORMAT: u8 = 56;
const OCCLUSION_QUERY_COUNTER: u8 = 92;
const TILE_BINNING_MODE_CFG: u8 = 120;
const TILE_RENDERING_MODE_CFG: u8 = 121;
const MULTICORE_RENDERING_SUPERTILE_CFG: u8 = 122;
const MULTICORE_RENDERING_TILE_LIST_SET_BASE: u8 = 123;
const TILE_COORDINATES: u8 = 124;
const TILE_COORDINATES_IMPLICIT: u8 = 125;
const TILE_LIST_INITIAL_BLOCK_SIZE: u8 = 126;

// Step 2b: state and draw packets.
const VERTEX_ARRAY_PRIMS: u8 = 36;
const GL_SHADER_STATE: u8 = 64;
const VCM_CACHE_SIZE: u8 = 71;
const TRANSFORM_FEEDBACK_SPECS: u8 = 74;
const COLOR_WRITE_MASKS: u8 = 87;
const ZERO_ALL_CENTROID_FLAGS: u8 = 88;
const SAMPLE_STATE: u8 = 91;
const CFG_BITS: u8 = 96;
const ZERO_ALL_FLAT_SHADE_FLAGS: u8 = 97;
const ZERO_ALL_NON_PERSPECTIVE_FLAGS: u8 = 99;
const POINT_SIZE: u8 = 104;
const LINE_WIDTH: u8 = 105;
const CLIP_WINDOW: u8 = 107;
const VIEWPORT_OFFSET: u8 = 108;
const CLIPPER_Z_MIN_MAX_CLIPPING_PLANES: u8 = 109;
const CLIPPER_XY_SCALING: u8 = 110;
const CLIPPER_Z_SCALE_AND_OFFSET: u8 = 111;
const PRIMITIVE_TRIANGLES: u64 = 4;

const BUFFER_NONE: u64 = 8;
const OUTPUT_RGBA8: u64 = 27;
const INTERNAL_TYPE_8: u64 = 2;
const LIST_TRIANGLES: u64 = 2;

fn store_none(cl: &mut Cl) {
    cl.emit(STORE_TILE_BUFFER_GENERAL, 13, f(BUFFER_NONE, 0, 4));
}

fn clear_tile_buffers(cl: &mut Cl) {
    // Clear all render targets and Z/stencil.
    cl.emit(CLEAR_TILE_BUFFERS, 2, f(1, 0, 1) | f(1, 1, 1));
}

/// How the triangle's shaders get the screen coordinates (step 2b).
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Draw {
    /// Step 2a: no primitives, clear only.
    Clear,
    /// The CPU precomputes clip and screen coordinates; the coordinate and
    /// vertex shaders only copy VPM rows (two attributes, one of them int).
    CopyShaders,
    /// One float attribute (clip coordinates); the shaders compute the
    /// screen coordinates with fmul/ffloor/ftoiz, as Mesa's do.
    MathShaders,
}

// QPU code (V3D 4.2), encoded by hand and checked against Mesa's
// qpu_disasm test vectors; see the step 2 notes for the field layout.
const NOP: u64 = 0x3c00_3186_bb80_0000;
const THRSW: u64 = 0x3c20_3186_bb80_0000;
const VPMWT: u64 = 0x3c00_3186_bb81_6000;
/// ldvpmv_in rfN, N (N < 6).
const fn ldvpmv_in(n: u64) -> u64 {
    0x3de0_2180_bc80_7000 | n << 32 | n
}
/// stvpmv N, rfN (N < 6): row index as small immediate, value from raddr_a.
const fn stvpmv_rf(n: u64) -> u64 {
    0x3de0_2180_f883_7000 | n << 6 | n
}
/// Variant A: copy VPM rows 0..5 (CS: Xc Yc Zc Wc Xs Ys).
const CS_COPY: [u64; 16] = [
    ldvpmv_in(0), ldvpmv_in(1), ldvpmv_in(2), ldvpmv_in(3), ldvpmv_in(4), ldvpmv_in(5),
    stvpmv_rf(0), stvpmv_rf(1), stvpmv_rf(2), stvpmv_rf(3), stvpmv_rf(4), stvpmv_rf(5),
    VPMWT, THRSW, NOP, NOP,
];
/// Variant A: copy VPM rows 0..3 (VS: Xs Ys Zs 1/Wc).
const VS_COPY: [u64; 12] = [
    ldvpmv_in(0), ldvpmv_in(1), ldvpmv_in(2), ldvpmv_in(3),
    stvpmv_rf(0), stvpmv_rf(1), stvpmv_rf(2), stvpmv_rf(3),
    VPMWT, THRSW, NOP, NOP,
];
/// Variant B, rows 4..11 of the coordinate shader: r0, r1 = Xs, Ys from
/// rf0, rf1 and the uniforms x_scale, y_scale.
const SCREEN_XY: [u64; 8] = [
    0x3c40_3186_bb80_0000, // ldunif (r5 = x_scale)
    0x5400_3006_bbb8_0000, // fmul r0, rf0, r5
    0x3c40_3186_bb80_0000, // ldunif (r5 = y_scale)
    0x5400_3046_bbb8_0040, // fmul r1, rf1, r5
    0x3c00_3180_f680_0000, // ffloor r0, r0
    0x3c00_3181_f680_1000, // ffloor r1, r1
    0x3c00_3180_f583_8000, // ftoiz r0, r0
    0x3c00_3181_f583_9000, // ftoiz r1, r1
];
const CS_MATH: [u64; 22] = [
    ldvpmv_in(0), ldvpmv_in(1), ldvpmv_in(2), ldvpmv_in(3),
    SCREEN_XY[0], SCREEN_XY[1], SCREEN_XY[2], SCREEN_XY[3],
    SCREEN_XY[4], SCREEN_XY[5], SCREEN_XY[6], SCREEN_XY[7],
    stvpmv_rf(0), stvpmv_rf(1), stvpmv_rf(2), stvpmv_rf(3),
    0x3de0_2180_f880_7004, // stvpmv 4, r0
    0x3de0_2180_f880_f005, // stvpmv 5, r1
    VPMWT, THRSW, NOP, NOP,
];
const VS_MATH: [u64; 22] = [
    ldvpmv_in(0), ldvpmv_in(1), ldvpmv_in(2),
    SCREEN_XY[0], SCREEN_XY[1], SCREEN_XY[2], SCREEN_XY[3],
    SCREEN_XY[4], SCREEN_XY[5], SCREEN_XY[6], SCREEN_XY[7],
    0x55e0_3086_bbf8_00a7, // fmul r2, rf2, 0.5 (z scale)
    0x3de0_3182_0583_a027, // fadd r2, r2, 0.5 (z offset)
    0x3de0_30c6_bbfc_0028, // mov r3, 1.0 (1/Wc, Wc = 1)
    0x3de0_2180_f880_7000, // stvpmv 0, r0
    0x3de0_2180_f880_f001, // stvpmv 1, r1
    0x3de0_2180_f881_7002, // stvpmv 2, r2
    0x3de0_2180_f881_f003, // stvpmv 3, r3
    VPMWT, THRSW, NOP, NOP,
];
/// Fragment shader: opaque red into render target 0 (default TLB config).
const FS_RED: [u64; 11] = [
    THRSW, THRSW, NOP, NOP,
    0x3de0_3006_bbfc_0028, // mov r0, 1.0
    0x3de0_3046_bbfc_0000, // mov r1, 0
    0x3c00_3187_3580_8000, // vfpack tlb, r0, r1 (R, G)
    0x3c00_3187_3580_1000, // vfpack tlb, r1, r0 (B, A)
    THRSW, NOP, NOP,
];

// The triangle in pixels: (64,64), (192,64), (128,192), z = 0.
const TRIANGLE: [(f32, f32); 3] = [(64.0, 64.0), (192.0, 64.0), (128.0, 192.0)];

/// Writes shader record, attribute records, vertices, uniforms and code for
/// `draw` into the arena; returns the GL_SHADER_STATE word (record | count).
fn geometry(bytes: &mut [u8], draw: Draw) -> u32 {
    let put = |bytes: &mut [u8], at: usize, words: &[u32]| {
        for (i, w) in words.iter().enumerate() {
            bytes[at + 4 * i..at + 4 * i + 4].copy_from_slice(&w.to_le_bytes());
        }
    };
    let code = |bytes: &mut [u8], at: usize, insts: &[u64]| {
        for (i, w) in insts.iter().enumerate() {
            bytes[at + 8 * i..at + 8 * i + 8].copy_from_slice(&w.to_le_bytes());
        }
    };
    let one = 1.0f32.to_bits();
    for i in 0..16 {
        put(bytes, DEFAULT_ATTRS + 16 * i, &[0, 0, 0, one]);
    }
    let (cw, ch) = (WIDTH as f32 / 2.0, HEIGHT as f32 / 2.0);
    // Clip coordinates, and screen coordinates relative to the viewport
    // centre in 1/256 pixel.
    let clip = |(x, y): (f32, f32)| ((x - cw) / cw, (y - ch) / ch);
    let screen = |(x, y): (f32, f32)| (((x - cw) * 256.0) as i32, ((y - ch) * 256.0) as i32);
    for (i, &v) in TRIANGLE.iter().enumerate() {
        let (xc, yc) = clip(v);
        put(bytes, VERTS0 + 16 * i, &[xc.to_bits(), yc.to_bits(), 0, one]);
        let (xs, ys) = screen(v);
        put(bytes, VERTS1 + 16 * i, &[xs as u32, ys as u32, 0.5f32.to_bits(), one]);
    }
    // Uniforms: x_scale, y_scale (variant B) and a spare word; the FS gets
    // zero words (the hardware prefetches one).
    let scale = [(cw * 256.0).to_bits(), (ch * 256.0).to_bits(), 0, 0];
    put(bytes, CS_UNIFS, &scale);
    put(bytes, VS_UNIFS, &scale);
    put(bytes, FS_UNIFS, &[0, 0, 0, 0]);
    let (cs, vs): (&[u64], &[u64]) = match draw {
        Draw::MathShaders => (&CS_MATH, &VS_MATH),
        _ => (&CS_COPY, &VS_COPY),
    };
    code(bytes, CS_CODE, cs);
    code(bytes, VS_CODE, vs);
    code(bytes, FS_CODE, &FS_RED);

    // GL Shader State Record (v42): clipping on, implicit point/line
    // varyings off, 1-sector VPM segments, shared input/output.
    let mut rec = [0u8; 36];
    rec[..8].copy_from_slice(&[0x02, 0x00, 0x04, 0x00, 0x01, 0x01, 0x01, 0x01]);
    let words = [
        va(DEFAULT_ATTRS),
        va(FS_CODE) | 0b101, // 4-way, NaNs, not single segment
        va(FS_UNIFS),
        va(VS_CODE) | 0b111, // 4-way, start in final section, NaNs
        va(VS_UNIFS),
        va(CS_CODE) | 0b111,
        va(CS_UNIFS),
    ];
    for (i, w) in words.iter().enumerate() {
        rec[8 + 4 * i..12 + 4 * i].copy_from_slice(&w.to_le_bytes());
    }
    bytes[SHADER_REC..SHADER_REC + 36].copy_from_slice(&rec);
    // Attribute records: address, vec size/type, read counts, stride, max index.
    let attr = |bytes: &mut [u8], n: usize, addr: u32, ty: u8, reads: u8| {
        let at = SHADER_REC + 36 + 16 * n;
        bytes[at..at + 4].copy_from_slice(&addr.to_le_bytes());
        bytes[at + 4] = ty;
        bytes[at + 5] = reads;
        bytes[at + 6..at + 8].fill(0);
        bytes[at + 8..at + 12].copy_from_slice(&16u32.to_le_bytes());
        bytes[at + 12..at + 16].copy_from_slice(&0xff_ffffu32.to_le_bytes());
    };
    match draw {
        Draw::MathShaders => {
            // vec4 float: CS reads 4, VS reads 3.
            attr(bytes, 0, va(VERTS0), 0x08, 0x34);
            va(SHADER_REC) | 1
        }
        _ => {
            // vec4 float, CS reads 4, VS none; vec4 signed int read as int,
            // CS reads 2 (Xs, Ys), VS reads 4.
            attr(bytes, 0, va(VERTS0), 0x08, 0x04);
            attr(bytes, 1, va(VERTS1), 0xb8, 0x42);
            va(SHADER_REC) | 2
        }
    }
}

/// The state and draw packets for one triangle (v3dX(emit_state)(),
/// v3d_emit_gl_shader_state(), v3d_draw_vbo() for a first draw).
fn draw_packets(cl: &mut Cl, shader_state: u32) {
    let fl = |x: f32| x.to_bits() as u64;
    cl.emit(CLIP_WINDOW, 9, f(0, 0, 16) | f(0, 16, 16) | f(WIDTH as u64, 32, 16) | f(HEIGHT as u64, 48, 16));
    // Both faces, depth func always.
    cl.emit(CFG_BITS, 4, f(1, 0, 1) | f(1, 1, 1) | f(7, 12, 3));
    cl.emit(POINT_SIZE, 5, f(fl(1.0), 0, 32));
    cl.emit(LINE_WIDTH, 5, f(fl(1.0), 0, 32));
    let (hw, hh) = (WIDTH as f32 / 2.0, HEIGHT as f32 / 2.0);
    cl.emit(CLIPPER_XY_SCALING, 9, f(fl(hw * 256.0), 0, 32) | f(fl(hh * 256.0), 32, 32));
    cl.emit(CLIPPER_Z_SCALE_AND_OFFSET, 9, f(fl(0.5), 0, 32) | f(fl(0.5), 32, 32));
    cl.emit(CLIPPER_Z_MIN_MAX_CLIPPING_PLANES, 9, f(fl(0.0), 0, 32) | f(fl(1.0), 32, 32));
    cl.emit(VIEWPORT_OFFSET, 9, f((hw * 256.0) as u64, 0, 22) | f((hh * 256.0) as u64, 32, 22));
    cl.emit(COLOR_WRITE_MASKS, 5, 0);
    cl.op(ZERO_ALL_FLAT_SHADE_FLAGS);
    cl.op(ZERO_ALL_NON_PERSPECTIVE_FLAGS);
    cl.op(ZERO_ALL_CENTROID_FLAGS);
    cl.emit(TRANSFORM_FEEDBACK_SPECS, 2, 0);
    cl.emit(SAMPLE_STATE, 5, f(0xf, 0, 4) | f(0x3f80, 16, 16));
    cl.emit(VCM_CACHE_SIZE, 2, f(4, 0, 4) | f(4, 4, 4));
    cl.emit(GL_SHADER_STATE, 5, f(shader_state as u64, 0, 32));
    cl.emit(VERTEX_ARRAY_PRIMS, 10, f(PRIMITIVE_TRIANGLES, 0, 8) | f(3, 8, 32) | f(0, 40, 32));
}

/// v3d_start_binning(), the draw (if any) and the epilogue's FLUSH.
fn binning_cl(shader_state: Option<u32>) -> Cl {
    let mut cl = Cl::default();
    cl.emit(
        TILE_BINNING_MODE_CFG,
        9,
        f(WIDTH as u64 - 1, 32, 16) | f(HEIGHT as u64 - 1, 48, 16) | f(0, 8, 4) | f(0, 12, 2),
    );
    cl.op(FLUSH_VCD_CACHE);
    cl.emit(OCCLUSION_QUERY_COUNTER, 5, 0);
    cl.op(START_TILE_BINNING);
    if let Some(state) = shader_state {
        draw_packets(&mut cl, state);
    }
    cl.op(FLUSH);
    cl
}

/// The generic per-tile list (v3d_rcl_emit_generic_per_tile_list()).
fn generic_tile_list() -> Cl {
    let mut cl = Cl::default();
    cl.op(TILE_COORDINATES_IMPLICIT);
    cl.op(END_OF_LOADS);
    cl.emit(PRIM_LIST_FORMAT, 2, f(LIST_TRIANGLES, 0, 6));
    cl.emit(SET_INSTANCEID, 5, 0);
    cl.emit(BRANCH_TO_IMPLICIT_TILE_LIST, 2, 0);
    // Render target 0, raster, rgba8, stride in bytes, sample 0.
    cl.emit(
        STORE_TILE_BUFFER_GENERAL,
        13,
        f(0, 0, 4) | f(0, 4, 3) | f(OUTPUT_RGBA8, 12, 6) | f(WIDTH as u64 * 4, 28, 20) | f(va(FB) as u64, 64, 32),
    );
    clear_tile_buffers(&mut cl);
    cl.op(END_OF_TILE_MARKER);
    cl.op(RETURN_FROM_SUB_LIST);
    cl
}

/// v3dX(emit_rcl)() and emit_render_layer() for one layer, one render target.
fn rcl(gtl_start: u32, gtl_end: u32) -> Cl {
    let mut cl = Cl::default();
    // Common (sub-id 0): one RT, image size, 32 bpp, early Z off.
    cl.emit(
        TILE_RENDERING_MODE_CFG,
        9,
        f(0, 0, 4) | f(0, 4, 4) | f(WIDTH as u64, 8, 16) | f(HEIGHT as u64, 24, 16) | f(0, 40, 2) | f(1, 46, 1),
    );
    // Clear colours part 1 (sub-id 3): RT 0, the 32 low bits.
    cl.emit(TILE_RENDERING_MODE_CFG, 9, f(3, 0, 4) | f(0, 4, 4) | f(CLEAR_COLOR as u64, 8, 32));
    // Colour (sub-id 1): RT 0 32 bpp, internal type 8, no clamp.
    cl.emit(TILE_RENDERING_MODE_CFG, 9, f(1, 0, 4) | f(0, 4, 2) | f(INTERNAL_TYPE_8, 6, 4) | f(0, 10, 2));
    // Z/S clear values (sub-id 2), last: Z 1.0, stencil 0.
    cl.emit(TILE_RENDERING_MODE_CFG, 9, f(2, 0, 4) | f(1.0f32.to_bits() as u64, 16, 32));
    // Auto-chained tile lists, first block 64 bytes (as the binner).
    cl.emit(TILE_LIST_INITIAL_BLOCK_SIZE, 2, f(1, 2, 1));
    cl.emit(MULTICORE_RENDERING_TILE_LIST_SET_BASE, 5, f(va(TILE_ALLOC) as u64, 0, 32));
    // Supertiles of 1x1 tiles (fewer than 256 of them).
    cl.emit(
        MULTICORE_RENDERING_SUPERTILE_CFG,
        9,
        f(0, 0, 8)
            | f(0, 8, 8)
            | f(TILES_X as u64, 16, 8)
            | f(TILES_Y as u64, 24, 8)
            | f(TILES_X as u64, 32, 12)
            | f(TILES_Y as u64, 44, 12)
            | f(0, 61, 3),
    );
    // The initial tile clear and the GFXH-1742 dummy store.
    for i in 0..2 {
        cl.emit(TILE_COORDINATES, 4, 0);
        cl.op(END_OF_LOADS);
        store_none(&mut cl);
        if i == 0 {
            clear_tile_buffers(&mut cl);
        }
        cl.op(END_OF_TILE_MARKER);
    }
    cl.op(FLUSH_VCD_CACHE);
    cl.emit(START_ADDRESS_OF_GENERIC_TILE_LIST, 9, f(gtl_start as u64, 0, 32) | f(gtl_end as u64, 32, 32));
    for y in 0..TILES_Y {
        for x in 0..TILES_X {
            cl.emit(SUPERTILE_COORDINATES, 3, f(x as u64, 0, 8) | f(y as u64, 8, 8));
        }
    }
    cl.op(END_OF_RENDERING);
    cl
}

fn cle_state(core: &Regs) {
    say(&format!(
        "render: CT0CS {:#x} CT0CA {:#x}, CT1CS {:#x} CT1CA {:#x}, BFC {:#x}, RFC {:#x}, CTL_INT_STS {:#x}",
        core.read(V3D_CLE_CT0CS),
        core.read(V3D_CLE_CT0CA),
        core.read(V3D_CLE_CT1CS),
        core.read(V3D_CLE_CT1CA),
        core.read(V3D_CLE_BFC),
        core.read(V3D_CLE_RFC),
        core.read(V3D_CTL_INT_STS)
    ));
}

/// Shows the picture (RGBA8 words, R in the low byte) in an Orbital window
/// for `seconds`, so vc4shot can take it from the screen (step 2c).
pub fn show(pixels: &[u32], seconds: u64) {
    use orbclient::{Color, Renderer, Window};
    let Some(mut window) = Window::new(64, 64, WIDTH, HEIGHT, "v3d") else {
        say("show: cannot open an Orbital window (is Orbital running?)");
        return;
    };
    for (dst, &p) in window.data_mut().iter_mut().zip(pixels) {
        *dst = Color::rgba(p as u8, (p >> 8) as u8, (p >> 16) as u8, (p >> 24) as u8);
    }
    window.sync();
    say(&format!("show: the picture is in an Orbital window ({WIDTH}x{HEIGHT} at 64,64) for {seconds} s"));
    std::thread::sleep(std::time::Duration::from_secs(seconds));
}

/// Is pixel centre (x + 0.5, y + 0.5) inside the triangle (either Y
/// direction is accepted by the caller via `flip`)?
fn inside(x: u32, y: u32, flip: bool) -> bool {
    let p = (x as f32 + 0.5, y as f32 + 0.5);
    let v: Vec<(f32, f32)> =
        TRIANGLE.iter().map(|&(vx, vy)| if flip { (vx, HEIGHT as f32 - vy) } else { (vx, vy) }).collect();
    let edge = |a: (f32, f32), b: (f32, f32)| (b.0 - a.0) * (p.1 - a.1) - (b.1 - a.1) * (p.0 - a.0);
    let (e0, e1, e2) = (edge(v[0], v[1]), edge(v[1], v[2]), edge(v[2], v[0]));
    (e0 >= 0.0 && e1 >= 0.0 && e2 >= 0.0) || (e0 <= 0.0 && e1 <= 0.0 && e2 <= 0.0)
}

/// Checks the triangle frame: red inside, the clear colour outside; pixels
/// on the edges may go either way.
fn check_triangle(pixels: &[u32]) -> bool {
    let red = pixels.iter().filter(|&&p| p == TRIANGLE_COLOR).count();
    let clear = pixels.iter().filter(|&&p| p == CLEAR_COLOR).count();
    let other = pixels.len() - red - clear;
    // A coarse picture, 32x16 characters: '#' red, '.' clear, '?' other.
    for row in 0..16u32 {
        let line: String = (0..32u32)
            .map(|col| match pixels[((row * 16 + 8) * WIDTH + col * 8 + 4) as usize] {
                TRIANGLE_COLOR => '#',
                CLEAR_COLOR => '.',
                _ => '?',
            })
            .collect();
        say(&format!("render: |{line}|"));
    }
    let at = |x: u32, y: u32| pixels[(y * WIDTH + x) as usize];
    // Base at row 64 or (flipped) at row 192.
    let flip = at(70, 70) != TRIANGLE_COLOR && at(70, HEIGHT - 71) == TRIANGLE_COLOR;
    let mut wrong = 0usize;
    let mut first_wrong = None;
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            // Skip pixels near an edge: in for one rule, out for a neighbour.
            let near = [(0i32, 0i32), (1, 0), (-1, 0), (0, 1), (0, -1)].iter().any(|&(dx, dy)| {
                let (nx, ny) = ((x as i32 + dx).clamp(0, WIDTH as i32 - 1), (y as i32 + dy).clamp(0, HEIGHT as i32 - 1));
                inside(nx as u32, ny as u32, flip) != inside(x, y, flip)
            });
            if near {
                continue;
            }
            let want = if inside(x, y, flip) { TRIANGLE_COLOR } else { CLEAR_COLOR };
            if at(x, y) != want {
                wrong += 1;
                first_wrong.get_or_insert((x, y, at(x, y)));
            }
        }
    }
    say(&format!(
        "render: triangle: {red} red, {clear} clear, {other} other pixels (area 8192 expected red); base at row {}; centre {:#x}; {wrong} wrong away from the edges, first {first_wrong:x?}",
        if flip { HEIGHT - 64 } else { 64 },
        at(WIDTH / 2, HEIGHT / 2)
    ));
    let ok = wrong == 0 && other == 0 && (7900..=8500).contains(&red);
    if ok {
        say(&format!("render: PASS: a red triangle on {CLEAR_COLOR:#010x}, drawn by the QPUs through bin and render control lists"));
    } else {
        say("render: FAIL: the triangle is not right");
    }
    ok
}

/// Runs one frame: clear only, or a red triangle on the clear colour; true
/// when the pixels are right. The picture is returned too.
pub fn run(hub: &Regs, core: &Regs, draw: Draw) -> (bool, Vec<u32>) {
    let mut pt = match Dma::<[u32]>::zeroed_slice(PT_ENTRIES) {
        Ok(dma) => unsafe { dma.assume_init() },
        Err(err) => {
            say(&format!("render: FAIL: no 4 MiB for the page table: {err:?}"));
            return (false, Vec::new());
        }
    };
    let mut arena = match Dma::<[u32]>::zeroed_slice(ARENA / 4) {
        Ok(dma) => unsafe { dma.assume_init() },
        Err(err) => {
            say(&format!("render: FAIL: no {} KiB of contiguous memory: {err:?}", ARENA / 1024));
            return (false, Vec::new());
        }
    };
    let pt_phys = pt.physical();
    let arena_phys = arena.physical();
    say(&format!("render: page table at {pt_phys:#x}, arena at {arena_phys:#x} (V3D {ARENA_VA:#x})"));
    for page in 0..ARENA / PAGE {
        pt[ARENA_VA as usize / PAGE + page] = V3D_PTE_WRITEABLE | V3D_PTE_VALID | ((arena_phys / PAGE + page) as u32);
    }

    say(&format!("render: draw {draw:?}"));
    let bcl;
    let gtl = generic_tile_list();
    let rcl = rcl(va(GTL), va(GTL) + gtl.len() as u32);
    {
        let bytes: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(arena.as_mut_ptr() as *mut u8, ARENA) };
        let shader_state = (draw != Draw::Clear).then(|| geometry(bytes, draw));
        bcl = binning_cl(shader_state);
        if shader_state.is_some() {
            say(&format!("render: shader record + attributes {:02x?}", &bytes[SHADER_REC..SHADER_REC + 68]));
        }
        bytes[BCL..BCL + bcl.len()].copy_from_slice(&bcl.0);
        bytes[RCL..RCL + rcl.len()].copy_from_slice(&rcl.0);
        bytes[GTL..GTL + gtl.len()].copy_from_slice(&gtl.0);
    }
    let fb = FB / 4;
    for i in 0..FB_SIZE / 4 {
        arena[fb + i] = 0xdead_beef;
    }
    say(&format!("render: BCL {} bytes {:02x?}", bcl.len(), bcl.0));
    say(&format!("render: generic tile list {} bytes {:02x?}", gtl.len(), gtl.0));
    say(&format!("render: RCL {} bytes, first {:02x?}", rcl.len(), &rcl.0[..rcl.len().min(64)]));

    // MMU as in step 1.
    core.write(V3D_CTL_L2TFLSTA, 0);
    core.write(V3D_CTL_L2TFLEND, !0);
    hub.write(V3D_MMU_PT_PA_BASE, (pt_phys / PAGE) as u32);
    hub.write(V3D_MMU_CTL, V3D_MMU_CTL_ENABLES);
    hub.write(V3D_MMU_ILLEGAL_ADDR, ((arena_phys + SCRATCH) / PAGE) as u32 | V3D_MMU_ILLEGAL_ADDR_ENABLE);
    hub.write(V3D_MMUC_CONTROL, V3D_MMUC_CONTROL_ENABLE);
    if !mmu_flush(hub) {
        say("render: FAIL: MMU flush");
        return (false, Vec::new());
    }
    let invalidate = || {
        core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_L2TFLS);
        core.write(V3D_CTL_SLCACTL, 0x0f0f_0f0f);
    };

    // Bin job (v3d_bin_job_run()).
    invalidate();
    let bfc = core.read(V3D_CLE_BFC) & 0xff;
    core.write(V3D_PTB_BPOA, va(OVERFLOW));
    core.write(V3D_PTB_BPOS, OVERFLOW_SIZE as u32);
    core.write(V3D_CLE_CT0QMA, va(TILE_ALLOC));
    core.write(V3D_CLE_CT0QMS, TILE_ALLOC_SIZE as u32);
    core.write(V3D_CLE_CT0QTS, va(TILE_STATE) | V3D_CLE_CT0QTS_ENABLE);
    core.write(V3D_CLE_CT0QBA, va(BCL));
    let start = std::time::Instant::now();
    core.write(V3D_CLE_CT0QEA, va(BCL) + bcl.len() as u32);
    let binned = wait("bin job", 1000, || core.read(V3D_CLE_BFC) & 0xff != bfc);
    say(&format!("render: bin job {} after {} us", if binned { "done" } else { "NOT done" }, start.elapsed().as_micros()));
    cle_state(core);
    state(hub, core);
    if !binned {
        say("render: FAIL: the bin job did not finish");
        return (false, Vec::new());
    }

    // Render job (v3d_render_job_run()).
    invalidate();
    let rfc = core.read(V3D_CLE_RFC) & 0xff;
    core.write(V3D_CLE_CT1QBA, va(RCL));
    let start = std::time::Instant::now();
    core.write(V3D_CLE_CT1QEA, va(RCL) + rcl.len() as u32);
    let rendered = wait("render job", 1000, || core.read(V3D_CLE_RFC) & 0xff != rfc);
    say(&format!("render: render job {} after {} us", if rendered { "done" } else { "NOT done" }, start.elapsed().as_micros()));
    cle_state(core);
    state(hub, core);
    if !rendered {
        say("render: FAIL: the render job did not finish");
        return (false, Vec::new());
    }
    core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_TMUWCF);
    wait("TMU write combiner flush", 100, || core.read(V3D_CTL_L2TCACTL) & V3D_L2TCACTL_TMUWCF == 0);
    core.write(V3D_CTL_L2TCACTL, V3D_L2TCACTL_L2TFLS | V3D_L2TCACTL_FLM_CLEAN);
    wait("L2T clean", 100, || core.read(V3D_CTL_L2TCACTL) & V3D_L2TCACTL_L2TFLS == 0);

    let pixels = &arena[fb..fb + FB_SIZE / 4];
    let right = pixels.iter().filter(|&&p| p == CLEAR_COLOR).count();
    let untouched = pixels.iter().filter(|&&p| p == 0xdead_beef).count();
    say(&format!(
        "render: pixels: {right} of {} have {CLEAR_COLOR:#010x}, {untouched} untouched; corners {:#x} {:#x} {:#x} {:#x}, centre {:#x}",
        pixels.len(),
        pixels[0],
        pixels[WIDTH as usize - 1],
        pixels[(HEIGHT as usize - 1) * WIDTH as usize],
        pixels[pixels.len() - 1],
        pixels[(HEIGHT as usize / 2) * WIDTH as usize + WIDTH as usize / 2]
    ));
    // The first row of tiles, one word per tile, for a quick look at partial results.
    let row: Vec<u32> = (0..TILES_X).map(|t| pixels[(t * TILE) as usize]).collect();
    say(&format!("render: first pixel of each tile in row 0: {row:#x?}"));
    if draw != Draw::Clear {
        let ok = check_triangle(pixels);
        return (ok, pixels.to_vec());
    }
    let ok = right == pixels.len();
    if ok {
        say(&format!("render: PASS: {WIDTH}x{HEIGHT} cleared to {CLEAR_COLOR:#010x} by bin and render control lists"));
    } else {
        say("render: FAIL: not every pixel has the clear colour");
    }
    (ok, pixels.to_vec())
}

/// Step 3b (redox-rpi#598): the same frame through v3dd, `loops` times. The
/// arena is a BO of v3dd's, a new one every tenth loop; the first frame is
/// checked as in `run`, every later one must equal it. No register access
/// here: v3dd owns V3D.
pub fn via_v3dd(draw: Draw, loops: u32) -> bool {
    let mut v3d = match super::client::Client::open() {
        Ok(v3d) => v3d,
        Err(err) => {
            say(&format!("render: FAIL: {err}"));
            return false;
        }
    };
    say(&format!("render: draw {draw:?} through v3dd, {loops} frames"));
    let mut first: Option<Vec<u32>> = None;
    let mut run = |v3d: &mut super::client::Client| -> Result<(u128, u128), String> {
        let (mut us_sum, mut us_max) = (0u128, 0u128);
        let mut arena = None;
        let mut lists = (0usize, 0usize);
        for i in 0..loops {
            if i % 10 == 0 {
                if let Some(old) = arena.take() {
                    v3d.free(&old)?;
                }
                let bo = v3d.bo(ARENA)?;
                ARENA_BASE.store(bo.va, std::sync::atomic::Ordering::Relaxed);
                // Control lists and geometry for this address.
                let mut bytes = vec![0u8; ARENA];
                let shader_state = (draw != Draw::Clear).then(|| geometry(&mut bytes, draw));
                let bcl = binning_cl(shader_state);
                let gtl = generic_tile_list();
                let rcl = rcl(va(GTL), va(GTL) + gtl.len() as u32);
                v3d.put_bytes(&bo, BCL, &bcl.0)?;
                v3d.put_bytes(&bo, RCL, &rcl.0)?;
                v3d.put_bytes(&bo, GTL, &gtl.0)?;
                if shader_state.is_some() {
                    v3d.put_bytes(&bo, GEOM, &bytes[GEOM..GEOM + PAGE])?;
                }
                if i == 0 {
                    say(&format!("render: arena BO at {:#x}, BCL {} bytes, RCL {} bytes", bo.va, bcl.len(), rcl.len()));
                }
                lists = (bcl.len(), rcl.len());
                arena = Some(bo);
            }
            let bo = arena.as_ref().unwrap();
            v3d.fill(bo, FB / 4, FB_SIZE / 4, 0xdead_beef)?;
            v3d.fill(bo, TILE_STATE / 4, PAGE / 4, 0)?;
            v3d.fill(bo, TILE_ALLOC / 4, TILE_ALLOC_SIZE / 4, 0)?;
            let us = v3d.job(&format!(
                "cl {:#x} {:#x} {:#x} {:#x} {:#x} {:#x} {:#x}",
                va(BCL),
                va(BCL) + lists.0 as u32,
                va(RCL),
                va(RCL) + lists.1 as u32,
                va(TILE_ALLOC),
                TILE_ALLOC_SIZE,
                va(TILE_STATE)
            ))?;
            us_sum += us;
            us_max = us_max.max(us);
            let pixels = v3d.get(bo, FB / 4, FB_SIZE / 4)?;
            match &first {
                None => {
                    let ok = if draw == Draw::Clear {
                        let right = pixels.iter().filter(|&&p| p == CLEAR_COLOR).count();
                        say(&format!("render: frame 0: {right} of {} pixels have {CLEAR_COLOR:#010x}", pixels.len()));
                        right == pixels.len()
                    } else {
                        check_triangle(&pixels)
                    };
                    if !ok {
                        return Err("frame 0 is not right".into());
                    }
                    first = Some(pixels);
                }
                Some(want) => {
                    if pixels != *want {
                        let diff = pixels.iter().zip(want).filter(|(a, b)| a != b).count();
                        return Err(format!("frame {i}: {diff} pixels differ from frame 0 (arena at {:#x})", bo.va));
                    }
                }
            }
        }
        if let Some(bo) = arena {
            v3d.free(&bo)?;
        }
        Ok((us_sum, us_max))
    };
    let outcome = run(&mut v3d);
    if let Ok(stats) = v3d.ask("stats") {
        say(&format!("render: v3dd stats: {stats}"));
    }
    match outcome {
        Ok((sum, max)) => {
            say(&format!(
                "render: PASS: {loops} of {loops} {draw:?} frames through /scheme/v3d right, {} us average, {max} us max (bin + render, daemon side)",
                sum / loops.max(1) as u128
            ));
            true
        }
        Err(err) => {
            say(&format!("render: FAIL: {err}"));
            false
        }
    }
}
