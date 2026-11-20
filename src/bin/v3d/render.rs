// SPDX-License-Identifier: GPL-2.0-only
//
// V3D step 2a (redox-rpi#596): a bin job and a render job from hand-built
// control lists. The binner gets no primitives; the render control list
// clears every tile to one colour and stores it to a raster buffer, which
// the CPU checks. No shaders yet.
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
const _: () = assert!(SCRATCH + PAGE <= ARENA);

fn va(offset: usize) -> u32 {
    ARENA_VA + offset as u32
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

/// v3d_start_binning() and the epilogue's FLUSH; prims are added between
/// START_TILE_BINNING and FLUSH later (step 2b).
fn bcl() -> Cl {
    let mut cl = Cl::default();
    cl.emit(
        TILE_BINNING_MODE_CFG,
        9,
        f(WIDTH as u64 - 1, 32, 16) | f(HEIGHT as u64 - 1, 48, 16) | f(0, 8, 4) | f(0, 12, 2),
    );
    cl.op(FLUSH_VCD_CACHE);
    cl.emit(OCCLUSION_QUERY_COUNTER, 5, 0);
    cl.op(START_TILE_BINNING);
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

/// Runs the clear-only frame; true when every pixel has CLEAR_COLOR. The
/// picture is returned too.
pub fn run(hub: &Regs, core: &Regs) -> (bool, Vec<u32>) {
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

    let bcl = bcl();
    let gtl = generic_tile_list();
    let rcl = rcl(va(GTL), va(GTL) + gtl.len() as u32);
    {
        let bytes: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(arena.as_mut_ptr() as *mut u8, ARENA) };
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
    let ok = right == pixels.len();
    if ok {
        say(&format!("render: PASS: {WIDTH}x{HEIGHT} cleared to {CLEAR_COLOR:#010x} by bin and render control lists"));
    } else {
        say("render: FAIL: not every pixel has the clear colour");
    }
    (ok, pixels.to_vec())
}
