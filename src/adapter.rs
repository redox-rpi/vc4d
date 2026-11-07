// SPDX-License-Identifier: GPL-2.0-only
//
// The display as Redox's driver-graphics sees it: one connector, one CRTC
// (the HVS channel the firmware enabled), a primary and a cursor plane.
//
// The adapter shape (dumb buffers, connector probing, the set_crtc and
// set_plane hooks) follows redox-rpi's rpi-fbd (MIT). How the planes become
// a display list follows Linux v6.12 drivers/gpu/drm/vc4/vc4_plane.c
// (vc4_plane_atomic_check() clipping to the screen) and vc4_hvs.c
// (vc4_hvs_atomic_flush(): write the list, then point the channel at it).

use std::alloc::{self, Layout};
use std::fmt;
use std::ptr::{self, NonNull};
use std::sync::Arc;

use common::dma::Dma;
use driver_graphics::kms::connector::{modeinfo_for_size, KmsConnectorDriver, KmsConnectorStatus};
use driver_graphics::kms::crtc::{KmsCrtc, KmsCrtcState};
use driver_graphics::kms::framebuffer::KmsFramebuffer;
use driver_graphics::kms::objects::{KmsObjectId, KmsObjects};
use driver_graphics::kms::plane::{KmsPlane, KmsPlaneDriver, KmsPlaneState};
use driver_graphics::{Buffer, Damage, DumbBufferConfig, GraphicsAdapter};
use syscall::PAGE_SIZE;

use crate::hvs::{self, Channel, Hvs, Plane};
use crate::shot::{self, ShotPlane};

/// Two places for our display lists, far above the firmware's. A new list
/// goes into the one the HVS is not using, then the channel is pointed at
/// it, so the HVS never reads a half-written list.
const LIST_SLOTS: [usize; 2] = [0x800, 0x840];

/// Orbital draws its pointer into a 64x64 ARGB buffer.
const CURSOR_SIZE: u32 = 64;

#[derive(Debug)]
pub struct Vc4Adapter {
    pub hvs: Hvs,
    pub channel: Channel,
    /// Bus address minus physical address of RAM.
    pub ram_bus_offset: usize,
    /// The firmware's list (its framebuffer, which vesad draws into): shown
    /// while no client has a picture on the primary plane.
    pub firmware_list: usize,
    pub next_slot: usize,
    pub primary: Option<Shown>,
    pub cursor: Option<Shown>,
    /// A client set no mode: the screen shows only the background.
    pub blanked: bool,
    /// Log every new display list, not only the first few.
    pub log_lists: bool,
    /// Display lists written so far.
    pub lists: u32,
    /// The planes shown now, for screenshots (shot.rs).
    pub shown: shot::Shown,
}

/// The first display lists always go to the log.
const LOGGED_LISTS: u32 = 4;

/// A plane in the current display list, with its buffer, which must live
/// as long as the HVS reads it.
#[derive(Debug)]
pub struct Shown {
    plane: Plane,
    /// The first visible pixel in our memory.
    pixels: *const u8,
    _buffer: Arc<Vc4Buffer>,
}

impl Vc4Adapter {
    /// Writes the display list for the current planes and points the
    /// channel at it.
    fn update(&mut self) {
        let index = self.channel.index;
        let Some(primary) = &self.primary else {
            println!("vc4d: no primary plane, firmware list again");
            self.hvs.set_list_start(index, self.firmware_list);
            self.shown.lock().unwrap().clear();
            return;
        };
        let planes: Vec<Plane> = if self.blanked {
            Vec::new()
        } else {
            [Some(primary), self.cursor.as_ref()]
                .into_iter()
                .flatten()
                .map(|shown| shown.plane)
                .collect()
        };
        *self.shown.lock().unwrap() = if self.blanked {
            Vec::new()
        } else {
            [Some(primary), self.cursor.as_ref()]
                .into_iter()
                .flatten()
                .map(|s| ShotPlane { plane: s.plane, pixels: s.pixels, _buffer: s._buffer.clone() })
                .collect()
        };
        let list = hvs::display_list(&planes);
        let start = LIST_SLOTS[self.next_slot];
        self.hvs.write_dlist(start, &list);
        self.hvs.set_list_start(index, start);
        self.next_slot = 1 - self.next_slot;
        self.lists += 1;
        if self.log_lists || self.lists <= LOGGED_LISTS {
            println!("vc4d: list {} at {start}: {planes:?} {list:08x?}", self.lists);
        }
    }

    /// The part of the framebuffer that is on screen, with the top left
    /// corner at (x, y) on the screen; None if nothing of it is.
    fn place(&self, fb: &KmsFramebuffer<Self>, x: i32, y: i32, alpha: bool) -> Option<Shown> {
        let Some(mem) = &fb.buffer.dma else {
            println!("vc4d: buffer {}x{} not in contiguous memory, not shown", fb.width, fb.height);
            return None;
        };
        let (screen_w, screen_h) = (self.channel.width as i32, self.channel.height as i32);
        // Columns and rows cut off at the left and top.
        let (skip_x, skip_y) = ((-x).max(0), (-y).max(0));
        let width = (fb.width as i32 - skip_x).min(screen_w - x.max(0));
        let height = (fb.height as i32 - skip_y).min(screen_h - y.max(0));
        if width <= 0 || height <= 0 {
            return None;
        }
        let offset = skip_y as usize * fb.pitch as usize + skip_x as usize * 4;
        Some(Shown {
            plane: Plane {
                bus: (mem.physical() + self.ram_bus_offset + offset) as u32,
                pitch: fb.pitch,
                x: x.max(0) as u32,
                y: y.max(0) as u32,
                width: width as u32,
                height: height as u32,
                alpha,
            },
            pixels: unsafe { fb.buffer.ptr.as_ptr().cast::<u8>().add(offset) },
            _buffer: fb.buffer.clone(),
        })
    }
}

#[derive(Debug)]
pub struct Connector;

impl KmsConnectorDriver for Connector {
    type State = ();
}

#[derive(Debug)]
pub struct Vc4Plane {
    cursor: bool,
}

impl KmsPlaneDriver for Vc4Plane {
    type State = ();
}

impl GraphicsAdapter for Vc4Adapter {
    type Connector = Connector;
    type Crtc = ();
    type Plane = Vc4Plane;

    type Buffer = Vc4Buffer;
    type Framebuffer = ();

    fn name(&self) -> &'static [u8] {
        b"vc4d"
    }

    fn desc(&self) -> &'static [u8] {
        b"VideoCore IV HVS"
    }

    fn init(&mut self, objects: &mut KmsObjects<Self>) {
        let (crtc, _primary_plane_id) = objects.add_crtc(
            (),
            (),
            Vc4Plane { cursor: false },
            (),
            Some((Vc4Plane { cursor: true }, ())),
        );
        objects.add_connector(Connector, (), &[crtc]);
    }

    fn get_unique(&self) -> String {
        "vc4d".to_owned()
    }

    fn min_max_fb_size(&self) -> (u32, u32, u32, u32) {
        (1, 4096, 1, 4096)
    }

    /// The HVS scans the dumb buffers out directly. They are DMA memory,
    /// uncached, where reading is slow: clients draw into a shadow buffer in
    /// normal memory and copy the changed part over.
    fn dumb_buffer_config(&self) -> Option<DumbBufferConfig> {
        Some(DumbBufferConfig {
            preferred_depth: 24,
            prefer_shadow: true,
        })
    }

    fn cursor_size(&self) -> Option<(u64, u64)> {
        Some((CURSOR_SIZE.into(), CURSOR_SIZE.into()))
    }

    /// The mode the firmware set up at power-on, the only one for now.
    fn probe_connector(
        &mut self,
        objects: &mut KmsObjects<Self>,
        id: KmsObjectId,
    ) -> syscall::Result<()> {
        let mut connector = objects.get_connector(id)?.lock().unwrap();
        connector.connection = KmsConnectorStatus::Connected;
        connector.connector_type = drm_sys::DRM_MODE_CONNECTOR_HDMIA;
        connector.modes = vec![modeinfo_for_size(self.channel.width, self.channel.height)];
        Ok(())
    }

    fn create_dumb_buffer(&mut self, width: u32, height: u32) -> (Self::Buffer, u32) {
        (Vc4Buffer::new(width as usize, height as usize), width * 4)
    }

    fn map_dumb_buffer(&mut self, buffer: &Self::Buffer) -> *mut u8 {
        buffer.ptr.as_ptr().cast::<u8>()
    }

    fn create_framebuffer(&mut self, _buffer: &Self::Buffer) -> Self::Framebuffer {}

    fn set_crtc(
        &mut self,
        _objects: &KmsObjects<Self>,
        crtc: &KmsCrtc<Self>,
        state: KmsCrtcState<Self>,
        _connector_ids: &[KmsObjectId],
    ) -> syscall::Result<()> {
        let blanked = state.mode.is_none();
        if blanked != self.blanked {
            self.blanked = blanked;
            self.update();
        }
        *crtc.state.lock().unwrap() = state;
        Ok(())
    }

    /// Damage needs nothing: the HVS shows the buffer itself. A new buffer,
    /// a moved cursor or a plane turned off gives a new display list.
    fn set_plane(
        &mut self,
        _objects: &KmsObjects<Self>,
        plane: &KmsPlane<Self>,
        new_plane_state: KmsPlaneState<Self>,
        _damage: Option<Damage>,
    ) -> syscall::Result<()> {
        let cursor = plane.driver_data.cursor;
        let shown = match (&new_plane_state.fb, new_plane_state.crtc_id) {
            (Some(fb), Some(_)) if cursor => {
                // Orbital gives the pointer's position; the image's top
                // left corner is the hotspot away from it.
                let (hot_x, hot_y) = new_plane_state.hotspot.unwrap_or((0, 0));
                let rect = &new_plane_state.crtc_rect;
                self.place(fb, rect.x - hot_x, rect.y - hot_y, true)
            }
            (Some(fb), Some(_)) => self.place(fb, 0, 0, false),
            _ => None,
        };
        let old = if cursor { &mut self.cursor } else { &mut self.primary };
        let changed = old.as_ref().map(|s| s.plane) != shown.as_ref().map(|s| s.plane);
        *old = shown;
        if changed {
            self.update();
        }
        *plane.state.lock().unwrap() = new_plane_state;
        Ok(())
    }
}

/// A dumb buffer: in DMA memory (physically contiguous, so the HVS can
/// read it), else in normal memory, which cannot be shown.
pub struct Vc4Buffer {
    width: usize,
    height: usize,
    ptr: NonNull<[u32]>,
    dma: Option<Dma<[u32]>>,
}

// The buffer's memory stays mapped as long as it lives; the watch thread
// only reads it for screenshots.
unsafe impl Send for Vc4Buffer {}
unsafe impl Sync for Vc4Buffer {}

impl fmt::Debug for Vc4Buffer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Vc4Buffer")
            .field("width", &self.width)
            .field("height", &self.height)
            .field("dma", &self.dma.is_some())
            .finish()
    }
}

impl Vc4Buffer {
    fn new(width: usize, height: usize) -> Vc4Buffer {
        let len = width * height;
        match Dma::<[u32]>::zeroed_slice(len) {
            Ok(dma) => {
                let mut dma = unsafe { dma.assume_init() };
                let ptr = NonNull::from(&mut *dma);
                report_contiguous(width, height, dma.physical(), ptr.as_ptr() as *mut u32 as usize, len * 4);
                Vc4Buffer { width, height, ptr, dma: Some(dma) }
            }
            Err(err) => {
                println!("vc4d: dumb buffer {width}x{height} in normal memory: {err}");
                let layout = Self::layout(len);
                let ptr = unsafe { alloc::alloc_zeroed(layout) };
                let ptr = ptr::slice_from_raw_parts_mut(ptr.cast(), len);
                let ptr = NonNull::new(ptr).unwrap_or_else(|| alloc::handle_alloc_error(layout));
                Vc4Buffer { width, height, ptr, dma: None }
            }
        }
    }

    fn layout(len: usize) -> Layout {
        Layout::array::<u32>(len).unwrap().align_to(PAGE_SIZE).unwrap()
    }
}

/// Logs where a buffer is and whether every page really follows the one
/// before it in physical memory, as the HVS reads it (Pi test #275).
fn report_contiguous(width: usize, height: usize, phys: usize, virt: usize, size: usize) {
    let gap = common::VirtaddrTranslationHandle::new().ok().and_then(|handle| {
        (1..size.div_ceil(PAGE_SIZE)).find(|i| handle.translate(virt + i * PAGE_SIZE).ok() != Some(phys + i * PAGE_SIZE))
    });
    match gap {
        None => println!("vc4d: buffer {width}x{height}: physical {phys:#x}-{:#x}, contiguous", phys + size),
        Some(page) => println!(
            "vc4d: buffer {width}x{height}: physical {phys:#x}, NOT CONTIGUOUS from page {page} (row {})",
            page * PAGE_SIZE / (width * 4).max(1)
        ),
    }
}

impl Drop for Vc4Buffer {
    fn drop(&mut self) {
        if self.dma.is_none() {
            let layout = Self::layout(self.ptr.len());
            unsafe { alloc::dealloc(self.ptr.as_ptr().cast(), layout) };
        }
    }
}

impl Buffer for Vc4Buffer {
    fn size(&self) -> usize {
        self.width * self.height * 4
    }
}
