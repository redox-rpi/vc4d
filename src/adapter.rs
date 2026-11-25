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
use driver_graphics::kms::plane::{KmsPlane, KmsPlaneDriver, KmsPlaneState, KmsPlaneType};
use driver_graphics::{Buffer, Damage, DumbBufferConfig, GraphicsAdapter};
use syscall::PAGE_SIZE;

use crate::hvs::{self, Channel, Hvs, Plane};
use crate::shot::{self, ShotPlane};

/// Two places for our display lists, far above the firmware's. A new list
/// goes into the one the HVS is not using, then the channel is pointed at
/// it, so the HVS never reads a half-written list.
/// Display list slots, used in turn. The HVS keeps reading a list (and
/// writes context words into it) until the frame after it was replaced, so
/// a slot is written again only two lists later, and at most one list is
/// written per frame (update): with two slots and several lists per frame,
/// vc4d overwrote the list the HVS was reading, and the rest of that frame
/// showed garbage (stripes on cursor moves, redox-rpi#563).
// 128 words each: five scaled planes take 16 words each.
const LIST_SLOTS: [usize; 3] = [0x800, 0x880, 0x900];

/// Orbital draws its pointer into a 64x64 ARGB buffer.
const CURSOR_SIZE: u32 = 64;
/// Overlay planes between the primary plane and the cursor, for windows
/// that a compositor shows by themselves (Orbital: the window being
/// dragged), so a move only changes the display list.
pub const OVERLAYS: usize = 3;

#[derive(Debug)]
pub struct Vc4Adapter {
    pub hvs: Hvs,
    pub channel: Channel,
    /// The mode clients see (/etc/vc4d-mode); the HVS scales it to the
    /// channel's size. Equal to the channel's size without scaling.
    pub screen: (u32, u32),
    /// Bus address minus physical address of RAM.
    pub ram_bus_offset: usize,
    /// The firmware's list (its framebuffer, which vesad draws into): shown
    /// while no client has a picture on the primary plane.
    pub firmware_list: usize,
    pub next_slot: usize,
    /// The frame count when the last list was written (None: none yet).
    pub list_frame: Option<u32>,
    /// The planes changed in a frame that already got a list: flush writes
    /// it in the next frame.
    pub list_pending: bool,
    pub primary: Option<Shown>,
    pub cursor: Option<Shown>,
    /// The overlay planes, bottom to top.
    pub overlays: Vec<Option<Shown>>,
    /// A client set no mode: the screen shows only the background.
    pub blanked: bool,
    /// Log every new display list, not only the first few.
    pub log_lists: bool,
    /// Display lists written so far.
    pub lists: u32,
    /// The planes shown now, for screenshots (shot.rs).
    pub shown: shot::Shown,
    /// Two scanout buffers for the primary plane (VC4D_FLIP, on unless
    /// `off`); None: the client's buffer is scanned out directly.
    pub flip: Option<Flip>,
    /// An HVS in memory (VC4D_FAKE): no frames to wait for.
    pub fake: bool,
}

/// A rectangle of the screen: x, y, width, height.
type Rect = (u32, u32, u32, u32);

fn union(a: Rect, b: Rect) -> Rect {
    if a.2 == 0 || a.3 == 0 {
        return b;
    }
    if b.2 == 0 || b.3 == 0 {
        return a;
    }
    let (x, y) = (a.0.min(b.0), a.1.min(b.1));
    let (right, bottom) = ((a.0 + a.2).max(b.0 + b.2), (a.1 + a.3).max(b.1 + b.3));
    (x, y, right - x, bottom - y)
}

fn clip(r: Rect, width: u32, height: u32) -> Rect {
    let x = r.0.min(width);
    let y = r.1.min(height);
    (x, y, r.2.min(width - x), r.3.min(height - y))
}

/// A client frame waiting to be copied: its buffer and what changed.
#[derive(Debug)]
struct Pending {
    src: Arc<Vc4Buffer>,
    pitch: u32,
    width: u32,
    height: u32,
    changed: Rect,
}

/// Double buffering for the primary plane. Clients (fbcond, Orbital) draw
/// into their own buffer in normal, cached memory and tell vc4d what
/// changed (dirty framebuffer). vc4d copies that part into the scanout
/// buffer the HVS is not reading, then points the display list at it; the
/// HVS takes the new list at the next frame. So the HVS never scans out a
/// half-copied picture (tearing), and nobody reads uncached memory.
///
/// Like the page flip of Linux's vc4 (vc4_crtc_page_flip()), but inside
/// the driver, because Redox's driver-graphics has no page flip ioctl and
/// Orbital copies into one buffer.
#[derive(Debug)]
pub struct Flip {
    bufs: [Arc<Vc4Buffer>; 2],
    /// The buffer in the display list now.
    front: usize,
    /// Which client buffer each scanout buffer last copied from (its
    /// address); a different one needs a full copy.
    synced: [usize; 2],
    /// What changed in the client's last frame: the buffer behind the
    /// front one lacks it.
    last: Rect,
    /// The frame count when the display list last changed.
    flip_frame: u32,
    /// Damage that came while the HVS still showed the list before: it is
    /// copied once the HVS took that list (flush), together with any newer
    /// damage. So a client never waits, and a burst of damage (console
    /// scrolling) becomes one copy per frame.
    pending: Option<Pending>,
    /// Copies done and bytes copied, for the log.
    pub copies: u64,
    pub bytes: u64,
}

impl Flip {
    /// Two buffers of the screen's size in contiguous memory, or None.
    pub fn new(width: u32, height: u32) -> Option<Flip> {
        let a = Vc4Buffer::new(width as usize, height as usize);
        let b = Vc4Buffer::new(width as usize, height as usize);
        if a.dma.is_none() || b.dma.is_none() {
            return None;
        }
        Some(Flip {
            bufs: [Arc::new(a), Arc::new(b)],
            front: 0,
            synced: [0, 0],
            last: (0, 0, 0, 0),
            flip_frame: 0,
            pending: None,
            copies: 0,
            bytes: 0,
        })
    }
}

/// The first display lists always go to the log.
const LOGGED_LISTS: u32 = 4;

/// A plane in the current display list, with its buffer, which must live
/// as long as the HVS reads it.
#[derive(Clone, Debug)]
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
        let frame = self.hvs.frame_count(index);
        if !self.fake && self.list_frame == Some(frame) {
            self.list_pending = true;
            return;
        }
        self.list_pending = false;
        self.list_frame = Some(frame);
        if let Some(flip) = &mut self.flip {
            flip.flip_frame = frame;
        }
        let Some(primary) = &self.primary else {
            println!("vc4d: no primary plane, firmware list again");
            self.hvs.set_list_start(index, self.firmware_list);
            self.shown.lock().unwrap().clear();
            return;
        };
        let planes: Vec<Plane> = if self.blanked {
            Vec::new()
        } else {
            std::iter::once(Some(primary))
                .chain(self.overlays.iter().map(Option::as_ref))
                .chain(std::iter::once(self.cursor.as_ref()))
                .flatten()
                .map(|shown| shown.plane)
                .collect()
        };
        *self.shown.lock().unwrap() = if self.blanked {
            Vec::new()
        } else {
            std::iter::once(Some(primary))
                .chain(self.overlays.iter().map(Option::as_ref))
                .chain(std::iter::once(self.cursor.as_ref()))
                .flatten()
                .map(|s| ShotPlane { plane: s.plane, pixels: s.pixels, _buffer: s._buffer.clone() })
                .collect()
        };
        let planes = self.to_output(planes);
        let list = self.hvs.list(&planes);
        let start = LIST_SLOTS[self.next_slot];
        self.hvs.write_dlist(start, &list);
        self.hvs.set_list_start(index, start);
        crate::trace::event("list", (start as u32, list.len() as u32, 0, 0), 0);
        self.next_slot = (self.next_slot + 1) % LIST_SLOTS.len();
        self.lists += 1;
        if self.log_lists || self.lists <= LOGGED_LISTS {
            println!("vc4d: list {} at {start}: {planes:?} {list:08x?}", self.lists);
        }
    }

    /// Planes in the clients' mode, scaled to the channel's size, each
    /// with its own line buffer.
    fn to_output(&self, planes: Vec<Plane>) -> Vec<Plane> {
        let (sw, sh) = self.screen;
        let (cw, ch) = (self.channel.width, self.channel.height);
        if (sw, sh) == (cw, ch) {
            return planes;
        }
        let mut lbm = 0;
        let generation = self.hvs.generation();
        planes
            .into_iter()
            .map(|mut p| {
                let sx = |v: u32| (u64::from(v) * u64::from(cw) / u64::from(sw)) as u32;
                let sy = |v: u32| (u64::from(v) * u64::from(ch) / u64::from(sh)) as u32;
                let (x, y) = (sx(p.x), sy(p.y));
                let width = (sx(p.x + p.width) - x).max(1);
                let height = (sy(p.y + p.height) - y).max(1);
                p.scale = Some(hvs::Scale { width, height, lbm });
                lbm += hvs::lbm_words(generation, p.width, p.height, width, height)
                    .next_multiple_of(generation.lbm_align());
                p.x = x;
                p.y = y;
                p
            })
            .collect()
    }

    /// Takes the client's frame for the primary plane. If the HVS already
    /// took the last display list, the frame is copied into the scanout
    /// buffer behind the shown one, which is returned as the new primary
    /// plane. Otherwise the frame waits in `pending` (flush copies it) and
    /// the shown plane stays.
    fn flip_primary(&mut self, fb: &KmsFramebuffer<Self>, damage: Option<Damage>) -> Option<Shown> {
        let (screen_w, screen_h) = self.screen;
        let (width, height) = (fb.width.min(screen_w), fb.height.min(screen_h));
        let full = (0, 0, width, height);
        let changed = damage.map_or(full, |d| clip((d.x, d.y, d.width, d.height), width, height));
        let flip = self.flip.as_mut()?;
        let changed = match &flip.pending {
            Some(p) if Arc::ptr_eq(&p.src, &fb.buffer) => union(p.changed, changed),
            Some(_) => full,
            None => changed,
        };
        flip.pending = Some(Pending { src: fb.buffer.clone(), pitch: fb.pitch, width, height, changed });
        let flip_frame = flip.flip_frame;
        if !self.fake
            && self.primary.is_some()
            && (self.list_pending || self.hvs.frame_count(self.channel.index) == flip_frame)
        {
            crate::trace::event("defer", changed, 0);
            return self.primary.clone();
        }
        self.copy_pending()
    }

    /// Copies the pending frame into the scanout buffer behind the shown
    /// one and returns that buffer as the primary plane.
    fn copy_pending(&mut self) -> Option<Shown> {
        let screen_w = self.screen.0;
        let flip = self.flip.as_mut()?;
        let p = flip.pending.take()?;
        let back = 1 - flip.front;
        let full = (0, 0, p.width, p.height);
        let src = p.src.ptr.as_ptr().cast::<u8>();
        let region = if flip.synced[back] != src as usize { full } else { union(p.changed, flip.last) };
        let dst_buf = flip.bufs[back].clone();
        let dst = dst_buf.ptr.as_ptr().cast::<u8>();
        let dst_pitch = screen_w as usize * 4;
        let (x, y, w, h) = clip(region, p.width, p.height);
        let copy_start = std::time::Instant::now();
        for row in y..y + h {
            unsafe {
                ptr::copy_nonoverlapping(
                    src.add(row as usize * p.pitch as usize + x as usize * 4),
                    dst.add(row as usize * dst_pitch + x as usize * 4),
                    w as usize * 4,
                );
            }
        }
        crate::trace::event("copy", (x, y, w, h), copy_start.elapsed().as_micros() as u64);
        flip.synced[back] = src as usize;
        flip.last = p.changed;
        flip.front = back;
        flip.copies += 1;
        flip.bytes += u64::from(w) * u64::from(h) * 4;
        let mem = dst_buf.dma.as_ref()?;
        Some(Shown {
            plane: Plane {
                bus: (mem.physical() + self.ram_bus_offset) as u32,
                pitch: dst_pitch as u32,
                x: 0,
                y: 0,
                width: p.width,
                height: p.height,
                alpha: false,
                scale: None,
            },
            pixels: dst,
            _buffer: dst_buf,
        })
    }

    /// Whether a frame or a display list waits for the next frame.
    pub fn flip_pending(&self) -> bool {
        self.list_pending || self.flip.as_ref().is_some_and(|f| f.pending.is_some())
    }

    /// Copies and shows a waiting frame once the HVS took the last list;
    /// called from the event loop's timer while flip_pending().
    pub fn flush(&mut self) {
        if self.list_pending {
            if self.list_frame == Some(self.hvs.frame_count(self.channel.index)) {
                return;
            }
            self.update();
            return;
        }
        let Some(flip) = &self.flip else { return };
        if flip.pending.is_none() || self.hvs.frame_count(self.channel.index) == flip.flip_frame {
            return;
        }
        if let Some(shown) = self.copy_pending() {
            self.primary = Some(shown);
            self.flipped();
        }
    }

    /// After a new primary plane: the new display list, and the frame it
    /// was written in.
    fn flipped(&mut self) {
        self.update();
        if let Some(flip) = &mut self.flip {
            if self.log_lists && flip.copies % 600 == 1 {
                println!("vc4d: flip: {} copies, {} MB", flip.copies, flip.bytes >> 20);
            }
        }
    }

    /// The part of the framebuffer that is on screen, with the top left
    /// corner at (x, y) on the screen; None if nothing of it is.
    fn place(&self, fb: &KmsFramebuffer<Self>, x: i32, y: i32, alpha: bool) -> Option<Shown> {
        let Some(mem) = &fb.buffer.dma else {
            println!("vc4d: buffer {}x{} not in contiguous memory, not shown", fb.width, fb.height);
            return None;
        };
        let (screen_w, screen_h) = (self.screen.0 as i32, self.screen.1 as i32);
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
                scale: None,
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
    /// The overlay plane's index, None for the primary and cursor planes.
    overlay: Option<usize>,
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
            Vc4Plane { cursor: false, overlay: None },
            (),
            Some((Vc4Plane { cursor: true, overlay: None }, ())),
        );
        for index in 0..OVERLAYS {
            objects.add_plane(&[crtc], KmsPlaneType::Overlay, false, Vc4Plane { cursor: false, overlay: Some(index) }, ());
        }
        self.overlays = vec![None; OVERLAYS];
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
            // With flip the clients' buffers are normal memory.
            prefer_shadow: self.flip.is_none(),
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
        connector.modes = vec![modeinfo_for_size(self.screen.0, self.screen.1)];
        Ok(())
    }

    fn create_dumb_buffer(&mut self, width: u32, height: u32) -> (Self::Buffer, u32) {
        // With flip a screen-sized buffer (the console's, Orbital's
        // framebuffer) is copied from, so it lives in cached memory; smaller
        // ones (the cursor, overlay windows) are scanned out directly.
        if self.flip.is_some() && width >= self.screen.0 && height >= self.screen.1 {
            return (Vc4Buffer::normal(width as usize, height as usize), width * 4);
        }
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

    /// Without flip, damage needs nothing: the HVS shows the buffer
    /// itself. With flip, damage on the primary plane is copied and shown
    /// (flip_primary). A new buffer, a moved cursor or a plane turned off
    /// gives a new display list.
    fn set_plane(
        &mut self,
        _objects: &KmsObjects<Self>,
        plane: &KmsPlane<Self>,
        new_plane_state: KmsPlaneState<Self>,
        damage: Option<Damage>,
    ) -> syscall::Result<()> {
        let cursor = plane.driver_data.cursor;
        let overlay = plane.driver_data.overlay;
        if let Some(index) = overlay {
            let rect = &new_plane_state.crtc_rect;
            crate::trace::event("overlay", (rect.x.max(0) as u32, rect.y.max(0) as u32, index as u32, new_plane_state.fb.is_some() as u32), 0);
        } else if cursor {
            let rect = &new_plane_state.crtc_rect;
            crate::trace::event("cursor", (rect.x.max(0) as u32, rect.y.max(0) as u32, 0, 0), 0);
        } else if let Some(d) = damage {
            crate::trace::event("damage", (d.x, d.y, d.width, d.height), 0);
        } else {
            crate::trace::event("damage", (0, 0, 0, 0), 0);
        }
        let shown = match (&new_plane_state.fb, new_plane_state.crtc_id) {
            (Some(fb), Some(_)) if cursor => {
                // Orbital gives the pointer's position; the image's top
                // left corner is the hotspot away from it.
                let (hot_x, hot_y) = new_plane_state.hotspot.unwrap_or((0, 0));
                let rect = &new_plane_state.crtc_rect;
                self.place(fb, rect.x - hot_x, rect.y - hot_y, true)
            }
            (Some(fb), Some(_)) if overlay.is_some() => {
                let rect = &new_plane_state.crtc_rect;
                let shown = self.place(fb, rect.x, rect.y, true);
                if self.log_lists {
                    println!(
                        "vc4d: overlay {}: at {},{} fb {}x{} dma {} -> {}",
                        overlay.unwrap_or(0), rect.x, rect.y, fb.width, fb.height,
                        fb.buffer.dma.is_some(), if shown.is_some() { "shown" } else { "refused" }
                    );
                }
                match shown {
                    Some(shown) => Some(shown),
                    // Not in contiguous memory (or off screen): the client
                    // has to draw the window itself.
                    None => return Err(syscall::Error::new(syscall::EINVAL)),
                }
            }
            (Some(fb), Some(_)) if self.flip.is_some() => self.flip_primary(fb, damage),
            (Some(fb), Some(_)) => self.place(fb, 0, 0, false),
            _ => None,
        };
        let old = match overlay {
            Some(index) => &mut self.overlays[index],
            None if cursor => &mut self.cursor,
            None => &mut self.primary,
        };
        let changed = old.as_ref().map(|s| s.plane) != shown.as_ref().map(|s| s.plane);
        let hidden = shown.is_none();
        *old = shown;
        // A compositor hides an overlay after drawing its window into the
        // primary plane; with that primary frame still waiting, the overlay
        // goes with the frame's list, else the window would vanish for a
        // frame.
        if changed && !(overlay.is_some() && hidden && self.flip_pending()) {
            self.flipped();
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
        match Dma::<[u32]>::zeroed_slice(width * height) {
            Ok(dma) => {
                let mut dma = unsafe { dma.assume_init() };
                let ptr = NonNull::from(&mut *dma);
                Vc4Buffer { width, height, ptr, dma: Some(dma) }
            }
            Err(err) => {
                println!("vc4d: dumb buffer {width}x{height} in normal memory: {err}");
                Self::normal(width, height)
            }
        }
    }

    /// A buffer in normal (cached) memory, which the HVS cannot show.
    fn normal(width: usize, height: usize) -> Vc4Buffer {
        let len = width * height;
        let layout = Self::layout(len);
        let ptr = unsafe { alloc::alloc_zeroed(layout) };
        // Back every page now: the buffer can be shared (a PRIME fd for the
        // V3D, a client's mapping), and pages nobody touched yet could end
        // up as different frames in different mappings (G4, pi-tests Test 642).
        if !ptr.is_null() {
            for page in (0..layout.size()).step_by(PAGE_SIZE) {
                unsafe { ptr.add(page).write_volatile(0) };
            }
        }
        let ptr = ptr::slice_from_raw_parts_mut(ptr.cast(), len);
        let ptr = NonNull::new(ptr).unwrap_or_else(|| alloc::handle_alloc_error(layout));
        Vc4Buffer { width, height, ptr, dma: None }
    }

    fn layout(len: usize) -> Layout {
        Layout::array::<u32>(len).unwrap().align_to(PAGE_SIZE).unwrap()
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
