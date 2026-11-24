// SPDX-License-Identifier: GPL-2.0-only
//
// The binary interface of v3dd (G2 step 3, redox-rpi#668): the V3D ioctls
// of Linux (include/uapi/drm/v3d_drm.h, v6.12), as Redox carries DRM
// ioctls. relibc turns ioctl(fd, DRM_IOCTL_V3D_..., &arg) into a scheme call
// with the DRM function number (DRM_COMMAND_BASE 0x40 + the V3D number) in
// metadata[0] and the argument as payload, written as redox-ioctl's
// define_ioctl_data! does: the struct without its pointer fields, then the
// arrays those pointers point to (here the BO handles of a submit). The
// structs below are that wire form. relibc lists the V3D numbers with
// redox-rpi's experiment v3d-ioctl (G3, redox-rpi#669).
//
// Mesa's v3d driver also needs DRM sync objects (G3 step 2): v3dd keeps
// them per file, next to the BOs.

pub const GEM_CLOSE: u64 = 0x09;
pub const SUBMIT_CL: u64 = 0x40;
pub const WAIT_BO: u64 = 0x41;
pub const CREATE_BO: u64 = 0x42;
pub const MMAP_BO: u64 = 0x43;
pub const GET_PARAM: u64 = 0x44;
pub const GET_BO_OFFSET: u64 = 0x45;
pub const SUBMIT_CSD: u64 = 0x47;
pub const SYNCOBJ_CREATE: u64 = 0xBF;
pub const SYNCOBJ_DESTROY: u64 = 0xC0;
pub const SYNCOBJ_WAIT: u64 = 0xC3;
pub const SYNCOBJ_RESET: u64 = 0xC4;
pub const SYNCOBJ_SIGNAL: u64 = 0xC5;

/// DRM_SYNCOBJ_CREATE_SIGNALED.
pub const SYNCOBJ_CREATE_SIGNALED: u32 = 1;
/// DRM_SYNCOBJ_WAIT_FLAGS_*.
pub const SYNCOBJ_WAIT_ALL: u32 = 1;
pub const SYNCOBJ_WAIT_FOR_SUBMIT: u32 = 2;
pub const SYNCOBJ_WAIT_AVAILABLE: u32 = 4;

/// DRM_V3D_SUBMIT_CL_FLUSH_CACHE: v3dd flushes the caches around every job.
pub const SUBMIT_CL_FLUSH_CACHE: u32 = 0x01;

/// enum drm_v3d_param.
pub const PARAM_UIFCFG: u32 = 0;
pub const PARAM_HUB_IDENT1: u32 = 1;
pub const PARAM_HUB_IDENT2: u32 = 2;
pub const PARAM_HUB_IDENT3: u32 = 3;
pub const PARAM_CORE0_IDENT0: u32 = 4;
pub const PARAM_CORE0_IDENT1: u32 = 5;
pub const PARAM_CORE0_IDENT2: u32 = 6;
pub const PARAM_SUPPORTS_TFU: u32 = 7;
pub const PARAM_SUPPORTS_CSD: u32 = 8;
pub const PARAM_SUPPORTS_SUPER_PAGES: u32 = 14;

/// struct drm_v3d_submit_cl without `bo_handles` (u64 pointer); then
/// `bo_handle_count` u32 handles.
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SubmitCl {
    pub bcl_start: u32,
    pub bcl_end: u32,
    pub rcl_start: u32,
    pub rcl_end: u32,
    pub in_sync_bcl: u32,
    pub in_sync_rcl: u32,
    pub out_sync: u32,
    pub qma: u32,
    pub qms: u32,
    pub qts: u32,
    pub bo_handle_count: u32,
    pub flags: u32,
    pub perfmon_id: u32,
    pub pad: u32,
    pub extensions: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct WaitBo {
    pub handle: u32,
    pub pad: u32,
    pub timeout_ns: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct CreateBo {
    pub size: u32,
    pub flags: u32,
    pub handle: u32,
    pub offset: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct MmapBo {
    pub handle: u32,
    pub flags: u32,
    pub offset: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct GetParam {
    pub param: u32,
    pub pad: u32,
    pub value: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct GetBoOffset {
    pub handle: u32,
    pub offset: u32,
}

/// struct drm_v3d_submit_csd without `bo_handles`; then the handles.
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SubmitCsd {
    pub cfg: [u32; 7],
    pub coef: [u32; 4],
    pub bo_handle_count: u32,
    pub in_sync: u32,
    pub out_sync: u32,
    pub perfmon_id: u32,
    pub extensions: u64,
    pub flags: u32,
    pub pad: u32,
}

/// struct drm_syncobj_create.
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SyncobjCreate {
    pub handle: u32,
    pub flags: u32,
}

/// struct drm_syncobj_destroy.
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SyncobjDestroy {
    pub handle: u32,
    pub pad: u32,
}

/// struct drm_syncobj_wait without `handles`; then the handles.
/// `timeout_nsec` is an absolute CLOCK_MONOTONIC time.
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SyncobjWait {
    pub timeout_nsec: i64,
    pub count_handles: u32,
    pub flags: u32,
    pub first_signaled: u32,
    pub pad: u32,
    pub deadline_nsec: u64,
}

/// Where `first_signaled` lies in the SyncobjWait payload.
pub const SYNCOBJ_WAIT_FIRST_SIGNALED: usize = 16;

/// struct drm_syncobj_array without `handles`; then the handles.
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct SyncobjArray {
    pub count_handles: u32,
    pub pad: u32,
}

/// struct drm_gem_close.
#[repr(C)]
#[derive(Clone, Copy, Default, Debug)]
pub struct GemClose {
    pub handle: u32,
    pub pad: u32,
}

/// The struct at the start of a payload, and the bytes after it.
pub fn read<T: Copy>(payload: &[u8]) -> Option<(T, &[u8])> {
    if payload.len() < size_of::<T>() {
        return None;
    }
    let value = unsafe { payload.as_ptr().cast::<T>().read_unaligned() };
    Some((value, &payload[size_of::<T>()..]))
}

/// Writes the struct back to the start of the payload.
pub fn write<T: Copy>(payload: &mut [u8], value: &T) {
    if payload.len() >= size_of::<T>() {
        unsafe { payload.as_mut_ptr().cast::<T>().write_unaligned(*value) };
    }
}

/// `count` u32 handles.
pub fn handles(bytes: &[u8], count: u32) -> Option<Vec<u32>> {
    let len = count as usize * 4;
    if bytes.len() != len {
        return None;
    }
    Some(bytes.chunks_exact(4).map(|c| u32::from_ne_bytes([c[0], c[1], c[2], c[3]])).collect())
}

/// The wire form of a struct followed by handles.
pub fn bytes<T: Copy>(value: &T, handles: &[u32]) -> Vec<u8> {
    let mut out = vec![0u8; size_of::<T>()];
    write(&mut out, value);
    for h in handles {
        out.extend_from_slice(&h.to_ne_bytes());
    }
    out
}
