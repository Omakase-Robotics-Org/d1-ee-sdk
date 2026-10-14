//! The fixed-size messages of `flange_bus.shm/1`.
//!
//! The conventions are `end_effector.shm/1`'s: `#[repr(C)]`, little-endian,
//! explicit `reserved` fields and no implicit padding, every message
//! starting with the shared [`Header`], text NUL-padded UTF-8, and type
//! names carrying the contract's major (`omakase_flange_v1_*`).

// The derive emits an `unsafe impl ZeroCopySend`: the promise that the type
// is self-contained plain data with a stable layout, which every message
// here is by construction (`#[repr(C)]`, integers and arrays of them only).
#![allow(unsafe_code)]

use iceoryx2::prelude::ZeroCopySend;

pub use crate::wire::Header;

use super::{ERROR_LEN, MAX_FRAME_LEN};

/// One CAN or CAN FD frame.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_flange_v1_Frame")]
pub struct Frame {
    /// The CAN identifier: 11 bits, or 29 with [`FRAME_EXTENDED_ID`].
    pub id: u32,
    /// `FRAME_*` bits.
    pub flags: u32,
    /// How many bytes of `data` are the frame's payload, `0` to
    /// [`MAX_FRAME_LEN`].
    pub len: u32,
    /// Zero.
    pub reserved: u32,
    /// The payload; bytes past `len` are zero.
    pub data: [u8; MAX_FRAME_LEN],
}

/// A frame the daemon took off the channel, on `<base>/rx`.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_flange_v1_RxFrame")]
pub struct RxFrame {
    /// The common header; `instance_id` is the daemon's.
    pub header: Header,
    /// `CLOCK_MONOTONIC` nanoseconds when the daemon took the frame off the
    /// channel.
    pub channel_mono_ns: u64,
    /// The frame.
    pub frame: Frame,
}

/// A frame the provider asks the daemon to put on the channel, on
/// `<base>/tx`.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_flange_v1_TxFrame")]
pub struct TxFrame {
    /// The common header; `instance_id` is the provider's.
    pub header: Header,
    /// `TX_*`: what the frame does to the device.
    pub kind: u32,
    /// Zero.
    pub reserved: u32,
    /// The frame.
    pub frame: Frame,
}

/// The provider's liveness, on `<base>/heartbeat`.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_flange_v1_Heartbeat")]
pub struct Heartbeat {
    /// The common header; `instance_id` is the provider's.
    pub header: Header,
    /// How often the provider publishes this, milliseconds.
    pub period_ms: u32,
    /// Zero.
    pub reserved: u32,
}

/// What the relay is doing, on `<base>/status` (daemon to provider),
/// latched.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_flange_v1_Status")]
pub struct Status {
    /// The common header; `instance_id` is the daemon's.
    pub header: Header,
    /// The provider instance whose tx frames the relay takes: the sender of
    /// the newest fresh heartbeat; `0` for none.
    pub bound_instance_id: u64,
    /// The header `seq` of the newest tx frame of `bound_instance_id` the
    /// relay has taken (put on the channel or refused); `0` for none.
    pub tx_seq_seen: u64,
    /// Frames taken off the channel and published on `rx`.
    pub rx_frames: u64,
    /// Tx frames put on the channel.
    pub tx_forwarded: u64,
    /// Tx frames of kind [`TX_MOTION`] refused while the soft-kill latch
    /// stood.
    pub tx_refused_latched: u64,
    /// Tx frames refused by the rate ceiling.
    pub tx_refused_rate: u64,
    /// Tx frames refused as malformed or unsupported by the channel.
    pub tx_refused_invalid: u64,
    /// Tx frames refused because their sender was not the bound instance.
    pub tx_refused_unbound: u64,
    /// Tx frames refused as older than [`super::TX_MAX_AGE_MS`].
    pub tx_refused_stale: u64,
    /// Tx frames of the bound instance that never reached the relay (gaps
    /// in their header `seq`).
    pub tx_lost: u64,
    /// Tx frames the channel failed to send.
    pub tx_failed: u64,
    /// `STATUS_*` bits.
    pub flags: u32,
    /// `CHANNEL_*` bits: what the channel is.
    pub channel_flags: u32,
    /// The tx rate ceiling, frames per second.
    pub max_tx_hz: u32,
    /// The longest frame payload the channel carries, bytes.
    pub max_frame_len: u32,
    /// How often the daemon publishes this when nothing changes,
    /// milliseconds.
    pub period_ms: u32,
    /// Zero.
    pub reserved: u32,
    /// Why the channel is down; empty while it is up.
    pub channel_error: [u8; ERROR_LEN],
}

/// [`Frame::flags`]: the identifier is 29 bits.
pub const FRAME_EXTENDED_ID: u32 = 1 << 0;
/// Every frame bit schema 1 defines.
pub const FRAME_ALL: u32 = FRAME_EXTENDED_ID;

/// [`TxFrame::kind`]: the frame can move the device (a target, an enable, a
/// mode change). Refused while the soft-kill latch stands. The zero value,
/// so a field left unset is the most restricted kind.
pub const TX_MOTION: u32 = 0;
/// The frame reads and commands nothing (a poll, a register read). Passes
/// the latch.
pub const TX_READ: u32 = 1;
/// The frame takes the actuators' authority away and never moves the
/// device (a disable). Passes the latch and is never refused by the rate
/// ceiling.
pub const TX_STOP: u32 = 2;

/// [`Status::flags`]: the channel's newest operation (a send or a receive)
/// succeeded.
pub const STATUS_CHANNEL_UP: u32 = 1 << 0;
/// The end effectors' soft-kill latch stands (or the daemon has not wired
/// it yet): frames of kind [`TX_MOTION`] are refused.
pub const STATUS_LATCHED: u32 = 1 << 1;
/// [`Status::flags`]: the arm controller that carries this channel reports a
/// fault on the channel's arm side (a nonzero error code or its error mode).
/// While it does, the controller may stop delivering frames the provider
/// sends, while frames it receives still arrive.
pub const STATUS_ARM_FAULTED: u32 = 1 << 2;
/// Every status bit schema 1 defines.
pub const STATUS_ALL: u32 = STATUS_CHANNEL_UP | STATUS_LATCHED | STATUS_ARM_FAULTED;

/// [`Status::channel_flags`]: the channel hands over a received frame only
/// when its bytes differ from the frame it handed over last (the D1 arm's
/// end module). A provider whose device answers a repeated question
/// identically must vary its questions.
pub const CHANNEL_SUPPRESSES_REPEATS: u32 = 1 << 0;
/// The channel carries 29-bit identifiers; without it a frame with
/// [`FRAME_EXTENDED_ID`] is refused.
pub const CHANNEL_EXTENDED_IDS: u32 = 1 << 1;
/// Every channel bit schema 1 defines.
pub const CHANNEL_ALL: u32 = CHANNEL_SUPPRESSES_REPEATS | CHANNEL_EXTENDED_IDS;

macro_rules! zeroed {
    ($($ty:ident),* $(,)?) => {
        $(
            impl $ty {
                /// Every field zero.
                #[must_use]
                pub fn zeroed() -> Self {
                    // SAFETY: every field is an integer or an array of
                    // integers (the header too), for which the all-zero bit
                    // pattern is valid.
                    unsafe { core::mem::zeroed() }
                }
            }
        )*
    };
}

zeroed!(Frame, RxFrame, TxFrame, Heartbeat, Status);
