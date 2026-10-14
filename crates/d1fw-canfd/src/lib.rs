//! Variable-length CANFD frames and the synchronous bus seam the dexterous
//! hands speak.
//!
//! The Damiao gripper's motors use classic 8-byte CAN frames, which
//! [`d1fw_proto_dmcan::CanFrame`] models as a fixed `[u8; 8]`. Both dexterous
//! hands need more than that: one LinkerHand O30 joint-vector write is a
//! 28-byte payload, and a Leadshine DH116S feedback frame is 62 bytes. So
//! they get their own frame type rather than a widened gripper one — the
//! gripper's fixed-width frame is load-bearing in its state machine, and
//! nothing about this port is allowed to change gripper behaviour.
//!
//! This crate performs no I/O. [`CanFdBus`] is the transport seam that the
//! daemon's arm end-module passthrough, the flange relay's provider side and
//! the simulators implement; [`FrameKind`] is what a driver says each frame
//! does, for a transport that gates on it.
//!
//! [`d1fw_proto_dmcan::CanFrame`]: https://docs.rs/d1fw-proto-dmcan

use std::time::Duration;

use thiserror::Error;

/// Legal CANFD payload lengths, indexed by DLC (0-15).
///
/// A CANFD frame's data field is not an arbitrary length: past eight bytes it
/// steps 12, 16, 20, 24, 32, 48, 64. A payload that is not one of these is
/// zero-padded up to the next legal one, exactly as the vendor SDKs and the
/// Python drivers do (`transport.py`'s `fd_padded_length`).
pub const DLC_LENGTHS: [usize; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];

/// The largest CANFD payload.
pub const MAX_PAYLOAD: usize = 64;

/// Round `length` up to the next legal CANFD payload length.
///
/// # Errors
///
/// [`CanFdError::Oversize`] when `length` exceeds [`MAX_PAYLOAD`].
pub fn fd_padded_length(length: usize) -> Result<usize, CanFdError> {
    DLC_LENGTHS
        .into_iter()
        .find(|legal| *legal >= length)
        .ok_or(CanFdError::Oversize { length })
}

/// One CANFD frame reduced to what the hand protocols need.
///
/// The identifier is an 11-bit standard id in both protocols (HOP's
/// `frame_id`, the DH116S's `0x500 + node`), so it is kept as a `u16` rather
/// than modelling the 29-bit extended form neither hand is driven in.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CanFdFrame {
    /// The standard CAN identifier.
    pub id: u16,
    /// The frame's payload, at most [`MAX_PAYLOAD`] bytes.
    pub data: Vec<u8>,
}

impl CanFdFrame {
    /// Build a frame from an identifier and payload.
    #[must_use]
    pub fn new<D: Into<Vec<u8>>>(id: u16, data: D) -> Self {
        Self {
            id,
            data: data.into(),
        }
    }

    /// Return this frame with its payload zero-padded to a legal CANFD length.
    ///
    /// # Errors
    ///
    /// [`CanFdError::Oversize`] when the payload exceeds [`MAX_PAYLOAD`].
    pub fn padded(&self) -> Result<Self, CanFdError> {
        let target = fd_padded_length(self.data.len())?;
        let mut data = self.data.clone();
        data.resize(target, 0);
        Ok(Self { id: self.id, data })
    }

    /// Render as `0xNNN [len] aa bb ...`, the Python drivers' trace spelling.
    #[must_use]
    pub fn hex(&self) -> String {
        let bytes = self
            .data
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect::<Vec<_>>()
            .join(" ");
        format!("0x{:03X} [{}] {bytes}", self.id, self.data.len())
    }
}

/// An error reported by a [`CanFdBus`] implementation or by frame validation.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum CanFdError {
    /// The bus has been closed and can no longer transfer frames.
    #[error("CANFD bus is closed")]
    Closed,
    /// The underlying transport failed.
    #[error("CANFD bus transport error: {0}")]
    Transport(String),
    /// A gating transport refused the frame (the flange relay: the sender is
    /// not admitted, the soft-kill latch stands, the rate ceiling is
    /// reached); the frame was not sent.
    #[error("CANFD frame refused: {0}")]
    Refused(String),
    /// The payload does not fit one CANFD frame.
    #[error("CANFD payload of {length} bytes exceeds the 64-byte frame limit")]
    Oversize {
        /// The rejected payload length.
        length: usize,
    },
}

/// What a frame does to the device it is sent to.
///
/// A transport that gates frames (the daemon's flange relay, which refuses a
/// frame that can move the device while the soft-kill latch stands) needs
/// the driver's word for it; a plain transport ignores it.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub enum FrameKind {
    /// It can move the device: a target, an enable, a mode or parameter
    /// write.
    Motion,
    /// It reads and commands nothing: a poll, a register read.
    Read,
    /// It takes the actuators' authority away and never moves the device: a
    /// disable.
    Stop,
}

/// Synchronous CANFD transport seam used by the dexterous-hand drivers.
///
/// A receive timeout is not an error: implementations return `Ok(None)` when
/// no frame arrives before `timeout`. [`CanFdBus::close`] must be idempotent.
pub trait CanFdBus: Send + Sync + 'static {
    /// Send one frame, zero-padding it to a legal CANFD length.
    ///
    /// # Errors
    ///
    /// Whatever the transport reports, or [`CanFdError::Closed`].
    fn send(&self, frame: &CanFdFrame) -> Result<(), CanFdError>;

    /// Send one frame that does `kind` to the device. A transport that does
    /// not gate frames sends it as [`CanFdBus::send`] does.
    ///
    /// # Errors
    ///
    /// Whatever [`CanFdBus::send`] reports, and a gating transport's refusal.
    fn send_as(&self, frame: &CanFdFrame, kind: FrameKind) -> Result<(), CanFdError> {
        let _ = kind;
        self.send(frame)
    }

    /// Wait until every frame sent through this bus has left this process
    /// for the channel: a relaying transport returns once its relay has
    /// taken them. A transport whose `send` writes the channel itself has
    /// nothing to wait for (the default).
    ///
    /// # Errors
    ///
    /// The relay did not take the frames in time, or the transport failed.
    fn flush(&self) -> Result<(), CanFdError> {
        Ok(())
    }

    /// Receive one frame, or `None` when `timeout` expires.
    ///
    /// # Errors
    ///
    /// Whatever the transport reports, or [`CanFdError::Closed`].
    fn recv(&self, timeout: Duration) -> Result<Option<CanFdFrame>, CanFdError>;

    /// Close the bus; repeated calls have no additional effect.
    fn close(&self);

    /// Whether this transport only hands over a received frame whose bytes
    /// DIFFER from the one it handed over last.
    ///
    /// Only the D1 arm's end module does this, and it is not a curiosity: an
    /// idle hand asked the same question twice answers identically, the
    /// second answer is never delivered, and the read times out on a hand
    /// that is working perfectly. Measured on arm A, 2026-08-13 (the Python
    /// `ArmPassthroughBus.suppresses_repeat_frames` comment records the
    /// experiment). The HOP driver compensates by varying the length it asks
    /// for, which is a no-op on every other bus — hence the default.
    fn suppresses_repeat_frames(&self) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::{CanFdError, CanFdFrame, fd_padded_length};

    #[test]
    fn padding_climbs_the_legal_dlc_ladder() {
        assert_eq!(fd_padded_length(0), Ok(0));
        assert_eq!(fd_padded_length(8), Ok(8));
        assert_eq!(fd_padded_length(9), Ok(12));
        // A 25-byte O30 joint image plus its 3-byte header is 28 bytes, and
        // 28 is NOT a legal CANFD length: it goes out as a 32-byte frame with
        // four bytes of padding the hand ignores, because EDL — not the frame
        // length — is what says how much of the payload counts.
        assert_eq!(fd_padded_length(24), Ok(24));
        assert_eq!(fd_padded_length(28), Ok(32));
        assert_eq!(fd_padded_length(64), Ok(64));
        assert_eq!(
            fd_padded_length(65),
            Err(CanFdError::Oversize { length: 65 })
        );
    }

    #[test]
    fn padded_extends_with_zeros_and_keeps_the_identifier() {
        let frame = CanFdFrame::new(0x001, vec![0x41, 0x39, 0x05, 0xFF]);
        let padded = frame.padded().expect("four bytes fit a CANFD frame");
        assert_eq!(padded.id, 0x001);
        assert_eq!(padded.data, vec![0x41, 0x39, 0x05, 0xFF]);

        let wide = CanFdFrame::new(0x001, vec![0xAA; 9]);
        let padded = wide.padded().expect("nine bytes fit a CANFD frame");
        assert_eq!(padded.data.len(), 12);
        assert_eq!(&padded.data[9..], &[0, 0, 0]);
    }

    #[test]
    fn hex_reads_like_the_python_driver_traces() {
        let frame = CanFdFrame::new(0x401, vec![0x41, 0x39, 0x05]);
        assert_eq!(frame.hex(), "0x401 [3] 41 39 05");
    }
}
