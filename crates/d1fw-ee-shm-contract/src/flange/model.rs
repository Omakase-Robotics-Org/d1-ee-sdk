//! The owned, validated form of each `flange_bus.shm/1` message.
//!
//! Every decode checks the header (schema version 1 of this contract, a
//! sender), every flag word against the bits schema 1 defines, and every
//! count against its field; every encode refuses what the receiver would.

use super::{
    EXTENDED_ID_MAX, MAX_FRAME_LEN, SCHEMA_VERSION, STANDARD_ID_MAX,
    wire::{self, Header},
};
use crate::model::{ContractError, get_optional_text, put_text};

fn invalid(message: impl Into<String>) -> ContractError {
    ContractError::Invalid(message.into())
}

/// A header for a message of this contract, sent now by `instance_id` as
/// its `seq`-th on one service.
#[must_use]
pub fn header(instance_id: u64, seq: u64) -> Header {
    Header {
        schema_version: SCHEMA_VERSION,
        reserved: 0,
        instance_id,
        seq,
        mono_ns: crate::clock::monotonic_ns(),
    }
}

/// Refuse a header of another schema version or with no sender.
///
/// # Errors
///
/// [`ContractError::SchemaVersion`], or [`ContractError::Invalid`] for an
/// `instance_id` of zero.
pub fn check_header(header: &Header) -> Result<(), ContractError> {
    if header.schema_version != SCHEMA_VERSION {
        return Err(ContractError::SchemaVersion {
            got: header.schema_version,
        });
    }
    if header.instance_id == 0 {
        return Err(invalid("instance_id 0 names no sender"));
    }
    Ok(())
}

/// One CAN or CAN FD frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// The identifier: at most [`STANDARD_ID_MAX`], or [`EXTENDED_ID_MAX`]
    /// when `extended`.
    pub id: u32,
    /// Whether the identifier is 29 bits.
    pub extended: bool,
    /// The payload, at most [`MAX_FRAME_LEN`] bytes.
    pub data: Vec<u8>,
}

impl Frame {
    /// A standard-identifier frame.
    #[must_use]
    pub fn standard(id: u16, data: impl Into<Vec<u8>>) -> Self {
        Self {
            id: u32::from(id),
            extended: false,
            data: data.into(),
        }
    }

    /// Refuse a frame no channel carries.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for a payload over [`MAX_FRAME_LEN`] or an
    /// identifier wider than its kind.
    pub fn validate(&self) -> Result<(), ContractError> {
        if self.data.len() > MAX_FRAME_LEN {
            return Err(invalid(format!(
                "a frame of {} bytes; a frame holds at most {MAX_FRAME_LEN}",
                self.data.len()
            )));
        }
        let max = if self.extended {
            EXTENDED_ID_MAX
        } else {
            STANDARD_ID_MAX
        };
        if self.id > max {
            return Err(invalid(format!(
                "identifier {:#x} is wider than a {} identifier",
                self.id,
                if self.extended {
                    "29-bit"
                } else {
                    "11-bit (standard)"
                }
            )));
        }
        Ok(())
    }

    /// The wire form.
    ///
    /// # Errors
    ///
    /// Whatever [`Self::validate`] refuses.
    pub fn to_wire(&self) -> Result<wire::Frame, ContractError> {
        self.validate()?;
        let mut out = wire::Frame::zeroed();
        out.id = self.id;
        out.flags = if self.extended {
            wire::FRAME_EXTENDED_ID
        } else {
            0
        };
        out.len = u32::try_from(self.data.len()).unwrap_or(u32::MAX);
        out.data[..self.data.len()].copy_from_slice(&self.data);
        Ok(out)
    }

    /// Read a wire frame.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for an undefined flag, a length over the
    /// field, or an identifier wider than its kind.
    pub fn from_wire(frame: &wire::Frame) -> Result<Self, ContractError> {
        if frame.flags & !wire::FRAME_ALL != 0 {
            return Err(invalid(format!(
                "frame flags set bits {:#x} that schema {SCHEMA_VERSION} does not define",
                frame.flags & !wire::FRAME_ALL
            )));
        }
        let len = usize::try_from(frame.len).unwrap_or(usize::MAX);
        if len > MAX_FRAME_LEN {
            return Err(invalid(format!(
                "frame len {len} exceeds the contract's {MAX_FRAME_LEN}"
            )));
        }
        let decoded = Self {
            id: frame.id,
            extended: frame.flags & wire::FRAME_EXTENDED_ID != 0,
            data: frame.data[..len].to_vec(),
        };
        decoded.validate()?;
        Ok(decoded)
    }
}

/// What a tx frame does to the device; the relay gates on it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TxKind {
    /// It can move the device; refused while the soft-kill latch stands.
    Motion,
    /// It reads and commands nothing; passes the latch.
    Read,
    /// It takes the actuators' authority away and never moves the device;
    /// passes the latch and the rate ceiling.
    Stop,
}

impl TxKind {
    /// The wire code.
    #[must_use]
    pub const fn code(self) -> u32 {
        match self {
            Self::Motion => wire::TX_MOTION,
            Self::Read => wire::TX_READ,
            Self::Stop => wire::TX_STOP,
        }
    }

    /// The kind a wire code names.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for a code schema 1 does not define.
    pub fn from_code(code: u32) -> Result<Self, ContractError> {
        match code {
            wire::TX_MOTION => Ok(Self::Motion),
            wire::TX_READ => Ok(Self::Read),
            wire::TX_STOP => Ok(Self::Stop),
            other => Err(invalid(format!("tx kind {other} is not a kind"))),
        }
    }
}

/// A received frame, as a provider reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RxFrame {
    /// The daemon instance that relayed it.
    pub daemon_instance_id: u64,
    /// Its place in the daemon's `rx` stream: a gap means frames lost.
    pub seq: u64,
    /// `CLOCK_MONOTONIC` nanoseconds when the daemon took it off the
    /// channel.
    pub channel_mono_ns: u64,
    /// The frame.
    pub frame: Frame,
}

impl RxFrame {
    /// The wire form of `frame`, taken off the channel at
    /// `channel_mono_ns`.
    ///
    /// # Errors
    ///
    /// Whatever [`Frame::validate`] refuses.
    pub fn to_wire(
        frame: &Frame,
        channel_mono_ns: u64,
        header: Header,
    ) -> Result<wire::RxFrame, ContractError> {
        Ok(wire::RxFrame {
            header,
            channel_mono_ns,
            frame: frame.to_wire()?,
        })
    }

    /// Read a wire received frame.
    ///
    /// # Errors
    ///
    /// Whatever [`check_header`] or [`Frame::from_wire`] refuses.
    pub fn from_wire(message: &wire::RxFrame) -> Result<Self, ContractError> {
        check_header(&message.header)?;
        Ok(Self {
            daemon_instance_id: message.header.instance_id,
            seq: message.header.seq,
            channel_mono_ns: message.channel_mono_ns,
            frame: Frame::from_wire(&message.frame)?,
        })
    }
}

/// A frame to send, as the daemon reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TxFrame {
    /// The provider instance that sent it.
    pub provider_instance_id: u64,
    /// Its place in the provider's `tx` stream.
    pub seq: u64,
    /// `CLOCK_MONOTONIC` nanoseconds when the provider produced it.
    pub mono_ns: u64,
    /// What it does to the device.
    pub kind: TxKind,
    /// The frame.
    pub frame: Frame,
}

impl TxFrame {
    /// The wire form of `frame` of `kind`.
    ///
    /// # Errors
    ///
    /// Whatever [`Frame::validate`] refuses.
    pub fn to_wire(
        frame: &Frame,
        kind: TxKind,
        header: Header,
    ) -> Result<wire::TxFrame, ContractError> {
        Ok(wire::TxFrame {
            header,
            kind: kind.code(),
            reserved: 0,
            frame: frame.to_wire()?,
        })
    }

    /// Read a wire frame to send.
    ///
    /// # Errors
    ///
    /// Whatever [`check_header`], [`TxKind::from_code`] or
    /// [`Frame::from_wire`] refuses.
    pub fn from_wire(message: &wire::TxFrame) -> Result<Self, ContractError> {
        check_header(&message.header)?;
        Ok(Self {
            provider_instance_id: message.header.instance_id,
            seq: message.header.seq,
            mono_ns: message.header.mono_ns,
            kind: TxKind::from_code(message.kind)?,
            frame: Frame::from_wire(&message.frame)?,
        })
    }
}

/// The provider's heartbeat.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Heartbeat {
    /// The provider instance.
    pub provider_instance_id: u64,
    /// `CLOCK_MONOTONIC` nanoseconds when it was sent.
    pub mono_ns: u64,
    /// How often the provider publishes it, milliseconds.
    pub period_ms: u32,
}

impl Heartbeat {
    /// The wire form.
    #[must_use]
    pub const fn to_wire(period_ms: u32, header: Header) -> wire::Heartbeat {
        wire::Heartbeat {
            header,
            period_ms,
            reserved: 0,
        }
    }

    /// Read a wire heartbeat.
    ///
    /// # Errors
    ///
    /// Whatever [`check_header`] refuses.
    pub fn from_wire(message: &wire::Heartbeat) -> Result<Self, ContractError> {
        check_header(&message.header)?;
        Ok(Self {
            provider_instance_id: message.header.instance_id,
            mono_ns: message.header.mono_ns,
            period_ms: message.period_ms,
        })
    }
}

/// The relay's counters since the daemon started.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counters {
    /// Frames taken off the channel and published on `rx`.
    pub rx_frames: u64,
    /// Tx frames put on the channel.
    pub tx_forwarded: u64,
    /// Motion frames refused while the soft-kill latch stood.
    pub tx_refused_latched: u64,
    /// Tx frames refused by the rate ceiling.
    pub tx_refused_rate: u64,
    /// Tx frames refused as malformed or unsupported by the channel.
    pub tx_refused_invalid: u64,
    /// Tx frames refused because their sender was not the bound instance.
    pub tx_refused_unbound: u64,
    /// Tx frames refused as stale.
    pub tx_refused_stale: u64,
    /// Tx frames of the bound instance that never reached the relay.
    pub tx_lost: u64,
    /// Tx frames the channel failed to send.
    pub tx_failed: u64,
}

/// What the channel is.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ChannelFacts {
    /// The channel hands over a received frame only when it differs from the
    /// frame it handed over last.
    pub suppresses_repeats: bool,
    /// The channel carries 29-bit identifiers.
    pub extended_ids: bool,
    /// The longest payload it carries, bytes.
    pub max_frame_len: u32,
}

/// The relay's status, as a provider reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Status {
    /// The daemon instance that publishes it.
    pub daemon_instance_id: u64,
    /// `CLOCK_MONOTONIC` nanoseconds when it was published.
    pub mono_ns: u64,
    /// The provider instance whose frames the relay takes.
    pub bound_instance_id: Option<u64>,
    /// The newest tx `seq` of the bound instance the relay has taken.
    pub tx_seq_seen: u64,
    /// The channel's newest operation (a send or a receive) succeeded.
    pub channel_up: bool,
    /// The soft-kill latch stands: motion frames are refused.
    pub latched: bool,
    /// The arm controller carrying the channel reports a fault on its side,
    /// so frames sent may not reach the device ([`wire::STATUS_ARM_FAULTED`]).
    pub arm_faulted: bool,
    /// What the channel is.
    pub channel: ChannelFacts,
    /// The tx rate ceiling, frames per second.
    pub max_tx_hz: u32,
    /// How often it is published when nothing changes, milliseconds.
    pub period_ms: u32,
    /// Why the channel is down.
    pub channel_error: Option<String>,
    /// The counters.
    pub counters: Counters,
}

impl Status {
    /// The wire form.
    #[must_use]
    pub fn to_wire(&self, header: Header) -> wire::Status {
        let mut out = wire::Status::zeroed();
        out.header = header;
        out.bound_instance_id = self.bound_instance_id.unwrap_or(0);
        out.tx_seq_seen = self.tx_seq_seen;
        let counters = &self.counters;
        out.rx_frames = counters.rx_frames;
        out.tx_forwarded = counters.tx_forwarded;
        out.tx_refused_latched = counters.tx_refused_latched;
        out.tx_refused_rate = counters.tx_refused_rate;
        out.tx_refused_invalid = counters.tx_refused_invalid;
        out.tx_refused_unbound = counters.tx_refused_unbound;
        out.tx_refused_stale = counters.tx_refused_stale;
        out.tx_lost = counters.tx_lost;
        out.tx_failed = counters.tx_failed;
        out.flags = if self.channel_up {
            wire::STATUS_CHANNEL_UP
        } else {
            0
        } | if self.latched {
            wire::STATUS_LATCHED
        } else {
            0
        } | if self.arm_faulted {
            wire::STATUS_ARM_FAULTED
        } else {
            0
        };
        out.channel_flags = if self.channel.suppresses_repeats {
            wire::CHANNEL_SUPPRESSES_REPEATS
        } else {
            0
        } | if self.channel.extended_ids {
            wire::CHANNEL_EXTENDED_IDS
        } else {
            0
        };
        out.max_tx_hz = self.max_tx_hz;
        out.max_frame_len = self.channel.max_frame_len;
        out.period_ms = self.period_ms;
        if let Some(error) = &self.channel_error {
            put_text(&mut out.channel_error, error);
        }
        out
    }

    /// Read a wire status.
    ///
    /// # Errors
    ///
    /// Whatever [`check_header`] refuses, an undefined flag, or a channel
    /// error that is not NUL-padded UTF-8.
    pub fn from_wire(message: &wire::Status) -> Result<Self, ContractError> {
        check_header(&message.header)?;
        for (what, value, known) in [
            ("status flags", message.flags, wire::STATUS_ALL),
            (
                "status channel_flags",
                message.channel_flags,
                wire::CHANNEL_ALL,
            ),
        ] {
            if value & !known != 0 {
                return Err(invalid(format!(
                    "{what} set bits {:#x} that schema {SCHEMA_VERSION} does not define",
                    value & !known
                )));
            }
        }
        Ok(Self {
            daemon_instance_id: message.header.instance_id,
            mono_ns: message.header.mono_ns,
            bound_instance_id: (message.bound_instance_id != 0)
                .then_some(message.bound_instance_id),
            tx_seq_seen: message.tx_seq_seen,
            channel_up: message.flags & wire::STATUS_CHANNEL_UP != 0,
            latched: message.flags & wire::STATUS_LATCHED != 0,
            arm_faulted: message.flags & wire::STATUS_ARM_FAULTED != 0,
            channel: ChannelFacts {
                suppresses_repeats: message.channel_flags & wire::CHANNEL_SUPPRESSES_REPEATS != 0,
                extended_ids: message.channel_flags & wire::CHANNEL_EXTENDED_IDS != 0,
                max_frame_len: message.max_frame_len,
            },
            max_tx_hz: message.max_tx_hz,
            period_ms: message.period_ms,
            channel_error: get_optional_text(&message.channel_error, "status channel_error")?,
            counters: Counters {
                rx_frames: message.rx_frames,
                tx_forwarded: message.tx_forwarded,
                tx_refused_latched: message.tx_refused_latched,
                tx_refused_rate: message.tx_refused_rate,
                tx_refused_invalid: message.tx_refused_invalid,
                tx_refused_unbound: message.tx_refused_unbound,
                tx_refused_stale: message.tx_refused_stale,
                tx_lost: message.tx_lost,
                tx_failed: message.tx_failed,
            },
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_round_trips_and_an_impossible_one_is_refused() {
        let frame = Frame::standard(0x501, vec![0x81, 0x06, 0x20]);
        let wire = frame.to_wire().unwrap();
        assert_eq!(wire.len, 3);
        assert_eq!(Frame::from_wire(&wire).unwrap(), frame);

        assert!(Frame::standard(0x800, vec![]).to_wire().is_err());
        assert!(Frame::standard(1, vec![0; 65]).to_wire().is_err());
        let extended = Frame {
            id: 0x1234_5678,
            extended: true,
            data: vec![1; 64],
        };
        assert_eq!(
            Frame::from_wire(&extended.to_wire().unwrap()).unwrap(),
            extended
        );

        let mut undefined = wire;
        undefined.flags = 1 << 5;
        assert!(Frame::from_wire(&undefined).is_err());
        let mut long = wire;
        long.len = 65;
        assert!(Frame::from_wire(&long).is_err());
    }

    #[test]
    fn a_tx_frame_carries_its_kind_and_an_unknown_kind_is_refused() {
        let frame = Frame::standard(2, vec![0x0c, 0x00, 0x19]);
        for kind in [TxKind::Motion, TxKind::Read, TxKind::Stop] {
            let wire = TxFrame::to_wire(&frame, kind, header(7, 3)).unwrap();
            let read = TxFrame::from_wire(&wire).unwrap();
            assert_eq!(
                (read.kind, read.seq, read.provider_instance_id),
                (kind, 3, 7)
            );
            assert_eq!(read.frame, frame);
        }
        let mut wire = TxFrame::to_wire(&frame, TxKind::Read, header(7, 1)).unwrap();
        wire.kind = 9;
        assert!(TxFrame::from_wire(&wire).is_err());
        wire.kind = super::wire::TX_READ;
        wire.header.schema_version = 2;
        assert!(matches!(
            TxFrame::from_wire(&wire),
            Err(ContractError::SchemaVersion { got: 2 })
        ));
    }

    #[test]
    fn a_status_round_trips() {
        let status = Status {
            daemon_instance_id: 5,
            mono_ns: 0,
            bound_instance_id: Some(9),
            tx_seq_seen: 41,
            channel_up: true,
            latched: true,
            arm_faulted: true,
            channel: ChannelFacts {
                suppresses_repeats: true,
                extended_ids: false,
                max_frame_len: 64,
            },
            max_tx_hz: 1000,
            period_ms: 100,
            channel_error: None,
            counters: Counters {
                rx_frames: 1,
                tx_forwarded: 2,
                tx_refused_latched: 3,
                tx_refused_rate: 4,
                tx_refused_invalid: 5,
                tx_refused_unbound: 6,
                tx_refused_stale: 7,
                tx_lost: 8,
                tx_failed: 9,
            },
        };
        let wire = status.to_wire(header(5, 1));
        let mut read = Status::from_wire(&wire).unwrap();
        read.mono_ns = 0;
        assert_eq!(read, status);

        let down = Status {
            channel_up: false,
            channel_error: Some("the channel is closed".to_owned()),
            bound_instance_id: None,
            ..status
        };
        let read = Status::from_wire(&down.to_wire(header(5, 2))).unwrap();
        assert_eq!(read.channel_error.as_deref(), Some("the channel is closed"));
        assert_eq!(read.bound_instance_id, None);

        let mut undefined = down.to_wire(header(5, 3));
        undefined.flags |= 1 << 7;
        assert!(Status::from_wire(&undefined).is_err());
    }
}
