//! The `flange_bus.shm/1` contract: one arm side's flange CAN channel,
//! relayed between `d1-firmwared` and the one provider process that drives
//! the device on it.
//!
//! The channel (the arm controller's end-module passthrough, which only the
//! daemon can reach) stays the daemon's: the daemon publishes every frame it
//! receives on `rx`, takes the provider's frames from `tx` and puts them on
//! the channel only while the provider's `heartbeat` is fresh, within the
//! side's tx rate ceiling and frame size, and, for a frame that can move
//! the device, only while the end effectors' soft-kill latch is released.
//! `status` tells the provider what the relay does and why it refused.
//!
//! The normative text is `spec/flange_bus.shm/1.md` at the repository root;
//! the byte layout of every message is pinned by
//! `spec/flange_bus.shm/1.layout.json`, which [`layout::document`] must
//! reproduce exactly. The contract shares the shared-memory root
//! ([`crate::root`]), the clock ([`crate::clock`]) and the 32-byte
//! [`crate::wire::Header`] with `end_effector.shm/1`; its type names carry
//! their own major (`omakase_flange_v1_*`), so the two contracts version
//! independently.

pub mod layout;
pub mod model;
pub mod wire;

use crate::{model::ContractError, ports::TopicSpec};

/// The contract's name and major version, as documents and logs cite it.
pub const CONTRACT: &str = "flange_bus.shm/1";

/// The value of `schema_version` in every message of this contract.
pub const SCHEMA_VERSION: u32 = 1;

/// The byte capacity of a frame's data: one CAN FD frame.
pub const MAX_FRAME_LEN: usize = 64;

/// The largest 11-bit (standard) CAN identifier.
pub const STANDARD_ID_MAX: u32 = 0x7FF;

/// The largest 29-bit (extended) CAN identifier.
pub const EXTENDED_ID_MAX: u32 = 0x1FFF_FFFF;

/// The byte capacity of the status's channel error text.
pub const ERROR_LEN: usize = 128;

/// How many tx frames a provider may have published that the daemon's
/// newest status does not yet count as taken (`tx_seq_seen`). Half the tx
/// topic's buffer, so a provider that keeps to it never has a frame
/// overwritten before the daemon reads it.
pub const TX_WINDOW: u64 = 32;

/// How old a tx frame may be (its header `mono_ns`) when the daemon takes
/// it, milliseconds; an older one is refused as stale rather than put on
/// the channel late.
pub const TX_MAX_AGE_MS: u64 = 100;

/// How often the daemon publishes `status` when nothing changes,
/// milliseconds. It also publishes it whenever its binding, the latch, the
/// channel's state or `tx_seq_seen` changes.
pub const STATUS_PERIOD_MS: u64 = 100;

/// The longest a provider may leave between two heartbeats, milliseconds.
pub const MAX_HEARTBEAT_PERIOD_MS: u64 = 100;

/// The tx rate ceiling a side gets unless its configuration names another,
/// frames per second.
pub const DEFAULT_MAX_TX_HZ: u32 = 1000;

/// The highest tx rate ceiling a configuration may name, frames per second.
pub const MAX_TX_HZ: u32 = 10_000;

/// The base name of a side's flange services.
#[must_use]
pub fn default_base(side: &str) -> String {
    format!("omakase/flange/{side}")
}

/// Suffix of the daemon's received-frame topic.
pub const RX: &str = "rx";
/// Suffix of the provider's frame-to-send topic.
pub const TX: &str = "tx";
/// Suffix of the daemon's relay-status topic.
pub const STATUS: &str = "status";
/// Suffix of the provider's heartbeat topic.
pub const HEARTBEAT: &str = "heartbeat";

/// The four service names of one side's flange channel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlangeNames(crate::names::ServiceNames);

impl FlangeNames {
    /// The names under `base`, which follows the rules of
    /// [`crate::names::ServiceNames::new`].
    ///
    /// # Errors
    ///
    /// Whatever [`crate::names::ServiceNames::new`] refuses.
    pub fn new(base: &str) -> Result<Self, ContractError> {
        crate::names::ServiceNames::new(base).map(Self)
    }

    /// The base name.
    #[must_use]
    pub fn base(&self) -> &str {
        self.0.base()
    }

    /// The full name of the service with `suffix`.
    #[must_use]
    pub fn name(&self, suffix: &str) -> String {
        self.0.name(suffix)
    }
}

/// `rx`: a stream of frames, newest kept. A provider that falls behind
/// loses the oldest, and counts them from the gaps in the header `seq`.
pub const RX_TOPIC: TopicSpec = TopicSpec {
    history_size: 0,
    buffer_size: 64,
    safe_overflow: true,
    max_publishers: 1,
    max_subscribers: 2,
};

/// `tx`: never replayed (a frame is sent once or not at all), read by the
/// daemon alone. A provider keeps within [`TX_WINDOW`], so its frames are
/// never overwritten unread; the daemon counts a gap in the header `seq`
/// as lost.
pub const TX_TOPIC: TopicSpec = TopicSpec {
    history_size: 0,
    buffer_size: 64,
    safe_overflow: true,
    max_publishers: 1,
    max_subscribers: 1,
};

/// `status`: latched, so a provider that starts learns at once whether it is
/// admitted and what the channel is.
pub const STATUS_TOPIC: TopicSpec = TopicSpec {
    history_size: 1,
    buffer_size: 2,
    safe_overflow: true,
    max_publishers: 1,
    max_subscribers: 4,
};

/// `heartbeat`: only the newest matters.
pub const HEARTBEAT_TOPIC: TopicSpec = STATUS_TOPIC;
