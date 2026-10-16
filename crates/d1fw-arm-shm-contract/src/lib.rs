//! The D1 arm contracts: how `d1-firmwared` publishes everything it reads
//! from the arm controller, and takes joint commands at the controller's
//! own rate, over iceoryx2 shared memory.
//!
//! Four contracts, each versioned on its own:
//!
//! | contract | service | direction | what |
//! | --- | --- | --- | --- |
//! | `arm_feedback.shm/1` | `omakase/arm/feedback` | daemon → readers | every controller feedback record, decoded and raw, as it arrives |
//! | `arm_status.shm/1` | `omakase/arm/status` | daemon → readers | the daemon's view of the arms (mode, lease, latch, tools), latched |
//! | `arm_events.shm/1` | `omakase/arm/events` | daemon → readers | what happened (state changes, guard refusals, stream stops) |
//! | `arm_command.shm/1` | `omakase/arm/command`, `omakase/arm/command_status` | client ↔ daemon | lease-gated joint targets through the motion guard |
//!
//! The normative texts are `spec/arm_*.shm/1.md` at the repository root;
//! the byte layout of every message is pinned by the `1.layout.json` beside
//! each, which [`layout`] must reproduce exactly, and whose SHA-256 each
//! contract's module pins.
//!
//! The contracts share the shared-memory root of `end_effector.shm/1`
//! ([`d1fw_ee_shm_contract::root`]), its clock and its 32-byte header.
//!
//! # Shape
//!
//! - [`wire`] — the fixed-size `#[repr(C)]` messages and their codes.
//! - [`names`] — the service names.
//! - [`ports`] — the iceoryx2 settings of every service, opened identically
//!   by the daemon and every client.
//! - [`model`] — header and text helpers, and the checks a command passes
//!   before the daemon looks at it.
//! - [`layout`] — the machine-readable layout of each contract and its pin.

pub mod layout;
pub mod model;
pub mod names;
pub mod ports;
pub mod wire;

pub use d1fw_ee_shm_contract::{clock, root};

/// `arm_feedback.shm/1`, as documents and logs cite it.
pub const FEEDBACK_CONTRACT: &str = "arm_feedback.shm/1";
/// `arm_status.shm/1`.
pub const STATUS_CONTRACT: &str = "arm_status.shm/1";
/// `arm_events.shm/1`.
pub const EVENTS_CONTRACT: &str = "arm_events.shm/1";
/// `arm_command.shm/1`.
pub const COMMAND_CONTRACT: &str = "arm_command.shm/1";

/// The value of `schema_version` in every message of each of the four
/// contracts (each is at its version 1).
pub const SCHEMA_VERSION: u32 = 1;

/// The iceoryx2 version the contracts are pinned to, as every iceoryx2
/// participant of the suite is.
pub const ICEORYX2_VERSION: &str = d1fw_ee_shm_contract::ICEORYX2_VERSION;

/// Arms on a D1: A (index 0, the physical left arm) and B (index 1).
pub const SIDES: usize = 2;

/// Joints per arm.
pub const JOINTS: usize = 7;

/// Cartesian degrees of freedom.
pub const CART_DOF: usize = 6;

/// The tool dynamics vector's length: mass, centre of mass (3) and inertia
/// (6).
pub const TOOL_DYN_LEN: usize = 10;

/// The byte capacity of a feedback sample's raw record.
pub const RAW_CAPACITY: usize = 2048;

/// The byte capacity of the parameter reply's name.
pub const PARA_NAME_LEN: usize = 32;

/// The byte capacity of the lease holder's name.
pub const HOLDER_LEN: usize = 64;

/// The byte capacity of an event's text.
pub const TEXT_LEN: usize = 128;

/// The byte capacity of a command refusal's message.
pub const MESSAGE_LEN: usize = 128;

/// How many values an event carries.
pub const EVENT_VALUES: usize = 4;

/// The vendor's healthy band for `frame_miss_count`: below this.
pub const FRAME_MISS_HEALTHY_BELOW: i16 = 20;

/// How often the daemon publishes `status` when nothing changes,
/// milliseconds.
pub const STATUS_PERIOD_MS: u64 = 100;

/// The age a command may reach before the daemon refuses it as stale, when
/// the command names none, milliseconds.
pub const DEFAULT_COMMAND_MAX_AGE_MS: u64 = 50;

/// The largest age a command may name, milliseconds.
pub const MAX_COMMAND_MAX_AGE_MS: u64 = 1_000;

/// The `arm_command` rate ceiling a daemon applies unless its configuration
/// names a lower one, commands per second.
pub const DEFAULT_MAX_COMMAND_HZ: u32 = 1_000;

/// The highest rate ceiling a daemon may be configured with, commands per
/// second.
pub const MAX_COMMAND_HZ: u32 = 1_000;

#[cfg(test)]
mod tests {
    use super::ICEORYX2_VERSION;

    #[test]
    fn pinned_iceoryx2_version_matches_the_workspace_manifest() {
        let manifest = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../Cargo.toml"),
        )
        .expect("read the workspace manifest");
        let expected = format!("iceoryx2 = \"={ICEORYX2_VERSION}\"");
        assert!(
            manifest.contains(&expected),
            "the workspace manifest must pin {expected}"
        );
    }
}
