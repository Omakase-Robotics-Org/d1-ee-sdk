//! The `end_effector.shm/1` contract: how an end effector driven by its own
//! process talks to `d1-firmwared` over iceoryx2 shared memory.
//!
//! The normative text is `spec/end_effector.shm/1.md` at the repository
//! root, and the byte layout of every message is pinned by
//! `spec/end_effector.shm/1.layout.json`, which [`layout::document`] must
//! reproduce exactly (this crate's `layout_matches_the_committed_spec`
//! test). Both sides of the wire build on this crate: the daemon's driver
//! (d1-firmwared's shared-memory driver) and the provider SDK
//! (`d1fw-ee-provider`), so the two can
//! only disagree by being built from different versions of it.
//!
//! # Shape
//!
//! - [`wire`] — the fixed-size `#[repr(C)]` messages iceoryx2 carries, and
//!   their bit and code constants.
//! - [`model`] — the owned, validated form of each message, and the
//!   conversions between the two. Every decode checks `schema_version` and
//!   every field's range, so a malformed message is refused at the boundary
//!   rather than half-read.
//! - [`names`] — the per-side service names.
//! - [`root`] — the shared-memory root (directory and prefix) every
//!   participant must use, and the iceoryx2 node built on it.
//! - [`ports`] — the per-service iceoryx2 settings, opened identically by
//!   both sides.
//! - [`layout`] — the machine-readable layout of every message.
//! - [`clock`] — the monotonic clock every `mono_ns` field is read from.
//! - [`flange`] — the sibling contract `flange_bus.shm/1`: one side's
//!   flange CAN channel relayed between the daemon and the provider that
//!   drives the device on it (`spec/flange_bus.shm/1.md`).

pub mod clock;
pub mod flange;
pub mod layout;
pub mod model;
pub mod names;
pub mod ports;
pub mod root;
pub mod wire;

/// The contract's name and major version, as documents and logs cite it.
pub const CONTRACT: &str = "end_effector.shm/1";

/// The value of `schema_version` in every message of this contract.
///
/// A message carrying any other value is refused by the receiver, whatever
/// its layout: the iceoryx2 type names carry the major version too (a
/// provider built against another major cannot open the services at all),
/// so a mismatch here is a provider that stamps a version it was not built
/// for.
pub const SCHEMA_VERSION: u32 = 1;

/// The iceoryx2 version this contract is pinned to.
///
/// iceoryx2's shared-memory layout is not guaranteed compatible across
/// releases, so a provider must be built against exactly this version, as
/// every other iceoryx2 participant of the suite is.
pub const ICEORYX2_VERSION: &str = "0.9.3";

/// The most joints an end effector may declare.
///
/// Every joint-carrying message has room for this many; the largest device
/// driven today (the LinkerHand O30) has 20 axes.
pub const MAX_JOINTS: usize = 32;

/// The byte capacity of the `driver`, `detail_schema` and `model` names.
pub const NAME_LEN: usize = 64;

/// The byte capacity of a joint name and of a `set_extra` key.
pub const KEY_LEN: usize = 32;

/// The most `set_extra` keys a descriptor may declare.
pub const MAX_SET_EXTRA: usize = 8;

/// The byte capacity of a fault text.
pub const FAULT_LEN: usize = 128;

/// The byte capacity of a response's error message.
pub const MESSAGE_LEN: usize = 256;

/// The byte capacity of a state's `detail` JSON.
pub const DETAIL_LEN: usize = 8192;

/// The byte capacity of a `set` command's `extra` JSON object.
pub const EXTRA_LEN: usize = 1024;

/// The byte capacity of a `registers` request's id list (a JSON array).
pub const IDS_LEN: usize = 1024;

/// The byte capacity of a fault-lane response body (JSON).
pub const BODY_LEN: usize = 16384;

#[cfg(test)]
mod tests {
    use super::ICEORYX2_VERSION;

    /// The pin this crate states must be the one the workspace builds.
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
