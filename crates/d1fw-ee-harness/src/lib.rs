//! The daemon's side of `end_effector.shm/1` and `flange_bus.shm/1`, for a
//! provider's tests.
//!
//! A provider is written against two contracts whose other side is
//! d1-firmwared. This crate stands that side up in a test's process, under
//! the rules the daemon applies (each contract's §5), so a provider can be
//! proven without the daemon:
//!
//! - [`Host`] — one arm side: it accepts the first valid declaration and
//!   keeps it, judges liveness by the heartbeat, validates a `set` and
//!   `joints` the way the daemon's REST surface does, paces them to the
//!   declared `max_command_hz`, raises the stop generation on `control`, and
//!   answers with the HTTP status the daemon would ([`HostError`]).
//! - [`Host::start_with_flange_channel`] — the same side with its flange CAN
//!   channel relayed to the provider (`flange_bus.shm/1`): any
//!   [`d1fw_canfd::CanFdBus`] stands in for the channel, a simulated device
//!   included; [`Host::latch`] raises the soft-kill latch and
//!   [`Host::set_arm_faulted`] the status bit `arm_faulted`.
//! - [`Recorder`] — a channel that records every frame and the kind it was
//!   relayed as; [`golden`] compares a run with a recording of the frames a
//!   device must see.
//! - [`ProviderProcess`] and [`conformance::lifecycle`] — a provider binary
//!   as a child process, and the lifecycle every provider must follow.
//!
//! The `d1-ee-test-provider` binary serves the SDK's `TestProvider` (a
//! simulated one-joint gripper) as a process, for a host-side test that
//! needs a provider to talk to.

pub mod conformance;
mod error;
pub mod golden;
mod host;
mod process;
mod recorder;
mod relay;

pub use error::{HostError, HostErrorKind, status};
pub use host::{Host, HostSettings, JOINTS_FUTURE_TOLERANCE_US, SetRequest};
pub use process::ProviderProcess;
pub use recorder::{Recorder, Sent};
pub use relay::FlangeSettings;
