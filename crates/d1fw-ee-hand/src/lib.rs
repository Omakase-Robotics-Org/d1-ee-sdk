//! What the multi-finger hand providers share.
//!
//! A hand on a D1 flange talks over the arm's end-module CAN channel, which
//! only `d1-firmwared` reaches. Its driver runs as a provider process of its
//! own: it serves the side over `end_effector.shm/1` and reaches the hand
//! through the daemon's relay of that channel, `flange_bus.shm/1`. Each hand
//! model's provider, in a repository of its own, supplies its wire — a
//! [`HandLink`] — and its facts; this crate is the rest:
//!
//! - [`link`] — the per-side link serialization with command priority (a
//!   read in flight yields the bus to a waiting command), the stale-sample
//!   cache, the probe band, and the reply timings;
//! - [`state`] — [`HandState`], the `hand.state/1` detail every hand
//!   publishes (the daemon's telemetry reads it by that id);
//! - [`provider`] — [`HandProvider`], the `end_effector.shm/1` provider of
//!   any [`HandLink`]: openness, joints, a state read that never interrupts
//!   a command, and a stop that disables without opening;
//! - [`channel`] — [`FlangeChannel`], the relayed channel as a
//!   `d1fw_canfd::CanFdBus`, so a link written against the bus seam runs
//!   over the relay unchanged and labels its frames with
//!   `d1fw_canfd::FrameKind`;
//! - [`tool`] — the tool a hand declares, read from its compiled-in
//!   `manipulation-kit` tool configuration;
//! - [`sim`] — a simulated device on the bus seam and the end module's
//!   repeat suppression around any bus;
//! - [`app`] — the provider process: its command line, its startup (channel,
//!   admission, probe) and its service until a signal;
//! - `harness` (feature `test-support`) — the scenario a hand crate's tests
//!   drive its provider with, on the SDK's host (`d1fw-ee-harness`).

pub mod app;
pub mod channel;
#[cfg(feature = "test-support")]
pub mod harness;
pub mod link;
pub mod provider;
pub mod sim;
pub mod state;
pub mod tool;

pub use channel::FlangeChannel;
pub use provider::{HandFacts, HandLink, HandProvider, Side};
pub use state::{HAND_STATE, HAND_STATE_SCHEMA, HandSample, HandState};
