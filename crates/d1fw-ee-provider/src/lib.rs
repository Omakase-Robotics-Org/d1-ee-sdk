//! The provider SDK of `end_effector.shm/1`: what a process that drives its
//! own end effector links to serve it to `d1-firmwared` over iceoryx2 shared
//! memory.
//!
//! A provider implements [`Provider`] — its [`Descriptor`], the two command
//! modes (`set` with an openness, `joints` with timestamped targets), a
//! state read, a stop, and optionally a fault clear, a register dump and a
//! stream of control-loop ticks — and calls [`serve`]. The SDK owns every
//! iceoryx2 port on a thread of its own, announces the descriptor, publishes
//! the heartbeat and the state on their periods, takes the daemon's
//! commands, and applies the contract's rules before a command reaches the
//! provider:
//!
//! - a command addressed to another provider instance or descriptor
//!   generation, a `set` past its start deadline and a `joints` sample older
//!   than [`ports::JOINTS_MAX_AGE_MS`] are dropped (answered
//!   `stale_target`), never executed late;
//! - a `set` naming a grip, a torque or an `extra` key the descriptor does
//!   not declare, a declared `force_percent` that is not a number in
//!   `[0, 100]`, and `joints` of the wrong count or outside a joint's
//!   limits, are refused before the provider sees them;
//! - a rise of the daemon's stop generation (its `control` topic) calls
//!   [`Provider::stop`] even when the `stop` request itself did not arrive;
//! - joint targets reach the provider one at a time, newest first: a target
//!   superseded while the provider was busy is skipped.
//!
//! A provider whose device sits on the arm's flange CAN channel, which only
//! the daemon reaches, opens [`FlangeBus`] as well: the sibling contract
//! `flange_bus.shm/1` relays that channel. It receives every frame the
//! channel delivers, and sends frames labelled with what they do to the
//! device ([`TxKind`]), which the relay gates on.
//!
//! A provider that takes the grip-force convention
//! (`spec/end_effector.shm/1.md` §9) lists [`FORCE_PERCENT`] in its
//! descriptor's `set_extra`, reads the value with [`force_percent`], maps it
//! with [`ForceRange::at_percent`], and puts a [`GripForce`] report at
//! [`GRIP_FORCE`] of every state's detail `data`.
//!
//! The daemon's side of both contracts is d1-firmwared's; `d1fw-ee-harness`
//! stands it up in a provider's tests. The contracts' texts are
//! `spec/end_effector.shm/1.md` and `spec/flange_bus.shm/1.md` at the
//! repository root.

use std::sync::Arc;

use async_trait::async_trait;
pub use d1fw_ee_shm_contract::{
    self as contract, CONTRACT, FORCE_PERCENT, FORCE_PERCENT_MAX, FORCE_PERCENT_MIN, GRIP_FORCE,
    SCHEMA_VERSION,
    model::{
        Capabilities, ContractError, Descriptor, Detail, ErrorKind, FaultClear, ForceSensing,
        GripPreset, Joint, JointUnit, ProviderError, Register, Registers, SetCommand, State, Tick,
        TickPhase, Tool,
    },
    names::{self, ServiceNames},
    ports,
    root::ShmRoot,
};
use tokio::sync::watch;

mod flange;
mod grip_force;
mod runtime;
#[cfg(any(test, feature = "test-support"))]
mod test_provider;
mod tool;
pub mod unit;

pub use d1fw_ee_shm_contract::flange::{
    FlangeNames,
    model::{ChannelFacts, Counters, Frame, RxFrame, Status as FlangeStatus, TxKind},
};
pub use flange::{FlangeBus, FlangeError, FlangeOptions, FlangeStats};
pub use grip_force::{
    ForceRange, ForceUnit, GripForce, GripForceMode, check_force_percent, force_percent,
};
pub use runtime::{ProviderHandle, ServeOptions, serve};
#[cfg(any(test, feature = "test-support"))]
pub use test_provider::{
    TEST_DETAIL_SCHEMA, TEST_GUARD_MODEL, TEST_TOOL, TestProvider, TestRecord,
};
pub use tool::tool_from_config;

/// An end effector served over the contract.
///
/// Every async method is called from the SDK's tokio tasks, never from the
/// thread that owns the iceoryx2 ports, so a slow device delays only its own
/// answer. Commands arrive already checked against [`Provider::descriptor`].
#[async_trait]
pub trait Provider: Send + Sync + 'static {
    /// What this provider drives. Read once, when [`serve`] starts; a
    /// provider whose declaration changes is restarted (the daemon keeps the
    /// first declaration it accepted for its lifetime and refuses a
    /// different one).
    fn descriptor(&self) -> Descriptor;

    /// Drive to an openness. With `wait` and the `stroke_completion`
    /// capability, answer when the stroke concludes; otherwise once the
    /// target is on its way.
    async fn set(&self, command: SetCommand) -> Result<(), ProviderError>;

    /// Drive every joint to its target. Never waits for motion.
    async fn joints(&self, targets: Vec<f64>, timestamp_us: u64) -> Result<(), ProviderError>;

    /// Read the current state. Called every state period; an error is
    /// published as a state with no reading and the error as its fault.
    async fn read(&self) -> Result<State, ProviderError>;

    /// The state a command left: called once a `set`, a `stop` or a
    /// `clear_fault` has completed, and published before the command's
    /// answer, so a client that reads the side after the answer sees what
    /// the command did (a stroke's outcome, a cleared fault) rather than the
    /// previous period's sample. The default is [`Self::read`]; a provider
    /// whose read costs a device exchange may answer from what the command
    /// itself established.
    async fn settled(&self) -> Result<State, ProviderError> {
        self.read().await
    }

    /// Stop whatever the end effector is doing and take its actuators'
    /// authority away, without opening: an end effector holding an object
    /// must not drop it. Must be idempotent: a soft kill may deliver it
    /// twice (the request and the control topic's stop generation).
    async fn stop(&self) -> Result<(), ProviderError>;

    /// Clear the device's fault latch, for a provider that declares
    /// `clear_fault`.
    async fn clear_fault(&self) -> Result<FaultClear, ProviderError> {
        Err(ProviderError::new(
            ErrorKind::Refused,
            "this provider does not clear faults",
        ))
    }

    /// Read registers (these ids, or the provider's default set), for a
    /// provider that declares `registers`.
    async fn registers(&self, ids: Option<Vec<String>>) -> Result<Registers, ProviderError> {
        let _ = ids;
        Err(ProviderError::new(
            ErrorKind::Refused,
            "this provider does not read registers",
        ))
    }

    /// Control-loop ticks, for a provider that declares `ticks`: the SDK
    /// publishes every value the receiver yields.
    fn ticks(&self) -> Option<watch::Receiver<Tick>> {
        None
    }

    /// The daemon appeared (`true`) or has published no `control` for longer
    /// than [`ServeOptions::daemon_stale_after`] (`false`). A provider keeps
    /// holding what it holds while the daemon is away and starts no motion
    /// of its own; the default does nothing. Called from the thread that
    /// owns the ports, so it must return at once.
    fn daemon_presence(&self, present: bool) {
        let _ = present;
    }
}

/// Check a `set` against a declaration, as the SDK does before the provider
/// sees it.
///
/// # Errors
///
/// [`ErrorKind::Refused`] without the `openness` capability;
/// [`ErrorKind::Unprocessable`] for a grip, torque or `extra` key the
/// descriptor does not declare, and for a declared [`FORCE_PERCENT`] that is
/// not a finite number in `[0, 100]`.
pub fn check_set(descriptor: &Descriptor, command: &SetCommand) -> Result<(), ProviderError> {
    let capabilities = &descriptor.capabilities;
    if !capabilities.openness {
        return Err(ProviderError::new(
            ErrorKind::Refused,
            "this end effector does not take an openness",
        ));
    }
    if let Some(grip) = command.grip
        && !capabilities.grip_presets.contains(&grip)
    {
        return Err(ProviderError::new(
            ErrorKind::Unprocessable,
            format!("this end effector takes no grip preset {grip:?}"),
        ));
    }
    if command.torque_nm.is_some() && !capabilities.grip_torque {
        return Err(ProviderError::new(
            ErrorKind::Unprocessable,
            "this end effector takes no explicit torque_nm",
        ));
    }
    if let Some(key) = command
        .extra
        .iter()
        .flat_map(|extra| extra.keys())
        .find(|key| !capabilities.set_extra.contains(key))
    {
        return Err(ProviderError::new(
            ErrorKind::Unprocessable,
            format!("this end effector takes no extra parameter {key:?}"),
        ));
    }
    force_percent(command)?;
    Ok(())
}

/// Check joint targets against a declaration, as the SDK does before the
/// provider sees them.
///
/// # Errors
///
/// [`ErrorKind::Refused`] without the `joints` capability;
/// [`ErrorKind::Unprocessable`] for a count other than the declared joints
/// or a target outside its joint's limits.
pub fn check_joints(descriptor: &Descriptor, targets: &[f64]) -> Result<(), ProviderError> {
    if !descriptor.capabilities.joints {
        return Err(ProviderError::new(
            ErrorKind::Refused,
            "this end effector does not take joint targets",
        ));
    }
    if targets.len() != descriptor.joints.len() {
        return Err(ProviderError::new(
            ErrorKind::Unprocessable,
            format!(
                "{} targets for {} joints",
                targets.len(),
                descriptor.joints.len()
            ),
        ));
    }
    for (joint, target) in descriptor.joints.iter().zip(targets) {
        if !(joint.min..=joint.max).contains(target) {
            return Err(ProviderError::new(
                ErrorKind::Unprocessable,
                format!(
                    "joint {} target {target} is outside [{}, {}]",
                    joint.name, joint.min, joint.max
                ),
            ));
        }
    }
    Ok(())
}

/// A provider behind an `Arc`, as [`serve`] takes it.
pub type SharedProvider = Arc<dyn Provider>;
