//! The daemon's side of `end_effector.shm/1` for one arm side
//! (`spec/end_effector.shm/1.md` §5), and the validation d1-firmwared
//! applies to a command before it reaches its driver.
//!
//! The host keeps what the daemon keeps on its side of the wire:
//!
//! - **The declaration.** The first valid descriptor is accepted and kept
//!   for the host's lifetime; a restarted provider announcing the same
//!   declaration is taken back, a different one is refused. The bus address
//!   may go from none to one value, once.
//! - **Liveness.** The side is available while the provider's heartbeat is
//!   younger than the stale threshold and names the accepted generation,
//!   and, on a side with a flange relay, while the declaration names a node.
//! - **The command ceiling.** `set` and `joints` never leave faster than the
//!   descriptor's `max_command_hz`: a `set` waits for its slot, an early
//!   `joints` target replaces the one waiting (newest wins).
//! - **The stop.** A stop raises the stop generation on `control` at once,
//!   drops a waiting `joints` target, then asks the provider to stop.
//!
//! iceoryx2's ports are not `Send`, so one OS thread owns them; the async
//! methods hand it work over a channel and await the answer on a oneshot.

use std::{
    sync::{
        Arc, Mutex, OnceLock,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
    time::{Duration, Instant},
};

use d1fw_canfd::CanFdBus;
use d1fw_ee_shm_contract::{
    clock,
    model::{
        self, Command, CommandEnvelope, CommandOutcome, ContractError, Control, FaultEnvelope,
        FaultOp, FaultOutcome, Heartbeat, JointTargets, SetCommand,
    },
    names::{self, ServiceNames},
    ports::{self, Ipc},
    root::ShmRoot,
    wire,
};
use iceoryx2::{
    pending_response::PendingResponse,
    port::{client::Client, publisher::Publisher, subscriber::Subscriber},
    prelude::Node,
};
use serde_json::{Map, Value};
use tokio::sync::{oneshot, watch};

use crate::{
    error::{HostError, HostErrorKind},
    relay::{FlangeRelay, FlangeSettings},
};

/// How long the host waits for a `stop` answer, and for a `set` that does
/// not wait for its stroke.
const REQUEST_TIMEOUT: Duration = Duration::from_secs(2);

/// How long the host waits for a `clear_fault` or `registers` answer: the
/// contract's ceiling on any request.
const FAULT_REQUEST_TIMEOUT: Duration = Duration::from_millis(model::MAX_SET_TIMEOUT_MS as u64);

/// What a `set` with `wait` may take beyond the descriptor's own
/// `set_timeout_ms` before the host gives up on the answer.
const SET_MARGIN: Duration = Duration::from_millis(500);

/// How far ahead of the host's wall clock a `joints` timestamp may lie.
pub const JOINTS_FUTURE_TOLERANCE_US: u64 = 1_000_000;

/// How long the port thread waits for work before it polls its ports again.
const TICK: Duration = Duration::from_millis(2);

/// How long the port thread waits before it tries again to open services it
/// could not open.
const REOPEN: Duration = Duration::from_secs(1);

/// How one side binds to its provider.
#[derive(Debug, Clone, PartialEq)]
pub struct HostSettings {
    /// The side's name (`a` or `b`).
    pub side: String,
    /// The shared-memory root.
    pub root: ShmRoot,
    /// The side's service names.
    pub names: ServiceNames,
    /// How long after the provider's newest heartbeat the side is reported
    /// unavailable.
    pub stale_after: Duration,
    /// The model the side must carry: a provider announcing another is
    /// refused.
    pub expect_model: Option<String>,
    /// The relay of the side's flange CAN channel, for a side whose device
    /// sits on it ([`Host::start_with_flange_channel`]).
    pub flange: Option<FlangeSettings>,
}

impl HostSettings {
    /// Side `side` under `root`, on its default service names, with the
    /// contract's default stale threshold and no model pinned.
    ///
    /// # Panics
    ///
    /// When `side` makes an invalid service name.
    #[must_use]
    pub fn new(side: &str, root: ShmRoot) -> Self {
        Self {
            side: side.to_owned(),
            root,
            names: ServiceNames::new(&names::default_base(side)).expect("a valid side name"),
            stale_after: Duration::from_millis(ports::DEFAULT_STALE_AFTER_MS),
            expect_model: None,
            flange: None,
        }
    }
}

/// A `set` as a client sends it to d1-firmwared.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SetRequest {
    /// `0.0` fully closed … `1.0` fully open.
    pub openness: f64,
    /// Answer when the stroke concludes; unset, `true`.
    pub wait: Option<bool>,
    /// A grip preset.
    pub grip: Option<model::GripPreset>,
    /// An explicit grip cap, newton-metres.
    pub torque_nm: Option<f64>,
    /// Driver-scoped parameters.
    pub extra: Option<Map<String, Value>>,
}

impl SetRequest {
    /// A blocking `set` to `openness`, no grip named.
    #[must_use]
    pub fn openness(openness: f64) -> Self {
        Self {
            openness,
            ..Self::default()
        }
    }

    /// The same request, answered at once (`wait = false`) or when the
    /// stroke concludes (`wait = true`).
    #[must_use]
    pub const fn wait(mut self, wait: bool) -> Self {
        self.wait = Some(wait);
        self
    }

    /// The same request with `grip`.
    #[must_use]
    pub const fn grip(mut self, grip: model::GripPreset) -> Self {
        self.grip = Some(grip);
        self
    }
}

#[derive(Debug, Default)]
struct Status {
    transport_error: Option<String>,
    link: Option<Link>,
    state: Option<(model::State, Instant)>,
}

#[derive(Debug)]
struct Link {
    instance_id: u64,
    node_id: Option<u16>,
    last_heartbeat: Option<Instant>,
    heartbeat_generation: u64,
    accepted_generation: Option<u64>,
    refusal: Option<String>,
}

impl Link {
    const fn new(instance_id: u64) -> Self {
        Self {
            instance_id,
            node_id: None,
            last_heartbeat: None,
            heartbeat_generation: 0,
            accepted_generation: None,
            refusal: None,
        }
    }
}

struct Shared {
    settings: HostSettings,
    instance_id: u64,
    declared: OnceLock<model::Descriptor>,
    found: OnceLock<model::Descriptor>,
    accepted: watch::Sender<bool>,
    status: Mutex<Status>,
    next_allowed: Mutex<Instant>,
    stop_generation: AtomicU64,
    command_seq: AtomicU64,
    fault_seq: AtomicU64,
    ticks: watch::Sender<Option<model::Tick>>,
    joints_newest_us: tokio::sync::Mutex<Option<u64>>,
}

impl Shared {
    fn service(&self) -> &str {
        self.settings.names.base()
    }

    fn availability(&self) -> Result<(u64, u64), String> {
        let status = self.status.lock().expect("status lock");
        let service = self.service();
        if let Some(error) = &status.transport_error {
            return Err(format!(
                "the shared-memory services of {service} cannot be opened: {error}"
            ));
        }
        let Some(link) = &status.link else {
            return Err(format!(
                "no end-effector provider has announced itself on {service} (root {}, prefix {})",
                self.settings.root.root_path(),
                self.settings.root.prefix()
            ));
        };
        if let Some(refusal) = &link.refusal {
            return Err(format!(
                "the end-effector provider on {service} was refused: {refusal}"
            ));
        }
        let Some(heartbeat) = link.last_heartbeat else {
            return Err(format!(
                "the end-effector provider on {service} has sent no heartbeat yet"
            ));
        };
        let age = heartbeat.elapsed();
        if age > self.settings.stale_after {
            return Err(format!(
                "the end-effector provider on {service} has sent no heartbeat for {} ms (stale \
                 after {} ms)",
                age.as_millis(),
                self.settings.stale_after.as_millis()
            ));
        }
        match link.accepted_generation {
            Some(generation) if generation == link.heartbeat_generation => {
                Ok((link.instance_id, generation))
            }
            _ => Err(format!(
                "the end-effector provider on {service} has not announced the descriptor its \
                 heartbeat names yet"
            )),
        }
    }

    fn descriptor(&self) -> Option<&model::Descriptor> {
        self.found.get().or_else(|| self.declared.get())
    }

    /// Why the side's device cannot be driven although its provider is
    /// there: on a side with a flange relay, a declaration without a node
    /// id means the device has not answered. The reason is the provider's
    /// own (its newest state's fault), or a generic one.
    fn device_missing(&self) -> Option<String> {
        self.settings.flange.as_ref()?;
        let status = self.status.lock().expect("status lock");
        let link = status.link.as_ref()?;
        if link.accepted_generation.is_none() || link.node_id.is_some() {
            return None;
        }
        Some(
            status
                .state
                .as_ref()
                .and_then(|(sample, _)| sample.fault.clone())
                .unwrap_or_else(|| {
                    format!(
                        "the end-effector provider on {} has not found its device on the flange \
                         channel (its declaration names no node id)",
                        self.service()
                    )
                }),
        )
    }

    fn unavailable(&self) -> HostError {
        HostError::unavailable(
            self.availability()
                .err()
                .or_else(|| self.device_missing())
                .unwrap_or_else(|| {
                    format!("the end-effector provider on {} is absent", self.service())
                }),
        )
    }

    fn device(&self) -> Result<(u64, u64), HostError> {
        let target = self.availability().map_err(HostError::unavailable)?;
        match self.device_missing() {
            Some(reason) => Err(HostError::unavailable(reason)),
            None => Ok(target),
        }
    }

    fn command_interval(&self) -> Option<Duration> {
        self.declared
            .get()
            .and_then(|descriptor| descriptor.capabilities.max_command_hz)
            .map(|hz| Duration::from_secs_f64(1.0 / f64::from(hz)))
    }

    fn take_slot_now(&self) -> bool {
        let Some(interval) = self.command_interval() else {
            return true;
        };
        let mut next = self.next_allowed.lock().expect("gate lock");
        let now = Instant::now();
        if now >= *next {
            *next = now + interval;
            true
        } else {
            false
        }
    }
}

enum Inbound {
    Command {
        message: Box<wire::CommandRequest>,
        deadline: Instant,
        reply: oneshot::Sender<Result<CommandOutcome, HostError>>,
    },
    Fault {
        message: Box<wire::FaultRequest>,
        deadline: Instant,
        reply: oneshot::Sender<Result<FaultOutcome, HostError>>,
    },
    Joints(Box<wire::JointsCommand>),
    Stop,
}

/// The daemon's side of one arm side, in a test's process.
pub struct Host {
    shared: Arc<Shared>,
    inbound: mpsc::Sender<Inbound>,
    stop: Arc<AtomicBool>,
    thread: Mutex<Option<std::thread::JoinHandle<()>>>,
    relay: Option<FlangeRelay>,
}

impl std::fmt::Debug for Host {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("Host")
            .field("side", &self.shared.settings.side)
            .field("service", &self.shared.service())
            .finish_non_exhaustive()
    }
}

impl Host {
    /// Serve `settings`' side without a flange relay.
    #[must_use]
    pub fn start(settings: HostSettings) -> Self {
        Self::start_inner(settings, None)
    }

    /// Serve `settings`' side and relay `channel` to its provider
    /// (`flange_bus.shm/1`) under `settings.flange`, or the side's default
    /// flange settings when that is `None`. The relay's soft-kill latch
    /// starts released and the arm-fault bit clear.
    #[must_use]
    pub fn start_with_flange_channel(
        mut settings: HostSettings,
        channel: Arc<dyn CanFdBus>,
    ) -> Self {
        let flange = settings
            .flange
            .clone()
            .unwrap_or_else(|| FlangeSettings::new(&settings.side));
        settings.flange = Some(flange.clone());
        let relay = FlangeRelay::start(
            settings.side.clone(),
            settings.root.clone(),
            flange,
            settings.stale_after,
            channel,
        );
        Self::start_inner(settings, Some(relay))
    }

    fn start_inner(settings: HostSettings, relay: Option<FlangeRelay>) -> Self {
        let shared = Arc::new(Shared {
            settings,
            instance_id: model::new_instance_id(),
            declared: OnceLock::new(),
            found: OnceLock::new(),
            accepted: watch::channel(false).0,
            status: Mutex::new(Status::default()),
            next_allowed: Mutex::new(Instant::now()),
            stop_generation: AtomicU64::new(0),
            command_seq: AtomicU64::new(0),
            fault_seq: AtomicU64::new(0),
            ticks: watch::channel(None).0,
            joints_newest_us: tokio::sync::Mutex::new(None),
        });
        let (inbound, receiver) = mpsc::channel();
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let shared = Arc::clone(&shared);
            let stop = Arc::clone(&stop);
            std::thread::Builder::new()
                .name(format!("ee-host-{}", shared.settings.side))
                .spawn(move || run(&shared, &receiver, &stop))
                .expect("spawn the host's port thread")
        };
        Self {
            shared,
            inbound,
            stop,
            thread: Mutex::new(Some(thread)),
            relay,
        }
    }

    /// The settings the side was started with.
    #[must_use]
    pub fn settings(&self) -> &HostSettings {
        &self.shared.settings
    }

    /// The side's service names.
    #[must_use]
    pub fn names(&self) -> &ServiceNames {
        &self.shared.settings.names
    }

    /// The accepted declaration, with the node a later announcement found.
    #[must_use]
    pub fn descriptor(&self) -> Option<model::Descriptor> {
        self.shared.descriptor().cloned()
    }

    /// Whether commands can reach the side's device now.
    #[must_use]
    pub fn available(&self) -> bool {
        self.shared.availability().is_ok() && self.shared.device_missing().is_none()
    }

    /// Why the side is unavailable, or `None` while it is available.
    #[must_use]
    pub fn unavailable_reason(&self) -> Option<String> {
        self.shared
            .availability()
            .err()
            .or_else(|| self.shared.device_missing())
    }

    /// Wait until a declaration has been accepted.
    pub async fn declared(&self) {
        let mut accepted = self.shared.accepted.subscribe();
        let _ = accepted.wait_for(|accepted| *accepted).await;
    }

    /// Wait up to `wait` until the side is available.
    ///
    /// # Panics
    ///
    /// When it does not become available, with the reason.
    pub async fn until_available(&self, wait: Duration) {
        self.until("the side to become available", wait, Self::available)
            .await;
    }

    /// Wait up to `wait` until `condition` holds of the host.
    ///
    /// # Panics
    ///
    /// When it does not, naming `what` and the side's unavailability reason.
    pub async fn until(&self, what: &str, wait: Duration, condition: impl Fn(&Self) -> bool) {
        let deadline = Instant::now() + wait;
        while !condition(self) {
            assert!(
                Instant::now() < deadline,
                "timed out waiting for {what}: {:?}",
                self.unavailable_reason()
            );
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    /// Set the relay's soft-kill latch: while it stands, a frame labelled
    /// motion is refused before the channel. Nothing on a side without a
    /// relay.
    pub fn latch(&self, latched: bool) {
        if let Some(relay) = &self.relay {
            relay.latch(latched);
        }
    }

    /// Report the arm controller carrying the side's channel as faulted, or
    /// not, in the relay's status (`arm_faulted`). Nothing on a side without
    /// a relay.
    pub fn set_arm_faulted(&self, faulted: bool) {
        if let Some(relay) = &self.relay {
            relay.set_arm_faulted(faulted);
        }
    }

    /// The newest tick of the accepted provider.
    #[must_use]
    pub fn ticks(&self) -> watch::Receiver<Option<model::Tick>> {
        self.shared.ticks.subscribe()
    }

    /// Validate `request` against the accepted declaration as d1-firmwared
    /// does, and run it.
    ///
    /// # Errors
    ///
    /// [`HostErrorKind::Unavailable`] with no declaration or no live
    /// provider; [`HostErrorKind::Refused`] without the `openness`
    /// capability; [`HostErrorKind::Unprocessable`] for an openness outside
    /// `[0, 1]` or an undeclared grip, torque or `extra` key; the provider's
    /// own refusal or failure; [`HostErrorKind::Timeout`].
    pub async fn set(&self, request: &SetRequest) -> Result<(), HostError> {
        let command = self.validate_set(request)?;
        let target = self.shared.device()?;
        let timeout = if command.wait {
            let set_timeout = self.shared.declared.get().map_or(
                model::DEFAULT_SET_TIMEOUT_MS,
                model::Descriptor::set_timeout_ms,
            );
            Duration::from_millis(u64::from(set_timeout)) + SET_MARGIN
        } else {
            REQUEST_TIMEOUT
        };
        self.take_slot().await;
        self.command(Command::Set(command), target, timeout).await
    }

    fn validate_set(&self, request: &SetRequest) -> Result<SetCommand, HostError> {
        let descriptor = self
            .shared
            .descriptor()
            .ok_or_else(|| self.shared.unavailable())?;
        let capabilities = &descriptor.capabilities;
        if !capabilities.openness {
            return Err(not_capable(descriptor, "set"));
        }
        let openness = request.openness;
        if !openness.is_finite() || !(0.0..=1.0).contains(&openness) {
            return Err(HostError::new(
                HostErrorKind::Unprocessable,
                format!("openness must be within [0, 1], got {openness}"),
            ));
        }
        if let Some(preset) = request.grip
            && !capabilities.grip_presets.contains(&preset)
        {
            return Err(HostError::new(
                HostErrorKind::Unprocessable,
                format!(
                    "end effector driver {} takes no grip preset {preset:?}",
                    descriptor.driver
                ),
            ));
        }
        if request.torque_nm.is_some() && !capabilities.grip_torque {
            return Err(HostError::new(
                HostErrorKind::Unprocessable,
                format!(
                    "end effector driver {} takes no explicit torque_nm grip",
                    descriptor.driver
                ),
            ));
        }
        if let Some(extra) = &request.extra
            && let Some(key) = extra
                .keys()
                .find(|key| !capabilities.set_extra.contains(key))
        {
            return Err(HostError::new(
                HostErrorKind::Unprocessable,
                format!(
                    "end effector driver {} takes no extra parameter {key:?}",
                    descriptor.driver
                ),
            ));
        }
        Ok(SetCommand {
            openness,
            wait: request.wait.unwrap_or(true),
            grip: request.grip,
            torque_nm: request.torque_nm,
            extra: request.extra.clone(),
        })
    }

    /// Validate `targets` against the declaration as d1-firmwared does,
    /// order them by `timestamp_us` (wall-clock microseconds), and hand them
    /// to the provider under the command ceiling (newest wins).
    ///
    /// # Errors
    ///
    /// [`HostErrorKind::Refused`] without the `joints` capability;
    /// [`HostErrorKind::Unprocessable`] for a wrong count, a target out of
    /// its joint's limits, or a timestamp too far ahead;
    /// [`HostErrorKind::Stale`] for a timestamp not newer than the newest
    /// accepted; [`HostErrorKind::Unavailable`].
    pub async fn joints(&self, targets: Vec<f64>, timestamp_us: u64) -> Result<(), HostError> {
        let descriptor = self
            .shared
            .descriptor()
            .ok_or_else(|| self.shared.unavailable())?;
        if !descriptor.capabilities.joints {
            return Err(not_capable(descriptor, "joints"));
        }
        if targets.len() != descriptor.joints.len() {
            return Err(HostError::new(
                HostErrorKind::Unprocessable,
                format!(
                    "{} targets for {} joints",
                    targets.len(),
                    descriptor.joints.len()
                ),
            ));
        }
        for (joint, target) in descriptor.joints.iter().zip(&targets) {
            if !target.is_finite() || !(joint.min..=joint.max).contains(target) {
                return Err(HostError::new(
                    HostErrorKind::Unprocessable,
                    format!(
                        "joint {} target must be within [{}, {}], got {target}",
                        joint.name, joint.min, joint.max
                    ),
                ));
            }
        }
        let mut newest = self.shared.joints_newest_us.lock().await;
        let horizon = clock::wall_clock_us().saturating_add(JOINTS_FUTURE_TOLERANCE_US);
        if timestamp_us > horizon {
            return Err(HostError::new(
                HostErrorKind::Unprocessable,
                format!("timestamp_us {timestamp_us} is ahead of the host clock"),
            ));
        }
        if newest.is_some_and(|newest| newest > horizon) {
            *newest = None;
        }
        if let Some(previous) = *newest
            && timestamp_us <= previous
        {
            return Err(HostError::new(
                HostErrorKind::Stale,
                format!(
                    "joints timestamp_us {timestamp_us} is not newer than the newest accepted \
                     ({previous})"
                ),
            ));
        }
        *newest = Some(timestamp_us);
        let (instance, generation) = self.shared.device()?;
        let message = JointTargets {
            target_instance_id: instance,
            descriptor_generation: generation,
            issued_mono_ns: 0,
            timestamp_us,
            targets,
        }
        .to_wire(model::header(
            self.shared.instance_id,
            self.shared.command_seq.fetch_add(1, Ordering::Relaxed) + 1,
        ))
        .map_err(|error| HostError::new(HostErrorKind::Unprocessable, error.to_string()))?;
        self.inbound
            .send(Inbound::Joints(Box::new(message)))
            .map_err(|_| self.shared.unavailable())
    }

    /// The provider's newest state, as d1-firmwared serves it: `live`
    /// lowered when the sample has aged past the stale threshold, a detail
    /// of another schema than the declared one dropped, velocities and
    /// efforts kept only with their capabilities.
    ///
    /// # Errors
    ///
    /// [`HostErrorKind::Unavailable`] when the side is, or no state has been
    /// published yet.
    pub async fn read(&self) -> Result<model::State, HostError> {
        self.shared.device()?;
        let detail_schema = self
            .shared
            .descriptor()
            .and_then(|descriptor| descriptor.detail_schema.clone());
        let capabilities = self
            .shared
            .declared
            .get()
            .map(|declared| declared.capabilities.clone())
            .unwrap_or_default();
        let status = self.shared.status.lock().expect("status lock");
        let Some((sample, received)) = &status.state else {
            return Err(HostError::unavailable(format!(
                "the end-effector provider on {} has published no state yet",
                self.shared.service()
            )));
        };
        let fresh = received.elapsed() <= self.shared.settings.stale_after;
        let mut state = sample.clone();
        state.live = state.live && fresh;
        if detail_schema.as_deref() != state.detail.as_ref().map(|detail| detail.schema.as_str()) {
            state.detail = None;
        }
        if !capabilities.joint_velocity {
            state.velocity = Vec::new();
        }
        if !capabilities.joint_effort {
            state.effort = Vec::new();
        }
        Ok(state)
    }

    /// Raise the stop generation at once, drop a waiting `joints` target,
    /// and ask the provider to stop.
    ///
    /// # Errors
    ///
    /// [`HostErrorKind::Unavailable`] with no live provider (the raised
    /// generation stands for when it returns); the provider's failure.
    pub async fn stop(&self) -> Result<(), HostError> {
        self.shared.stop_generation.fetch_add(1, Ordering::Relaxed);
        let _ = self.inbound.send(Inbound::Stop);
        let target = self.shared.availability().map_err(|reason| {
            HostError::unavailable(format!(
                "{reason}; the stop is published on {}, and the provider stops on it when it \
                 returns",
                self.shared.settings.names.name(names::CONTROL)
            ))
        })?;
        self.command(Command::Stop, target, REQUEST_TIMEOUT).await
    }

    /// Clear the device's fault latch, for a provider declaring
    /// `clear_fault`.
    ///
    /// # Errors
    ///
    /// [`HostErrorKind::Refused`] without the capability; the provider's own
    /// refusal or failure.
    pub async fn clear_fault(&self) -> Result<model::FaultClear, HostError> {
        let descriptor = self.descriptor();
        match &descriptor {
            Some(descriptor) if descriptor.capabilities.clear_fault => {}
            Some(descriptor) => return Err(not_capable(descriptor, "clear_fault")),
            None => return Err(self.shared.unavailable()),
        }
        let body = self.fault_request(FaultOp::ClearFault).await?;
        self.body(body, "fault clear")
    }

    /// Read registers, for a provider declaring `registers`.
    ///
    /// # Errors
    ///
    /// [`HostErrorKind::Refused`] without the capability; the provider's own
    /// refusal or failure.
    pub async fn registers(&self, ids: Option<Vec<String>>) -> Result<model::Registers, HostError> {
        let descriptor = self.descriptor();
        match &descriptor {
            Some(descriptor) if descriptor.capabilities.registers => {}
            Some(descriptor) => return Err(not_capable(descriptor, "registers")),
            None => return Err(self.shared.unavailable()),
        }
        let body = self.fault_request(FaultOp::Registers(ids)).await?;
        self.body(body, "register dump")
    }

    async fn request<T>(
        &self,
        make: impl FnOnce(oneshot::Sender<Result<T, HostError>>) -> Inbound,
    ) -> Result<T, HostError> {
        let (reply, answer) = oneshot::channel();
        self.inbound.send(make(reply)).map_err(|_| {
            HostError::unavailable(format!(
                "the port thread of {} has stopped",
                self.shared.service()
            ))
        })?;
        answer.await.unwrap_or_else(|_| {
            Err(HostError::unavailable(format!(
                "the port thread of {} dropped the request",
                self.shared.service()
            )))
        })
    }

    async fn take_slot(&self) {
        let Some(interval) = self.shared.command_interval() else {
            return;
        };
        loop {
            let wait = {
                let mut next = self.shared.next_allowed.lock().expect("gate lock");
                let now = Instant::now();
                if now >= *next {
                    *next = now + interval;
                    return;
                }
                *next - now
            };
            tokio::time::sleep(wait).await;
        }
    }

    async fn command(
        &self,
        command: Command,
        target: (u64, u64),
        timeout: Duration,
    ) -> Result<(), HostError> {
        let envelope = CommandEnvelope {
            target_instance_id: target.0,
            descriptor_generation: target.1,
            start_by_mono_ns: clock::monotonic_ns()
                .saturating_add(ports::START_WINDOW_MS * 1_000_000),
            command,
        };
        let seq = self.shared.command_seq.fetch_add(1, Ordering::Relaxed) + 1;
        let message = envelope
            .to_wire(model::header(self.shared.instance_id, seq))
            .map_err(|error| HostError::new(HostErrorKind::Unprocessable, error.to_string()))?;
        let outcome = self
            .request(|reply| Inbound::Command {
                message: Box::new(message),
                deadline: Instant::now() + timeout,
                reply,
            })
            .await?;
        outcome.map_err(|error| HostError::from_provider(self.shared.service(), error))
    }

    async fn fault_request(&self, op: FaultOp) -> Result<Value, HostError> {
        let (instance, _) = self.shared.device()?;
        let seq = self.shared.fault_seq.fetch_add(1, Ordering::Relaxed) + 1;
        let message = FaultEnvelope {
            target_instance_id: instance,
            op,
        }
        .to_wire(model::header(self.shared.instance_id, seq))
        .map_err(|error| HostError::new(HostErrorKind::Invalid, error.to_string()))?;
        let outcome = self
            .request(|reply| Inbound::Fault {
                message: Box::new(message),
                deadline: Instant::now() + FAULT_REQUEST_TIMEOUT,
                reply,
            })
            .await?;
        outcome.map_err(|error| HostError::from_provider(self.shared.service(), error))
    }

    fn body<T: serde::de::DeserializeOwned>(
        &self,
        body: Value,
        what: &str,
    ) -> Result<T, HostError> {
        serde_json::from_value(body).map_err(|error| {
            HostError::new(
                HostErrorKind::Device,
                format!(
                    "end-effector provider on {} answered a {what} body that does not parse: \
                     {error}",
                    self.shared.service()
                ),
            )
        })
    }
}

impl Drop for Host {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.lock().expect("thread lock").take() {
            let _ = thread.join();
        }
    }
}

fn not_capable(descriptor: &model::Descriptor, verb: &str) -> HostError {
    HostError::new(
        HostErrorKind::Refused,
        format!(
            "end effector driver {} ({}) does not support {verb}",
            descriptor.driver, descriptor.model
        ),
    )
}

type CommandPending = PendingResponse<Ipc, wire::CommandRequest, (), wire::CommandResponse, ()>;
type FaultPending = PendingResponse<Ipc, wire::FaultRequest, (), wire::FaultResponse, ()>;

struct Ports {
    descriptor: Subscriber<Ipc, wire::Descriptor, ()>,
    heartbeat: Subscriber<Ipc, wire::Heartbeat, ()>,
    state: Subscriber<Ipc, wire::State, ()>,
    tick: Subscriber<Ipc, wire::Tick, ()>,
    joints: Publisher<Ipc, wire::JointsCommand, ()>,
    control: Publisher<Ipc, wire::Control, ()>,
    command: Client<Ipc, wire::CommandRequest, (), wire::CommandResponse, ()>,
    fault: Client<Ipc, wire::FaultRequest, (), wire::FaultResponse, ()>,
    // Declared last: fields drop in order, and the node must outlive its
    // ports.
    _node: Node<Ipc>,
}

fn transport(what: &str, error: impl std::fmt::Debug) -> ContractError {
    ContractError::Transport(format!("{what}: {error:?}"))
}

impl Ports {
    fn open(shared: &Shared) -> Result<Self, ContractError> {
        let settings = &shared.settings;
        let node = settings.root.node(&format!("ee-host-{}", settings.side))?;
        let names = &settings.names;
        let descriptor = ports::open_topic::<wire::Descriptor>(
            &node,
            &names.name(names::DESCRIPTOR),
            ports::DESCRIPTOR_TOPIC,
        )?
        .subscriber_builder()
        .create()
        .map_err(|error| transport("create the descriptor subscriber", error))?;
        let heartbeat = ports::open_topic::<wire::Heartbeat>(
            &node,
            &names.name(names::HEARTBEAT),
            ports::HEARTBEAT_TOPIC,
        )?
        .subscriber_builder()
        .create()
        .map_err(|error| transport("create the heartbeat subscriber", error))?;
        let state =
            ports::open_topic::<wire::State>(&node, &names.name(names::STATE), ports::STATE_TOPIC)?
                .subscriber_builder()
                .create()
                .map_err(|error| transport("create the state subscriber", error))?;
        let tick =
            ports::open_topic::<wire::Tick>(&node, &names.name(names::TICK), ports::TICK_TOPIC)?
                .subscriber_builder()
                .create()
                .map_err(|error| transport("create the tick subscriber", error))?;
        let joints = ports::open_topic::<wire::JointsCommand>(
            &node,
            &names.name(names::JOINTS),
            ports::JOINTS_TOPIC,
        )?
        .publisher_builder()
        .create()
        .map_err(|error| {
            transport(
                &format!(
                    "create the joints publisher of {} (does another host bind this side?)",
                    names.base()
                ),
                error,
            )
        })?;
        let control = ports::open_topic::<wire::Control>(
            &node,
            &names.name(names::CONTROL),
            ports::CONTROL_TOPIC,
        )?
        .publisher_builder()
        .create()
        .map_err(|error| transport("create the control publisher", error))?;
        let command = ports::open_rpc::<wire::CommandRequest, wire::CommandResponse>(
            &node,
            &names.name(names::COMMAND),
            ports::RPC,
        )?
        .client_builder()
        .create()
        .map_err(|error| transport("create the command client", error))?;
        let fault = ports::open_rpc::<wire::FaultRequest, wire::FaultResponse>(
            &node,
            &names.name(names::FAULT),
            ports::RPC,
        )?
        .client_builder()
        .create()
        .map_err(|error| transport("create the fault client", error))?;
        Ok(Self {
            descriptor,
            heartbeat,
            state,
            tick,
            joints,
            control,
            command,
            fault,
            _node: node,
        })
    }
}

struct Waiting<P, T> {
    pending: P,
    deadline: Instant,
    reply: Option<oneshot::Sender<Result<T, HostError>>>,
}

struct Loop {
    ports: Option<Ports>,
    next_open: Instant,
    held_joints: Option<Box<wire::JointsCommand>>,
    commands: Vec<Waiting<CommandPending, CommandOutcome>>,
    faults: Vec<Waiting<FaultPending, FaultOutcome>>,
    next_control: Instant,
    control_seq: u64,
}

fn run(shared: &Arc<Shared>, inbound: &mpsc::Receiver<Inbound>, stop: &AtomicBool) {
    let mut state = Loop {
        ports: None,
        next_open: Instant::now(),
        held_joints: None,
        commands: Vec::new(),
        faults: Vec::new(),
        next_control: Instant::now(),
        control_seq: 0,
    };
    while !stop.load(Ordering::Relaxed) {
        if state.ports.is_none() && Instant::now() >= state.next_open {
            open(shared, &mut state);
        }
        let first = match inbound.recv_timeout(TICK) {
            Ok(work) => Some(work),
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };
        for work in first
            .into_iter()
            .chain(std::iter::from_fn(|| inbound.try_recv().ok()))
        {
            take(shared, &mut state, work);
        }
        if state.ports.is_some() {
            receive(shared, &state);
            publish(shared, &mut state);
            answer(shared, &mut state);
        }
    }
}

fn open(shared: &Shared, state: &mut Loop) {
    match Ports::open(shared) {
        Ok(ports) => {
            shared.status.lock().expect("status lock").transport_error = None;
            state.ports = Some(ports);
            state.next_control = Instant::now();
        }
        Err(error) => {
            shared.status.lock().expect("status lock").transport_error = Some(error.to_string());
            state.next_open = Instant::now() + REOPEN;
        }
    }
}

fn not_open(shared: &Shared, what: &str) -> HostError {
    HostError::unavailable(format!(
        "{what} on {}: the shared-memory services are not open",
        shared.service()
    ))
}

fn take(shared: &Shared, state: &mut Loop, work: Inbound) {
    match work {
        Inbound::Command {
            message,
            deadline,
            reply,
        } => {
            let Some(ports) = &state.ports else {
                let _ = reply.send(Err(not_open(shared, "command")));
                return;
            };
            match ports.command.send_copy(*message) {
                Ok(pending) if pending.number_of_server_connections() > 0 => {
                    state.commands.push(Waiting {
                        pending,
                        deadline,
                        reply: Some(reply),
                    });
                }
                Ok(_) => {
                    let _ = reply.send(Err(HostError::unavailable(format!(
                        "no end-effector provider serves {}",
                        shared.settings.names.name(names::COMMAND)
                    ))));
                }
                Err(error) => {
                    let _ = reply.send(Err(HostError::unavailable(format!(
                        "send the command to {}: {error:?}",
                        shared.service()
                    ))));
                }
            }
        }
        Inbound::Fault {
            message,
            deadline,
            reply,
        } => {
            let Some(ports) = &state.ports else {
                let _ = reply.send(Err(not_open(shared, "fault request")));
                return;
            };
            match ports.fault.send_copy(*message) {
                Ok(pending) if pending.number_of_server_connections() > 0 => {
                    state.faults.push(Waiting {
                        pending,
                        deadline,
                        reply: Some(reply),
                    });
                }
                Ok(_) => {
                    let _ = reply.send(Err(HostError::unavailable(format!(
                        "no end-effector provider serves {}",
                        shared.settings.names.name(names::FAULT)
                    ))));
                }
                Err(error) => {
                    let _ = reply.send(Err(HostError::unavailable(format!(
                        "send the fault request to {}: {error:?}",
                        shared.service()
                    ))));
                }
            }
        }
        Inbound::Joints(message) => state.held_joints = Some(message),
        Inbound::Stop => {
            state.held_joints = None;
            state.next_control = Instant::now();
        }
    }
}

fn link_for(status: &mut Status, instance: u64) {
    if status
        .link
        .as_ref()
        .is_some_and(|link| link.instance_id == instance)
    {
        return;
    }
    status.link = Some(Link::new(instance));
    status.state = None;
}

/// Accept or refuse a decoded declaration: the first valid one is kept for
/// the host's lifetime; a later one is accepted when it declares the same
/// end effector in everything but the bus address, which may go from none
/// to one value, once.
fn judge(shared: &Shared, declared: &model::Descriptor) -> Result<(), String> {
    if let Some(model) = &shared.settings.expect_model
        && declared.model != *model
    {
        return Err(format!(
            "it announces model {:?}; the side pins {model:?}",
            declared.model
        ));
    }
    if let Some(accepted) = shared.declared.get() {
        if !accepted.same_but_node(declared) {
            return Err(format!(
                "its declaration (driver {:?}, model {:?}, {} joints) differs from the accepted \
                 one (driver {:?}, model {:?}, {} joints)",
                declared.driver,
                declared.model,
                declared.joints.len(),
                accepted.driver,
                accepted.model,
                accepted.joints.len()
            ));
        }
        let bound = shared
            .descriptor()
            .and_then(|descriptor| descriptor.node_id);
        return match (bound, declared.node_id) {
            (_, None) => Ok(()),
            (Some(bound), Some(node)) if bound == node => Ok(()),
            (Some(bound), Some(node)) => Err(format!(
                "it found its device at node {node}; the side is bound to node {bound}"
            )),
            (None, Some(_)) => {
                let _ = shared.found.set(declared.clone());
                Ok(())
            }
        };
    }
    let _ = shared.declared.set(declared.clone());
    shared.accepted.send_replace(true);
    Ok(())
}

fn on_descriptor(shared: &Shared, message: &wire::Descriptor) {
    let decoded = model::Descriptor::from_wire(message);
    let mut status = shared.status.lock().expect("status lock");
    link_for(&mut status, message.header.instance_id);
    let Some(link) = status.link.as_mut() else {
        return;
    };
    let verdict = match decoded {
        Ok((declared, generation)) => {
            if link.accepted_generation == Some(generation) {
                return;
            }
            judge(shared, &declared).map(|()| (generation, declared.node_id))
        }
        Err(error) => Err(format!("its descriptor is malformed: {error}")),
    };
    match verdict {
        Ok((generation, node_id)) => {
            let redeclared = link.accepted_generation.is_some();
            link.accepted_generation = Some(generation);
            link.node_id = node_id;
            link.refusal = None;
            if redeclared {
                status.state = None;
            }
        }
        Err(reason) => {
            link.accepted_generation = None;
            link.refusal = Some(reason);
        }
    }
}

fn on_heartbeat(shared: &Shared, message: &wire::Heartbeat) {
    if let Ok(heartbeat) = Heartbeat::from_wire(message) {
        let mut status = shared.status.lock().expect("status lock");
        link_for(&mut status, message.header.instance_id);
        if let Some(link) = status.link.as_mut() {
            link.last_heartbeat = Some(Instant::now());
            link.heartbeat_generation = heartbeat.descriptor_generation;
        }
    }
}

fn accepted(shared: &Shared) -> Option<(u64, u64, usize)> {
    let status = shared.status.lock().expect("status lock");
    let link = status.link.as_ref()?;
    let generation = link.accepted_generation?;
    let dof = shared.declared.get()?.joints.len();
    Some((link.instance_id, generation, dof))
}

fn receive(shared: &Shared, state: &Loop) {
    let Some(ports) = &state.ports else {
        return;
    };
    let mut descriptors = Vec::new();
    while let Ok(Some(sample)) = ports.descriptor.receive() {
        descriptors.push(*sample.payload());
    }
    let mut heartbeats = Vec::new();
    while let Ok(Some(sample)) = ports.heartbeat.receive() {
        heartbeats.push(*sample.payload());
    }
    let mut newest_state = None;
    while let Ok(Some(sample)) = ports.state.receive() {
        newest_state = Some(Box::new(*sample.payload()));
    }
    let mut ticks = Vec::new();
    while let Ok(Some(sample)) = ports.tick.receive() {
        ticks.push(Box::new(*sample.payload()));
    }
    for message in &descriptors {
        on_descriptor(shared, message);
    }
    for message in &heartbeats {
        on_heartbeat(shared, message);
    }
    let Some((instance, generation, dof)) = accepted(shared) else {
        return;
    };
    if let Some(message) = newest_state
        && message.header.instance_id == instance
        && let Ok((sample, sample_generation)) = model::State::from_wire(&message, dof)
        && sample_generation == generation
    {
        shared.status.lock().expect("status lock").state = Some((sample, Instant::now()));
    }
    for message in ticks {
        if message.header.instance_id != instance {
            continue;
        }
        if let Ok((tick, tick_generation)) = model::Tick::from_wire(&message)
            && tick_generation == generation
        {
            shared.ticks.send_replace(Some(tick));
        }
    }
}

fn publish(shared: &Shared, state: &mut Loop) {
    let Some(ports) = &state.ports else {
        return;
    };
    let now = Instant::now();
    if now >= state.next_control {
        state.next_control = now + Duration::from_millis(ports::CONTROL_PERIOD_MS);
        state.control_seq += 1;
        let control = Control {
            stop_generation: shared.stop_generation.load(Ordering::Relaxed),
            period_ms: u32::try_from(ports::CONTROL_PERIOD_MS).unwrap_or(u32::MAX),
        }
        .to_wire(model::header(shared.instance_id, state.control_seq));
        let _ = ports.control.send_copy(control);
    }
    if state.held_joints.is_some()
        && shared.take_slot_now()
        && let Some(mut message) = state.held_joints.take()
    {
        // Stamped when it leaves: a provider ages a sample from this stamp.
        message.header.mono_ns = clock::monotonic_ns();
        let _ = ports.joints.send_copy(*message);
    }
}

type Settled<T> = (oneshot::Sender<Result<T, HostError>>, Result<T, HostError>);

fn settle<P, T>(
    waiting: &mut Vec<Waiting<P, T>>,
    service: &str,
    now: Instant,
    poll: impl Fn(&P) -> Option<Result<T, HostError>>,
    connected: impl Fn(&P) -> bool,
) -> Vec<Settled<T>> {
    let mut settled = Vec::new();
    waiting.retain_mut(|entry| {
        let outcome = match poll(&entry.pending) {
            Some(outcome) => outcome,
            None if now > entry.deadline => Err(HostError::new(
                HostErrorKind::Timeout,
                format!("the end-effector provider on {service} did not answer in time"),
            )),
            None if !connected(&entry.pending) => Err(HostError::unavailable(format!(
                "the end-effector provider on {service} went away before it answered"
            ))),
            None => return true,
        };
        if let Some(reply) = entry.reply.take() {
            settled.push((reply, outcome));
        }
        false
    });
    settled
}

fn malformed(service: &str, error: &ContractError) -> HostError {
    HostError::new(
        HostErrorKind::Device,
        format!("the end-effector provider on {service} answered a malformed response: {error}"),
    )
}

/// Deliver every answer that arrived. A provider publishes the state a
/// command left before it answers the command (§4), so the topics are read
/// once more before any answer is delivered.
fn answer(shared: &Shared, state: &mut Loop) {
    let now = Instant::now();
    let service = shared.service();
    let commands = settle(
        &mut state.commands,
        service,
        now,
        |pending| match pending.receive() {
            Ok(Some(response)) => Some(
                model::read_command_response(response.payload())
                    .map_err(|error| malformed(service, &error)),
            ),
            Ok(None) => None,
            Err(error) => Some(Err(HostError::new(
                HostErrorKind::Device,
                format!("receive the answer of {service}: {error:?}"),
            ))),
        },
        PendingResponse::is_connected,
    );
    let faults = settle(
        &mut state.faults,
        service,
        now,
        |pending| match pending.receive() {
            Ok(Some(response)) => Some(
                model::read_fault_response(response.payload())
                    .map_err(|error| malformed(service, &error)),
            ),
            Ok(None) => None,
            Err(error) => Some(Err(HostError::new(
                HostErrorKind::Device,
                format!("receive the answer of {service}: {error:?}"),
            ))),
        },
        PendingResponse::is_connected,
    );
    if commands.is_empty() && faults.is_empty() {
        return;
    }
    receive(shared, state);
    for (reply, outcome) in commands {
        let _ = reply.send(outcome);
    }
    for (reply, outcome) in faults {
        let _ = reply.send(outcome);
    }
}
