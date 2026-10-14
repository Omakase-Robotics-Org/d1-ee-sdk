//! The SDK's runtime: one OS thread that owns every iceoryx2 port, and the
//! tokio tasks that call the provider.
//!
//! iceoryx2's ports, and above all an `ActiveRequest` (the handle a
//! response is sent on), are not `Send`, so they never leave the port
//! thread. Every call into the provider runs on the tokio runtime [`serve`]
//! was called from, and its result comes back to the port thread over a
//! channel, tagged with the request it answers.

use std::{
    collections::HashMap,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
    time::{Duration, Instant},
};

use d1fw_ee_shm_contract::{
    clock,
    model::{
        self, Command, CommandEnvelope, CommandOutcome, ContractError, Control, Descriptor,
        ErrorKind, FaultEnvelope, FaultOp, FaultOutcome, Heartbeat, JointTargets, ProviderError,
        State, Tick,
    },
    names::{self, ServiceNames},
    ports::{self, Ipc},
    root::ShmRoot,
    wire,
};
use iceoryx2::{
    active_request::ActiveRequest,
    port::{publisher::Publisher, server::Server, subscriber::Subscriber},
    prelude::Node,
};
use tokio::{runtime::Handle, sync::watch, task::JoinHandle};
use tracing::{debug, info, warn};

use crate::{Provider, check_joints, check_set};

/// How long the port thread waits for work before it polls its ports again.
/// The latency a command sees before it is handed to the provider.
const TICK: Duration = Duration::from_millis(2);

/// How long a state read may take before its sample is published as having
/// no reading.
const READ_TIMEOUT: Duration = Duration::from_secs(1);

/// How a provider is served.
#[derive(Debug, Clone)]
pub struct ServeOptions {
    /// The shared-memory root; on a robot, [`ShmRoot::default`].
    pub root: ShmRoot,
    /// The side's service names.
    pub names: ServiceNames,
    /// The iceoryx2 node name.
    pub node_name: String,
    /// How often the state is read and published. At most 1 s; the daemon
    /// reports a reading older than its stale threshold as not live.
    pub state_period: Duration,
    /// How often the heartbeat is published; at most
    /// [`ports::MAX_HEARTBEAT_PERIOD_MS`].
    pub heartbeat_period: Duration,
    /// How long without the daemon's `control` before the provider is told
    /// the daemon is absent.
    pub daemon_stale_after: Duration,
}

impl ServeOptions {
    /// Serve the side named `names` under `root`, with the contract's
    /// default periods (state 50 Hz, heartbeat 20 Hz).
    #[must_use]
    pub fn new(root: ShmRoot, names: ServiceNames) -> Self {
        Self {
            root,
            names,
            node_name: "ee-provider".to_owned(),
            state_period: Duration::from_millis(20),
            heartbeat_period: Duration::from_millis(50),
            daemon_stale_after: Duration::from_millis(ports::DEFAULT_STALE_AFTER_MS),
        }
    }

    fn validate(&self) -> Result<(), ContractError> {
        if self.heartbeat_period.is_zero()
            || self.heartbeat_period > Duration::from_millis(ports::MAX_HEARTBEAT_PERIOD_MS)
        {
            return Err(ContractError::Invalid(format!(
                "heartbeat_period {:?} must be above zero and at most {} ms",
                self.heartbeat_period,
                ports::MAX_HEARTBEAT_PERIOD_MS
            )));
        }
        if self.state_period.is_zero() || self.state_period > Duration::from_secs(1) {
            return Err(ContractError::Invalid(format!(
                "state_period {:?} must be above zero and at most 1 s",
                self.state_period
            )));
        }
        Ok(())
    }
}

/// A provider being served. Dropping it stops serving: every port is closed
/// before the drop returns, so another provider can bind the side at once.
pub struct ProviderHandle {
    instance_id: u64,
    /// The declaration being served.
    declared: std::sync::Mutex<Descriptor>,
    redeclare: mpsc::Sender<Descriptor>,
    stop: Arc<AtomicBool>,
    daemon_present: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
    tasks: Vec<JoinHandle<()>>,
}

impl std::fmt::Debug for ProviderHandle {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("ProviderHandle")
            .field("instance_id", &self.instance_id)
            .finish_non_exhaustive()
    }
}

impl ProviderHandle {
    /// This provider instance's id, as every message it sends carries it.
    #[must_use]
    pub const fn instance_id(&self) -> u64 {
        self.instance_id
    }

    /// Whether the daemon's `control` has arrived within
    /// [`ServeOptions::daemon_stale_after`].
    #[must_use]
    pub fn daemon_present(&self) -> bool {
        self.daemon_present.load(Ordering::Relaxed)
    }

    /// Announce `descriptor` in place of the declaration being served, as
    /// its next generation: the same declaration with the bus address its
    /// device answered at, when it was announced before the device answered
    /// (`node_id: None`). Nothing else may differ — the daemon keeps the
    /// first declaration it accepted for its lifetime and takes a later one
    /// only when it differs in the address, from none to one value
    /// (`end_effector.shm/1` §5). Commands the daemon validated against the
    /// previous generation are answered `stale_target`.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for a declaration that differs in anything
    /// but `node_id`, or that the contract refuses;
    /// [`ContractError::Transport`] when the port thread has stopped.
    pub fn redeclare(&self, descriptor: Descriptor) -> Result<(), ContractError> {
        let mut declared = self
            .declared
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if !declared.same_but_node(&descriptor) {
            return Err(ContractError::Invalid(
                "a redeclaration may change only node_id".to_owned(),
            ));
        }
        descriptor.to_wire(1, model::header(self.instance_id, 1))?;
        self.redeclare
            .send(descriptor.clone())
            .map_err(|_| ContractError::Transport("the port thread has stopped".to_owned()))?;
        *declared = descriptor;
        Ok(())
    }

    fn halt(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for task in self.tasks.drain(..) {
            task.abort();
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for ProviderHandle {
    fn drop(&mut self) {
        self.halt();
    }
}

/// What the tokio side hands the port thread.
enum Outbound {
    State(State),
    Tick(Tick),
    CommandDone { id: u64, outcome: CommandOutcome },
    FaultDone { id: u64, outcome: FaultOutcome },
}

/// Serve `provider` until the returned handle is dropped.
///
/// Must be called from within a tokio runtime, which the provider's calls
/// run on. Returns once every port exists.
///
/// # Errors
///
/// [`ContractError::Invalid`] for options or a descriptor the contract
/// refuses; [`ContractError::Transport`] when the root, the node or a port
/// cannot be created — among others when another provider already serves
/// the side (every topic admits one publisher).
pub async fn serve(
    provider: Arc<dyn Provider>,
    options: ServeOptions,
) -> Result<ProviderHandle, ContractError> {
    options.validate()?;
    let descriptor = provider.descriptor();
    let instance_id = model::new_instance_id();
    descriptor.to_wire(1, model::header(instance_id, 1))?;

    let runtime = Handle::current();
    let stop = Arc::new(AtomicBool::new(false));
    let daemon_present = Arc::new(AtomicBool::new(false));
    let (outbound, outbound_receiver) = mpsc::channel();
    let (ready, ready_receiver) = tokio::sync::oneshot::channel();
    let (joints_sender, joints_receiver) = watch::channel::<Option<JointTargets>>(None);
    let (redeclare, redeclarations) = mpsc::channel();
    let settled = Arc::new(AtomicU64::new(0));

    let thread = {
        let context = PortContext {
            options: options.clone(),
            descriptor: descriptor.clone(),
            instance_id,
            provider: Arc::clone(&provider),
            runtime: runtime.clone(),
            stop: Arc::clone(&stop),
            daemon_present: Arc::clone(&daemon_present),
            outbound: outbound.clone(),
            joints: joints_sender,
            settled: Arc::clone(&settled),
        };
        std::thread::Builder::new()
            .name("ee-provider-ports".to_owned())
            .spawn(move || run_ports(context, &outbound_receiver, &redeclarations, ready))
            .map_err(|error| ContractError::Transport(format!("spawn the port thread: {error}")))?
    };
    let mut handle = ProviderHandle {
        instance_id,
        declared: std::sync::Mutex::new(descriptor.clone()),
        redeclare,
        stop,
        daemon_present,
        thread: Some(thread),
        tasks: Vec::new(),
    };
    match ready_receiver.await {
        Ok(Ok(())) => {}
        Ok(Err(error)) => return Err(error),
        Err(_) => {
            return Err(ContractError::Transport(
                "the port thread ended before it was ready".to_owned(),
            ));
        }
    }

    handle.tasks.push(tokio::spawn(read_states(
        Arc::clone(&provider),
        options.state_period,
        outbound.clone(),
        settled,
    )));
    handle.tasks.push(tokio::spawn(drive_joints(
        Arc::clone(&provider),
        joints_receiver,
    )));
    if let Some(ticks) = provider.ticks() {
        handle
            .tasks
            .push(tokio::spawn(forward_ticks(ticks, outbound)));
    }
    info!(
        contract = d1fw_ee_shm_contract::CONTRACT,
        service = options.names.base(),
        root = options.root.root_path(),
        prefix = options.root.prefix(),
        instance_id,
        driver = descriptor.driver.as_str(),
        model = descriptor.model.as_str(),
        "end-effector provider serving"
    );
    Ok(handle)
}

/// Read the state every period and hand it to the port thread.
///
/// A read that a command completed during is not handed on: the state the
/// command left was published before its answer ([`settled_state`]), and a
/// sample taken while the command ran could be older than it.
async fn read_states(
    provider: Arc<dyn Provider>,
    period: Duration,
    outbound: mpsc::Sender<Outbound>,
    settled: Arc<AtomicU64>,
) {
    let mut interval = tokio::time::interval(period);
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        interval.tick().await;
        let commands_before = settled.load(Ordering::Acquire);
        let state = match tokio::time::timeout(READ_TIMEOUT, provider.read()).await {
            Ok(Ok(state)) => state,
            Ok(Err(error)) => State {
                fault: Some(format!("state read failed: {}", error.message)),
                ..State::default()
            },
            Err(_) => State {
                fault: Some(format!("state read took longer than {READ_TIMEOUT:?}")),
                ..State::default()
            },
        };
        if settled.load(Ordering::Acquire) != commands_before {
            continue;
        }
        if outbound.send(Outbound::State(state)).is_err() {
            return;
        }
    }
}

/// Hand the newest joint targets to the provider, one call at a time.
async fn drive_joints(
    provider: Arc<dyn Provider>,
    mut targets: watch::Receiver<Option<JointTargets>>,
) {
    while targets.changed().await.is_ok() {
        let Some(command) = targets.borrow_and_update().clone() else {
            continue;
        };
        if let Err(error) = provider.joints(command.targets, command.timestamp_us).await {
            debug!(%error, "joint targets refused by the provider");
        }
    }
}

async fn forward_ticks(mut ticks: watch::Receiver<Tick>, outbound: mpsc::Sender<Outbound>) {
    while ticks.changed().await.is_ok() {
        let tick = ticks.borrow_and_update().clone();
        if outbound.send(Outbound::Tick(tick)).is_err() {
            return;
        }
    }
}

struct PortContext {
    options: ServeOptions,
    descriptor: Descriptor,
    instance_id: u64,
    provider: Arc<dyn Provider>,
    runtime: Handle,
    stop: Arc<AtomicBool>,
    daemon_present: Arc<AtomicBool>,
    outbound: mpsc::Sender<Outbound>,
    joints: watch::Sender<Option<JointTargets>>,
    /// Counts the commands that completed; see [`read_states`].
    settled: Arc<AtomicU64>,
}

type CommandServer = Server<Ipc, wire::CommandRequest, (), wire::CommandResponse, ()>;
type FaultServer = Server<Ipc, wire::FaultRequest, (), wire::FaultResponse, ()>;
type CommandRequest = ActiveRequest<Ipc, wire::CommandRequest, (), wire::CommandResponse, ()>;
type FaultRequest = ActiveRequest<Ipc, wire::FaultRequest, (), wire::FaultResponse, ()>;

/// Every port of a provider, and the node they belong to.
struct Ports {
    descriptor: Publisher<Ipc, wire::Descriptor, ()>,
    heartbeat: Publisher<Ipc, wire::Heartbeat, ()>,
    state: Publisher<Ipc, wire::State, ()>,
    tick: Publisher<Ipc, wire::Tick, ()>,
    joints: Subscriber<Ipc, wire::JointsCommand, ()>,
    control: Subscriber<Ipc, wire::Control, ()>,
    command: CommandServer,
    fault: FaultServer,
    // Declared last: fields drop in order, and the node must outlive its
    // ports.
    _node: Node<Ipc>,
}

fn transport(what: &str, error: impl std::fmt::Debug) -> ContractError {
    ContractError::Transport(format!("{what}: {error:?}"))
}

impl Ports {
    fn open(options: &ServeOptions) -> Result<Self, ContractError> {
        let node = options.root.node(&options.node_name)?;
        let names = &options.names;
        let publisher = |suffix: &str| names.name(suffix);
        let descriptor = ports::open_topic::<wire::Descriptor>(
            &node,
            &publisher(names::DESCRIPTOR),
            ports::DESCRIPTOR_TOPIC,
        )?
        .publisher_builder()
        .create()
        .map_err(|error| {
            transport(
                &format!(
                    "create the descriptor publisher of {} (is another provider serving this \
                     side?)",
                    names.base()
                ),
                error,
            )
        })?;
        let heartbeat = ports::open_topic::<wire::Heartbeat>(
            &node,
            &publisher(names::HEARTBEAT),
            ports::HEARTBEAT_TOPIC,
        )?
        .publisher_builder()
        .create()
        .map_err(|error| transport("create the heartbeat publisher", error))?;
        let state =
            ports::open_topic::<wire::State>(&node, &publisher(names::STATE), ports::STATE_TOPIC)?
                .publisher_builder()
                .create()
                .map_err(|error| transport("create the state publisher", error))?;
        let tick =
            ports::open_topic::<wire::Tick>(&node, &publisher(names::TICK), ports::TICK_TOPIC)?
                .publisher_builder()
                .create()
                .map_err(|error| transport("create the tick publisher", error))?;
        let joints = ports::open_topic::<wire::JointsCommand>(
            &node,
            &publisher(names::JOINTS),
            ports::JOINTS_TOPIC,
        )?
        .subscriber_builder()
        .create()
        .map_err(|error| transport("create the joints subscriber", error))?;
        let control = ports::open_topic::<wire::Control>(
            &node,
            &publisher(names::CONTROL),
            ports::CONTROL_TOPIC,
        )?
        .subscriber_builder()
        .create()
        .map_err(|error| transport("create the control subscriber", error))?;
        let command = ports::open_rpc::<wire::CommandRequest, wire::CommandResponse>(
            &node,
            &publisher(names::COMMAND),
            ports::RPC,
        )?
        .server_builder()
        .create()
        .map_err(|error| transport("create the command server", error))?;
        let fault = ports::open_rpc::<wire::FaultRequest, wire::FaultResponse>(
            &node,
            &publisher(names::FAULT),
            ports::RPC,
        )?
        .server_builder()
        .create()
        .map_err(|error| transport("create the fault server", error))?;
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

/// The per-service sequence numbers of one sender.
#[derive(Default)]
struct Seqs {
    descriptor: u64,
    heartbeat: u64,
    state: u64,
    tick: u64,
    command: u64,
    fault: u64,
}

fn next(seq: &mut u64) -> u64 {
    *seq += 1;
    *seq
}

/// What the port thread knows of the daemon.
#[derive(Default)]
struct Daemon {
    /// The daemon instance whose stop generation is the baseline.
    instance_id: u64,
    /// The newest stop generation acted on (or taken as the baseline).
    stop_generation: u64,
    /// When its newest `control` arrived.
    last_seen: Option<Instant>,
    /// What the provider was last told.
    reported_present: bool,
}

fn run_ports(
    mut context: PortContext,
    outbound: &mpsc::Receiver<Outbound>,
    redeclarations: &mpsc::Receiver<Descriptor>,
    ready: tokio::sync::oneshot::Sender<Result<(), ContractError>>,
) {
    let ports = match Ports::open(&context.options) {
        Ok(ports) => {
            let _ = ready.send(Ok(()));
            ports
        }
        Err(error) => {
            let _ = ready.send(Err(error));
            return;
        }
    };
    let mut generation = 1;
    let mut seqs = Seqs::default();
    let mut daemon = Daemon::default();
    let mut commands: HashMap<u64, CommandRequest> = HashMap::new();
    let mut faults: HashMap<u64, FaultRequest> = HashMap::new();
    let mut next_request_id = 0_u64;
    let mut next_heartbeat = Instant::now();
    let mut next_descriptor = Instant::now();
    let mut heartbeat = Heartbeat {
        descriptor_generation: generation,
        state_period_ms: u32::try_from(context.options.state_period.as_millis()).unwrap_or(1000),
    };

    while !context.stop.load(Ordering::Relaxed) {
        while let Ok(descriptor) = redeclarations.try_recv() {
            generation += 1;
            heartbeat.descriptor_generation = generation;
            info!(
                generation,
                node_id = descriptor.node_id,
                "end-effector provider redeclared"
            );
            context.descriptor = descriptor;
            next_descriptor = Instant::now();
            next_heartbeat = Instant::now();
        }
        let first = match outbound.recv_timeout(TICK) {
            Ok(message) => Some(message),
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };
        for message in first
            .into_iter()
            .chain(std::iter::from_fn(|| outbound.try_recv().ok()))
        {
            send_outbound(
                &ports,
                &context,
                generation,
                &mut seqs,
                &mut commands,
                &mut faults,
                message,
            );
        }

        let now = Instant::now();
        if now >= next_descriptor {
            next_descriptor = now + Duration::from_millis(ports::LATCHED_REPUBLISH_MS);
            match context.descriptor.to_wire(
                generation,
                model::header(context.instance_id, next(&mut seqs.descriptor)),
            ) {
                Ok(message) => {
                    if let Err(error) = ports.descriptor.send_copy(message) {
                        warn!(?error, "descriptor publish failed");
                    }
                }
                Err(error) => warn!(%error, "descriptor no longer encodes"),
            }
        }
        if now >= next_heartbeat {
            next_heartbeat = now + context.options.heartbeat_period;
            let message = heartbeat.to_wire(model::header(
                context.instance_id,
                next(&mut seqs.heartbeat),
            ));
            if let Err(error) = ports.heartbeat.send_copy(message) {
                warn!(?error, "heartbeat publish failed");
            }
        }

        receive_control(&ports, &context, &mut daemon, now);
        receive_joints(&ports, &context, generation);
        receive_commands(
            &ports,
            &context,
            generation,
            &mut seqs,
            &mut commands,
            &mut next_request_id,
        );
        receive_faults(
            &ports,
            &context,
            &mut seqs,
            &mut faults,
            &mut next_request_id,
        );
    }
    // Requests still parked are dropped with their ports: the daemon sees
    // the provider go away and reports it, rather than a silent success.
}

fn send_outbound(
    ports: &Ports,
    context: &PortContext,
    generation: u64,
    seqs: &mut Seqs,
    commands: &mut HashMap<u64, CommandRequest>,
    faults: &mut HashMap<u64, FaultRequest>,
    message: Outbound,
) {
    let instance = context.instance_id;
    match message {
        Outbound::State(mut state) => {
            // An array the descriptor does not declare is sent as NaN
            // throughout, as the contract requires of a provider.
            let capabilities = &context.descriptor.capabilities;
            if !capabilities.joint_velocity {
                state.velocity.clear();
            }
            if !capabilities.joint_effort {
                state.effort.clear();
            }
            match state.to_wire(generation, model::header(instance, next(&mut seqs.state))) {
                Ok(message) => {
                    if let Err(error) = ports.state.send_copy(message) {
                        warn!(?error, "state publish failed");
                    }
                }
                Err(error) => warn!(%error, "the provider's state does not encode; not sent"),
            }
        }
        Outbound::Tick(tick) => {
            match tick.to_wire(generation, model::header(instance, next(&mut seqs.tick))) {
                Ok(message) => {
                    if let Err(error) = ports.tick.send_copy(message) {
                        debug!(?error, "tick publish failed");
                    }
                }
                Err(error) => warn!(%error, "the provider's tick does not encode; not sent"),
            }
        }
        Outbound::CommandDone { id, outcome } => {
            if let Some(request) = commands.remove(&id) {
                let response = model::command_response(
                    &outcome,
                    model::header(instance, next(&mut seqs.command)),
                );
                if let Err(error) = request.send_copy(response) {
                    warn!(?error, "command response send failed");
                }
            }
        }
        Outbound::FaultDone { id, outcome } => {
            if let Some(request) = faults.remove(&id) {
                let response =
                    model::fault_response(&outcome, model::header(instance, next(&mut seqs.fault)));
                if let Err(error) = request.send_copy(response) {
                    warn!(?error, "fault response send failed");
                }
            }
        }
    }
}

fn receive_control(ports: &Ports, context: &PortContext, daemon: &mut Daemon, now: Instant) {
    loop {
        let sample = match ports.control.receive() {
            Ok(Some(sample)) => sample,
            Ok(None) => break,
            Err(error) => {
                warn!(?error, "control receive failed");
                break;
            }
        };
        let control = match Control::from_wire(sample.payload()) {
            Ok(control) => control,
            Err(error) => {
                warn!(%error, "control refused");
                continue;
            }
        };
        let sender = sample.payload().header.instance_id;
        daemon.last_seen = Some(now);
        if sender != daemon.instance_id {
            // A daemon seen for the first time (this provider started, or
            // the daemon restarted): its stop generation is where counting
            // starts, not a stop to act on.
            daemon.instance_id = sender;
            daemon.stop_generation = control.stop_generation;
            continue;
        }
        if control.stop_generation > daemon.stop_generation {
            daemon.stop_generation = control.stop_generation;
            let provider = Arc::clone(&context.provider);
            context.runtime.spawn(async move {
                if let Err(error) = provider.stop().await {
                    warn!(%error, "stop on the daemon's stop generation failed");
                }
            });
        }
    }
    let present = daemon
        .last_seen
        .is_some_and(|seen| now.duration_since(seen) <= context.options.daemon_stale_after);
    context.daemon_present.store(present, Ordering::Relaxed);
    if present != daemon.reported_present {
        daemon.reported_present = present;
        info!(present, "d1-firmwared presence changed");
        context.provider.daemon_presence(present);
    }
}

fn receive_joints(ports: &Ports, context: &PortContext, generation: u64) {
    let mut newest = None;
    loop {
        match ports.joints.receive() {
            Ok(Some(sample)) => match JointTargets::from_wire(sample.payload()) {
                Ok(targets) => newest = Some(targets),
                Err(error) => warn!(%error, "joint targets refused"),
            },
            Ok(None) => break,
            Err(error) => {
                warn!(?error, "joints receive failed");
                break;
            }
        }
    }
    let Some(targets) = newest else {
        return;
    };
    let age_ns = clock::monotonic_ns().saturating_sub(targets.issued_mono_ns);
    if targets.target_instance_id != context.instance_id
        || targets.descriptor_generation != generation
    {
        debug!("joint targets for another provider instance or generation dropped");
    } else if age_ns > ports::JOINTS_MAX_AGE_MS * 1_000_000 {
        debug!(age_ms = age_ns / 1_000_000, "stale joint targets dropped");
    } else if let Err(error) = check_joints(&context.descriptor, &targets.targets) {
        warn!(%error, "joint targets refused");
    } else {
        context.joints.send_replace(Some(targets));
    }
}

fn receive_commands(
    ports: &Ports,
    context: &PortContext,
    generation: u64,
    seqs: &mut Seqs,
    parked: &mut HashMap<u64, CommandRequest>,
    next_id: &mut u64,
) {
    loop {
        let request = match ports.command.receive() {
            Ok(Some(request)) => request,
            Ok(None) => break,
            Err(error) => {
                warn!(?error, "command receive failed");
                break;
            }
        };
        let outcome = match triage(context, generation, request.payload()) {
            Triage::Run(set) => {
                dispatch_command(context, parked, next_id, request, set);
                continue;
            }
            Triage::Answer(outcome) => outcome,
        };
        let response = model::command_response(
            &outcome,
            model::header(context.instance_id, next(&mut seqs.command)),
        );
        if let Err(error) = request.send_copy(response) {
            warn!(?error, "command response send failed");
        }
    }
}

/// What to do with one `command` request.
enum Triage {
    /// Run it on the provider: a `set` when given, a stop otherwise.
    Run(Option<model::SetCommand>),
    /// Answer it now without running it.
    Answer(CommandOutcome),
}

fn triage(context: &PortContext, generation: u64, message: &wire::CommandRequest) -> Triage {
    let stale =
        |message: &str| Triage::Answer(Err(ProviderError::new(ErrorKind::StaleTarget, message)));
    let envelope = match CommandEnvelope::from_wire(message) {
        Ok(envelope) => envelope,
        Err(error) => {
            return Triage::Answer(Err(ProviderError::new(
                ErrorKind::Invalid,
                error.to_string(),
            )));
        }
    };
    let set = match envelope.command {
        // A stop is executed whoever it was addressed to and however late:
        // stopping is never the wrong thing to do late.
        Command::Stop => return Triage::Run(None),
        Command::Set(set) => set,
    };
    if envelope.target_instance_id != context.instance_id {
        return stale("the request is for another provider instance");
    }
    if envelope.descriptor_generation != generation {
        return stale("the request was validated against another descriptor generation");
    }
    if clock::monotonic_ns() > envelope.start_by_mono_ns {
        return stale("the request arrived after its start deadline");
    }
    match check_set(&context.descriptor, &set) {
        Ok(()) => Triage::Run(Some(set)),
        Err(error) => Triage::Answer(Err(error)),
    }
}

/// Park a request and run it on the runtime: a `set` when `set` is given,
/// a stop otherwise.
fn dispatch_command(
    context: &PortContext,
    parked: &mut HashMap<u64, CommandRequest>,
    next_id: &mut u64,
    request: CommandRequest,
    set: Option<model::SetCommand>,
) {
    let id = next(next_id);
    parked.insert(id, request);
    let provider = Arc::clone(&context.provider);
    let outbound = context.outbound.clone();
    let settled = Arc::clone(&context.settled);
    context.runtime.spawn(async move {
        let outcome = match set {
            Some(set) => provider.set(set).await,
            None => provider.stop().await,
        };
        settled.fetch_add(1, Ordering::AcqRel);
        let _ = outbound.send(Outbound::State(settled_state(provider.as_ref()).await));
        let _ = outbound.send(Outbound::CommandDone { id, outcome });
    });
}

/// The state a command left, published before the command's answer
/// ([`crate::Provider::settled`]); a read that fails is published as a
/// state with no reading, as the periodic read publishes it.
async fn settled_state(provider: &dyn Provider) -> State {
    match tokio::time::timeout(READ_TIMEOUT, provider.settled()).await {
        Ok(Ok(state)) => state,
        Ok(Err(error)) => State {
            fault: Some(format!("state read failed: {}", error.message)),
            ..State::default()
        },
        Err(_) => State {
            fault: Some(format!("state read took longer than {READ_TIMEOUT:?}")),
            ..State::default()
        },
    }
}

fn receive_faults(
    ports: &Ports,
    context: &PortContext,
    seqs: &mut Seqs,
    parked: &mut HashMap<u64, FaultRequest>,
    next_id: &mut u64,
) {
    loop {
        let request = match ports.fault.receive() {
            Ok(Some(request)) => request,
            Ok(None) => break,
            Err(error) => {
                warn!(?error, "fault receive failed");
                break;
            }
        };
        let refusal = match FaultEnvelope::from_wire(request.payload()) {
            Err(error) => Some(ProviderError::new(ErrorKind::Invalid, error.to_string())),
            Ok(envelope) if envelope.target_instance_id != context.instance_id => {
                Some(ProviderError::new(
                    ErrorKind::StaleTarget,
                    "the request is for another provider instance",
                ))
            }
            Ok(envelope) => {
                let id = next(next_id);
                parked.insert(id, request);
                let provider = Arc::clone(&context.provider);
                let outbound = context.outbound.clone();
                let settled = Arc::clone(&context.settled);
                context.runtime.spawn(async move {
                    let outcome = match envelope.op {
                        FaultOp::ClearFault => {
                            let cleared = provider.clear_fault().await;
                            settled.fetch_add(1, Ordering::AcqRel);
                            let _ = outbound
                                .send(Outbound::State(settled_state(provider.as_ref()).await));
                            cleared.and_then(|clear| {
                                serde_json::to_value(clear).map_err(|error| {
                                    ProviderError::new(ErrorKind::Device, error.to_string())
                                })
                            })
                        }
                        FaultOp::Registers(ids) => provider.registers(ids).await.and_then(|dump| {
                            serde_json::to_value(dump).map_err(|error| {
                                ProviderError::new(ErrorKind::Device, error.to_string())
                            })
                        }),
                    };
                    let _ = outbound.send(Outbound::FaultDone { id, outcome });
                });
                continue;
            }
        };
        if let Some(error) = refusal {
            let response = model::fault_response(
                &Err(error),
                model::header(context.instance_id, next(&mut seqs.fault)),
            );
            if let Err(error) = request.send_copy(response) {
                warn!(?error, "fault response send failed");
            }
        }
    }
}
