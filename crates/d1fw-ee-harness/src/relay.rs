//! The daemon's side of `flange_bus.shm/1` (`spec/flange_bus.shm/1.md` §5):
//! one arm side's flange CAN channel, relayed to the provider that drives
//! the device on it, under the same gates d1-firmwared applies — the bound
//! provider's heartbeat, the frame size, the tx age limit, the soft-kill
//! latch for a motion frame, and the rate ceiling.
//!
//! Two threads: one blocks on the channel's receive and hands every frame
//! to the other, which owns the iceoryx2 ports (they are not `Send`),
//! publishes the frames on `rx`, takes the provider's `tx` frames through
//! the gates, and publishes `status`.

use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    time::{Duration, Instant},
};

use d1fw_canfd::{CanFdBus, CanFdFrame, FrameKind};
use d1fw_ee_shm_contract::{
    clock,
    flange::{
        self, FlangeNames, MAX_FRAME_LEN, TX_MAX_AGE_MS,
        model::{self, ChannelFacts, Counters, Frame, Heartbeat, RxFrame, Status, TxFrame, TxKind},
        wire,
    },
    model::ContractError,
    ports::{self, Ipc},
    root::ShmRoot,
};
use iceoryx2::{
    port::{publisher::Publisher, subscriber::Subscriber},
    prelude::Node,
};
use tracing::{info, warn};

/// How long the port thread waits for a received frame before it polls the
/// provider's topics again: the most a tx frame waits before it is taken.
const TICK: Duration = Duration::from_millis(1);

/// How long the receive thread blocks on the channel per call.
const RX_WAIT: Duration = Duration::from_millis(5);

/// How long the receive thread pauses after a receive that returned at
/// once with nothing (a transport that does not wait, such as a simulator).
const RX_IDLE: Duration = Duration::from_micros(250);

/// How long the receive thread waits after the channel failed before it
/// tries again.
const RX_BACKOFF: Duration = Duration::from_millis(100);

/// How long the port thread waits before it tries again to open services it
/// could not open.
const REOPEN: Duration = Duration::from_secs(1);

/// How much of the rate ceiling's budget may be spent at once: 50 ms of
/// frames.
const BURST: Duration = Duration::from_millis(50);

/// One side's flange relay.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlangeSettings {
    /// The side's flange service names (`omakase/flange/<side>`).
    pub names: FlangeNames,
    /// The tx rate ceiling, frames per second.
    pub max_tx_hz: u32,
}

impl FlangeSettings {
    /// The relay of `side` under its default names and ceiling.
    ///
    /// # Panics
    ///
    /// Never: the default base is a valid name.
    #[must_use]
    pub fn new(side: &str) -> Self {
        Self {
            names: FlangeNames::new(&flange::default_base(side))
                .expect("the default flange base is a valid name"),
            max_tx_hz: flange::DEFAULT_MAX_TX_HZ,
        }
    }
}

/// What the receive thread hands the port thread.
enum Received {
    Frame { frame: CanFdFrame, mono_ns: u64 },
    Failed(String),
}

/// What the port thread and the driver share.
struct Shared {
    side: String,
    root: ShmRoot,
    settings: FlangeSettings,
    stale_after: Duration,
    instance_id: u64,
    bus: Arc<dyn CanFdBus>,
    /// The soft-kill latch: while it stands, every motion frame is refused.
    latched: AtomicBool,
    /// The status bit `arm_faulted`.
    arm_faulted: AtomicBool,
}

impl Shared {
    fn latched(&self) -> bool {
        self.latched.load(Ordering::Relaxed)
    }

    fn arm_faulted(&self) -> bool {
        self.arm_faulted.load(Ordering::Relaxed)
    }

    fn base(&self) -> &str {
        self.settings.names.base()
    }
}

/// One side's running relay. Dropping it stops both threads and closes its
/// ports.
pub(crate) struct FlangeRelay {
    shared: Arc<Shared>,
    stop: Arc<AtomicBool>,
    threads: Vec<std::thread::JoinHandle<()>>,
}

impl FlangeRelay {
    /// Relay `bus` for `side` under `root`, admitting a provider whose
    /// heartbeat is younger than `stale_after`.
    pub(crate) fn start(
        side: String,
        root: ShmRoot,
        settings: FlangeSettings,
        stale_after: Duration,
        bus: Arc<dyn CanFdBus>,
    ) -> Self {
        let shared = Arc::new(Shared {
            side,
            root,
            settings,
            stale_after,
            instance_id: d1fw_ee_shm_contract::model::new_instance_id(),
            bus,
            latched: AtomicBool::new(false),
            arm_faulted: AtomicBool::new(false),
        });
        let stop = Arc::new(AtomicBool::new(false));
        let (received, inbound) = mpsc::channel();
        let receiver = {
            let shared = Arc::clone(&shared);
            let stop = Arc::clone(&stop);
            std::thread::Builder::new()
                .name(format!("flange-rx-{}", shared.side))
                .spawn(move || receive_channel(&shared, &received, &stop))
                .expect("spawn the flange receive thread")
        };
        let ports = {
            let shared = Arc::clone(&shared);
            let stop = Arc::clone(&stop);
            std::thread::Builder::new()
                .name(format!("flange-{}", shared.side))
                .spawn(move || run(&shared, &inbound, &stop))
                .expect("spawn the flange relay thread")
        };
        info!(
            side = shared.side.as_str(),
            contract = flange::CONTRACT,
            service = shared.base(),
            max_tx_hz = shared.settings.max_tx_hz,
            "flange relay started"
        );
        Self {
            shared,
            stop,
            threads: vec![receiver, ports],
        }
    }

    /// Raise or release the soft-kill latch.
    pub(crate) fn latch(&self, latched: bool) {
        self.shared.latched.store(latched, Ordering::Relaxed);
    }

    /// Set or clear the status bit `arm_faulted`.
    pub(crate) fn set_arm_faulted(&self, faulted: bool) {
        self.shared.arm_faulted.store(faulted, Ordering::Relaxed);
    }
}

impl Drop for FlangeRelay {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        for thread in self.threads.drain(..) {
            let _ = thread.join();
        }
    }
}

fn receive_channel(shared: &Shared, received: &mpsc::Sender<Received>, stop: &AtomicBool) {
    while !stop.load(Ordering::Relaxed) {
        let started = Instant::now();
        match shared.bus.recv(RX_WAIT) {
            Ok(Some(frame)) => {
                let mono_ns = clock::monotonic_ns();
                if received.send(Received::Frame { frame, mono_ns }).is_err() {
                    return;
                }
            }
            Ok(None) => {
                if started.elapsed() < RX_IDLE {
                    std::thread::sleep(RX_IDLE);
                }
            }
            Err(error) => {
                if received.send(Received::Failed(error.to_string())).is_err() {
                    return;
                }
                std::thread::sleep(RX_BACKOFF);
            }
        }
    }
}

/// Every port of the relay, and the node they belong to.
struct Ports {
    rx: Publisher<Ipc, wire::RxFrame, ()>,
    status: Publisher<Ipc, wire::Status, ()>,
    tx: Subscriber<Ipc, wire::TxFrame, ()>,
    heartbeat: Subscriber<Ipc, wire::Heartbeat, ()>,
    // Declared last: fields drop in order, and the node must outlive its
    // ports.
    _node: Node<Ipc>,
}

fn transport(what: &str, error: impl std::fmt::Debug) -> ContractError {
    ContractError::Transport(format!("{what}: {error:?}"))
}

impl Ports {
    fn open(shared: &Shared) -> Result<Self, ContractError> {
        let node = shared
            .root
            .node(&format!("ee-host-flange-{}", shared.side))?;
        let names = &shared.settings.names;
        let rx =
            ports::open_topic::<wire::RxFrame>(&node, &names.name(flange::RX), flange::RX_TOPIC)?
                .publisher_builder()
                .create()
                .map_err(|error| {
                    transport(
                        &format!(
                            "create the rx publisher of {} (does another host relay this side?)",
                            names.base()
                        ),
                        error,
                    )
                })?;
        let status = ports::open_topic::<wire::Status>(
            &node,
            &names.name(flange::STATUS),
            flange::STATUS_TOPIC,
        )?
        .publisher_builder()
        .create()
        .map_err(|error| transport("create the status publisher", error))?;
        let tx =
            ports::open_topic::<wire::TxFrame>(&node, &names.name(flange::TX), flange::TX_TOPIC)?
                .subscriber_builder()
                .create()
                .map_err(|error| transport("create the tx subscriber", error))?;
        let heartbeat = ports::open_topic::<wire::Heartbeat>(
            &node,
            &names.name(flange::HEARTBEAT),
            flange::HEARTBEAT_TOPIC,
        )?
        .subscriber_builder()
        .create()
        .map_err(|error| transport("create the heartbeat subscriber", error))?;
        Ok(Self {
            rx,
            status,
            tx,
            heartbeat,
            _node: node,
        })
    }
}

/// The provider the relay takes frames from.
struct Binding {
    instance_id: u64,
    /// Its newest heartbeat's `mono_ns`.
    heartbeat_mono_ns: u64,
    /// The newest tx `seq` taken from it.
    tx_seq_seen: u64,
}

/// The rate ceiling's budget.
struct Bucket {
    tokens: f64,
    capacity: f64,
    rate: f64,
    refilled: Instant,
}

impl Bucket {
    fn new(max_tx_hz: u32) -> Self {
        let rate = f64::from(max_tx_hz.max(1));
        let capacity = (rate * BURST.as_secs_f64()).max(1.0);
        Self {
            tokens: capacity,
            capacity,
            rate,
            refilled: Instant::now(),
        }
    }

    fn take(&mut self) -> bool {
        let now = Instant::now();
        let elapsed = now.duration_since(self.refilled).as_secs_f64();
        self.refilled = now;
        self.tokens = elapsed.mul_add(self.rate, self.tokens).min(self.capacity);
        if self.tokens >= 1.0 {
            self.tokens -= 1.0;
            true
        } else {
            false
        }
    }
}

/// Why a tx frame was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Refusal {
    Invalid,
    Unbound,
    Stale,
    Latched,
    Rate,
}

impl Refusal {
    const fn reason(self) -> &'static str {
        match self {
            Self::Invalid => "malformed or unsupported by the channel",
            Self::Unbound => "from a provider instance the relay has not bound",
            Self::Stale => "older than the contract's tx age limit",
            Self::Latched => "a motion frame while the soft-kill latch stands",
            Self::Rate => "beyond the side's tx rate ceiling",
        }
    }
}

/// The relay thread's bookkeeping.
struct Relay {
    ports: Option<Ports>,
    next_open: Instant,
    binding: Option<Binding>,
    counters: Counters,
    channel_up: bool,
    channel_error: Option<String>,
    bucket: Bucket,
    rx_seq: u64,
    status_seq: u64,
    next_status: Instant,
    /// What the newest published status said (binding, `tx_seq_seen`,
    /// latch, arm fault, channel), to publish again on a change.
    published: Option<(Option<u64>, u64, bool, bool, bool)>,
    /// The refusal reasons already logged for the bound instance, so a
    /// provider that keeps sending a refused frame is reported once.
    logged: Vec<Refusal>,
}

fn run(shared: &Shared, inbound: &mpsc::Receiver<Received>, stop: &AtomicBool) {
    let mut relay = Relay {
        ports: None,
        next_open: Instant::now(),
        binding: None,
        counters: Counters::default(),
        channel_up: true,
        channel_error: None,
        bucket: Bucket::new(shared.settings.max_tx_hz),
        rx_seq: 0,
        status_seq: 0,
        next_status: Instant::now(),
        published: None,
        logged: Vec::new(),
    };
    while !stop.load(Ordering::Relaxed) {
        if relay.ports.is_none() && Instant::now() >= relay.next_open {
            open(shared, &mut relay);
        }
        let first = match inbound.recv_timeout(TICK) {
            Ok(received) => Some(received),
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };
        for received in first
            .into_iter()
            .chain(std::iter::from_fn(|| inbound.try_recv().ok()))
        {
            match received {
                Received::Frame { frame, mono_ns } => {
                    publish_rx(shared, &mut relay, &frame, mono_ns)
                }
                Received::Failed(error) => channel_failed(shared, &mut relay, error),
            }
        }
        if relay.ports.is_some() {
            receive_heartbeats(shared, &mut relay);
            receive_tx(shared, &mut relay);
            publish_status(shared, &mut relay);
        }
    }
}

fn open(shared: &Shared, relay: &mut Relay) {
    match Ports::open(shared) {
        Ok(ports) => {
            info!(
                side = shared.side.as_str(),
                service = shared.base(),
                root = shared.root.root_path(),
                prefix = shared.root.prefix(),
                "flange relay services open"
            );
            relay.ports = Some(ports);
        }
        Err(error) => {
            warn!(
                side = shared.side.as_str(),
                service = shared.base(),
                %error,
                "flange relay services cannot be opened; retrying"
            );
            relay.next_open = Instant::now() + REOPEN;
        }
    }
}

fn publish_rx(shared: &Shared, relay: &mut Relay, frame: &CanFdFrame, mono_ns: u64) {
    if !relay.channel_up && relay.channel_error.is_some() {
        relay.channel_up = true;
        relay.channel_error = None;
    }
    let Some(ports) = &relay.ports else {
        return;
    };
    let frame = Frame::standard(frame.id, frame.data.clone());
    relay.rx_seq += 1;
    match RxFrame::to_wire(
        &frame,
        mono_ns,
        model::header(shared.instance_id, relay.rx_seq),
    ) {
        Ok(message) => {
            if let Err(error) = ports.rx.send_copy(message) {
                warn!(?error, service = shared.base(), "rx publish failed");
            } else {
                relay.counters.rx_frames += 1;
            }
        }
        Err(error) => warn!(%error, service = shared.base(), "a received frame does not encode"),
    }
}

fn channel_failed(shared: &Shared, relay: &mut Relay, error: String) {
    if relay.channel_up {
        warn!(
            side = shared.side.as_str(),
            %error,
            "the flange channel failed"
        );
    }
    relay.channel_up = false;
    relay.channel_error = Some(error);
}

fn receive_heartbeats(shared: &Shared, relay: &mut Relay) {
    let Some(ports) = &relay.ports else {
        return;
    };
    let mut newest: Option<Heartbeat> = None;
    loop {
        match ports.heartbeat.receive() {
            Ok(Some(sample)) => match Heartbeat::from_wire(sample.payload()) {
                Ok(heartbeat) => newest = Some(heartbeat),
                Err(error) => warn!(%error, service = shared.base(), "heartbeat refused"),
            },
            Ok(None) => break,
            Err(error) => {
                warn!(?error, service = shared.base(), "heartbeat receive failed");
                break;
            }
        }
    }
    let now = clock::monotonic_ns();
    let stale_ns = u64::try_from(shared.stale_after.as_nanos()).unwrap_or(u64::MAX);
    if let Some(heartbeat) = newest
        && now.saturating_sub(heartbeat.mono_ns) <= stale_ns
    {
        match &mut relay.binding {
            Some(binding) if binding.instance_id == heartbeat.provider_instance_id => {
                binding.heartbeat_mono_ns = heartbeat.mono_ns;
            }
            _ => {
                info!(
                    side = shared.side.as_str(),
                    provider_instance_id = heartbeat.provider_instance_id,
                    "flange relay bound a provider"
                );
                relay.binding = Some(Binding {
                    instance_id: heartbeat.provider_instance_id,
                    heartbeat_mono_ns: heartbeat.mono_ns,
                    tx_seq_seen: 0,
                });
                relay.logged.clear();
            }
        }
    }
    if let Some(binding) = &relay.binding
        && now.saturating_sub(binding.heartbeat_mono_ns) > stale_ns
    {
        warn!(
            side = shared.side.as_str(),
            provider_instance_id = binding.instance_id,
            stale_after_ms = shared.stale_after.as_millis(),
            "flange relay unbound a provider whose heartbeat went stale"
        );
        relay.binding = None;
    }
}

fn receive_tx(shared: &Shared, relay: &mut Relay) {
    loop {
        let Some(ports) = &relay.ports else {
            return;
        };
        let message = match ports.tx.receive() {
            Ok(Some(sample)) => *sample.payload(),
            Ok(None) => return,
            Err(error) => {
                warn!(?error, service = shared.base(), "tx receive failed");
                return;
            }
        };
        take_tx(shared, relay, &message);
    }
}

fn take_tx(shared: &Shared, relay: &mut Relay, message: &wire::TxFrame) {
    let sender = message.header.instance_id;
    let bound = relay
        .binding
        .as_mut()
        .filter(|binding| binding.instance_id == sender);
    if let Some(binding) = bound {
        let seq = message.header.seq;
        if binding.tx_seq_seen != 0 && seq > binding.tx_seq_seen + 1 {
            relay.counters.tx_lost += seq - binding.tx_seq_seen - 1;
        }
        binding.tx_seq_seen = binding.tx_seq_seen.max(seq);
    }
    match gate(shared, relay, message) {
        Ok((frame, kind)) => send(shared, relay, &frame, kind),
        Err(refusal) => refuse(shared, relay, refusal),
    }
}

fn gate(
    shared: &Shared,
    relay: &mut Relay,
    message: &wire::TxFrame,
) -> Result<(CanFdFrame, TxKind), Refusal> {
    let decoded = TxFrame::from_wire(message).map_err(|_| Refusal::Invalid)?;
    // The channel is a `CanFdBus`: 11-bit identifiers and at most
    // `d1fw_canfd::MAX_PAYLOAD` bytes.
    if decoded.frame.extended || decoded.frame.data.len() > d1fw_canfd::MAX_PAYLOAD {
        return Err(Refusal::Invalid);
    }
    let id = u16::try_from(decoded.frame.id).map_err(|_| Refusal::Invalid)?;
    if relay
        .binding
        .as_ref()
        .is_none_or(|binding| binding.instance_id != decoded.provider_instance_id)
    {
        return Err(Refusal::Unbound);
    }
    let age_ns = clock::monotonic_ns().saturating_sub(decoded.mono_ns);
    if age_ns > TX_MAX_AGE_MS * 1_000_000 {
        return Err(Refusal::Stale);
    }
    if decoded.kind == TxKind::Motion && shared.latched() {
        return Err(Refusal::Latched);
    }
    if decoded.kind != TxKind::Stop && !relay.bucket.take() {
        return Err(Refusal::Rate);
    }
    Ok((CanFdFrame::new(id, decoded.frame.data), decoded.kind))
}

fn refuse(shared: &Shared, relay: &mut Relay, refusal: Refusal) {
    let counters = &mut relay.counters;
    match refusal {
        Refusal::Invalid => counters.tx_refused_invalid += 1,
        Refusal::Unbound => counters.tx_refused_unbound += 1,
        Refusal::Stale => counters.tx_refused_stale += 1,
        Refusal::Latched => counters.tx_refused_latched += 1,
        Refusal::Rate => counters.tx_refused_rate += 1,
    }
    if !relay.logged.contains(&refusal) {
        relay.logged.push(refusal);
        warn!(
            side = shared.side.as_str(),
            reason = refusal.reason(),
            "flange relay refused a tx frame; further refusals for this reason are counted in \
             its status only"
        );
    }
}

fn send(shared: &Shared, relay: &mut Relay, frame: &CanFdFrame, kind: TxKind) {
    let kind = match kind {
        TxKind::Motion => FrameKind::Motion,
        TxKind::Read => FrameKind::Read,
        TxKind::Stop => FrameKind::Stop,
    };
    match shared.bus.send_as(frame, kind) {
        Ok(()) => {
            relay.counters.tx_forwarded += 1;
            if !relay.channel_up {
                info!(
                    side = shared.side.as_str(),
                    "the flange channel is up again"
                );
            }
            relay.channel_up = true;
            relay.channel_error = None;
        }
        Err(error) => {
            relay.counters.tx_failed += 1;
            channel_failed(shared, relay, error.to_string());
        }
    }
}

fn publish_status(shared: &Shared, relay: &mut Relay) {
    let latched = shared.latched();
    let arm_faulted = shared.arm_faulted();
    let bound = relay.binding.as_ref().map(|binding| binding.instance_id);
    let seen = relay
        .binding
        .as_ref()
        .map_or(0, |binding| binding.tx_seq_seen);
    let summary = (bound, seen, latched, arm_faulted, relay.channel_up);
    let now = Instant::now();
    if relay.published == Some(summary) && now < relay.next_status {
        return;
    }
    let Some(ports) = &relay.ports else {
        return;
    };
    relay.status_seq += 1;
    let status = Status {
        daemon_instance_id: shared.instance_id,
        mono_ns: 0,
        bound_instance_id: bound,
        tx_seq_seen: seen,
        channel_up: relay.channel_up,
        latched,
        arm_faulted,
        channel: ChannelFacts {
            suppresses_repeats: shared.bus.suppresses_repeat_frames(),
            extended_ids: false,
            max_frame_len: u32::try_from(MAX_FRAME_LEN.min(d1fw_canfd::MAX_PAYLOAD))
                .unwrap_or(u32::MAX),
        },
        max_tx_hz: shared.settings.max_tx_hz,
        period_ms: u32::try_from(flange::STATUS_PERIOD_MS).unwrap_or(u32::MAX),
        channel_error: relay.channel_error.clone(),
        counters: relay.counters,
    };
    if let Err(error) = ports
        .status
        .send_copy(status.to_wire(model::header(shared.instance_id, relay.status_seq)))
    {
        warn!(?error, service = shared.base(), "status publish failed");
    }
    relay.published = Some(summary);
    relay.next_status = now + Duration::from_millis(flange::STATUS_PERIOD_MS);
}
