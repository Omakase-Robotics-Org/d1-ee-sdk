//! The provider's side of `flange_bus.shm/1`: the arm side's flange CAN
//! channel, which the daemon relays.
//!
//! [`FlangeBus`] owns the four ports on a thread of its own (iceoryx2's
//! ports are not `Send`), publishes the heartbeat, and gives a device
//! driver a blocking `send` and `recv` it can call from any thread. Before a
//! frame is published, `send` applies what the relay's newest status says,
//! so a driver hears at once why a frame would be refused instead of
//! waiting out an answer that cannot come: not admitted, the channel down,
//! a motion frame under the soft-kill latch, a frame the channel does not
//! carry. It also keeps the contract's backpressure: at most
//! [`flange::TX_WINDOW`] frames beyond what the relay has taken, paced to
//! the relay's rate ceiling (a stop is never held back by the ceiling).

use std::{
    collections::VecDeque,
    sync::{
        Arc, Condvar, Mutex, MutexGuard,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    time::{Duration, Instant},
};

use d1fw_ee_shm_contract::{
    flange::{
        self, FlangeNames, TX_WINDOW,
        model::{self, Frame, Heartbeat, RxFrame, Status, TxFrame, TxKind},
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
use tracing::warn;

/// How long the port thread waits for a frame to send before it polls the
/// daemon's topics again: the most a received frame waits before a `recv`
/// can take it.
const POLL: Duration = Duration::from_micros(500);

/// How many received frames are kept for `recv`; past it the oldest is
/// dropped and counted.
const RX_QUEUE: usize = 1024;

/// How long a `send` waits for the window or the rate ceiling before it
/// gives up.
const SEND_WAIT: Duration = Duration::from_millis(100);

/// How long without a status before the daemon is taken to be gone.
const DAEMON_STALE: Duration = Duration::from_millis(500);

/// How the flange channel is opened.
#[derive(Debug, Clone)]
pub struct FlangeOptions {
    /// The shared-memory root; on a robot, [`ShmRoot::default`].
    pub root: ShmRoot,
    /// The side's flange service names (`omakase/flange/<side>`).
    pub names: FlangeNames,
    /// The iceoryx2 node name.
    pub node_name: String,
    /// How often the heartbeat is published; at most
    /// [`flange::MAX_HEARTBEAT_PERIOD_MS`].
    pub heartbeat_period: Duration,
}

impl FlangeOptions {
    /// The channel named `names` under `root`, with a heartbeat every 50 ms.
    #[must_use]
    pub fn new(root: ShmRoot, names: FlangeNames) -> Self {
        Self {
            root,
            names,
            node_name: "flange-provider".to_owned(),
            heartbeat_period: Duration::from_millis(50),
        }
    }
}

/// Why a frame was not sent or received.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlangeError {
    /// The relay does not take this provider's frames (yet): no status from
    /// the daemon, or one that binds another instance.
    NotAdmitted(String),
    /// The channel is down; the daemon's reason.
    ChannelDown(String),
    /// A motion frame while the soft-kill latch stands.
    Latched,
    /// A frame the contract or the channel does not carry.
    Invalid(String),
    /// The relay did not take frames fast enough for this one to be sent
    /// within the wait.
    Busy(String),
    /// The flange port thread has stopped.
    Closed,
}

impl std::fmt::Display for FlangeError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotAdmitted(why) => write!(formatter, "not admitted by the flange relay: {why}"),
            Self::ChannelDown(why) => write!(formatter, "the flange channel is down: {why}"),
            Self::Latched => formatter.write_str(
                "the soft-kill latch stands: the relay refuses a frame that can move the device",
            ),
            Self::Invalid(why) => write!(formatter, "the flange channel does not carry it: {why}"),
            Self::Busy(why) => write!(formatter, "the flange relay is busy: {why}"),
            Self::Closed => formatter.write_str("the flange port thread has stopped"),
        }
    }
}

impl std::error::Error for FlangeError {}

/// What this client counted.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FlangeStats {
    /// Frames published on `tx`.
    pub tx_published: u64,
    /// Received frames that never reached `recv`: gaps in the daemon's
    /// `rx` sequence, and frames dropped from a full queue.
    pub rx_lost: u64,
}

/// The rate ceiling's budget, kept like the relay keeps it.
struct Bucket {
    tokens: f64,
    rate: f64,
    capacity: f64,
    refilled: Instant,
}

impl Bucket {
    fn new() -> Self {
        Self {
            tokens: 0.0,
            rate: 0.0,
            capacity: 0.0,
            refilled: Instant::now(),
        }
    }

    /// Take a token at a ceiling of `max_tx_hz`, or say how long until one
    /// is there.
    fn take(&mut self, max_tx_hz: u32) -> Result<(), Duration> {
        let rate = f64::from(max_tx_hz.max(1));
        #[expect(
            clippy::float_cmp,
            reason = "the rate is set from the same integer each time"
        )]
        if rate != self.rate {
            self.rate = rate;
            self.capacity = (rate * 0.05).max(1.0);
            self.tokens = self.capacity;
        }
        let now = Instant::now();
        let elapsed = now.duration_since(self.refilled).as_secs_f64();
        self.refilled = now;
        self.tokens = elapsed.mul_add(self.rate, self.tokens).min(self.capacity);
        if self.tokens >= 1.0 {
            self.tokens -= 1.0;
            Ok(())
        } else {
            Err(Duration::from_secs_f64((1.0 - self.tokens) / self.rate))
        }
    }
}

/// What `send`, `recv` and the port thread share.
struct State {
    status: Option<(Status, Instant)>,
    rx: VecDeque<RxFrame>,
    /// The daemon instance and `seq` of the newest received frame.
    last_rx: Option<(u64, u64)>,
    tx_seq: u64,
    bucket: Bucket,
    stats: FlangeStats,
    closed: bool,
}

struct Inner {
    instance_id: u64,
    state: Mutex<State>,
    changed: Condvar,
    outbound: Mutex<mpsc::Sender<wire::TxFrame>>,
}

impl Inner {
    fn lock(&self) -> MutexGuard<'_, State> {
        self.state
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }
}

/// One side's flange channel, as its provider sees it. Dropping it stops
/// the port thread and closes the ports.
pub struct FlangeBus {
    inner: Arc<Inner>,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl std::fmt::Debug for FlangeBus {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("FlangeBus")
            .field("instance_id", &self.inner.instance_id)
            .finish_non_exhaustive()
    }
}

impl FlangeBus {
    /// Open the channel: the port thread creates its node and its four
    /// ports, and starts the heartbeat. Returns once every port exists.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for a heartbeat period the contract does
    /// not allow; [`ContractError::Transport`] when the root, the node or a
    /// port cannot be created — among others when another provider already
    /// holds the side (`tx` and `heartbeat` admit one publisher).
    pub fn open(options: FlangeOptions) -> Result<Self, ContractError> {
        if options.heartbeat_period.is_zero()
            || options.heartbeat_period > Duration::from_millis(flange::MAX_HEARTBEAT_PERIOD_MS)
        {
            return Err(ContractError::Invalid(format!(
                "heartbeat_period {:?} must be above zero and at most {} ms",
                options.heartbeat_period,
                flange::MAX_HEARTBEAT_PERIOD_MS
            )));
        }
        let (outbound, frames) = mpsc::channel();
        let inner = Arc::new(Inner {
            instance_id: d1fw_ee_shm_contract::model::new_instance_id(),
            state: Mutex::new(State {
                status: None,
                rx: VecDeque::new(),
                last_rx: None,
                tx_seq: 0,
                bucket: Bucket::new(),
                stats: FlangeStats::default(),
                closed: false,
            }),
            changed: Condvar::new(),
            outbound: Mutex::new(outbound),
        });
        let stop = Arc::new(AtomicBool::new(false));
        let (ready, opened) = mpsc::channel();
        let thread = {
            let inner = Arc::clone(&inner);
            let stop = Arc::clone(&stop);
            std::thread::Builder::new()
                .name("flange-provider-ports".to_owned())
                .spawn(move || run(&inner, &options, &frames, &stop, &ready))
                .map_err(|error| {
                    ContractError::Transport(format!("spawn the flange port thread: {error}"))
                })?
        };
        let mut bus = Self {
            inner,
            stop,
            thread: Some(thread),
        };
        match opened.recv() {
            Ok(Ok(())) => Ok(bus),
            Ok(Err(error)) => {
                bus.halt();
                Err(error)
            }
            Err(_) => {
                bus.halt();
                Err(ContractError::Transport(
                    "the flange port thread ended before it was ready".to_owned(),
                ))
            }
        }
    }

    /// This provider instance's id, as every message it sends carries it.
    #[must_use]
    pub fn instance_id(&self) -> u64 {
        self.inner.instance_id
    }

    /// The relay's newest status, when one arrived within the last 500 ms.
    #[must_use]
    pub fn status(&self) -> Option<Status> {
        let state = self.inner.lock();
        fresh(&state).cloned()
    }

    /// Whether the relay's newest status (within the last 500 ms) says the
    /// arm controller carrying the channel reports a fault on its side, so
    /// frames sent may not reach the device. Cheap enough to ask on every
    /// tick of a control loop: a lock and a flag, nothing cloned.
    #[must_use]
    pub fn arm_faulted(&self) -> bool {
        let state = self.inner.lock();
        fresh(&state).is_some_and(|status| status.arm_faulted)
    }

    /// What this client counted.
    #[must_use]
    pub fn stats(&self) -> FlangeStats {
        self.inner.lock().stats
    }

    /// Wait until the relay admits this provider (a fresh status binds it)
    /// with the channel up, and return that status.
    ///
    /// # Errors
    ///
    /// The reason it is not admitted when `timeout` passes first, or
    /// [`FlangeError::Closed`].
    pub fn wait_admitted(&self, timeout: Duration) -> Result<Status, FlangeError> {
        let deadline = Instant::now() + timeout;
        let mut state = self.inner.lock();
        loop {
            let refusal = match admission(&state, self.inner.instance_id) {
                Ok(status) => return Ok(status.clone()),
                Err(refusal) => refusal,
            };
            let now = Instant::now();
            if now >= deadline {
                return Err(refusal);
            }
            state = self
                .inner
                .changed
                .wait_timeout(state, (deadline - now).min(Duration::from_millis(50)))
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .0;
        }
    }

    /// Publish `frame` for the relay to put on the channel, `kind` saying
    /// what it does to the device. Returns once it is published: the relay
    /// sends it within about a millisecond, or refuses and counts it.
    ///
    /// # Errors
    ///
    /// [`FlangeError::Invalid`] for a frame the contract or the channel does
    /// not carry; [`FlangeError::NotAdmitted`], [`FlangeError::ChannelDown`]
    /// and [`FlangeError::Latched`] from the relay's newest status;
    /// [`FlangeError::Busy`] when the window or the rate ceiling does not
    /// open within 100 ms; [`FlangeError::Closed`].
    pub fn send(&self, frame: &Frame, kind: TxKind) -> Result<(), FlangeError> {
        frame
            .validate()
            .map_err(|error| FlangeError::Invalid(error.to_string()))?;
        let deadline = Instant::now() + SEND_WAIT;
        let mut state = self.inner.lock();
        loop {
            let status = admission(&state, self.inner.instance_id)?.clone();
            if kind == TxKind::Motion && status.latched {
                return Err(FlangeError::Latched);
            }
            if frame.data.len() > usize::try_from(status.channel.max_frame_len).unwrap_or(0) {
                return Err(FlangeError::Invalid(format!(
                    "a frame of {} bytes; the channel carries at most {}",
                    frame.data.len(),
                    status.channel.max_frame_len
                )));
            }
            if frame.extended && !status.channel.extended_ids {
                return Err(FlangeError::Invalid(
                    "a 29-bit identifier; the channel carries 11-bit ones only".to_owned(),
                ));
            }
            let wait = if state.tx_seq.saturating_sub(status.tx_seq_seen) >= TX_WINDOW {
                Some((
                    Duration::from_millis(1),
                    "the relay has not taken the frames already published",
                ))
            } else if kind == TxKind::Stop {
                None
            } else {
                state
                    .bucket
                    .take(status.max_tx_hz)
                    .err()
                    .map(|wait| (wait, "the side's tx rate ceiling is reached"))
            };
            let Some((wait, why)) = wait else {
                break;
            };
            let now = Instant::now();
            if now >= deadline {
                return Err(FlangeError::Busy(why.to_owned()));
            }
            state = self
                .inner
                .changed
                .wait_timeout(state, wait.min(deadline - now))
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .0;
        }
        state.tx_seq += 1;
        let message = TxFrame::to_wire(
            frame,
            kind,
            model::header(self.inner.instance_id, state.tx_seq),
        )
        .map_err(|error| FlangeError::Invalid(error.to_string()))?;
        state.stats.tx_published += 1;
        drop(state);
        self.inner
            .outbound
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .send(message)
            .map_err(|_| FlangeError::Closed)
    }

    /// Wait until the relay has taken every frame this provider published
    /// (sent it to the channel, or refused and counted it).
    ///
    /// # Errors
    ///
    /// [`FlangeError::Busy`] when it has not within `timeout`; the
    /// admission errors of [`Self::send`]; [`FlangeError::Closed`].
    pub fn flush(&self, timeout: Duration) -> Result<(), FlangeError> {
        let deadline = Instant::now() + timeout;
        let mut state = self.inner.lock();
        loop {
            let seen = admission(&state, self.inner.instance_id)?.tx_seq_seen;
            if seen >= state.tx_seq {
                return Ok(());
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(FlangeError::Busy(format!(
                    "the relay has taken frames up to {seen} of {} within {} ms",
                    state.tx_seq,
                    timeout.as_millis()
                )));
            }
            state = self
                .inner
                .changed
                .wait_timeout(state, deadline - now)
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .0;
        }
    }

    /// The next received frame, or `None` when none arrives within
    /// `timeout`.
    ///
    /// # Errors
    ///
    /// [`FlangeError::Closed`].
    pub fn recv(&self, timeout: Duration) -> Result<Option<RxFrame>, FlangeError> {
        let deadline = Instant::now() + timeout;
        let mut state = self.inner.lock();
        loop {
            if let Some(frame) = state.rx.pop_front() {
                return Ok(Some(frame));
            }
            if state.closed {
                return Err(FlangeError::Closed);
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            state = self
                .inner
                .changed
                .wait_timeout(state, deadline - now)
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .0;
        }
    }

    fn halt(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for FlangeBus {
    fn drop(&mut self) {
        self.halt();
    }
}

fn fresh(state: &State) -> Option<&Status> {
    state
        .status
        .as_ref()
        .filter(|(_, at)| at.elapsed() <= DAEMON_STALE)
        .map(|(status, _)| status)
}

/// The newest status when it admits `instance` with the channel up, or why
/// not.
fn admission(state: &State, instance: u64) -> Result<&Status, FlangeError> {
    if state.closed {
        return Err(FlangeError::Closed);
    }
    let Some(status) = fresh(state) else {
        return Err(FlangeError::NotAdmitted(format!(
            "no status from d1-firmwared for {} ms (is the side declared with flange_bus = true?)",
            DAEMON_STALE.as_millis()
        )));
    };
    if status.bound_instance_id != Some(instance) {
        return Err(FlangeError::NotAdmitted(
            "the relay has not bound this provider's heartbeat yet".to_owned(),
        ));
    }
    if !status.channel_up {
        return Err(FlangeError::ChannelDown(
            status
                .channel_error
                .clone()
                .unwrap_or_else(|| "no reason given".to_owned()),
        ));
    }
    Ok(status)
}

/// Every port of the provider's side, and the node they belong to.
struct Ports {
    tx: Publisher<Ipc, wire::TxFrame, ()>,
    heartbeat: Publisher<Ipc, wire::Heartbeat, ()>,
    rx: Subscriber<Ipc, wire::RxFrame, ()>,
    status: Subscriber<Ipc, wire::Status, ()>,
    // Declared last: fields drop in order, and the node must outlive its
    // ports.
    _node: Node<Ipc>,
}

fn transport(what: &str, error: impl std::fmt::Debug) -> ContractError {
    ContractError::Transport(format!("{what}: {error:?}"))
}

impl Ports {
    fn open(options: &FlangeOptions) -> Result<Self, ContractError> {
        let node = options.root.node(&options.node_name)?;
        let names = &options.names;
        let tx = ports::open_topic::<wire::TxFrame>(
            &node,
            &names.name(flange::TX),
            flange::TX_TOPIC,
        )?
        .publisher_builder()
        .create()
        .map_err(|error| {
            transport(
                &format!(
                    "create the tx publisher of {} (is another provider serving this side?)",
                    names.base()
                ),
                error,
            )
        })?;
        let heartbeat = ports::open_topic::<wire::Heartbeat>(
            &node,
            &names.name(flange::HEARTBEAT),
            flange::HEARTBEAT_TOPIC,
        )?
        .publisher_builder()
        .create()
        .map_err(|error| transport("create the heartbeat publisher", error))?;
        let rx =
            ports::open_topic::<wire::RxFrame>(&node, &names.name(flange::RX), flange::RX_TOPIC)?
                .subscriber_builder()
                .create()
                .map_err(|error| transport("create the rx subscriber", error))?;
        let status = ports::open_topic::<wire::Status>(
            &node,
            &names.name(flange::STATUS),
            flange::STATUS_TOPIC,
        )?
        .subscriber_builder()
        .create()
        .map_err(|error| transport("create the status subscriber", error))?;
        Ok(Self {
            tx,
            heartbeat,
            rx,
            status,
            _node: node,
        })
    }
}

fn run(
    inner: &Inner,
    options: &FlangeOptions,
    frames: &mpsc::Receiver<wire::TxFrame>,
    stop: &AtomicBool,
    ready: &mpsc::Sender<Result<(), ContractError>>,
) {
    let ports = match Ports::open(options) {
        Ok(ports) => {
            let _ = ready.send(Ok(()));
            ports
        }
        Err(error) => {
            let _ = ready.send(Err(error));
            return;
        }
    };
    let period_ms = u32::try_from(options.heartbeat_period.as_millis()).unwrap_or(u32::MAX);
    let mut heartbeat_seq = 0_u64;
    let mut next_heartbeat = Instant::now();
    while !stop.load(Ordering::Relaxed) {
        let first = match frames.recv_timeout(POLL) {
            Ok(frame) => Some(frame),
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };
        for frame in first
            .into_iter()
            .chain(std::iter::from_fn(|| frames.try_recv().ok()))
        {
            if let Err(error) = ports.tx.send_copy(frame) {
                warn!(?error, "tx publish failed");
            }
        }
        let now = Instant::now();
        if now >= next_heartbeat {
            next_heartbeat = now + options.heartbeat_period;
            heartbeat_seq += 1;
            let message =
                Heartbeat::to_wire(period_ms, model::header(inner.instance_id, heartbeat_seq));
            if let Err(error) = ports.heartbeat.send_copy(message) {
                warn!(?error, "heartbeat publish failed");
            }
        }
        if receive(inner, &ports) {
            inner.changed.notify_all();
        }
    }
    inner.lock().closed = true;
    inner.changed.notify_all();
}

/// Take what the daemon published; `true` when something changed.
fn receive(inner: &Inner, ports: &Ports) -> bool {
    let mut changed = false;
    let mut newest_status = None;
    loop {
        match ports.status.receive() {
            Ok(Some(sample)) => match Status::from_wire(sample.payload()) {
                Ok(status) => newest_status = Some(status),
                Err(error) => warn!(%error, "status refused"),
            },
            Ok(None) => break,
            Err(error) => {
                warn!(?error, "status receive failed");
                break;
            }
        }
    }
    let mut received = Vec::new();
    loop {
        match ports.rx.receive() {
            Ok(Some(sample)) => match RxFrame::from_wire(sample.payload()) {
                Ok(frame) => received.push(frame),
                Err(error) => warn!(%error, "received frame refused"),
            },
            Ok(None) => break,
            Err(error) => {
                warn!(?error, "rx receive failed");
                break;
            }
        }
    }
    if newest_status.is_none() && received.is_empty() {
        return false;
    }
    let mut state = inner.lock();
    if let Some(status) = newest_status {
        state.status = Some((status, Instant::now()));
        changed = true;
    }
    for frame in received {
        if let Some((daemon, seq)) = state.last_rx
            && daemon == frame.daemon_instance_id
            && frame.seq > seq + 1
        {
            state.stats.rx_lost += frame.seq - seq - 1;
        }
        state.last_rx = Some((frame.daemon_instance_id, frame.seq));
        if state.rx.len() >= RX_QUEUE {
            state.rx.pop_front();
            state.stats.rx_lost += 1;
        }
        state.rx.push_back(frame);
        changed = true;
    }
    changed
}
