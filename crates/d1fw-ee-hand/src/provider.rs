//! A hand as an `end_effector.shm/1` provider: the facts a hand model
//! declares, the link its device crate implements, and [`HandProvider`],
//! which serves any such link.

use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

use async_trait::async_trait;
use d1fw_ee_provider::{
    Capabilities, Descriptor, Joint, JointUnit, Provider, ProviderError, SetCommand, State, Tool,
};

use crate::{
    link::{CommandsWaiting, ReadError, SerializedLink, StaleCache, validate_fractions},
    state::{HAND_STATE, HandSample, HandState},
};

/// An arm side, as the provider's command line names it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    /// Arm A, the physical LEFT arm.
    A,
    /// Arm B, the physical RIGHT arm.
    B,
}

impl Side {
    /// `"a"` or `"b"`.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::A => "a",
            Self::B => "b",
        }
    }
}

impl std::str::FromStr for Side {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        match text {
            "a" => Ok(Self::A),
            "b" => Ok(Self::B),
            other => Err(format!("a side is \"a\" or \"b\", not {other:?}")),
        }
    }
}

/// What a hand model declares.
#[derive(Debug, Clone)]
pub struct HandFacts {
    /// The driver name the side's descriptor carries.
    pub driver: &'static str,
    /// The model id, `"<maker>/<model>"`.
    pub model: &'static str,
    /// The axes in wire order: the joints of the `joints` mode, each a
    /// fraction `0.0` … `1.0` of its wire range.
    pub axes: &'static [&'static str],
    /// The vendor-rated command ceiling, when one is published.
    pub max_command_hz: Option<u32>,
    /// The capability tokens its state repeats.
    pub features: &'static [&'static str],
    /// What the arm controller compensates for while the hand is on the
    /// flange.
    pub tool: Tool,
    /// The openness sampled positions (fractions, wire order) stand for.
    pub openness: fn(&[f64]) -> Option<f64>,
}

impl HandFacts {
    /// The declaration of this model found at `node`.
    #[must_use]
    pub fn descriptor(&self, node: u16) -> Descriptor {
        Descriptor {
            driver: self.driver.to_owned(),
            detail_schema: Some(HAND_STATE.to_owned()),
            model: self.model.to_owned(),
            node_id: Some(node),
            joints: self
                .axes
                .iter()
                .map(|name| Joint {
                    name: (*name).to_owned(),
                    unit: JointUnit::Fraction,
                    min: 0.0,
                    max: 1.0,
                })
                .collect(),
            capabilities: Capabilities {
                openness: true,
                joints: true,
                max_command_hz: self.max_command_hz,
                ..Capabilities::default()
            },
            set_timeout_ms: None,
            tool: Some(self.tool),
            // The daemon's guard has no envelope of a hand; it checks one
            // with its default and reports that it did.
            guard_model: None,
        }
    }
}

/// One serialized connection to a hand at one node, as its device crate
/// implements it. Every method blocks on the bus; [`HandProvider`] calls
/// them on a blocking thread, one at a time.
pub trait HandLink: Send + 'static {
    /// One read that commands nothing: whether the hand answers at this
    /// link's node.
    fn probe(&mut self) -> bool;
    /// Enable the hand: control mode and every joint, verified the way the
    /// hand allows.
    ///
    /// # Errors
    ///
    /// The hand's or the transport's failure.
    fn enable(&mut self) -> Result<(), ProviderError>;
    /// Disable every joint, leaving the fingers where they are.
    ///
    /// # Errors
    ///
    /// The hand's or the transport's failure.
    fn disable(&mut self) -> Result<(), ProviderError>;
    /// Command every axis to a fraction of its wire range, in wire order.
    ///
    /// # Errors
    ///
    /// The hand's or the transport's failure.
    fn set_fractions(&mut self, axes: &[f64]) -> Result<(), ProviderError>;
    /// Drive the hand toward a closedness, `0.0` open … `1.0` a fist, by the
    /// model's own posture mapping.
    ///
    /// # Errors
    ///
    /// The hand's or the transport's failure.
    fn set_closedness(&mut self, closed: f64) -> Result<(), ProviderError>;
    /// One live sample.
    ///
    /// # Errors
    ///
    /// [`ReadError::Yielded`] when a waiting command took the bus, and the
    /// hand's or the transport's failure.
    fn sample(&mut self) -> Result<HandSample, ReadError>;
}

/// A hand on one side, served over `end_effector.shm/1`.
///
/// Openness drives the model's posture mapping; the joints are the axes in
/// wire order, each a fraction of its wire range; the first command after a
/// stop enables the joints. `set` returns once the command is on the wire
/// (the hands report no stroke outcome, so `wait` changes nothing). A stop
/// disables without opening, and a stop that follows a completed stop with
/// no command in between sends nothing: the daemon delivers one stop twice
/// (its stop generation and its `stop` request).
pub struct HandProvider<L> {
    facts: HandFacts,
    side: Side,
    node: u16,
    link: SerializedLink<L>,
    /// Whether the joints were enabled since the last disable or stop.
    enabled: AtomicBool,
    /// Whether the newest stop's disable reached the hand and nothing has
    /// commanded it since.
    stopped: Arc<AtomicBool>,
    /// The last state a live read produced, answered (`live: false`) while
    /// the link is busy or a read yields.
    last: StaleCache<HandState>,
    /// Whether a state read samples the hand; without it every state is the
    /// last sample, not live.
    poll: bool,
}

impl<L: HandLink> HandProvider<L> {
    /// The provider of the `facts` hand found at `node` on `side`, over the
    /// link `open` builds (it receives the counter its reads poll).
    pub fn new(
        facts: HandFacts,
        side: Side,
        node: u16,
        open: impl FnOnce(CommandsWaiting) -> L,
    ) -> Self {
        Self {
            facts,
            side,
            node,
            link: SerializedLink::new(open),
            enabled: AtomicBool::new(false),
            stopped: Arc::new(AtomicBool::new(false)),
            last: StaleCache::default(),
            poll: true,
        }
    }

    /// Serve states without sampling the hand: every state read answers the
    /// last sample (none here), marked not live, and puts nothing on the
    /// wire. For a test that compares the frames commands produce.
    #[must_use]
    pub const fn without_polling(mut self) -> Self {
        self.poll = false;
        self
    }

    /// The node the hand answers at.
    #[must_use]
    pub const fn node(&self) -> u16 {
        self.node
    }

    /// Run one motion command, enabling the joints first when they are not.
    async fn command(
        &self,
        action: impl FnOnce(&mut L) -> Result<(), ProviderError> + Send + 'static,
    ) -> Result<(), ProviderError> {
        self.stopped.store(false, Ordering::Release);
        if !self.enabled.load(Ordering::Acquire) {
            self.link.command(L::enable).await?;
            self.enabled.store(true, Ordering::Release);
        }
        self.link.command(action).await
    }

    /// The last live sample, or an unsampled one, marked not live either way.
    fn stale(&self) -> HandState {
        let mut state = self.last.last().unwrap_or_else(|| {
            HandState::unsampled(
                self.facts.model,
                self.side.as_str(),
                self.facts.axes.len(),
                self.facts.features,
            )
        });
        state.live = false;
        state
    }

    fn reading(&self, state: &HandState) -> State {
        let positions_known = state.live || !state.positions.is_empty();
        State {
            openness: positions_known
                .then(|| (self.facts.openness)(&state.positions))
                .flatten(),
            joints: state.positions.clone(),
            live: state.live,
            fault: state.fault(),
            detail: Some(state.detail()),
            ..State::default()
        }
    }
}

#[async_trait]
impl<L: HandLink> Provider for HandProvider<L> {
    fn descriptor(&self) -> Descriptor {
        self.facts.descriptor(self.node)
    }

    async fn set(&self, command: SetCommand) -> Result<(), ProviderError> {
        let closed = (1.0 - command.openness).clamp(0.0, 1.0);
        self.command(move |link| link.set_closedness(closed)).await
    }

    async fn joints(&self, targets: Vec<f64>, _timestamp_us: u64) -> Result<(), ProviderError> {
        validate_fractions(&targets, self.facts.axes.len())?;
        self.command(move |link| link.set_fractions(&targets)).await
    }

    async fn read(&self) -> Result<State, ProviderError> {
        if !self.poll {
            return Ok(self.reading(&self.stale()));
        }
        // A busy hand is not interrupted for a status read: a command owns
        // the link, and the last sample with `live: false` is a truthful
        // answer where waiting would not be.
        let state = match self.link.try_read(L::sample).await? {
            None | Some(Err(ReadError::Yielded)) => self.stale(),
            Some(Ok(sample)) => {
                let state = HandState::live(
                    self.facts.model,
                    self.side.as_str(),
                    sample,
                    self.facts.features,
                );
                self.last.remember(state.clone());
                state
            }
            Some(Err(error)) => return Err(error.into()),
        };
        Ok(self.reading(&state))
    }

    /// A stop DISABLES; it does not open. Both hands hold their pose when
    /// disabled, so opening on a kill would drop whatever is held, from
    /// whatever height the arm is at.
    async fn stop(&self) -> Result<(), ProviderError> {
        // Marked first: a disable that fails half-way must not leave the
        // provider believing the joints are still under its authority.
        self.enabled.store(false, Ordering::Release);
        let stopped = Arc::clone(&self.stopped);
        self.link
            .command(move |link| {
                if stopped.load(Ordering::Acquire) {
                    return Ok(());
                }
                link.disable()?;
                stopped.store(true, Ordering::Release);
                Ok(())
            })
            .await
    }
}
