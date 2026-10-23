//! A hand as an `end_effector.shm/1` provider: the facts a hand model
//! declares, the link its device crate implements, and [`HandProvider`],
//! which serves any such link.

use std::sync::{
    Arc, Mutex, PoisonError,
    atomic::{AtomicBool, Ordering},
};

use async_trait::async_trait;
use d1fw_ee_provider::{
    Capabilities, Descriptor, ErrorKind, FORCE_PERCENT, ForceRange, ForceUnit, GripForce, Joint,
    JointUnit, Provider, ProviderError, SetCommand, State, Tool, force_percent,
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
    /// The hand's adjustable grip force, which `force_percent` maps onto
    /// (its link applies it with [`HandLink::set_force_cap`]); `None` for a
    /// hand whose grip force cannot be set, which takes `force_percent` and
    /// reports it `fixed`.
    pub force: Option<HandForce>,
}

/// A hand's adjustable grip force, in the device's native unit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HandForce {
    /// The native unit, a token of the spec's vocabulary.
    pub unit: ForceUnit,
    /// The native values `force_percent` 0 and 100 map to.
    pub range: ForceRange,
}

impl HandForce {
    /// The report of `applied` in effect (`None` before the first `set`)
    /// for the newest `set`'s `requested` percent.
    #[must_use]
    pub const fn report(&self, requested: Option<f64>, applied: Option<f64>) -> GripForce {
        GripForce::adjustable(requested, applied, self.unit, self.range)
    }
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
                // Every hand takes the grip-force convention: one with an
                // adjustable force maps it, one without reports it fixed.
                set_extra: vec![FORCE_PERCENT.to_owned()],
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
    /// Cap the grip force of every motion command from now on at `cap`, in
    /// the unit of [`HandFacts::force`], or restore the device's default
    /// with `None` (the force every command carried before the grip-force
    /// convention); answer the value now in effect, rounded to what the
    /// wire takes. [`HandProvider`] calls it before the closedness command
    /// of every `set`, and only for a hand whose facts declare a force; the
    /// cap stays in effect for streamed joints until the next `set`. A link
    /// whose wire carries the force inside its motion frames writes nothing
    /// here.
    ///
    /// # Errors
    ///
    /// The hand's or the transport's failure. The default refuses: a hand
    /// that declares a force implements this.
    fn set_force_cap(&mut self, cap: Option<f64>) -> Result<f64, ProviderError> {
        let _ = cap;
        Err(ProviderError::new(
            ErrorKind::Device,
            "this hand link applies no grip-force cap",
        ))
    }
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
/// stop enables the joints. A `set`'s `force_percent` (spec §9) is mapped
/// onto [`HandFacts::force`] and applied before the closedness command, a
/// `set` without one restores the device's default, and every state carries
/// the `grip_force` report. `set` returns once the command is on the wire
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
    /// The grip force in effect, as every state reports it.
    grip_force: Arc<Mutex<GripForce>>,
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
        let grip_force = facts
            .force
            .map_or_else(|| GripForce::fixed(None), |force| force.report(None, None));
        Self {
            facts,
            side,
            node,
            link: SerializedLink::new(open),
            enabled: AtomicBool::new(false),
            stopped: Arc::new(AtomicBool::new(false)),
            last: StaleCache::default(),
            poll: true,
            grip_force: Arc::new(Mutex::new(grip_force)),
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

    /// The grip-force report every state carries now.
    #[must_use]
    pub fn grip_force(&self) -> GripForce {
        *self
            .grip_force
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
    }

    fn reading(&self, state: &HandState) -> State {
        let positions_known = state.live || !state.positions.is_empty();
        let state = HandState {
            grip_force: Some(self.grip_force()),
            ..state.clone()
        };
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
        let requested = force_percent(&command)?;
        let closed = (1.0 - command.openness).clamp(0.0, 1.0);
        let report = Arc::clone(&self.grip_force);
        match self.facts.force {
            Some(force) => {
                self.command(move |link| {
                    let cap = requested.map(|percent| force.range.at_percent(percent));
                    let applied = link.set_force_cap(cap)?;
                    *report.lock().unwrap_or_else(PoisonError::into_inner) =
                        force.report(requested, Some(applied));
                    link.set_closedness(closed)
                })
                .await
            }
            None => {
                *report.lock().unwrap_or_else(PoisonError::into_inner) =
                    GripForce::fixed(requested);
                self.command(move |link| link.set_closedness(closed)).await
            }
        }
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

#[cfg(test)]
mod tests {
    use d1fw_ee_provider::{GRIP_FORCE, GripForceMode, check_set};
    use serde_json::{Map, Value, json};

    use super::*;

    /// What the simulated link was told, in order.
    #[derive(Debug, Clone, PartialEq)]
    enum Call {
        Enable,
        Cap(Option<f64>),
        Closed(f64),
        Fractions(Vec<f64>),
    }

    /// A two-axis device whose force is a native integer in `[10, 50]`,
    /// default `30`, carried inside its motion frames: the cap is remembered
    /// and stamped on every motion it is told.
    struct SimLink {
        calls: Arc<Mutex<Vec<Call>>>,
        cap: f64,
        motions: Arc<Mutex<Vec<f64>>>,
    }

    const FORCE: HandForce = HandForce {
        unit: ForceUnit::Milliampere,
        range: ForceRange {
            min: 10.0,
            max: 50.0,
        },
    };
    const DEFAULT: f64 = 30.0;

    impl HandLink for SimLink {
        fn probe(&mut self) -> bool {
            true
        }
        fn enable(&mut self) -> Result<(), ProviderError> {
            self.calls.lock().unwrap().push(Call::Enable);
            Ok(())
        }
        fn disable(&mut self) -> Result<(), ProviderError> {
            Ok(())
        }
        fn set_fractions(&mut self, axes: &[f64]) -> Result<(), ProviderError> {
            self.calls
                .lock()
                .unwrap()
                .push(Call::Fractions(axes.to_vec()));
            self.motions.lock().unwrap().push(self.cap);
            Ok(())
        }
        fn set_closedness(&mut self, closed: f64) -> Result<(), ProviderError> {
            self.calls.lock().unwrap().push(Call::Closed(closed));
            self.motions.lock().unwrap().push(self.cap);
            Ok(())
        }
        fn set_force_cap(&mut self, cap: Option<f64>) -> Result<f64, ProviderError> {
            self.calls.lock().unwrap().push(Call::Cap(cap));
            self.cap = cap.map_or(DEFAULT, f64::round);
            Ok(self.cap)
        }
        fn sample(&mut self) -> Result<HandSample, ReadError> {
            Ok(HandSample {
                positions: vec![0.0, 0.0],
                positions_wire: vec![0, 0],
                enabled: vec![true, true],
                ..HandSample::default()
            })
        }
    }

    fn facts(force: Option<HandForce>) -> HandFacts {
        HandFacts {
            driver: "sim-hand",
            model: "maker/sim",
            axes: &["one", "two"],
            max_command_hz: None,
            features: &[],
            tool: Tool {
                mass_kg: 0.1,
                com_mm: [0.0; 3],
                inertia: [0.0; 6],
                tcp_xyz_mm: [0.0; 3],
                tcp_rpy_deg: [0.0; 3],
            },
            openness: crate::state::open_from_closed,
            force,
        }
    }

    type Log<T> = Arc<Mutex<Vec<T>>>;

    fn hand(force: Option<HandForce>) -> (HandProvider<SimLink>, Log<Call>, Log<f64>) {
        let calls = Arc::new(Mutex::new(Vec::new()));
        let motions = Arc::new(Mutex::new(Vec::new()));
        let provider = HandProvider::new(facts(force), Side::A, 1, {
            let calls = Arc::clone(&calls);
            let motions = Arc::clone(&motions);
            move |_| SimLink {
                calls,
                cap: DEFAULT,
                motions,
            }
        });
        (provider, calls, motions)
    }

    fn set(openness: f64, percent: Option<Value>) -> SetCommand {
        SetCommand {
            openness,
            wait: true,
            grip: None,
            torque_nm: None,
            extra: percent.map(|value| Map::from_iter([(FORCE_PERCENT.to_owned(), value)])),
        }
    }

    async fn reported(provider: &HandProvider<SimLink>) -> GripForce {
        let state = provider.read().await.unwrap();
        let data = state.detail.unwrap().data;
        serde_json::from_value(data[GRIP_FORCE].clone()).unwrap()
    }

    #[test]
    fn every_hand_declares_force_percent() {
        for force in [None, Some(FORCE)] {
            let descriptor = facts(force).descriptor(1);
            assert_eq!(
                descriptor.capabilities.set_extra,
                vec![FORCE_PERCENT.to_owned()]
            );
            assert_eq!(check_set(&descriptor, &set(0.0, Some(json!(50)))), Ok(()));
            assert!(check_set(&descriptor, &set(0.0, Some(json!(101)))).is_err());
        }
    }

    #[tokio::test]
    async fn an_adjustable_hand_caps_before_it_closes_and_restores_its_default() {
        let (provider, calls, motions) = hand(Some(FORCE));
        assert_eq!(
            reported(&provider).await,
            FORCE.report(None, None),
            "unit and range before the first set, nothing applied yet"
        );

        provider.set(set(0.0, Some(json!(12.5)))).await.unwrap();
        assert_eq!(
            *calls.lock().unwrap(),
            vec![Call::Enable, Call::Cap(Some(15.0)), Call::Closed(1.0)],
            "the cap reaches the link before the closedness command"
        );
        assert_eq!(
            reported(&provider).await,
            FORCE.report(Some(12.5), Some(15.0))
        );
        assert_eq!(
            reported(&provider).await.mode,
            GripForceMode::Adjustable,
            "every state carries the report, not only the one after the set"
        );

        // The cap stays in effect for streamed joints.
        provider.joints(vec![0.5, 0.5], 1).await.unwrap();
        provider.set(set(0.5, Some(json!(100)))).await.unwrap();
        provider.set(set(0.5, Some(json!(0)))).await.unwrap();
        assert_eq!(
            reported(&provider).await,
            FORCE.report(Some(0.0), Some(10.0))
        );

        // Without force_percent the default is restored and nothing is
        // reported as requested.
        provider.set(set(1.0, None)).await.unwrap();
        assert_eq!(reported(&provider).await, FORCE.report(None, Some(30.0)));
        assert_eq!(
            *motions.lock().unwrap(),
            vec![15.0, 15.0, 50.0, 10.0, 30.0],
            "the force each motion carried"
        );
    }

    #[tokio::test]
    async fn a_hand_without_a_force_range_takes_the_percent_and_reports_it_fixed() {
        let (provider, calls, _) = hand(None);
        assert_eq!(reported(&provider).await, GripForce::fixed(None));

        provider.set(set(0.0, Some(json!(40)))).await.unwrap();
        assert_eq!(reported(&provider).await, GripForce::fixed(Some(40.0)));
        provider.set(set(1.0, None)).await.unwrap();
        assert_eq!(reported(&provider).await, GripForce::fixed(None));
        assert!(
            !calls
                .lock()
                .unwrap()
                .iter()
                .any(|call| matches!(call, Call::Cap(_))),
            "a hand without a force range is never asked to cap"
        );
    }

    #[tokio::test]
    async fn a_malformed_percent_is_refused_before_the_wire() {
        let (provider, calls, _) = hand(Some(FORCE));
        for value in [json!(-1), json!(100.5), json!("firm")] {
            let refused = provider.set(set(0.0, Some(value))).await.unwrap_err();
            assert_eq!(refused.kind, ErrorKind::Unprocessable);
        }
        assert!(calls.lock().unwrap().is_empty());
        assert_eq!(reported(&provider).await, FORCE.report(None, None));
    }
}
