//! The scenario the hand crates' tests drive a provider with (feature
//! `test-support`), on the SDK's host (`d1fw-ee-harness`): the daemon's
//! side of both contracts with the flange relayed to a simulated hand.
//!
//! The fixed scenario is the one a hand's golden frames
//! (`tests/golden/flange_scenario.json` of a hand crate) were recorded for:
//! establish; `set` to openness 1, then 0 with the `firm` grip (which a hand
//! does not declare, so nothing reaches the wire), then 0, then 1; a
//! `joints` stream at 50 Hz for 2 s; a settle of 200 ms; `stop`;
//! `clear_fault`.

use std::{
    path::Path,
    sync::Arc,
    time::{Duration, Instant},
};

use d1fw_canfd::{CanFdBus, FrameKind};
use d1fw_ee_harness::{
    Host, HostSettings, ProviderProcess, SetRequest,
    golden::{self, Comparison, Steps, Tolerance},
    status,
};
use d1fw_ee_provider::{
    FlangeBus, FlangeNames, FlangeOptions, GripPreset, ProviderHandle, ServeOptions, ServiceNames,
    ShmRoot, contract::clock::wall_clock_us,
};
use serde_json::Value;

pub use d1fw_ee_harness::Recorder;

use crate::{
    HandProvider, Side,
    app::HandDevice,
    channel::FlangeChannel,
    link::{CommandsWaiting, detect_node, probe_order},
};

/// How far a frame's offset from its step's start may lie from the recorded
/// one: one command period of the 50 Hz stream.
pub const TIMING_TOLERANCE: Duration = Duration::from_millis(20);

/// The joint stream's period and length: 50 Hz for 2 s.
const JOINTS_PERIOD: Duration = Duration::from_millis(20);
const JOINTS_COUNT: usize = 100;

/// How long the scenario waits after the joint stream before it stops.
const SETTLE: Duration = Duration::from_millis(200);

/// The host of side A under `root`, relaying `channel` at the default tx
/// ceiling, on the side's default service names.
#[must_use]
pub fn host(root: &ShmRoot, channel: Arc<dyn CanFdBus>) -> Host {
    Host::start_with_flange_channel(HostSettings::new("a", root.clone()), channel)
}

/// A hand provider served in this process the way the provider binary
/// serves it: over a [`FlangeBus`] on `root`'s `side`, under the side's
/// default service names, admitted by the relay, the hand found by the
/// probe. With `poll` false its state reads put nothing on the wire, so the
/// frames on the channel are the commands' alone.
///
/// # Panics
///
/// When the channel cannot be opened, the relay does not admit it within
/// 5 s, or no hand answers the probe.
pub async fn provide<D: HandDevice>(
    device: D,
    root: &ShmRoot,
    side: Side,
    poll: bool,
) -> (ProviderHandle, Arc<FlangeBus>) {
    let mut options = FlangeOptions::new(
        root.clone(),
        FlangeNames::new(&d1fw_ee_provider::contract::flange::default_base(
            side.as_str(),
        ))
        .expect("a valid base"),
    );
    options.node_name = format!("hand-flange-{}", side.as_str());
    let flange = Arc::new(FlangeBus::open(options).expect("open the flange channel"));
    let waiting = Arc::clone(&flange);
    tokio::task::spawn_blocking(move || waiting.wait_admitted(Duration::from_secs(5)))
        .await
        .expect("the admission wait")
        .expect("the relay admits the provider");
    let channel = FlangeChannel::new(Arc::clone(&flange));
    let device = Arc::new(device);
    let node = {
        let device = Arc::clone(&device);
        let channel = channel.clone();
        detect_node(probe_order(device.probe_first(side)), move |node| {
            crate::HandLink::probe(&mut device.open(
                Box::new(channel.clone()),
                node,
                CommandsWaiting::default(),
            ))
        })
        .await
        .expect("a hand answers the probe")
    };
    channel.drain();
    let provider = HandProvider::new(device.facts(), side, node, {
        let device = Arc::clone(&device);
        let channel = channel.clone();
        move |waiting| device.open(Box::new(channel), node, waiting)
    });
    let provider = if poll {
        provider
    } else {
        provider.without_polling()
    };
    let mut options = ServeOptions::new(
        root.clone(),
        ServiceNames::new(&d1fw_ee_provider::names::default_base(side.as_str()))
            .expect("a valid base"),
    );
    options.node_name = format!("hand-{}", side.as_str());
    let handle = d1fw_ee_provider::serve(Arc::new(provider), options)
        .await
        .expect("serve the hand");
    (handle, flange)
}

fn set(openness: f64, grip: Option<GripPreset>) -> SetRequest {
    SetRequest {
        openness,
        grip,
        ..SetRequest::default()
    }
}

/// The joint targets of the stream's `index`th command: every axis a
/// different fraction, so a frame that carried the wrong target differs.
fn targets(index: usize, dof: usize) -> Vec<f64> {
    (0..dof)
        .map(|axis| {
            #[expect(clippy::cast_precision_loss, reason = "both are below a few hundred")]
            let value = ((index + axis) % 11) as f64 / 10.0;
            value
        })
        .collect()
}

/// Run the scenario on `host`, whose channel is `recorder`, and return its
/// steps: each step's name, HTTP status and frames.
///
/// # Panics
///
/// When the side has no declaration or a streamed command is refused.
pub async fn run_scenario<B: CanFdBus>(host: &Host, recorder: &Recorder<B>) -> Vec<Value> {
    let dof = host.descriptor().expect("a fitted side").joints.len();
    let mut steps = Steps::new(recorder);
    steps.discard();

    let started = Instant::now();
    steps.record("establish", started, 200);

    for (name, request) in [
        ("set openness 1.0", set(1.0, None)),
        (
            "set openness 0.0 grip firm",
            set(0.0, Some(GripPreset::Firm)),
        ),
        ("set openness 0.0", set(0.0, None)),
        ("set openness 1.0 again", set(1.0, None)),
    ] {
        let started = Instant::now();
        let outcome = status(&host.set(&request).await);
        steps.record(name, started, outcome);
    }

    let started = Instant::now();
    let mut statuses = Vec::new();
    for index in 0..JOINTS_COUNT {
        let due = started + JOINTS_PERIOD * u32::try_from(index).expect("a hundred commands");
        tokio::time::sleep_until(due.into()).await;
        statuses.push(status(
            &host.joints(targets(index, dof), wall_clock_us()).await,
        ));
    }
    tokio::time::sleep(SETTLE).await;
    assert!(
        statuses.iter().all(|status| *status == 200),
        "every streamed command is accepted: {statuses:?}"
    );
    steps.record("joints 50 Hz for 2 s", started, 200);

    let started = Instant::now();
    let outcome = status(&host.stop().await);
    steps.record("stop", started, outcome);

    let started = Instant::now();
    let outcome = status(&host.clear_fault().await);
    steps.record("clear_fault", started, outcome);

    steps.into_steps()
}

/// Assert `steps` reproduce the fixture at `path` (see
/// [`golden::compare`]), each frame within [`TIMING_TOLERANCE`] of its
/// recorded offset and relayed as `expected_kind(step, id, data)` names.
///
/// # Panics
///
/// On any difference, naming the step and the frame.
#[must_use]
pub fn compare(
    path: &Path,
    steps: &[Value],
    expected_kind: impl Fn(&str, u64, &str) -> &'static str,
) -> Comparison {
    golden::compare(
        path,
        steps,
        Tolerance {
            base_us: u64::try_from(TIMING_TOLERANCE.as_micros()).expect("a few ms"),
            per_frame_us: 0,
            gap_us: None,
        },
        expected_kind,
    )
}

/// What a hand provider's process must declare, for [`conformance`].
#[derive(Debug, Clone)]
pub struct Expected {
    /// The descriptor's driver.
    pub driver: &'static str,
    /// The descriptor's model.
    pub model: &'static str,
    /// The node the probe must find.
    pub node: u16,
    /// The declared tool's mass, kilograms.
    pub tool_mass_kg: f64,
}

fn start(binary: &Path, root: &ShmRoot) -> ProviderProcess {
    let mut command = std::process::Command::new(binary);
    command
        .args(["--side", "a", "--state-period-ms", "50"])
        .env("RUST_LOG", "warn");
    ProviderProcess::start(command, root)
}

/// The provider process `binary` against the host's side A with its flange
/// relayed to `device` (a simulated hand at the node `expected` names): it
/// is found by its probe and announces `expected`; it drives the hand and
/// publishes its state; a stop reaches it as one stop frame; under the
/// soft-kill latch its motion is refused and its stop passes; killed with
/// SIGKILL, the side turns unavailable within the stale threshold and a stop
/// is refused there; restarted, it is taken back, takes the stop raised
/// while it was away as its baseline (no stop frame), and SIGTERM ends it
/// with status 0.
///
/// # Panics
///
/// On any step that does not hold, naming it.
pub async fn conformance<B: CanFdBus + Clone + 'static>(
    binary: &Path,
    device: B,
    expected: &Expected,
) {
    let wait = Duration::from_secs(10);
    let root = ShmRoot::scratch();
    let recorder = Recorder::new(device);
    let host = host(&root, Arc::new(recorder.clone()));
    let mut process = start(binary, &root);
    host.until_available(wait).await;

    let descriptor = host.descriptor().expect("an accepted declaration");
    assert_eq!(descriptor.driver, expected.driver);
    assert_eq!(descriptor.model, expected.model);
    assert_eq!(
        descriptor.node_id,
        Some(expected.node),
        "the probe found the hand"
    );
    assert_eq!(
        descriptor.tool.as_ref().map(|tool| tool.mass_kg),
        Some(expected.tool_mass_kg),
        "the declared tool is the side's"
    );
    assert_eq!(descriptor.detail_schema.as_deref(), Some("hand.state/1"));

    recorder.take();
    host.set(&set(0.0, None)).await.expect("a fist");
    assert!(recorder.take_kinds().contains(&FrameKind::Motion));
    let state = live_state(&host, wait).await;
    assert_eq!(
        state.detail.as_ref().map(|detail| detail.schema.as_str()),
        Some("hand.state/1")
    );

    // The stop: its generation and its request reach the provider as one
    // disable on the wire.
    recorder.take();
    host.stop().await.expect("the provider stops");
    tokio::time::sleep(Duration::from_millis(150)).await;
    let stops = recorder.take_kinds();
    assert_eq!(
        stops
            .iter()
            .filter(|kind| **kind == FrameKind::Stop)
            .count(),
        1,
        "one stop frame: {stops:?}"
    );

    // Under the soft-kill latch: motion is refused before the wire, a stop
    // still passes.
    host.latch(true);
    tokio::time::sleep(Duration::from_millis(150)).await;
    recorder.take();
    let refused = host.set(&set(1.0, None)).await;
    assert!(
        refused.is_err(),
        "a set under the latch is refused: {refused:?}"
    );
    assert!(
        !recorder.take_kinds().contains(&FrameKind::Motion),
        "no motion frame reached the channel under the latch"
    );
    host.stop().await.expect("a stop under the latch");
    host.latch(false);

    // SIGKILL: the side turns unavailable, and a stop raised now is refused.
    process.kill();
    host.until("the side to become unavailable", wait, |host| {
        !host.available()
    })
    .await;
    let reason = host.unavailable_reason().unwrap_or_default();
    assert!(reason.contains("heartbeat"), "{reason}");
    assert!(host.stop().await.is_err(), "a stop with no provider");

    // Restarted: taken back, and the stop raised while it was away is its
    // baseline, not a stop to act on.
    recorder.take();
    let process = start(binary, &root);
    host.until_available(wait).await;
    tokio::time::sleep(Duration::from_millis(500)).await;
    let after_restart = recorder.take_kinds();
    assert!(
        !after_restart.contains(&FrameKind::Stop),
        "the restarted provider acted on an old stop: {after_restart:?}"
    );
    host.set(&set(1.0, None))
        .await
        .expect("the restarted provider drives the hand");

    let status = process.terminate();
    assert!(
        status.success(),
        "SIGTERM ends the provider cleanly: {status}"
    );
}

/// The side's state, once one is live.
async fn live_state(host: &Host, wait: Duration) -> d1fw_ee_provider::State {
    let deadline = Instant::now() + wait;
    loop {
        if let Ok(state) = host.read().await
            && state.live
        {
            return state;
        }
        assert!(
            Instant::now() < deadline,
            "timed out waiting for a live state"
        );
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}
