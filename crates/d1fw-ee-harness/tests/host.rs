//! The host against the SDK's own provider: the declaration, validation and
//! statuses, the stop, the relay's gates over a simulated device, and the
//! lifecycle of a provider process.

use std::{
    process::Command,
    sync::{Arc, Mutex, atomic::Ordering},
    time::Duration,
};

use d1fw_canfd::{CanFdBus, CanFdError, CanFdFrame, FrameKind};
use d1fw_ee_harness::{
    Host, HostErrorKind, HostSettings, Recorder, SetRequest, conformance, status,
};
use d1fw_ee_provider::{
    FlangeBus, FlangeNames, FlangeOptions, Frame, GripPreset, ServeOptions, ShmRoot, TEST_TOOL,
    TestProvider, TxKind, contract::clock::wall_clock_us, contract::flange,
};

const WAIT: Duration = Duration::from_secs(10);

async fn served(
    root: &ShmRoot,
) -> (
    Host,
    Arc<d1fw_ee_provider::TestRecord>,
    d1fw_ee_provider::ProviderHandle,
) {
    let host = Host::start(HostSettings::new("a", root.clone()));
    let provider = TestProvider::default();
    let record = provider.record();
    let handle = d1fw_ee_provider::serve(
        Arc::new(provider),
        ServeOptions::new(root.clone(), host.names().clone()),
    )
    .await
    .expect("serve the test provider");
    host.until_available(WAIT).await;
    (host, record, handle)
}

#[tokio::test(flavor = "multi_thread")]
async fn commands_are_validated_and_answered_with_the_daemons_statuses() {
    let root = ShmRoot::scratch();
    let (host, record, _handle) = served(&root).await;
    let descriptor = host.descriptor().expect("accepted");
    assert_eq!(descriptor.tool, Some(TEST_TOOL));

    assert_eq!(status(&host.set(&SetRequest::openness(0.25)).await), 200);
    assert_eq!(record.sets.lock().unwrap().len(), 1);
    assert_eq!(status(&host.set(&SetRequest::openness(1.5)).await), 422);
    let undeclared = SetRequest {
        extra: Some(serde_json::Map::from_iter([("force".to_owned(), 1.into())])),
        ..SetRequest::openness(0.5)
    };
    assert_eq!(status(&host.set(&undeclared).await), 422);
    assert_eq!(
        status(
            &host
                .set(&SetRequest::openness(0.0).grip(GripPreset::Firm))
                .await
        ),
        200
    );
    assert_eq!(
        record.sets.lock().unwrap().len(),
        2,
        "refused sets never reach it"
    );

    *record.fail_next_set.lock().unwrap() = Some(d1fw_ee_provider::ProviderError::new(
        d1fw_ee_provider::ErrorKind::Refused,
        "busy",
    ));
    let refused = host.set(&SetRequest::openness(0.5)).await.unwrap_err();
    assert_eq!(refused.kind, HostErrorKind::Refused);
    assert_eq!(refused.http_status(), 409);

    let stamp = wall_clock_us();
    host.joints(vec![0.5], stamp).await.expect("joints");
    let stale = host.joints(vec![0.4], stamp).await.unwrap_err();
    assert_eq!(stale.http_status(), 409);
    assert_eq!(
        host.joints(vec![2.0], stamp + 1)
            .await
            .unwrap_err()
            .http_status(),
        422
    );

    host.stop().await.expect("stop");
    assert!(record.stops.load(Ordering::Relaxed) >= 1);
    host.clear_fault().await.expect("clear_fault");
    let registers = host.registers(None).await.expect("registers");
    assert_eq!(registers.entries.len(), 1);
    assert_eq!(
        host.registers(Some(vec!["nope".to_owned()]))
            .await
            .unwrap_err()
            .http_status(),
        422
    );
    let state = host.read().await.expect("a state");
    assert!(state.live);
    assert_eq!(state.velocity.len(), 1, "joint_velocity is declared");
    assert!(state.effort.is_empty(), "joint_effort is not");
}

#[tokio::test(flavor = "multi_thread")]
async fn a_pinned_model_refuses_another_declaration() {
    let root = ShmRoot::scratch();
    let mut settings = HostSettings::new("a", root.clone());
    settings.expect_model = Some("acme/other".to_owned());
    let host = Host::start(settings);
    let _handle = d1fw_ee_provider::serve(
        Arc::new(TestProvider::default()),
        ServeOptions::new(root.clone(), host.names().clone()),
    )
    .await
    .expect("serve");
    host.until("a refusal", WAIT, |host| {
        host.unavailable_reason()
            .is_some_and(|reason| reason.contains("pins"))
    })
    .await;
    assert!(host.descriptor().is_none());
}

/// A device that answers every frame with the same frame, id + 1.
#[derive(Clone, Default)]
struct Echo {
    replies: Arc<Mutex<Vec<CanFdFrame>>>,
}

impl CanFdBus for Echo {
    fn send(&self, frame: &CanFdFrame) -> Result<(), CanFdError> {
        self.replies
            .lock()
            .unwrap()
            .push(CanFdFrame::new(frame.id + 1, frame.data.clone()));
        Ok(())
    }

    fn recv(&self, timeout: Duration) -> Result<Option<CanFdFrame>, CanFdError> {
        let reply = {
            let mut replies = self.replies.lock().unwrap();
            (!replies.is_empty()).then(|| replies.remove(0))
        };
        if reply.is_none() {
            std::thread::sleep(timeout.min(Duration::from_millis(1)));
        }
        Ok(reply)
    }

    fn close(&self) {}
}

#[tokio::test(flavor = "multi_thread")]
async fn the_relay_gates_motion_on_the_latch_and_reports_the_arm_fault() {
    let root = ShmRoot::scratch();
    let recorder = Recorder::new(Echo::default());
    let host = Host::start_with_flange_channel(
        HostSettings::new("a", root.clone()),
        Arc::new(recorder.clone()),
    );
    let bus = Arc::new(
        FlangeBus::open(FlangeOptions::new(
            root.clone(),
            FlangeNames::new(&flange::default_base("a")).unwrap(),
        ))
        .expect("open the flange"),
    );
    let waiting = Arc::clone(&bus);
    tokio::task::spawn_blocking(move || waiting.wait_admitted(Duration::from_secs(5)))
        .await
        .unwrap()
        .expect("admitted");

    let send = |kind| {
        let bus = Arc::clone(&bus);
        async move {
            tokio::task::spawn_blocking(move || bus.send(&Frame::standard(0x10, vec![1, 2]), kind))
                .await
                .unwrap()
        }
    };
    send(TxKind::Motion).await.expect("motion passes");
    let reply = {
        let bus = Arc::clone(&bus);
        tokio::task::spawn_blocking(move || bus.recv(Duration::from_secs(1)))
            .await
            .unwrap()
            .expect("receive")
            .expect("an answer")
    };
    assert_eq!(reply.frame.id, 0x11);
    assert_eq!(recorder.take_kinds(), vec![FrameKind::Motion]);

    host.latch(true);
    tokio::time::sleep(Duration::from_millis(200)).await;
    assert!(
        send(TxKind::Motion).await.is_err(),
        "motion refused under the latch"
    );
    send(TxKind::Stop).await.expect("a stop passes the latch");
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert_eq!(recorder.take_kinds(), vec![FrameKind::Stop]);
    host.latch(false);

    host.set_arm_faulted(true);
    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    while !bus.arm_faulted() {
        assert!(
            std::time::Instant::now() < deadline,
            "arm_faulted never set"
        );
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn the_test_provider_process_follows_the_lifecycle() {
    let root = ShmRoot::scratch();
    let host = Host::start(HostSettings::new("a", root.clone()));
    let observed = conformance::lifecycle(&host, || {
        let mut command = Command::new(env!("CARGO_BIN_EXE_d1-ee-test-provider"));
        command.args(["--side", "a", "--state-period-ms", "50"]);
        command
    })
    .await;
    assert_eq!(observed.descriptor.model, "test/one-joint");
    assert_eq!(
        observed.verbs,
        vec!["set", "clear_fault", "registers", "joints", "stop"]
    );
}

/// A hold (§4 rule 10) reaches a provider that lists `hold` in `set_extra`,
/// openness declared or not, as a `set` carrying `{"hold": true}`; a
/// provider that does not list it is refused before anything is sent.
#[tokio::test(flavor = "multi_thread")]
async fn a_hold_reaches_only_a_provider_that_declares_it() {
    let root = ShmRoot::scratch();
    let (host, record, _handle) = served(&root).await;
    assert_eq!(status(&host.hold().await), 409, "no hold declared");
    assert_eq!(record.holds.load(Ordering::Relaxed), 0);

    let root = ShmRoot::scratch();
    let host = Host::start(HostSettings::new("b", root.clone()));
    let mut descriptor = TestProvider::gripper_descriptor();
    descriptor.capabilities.openness = false;
    descriptor.capabilities.stroke_completion = false;
    descriptor.capabilities.grip_presets.clear();
    descriptor.capabilities.grip_torque = false;
    descriptor.capabilities.set_extra = vec!["hold".to_owned()];
    let provider = TestProvider::with_descriptor(descriptor);
    let record = provider.record();
    let _handle = d1fw_ee_provider::serve(
        Arc::new(provider),
        ServeOptions::new(root.clone(), host.names().clone()),
    )
    .await
    .expect("serve the holding provider");
    host.until_available(WAIT).await;
    assert_eq!(status(&host.hold().await), 200);
    assert_eq!(record.holds.load(Ordering::Relaxed), 1);
    assert!(
        record.sets.lock().unwrap().is_empty(),
        "a hold is not a set"
    );
    assert_eq!(
        status(&host.set(&SetRequest::openness(0.5)).await),
        409,
        "an openness is still refused without the capability"
    );
}
