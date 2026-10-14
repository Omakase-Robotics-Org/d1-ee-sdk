//! The SDK's rules, seen from a raw client of the contract: what reaches
//! the provider, what is answered without reaching it, and what is dropped.

use std::{
    sync::{Arc, atomic::Ordering},
    time::{Duration, Instant},
};

use contract::{
    clock,
    model::{self, Command, CommandEnvelope, JointTargets, SetCommand},
    names as suffix, ports, wire,
};
use d1fw_ee_provider::{
    ErrorKind, ServeOptions, ServiceNames, ShmRoot, TestProvider, contract, serve,
};

fn names() -> ServiceNames {
    ServiceNames::new("vendor/ee/b").unwrap()
}

/// A client of the provider's services, as the daemon is one.
struct RawDaemon {
    descriptor: iceoryx2::port::subscriber::Subscriber<ports::Ipc, wire::Descriptor, ()>,
    command: iceoryx2::port::client::Client<
        ports::Ipc,
        wire::CommandRequest,
        (),
        wire::CommandResponse,
        (),
    >,
    joints: iceoryx2::port::publisher::Publisher<ports::Ipc, wire::JointsCommand, ()>,
    _node: iceoryx2::prelude::Node<ports::Ipc>,
}

impl RawDaemon {
    fn open(root: &ShmRoot) -> Self {
        let node = root.node("raw-daemon").unwrap();
        let names = names();
        Self {
            descriptor: ports::open_topic::<wire::Descriptor>(
                &node,
                &names.name(suffix::DESCRIPTOR),
                ports::DESCRIPTOR_TOPIC,
            )
            .unwrap()
            .subscriber_builder()
            .create()
            .unwrap(),
            command: ports::open_rpc::<wire::CommandRequest, wire::CommandResponse>(
                &node,
                &names.name(suffix::COMMAND),
                ports::RPC,
            )
            .unwrap()
            .client_builder()
            .create()
            .unwrap(),
            joints: ports::open_topic::<wire::JointsCommand>(
                &node,
                &names.name(suffix::JOINTS),
                ports::JOINTS_TOPIC,
            )
            .unwrap()
            .publisher_builder()
            .create()
            .unwrap(),
            _node: node,
        }
    }

    /// The provider instance and generation, from its descriptor.
    async fn target(&self) -> (u64, u64) {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if let Ok(Some(sample)) = self.descriptor.receive() {
                let (_, generation) = model::Descriptor::from_wire(sample.payload()).unwrap();
                return (sample.payload().header.instance_id, generation);
            }
            assert!(Instant::now() < deadline, "no descriptor");
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    }

    async fn command(&self, envelope: &CommandEnvelope) -> model::CommandOutcome {
        let pending = self
            .command
            .send_copy(envelope.to_wire(model::header(99, 1)).unwrap())
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if let Some(response) = pending.receive().unwrap() {
                return model::read_command_response(response.payload()).unwrap();
            }
            assert!(Instant::now() < deadline, "no answer");
            tokio::time::sleep(Duration::from_millis(2)).await;
        }
    }
}

fn set(target: (u64, u64), start_by_mono_ns: u64, command: SetCommand) -> CommandEnvelope {
    CommandEnvelope {
        target_instance_id: target.0,
        descriptor_generation: target.1,
        start_by_mono_ns,
        command: Command::Set(command),
    }
}

fn soon() -> u64 {
    clock::monotonic_ns() + 250_000_000
}

#[tokio::test(flavor = "multi_thread")]
async fn only_a_set_for_this_instance_generation_and_deadline_runs() {
    let root = ShmRoot::scratch();
    let provider = TestProvider::default();
    let record = provider.record();
    let _serving = serve(Arc::new(provider), ServeOptions::new(root.clone(), names()))
        .await
        .unwrap();
    let daemon = RawDaemon::open(&root);
    let target = daemon.target().await;
    let half = SetCommand {
        openness: 0.5,
        ..SetCommand::default()
    };

    assert_eq!(
        daemon.command(&set(target, soon(), half.clone())).await,
        Ok(())
    );
    for (envelope, why) in [
        (
            set((target.0 + 1, target.1), soon(), half.clone()),
            "another instance",
        ),
        (
            set((target.0, target.1 + 1), soon(), half.clone()),
            "another generation",
        ),
        (
            set(target, clock::monotonic_ns() - 1, half.clone()),
            "past its deadline",
        ),
    ] {
        let outcome = daemon.command(&envelope).await;
        assert_eq!(
            outcome.map_err(|error| error.kind),
            Err(ErrorKind::StaleTarget),
            "{why}"
        );
    }
    let undeclared = SetCommand {
        openness: 0.5,
        extra: Some(serde_json::Map::from_iter([(
            "i_des".to_owned(),
            serde_json::json!(0.1),
        )])),
        ..SetCommand::default()
    };
    assert_eq!(
        daemon
            .command(&set(target, soon(), undeclared))
            .await
            .map_err(|error| error.kind),
        Err(ErrorKind::Unprocessable)
    );
    assert_eq!(
        record.sets.lock().unwrap().len(),
        1,
        "only the valid set ran"
    );

    // A stop runs whoever it was addressed to.
    let stop = CommandEnvelope {
        target_instance_id: 1,
        descriptor_generation: 0,
        start_by_mono_ns: 0,
        command: Command::Stop,
    };
    assert_eq!(daemon.command(&stop).await, Ok(()));
    assert_eq!(record.stops.load(Ordering::Relaxed), 1);
}

#[tokio::test(flavor = "multi_thread")]
async fn joint_targets_out_of_date_or_out_of_limits_are_dropped() {
    let root = ShmRoot::scratch();
    let provider = TestProvider::default();
    let record = provider.record();
    let _serving = serve(Arc::new(provider), ServeOptions::new(root.clone(), names()))
        .await
        .unwrap();
    let daemon = RawDaemon::open(&root);
    let target = daemon.target().await;
    let send = |targets: Vec<f64>, mono_ns: u64, timestamp_us: u64| {
        let mut header = model::header(99, timestamp_us);
        header.mono_ns = mono_ns;
        let message = JointTargets {
            target_instance_id: target.0,
            descriptor_generation: target.1,
            issued_mono_ns: 0,
            timestamp_us,
            targets,
        }
        .to_wire(header)
        .unwrap();
        daemon.joints.send_copy(message).unwrap();
    };
    let old = clock::monotonic_ns() - (ports::JOINTS_MAX_AGE_MS + 50) * 1_000_000;
    send(vec![0.2], old, 1);
    tokio::time::sleep(Duration::from_millis(50)).await;
    send(vec![1.5], clock::monotonic_ns(), 2);
    tokio::time::sleep(Duration::from_millis(50)).await;
    send(vec![0.4], clock::monotonic_ns(), 3);
    let deadline = Instant::now() + Duration::from_secs(5);
    while record.joints.lock().unwrap().is_empty() {
        assert!(Instant::now() < deadline, "no joint targets arrived");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert_eq!(*record.joints.lock().unwrap(), vec![(vec![0.4], 3)]);
}

#[tokio::test(flavor = "multi_thread")]
async fn options_and_declarations_the_contract_refuses_do_not_serve() {
    let root = ShmRoot::scratch();
    let mut options = ServeOptions::new(root.clone(), names());
    options.heartbeat_period = Duration::from_millis(500);
    assert!(
        serve(Arc::new(TestProvider::default()), options)
            .await
            .is_err()
    );
    let mut bad = TestProvider::gripper_descriptor();
    bad.joints[0].max = bad.joints[0].min;
    assert!(
        serve(
            Arc::new(TestProvider::with_descriptor(bad)),
            ServeOptions::new(root.clone(), names())
        )
        .await
        .is_err()
    );
}

/// The newest state the provider published, read raw off its topic.
async fn newest_state(root: &ShmRoot) -> wire::State {
    let node = root.node("raw-state-reader").unwrap();
    let subscriber =
        ports::open_topic::<wire::State>(&node, &names().name(suffix::STATE), ports::STATE_TOPIC)
            .unwrap()
            .subscriber_builder()
            .create()
            .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if let Ok(Some(sample)) = subscriber.receive() {
            return *sample.payload();
        }
        assert!(Instant::now() < deadline, "no state");
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

#[tokio::test(flavor = "multi_thread")]
async fn the_descriptor_carries_the_tool_and_the_state_its_declared_arrays() {
    let root = ShmRoot::scratch();
    let _serving = serve(
        Arc::new(TestProvider::default()),
        ServeOptions::new(root.clone(), names()),
    )
    .await
    .expect("serve");
    let daemon = RawDaemon::open(&root);
    daemon.target().await;
    let deadline = Instant::now() + Duration::from_secs(5);
    let declared = loop {
        if let Ok(Some(sample)) = daemon.descriptor.receive() {
            break model::Descriptor::from_wire(sample.payload()).unwrap().0;
        }
        assert!(Instant::now() < deadline, "no descriptor republish");
        tokio::time::sleep(Duration::from_millis(5)).await;
    };
    assert_eq!(declared.tool, Some(d1fw_ee_provider::TEST_TOOL));
    assert_eq!(
        declared.guard_model.as_deref(),
        Some(d1fw_ee_provider::TEST_GUARD_MODEL)
    );
    let state = newest_state(&root).await;
    let (read, _) = model::State::from_wire(&state, 1).unwrap();
    assert_eq!(read.velocity, vec![Some(0.0)]);
    assert!(read.effort.is_empty());
}

#[tokio::test(flavor = "multi_thread")]
async fn an_undeclared_array_is_sent_as_nan() {
    let root = ShmRoot::scratch();
    let mut descriptor = TestProvider::gripper_descriptor();
    descriptor.capabilities.joint_velocity = false;
    // The test provider reports a velocity whatever it declares.
    let _serving = serve(
        Arc::new(TestProvider::with_descriptor(descriptor)),
        ServeOptions::new(root.clone(), names()),
    )
    .await
    .expect("serve");
    let state = newest_state(&root).await;
    assert_eq!(state.joint_count, 1);
    assert!(state.velocity.iter().all(|value| value.is_nan()));
}
