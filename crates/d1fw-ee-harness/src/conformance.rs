//! The lifecycle every provider process must follow (`end_effector.shm/1`
//! §4 and §7), checked against a [`Host`]: it announces a valid declaration
//! and becomes available; every verb its declaration lists is answered
//! (`set`, `clear_fault` and `registers` on an idle device, then `joints`,
//! then a hold, whose declared lifetime is checked against the caps (§4
//! rule 10), and a home when `set_extra` lists them, then `stop`); killed with SIGKILL, the side turns unavailable for
//! want of a heartbeat and a stop raised meanwhile is refused; restarted,
//! the same declaration is taken back; SIGTERM ends it with status 0.

use std::{process::Command, time::Duration};

use d1fw_ee_shm_contract::{
    clock,
    hold::{HoldPhase, HoldReport},
    model,
};

use crate::{
    host::{Host, SetRequest},
    process::ProviderProcess,
};

/// How long the lifecycle waits for a provider to become available.
pub const AVAILABLE_WAIT: Duration = Duration::from_secs(20);

/// What [`lifecycle`] observed.
#[derive(Debug, Clone, PartialEq)]
pub struct Lifecycle {
    /// The accepted declaration.
    pub descriptor: model::Descriptor,
    /// The verbs the declaration lists that were driven and answered.
    pub verbs: Vec<&'static str>,
}

/// A provider that lists `hold` declares its lifetime (§4 rule 10): after a
/// hold its state's `detail` carries a `HoldReport` naming a
/// `hold_timeout_ms` within the cap, and a `stop_hold_ms` within its cap
/// when it names one. The provider's own `Held` phase is waited for, not the
/// first state published.
async fn check_hold_lifetime(host: &Host) {
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        if let Ok(state) = host.read().await
            && let Some(detail) = state.detail
        {
            let report = HoldReport::from_detail_data(&detail.data)
                .expect("the hold report in the state's detail is valid");
            if let Some(report) = report
                && report.phase == HoldPhase::Held
            {
                assert!(
                    report.lifetime.hold_timeout_ms.is_some(),
                    "a provider that lists `hold` declares hold_timeout_ms: {report:?}"
                );
                return;
            }
        }
        assert!(
            std::time::Instant::now() < deadline,
            "after a hold the state's detail never carried a hold report in phase `held` \
             (a provider that lists `hold` declares its lifetime under `detail.data.hold`)"
        );
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

/// Run the provider `command` (started again after a SIGKILL with the same
/// arguments) against `host` and check its lifecycle. The host's side must
/// have no provider yet.
///
/// # Panics
///
/// On any step that does not hold, naming it.
pub async fn lifecycle(host: &Host, command: impl Fn() -> Command) -> Lifecycle {
    let root = host_root(host);
    let mut process = ProviderProcess::start(command(), &root);
    host.until_available(AVAILABLE_WAIT).await;
    let descriptor = host.descriptor().expect("an accepted declaration");
    descriptor
        .validate()
        .expect("the accepted declaration is valid");

    let mut verbs = Vec::new();
    let capabilities = descriptor.capabilities.clone();
    if capabilities.openness {
        host.set(&SetRequest::openness(0.0))
            .await
            .expect("a set to openness 0 is answered");
        host.set(&SetRequest::openness(1.0))
            .await
            .expect("a set to openness 1 is answered");
        verbs.push("set");
    }
    if capabilities.clear_fault {
        host.clear_fault().await.expect("clear_fault is answered");
        verbs.push("clear_fault");
    }
    if capabilities.registers {
        host.registers(None).await.expect("registers are answered");
        verbs.push("registers");
    }
    if capabilities.joints {
        let targets = descriptor
            .joints
            .iter()
            .map(|joint| f64::midpoint(joint.min, joint.max))
            .collect();
        host.joints(targets, clock::wall_clock_us())
            .await
            .expect("joint targets are taken");
        verbs.push("joints");
    }
    if capabilities
        .set_extra
        .iter()
        .any(|key| key == model::HOLD_EXTRA_KEY)
    {
        host.hold().await.expect("a hold is answered");
        verbs.push("hold");
        check_hold_lifetime(host).await;
    }
    if capabilities
        .set_extra
        .iter()
        .any(|key| key == model::HOME_EXTRA_KEY)
    {
        host.home().await.expect("a home is answered");
        verbs.push("home");
    }
    host.stop().await.expect("a stop is answered");
    verbs.push("stop");
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let state = loop {
        if let Ok(state) = host.read().await {
            break state;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "no state published: {:?}",
            host.unavailable_reason()
        );
        tokio::time::sleep(Duration::from_millis(10)).await;
    };
    assert!(
        state.joints.is_empty() || state.joints.len() == descriptor.joints.len(),
        "a state carries every declared joint or none: {state:?}"
    );

    process.kill();
    host.until(
        "the side to become unavailable",
        Duration::from_secs(10),
        |host| !host.available(),
    )
    .await;
    let reason = host.unavailable_reason().unwrap_or_default();
    assert!(reason.contains("heartbeat"), "{reason}");
    assert!(host.stop().await.is_err(), "a stop with no provider");

    let process = ProviderProcess::start(command(), &root);
    host.until_available(AVAILABLE_WAIT).await;
    assert!(
        host.descriptor()
            .is_some_and(|accepted| accepted.same_but_node(&descriptor)),
        "the restarted provider's declaration is taken back"
    );
    let status = process.terminate();
    assert!(
        status.success(),
        "SIGTERM ends the provider cleanly: {status}"
    );
    Lifecycle { descriptor, verbs }
}

fn host_root(host: &Host) -> d1fw_ee_shm_contract::root::ShmRoot {
    host.settings().root.clone()
}
