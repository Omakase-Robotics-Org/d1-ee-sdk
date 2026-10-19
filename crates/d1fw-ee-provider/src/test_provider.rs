//! A simulated one-joint gripper served over the contract, for tests.

use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicU64, AtomicUsize, Ordering},
    },
    time::{Duration, Instant},
};

use async_trait::async_trait;
use serde_json::json;
use tokio::sync::watch;

use crate::{
    Capabilities, Descriptor, Detail, ErrorKind, FaultClear, ForceSensing, GripPreset,
    HOLD_DETAIL_KEY, HoldEvent, HoldEventKind, HoldLifetime, HoldPhase, HoldReport, Joint,
    JointUnit, Provider, ProviderError, Register, Registers, SetCommand, State, Tick, Tool,
    hold::{HOLD_NOTICE_LEAD_MS, STOP_NOTICE_LEAD_MS},
};

/// The detail schema id [`TestProvider`] declares; no consumer decodes it.
pub const TEST_DETAIL_SCHEMA: &str = "test.position/1";

/// The tool [`TestProvider::gripper_descriptor`] declares: 0.5 kg, its
/// centre of mass 40 mm and its tool centre point 110 mm along the flange's
/// Z axis.
pub const TEST_TOOL: Tool = Tool {
    mass_kg: 0.5,
    com_mm: [0.0, 0.0, 40.0],
    inertia: [0.0004, 0.0, 0.0, 0.0004, 0.0, 0.0002],
    tcp_xyz_mm: [0.0, 0.0, 110.0],
    tcp_rpy_deg: [0.0, 0.0, 0.0],
};

/// The guard envelope id [`TestProvider::gripper_descriptor`] declares; no
/// guard knows it.
pub const TEST_GUARD_MODEL: &str = "test/one-joint";

/// The jaw's open end, radians.
const OPEN_RAD: f64 = 1.0;

/// What a [`TestProvider`] was asked, and what it reports.
#[derive(Debug, Default)]
pub struct TestRecord {
    /// Every `set`, in arrival order.
    pub sets: Mutex<Vec<SetCommand>>,
    /// Every `joints` call: targets and timestamp.
    pub joints: Mutex<Vec<(Vec<f64>, u64)>>,
    /// How many stops arrived.
    pub stops: AtomicUsize,
    /// How many holds arrived (a hold is not recorded in `sets`).
    pub holds: AtomicUsize,
    /// How many homes arrived (not recorded in `sets`); a home moves the
    /// jaw to its open end.
    pub homes: AtomicUsize,
    /// How many fault clears arrived.
    pub clear_faults: AtomicUsize,
    /// Every register read's id list.
    pub register_reads: Mutex<Vec<Option<Vec<String>>>>,
    /// Every presence change the SDK reported.
    pub presence: Mutex<Vec<bool>>,
    /// The fault the device reports, settable by a test.
    pub fault: Mutex<Option<String>>,
    /// The error the next `set` answers with, settable by a test.
    pub fail_next_set: Mutex<Option<ProviderError>>,
    /// How long a `set` with `wait` takes, milliseconds.
    pub stroke_ms: AtomicU64,
    /// How long a `joints` call takes before it completes, milliseconds.
    pub joints_ms: AtomicU64,
    /// The order the provider's calls took effect in: `"joints"` when a
    /// `joints` call completed, `"set"`, `"hold"`, `"home"` or `"stop"` when
    /// one began.
    pub calls: Mutex<Vec<&'static str>>,
    /// The jaw position, radians.
    position: Mutex<f64>,
    /// The lifetime the provider declares for its holds (§4 rules 4 and
    /// 10), settable by a test; the default is a 60 s hold and a 2 s stop
    /// pose.
    pub lifetime: Mutex<HoldLifetime>,
    /// The report the provider carries in its detail: phase and events.
    report: Mutex<HoldReport>,
    /// When the current hold or stop pose started.
    since: Mutex<Option<Instant>>,
    /// Whether the notice of the current phase was given.
    noticed: Mutex<bool>,
}

impl TestRecord {
    /// The jaw position, radians.
    #[must_use]
    pub fn position(&self) -> f64 {
        *self.position.lock().expect("position lock")
    }

    /// The events recorded so far, oldest first (at most the last eight),
    /// after advancing the lifetime to now.
    #[must_use]
    pub fn hold_events(&self) -> Vec<HoldEvent> {
        self.advance(Instant::now());
        self.report.lock().expect("report lock").events.clone()
    }

    /// The phase after advancing the lifetime to now.
    #[must_use]
    pub fn hold_phase(&self) -> HoldPhase {
        self.advance(Instant::now());
        self.report.lock().expect("report lock").phase
    }

    fn begin(&self, phase: HoldPhase) {
        self.report.lock().expect("report lock").phase = phase;
        *self.since.lock().expect("since lock") = (phase != HoldPhase::None).then(Instant::now);
        *self.noticed.lock().expect("noticed lock") = false;
    }

    fn event(&self, kind: HoldEventKind, remaining_ms: u32) {
        self.report.lock().expect("report lock").record(HoldEvent {
            kind,
            mono_ns: d1fw_ee_shm_contract::clock::monotonic_ns(),
            remaining_ms,
        });
    }

    /// A stop: the pose is kept at reduced effort for `stop_hold_ms` when
    /// declared, otherwise relaxed at once.
    fn begin_stop(&self) {
        let declared = self.lifetime.lock().expect("lifetime lock").stop_hold_ms;
        if declared.is_some() {
            self.begin(HoldPhase::StopHolding);
        } else {
            self.begin(HoldPhase::None);
            self.event(HoldEventKind::StopRelaxed, 0);
        }
    }

    /// Move the lifetime on to `now`: a notice before its expiry, the
    /// expiry at the end of the period.
    fn advance(&self, now: Instant) {
        let phase = self.report.lock().expect("report lock").phase;
        let Some(since) = *self.since.lock().expect("since lock") else {
            return;
        };
        let lifetime = *self.lifetime.lock().expect("lifetime lock");
        let (total, lead, notice, end) = match phase {
            HoldPhase::None => return,
            HoldPhase::Held => (
                lifetime.hold_timeout_ms.unwrap_or(u32::MAX),
                HOLD_NOTICE_LEAD_MS,
                HoldEventKind::HoldExpiring,
                HoldEventKind::HoldExpired,
            ),
            HoldPhase::StopHolding => (
                lifetime.stop_hold_ms.unwrap_or(0),
                STOP_NOTICE_LEAD_MS,
                HoldEventKind::StopRelaxing,
                HoldEventKind::StopRelaxed,
            ),
        };
        let elapsed = now.saturating_duration_since(since);
        let total_d = Duration::from_millis(u64::from(total));
        let notice_at = total_d.saturating_sub(Duration::from_millis(u64::from(lead)));
        let give_notice = {
            let mut noticed = self.noticed.lock().expect("noticed lock");
            let due = !*noticed && elapsed >= notice_at;
            *noticed |= due;
            due
        };
        if give_notice {
            let remaining = total_d.saturating_sub(elapsed).as_millis();
            self.event(notice, u32::try_from(remaining).unwrap_or(u32::MAX));
        }
        if elapsed >= total_d {
            self.event(end, 0);
            if phase == HoldPhase::Held {
                // An expired hold is a stop (§4 rule 10).
                self.begin_stop();
            } else {
                self.begin(HoldPhase::None);
            }
        }
    }
}

/// A one-joint gripper (`jaw`, radians over `[0, 1]`) that moves at once,
/// records every call in a [`TestRecord`], and publishes a tick per command.
pub struct TestProvider {
    descriptor: Descriptor,
    record: Arc<TestRecord>,
    ticks: watch::Sender<Tick>,
}

impl Default for TestProvider {
    fn default() -> Self {
        Self::with_descriptor(Self::gripper_descriptor())
    }
}

impl TestProvider {
    /// The declaration [`TestProvider::default`] serves: openness, joints,
    /// stroke completion, the three presets, `torque_nm`, an `extra` key
    /// `speed`, fault clear, registers, ticks, joint velocities, a 200 Hz
    /// command ceiling, the tool [`TEST_TOOL`] and the guard envelope id
    /// [`TEST_GUARD_MODEL`].
    #[must_use]
    pub fn gripper_descriptor() -> Descriptor {
        Descriptor {
            driver: "test-gripper".to_owned(),
            detail_schema: Some(TEST_DETAIL_SCHEMA.to_owned()),
            model: "test/one-joint".to_owned(),
            node_id: Some(7),
            joints: vec![Joint {
                name: "jaw".to_owned(),
                unit: JointUnit::Rad,
                min: 0.0,
                max: OPEN_RAD,
            }],
            capabilities: Capabilities {
                openness: true,
                joints: true,
                stroke_completion: true,
                grip_presets: vec![GripPreset::Soft, GripPreset::Firm, GripPreset::Strong],
                grip_torque: true,
                set_extra: vec!["speed".to_owned()],
                force_sensing: ForceSensing::Estimated,
                contact_sensing: false,
                clear_fault: true,
                registers: true,
                ticks: true,
                joint_velocity: true,
                joint_effort: false,
                max_command_hz: Some(200),
            },
            set_timeout_ms: Some(2000),
            tool: Some(TEST_TOOL),
            guard_model: Some(TEST_GUARD_MODEL.to_owned()),
        }
    }

    /// A provider serving `descriptor` (its first joint is the simulated
    /// jaw).
    #[must_use]
    pub fn with_descriptor(descriptor: Descriptor) -> Self {
        let record = Arc::new(TestRecord::default());
        *record.position.lock().expect("position lock") = OPEN_RAD;
        *record.lifetime.lock().expect("lifetime lock") = HoldLifetime {
            hold_timeout_ms: Some(60_000),
            stop_hold_ms: Some(2_000),
        };
        Self {
            descriptor,
            record,
            ticks: watch::channel(Tick::default()).0,
        }
    }

    /// What this provider was asked.
    #[must_use]
    pub fn record(&self) -> Arc<TestRecord> {
        Arc::clone(&self.record)
    }

    fn move_to(&self, position: f64) {
        *self.record.position.lock().expect("position lock") = position;
        self.ticks.send_modify(|tick| {
            tick.seq += 1;
            tick.timestamp_us = d1fw_ee_shm_contract::clock::wall_clock_us();
            tick.measured = vec![position];
            tick.commanded = Some(vec![position]);
            tick.phase = crate::TickPhase::Idle;
        });
    }
}

#[async_trait]
impl Provider for TestProvider {
    fn descriptor(&self) -> Descriptor {
        self.descriptor.clone()
    }

    async fn set(&self, command: SetCommand) -> Result<(), ProviderError> {
        if let Some(error) = self.record.fail_next_set.lock().expect("lock").take() {
            return Err(error);
        }
        let verb = if command.is_hold() {
            "hold"
        } else if command.is_home() {
            "home"
        } else {
            "set"
        };
        self.record.calls.lock().expect("lock").push(verb);
        if command.is_hold() {
            // Held where it is: nothing moves. A new hold restarts the
            // lifetime.
            self.record.holds.fetch_add(1, Ordering::Relaxed);
            self.record.begin(HoldPhase::Held);
            return Ok(());
        }
        if command.is_home() {
            self.record.homes.fetch_add(1, Ordering::Relaxed);
            self.record.begin(HoldPhase::Held);
            self.move_to(OPEN_RAD);
            return Ok(());
        }
        self.record.begin(HoldPhase::None);
        let wait = command.wait;
        let openness = command.openness;
        self.record.sets.lock().expect("lock").push(command);
        let stroke = self.record.stroke_ms.load(Ordering::Relaxed);
        if wait && stroke > 0 {
            tokio::time::sleep(std::time::Duration::from_millis(stroke)).await;
        }
        self.move_to(openness * OPEN_RAD);
        Ok(())
    }

    async fn joints(&self, targets: Vec<f64>, timestamp_us: u64) -> Result<(), ProviderError> {
        let delay = self.record.joints_ms.load(Ordering::Relaxed);
        if delay > 0 {
            tokio::time::sleep(Duration::from_millis(delay)).await;
        }
        let position = targets.first().copied().unwrap_or(OPEN_RAD);
        self.record.begin(HoldPhase::None);
        self.record.calls.lock().expect("lock").push("joints");
        self.record
            .joints
            .lock()
            .expect("lock")
            .push((targets, timestamp_us));
        self.move_to(position);
        Ok(())
    }

    async fn read(&self) -> Result<State, ProviderError> {
        let position = self.record.position();
        self.record.advance(Instant::now());
        let mut report = self.record.report.lock().expect("report lock").clone();
        report.lifetime = *self.record.lifetime.lock().expect("lifetime lock");
        Ok(State {
            openness: Some((position / OPEN_RAD).clamp(0.0, 1.0)),
            joints: vec![position],
            // The jaw moves at once, so it is never seen moving.
            velocity: vec![Some(0.0)],
            live: true,
            holding: Some(report.phase == HoldPhase::Held),
            fault: self.record.fault.lock().expect("lock").clone(),
            detail: self.descriptor.detail_schema.clone().map(|schema| Detail {
                schema,
                data: json!({"position_rad": position, HOLD_DETAIL_KEY: report.to_value()}),
            }),
            ..State::default()
        })
    }

    async fn stop(&self) -> Result<(), ProviderError> {
        self.record.stops.fetch_add(1, Ordering::Relaxed);
        self.record.calls.lock().expect("lock").push("stop");
        self.record.begin_stop();
        Ok(())
    }

    async fn clear_fault(&self) -> Result<FaultClear, ProviderError> {
        self.record.clear_faults.fetch_add(1, Ordering::Relaxed);
        let cleared = self.record.fault.lock().expect("lock").take();
        Ok(FaultClear {
            ok: true,
            fault: None,
            details: json!({"cleared": cleared}),
        })
    }

    async fn registers(&self, ids: Option<Vec<String>>) -> Result<Registers, ProviderError> {
        self.record
            .register_reads
            .lock()
            .expect("lock")
            .push(ids.clone());
        let ids = ids.unwrap_or_else(|| vec!["position".to_owned()]);
        if let Some(unknown) = ids.iter().find(|id| id.as_str() != "position") {
            return Err(ProviderError::new(
                ErrorKind::Unprocessable,
                format!("no register {unknown:?}"),
            ));
        }
        Ok(Registers {
            entries: ids
                .into_iter()
                .map(|id| Register {
                    id,
                    name: Some("jaw position".to_owned()),
                    value: json!(self.record.position()),
                    details: json!({"raw": "0x00"}),
                })
                .collect(),
            details: json!({"node_id": 7}),
        })
    }

    fn ticks(&self) -> Option<watch::Receiver<Tick>> {
        Some(self.ticks.subscribe())
    }

    fn daemon_presence(&self, present: bool) {
        self.record.presence.lock().expect("lock").push(present);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{HoldEventKind as Kind, SetCommand};

    fn make(hold_ms: Option<u32>, stop_ms: Option<u32>) -> (TestProvider, Arc<TestRecord>) {
        let provider = TestProvider::default();
        let record = provider.record();
        *record.lifetime.lock().expect("lock") = HoldLifetime {
            hold_timeout_ms: hold_ms,
            stop_hold_ms: stop_ms,
        };
        (provider, record)
    }

    /// The events so far, without advancing the clock.
    fn kinds(record: &TestRecord) -> Vec<Kind> {
        let report = record.report.lock().expect("lock");
        report.events.iter().map(|e| e.kind).collect()
    }

    fn phase(record: &TestRecord) -> HoldPhase {
        record.report.lock().expect("lock").phase
    }

    /// Advance the lifetime to `ms` after the current period began, so the
    /// test does not depend on how long the machine took to run it.
    fn at(record: &TestRecord, ms: u64) {
        let since = record.since.lock().expect("lock").expect("a period began");
        record.advance(since + Duration::from_millis(ms));
    }

    /// An expired hold is a stop: the notice comes first, then the expiry,
    /// then the stop's own pose when declared.
    #[tokio::test]
    async fn an_expired_hold_is_treated_as_a_stop() {
        let (provider, record) = make(Some(5_000), Some(300));
        provider.set(SetCommand::hold()).await.expect("hold");
        assert_eq!(phase(&record), HoldPhase::Held);
        at(&record, 3_999);
        assert!(kinds(&record).is_empty(), "no notice before 1 s ahead");
        at(&record, 4_000);
        assert_eq!(kinds(&record), [Kind::HoldExpiring], "noticed 1 s ahead");
        assert_eq!(record.report.lock().unwrap().events[0].remaining_ms, 1_000);
        at(&record, 4_999);
        assert_eq!(kinds(&record), [Kind::HoldExpiring], "still held");
        assert_eq!(phase(&record), HoldPhase::Held);
        at(&record, 5_000);
        assert_eq!(
            kinds(&record),
            [Kind::HoldExpiring, Kind::HoldExpired],
            "expired: a stop, whose pose is kept"
        );
        assert_eq!(phase(&record), HoldPhase::StopHolding);
        // The stop pose lasts 300 ms, which is inside the 500 ms lead, so
        // it is noticed as soon as the clock moves.
        at(&record, 0);
        assert_eq!(kinds(&record).last(), Some(&Kind::StopRelaxing));
        at(&record, 300);
        assert_eq!(phase(&record), HoldPhase::None);
        assert_eq!(
            kinds(&record),
            [
                Kind::HoldExpiring,
                Kind::HoldExpired,
                Kind::StopRelaxing,
                Kind::StopRelaxed
            ]
        );
    }

    /// A new hold restarts the lifetime, so a hold can be renewed.
    #[tokio::test]
    async fn a_new_hold_restarts_the_lifetime() {
        let (provider, record) = make(Some(5_000), None);
        provider.set(SetCommand::hold()).await.expect("hold");
        at(&record, 4_000);
        assert_eq!(kinds(&record), [Kind::HoldExpiring]);
        provider.set(SetCommand::hold()).await.expect("renewed");
        assert_eq!(phase(&record), HoldPhase::Held);
        at(&record, 3_999);
        assert!(
            !kinds(&record).contains(&Kind::HoldExpired),
            "the first hold's 5 s ended, the renewed one's has not"
        );
        assert_eq!(phase(&record), HoldPhase::Held);
    }

    /// A stop keeps the pose for `stop_hold_ms` and then relaxes, noticing
    /// first; without a declared duration it relaxes at once.
    #[tokio::test]
    async fn a_stop_keeps_the_pose_then_relaxes() {
        let (provider, record) = make(Some(60_000), Some(700));
        provider.stop().await.expect("stop");
        assert_eq!(phase(&record), HoldPhase::StopHolding);
        at(&record, 199);
        assert!(kinds(&record).is_empty(), "0.5 s ahead of 0.7 s is 0.2 s");
        at(&record, 200);
        assert_eq!(kinds(&record), [Kind::StopRelaxing]);
        at(&record, 700);
        assert_eq!(kinds(&record), [Kind::StopRelaxing, Kind::StopRelaxed]);
        assert_eq!(phase(&record), HoldPhase::None);

        let (provider, record) = make(Some(60_000), None);
        provider.stop().await.expect("stop");
        assert_eq!(phase(&record), HoldPhase::None);
        assert_eq!(kinds(&record), [Kind::StopRelaxed]);
    }

    /// The state carries the report, within the caps.
    #[tokio::test]
    async fn the_state_carries_a_valid_report() {
        let (provider, _record) = make(Some(60_000), Some(2_000));
        provider.set(SetCommand::hold()).await.expect("hold");
        let state = provider.read().await.expect("read");
        assert_eq!(state.holding, Some(true));
        let report = HoldReport::from_detail_data(&state.detail.expect("detail").data)
            .expect("valid")
            .expect("present");
        assert_eq!(report.phase, HoldPhase::Held);
        assert_eq!(report.lifetime.hold_timeout_ms, Some(60_000));
    }
}
