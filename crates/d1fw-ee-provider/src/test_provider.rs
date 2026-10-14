//! A simulated one-joint gripper served over the contract, for tests.

use std::sync::{
    Arc, Mutex,
    atomic::{AtomicU64, AtomicUsize, Ordering},
};

use async_trait::async_trait;
use serde_json::json;
use tokio::sync::watch;

use crate::{
    Capabilities, Descriptor, Detail, ErrorKind, FaultClear, ForceSensing, GripPreset, Joint,
    JointUnit, Provider, ProviderError, Register, Registers, SetCommand, State, Tick, Tool,
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
    /// The jaw position, radians.
    position: Mutex<f64>,
}

impl TestRecord {
    /// The jaw position, radians.
    #[must_use]
    pub fn position(&self) -> f64 {
        *self.position.lock().expect("position lock")
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
        let position = targets.first().copied().unwrap_or(OPEN_RAD);
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
        Ok(State {
            openness: Some((position / OPEN_RAD).clamp(0.0, 1.0)),
            joints: vec![position],
            // The jaw moves at once, so it is never seen moving.
            velocity: vec![Some(0.0)],
            live: true,
            holding: Some(false),
            fault: self.record.fault.lock().expect("lock").clone(),
            detail: self.descriptor.detail_schema.clone().map(|schema| Detail {
                schema,
                data: json!({"position_rad": position}),
            }),
            ..State::default()
        })
    }

    async fn stop(&self) -> Result<(), ProviderError> {
        self.record.stops.fetch_add(1, Ordering::Relaxed);
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
