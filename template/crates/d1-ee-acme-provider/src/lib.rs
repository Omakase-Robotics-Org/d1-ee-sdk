//! The ACME G1 gripper as a D1 end-effector provider: one jaw joint, served
//! to `d1-firmwared` over `end_effector.shm/1` by `d1fw_ee_provider::serve`.
//!
//! The SDK checks every command against [`AcmeProvider::descriptor`] before
//! it arrives here, so this crate holds only what is specific to the device:
//! the declaration, and the mapping of `set`, `joints`, `read` and `stop`
//! onto the [`Transport`].

use std::sync::{Arc, Mutex};

use async_trait::async_trait;
use d1fw_ee_provider::{
    Capabilities, Descriptor, ErrorKind, Joint, JointUnit, Provider, ProviderError, SetCommand,
    State,
};

pub mod app;
pub mod config;
pub mod transport;

pub use config::{Config, Settings};
pub use transport::{FakeTransport, Transport, TransportError};

/// The descriptor's `model`; a manifest `models[].model`.
pub const MODEL: &str = "acme/g1";

/// The descriptor's `driver`; the same entry's `driver`.
pub const DRIVER: &str = "acme-g1";

/// The guard envelope id the descriptor declares; listed in the manifest's
/// `guard_models`. An id the daemon's guard has no envelope for is checked
/// with the default envelope, and the daemon reports that.
pub const GUARD_MODEL: &str = "acme/g1";

/// The one joint.
pub const JOINT: &str = "jaw";

/// The vendor-rated command ceiling, hertz.
pub const MAX_COMMAND_HZ: u32 = 50;

/// The provider over one transport.
pub struct AcmeProvider {
    settings: Settings,
    transport: Arc<Mutex<Box<dyn Transport>>>,
}

impl AcmeProvider {
    /// Serve the gripper behind `transport` with `settings`.
    #[must_use]
    pub fn new(settings: Settings, transport: Box<dyn Transport>) -> Self {
        Self {
            settings,
            transport: Arc::new(Mutex::new(transport)),
        }
    }

    /// Run `call` on the transport from a blocking task: a device exchange
    /// never stalls the SDK's async tasks.
    async fn device<T: Send + 'static>(
        &self,
        call: impl FnOnce(&mut dyn Transport) -> Result<T, TransportError> + Send + 'static,
    ) -> Result<T, ProviderError> {
        let transport = Arc::clone(&self.transport);
        tokio::task::spawn_blocking(move || {
            let mut transport = transport
                .lock()
                .map_err(|_| TransportError("the transport lock is poisoned".to_owned()))?;
            call(transport.as_mut())
        })
        .await
        .map_err(|error| ProviderError::new(ErrorKind::Device, error.to_string()))?
        .map_err(|error| ProviderError::new(ErrorKind::Device, error.0))
    }
}

/// The declaration: one jaw joint in fractions of the stroke (`0` closed,
/// `1` open), driven by an openness or by joint targets.
#[must_use]
pub fn descriptor(settings: Settings) -> Descriptor {
    Descriptor {
        driver: DRIVER.to_owned(),
        detail_schema: None,
        model: MODEL.to_owned(),
        node_id: Some(u16::from(settings.device_id)),
        joints: vec![Joint {
            name: JOINT.to_owned(),
            unit: JointUnit::Fraction,
            min: 0.0,
            max: 1.0,
        }],
        capabilities: Capabilities {
            openness: true,
            joints: true,
            max_command_hz: Some(MAX_COMMAND_HZ),
            ..Capabilities::default()
        },
        set_timeout_ms: None,
        // A real provider declares its measured mass, centre of mass,
        // inertia and tool centre point here (`d1fw_ee_provider::Tool`); the
        // daemon registers it on the arm when it accepts the declaration.
        tool: None,
        guard_model: Some(GUARD_MODEL.to_owned()),
    }
}

#[async_trait]
impl Provider for AcmeProvider {
    fn descriptor(&self) -> Descriptor {
        descriptor(self.settings)
    }

    async fn set(&self, command: SetCommand) -> Result<(), ProviderError> {
        let speed = self.settings.speed_percent;
        self.device(move |device| device.move_to(command.openness, speed))
            .await
    }

    async fn joints(&self, targets: Vec<f64>, _timestamp_us: u64) -> Result<(), ProviderError> {
        // The SDK has checked the count and the limits.
        let target = targets[0];
        let speed = self.settings.speed_percent;
        self.device(move |device| device.move_to(target, speed))
            .await
    }

    async fn read(&self) -> Result<State, ProviderError> {
        let position = self.device(|device| device.position()).await?;
        Ok(State {
            openness: Some(position),
            joints: vec![position],
            live: true,
            ..State::default()
        })
    }

    async fn stop(&self) -> Result<(), ProviderError> {
        self.device(|device| device.halt()).await
    }
}
