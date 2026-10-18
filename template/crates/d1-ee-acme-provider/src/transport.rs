//! The device seam: what the provider asks of the gripper, and a fake
//! gripper behind it for tests and for a robot-free run (`--fake`).
//!
//! A real transport implements [`Transport`] over the device's own bus: the
//! arm's flange channel (`d1fw_ee_provider::FlangeBus`, every frame labelled
//! `motion`, `read` or `stop`), a SocketCAN interface, a serial port or a
//! socket. Every method is blocking and bounded by the device's own reply
//! timeout; the provider calls them from a blocking task.

use std::fmt;

/// Why the device did not do what was asked.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransportError(pub String);

impl fmt::Display for TransportError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl std::error::Error for TransportError {}

/// The gripper as the provider drives it. Positions are fractions of the
/// stroke, `0.0` closed and `1.0` open.
pub trait Transport: Send + 'static {
    /// Start a stroke to `position` at `speed_percent`; returns once the
    /// device took the target, not when it arrives.
    ///
    /// # Errors
    ///
    /// The device refused or did not answer.
    fn move_to(&mut self, position: f64, speed_percent: u8) -> Result<(), TransportError>;

    /// Read the current position.
    ///
    /// # Errors
    ///
    /// The device did not answer.
    fn position(&mut self) -> Result<f64, TransportError>;

    /// Halt and take the motor's torque away without opening. Called twice
    /// for one stop at most; a second call is harmless.
    ///
    /// # Errors
    ///
    /// The device did not answer.
    fn halt(&mut self) -> Result<(), TransportError>;
}

/// A gripper that reaches every target at once.
#[derive(Debug, Clone)]
pub struct FakeTransport {
    position: f64,
    halts: usize,
}

impl Default for FakeTransport {
    fn default() -> Self {
        Self {
            position: 1.0,
            halts: 0,
        }
    }
}

impl FakeTransport {
    /// How many halts arrived.
    #[must_use]
    pub const fn halts(&self) -> usize {
        self.halts
    }
}

impl Transport for FakeTransport {
    fn move_to(&mut self, position: f64, _speed_percent: u8) -> Result<(), TransportError> {
        if !(0.0..=1.0).contains(&position) {
            return Err(TransportError(format!(
                "position {position} is outside the stroke"
            )));
        }
        self.position = position;
        Ok(())
    }

    fn position(&mut self) -> Result<f64, TransportError> {
        Ok(self.position)
    }

    fn halt(&mut self) -> Result<(), TransportError> {
        self.halts += 1;
        Ok(())
    }
}
