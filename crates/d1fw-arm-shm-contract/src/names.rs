//! The service names of the arm contracts.
//!
//! Every service is a fixed suffix under one base, `omakase/arm` unless the
//! daemon's configuration names another; the base follows the rules of
//! [`d1fw_ee_shm_contract::names::ServiceNames::new`].

use d1fw_ee_shm_contract::{model::ContractError, names::ServiceNames};

/// The base the arm services are bound to unless configured otherwise.
pub const DEFAULT_BASE: &str = "omakase/arm";

/// Suffix of `arm_feedback.shm/1`'s topic.
pub const FEEDBACK: &str = "feedback";
/// Suffix of `arm_status.shm/1`'s topic.
pub const STATUS: &str = "status";
/// Suffix of `arm_events.shm/1`'s topic.
pub const EVENTS: &str = "events";
/// Suffix of `arm_command.shm/1`'s command topic, and of its event service
/// (the same name, the event messaging pattern).
pub const COMMAND: &str = "command";
/// Suffix of `arm_command.shm/1`'s status topic.
pub const COMMAND_STATUS: &str = "command_status";

/// The service names under one base.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ArmNames(ServiceNames);

impl Default for ArmNames {
    fn default() -> Self {
        Self::new(DEFAULT_BASE).expect("the default base is valid")
    }
}

impl ArmNames {
    /// The names under `base`.
    ///
    /// # Errors
    ///
    /// Whatever [`ServiceNames::new`] refuses.
    pub fn new(base: &str) -> Result<Self, ContractError> {
        ServiceNames::new(base).map(Self)
    }

    /// The base name.
    #[must_use]
    pub fn base(&self) -> &str {
        self.0.base()
    }

    /// The full name of the service with `suffix`.
    #[must_use]
    pub fn name(&self, suffix: &str) -> String {
        self.0.name(suffix)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_base_names_every_service_under_it() {
        let names = ArmNames::default();
        assert_eq!(names.name(FEEDBACK), "omakase/arm/feedback");
        assert_eq!(names.name(COMMAND_STATUS), "omakase/arm/command_status");
        assert!(ArmNames::new("/omakase/arm").is_err());
    }
}
