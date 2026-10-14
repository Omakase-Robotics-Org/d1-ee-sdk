//! The service names of one side.
//!
//! A side is bound by a base name (`omakase/ee/a` by default); its eight
//! services are the base followed by `/` and a fixed suffix. The base is
//! what the daemon's configuration and the provider's command line name, so
//! two end effectors on one host differ only in it.

use crate::model::ContractError;

/// The longest base name accepted, in bytes. The longest suffix
/// (`/descriptor`) still leaves the full name far below iceoryx2's 255-byte
/// service-name limit.
pub const MAX_BASE_LEN: usize = 128;

/// The base name a side is bound to unless its configuration names another.
#[must_use]
pub fn default_base(side: &str) -> String {
    format!("omakase/ee/{side}")
}

/// Suffix of the provider's descriptor topic.
pub const DESCRIPTOR: &str = "descriptor";
/// Suffix of the provider's heartbeat topic.
pub const HEARTBEAT: &str = "heartbeat";
/// Suffix of the provider's state topic.
pub const STATE: &str = "state";
/// Suffix of the provider's tick topic.
pub const TICK: &str = "tick";
/// Suffix of the daemon's joint-target topic.
pub const JOINTS: &str = "joints";
/// Suffix of the daemon's control topic.
pub const CONTROL: &str = "control";
/// Suffix of the `set` / `stop` request-response service.
pub const COMMAND: &str = "command";
/// Suffix of the `clear_fault` / `registers` request-response service.
pub const FAULT: &str = "fault";

/// The eight service names of one side.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ServiceNames {
    base: String,
}

impl ServiceNames {
    /// The names under `base`.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] when `base` is empty, longer than
    /// [`MAX_BASE_LEN`], starts or ends with `/`, contains an empty segment,
    /// or uses a character outside `a-z`, `0-9`, `_`, `-`, `.` and `/`.
    pub fn new(base: &str) -> Result<Self, ContractError> {
        let refuse = |why: &str| {
            Err(ContractError::Invalid(format!(
                "shared-memory service base {base:?} {why}"
            )))
        };
        if base.is_empty() || base.len() > MAX_BASE_LEN {
            return refuse(&format!("must be 1 to {MAX_BASE_LEN} bytes"));
        }
        if base.split('/').any(str::is_empty) {
            return refuse("must not start or end with '/' or contain an empty segment");
        }
        if !base.bytes().all(|byte| {
            byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(byte, b'_' | b'-' | b'.' | b'/')
        }) {
            return refuse("may use only a-z, 0-9, '_', '-', '.' and '/'");
        }
        Ok(Self {
            base: base.to_owned(),
        })
    }

    /// The base name.
    #[must_use]
    pub fn base(&self) -> &str {
        &self.base
    }

    /// The full name of the service with `suffix`.
    #[must_use]
    pub fn name(&self, suffix: &str) -> String {
        format!("{}/{suffix}", self.base)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_base_names_eight_services_under_it() {
        let names = ServiceNames::new(&default_base("a")).unwrap();
        assert_eq!(names.name(DESCRIPTOR), "omakase/ee/a/descriptor");
        assert_eq!(names.name(FAULT), "omakase/ee/a/fault");
    }

    #[test]
    fn a_malformed_base_is_refused() {
        for base in [
            "",
            "/omakase/ee/a",
            "omakase/ee/a/",
            "omakase//a",
            "Omakase/ee",
            "omakase ee",
        ] {
            assert!(ServiceNames::new(base).is_err(), "{base:?}");
        }
        assert!(ServiceNames::new(&"a".repeat(MAX_BASE_LEN + 1)).is_err());
        assert!(ServiceNames::new("vendor-x/ee.v2/left_1").is_ok());
    }
}
