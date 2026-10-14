//! What the host answers a refused or failed command with, and the HTTP
//! status d1-firmwared's REST surface answers the same refusal with.

use std::fmt;

use d1fw_ee_shm_contract::model::{ErrorKind, ProviderError};

/// Why a command did not complete.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostErrorKind {
    /// The request itself is malformed.
    Invalid,
    /// A value the end effector cannot take: an openness outside `[0, 1]`,
    /// an undeclared grip, torque or `extra` key, a joint target out of its
    /// limits.
    Unprocessable,
    /// A verb the declaration does not list, or a refusal of the provider's.
    Refused,
    /// A `joints` timestamp not newer than the newest accepted one.
    Stale,
    /// The provider did not answer in time.
    Timeout,
    /// No provider is available on the side, or it changed under the
    /// request.
    Unavailable,
    /// The device failed, or the provider answered something malformed.
    Device,
}

impl HostErrorKind {
    /// The HTTP status d1-firmwared answers this refusal with.
    #[must_use]
    pub const fn http_status(self) -> u16 {
        match self {
            Self::Invalid => 400,
            Self::Unprocessable => 422,
            Self::Refused | Self::Stale => 409,
            Self::Timeout => 504,
            Self::Unavailable | Self::Device => 502,
        }
    }
}

/// A refused or failed command, with the reason.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HostError {
    /// What kind of refusal.
    pub kind: HostErrorKind,
    /// Why.
    pub message: String,
}

impl HostError {
    /// A refusal of `kind` for `message`.
    #[must_use]
    pub fn new(kind: HostErrorKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
        }
    }

    /// The HTTP status d1-firmwared answers this refusal with.
    #[must_use]
    pub const fn http_status(&self) -> u16 {
        self.kind.http_status()
    }

    pub(crate) fn unavailable(message: impl Into<String>) -> Self {
        Self::new(HostErrorKind::Unavailable, message)
    }

    /// The host's error for a provider's refusal or failure on `service`, as
    /// d1-firmwared maps it.
    #[must_use]
    pub fn from_provider(service: &str, error: ProviderError) -> Self {
        let message = format!("end-effector provider on {service}: {}", error.message);
        match error.kind {
            ErrorKind::Refused => Self::new(HostErrorKind::Refused, message),
            ErrorKind::Unprocessable => Self::new(HostErrorKind::Unprocessable, message),
            ErrorKind::Unavailable => Self::new(HostErrorKind::Unavailable, message),
            ErrorKind::Device => Self::new(HostErrorKind::Device, message),
            ErrorKind::Timeout => Self::new(HostErrorKind::Timeout, message),
            // Not executed: the provider restarted or re-declared between the
            // validation and the request's arrival, or the request reached it
            // too late to start. A retry is valid.
            ErrorKind::StaleTarget => Self::new(
                HostErrorKind::Unavailable,
                format!("{message} (not executed; the provider changed or the request was late)"),
            ),
            // The host built the request, so a malformed one is a defect on
            // either side of the contract, not the caller's.
            ErrorKind::Invalid => Self::new(
                HostErrorKind::Device,
                format!("{message} (the provider refused the request as malformed)"),
            ),
        }
    }
}

impl fmt::Display for HostError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{:?}: {}", self.kind, self.message)
    }
}

impl std::error::Error for HostError {}

/// The HTTP status of a command's outcome: 200 for success, the refusal's
/// status otherwise.
#[must_use]
pub fn status<T>(result: &Result<T, HostError>) -> u16 {
    result.as_ref().map_or_else(HostError::http_status, |_| 200)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn provider_errors_keep_their_kind_and_status() {
        let map = |kind| HostError::from_provider("omakase/ee/a", ProviderError::new(kind, "why"));
        assert_eq!(map(ErrorKind::Refused).http_status(), 409);
        assert_eq!(map(ErrorKind::Unprocessable).http_status(), 422);
        assert_eq!(map(ErrorKind::Timeout).http_status(), 504);
        assert_eq!(map(ErrorKind::StaleTarget).kind, HostErrorKind::Unavailable);
        assert_eq!(map(ErrorKind::Invalid).kind, HostErrorKind::Device);
        assert_eq!(status::<()>(&Ok(())), 200);
    }
}
