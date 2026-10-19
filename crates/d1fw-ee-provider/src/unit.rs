//! How a provider runs on a D1: the systemd template unit d1-firmwared
//! installs, and where a provider package puts its program and its
//! configuration.
//!
//! - The unit `d1-firmwared-ee-provider@<side>` runs
//!   `EE_PROVIDER_COMMAND` from [`PROVIDER_CONFIG_DIR`]`/<side>.env` as the
//!   daemon's account, starts and stops with the daemon, and is skipped
//!   while the command's program is not installed ([`PROVIDER_UNIT`]).
//! - A provider package installs its program as
//!   `/opt/<package>/bin/<program>` ([`program_path`]) and writes no
//!   configuration and enables nothing: which side carries which end
//!   effector is the robot's commissioning (provisioning, or
//!   `d1-firmwared config ee-providers` for a model the daemon knows).
//! - A provider that reads a configuration file of its own reads
//!   [`PROVIDER_CONFIG_DIR`]`/<side>.toml`.

/// The text of the template unit `d1-firmwared-ee-provider@.service`, as
/// committed in this repository's `deploy/systemd/`. d1-firmwared's package
/// ships this text; its test holds the packaged copy equal to it.
pub const PROVIDER_UNIT: &str =
    include_str!("../../../deploy/systemd/d1-firmwared-ee-provider@.service");

/// The directory of each side's provider files: `<side>.env` (the unit's
/// `EE_PROVIDER_COMMAND`) and, for a provider that reads one, `<side>.toml`.
pub const PROVIDER_CONFIG_DIR: &str = "/etc/d1-firmwared/ee-provider";

/// Where the package `package` installs its provider program `program`.
#[must_use]
pub fn program_path(package: &str, program: &str) -> String {
    format!("/opt/{package}/bin/{program}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_unit_runs_the_sides_command_as_the_daemons_account() {
        for line in [
            "User=doraemon",
            "EnvironmentFile=/etc/d1-firmwared/ee-provider/%i.env",
            "ExecStart=/bin/sh -c 'exec $$EE_PROVIDER_COMMAND'",
            "ExecCondition=/bin/sh -c 'set -- $$EE_PROVIDER_COMMAND; test -x \"$$1\"'",
            "Requires=d1-firmwared.service",
            "WantedBy=d1-firmwared.service",
            "ReadWritePaths=/run/d1-firmwared",
        ] {
            assert!(
                PROVIDER_UNIT.lines().any(|unit| unit == line),
                "the unit lacks {line}"
            );
        }
        assert_eq!(
            program_path("d1-ee-damiao-custom", "d1-ee-damiao-custom-provider"),
            "/opt/d1-ee-damiao-custom/bin/d1-ee-damiao-custom-provider"
        );
    }
}
