//! Headers, and the checks a command passes before the daemon looks at what
//! it asks for.

use d1fw_ee_shm_contract::clock;
pub use d1fw_ee_shm_contract::model::{ContractError, new_instance_id};

use crate::{
    DEFAULT_COMMAND_MAX_AGE_MS, JOINTS, MAX_COMMAND_MAX_AGE_MS, SCHEMA_VERSION, SIDES, wire,
};

/// A header for a message sent now by `instance_id` as its `seq`-th on one
/// service.
#[must_use]
pub fn header(instance_id: u64, seq: u64) -> wire::Header {
    header_at(instance_id, seq, clock::monotonic_ns())
}

/// A header stamped `mono_ns`.
#[must_use]
pub const fn header_at(instance_id: u64, seq: u64, mono_ns: u64) -> wire::Header {
    wire::Header {
        schema_version: SCHEMA_VERSION,
        reserved: 0,
        instance_id,
        seq,
        mono_ns,
    }
}

/// Refuse a header of another schema version or with no sender.
///
/// # Errors
///
/// [`ContractError::SchemaVersion`], or [`ContractError::Invalid`] for an
/// `instance_id` of zero.
pub fn check_header(header: &wire::Header) -> Result<(), ContractError> {
    if header.schema_version != SCHEMA_VERSION {
        return Err(ContractError::SchemaVersion {
            got: header.schema_version,
        });
    }
    if header.instance_id == 0 {
        return Err(ContractError::Invalid(
            "instance_id 0 names no sender".to_owned(),
        ));
    }
    Ok(())
}

/// A command that passed [`check_command`]: which arms it addresses and
/// their targets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CheckedCommand {
    /// Whether arm A and arm B are addressed.
    pub sides: [bool; SIDES],
    /// The targets, degrees; an arm not addressed carries what the command
    /// carried, unread.
    pub target_deg: [[f64; JOINTS]; SIDES],
}

/// Why a command did not pass [`check_command`]: a `RESULT_*` code and a
/// sentence.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Refusal {
    /// `RESULT_*`.
    pub result: u32,
    /// What was wrong.
    pub message: String,
}

impl Refusal {
    /// A refusal with `result`.
    #[must_use]
    pub fn new(result: u32, message: impl Into<String>) -> Self {
        Self {
            result,
            message: message.into(),
        }
    }
}

/// The age limit a command asks for, nanoseconds: its `max_age_ns`, or the
/// default when it names none, never more than
/// [`MAX_COMMAND_MAX_AGE_MS`].
#[must_use]
pub fn command_max_age_ns(command: &wire::Command) -> u64 {
    let ceiling = MAX_COMMAND_MAX_AGE_MS * 1_000_000;
    match command.max_age_ns {
        0 => DEFAULT_COMMAND_MAX_AGE_MS * 1_000_000,
        age => age.min(ceiling),
    }
}

/// The checks the contract makes of every command before the daemon reads
/// what it asks for, against `now_ns` (`CLOCK_MONOTONIC`): the header, the
/// side mask, the flags, the targets' finiteness, and the age.
///
/// The lease, the latch, the arms' mode and the motion guard are the
/// daemon's checks, made after these.
///
/// # Errors
///
/// [`wire::RESULT_INVALID`] or [`wire::RESULT_STALE`], with the reason.
pub fn check_command(command: &wire::Command, now_ns: u64) -> Result<CheckedCommand, Refusal> {
    check_header(&command.header)
        .map_err(|error| Refusal::new(wire::RESULT_INVALID, error.to_string()))?;
    if command.sides == 0 || command.sides & !wire::SIDE_MASK_ALL != 0 {
        return Err(Refusal::new(
            wire::RESULT_INVALID,
            format!(
                "sides {:#x} must name arm A (bit 0), arm B (bit 1) or both, and nothing else",
                command.sides
            ),
        ));
    }
    if command.flags != 0 {
        return Err(Refusal::new(
            wire::RESULT_INVALID,
            format!(
                "flags {:#x}: schema 1 defines no command flag",
                command.flags
            ),
        ));
    }
    let sides = [
        command.sides & wire::SIDE_MASK_A != 0,
        command.sides & wire::SIDE_MASK_B != 0,
    ];
    for (index, side) in sides.iter().enumerate() {
        if *side
            && !command.target_deg[index]
                .iter()
                .all(|value| value.is_finite())
        {
            return Err(Refusal::new(
                wire::RESULT_INVALID,
                format!(
                    "arm {}'s target is not finite",
                    if index == 0 { "a" } else { "b" }
                ),
            ));
        }
    }
    let age = now_ns.saturating_sub(command.header.mono_ns);
    let limit = command_max_age_ns(command);
    if age > limit {
        return Err(Refusal::new(
            wire::RESULT_STALE,
            format!(
                "the command is {} µs old, over its {} µs limit",
                age / 1_000,
                limit / 1_000
            ),
        ));
    }
    Ok(CheckedCommand {
        sides,
        target_deg: command.target_deg,
    })
}

/// The lease token a `POST /v1/arm/lease` response carries (16 hexadecimal
/// digits), as the `u64` a command carries.
///
/// # Errors
///
/// [`ContractError::Invalid`] when `token` is not 1 to 16 hexadecimal
/// digits, or is zero.
pub fn parse_lease_token(token: &str) -> Result<u64, ContractError> {
    let token = token.trim();
    let value = (!token.is_empty() && token.len() <= 16)
        .then(|| u64::from_str_radix(token, 16).ok())
        .flatten()
        .filter(|value| *value != 0);
    value.ok_or_else(|| {
        ContractError::Invalid(format!(
            "lease token {token:?} is not 1 to 16 hexadecimal digits naming a non-zero value"
        ))
    })
}

/// The text form of a lease token, as REST carries it.
#[must_use]
pub fn format_lease_token(token: u64) -> String {
    format!("{token:016x}")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn command(now: u64) -> wire::Command {
        let mut command = wire::Command::zeroed();
        command.header = header_at(7, 1, now);
        command.lease_token = 1;
        command.sides = wire::SIDE_MASK_ALL;
        command
    }

    #[test]
    fn a_well_formed_command_passes() {
        let checked = check_command(&command(1_000), 1_000).unwrap();
        assert_eq!(checked.sides, [true, true]);
    }

    #[test]
    fn malformed_commands_are_invalid() {
        let now = 10_000_000;
        let mut bad = command(now);
        bad.sides = 0;
        assert_eq!(
            check_command(&bad, now).unwrap_err().result,
            wire::RESULT_INVALID
        );
        let mut bad = command(now);
        bad.sides = 4;
        assert_eq!(
            check_command(&bad, now).unwrap_err().result,
            wire::RESULT_INVALID
        );
        let mut bad = command(now);
        bad.flags = 1;
        assert_eq!(
            check_command(&bad, now).unwrap_err().result,
            wire::RESULT_INVALID
        );
        let mut bad = command(now);
        bad.target_deg[1][3] = f64::NAN;
        assert_eq!(
            check_command(&bad, now).unwrap_err().result,
            wire::RESULT_INVALID
        );
        let mut bad = command(now);
        bad.header.schema_version = 2;
        assert_eq!(
            check_command(&bad, now).unwrap_err().result,
            wire::RESULT_INVALID
        );
        // A non-finite target on an arm the command does not address is not
        // read.
        let mut fine = command(now);
        fine.sides = wire::SIDE_MASK_A;
        fine.target_deg[1][3] = f64::NAN;
        assert!(check_command(&fine, now).is_ok());
    }

    #[test]
    fn an_old_command_is_stale_against_its_own_or_the_default_limit() {
        let sent = 1_000_000_000;
        let fresh = sent + DEFAULT_COMMAND_MAX_AGE_MS * 1_000_000;
        assert!(check_command(&command(sent), fresh).is_ok());
        assert_eq!(
            check_command(&command(sent), fresh + 1).unwrap_err().result,
            wire::RESULT_STALE
        );
        let mut patient = command(sent);
        patient.max_age_ns = 10 * DEFAULT_COMMAND_MAX_AGE_MS * 1_000_000;
        assert!(check_command(&patient, fresh + 1).is_ok());
        patient.max_age_ns = u64::MAX;
        assert_eq!(
            command_max_age_ns(&patient),
            MAX_COMMAND_MAX_AGE_MS * 1_000_000
        );
    }

    #[test]
    fn lease_tokens_round_trip_as_sixteen_hex_digits() {
        let token = 0x00ab_cdef_0123_4567;
        let text = format_lease_token(token);
        assert_eq!(text, "00abcdef01234567");
        assert_eq!(parse_lease_token(&text).unwrap(), token);
        assert!(parse_lease_token("0").is_err());
        assert!(parse_lease_token("").is_err());
        assert!(parse_lease_token("xyz").is_err());
        assert!(parse_lease_token("00000000000000001").is_err());
    }

    #[test]
    fn text_is_cut_at_a_character_boundary() {
        let mut field = [0_u8; 4];
        wire::write_text(&mut field, "aéé");
        assert_eq!(wire::read_text(&field), Some("aé"));
    }
}
