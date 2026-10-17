//! The lifetime of a hold and of a stop's pose (§4 rules 4 and 10): the
//! declared durations and their caps, the advance notices, the settle
//! definition, and the report a provider carries in its state's `detail`.
//!
//! None of this is a layout field: `end_effector.shm/1` is frozen, so the
//! declaration travels in the `hold` key of the state's `detail` `data`
//! object, which the daemon reads as opaque JSON and a client reads under
//! the provider's `detail_schema`.

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::model::{ContractError, Joint};

/// The key of the `detail` `data` object that carries a [`HoldReport`].
pub const HOLD_DETAIL_KEY: &str = "hold";

/// The longest `hold_timeout_ms` a provider may declare: 5 minutes.
///
/// A hold keeps actuators energised at a grasp effort. Five minutes covers
/// carrying an object to where it is put down and is well inside the
/// continuous stall rating of the small actuators of a hand, while a hold
/// an operator forgot ends by itself instead of heating the hand until a
/// thermal fault. A longer hold is made by sending `hold` again, which
/// restarts the lifetime.
pub const MAX_HOLD_TIMEOUT_MS: u32 = 300_000;

/// The longest `stop_hold_ms` a provider may declare: 3 seconds.
///
/// After a stop a back-drivable hand keeps its pose at reduced effort so the
/// object is not dropped while the arm comes to rest and a person reaches
/// it. Three seconds covers that; a longer stop would leave actuators
/// energised after the operator was told everything stopped.
pub const MAX_STOP_HOLD_MS: u32 = 3_000;

/// How long before a hold expires the provider gives the advance notice,
/// milliseconds. A hold shorter than this is noticed when it starts.
pub const HOLD_NOTICE_LEAD_MS: u32 = 1_000;

/// How long before a stop's pose is relaxed the provider gives the advance
/// notice, milliseconds. A stop pose shorter than this is noticed when the
/// stop arrives.
pub const STOP_NOTICE_LEAD_MS: u32 = 500;

/// The most events a report carries; the oldest is dropped first.
pub const MAX_HOLD_EVENTS: usize = 8;

/// A joint is settled when its measured position is within this fraction of
/// its range of its set-point (`Joint::max - Joint::min`).
pub const SETTLE_POSITION_FRACTION: f64 = 0.02;

/// A joint is settled when its measured speed is below this fraction of its
/// range per second.
pub const SETTLE_SPEED_FRACTION_PER_S: f64 = 0.05;

/// An end effector is settled when every joint has been settled for this
/// long, milliseconds. Only then does a provider clear `tracking`.
pub const SETTLE_DWELL_MS: u32 = 100;

/// The durations a provider declares for its holds.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HoldLifetime {
    /// How long a hold lasts before the provider treats it as a stop,
    /// milliseconds, `1..=`[`MAX_HOLD_TIMEOUT_MS`]. A provider that lists
    /// `hold` MUST declare it. A new `hold` restarts it.
    pub hold_timeout_ms: Option<u32>,
    /// How long a back-drivable multi-finger hand keeps its pose at reduced
    /// effort after a stop before it relaxes, milliseconds,
    /// `1..=`[`MAX_STOP_HOLD_MS`]. `None` (or a hand that is not
    /// back-drivable) relaxes at once.
    pub stop_hold_ms: Option<u32>,
}

impl HoldLifetime {
    /// Refuse a declaration beyond the caps.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] naming the first value out of range.
    pub fn validate(&self) -> Result<(), ContractError> {
        for (name, value, cap) in [
            ("hold_timeout_ms", self.hold_timeout_ms, MAX_HOLD_TIMEOUT_MS),
            ("stop_hold_ms", self.stop_hold_ms, MAX_STOP_HOLD_MS),
        ] {
            if let Some(value) = value
                && !(1..=cap).contains(&value)
            {
                return Err(ContractError::Invalid(format!(
                    "{name} {value} is outside 1..={cap}"
                )));
            }
        }
        Ok(())
    }
}

/// What a pose is in right now.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoldPhase {
    /// No hold and no stop pose: the actuators follow their own control or
    /// are relaxed.
    #[default]
    None,
    /// A hold is keeping the last set-point (§4 rule 10).
    Held,
    /// A stop keeps the pose at reduced effort (§4 rule 4).
    StopHolding,
}

/// What happened to a hold or a stop pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoldEventKind {
    /// The hold will expire within [`HOLD_NOTICE_LEAD_MS`].
    HoldExpiring,
    /// The hold expired and the provider treated it as a stop.
    HoldExpired,
    /// The stop's pose will be relaxed within [`STOP_NOTICE_LEAD_MS`].
    StopRelaxing,
    /// The stop's pose was relaxed: the actuators are no longer holding.
    StopRelaxed,
}

impl HoldEventKind {
    /// The notices, which come before the thing they announce.
    #[must_use]
    pub fn is_notice(self) -> bool {
        matches!(self, Self::HoldExpiring | Self::StopRelaxing)
    }
}

/// One event of a hold's life, in the provider's own log and in its report
/// so a dropped object can be traced to the moment its hold ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HoldEvent {
    /// What happened.
    pub kind: HoldEventKind,
    /// `CLOCK_MONOTONIC` nanoseconds when the provider recorded it.
    pub mono_ns: u64,
    /// For a notice, how long until the event it announces, milliseconds;
    /// zero otherwise.
    pub remaining_ms: u32,
}

/// The report a provider carries under [`HOLD_DETAIL_KEY`].
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HoldReport {
    /// The declared durations.
    #[serde(flatten)]
    pub lifetime: HoldLifetime,
    /// What the pose is in now.
    pub phase: HoldPhase,
    /// The last events, oldest first, at most [`MAX_HOLD_EVENTS`].
    #[serde(default)]
    pub events: Vec<HoldEvent>,
}

impl HoldReport {
    /// Append `event`, dropping the oldest beyond [`MAX_HOLD_EVENTS`].
    pub fn record(&mut self, event: HoldEvent) {
        self.events.push(event);
        let excess = self.events.len().saturating_sub(MAX_HOLD_EVENTS);
        self.events.drain(..excess);
    }

    /// The report as the `detail` `data` fragment under
    /// [`HOLD_DETAIL_KEY`].
    #[must_use]
    pub fn to_value(&self) -> Value {
        serde_json::to_value(self).expect("a report serialises")
    }

    /// Read the report out of a `detail` `data` object: `Ok(None)` when it
    /// carries none.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] when the key is present but is not a
    /// report within the caps.
    pub fn from_detail_data(data: &Value) -> Result<Option<Self>, ContractError> {
        let Some(value) = data.get(HOLD_DETAIL_KEY) else {
            return Ok(None);
        };
        let report: Self = serde_json::from_value(value.clone())
            .map_err(|error| ContractError::Invalid(format!("detail.data.hold: {error}")))?;
        report.lifetime.validate()?;
        if report.events.len() > MAX_HOLD_EVENTS {
            return Err(ContractError::Invalid(format!(
                "detail.data.hold carries {} events; at most {MAX_HOLD_EVENTS}",
                report.events.len()
            )));
        }
        Ok(Some(report))
    }
}

impl Joint {
    /// Whether this joint is settled at `set_point` (§3.4): within
    /// [`SETTLE_POSITION_FRACTION`] of its range of it and, when its speed
    /// is known, slower than [`SETTLE_SPEED_FRACTION_PER_S`] of its range
    /// per second. A joint whose speed is unknown is judged on position
    /// alone. The caller requires [`SETTLE_DWELL_MS`] of it.
    #[must_use]
    pub fn settled(&self, measured: f64, set_point: f64, speed: Option<f64>) -> bool {
        let range = self.max - self.min;
        (measured - set_point).abs() <= SETTLE_POSITION_FRACTION * range
            && speed.is_none_or(|v| v.abs() <= SETTLE_SPEED_FRACTION_PER_S * range)
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;
    use crate::model::JointUnit;

    #[test]
    fn caps_are_enforced() {
        let ok = HoldLifetime {
            hold_timeout_ms: Some(MAX_HOLD_TIMEOUT_MS),
            stop_hold_ms: Some(MAX_STOP_HOLD_MS),
        };
        ok.validate().expect("the caps themselves are allowed");
        for lifetime in [
            HoldLifetime {
                hold_timeout_ms: Some(0),
                stop_hold_ms: None,
            },
            HoldLifetime {
                hold_timeout_ms: Some(MAX_HOLD_TIMEOUT_MS + 1),
                stop_hold_ms: None,
            },
            HoldLifetime {
                hold_timeout_ms: None,
                stop_hold_ms: Some(MAX_STOP_HOLD_MS + 1),
            },
        ] {
            assert!(lifetime.validate().is_err(), "{lifetime:?}");
        }
    }

    #[test]
    fn a_report_round_trips_through_detail_data() {
        let mut report = HoldReport {
            lifetime: HoldLifetime {
                hold_timeout_ms: Some(60_000),
                stop_hold_ms: Some(2_000),
            },
            phase: HoldPhase::Held,
            events: Vec::new(),
        };
        report.record(HoldEvent {
            kind: HoldEventKind::HoldExpiring,
            mono_ns: 7,
            remaining_ms: 1_000,
        });
        let data = json!({"other": 1, HOLD_DETAIL_KEY: report.to_value()});
        assert_eq!(
            HoldReport::from_detail_data(&data).expect("valid"),
            Some(report)
        );
        assert_eq!(
            HoldReport::from_detail_data(&json!({})).expect("absent"),
            None
        );
    }

    #[test]
    fn a_report_beyond_the_caps_or_with_unknown_keys_is_refused() {
        for hold in [
            json!({"hold_timeout_ms": 300_001, "phase": "held"}),
            json!({"stop_hold_ms": 3_001, "phase": "none"}),
            json!({"phase": "held", "surprise": true}),
            json!({"phase": "levitating"}),
        ] {
            assert!(
                HoldReport::from_detail_data(&json!({ HOLD_DETAIL_KEY: hold })).is_err(),
                "{hold}"
            );
        }
    }

    #[test]
    fn the_event_list_keeps_the_newest() {
        let mut report = HoldReport::default();
        for n in 0..=MAX_HOLD_EVENTS as u64 {
            report.record(HoldEvent {
                kind: HoldEventKind::StopRelaxed,
                mono_ns: n,
                remaining_ms: 0,
            });
        }
        assert_eq!(report.events.len(), MAX_HOLD_EVENTS);
        assert_eq!(report.events[0].mono_ns, 1);
    }

    #[test]
    fn settled_needs_position_and_speed() {
        let joint = Joint {
            name: "j".to_owned(),
            unit: JointUnit::Rad,
            min: 0.0,
            max: 2.0,
        };
        assert!(joint.settled(1.0, 1.03, Some(0.05)));
        assert!(!joint.settled(1.0, 1.05, Some(0.0)), "2 % of 2.0 is 0.04");
        assert!(!joint.settled(1.0, 1.0, Some(0.11)), "5 % of 2.0 is 0.1/s");
        assert!(
            joint.settled(1.0, 1.0, None),
            "unknown speed: position only"
        );
    }
}
