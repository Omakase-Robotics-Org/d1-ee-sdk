//! The grip-force convention (`spec/end_effector.shm/1.md` §9): the
//! `force_percent` parameter a provider reads from a `set`'s `extra`, and
//! the `grip_force` report it puts in every state's detail `data`.

use serde::{Deserialize, Serialize};

use crate::{
    ErrorKind, ProviderError, SetCommand,
    contract::{FORCE_PERCENT, FORCE_PERCENT_MAX, FORCE_PERCENT_MIN},
};

/// How an end effector takes `force_percent`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GripForceMode {
    /// The provider maps `force_percent` onto an adjustable force.
    Adjustable,
    /// The end effector has no adjustable grip force: the provider accepted
    /// the percent and applied its fixed force.
    Fixed,
    /// The side's provider does not declare `force_percent`: the daemon
    /// dropped it and the provider applied its own default. Only the daemon
    /// reports this mode.
    NotTaken,
}

/// A native force unit of the spec's vocabulary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ForceUnit {
    /// A vendor register in percent of the device's own scale.
    PercentOfDeviceScale,
    /// Newton-metres.
    NewtonMetre,
    /// Newtons.
    Newton,
    /// Milliamperes.
    Milliampere,
    /// Amperes.
    Ampere,
    /// Per mille of the actuator's rated current.
    PermilleOfRatedCurrent,
}

/// A closed range of native force values.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ForceRange {
    /// The value `force_percent` 0 maps to.
    pub min: f64,
    /// The value `force_percent` 100 maps to.
    pub max: f64,
}

impl ForceRange {
    /// The native value `percent` names, linear over the range:
    /// `min + percent / 100 * (max - min)`. Not rounded; a device that takes
    /// integers rounds the result itself.
    #[must_use]
    pub fn at_percent(&self, percent: f64) -> f64 {
        self.min + percent / FORCE_PERCENT_MAX * (self.max - self.min)
    }

    fn is_finite(&self) -> bool {
        self.min.is_finite() && self.max.is_finite()
    }
}

/// The applied-force report, `detail.data.grip_force` of every state a
/// provider that declares `force_percent` publishes. Every key serializes,
/// an absent value as `null`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GripForce {
    /// How the end effector takes `force_percent`.
    pub mode: GripForceMode,
    /// The `force_percent` of the newest `set`; `None` when it named none or
    /// no `set` has arrived.
    pub requested_percent: Option<f64>,
    /// The force setting in effect, in `unit`, after rounding.
    pub applied_value: Option<f64>,
    /// The unit of `applied_value` and `range`; `None` exactly when both
    /// are.
    pub unit: Option<ForceUnit>,
    /// The native values `force_percent` 0 and 100 map to; only in mode
    /// [`GripForceMode::Adjustable`].
    pub range: Option<ForceRange>,
    /// The approximate grip force in newtons of the value in effect, only
    /// where the vendor's documentation gives a basis.
    pub approx_newtons: Option<f64>,
    /// The approximate newtons at `range.min` and `range.max`, by the same
    /// rule.
    pub approx_newtons_range: Option<ForceRange>,
}

impl GripForce {
    /// An adjustable force over `range`: `applied` (in `unit`) in effect,
    /// `None` while not known (before the first `set`), for the newest
    /// `set`'s `requested` percent. No newtons.
    #[must_use]
    pub const fn adjustable(
        requested: Option<f64>,
        applied: Option<f64>,
        unit: ForceUnit,
        range: ForceRange,
    ) -> Self {
        Self {
            mode: GripForceMode::Adjustable,
            requested_percent: requested,
            applied_value: applied,
            unit: Some(unit),
            range: Some(range),
            approx_newtons: None,
            approx_newtons_range: None,
        }
    }

    /// A fixed force, its value not published.
    #[must_use]
    pub const fn fixed(requested: Option<f64>) -> Self {
        Self {
            mode: GripForceMode::Fixed,
            requested_percent: requested,
            applied_value: None,
            unit: None,
            range: None,
            approx_newtons: None,
            approx_newtons_range: None,
        }
    }

    /// The report the daemon writes for a provider that does not declare
    /// `force_percent`.
    #[must_use]
    pub const fn not_taken(requested: Option<f64>) -> Self {
        Self {
            mode: GripForceMode::NotTaken,
            ..Self::fixed(requested)
        }
    }

    /// Check the report's own rules: every number finite, `unit` present
    /// exactly when `applied_value` or `range` is, `range` only in mode
    /// adjustable, the
    /// requested percent in `[0, 100]`, and nothing applied in mode
    /// not_taken.
    ///
    /// # Errors
    ///
    /// The first rule broken, in words.
    pub fn validate(&self) -> Result<(), String> {
        let finite = [
            self.requested_percent,
            self.applied_value,
            self.approx_newtons,
        ]
        .into_iter()
        .flatten()
        .all(f64::is_finite)
            && [self.range, self.approx_newtons_range]
                .iter()
                .flatten()
                .all(ForceRange::is_finite);
        if !finite {
            return Err("every number of a grip_force report must be finite".to_owned());
        }
        if let Some(percent) = self.requested_percent
            && !(FORCE_PERCENT_MIN..=FORCE_PERCENT_MAX).contains(&percent)
        {
            return Err(format!(
                "requested_percent {percent} is outside [{FORCE_PERCENT_MIN}, {FORCE_PERCENT_MAX}]"
            ));
        }
        if self.unit.is_some() != (self.applied_value.is_some() || self.range.is_some()) {
            return Err("unit is null exactly when applied_value and range are null".to_owned());
        }
        if self.range.is_some() && self.mode != GripForceMode::Adjustable {
            return Err("range is null unless mode is adjustable".to_owned());
        }
        if self.mode == GripForceMode::NotTaken && self.applied_value.is_some() {
            return Err("a not_taken report applies no value".to_owned());
        }
        Ok(())
    }
}

/// The `force_percent` a `set` carries in its `extra`, or `None` without
/// one.
///
/// # Errors
///
/// [`ErrorKind::Unprocessable`] for a value that is not a number, not
/// finite, or outside `[0, 100]`.
pub fn force_percent(command: &SetCommand) -> Result<Option<f64>, ProviderError> {
    let Some(value) = command
        .extra
        .as_ref()
        .and_then(|extra| extra.get(FORCE_PERCENT))
    else {
        return Ok(None);
    };
    let Some(percent) = value.as_f64() else {
        return Err(ProviderError::new(
            ErrorKind::Unprocessable,
            format!("{FORCE_PERCENT} must be a number, got {value}"),
        ));
    };
    check_force_percent(percent).map(Some)
}

/// `percent` when it is a finite number in `[0, 100]`.
///
/// # Errors
///
/// [`ErrorKind::Unprocessable`] otherwise.
pub fn check_force_percent(percent: f64) -> Result<f64, ProviderError> {
    if percent.is_finite() && (FORCE_PERCENT_MIN..=FORCE_PERCENT_MAX).contains(&percent) {
        Ok(percent)
    } else {
        Err(ProviderError::new(
            ErrorKind::Unprocessable,
            format!(
                "{FORCE_PERCENT} must be a finite number within [{FORCE_PERCENT_MIN}, {FORCE_PERCENT_MAX}], got {percent}"
            ),
        ))
    }
}

#[cfg(test)]
mod tests {
    use serde_json::{Map, Value, json};

    use super::*;

    fn with_extra(extra: Value) -> SetCommand {
        SetCommand {
            openness: 0.0,
            wait: true,
            grip: None,
            torque_nm: None,
            extra: Some(serde_json::from_value::<Map<String, Value>>(extra).unwrap()),
        }
    }

    #[test]
    fn the_report_round_trips_as_the_spec_example() {
        let report = GripForce {
            approx_newtons: Some(37.0),
            approx_newtons_range: Some(ForceRange {
                min: 30.0,
                max: 100.0,
            }),
            ..GripForce::adjustable(
                Some(10.0),
                Some(28.0),
                ForceUnit::PercentOfDeviceScale,
                ForceRange {
                    min: 20.0,
                    max: 100.0,
                },
            )
        };
        let text = r#"{"mode":"adjustable","requested_percent":10.0,"applied_value":28.0,"unit":"percent_of_device_scale","range":{"min":20.0,"max":100.0},"approx_newtons":37.0,"approx_newtons_range":{"min":30.0,"max":100.0}}"#;
        assert_eq!(serde_json::to_string(&report).unwrap(), text);
        assert_eq!(serde_json::from_str::<GripForce>(text).unwrap(), report);
        assert_eq!(report.validate(), Ok(()));
    }

    #[test]
    fn fixed_and_not_taken_serialize_every_key_as_null() {
        assert_eq!(
            serde_json::to_string(&GripForce::fixed(Some(40.0))).unwrap(),
            r#"{"mode":"fixed","requested_percent":40.0,"applied_value":null,"unit":null,"range":null,"approx_newtons":null,"approx_newtons_range":null}"#
        );
        assert_eq!(
            serde_json::to_string(&GripForce::not_taken(None)).unwrap(),
            r#"{"mode":"not_taken","requested_percent":null,"applied_value":null,"unit":null,"range":null,"approx_newtons":null,"approx_newtons_range":null}"#
        );
    }

    #[test]
    fn every_unit_token_is_the_spec_vocabulary() {
        let tokens: Vec<String> = [
            ForceUnit::PercentOfDeviceScale,
            ForceUnit::NewtonMetre,
            ForceUnit::Newton,
            ForceUnit::Milliampere,
            ForceUnit::Ampere,
            ForceUnit::PermilleOfRatedCurrent,
        ]
        .iter()
        .map(|unit| {
            serde_json::to_value(unit)
                .unwrap()
                .as_str()
                .unwrap()
                .to_owned()
        })
        .collect();
        let spec = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../spec/end_effector.shm/1.md"),
        )
        .unwrap();
        for token in &tokens {
            assert!(spec.contains(&format!("| `{token}` |")), "{token}");
        }
        assert!(serde_json::from_str::<ForceUnit>(r#""furlong""#).is_err());
    }

    #[test]
    fn a_report_breaking_a_rule_is_named() {
        let range = ForceRange {
            min: 0.0,
            max: 10.0,
        };
        let bad = [
            GripForce {
                applied_value: Some(f64::NAN),
                ..GripForce::adjustable(None, Some(1.0), ForceUnit::Newton, range)
            },
            GripForce {
                unit: None,
                ..GripForce::adjustable(None, Some(1.0), ForceUnit::Newton, range)
            },
            GripForce {
                unit: None,
                ..GripForce::adjustable(None, None, ForceUnit::Newton, range)
            },
            GripForce {
                unit: Some(ForceUnit::Newton),
                ..GripForce::fixed(None)
            },
            GripForce {
                range: Some(range),
                ..GripForce::fixed(None)
            },
            GripForce::fixed(Some(101.0)),
            GripForce {
                applied_value: Some(1.0),
                unit: Some(ForceUnit::Newton),
                ..GripForce::not_taken(None)
            },
        ];
        for report in bad {
            assert!(report.validate().is_err(), "{report:?}");
        }
        assert_eq!(GripForce::fixed(Some(0.0)).validate(), Ok(()));
        assert_eq!(
            GripForce::adjustable(None, None, ForceUnit::Newton, range).validate(),
            Ok(()),
            "an adjustable force before its first set names its unit and range"
        );
    }

    #[test]
    fn a_percent_maps_linearly_onto_the_native_range() {
        let range = ForceRange {
            min: 20.0,
            max: 100.0,
        };
        assert!((range.at_percent(0.0) - 20.0).abs() < 1e-12);
        assert!((range.at_percent(10.0) - 28.0).abs() < 1e-12);
        assert!((range.at_percent(100.0) - 100.0).abs() < 1e-12);
    }

    #[test]
    fn the_parse_accepts_the_range_and_refuses_everything_else() {
        let none = SetCommand {
            extra: None,
            ..with_extra(json!({}))
        };
        assert_eq!(force_percent(&none), Ok(None));
        assert_eq!(force_percent(&with_extra(json!({"speed": 3}))), Ok(None));
        for (value, expected) in [
            (json!(0), 0.0),
            (json!(0.0), 0.0),
            (json!(37.5), 37.5),
            (json!(100), 100.0),
        ] {
            assert_eq!(
                force_percent(&with_extra(json!({ FORCE_PERCENT: value }))),
                Ok(Some(expected)),
                "{value}"
            );
        }
        for value in [
            json!(-0.001),
            json!(100.001),
            json!(-1),
            json!(1e300),
            json!("50"),
            json!(null),
            json!(true),
            json!([50]),
            json!({"value": 50}),
        ] {
            assert_eq!(
                force_percent(&with_extra(json!({ FORCE_PERCENT: value })))
                    .map_err(|error| error.kind),
                Err(ErrorKind::Unprocessable),
                "{value}"
            );
        }
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                check_force_percent(value).map_err(|error| error.kind),
                Err(ErrorKind::Unprocessable)
            );
        }
    }

    #[test]
    fn check_set_applies_the_range_only_to_a_declared_key() {
        let undeclared = crate::TestProvider::gripper_descriptor();
        let mut declared = undeclared.clone();
        declared
            .capabilities
            .set_extra
            .push(FORCE_PERCENT.to_owned());
        let kind = |descriptor, extra| {
            crate::check_set(descriptor, &with_extra(extra)).map_err(|error| error.kind)
        };

        assert_eq!(kind(&declared, json!({ FORCE_PERCENT: 55 })), Ok(()));
        assert_eq!(
            kind(&declared, json!({ FORCE_PERCENT: 55, "speed": 2 })),
            Ok(())
        );
        assert_eq!(kind(&declared, json!({ "speed": 2 })), Ok(()));
        assert_eq!(
            kind(&declared, json!({ FORCE_PERCENT: 120 })),
            Err(ErrorKind::Unprocessable)
        );
        assert_eq!(
            kind(&declared, json!({ FORCE_PERCENT: "soft" })),
            Err(ErrorKind::Unprocessable)
        );
        // Undeclared, the key itself is refused, whatever its value.
        assert_eq!(
            kind(&undeclared, json!({ FORCE_PERCENT: 55 })),
            Err(ErrorKind::Unprocessable)
        );
    }
}
