//! The `hand.state/1` detail a hand provider publishes is the one the
//! committed schema (`schema/hand.state-1.json`) describes: the same fields,
//! every one of the documented type. The daemon's telemetry decodes this
//! detail by its schema id, so a field renamed or dropped here is a hand
//! metric gone dark.

use std::collections::BTreeSet;

use d1fw_ee_hand::{HAND_STATE_SCHEMA, HandSample, HandState};
use d1fw_ee_provider::{ForceRange, ForceUnit, GripForce};
use serde_json::{Value, json};

fn documented() -> Value {
    serde_json::from_str(HAND_STATE_SCHEMA).expect("the committed schema is JSON")
}

fn errors(schema: &Value, data: &Value) -> Vec<String> {
    let validator = jsonschema::validator_for(schema).expect("a usable schema");
    validator
        .iter_errors(data)
        .map(|error| error.to_string())
        .collect()
}

#[test]
fn a_published_hand_state_is_the_documented_one() {
    let schema = documented();
    assert!(schema.is_object(), "the schema is an object");
    let mut state = HandState::live(
        "maker/hand",
        "a",
        HandSample {
            positions: vec![0.0, 0.5],
            positions_wire: vec![0, 128],
            enabled: vec![true, false],
            error_code: 7,
            faults: vec!["over current".to_owned()],
        },
        &["command-priority"],
    );
    state.grip_force = Some(GripForce::adjustable(
        Some(25.0),
        Some(750.0),
        ForceUnit::PermilleOfRatedCurrent,
        ForceRange {
            min: 0.0,
            max: 3000.0,
        },
    ));
    let detail = state.detail();
    assert_eq!(detail.schema, "hand.state/1");

    let published: BTreeSet<String> = detail
        .data
        .as_object()
        .expect("an object")
        .keys()
        .cloned()
        .collect();
    let described: BTreeSet<String> = schema["properties"]
        .as_object()
        .expect("documented properties")
        .keys()
        .cloned()
        .collect();
    assert_eq!(
        published, described,
        "the published fields are the documented ones"
    );

    let errors = errors(&schema, &detail.data);
    assert!(errors.is_empty(), "{errors:?}");
}

/// `grip_force` is an additive, optional property: a state without it (a
/// hand provider built before the convention) is still a valid one, and
/// every report the SDK can build validates.
#[test]
fn the_grip_force_report_is_optional_and_documented() {
    let schema = documented();
    let mut state = HandState::unsampled("maker/hand", "a", 2, &[]);
    let without = state.detail().data;
    assert!(without.get("grip_force").is_none());
    assert!(errors(&schema, &without).is_empty());
    assert!(
        !schema["required"]
            .as_array()
            .expect("a required list")
            .contains(&json!("grip_force"))
    );

    for report in [
        GripForce::fixed(None),
        GripForce::fixed(Some(0.0)),
        GripForce::not_taken(Some(100.0)),
        GripForce::adjustable(
            None,
            None,
            ForceUnit::Milliampere,
            ForceRange {
                min: 10.0,
                max: 50.0,
            },
        ),
        GripForce {
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
        },
    ] {
        state.grip_force = Some(report);
        let data = state.detail().data;
        let found = errors(&schema, &data);
        assert!(found.is_empty(), "{report:?}: {found:?}");
    }

    let mut data = state.detail().data;
    for (key, bad) in [
        ("mode", json!("bogus")),
        ("unit", json!("furlong")),
        ("requested_percent", json!(150.0)),
        ("range", json!({"min": 1.0})),
    ] {
        let good = data["grip_force"][key].clone();
        data["grip_force"][key] = bad;
        assert!(!errors(&schema, &data).is_empty(), "{key}");
        data["grip_force"][key] = good;
    }
    data["grip_force"]
        .as_object_mut()
        .expect("an object")
        .remove("applied_value");
    assert!(
        !errors(&schema, &data).is_empty(),
        "every key of a report is present"
    );
}
