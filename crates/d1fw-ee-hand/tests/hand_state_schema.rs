//! The `hand.state/1` detail a hand provider publishes is the one the
//! committed schema (`schema/hand.state-1.json`) describes: the same fields,
//! every one of the documented type. The daemon's telemetry decodes this
//! detail by its schema id, so a field renamed or dropped here is a hand
//! metric gone dark.

use std::collections::BTreeSet;

use d1fw_ee_hand::{HAND_STATE_SCHEMA, HandSample, HandState};
use serde_json::Value;

fn documented() -> Value {
    serde_json::from_str(HAND_STATE_SCHEMA).expect("the committed schema is JSON")
}

#[test]
fn a_published_hand_state_is_the_documented_one() {
    let schema = documented();
    assert!(schema.is_object(), "the schema is an object");
    let state = HandState::live(
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

    let validator = jsonschema::validator_for(&schema).expect("a usable schema");
    let errors: Vec<String> = validator
        .iter_errors(&detail.data)
        .map(|error| error.to_string())
        .collect();
    assert!(errors.is_empty(), "{errors:?}");
}
