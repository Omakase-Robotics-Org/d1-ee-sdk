//! The tool a provider declares, read from its `manipulation-kit` tool
//! configuration.
//!
//! A provider compiles in the kit's `config/tool_configs/<model>.json` (a
//! byte-for-byte copy under its own `assets/tool_configs/`, which
//! the provider's repository holds equal to the kit's file)
//! and declares its registration as the descriptor's `tool`; the daemon
//! registers it on the side's arm when it accepts the declaration.

use crate::Tool;
use serde::Deserialize;

/// One `omakase_hands.tool_config.v1` document. Only the fields the
/// registration needs are read; `_comment`, `units`, `provisional` and
/// `unmeasured` are provenance for a person.
#[derive(Debug, Deserialize)]
struct ToolDocument {
    /// The `"<maker>/<model>"` id.
    model: String,
    /// `[x, y, z]` mm then `[r, p, y]` deg, XYZ order.
    kinematics: [f64; 6],
    /// `[mass kg, com x, com y, com z (mm), Ixx, Ixy, Ixz, Iyy, Iyz, Izz]`.
    dynamics: [f64; 10],
}

/// The tool `document` (a tool configuration's JSON) describes for `model`.
///
/// # Panics
///
/// When the document does not parse, names another model, or describes a
/// tool the contract refuses: the document is compiled in, so that is a
/// broken build, not a condition a caller could handle.
#[must_use]
pub fn tool_from_config(model: &str, document: &str) -> Tool {
    let document: ToolDocument = serde_json::from_str(document)
        .unwrap_or_else(|error| panic!("the tool configuration of {model} must parse: {error}"));
    assert_eq!(
        document.model, model,
        "the tool configuration compiled in for {model} names another model"
    );
    let kinematics = document.kinematics;
    let dynamics = document.dynamics;
    let tool = Tool {
        mass_kg: dynamics[0],
        com_mm: [dynamics[1], dynamics[2], dynamics[3]],
        inertia: [
            dynamics[4],
            dynamics[5],
            dynamics[6],
            dynamics[7],
            dynamics[8],
            dynamics[9],
        ],
        tcp_xyz_mm: [kinematics[0], kinematics[1], kinematics[2]],
        tcp_rpy_deg: [kinematics[3], kinematics[4], kinematics[5]],
    };
    tool.validate()
        .unwrap_or_else(|error| panic!("the tool configuration of {model}: {error}"));
    tool
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_configuration_maps_onto_the_registration_fields() {
        let tool = tool_from_config(
            "maker/hand",
            r#"{"model": "maker/hand", "kinematics": [1, 2, 3, 4, 5, 6],
                "dynamics": [0.5, 7, 8, 9, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]}"#,
        );
        assert_eq!(tool.mass_kg, 0.5);
        assert_eq!(tool.com_mm, [7.0, 8.0, 9.0]);
        assert_eq!(tool.inertia, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]);
        assert_eq!(tool.tcp_xyz_mm, [1.0, 2.0, 3.0]);
        assert_eq!(tool.tcp_rpy_deg, [4.0, 5.0, 6.0]);
    }

    #[test]
    #[should_panic(expected = "names another model")]
    fn a_configuration_of_another_model_is_a_broken_build() {
        let _ = tool_from_config(
            "maker/hand",
            r#"{"model": "maker/other", "kinematics": [0, 0, 0, 0, 0, 0],
                "dynamics": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"#,
        );
    }
}
