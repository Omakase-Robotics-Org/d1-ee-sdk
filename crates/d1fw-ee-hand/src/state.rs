//! The sampled hand state a hand provider publishes as its `hand.state/1`
//! detail.
//!
//! The detail is opaque to the daemon's end-effector resource; the daemon's
//! telemetry reads it by its schema id (`hand.state/1`) for the hand metrics
//! and the state log. Its JSON Schema is committed beside this crate
//! (`schema/hand.state-1.json`, [`HAND_STATE_SCHEMA`]); every field and its
//! meaning here must stay what that schema says, and
//! `tests/hand_state_schema.rs` validates a serialized state against it.

use d1fw_ee_provider::Detail;
use serde::Serialize;

/// The schema id of the detail: a multi-finger hand's sampled state.
pub const HAND_STATE: &str = "hand.state/1";

/// The JSON Schema (draft 2020-12) of a [`HAND_STATE`] detail's `data`.
pub const HAND_STATE_SCHEMA: &str = include_str!("../schema/hand.state-1.json");

/// One sampled snapshot of a hand.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct HandState {
    /// Which model answered (`"<maker>/<model>"`).
    pub model: String,
    /// Which arm side it is mounted on.
    pub side: String,
    /// Per-axis measured position as a fraction of the axis's range.
    pub positions: Vec<f64>,
    /// Per-axis measured position in the raw vendor unit.
    pub positions_wire: Vec<u32>,
    /// Per-axis enabled flags, as the hand reports them.
    pub enabled: Vec<bool>,
    /// Whether the provider believes every axis is enabled.
    pub all_enabled: bool,
    /// The most recent device error record, `0` for none; its meaning is
    /// the model's (`faults` says it in words).
    pub error_code: u32,
    /// Human descriptions of every fault currently reported.
    pub faults: Vec<String>,
    /// Whether `positions` came from a reading taken for this sample rather
    /// than from the last one: a read competes with the command path for
    /// the arm's single passthrough channel, so a busy hand answers with its
    /// last sample and says so here.
    pub live: bool,
    /// Capability tokens, repeated here so a consumer that only polls state
    /// does not have to fetch capabilities separately.
    pub features: Vec<String>,
}

/// What one live sample read off a hand's wire.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct HandSample {
    /// Per-axis position as a fraction of the axis's range.
    pub positions: Vec<f64>,
    /// Per-axis position in the raw vendor unit.
    pub positions_wire: Vec<u32>,
    /// Per-axis enabled flags.
    pub enabled: Vec<bool>,
    /// The model's error record, `0` for none.
    pub error_code: u32,
    /// Human descriptions of every fault reported.
    pub faults: Vec<String>,
}

impl HandState {
    /// A live sample of the `model` hand on `side` as the shared hand state.
    #[must_use]
    pub fn live(model: &str, side: &str, sample: HandSample, features: &[&str]) -> Self {
        Self {
            model: model.to_owned(),
            side: side.to_owned(),
            positions: sample.positions,
            positions_wire: sample.positions_wire,
            all_enabled: sample.enabled.iter().all(|flag| *flag),
            enabled: sample.enabled,
            error_code: sample.error_code,
            faults: sample.faults,
            live: true,
            features: features.iter().map(|token| (*token).to_owned()).collect(),
        }
    }

    /// The state of a `model` hand on `side` with `axes` axes that has not
    /// been sampled yet: every axis at `0`, none enabled, not live.
    #[must_use]
    pub fn unsampled(model: &str, side: &str, axes: usize, features: &[&str]) -> Self {
        Self {
            model: model.to_owned(),
            side: side.to_owned(),
            positions: vec![0.0; axes],
            positions_wire: vec![0; axes],
            enabled: vec![false; axes],
            all_enabled: false,
            error_code: 0,
            faults: Vec::new(),
            live: false,
            features: features.iter().map(|token| (*token).to_owned()).collect(),
        }
    }

    /// This state as a [`HAND_STATE`] detail.
    #[must_use]
    pub fn detail(&self) -> Detail {
        Detail {
            schema: HAND_STATE.to_owned(),
            // Plain fields always serialize; the fallback keeps the detail
            // well-formed rather than turning a read into an error.
            data: serde_json::to_value(self)
                .unwrap_or_else(|_| serde_json::Value::Object(serde_json::Map::new())),
        }
    }

    /// The fault the neutral state reports: the first fault text, else the
    /// non-zero error code.
    #[must_use]
    pub fn fault(&self) -> Option<String> {
        self.faults
            .first()
            .cloned()
            .or_else(|| (self.error_code != 0).then(|| format!("error code {}", self.error_code)))
    }
}

/// One minus the mean of `closed`, or `None` when it is empty: the openness
/// a hand's sampled closedness per driven axis stands for.
#[must_use]
pub fn open_from_closed(closed: &[f64]) -> Option<f64> {
    if closed.is_empty() {
        return None;
    }
    #[expect(
        clippy::cast_precision_loss,
        reason = "at most twenty axes, far inside f64's exact integers"
    )]
    let mean = closed.iter().sum::<f64>() / closed.len() as f64;
    Some((1.0 - mean).clamp(0.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_state_is_its_detail_and_names_its_first_fault() {
        let state = HandState {
            positions: vec![0.0, 0.3, 0.3],
            error_code: 7,
            live: true,
            ..HandState::default()
        };
        let detail = state.detail();
        assert_eq!(detail.schema, "hand.state/1");
        assert_eq!(detail.data["error_code"], 7);
        assert_eq!(state.fault().as_deref(), Some("error code 7"));
        assert!((open_from_closed(&state.positions[1..]).unwrap() - 0.7).abs() < 1e-12);
        assert_eq!(open_from_closed(&[]), None);

        let blank = HandState::unsampled("maker/one", "a", 3, &["token"]);
        assert!(!blank.live && !blank.all_enabled);
        assert_eq!(blank.positions_wire, vec![0; 3]);
        assert_eq!(blank.features, vec!["token"]);
        assert_eq!(blank.fault(), None);
    }
}
