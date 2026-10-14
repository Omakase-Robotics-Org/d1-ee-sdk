//! Golden-frame scenarios: what a provider put on its flange channel, step
//! by step, compared with a recording.
//!
//! A recording is a JSON document `{"steps": [{"step", "status", "frames":
//! [{"t_us", "id", "data"}]}]}`: each step's name, the HTTP status
//! d1-firmwared answered it with, and every frame the channel received
//! during it — the offset from the step's start in microseconds, the 11-bit
//! identifier, and the bytes as lowercase hex. A run records the same shape,
//! each frame also carrying the `kind` it was relayed as.

use std::{path::Path, time::Instant};

use d1fw_canfd::{CanFdBus, FrameKind};
use serde_json::{Value, json};

use crate::recorder::{Recorder, Sent};

/// The bytes as lowercase hex.
#[must_use]
pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

/// A frame kind as a recording names it.
#[must_use]
pub const fn kind_name(kind: FrameKind) -> &'static str {
    match kind {
        FrameKind::Motion => "motion",
        FrameKind::Read => "read",
        FrameKind::Stop => "stop",
    }
}

/// `frames` as a recording lists them, offsets from `started`.
#[must_use]
pub fn frames_json(frames: Vec<Sent>, started: Instant) -> Vec<Value> {
    frames
        .into_iter()
        .map(|(at, frame, kind)| {
            json!({
                "t_us": u64::try_from(at.saturating_duration_since(started).as_micros())
                    .unwrap_or(u64::MAX),
                "id": frame.id,
                "data": hex(&frame.data),
                "kind": kind_name(kind),
            })
        })
        .collect()
}

/// The steps of a run, recorded from a channel.
pub struct Steps<'a, B> {
    recorder: &'a Recorder<B>,
    steps: Vec<Value>,
}

impl<'a, B: CanFdBus> Steps<'a, B> {
    /// Record steps from `recorder`.
    #[must_use]
    pub const fn new(recorder: &'a Recorder<B>) -> Self {
        Self {
            recorder,
            steps: Vec::new(),
        }
    }

    /// Drop what the channel received since the last step.
    pub fn discard(&self) {
        self.recorder.take();
    }

    /// Close step `name`, started at `started` and answered with `status`,
    /// with every frame the channel received since the last step.
    pub fn record(&mut self, name: &str, started: Instant, status: u16) {
        let frames = frames_json(self.recorder.take(), started);
        self.steps
            .push(json!({"step": name, "status": status, "frames": frames}));
    }

    /// The recorded steps.
    #[must_use]
    pub fn into_steps(self) -> Vec<Value> {
        self.steps
    }
}

/// How far a run's frame timings may lie from a recording's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tolerance {
    /// How far a frame's offset from its step's start may lie from the
    /// recorded one…
    pub base_us: u64,
    /// …plus this much for every frame before it in its step (a provider
    /// that paces its exchanges with a sleep after each one carries every
    /// delay forward).
    pub per_frame_us: u64,
    /// How far the gap since the previous frame of the step may lie from
    /// the recorded gap; `None` to leave gaps unchecked.
    pub gap_us: Option<u64>,
}

/// How a run's frames compare with a recording.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Comparison {
    /// Frames compared.
    pub frames: usize,
    /// The largest difference between a frame's offset and the recorded
    /// one, microseconds.
    pub offset_us: u64,
    /// The largest difference between a frame's gap and the recorded one,
    /// microseconds.
    pub gap_us: u64,
}

/// Read the recording at `path`.
///
/// # Panics
///
/// When it cannot be read or is not JSON.
#[must_use]
pub fn read_recording(path: &Path) -> Value {
    serde_json::from_str(
        &std::fs::read_to_string(path)
            .unwrap_or_else(|error| panic!("read {}: {error}", path.display())),
    )
    .unwrap_or_else(|error| panic!("{} is not JSON: {error}", path.display()))
}

/// Assert that `steps` reproduce the recording at `path`: the same steps in
/// the same order with the same statuses; the same frames in the same order
/// (identifiers, lengths and bytes); each frame within `tolerance`; and each
/// relayed as the kind `expected_kind(step, id, data)` names.
///
/// # Panics
///
/// On any difference, naming the step and the frame.
#[must_use]
pub fn compare(
    path: &Path,
    steps: &[Value],
    tolerance: Tolerance,
    expected_kind: impl Fn(&str, u64, &str) -> &'static str,
) -> Comparison {
    let recorded = read_recording(path);
    let name = recorded["scenario"]
        .as_str()
        .map_or_else(|| path.display().to_string(), str::to_owned);
    let recorded = recorded["steps"].as_array().expect("recorded steps");
    assert_eq!(steps.len(), recorded.len(), "{name}: the step count");
    let mut comparison = Comparison::default();
    for (step, expected) in steps.iter().zip(recorded) {
        let step_name = expected["step"].as_str().unwrap_or_default();
        let label = format!("{name}: {step_name}");
        assert_eq!(step["step"], expected["step"], "{label}: the step order");
        assert_eq!(step["status"], expected["status"], "{label}: status");
        let frames = step["frames"].as_array().expect("frames");
        let expected_frames = expected["frames"].as_array().expect("recorded frames");
        let strip = |frames: &[Value]| -> Vec<(Value, Value)> {
            frames
                .iter()
                .map(|frame| (frame["id"].clone(), frame["data"].clone()))
                .collect()
        };
        assert_eq!(strip(frames), strip(expected_frames), "{label}: the frames");
        let mut previous: Option<(u64, u64)> = None;
        for (index, (frame, was)) in frames.iter().zip(expected_frames).enumerate() {
            let id = frame["id"].as_u64().expect("an id");
            let data = frame["data"].as_str().expect("data");
            assert_eq!(
                frame["kind"].as_str(),
                Some(expected_kind(step_name, id, data)),
                "{label}: frame {index} ({id:#x} {data}) relayed as the wrong kind"
            );
            let at = frame["t_us"].as_u64().expect("an offset");
            let was = was["t_us"].as_u64().expect("a recorded offset");
            let offset = at.abs_diff(was);
            let allowed = tolerance.base_us
                + tolerance.per_frame_us * u64::try_from(index).expect("a few hundred");
            assert!(
                offset <= allowed,
                "{label}: frame {index} at {at} us, recorded at {was} us (tolerance {allowed} us)"
            );
            if let (Some(gap_tolerance), Some((at_before, was_before))) =
                (tolerance.gap_us, previous)
            {
                let gap = (at - at_before).abs_diff(was - was_before);
                assert!(
                    gap <= gap_tolerance,
                    "{label}: frame {index} came {} us after the one before it, recorded {} us",
                    at - at_before,
                    was - was_before
                );
                comparison.gap_us = comparison.gap_us.max(gap);
            }
            previous = Some((at, was));
            comparison.offset_us = comparison.offset_us.max(offset);
            comparison.frames += 1;
        }
    }
    comparison
}
