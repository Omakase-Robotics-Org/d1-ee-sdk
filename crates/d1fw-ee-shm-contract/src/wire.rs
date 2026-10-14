//! The fixed-size messages of `end_effector.shm/1`.
//!
//! Every message is `#[repr(C)]`, little-endian (the byte order of every
//! host the contract runs on), and laid out with explicit `reserved` fields
//! so that it has no implicit padding: the sum of its fields' sizes is its
//! size, which `crate::layout` asserts. Every message starts with a
//! [`Header`]. Strings are UTF-8, NUL-padded to their field's capacity (no
//! NUL inside the text, and nothing but NUL after it). An optional `f64` is
//! NaN when absent. A `reserved` field is written as zero and ignored on
//! receipt.
//!
//! The iceoryx2 type name of each message carries the contract's major
//! version (`omakase_ee_v1_*`), so a provider built against another major
//! cannot open the services at all; `Header::schema_version` is checked on
//! every message as well.

// The derive emits an `unsafe impl ZeroCopySend`: the promise that the type
// is self-contained plain data with a stable layout, which every message
// here is by construction (`#[repr(C)]`, integers, floats and arrays of
// them only).
#![allow(unsafe_code)]

use iceoryx2::prelude::ZeroCopySend;

use crate::{
    BODY_LEN, DETAIL_LEN, EXTRA_LEN, FAULT_LEN, IDS_LEN, KEY_LEN, MAX_JOINTS, MAX_SET_EXTRA,
    MESSAGE_LEN, NAME_LEN,
};

/// The header every message starts with.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_Header")]
pub struct Header {
    /// [`crate::SCHEMA_VERSION`].
    pub schema_version: u32,
    /// Zero.
    pub reserved: u32,
    /// The sender's instance: a random non-zero value a provider or the
    /// daemon draws once when it starts, so a receiver can tell a restarted
    /// peer from the one it was talking to.
    pub instance_id: u64,
    /// Per sender and service, strictly increasing from 1.
    pub seq: u64,
    /// `CLOCK_MONOTONIC` nanoseconds when the message was produced.
    pub mono_ns: u64,
}

/// One joint of a [`Descriptor`].
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_Joint")]
pub struct Joint {
    /// The joint's name, unique on its end effector.
    pub name: [u8; KEY_LEN],
    /// [`JOINT_UNIT_RAD`] or [`JOINT_UNIT_FRACTION`].
    pub unit: u32,
    /// Zero.
    pub reserved: u32,
    /// The smallest target accepted, in `unit`.
    pub min: f64,
    /// The largest target accepted, in `unit`.
    pub max: f64,
}

/// One key of a [`Descriptor`]'s `set_extra` list.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_ExtraKey")]
pub struct ExtraKey {
    /// The key a `set` may carry in its `extra` object.
    pub name: [u8; KEY_LEN],
}

/// The tool a [`Descriptor`] declares for its arm's controller: what the
/// end effector weighs, where its mass sits and where its tool centre point
/// is, in the flange frame. The same values, in the same units and order, as
/// a client registers with `POST /v1/arm/{side}/tool`.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_Tool")]
pub struct Tool {
    /// The mass, kilograms.
    pub mass_kg: f64,
    /// The centre of mass, millimetres, XYZ.
    pub com_mm: [f64; 3],
    /// The inertia tensor about the centre of mass, kg·m², as `Ixx, Ixy,
    /// Ixz, Iyy, Iyz, Izz`.
    pub inertia: [f64; 6],
    /// The tool centre point's translation, millimetres, XYZ.
    pub tcp_xyz_mm: [f64; 3],
    /// The tool centre point's rotation, degrees, roll/pitch/yaw (XYZ
    /// order).
    pub tcp_rpy_deg: [f64; 3],
}

/// What the provider drives: its identity, joints and capabilities.
///
/// Published by the provider on `<base>/descriptor`, latched (history 1) and
/// republished every [`crate::ports::LATCHED_REPUBLISH_MS`].
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_Descriptor")]
pub struct Descriptor {
    /// The common header.
    pub header: Header,
    /// Starts at 1 and increases whenever the provider's declaration
    /// changes; heartbeats, states, ticks and commands name the generation
    /// they belong to.
    pub generation: u64,
    /// The provider's driver name.
    pub driver: [u8; NAME_LEN],
    /// The schema id (`<id>/<major>`, the provider's own) the state's
    /// `detail` follows; empty when the provider publishes no detail.
    pub detail_schema: [u8; NAME_LEN],
    /// The model.
    pub model: [u8; NAME_LEN],
    /// The bus address the provider found its device at, `-1` for none.
    pub node_id: i32,
    /// How many entries of `joints` are used.
    pub joint_count: u32,
    /// The joints of the `joints` command mode, in command order.
    pub joints: [Joint; MAX_JOINTS],
    /// `CAP_*` bits.
    pub capabilities: u32,
    /// `GRIP_*` bits: the presets `set` accepts.
    pub grip_presets: u32,
    /// `FORCE_SENSING_*`.
    pub force_sensing: u32,
    /// The vendor-rated ceiling on commands, hertz; `0` when none is
    /// published.
    pub max_command_hz: u32,
    /// How long a `set` with `wait` may take, milliseconds; `0` for the
    /// contract's default ([`crate::model::DEFAULT_SET_TIMEOUT_MS`]).
    pub set_timeout_ms: u32,
    /// How many entries of `set_extra` are used.
    pub set_extra_count: u32,
    /// The keys `set` accepts in its `extra` object.
    pub set_extra: [ExtraKey; MAX_SET_EXTRA],
    /// `1` when `tool` is declared, `0` when it is not (and `tool` is then
    /// zero and ignored).
    pub tool_declared: u32,
    /// Zero.
    pub reserved: u32,
    /// The tool the daemon registers on the side's arm, valid with
    /// `tool_declared`.
    pub tool: Tool,
    /// The id of the collision envelope the arm guard checks this end
    /// effector with (`d1/parallel_gripper`); empty for none declared.
    pub guard_model: [u8; NAME_LEN],
}

/// The provider's liveness, on `<base>/heartbeat`.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_Heartbeat")]
pub struct Heartbeat {
    /// The common header.
    pub header: Header,
    /// The descriptor generation the provider currently declares.
    pub descriptor_generation: u64,
    /// How often the provider publishes its state, milliseconds.
    pub state_period_ms: u32,
    /// Zero.
    pub reserved: u32,
}

/// One state sample, on `<base>/state`.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_State")]
pub struct State {
    /// The common header.
    pub header: Header,
    /// The descriptor generation the sample is in terms of.
    pub descriptor_generation: u64,
    /// The openness estimate, `0.0` closed … `1.0` open; NaN for none.
    pub openness: f64,
    /// The actuator torque, newton-metres; NaN for none.
    pub torque_nm: f64,
    /// `STATE_*` bits.
    pub flags: u32,
    /// How many entries of `joints` are used: the descriptor's joint count,
    /// or `0` when the provider has no reading.
    pub joint_count: u32,
    /// Measured joint positions, in descriptor order and units.
    pub joints: [f64; MAX_JOINTS],
    /// Measured joint velocities, in the joint's unit per second; NaN for a
    /// joint whose velocity is not known. Read only with
    /// [`CAP_JOINT_VELOCITY`].
    pub velocity: [f64; MAX_JOINTS],
    /// The effort each joint's actuator reports, newton-metres for a rotary
    /// actuator and newtons for a linear one; NaN for a joint whose effort
    /// is not known. Read only with [`CAP_JOINT_EFFORT`].
    pub effort: [f64; MAX_JOINTS],
    /// The fault the device reports; empty for none.
    pub fault: [u8; FAULT_LEN],
    /// How many bytes of `detail` are used; `0` for no detail.
    pub detail_len: u32,
    /// Zero.
    pub reserved: u32,
    /// The detail object `{"schema", "data"}`, as UTF-8 JSON.
    pub detail: [u8; DETAIL_LEN],
}

/// One control-loop tick, on `<base>/tick`.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_Tick")]
pub struct Tick {
    /// The common header.
    pub header: Header,
    /// The descriptor generation the tick is in terms of.
    pub descriptor_generation: u64,
    /// Ticks the provider's loop has run, strictly increasing.
    pub tick_seq: u64,
    /// Wall-clock microseconds since the Unix epoch when the tick was
    /// folded.
    pub timestamp_us: u64,
    /// `PHASE_*`.
    pub phase: u32,
    /// How many entries of each joint array are used.
    pub joint_count: u32,
    /// `TICK_*` bits.
    pub flags: u32,
    /// Zero.
    pub reserved: u32,
    /// Measured joint positions.
    pub measured: [f64; MAX_JOINTS],
    /// Commanded joint positions, valid with [`TICK_COMMANDED`].
    pub commanded: [f64; MAX_JOINTS],
    /// Per-joint effort, newton-metres, valid with [`TICK_EFFORT`].
    pub effort: [f64; MAX_JOINTS],
    /// The fault on this tick; empty for none.
    pub fault: [u8; FAULT_LEN],
}

/// Streamed joint targets, on `<base>/joints` (daemon to provider).
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_JointsCommand")]
pub struct JointsCommand {
    /// The common header; `instance_id` is the daemon's.
    pub header: Header,
    /// The provider instance the targets were validated for.
    pub target_instance_id: u64,
    /// The descriptor generation the targets were validated against.
    pub descriptor_generation: u64,
    /// The client's wall-clock microseconds when it produced the targets.
    pub timestamp_us: u64,
    /// How many entries of `targets` are used.
    pub joint_count: u32,
    /// Zero.
    pub reserved: u32,
    /// One target per declared joint, in descriptor order and units.
    pub targets: [f64; MAX_JOINTS],
}

/// The daemon's liveness and stop generation, on `<base>/control` (daemon
/// to provider), latched.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_Control")]
pub struct Control {
    /// The common header; `instance_id` is the daemon's.
    pub header: Header,
    /// Increases by one on every stop the daemon issues on this side. A
    /// provider that sees it increase stops, whether or not the `stop`
    /// request reached it.
    pub stop_generation: u64,
    /// How often the daemon publishes this, milliseconds.
    pub period_ms: u32,
    /// Zero.
    pub reserved: u32,
}

/// A `set` or `stop` request, on `<base>/command` (daemon to provider).
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_CommandRequest")]
pub struct CommandRequest {
    /// The common header; `instance_id` is the daemon's.
    pub header: Header,
    /// The provider instance the request was validated for.
    pub target_instance_id: u64,
    /// The descriptor generation it was validated against.
    pub descriptor_generation: u64,
    /// `CLOCK_MONOTONIC` nanoseconds after which a `set` must not be
    /// started (answered [`STATUS_STALE_TARGET`]); a `stop` is executed
    /// whatever its age.
    pub start_by_mono_ns: u64,
    /// [`OP_SET`] or [`OP_STOP`].
    pub op: u32,
    /// `SET_*` bits.
    pub flags: u32,
    /// `GRIP_INDEX_*`, valid with [`SET_GRIP`].
    pub grip: u32,
    /// How many bytes of `extra` are used; `0` for no `extra`.
    pub extra_len: u32,
    /// The openness, `0.0` closed … `1.0` open.
    pub openness: f64,
    /// The grip cap, newton-metres, valid with [`SET_TORQUE`].
    pub torque_nm: f64,
    /// The driver-scoped `extra` object, as UTF-8 JSON.
    pub extra: [u8; EXTRA_LEN],
}

/// The answer to a [`CommandRequest`].
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_CommandResponse")]
pub struct CommandResponse {
    /// The common header; `instance_id` is the provider's.
    pub header: Header,
    /// `STATUS_*`.
    pub status: u32,
    /// Zero.
    pub reserved: u32,
    /// Why, when `status` is not [`STATUS_OK`].
    pub message: [u8; MESSAGE_LEN],
}

/// A `clear_fault` or `registers` request, on `<base>/fault` (daemon to
/// provider).
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_FaultRequest")]
pub struct FaultRequest {
    /// The common header; `instance_id` is the daemon's.
    pub header: Header,
    /// The provider instance the request is for.
    pub target_instance_id: u64,
    /// [`OP_CLEAR_FAULT`] or [`OP_REGISTERS`].
    pub op: u32,
    /// How many bytes of `ids` are used; `0` for the provider's default set.
    pub ids_len: u32,
    /// The register ids, a JSON array of strings.
    pub ids: [u8; IDS_LEN],
}

/// The answer to a [`FaultRequest`].
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_ee_v1_FaultResponse")]
pub struct FaultResponse {
    /// The common header; `instance_id` is the provider's.
    pub header: Header,
    /// `STATUS_*`.
    pub status: u32,
    /// How many bytes of `body` are used.
    pub body_len: u32,
    /// Why, when `status` is not [`STATUS_OK`].
    pub message: [u8; MESSAGE_LEN],
    /// With [`STATUS_OK`]: `{"ok", "fault", "details"}` for a fault clear,
    /// `{"entries", "details"}` for a register dump, as UTF-8 JSON.
    pub body: [u8; BODY_LEN],
}

/// [`Joint::unit`]: radians.
pub const JOINT_UNIT_RAD: u32 = 0;
/// [`Joint::unit`]: a fraction of the axis's range, `0.0` … `1.0`.
pub const JOINT_UNIT_FRACTION: u32 = 1;

/// [`Descriptor::capabilities`]: `set` with an openness is accepted.
pub const CAP_OPENNESS: u32 = 1 << 0;
/// `joints` is accepted.
pub const CAP_JOINTS: u32 = 1 << 1;
/// `set` with `wait` answers when the stroke concludes.
pub const CAP_STROKE_COMPLETION: u32 = 1 << 2;
/// `set` accepts an explicit `torque_nm`.
pub const CAP_GRIP_TORQUE: u32 = 1 << 3;
/// The state reports whether a closing stroke stopped on an object.
pub const CAP_CONTACT_SENSING: u32 = 1 << 4;
/// `clear_fault` is served.
pub const CAP_CLEAR_FAULT: u32 = 1 << 5;
/// `registers` is served.
pub const CAP_REGISTERS: u32 = 1 << 6;
/// The provider publishes control-loop ticks.
pub const CAP_TICKS: u32 = 1 << 7;
/// The state carries per-joint velocities ([`State::velocity`]).
pub const CAP_JOINT_VELOCITY: u32 = 1 << 8;
/// The state carries per-joint efforts ([`State::effort`]).
pub const CAP_JOINT_EFFORT: u32 = 1 << 9;
/// Every capability bit schema 1 defines.
pub const CAP_ALL: u32 = (1 << 10) - 1;

/// [`Descriptor::grip_presets`]: `soft`.
pub const GRIP_SOFT: u32 = 1 << 0;
/// `firm`.
pub const GRIP_FIRM: u32 = 1 << 1;
/// `strong`.
pub const GRIP_STRONG: u32 = 1 << 2;
/// Every grip bit schema 1 defines.
pub const GRIP_ALL: u32 = GRIP_SOFT | GRIP_FIRM | GRIP_STRONG;

/// [`Descriptor::force_sensing`]: none.
pub const FORCE_SENSING_NONE: u32 = 0;
/// Inferred from motor current or torque.
pub const FORCE_SENSING_ESTIMATED: u32 = 1;
/// Measured by a force or torque sensor.
pub const FORCE_SENSING_MEASURED: u32 = 2;

/// [`State::flags`]: the reading was taken for this sample.
pub const STATE_LIVE: u32 = 1 << 0;
/// A streamed target is being driven toward.
pub const STATE_TRACKING: u32 = 1 << 1;
/// `holding` is known; its value is [`STATE_HOLDING`].
pub const STATE_HOLDING_KNOWN: u32 = 1 << 2;
/// An object is held.
pub const STATE_HOLDING: u32 = 1 << 3;
/// `contact` is known; its value is [`STATE_CONTACT`].
pub const STATE_CONTACT_KNOWN: u32 = 1 << 4;
/// The newest closing stroke stopped on an object.
pub const STATE_CONTACT: u32 = 1 << 5;
/// `cap_reached` is known; its value is [`STATE_CAP_REACHED`].
pub const STATE_CAP_REACHED_KNOWN: u32 = 1 << 6;
/// The newest closing stroke concluded at its force cap.
pub const STATE_CAP_REACHED: u32 = 1 << 7;
/// The provider had a detail that did not fit [`crate::DETAIL_LEN`] and
/// sent none rather than a truncated one.
pub const STATE_DETAIL_DROPPED: u32 = 1 << 8;
/// Every state bit schema 1 defines.
pub const STATE_ALL: u32 = (1 << 9) - 1;

/// [`Tick::phase`]: no command owns the actuators.
pub const PHASE_IDLE: u32 = 0;
/// Closing under a motion reference.
pub const PHASE_CLOSING: u32 = 1;
/// Opening toward a destination commanded in one step.
pub const PHASE_OPENING: u32 = 2;
/// Holding what a closing stroke met.
pub const PHASE_HOLD: u32 = 3;

/// [`Tick::flags`]: `commanded` is valid.
pub const TICK_COMMANDED: u32 = 1 << 0;
/// `effort` is valid.
pub const TICK_EFFORT: u32 = 1 << 1;

/// [`CommandRequest::op`]: drive to an openness.
pub const OP_SET: u32 = 1;
/// Stop and take the actuators' authority away. Never opens.
pub const OP_STOP: u32 = 2;
/// [`FaultRequest::op`]: clear the device's fault latch.
pub const OP_CLEAR_FAULT: u32 = 1;
/// Read the device's registers.
pub const OP_REGISTERS: u32 = 2;

/// [`CommandRequest::flags`]: answer when the stroke concludes.
pub const SET_WAIT: u32 = 1 << 0;
/// `grip` is valid.
pub const SET_GRIP: u32 = 1 << 1;
/// `torque_nm` is valid.
pub const SET_TORQUE: u32 = 1 << 2;

/// [`CommandRequest::grip`]: `soft`.
pub const GRIP_INDEX_SOFT: u32 = 0;
/// `firm`.
pub const GRIP_INDEX_FIRM: u32 = 1;
/// `strong`.
pub const GRIP_INDEX_STRONG: u32 = 2;

/// A response's status: done.
pub const STATUS_OK: u32 = 0;
/// Refused: a capability not declared, a stroke stopped or halted.
pub const STATUS_REFUSED: u32 = 1;
/// A value the provider cannot take (a torque outside its band, an `extra`
/// value it does not accept).
pub const STATUS_UNPROCESSABLE: u32 = 2;
/// The device cannot be driven now.
pub const STATUS_UNAVAILABLE: u32 = 3;
/// The device failed.
pub const STATUS_DEVICE: u32 = 4;
/// The device did not answer in time.
pub const STATUS_TIMEOUT: u32 = 5;
/// The request was for another provider instance or descriptor generation,
/// or arrived after its start deadline; it was not executed.
pub const STATUS_STALE_TARGET: u32 = 6;
/// The request itself was malformed.
pub const STATUS_INVALID: u32 = 7;

/// The iceoryx2 type name a message is registered under.
#[must_use]
pub fn type_name<T: ZeroCopySend>() -> &'static str {
    // SAFETY: `type_name` has no precondition for the caller; it is unsafe
    // only to implement, since a wrong name lets two different types share a
    // service.
    unsafe { T::type_name() }
}

macro_rules! zeroed {
    ($($ty:ident),* $(,)?) => {
        $(
            impl $ty {
                /// Every field zero (every array empty, every count 0).
                #[must_use]
                pub fn zeroed() -> Self {
                    // A literal per type would restate every field; the
                    // all-zero bit pattern is valid for each of them
                    // (integers, floats, arrays of those), which
                    // `ZeroCopySend`'s plain-data promise already rests on.
                    // SAFETY: see above; no field has an invalid all-zero
                    // representation.
                    unsafe { core::mem::zeroed() }
                }
            }
        )*
    };
}

zeroed!(
    Header,
    Joint,
    ExtraKey,
    Tool,
    Descriptor,
    Heartbeat,
    State,
    Tick,
    JointsCommand,
    Control,
    CommandRequest,
    CommandResponse,
    FaultRequest,
    FaultResponse,
);
