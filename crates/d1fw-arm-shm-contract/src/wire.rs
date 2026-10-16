//! The fixed-size messages of the four arm contracts.
//!
//! The conventions are `end_effector.shm/1`'s: `#[repr(C)]`, little-endian,
//! explicit `reserved` fields written 0 and ignored on receipt, no implicit
//! padding (the sum of the field sizes is the message size, which
//! [`crate::layout`] asserts), NUL-padded UTF-8 text, and every message
//! starting with the shared 32-byte [`Header`]. Each type name carries its
//! own contract's major (`omakase_arm_feedback_v1_*`,
//! `omakase_arm_status_v1_*`, `omakase_arm_events_v1_*`,
//! `omakase_arm_command_v1_*`), so the four contracts version
//! independently and a peer built against another major cannot open the
//! service at all.
//!
//! Numbers the arm controller reports are carried in the controller's own
//! representation (`f32`, `i16`, `i32`, `u8`) and its own units, without
//! conversion, so a reader sees exactly what the daemon received.

// The derive emits an `unsafe impl ZeroCopySend`: the promise that the type
// is self-contained plain data with a stable layout, which every message
// here is by construction (`#[repr(C)]`, integers, floats and arrays of
// them only).
#![allow(unsafe_code)]

use iceoryx2::prelude::ZeroCopySend;

pub use d1fw_ee_shm_contract::wire::Header;

use crate::{
    CART_DOF, EVENT_VALUES, HOLDER_LEN, JOINTS, MESSAGE_LEN, PARA_NAME_LEN, RAW_CAPACITY, SIDES,
    TEXT_LEN, TOOL_DYN_LEN,
};

// --- arm_feedback.shm/1 ---------------------------------------------------

/// One arm side of one controller feedback record, decoded.
///
/// Field by field what the controller reported for this side, in its units.
/// The controller field each one is read from is named in the contract
/// text (`spec/arm_feedback.shm/1.md` §3.3).
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_arm_feedback_v1_SideFeedback")]
pub struct SideFeedback {
    /// The controller's current state code (`STATE_*`).
    pub cur_state: i32,
    /// The controller's command-state word.
    pub cmd_state: i32,
    /// The controller's error code; `0` while it reports none.
    pub error_code: i32,
    /// The selected impedance type: `1` joint, `2` Cartesian, `3` force.
    pub imp_type: i32,
    /// The controller's output frame serial.
    pub out_frame_serial: i32,
    /// The serial of the newest host command frame the controller took.
    pub in_frame_serial: i32,
    /// The controller's own cycle counter, modulo 1 000 000.
    pub sys_cyc: i32,
    /// The Cartesian stiffness/damping type in effect.
    pub cart_kd_type: i32,
    /// The force-feedback type in effect.
    pub force_fb_type: i32,
    /// The force-control type in effect.
    pub force_type: i32,
    /// Host command frames the controller reckons it did not receive,
    /// refreshed every 200 control cycles; the vendor's healthy band is
    /// below [`crate::FRAME_MISS_HEALTHY_BELOW`].
    pub frame_miss_count: i16,
    /// The high-water mark of `frame_miss_count` since the controller
    /// started.
    pub max_frame_miss_count: i16,
    /// Controller realtime cycles that missed their deadline.
    pub sys_cyc_miss_count: i16,
    /// The high-water mark of `sys_cyc_miss_count`.
    pub max_sys_cyc_miss_count: i16,
    /// The joint velocity ratio in effect, percent.
    pub vel_ratio: i16,
    /// The joint acceleration ratio in effect, percent.
    pub acc_ratio: i16,
    /// The tool-tip digital inputs.
    pub tip_di: u8,
    /// The controller's low-speed (stationary) flag: nonzero while the arm
    /// is not moving.
    pub low_speed: u8,
    /// The controller's trajectory state.
    pub traj_state: u8,
    /// Zero.
    pub reserved: u8,
    /// Measured joint positions, degrees.
    pub pos: [f32; JOINTS],
    /// Measured joint velocities, degrees per second.
    pub vel: [f32; JOINTS],
    /// The controller's per-joint position error.
    pub pos_e: [f32; JOINTS],
    /// The joint command the controller echoes on its output side, degrees.
    pub cmd_echo: [f32; JOINTS],
    /// The per-joint current (commanded torque) the controller reports.
    pub current: [f32; JOINTS],
    /// The per-joint torque the joint sensors report, newton-metres.
    pub torque: [f32; JOINTS],
    /// Joint temperatures, degrees Celsius.
    pub temperature: [f32; JOINTS],
    /// The controller's estimated joint friction.
    pub est_friction: [f32; JOINTS],
    /// The time derivative of `est_friction`.
    pub est_friction_dot: [f32; JOINTS],
    /// The controller's estimated external joint force.
    pub est_force: [f32; JOINTS],
    /// The joint command the controller holds on its input side: the newest
    /// host target it accepted, degrees.
    pub accepted_cmd: [f32; JOINTS],
    /// Joint stiffness in effect.
    pub joint_k: [f32; JOINTS],
    /// Joint damping in effect.
    pub joint_d: [f32; JOINTS],
    /// The force loop's seven control parameters in effect.
    pub force_pidul: [f32; JOINTS],
    /// The controller's estimated Cartesian force/torque at the tool.
    pub est_cart_fn: [f32; CART_DOF],
    /// Cartesian stiffness in effect.
    pub cart_k: [f32; CART_DOF],
    /// Cartesian damping in effect.
    pub cart_d: [f32; CART_DOF],
    /// The force-controlled Cartesian directions in effect.
    pub force_dir: [f32; CART_DOF],
    /// The tool kinematics registered on the controller: TCP translation
    /// (millimetres) and rotation (degrees).
    pub tool_kine: [f32; CART_DOF],
    /// The tool dynamics registered on the controller: mass (kg), centre of
    /// mass (mm) and inertia.
    pub tool_dyn: [f32; TOOL_DYN_LEN],
    /// The null-space stiffness of the redundant seventh joint.
    pub cart_kn: f32,
    /// The null-space damping of the redundant seventh joint.
    pub cart_dn: f32,
    /// The compliance adjustment limit in effect, millimetres.
    pub force_adj_lmt: f32,
    /// The force command in effect, newtons.
    pub force_cmd: f32,
}

/// Every signal of one controller feedback record, on `omakase/arm/feedback`
/// (daemon to readers), one sample per record the daemon received.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_arm_feedback_v1_Feedback")]
pub struct Feedback {
    /// The common header; `instance_id` is the daemon's, `seq` counts the
    /// records published since it started, `mono_ns` is when it published
    /// this one.
    pub header: Header,
    /// `CLOCK_MONOTONIC` nanoseconds when the daemon received the record.
    pub rx_mono_ns: u64,
    /// The controller's firmware version, `0` while the daemon has not
    /// read it.
    pub controller_version: i32,
    /// `PROTOCOL_*`: the host-to-controller wire generation the daemon
    /// speaks.
    pub protocol: u32,
    /// `FEEDBACK_*` bits.
    pub flags: u32,
    /// `RAW_*`: what `raw` holds.
    pub raw_format: u32,
    /// How many bytes of `raw` are the record.
    pub raw_len: u32,
    /// The parameter reply's type (`0` int32, `1` float, `2` string).
    pub para_type: u8,
    /// The parameter reply's instruction.
    pub para_ins: u8,
    /// The serial of the parameter command the reply slot belongs to.
    pub para_cmd_serial: i16,
    /// The controller's return serial for that command: the serial, or
    /// `error_code * 100 + serial` on a refusal.
    pub para_ret_serial: i16,
    /// Zero.
    pub reserved: u16,
    /// The parameter reply's integer value.
    pub para_value_i: i32,
    /// The parameter reply's float value.
    pub para_value_f: f32,
    /// Zero.
    pub reserved2: u32,
    /// The parameter reply's name, NUL-padded.
    pub para_name: [u8; PARA_NAME_LEN],
    /// Arm A (index 0, the physical left arm) and arm B (index 1).
    pub sides: [SideFeedback; SIDES],
    /// The record as received, byte for byte; bytes past `raw_len` are
    /// zero.
    pub raw: [u8; RAW_CAPACITY],
}

/// [`Feedback::protocol`]: the protocol is not known.
pub const PROTOCOL_UNKNOWN: u32 = 0;
/// [`Feedback::protocol`]: the `FX` command frame (controller firmware up
/// to 100341).
pub const PROTOCOL_FX: u32 = 1;
/// [`Feedback::protocol`]: the `FY` command frame (controller firmware from
/// 100343).
pub const PROTOCOL_FY: u32 = 2;

/// [`Feedback::flags`]: the controller's firmware family contradicts the
/// configured protocol, so the daemon refuses every motion command.
pub const FEEDBACK_PROTOCOL_MISMATCH: u32 = 1 << 0;
/// [`Feedback::flags`]: the record was longer than [`RAW_CAPACITY`] and
/// `raw` holds its first bytes only.
pub const FEEDBACK_RAW_TRUNCATED: u32 = 1 << 1;
/// Every feedback bit schema 1 defines.
pub const FEEDBACK_ALL: u32 = FEEDBACK_PROTOCOL_MISMATCH | FEEDBACK_RAW_TRUNCATED;

/// [`Feedback::raw_format`]: the controller's 1428-byte dual-arm state
/// record (`DCSS`), as it follows the two-byte `FX` tag of a feedback
/// datagram.
pub const RAW_DCSS_1428: u32 = 1;

/// [`SideFeedback::cur_state`]: the controller codes of its states.
pub const STATE_IDLE: i32 = 0;
/// Position mode.
pub const STATE_POSITION: i32 = 1;
/// PVT mode.
pub const STATE_PVT: i32 = 2;
/// Torque (impedance) mode.
pub const STATE_TORQUE: i32 = 3;
/// Released (floating).
pub const STATE_RELEASE: i32 = 4;
/// The controller's error state: a protective stop.
pub const STATE_ERROR: i32 = 100;
/// Transitioning to position.
pub const STATE_TO_POSITION: i32 = 101;
/// Transitioning to PVT.
pub const STATE_TO_PVT: i32 = 102;
/// Transitioning to torque.
pub const STATE_TO_TORQUE: i32 = 103;
/// Transitioning to released.
pub const STATE_TO_RELEASE: i32 = 104;
/// Transitioning to idle.
pub const STATE_TO_IDLE: i32 = 109;

// --- arm_status.shm/1 -----------------------------------------------------

/// One arm side as the daemon sees it.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_arm_status_v1_SideStatus")]
pub struct SideStatus {
    /// `MODE_*`: the arm's mode as the daemon reads it.
    pub mode: u32,
    /// The controller's error code; `0` while it reports none.
    pub error_code: i32,
    /// `SIDE_*` bits.
    pub flags: u32,
    /// `TOOL_*`: where the tool registered on this arm came from.
    pub tool_source: u32,
    /// The registered tool's mass, kilograms; NaN while none is registered.
    pub tool_mass_kg: f64,
    /// Host command frames the controller reckons it did not receive.
    pub frame_miss_count: i32,
    /// Its high-water mark since the controller started.
    pub max_frame_miss_count: i32,
    /// Controller realtime cycles that missed their deadline.
    pub sys_cyc_miss_count: i32,
    /// Zero.
    pub reserved: u32,
}

/// The daemon's view of the arms, on `omakase/arm/status` (daemon to
/// readers), latched.
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_arm_status_v1_Status")]
pub struct Status {
    /// The common header; `instance_id` is the daemon's.
    pub header: Header,
    /// The header `seq` of the newest `arm_feedback` sample; `0` before the
    /// first.
    pub feedback_seq: u64,
    /// How old the newest controller record was when this was published,
    /// nanoseconds; `u64::MAX` before the first.
    pub feedback_age_ns: u64,
    /// The lease's grant counter; `0` while no lease is held.
    pub lease_epoch: u64,
    /// Whole seconds until the lease expires, rounded up; `0` while none is
    /// held.
    pub lease_expires_in_s: u64,
    /// Commands the `arm_command` path wrote to the controller since the
    /// daemon started.
    pub commands_accepted: u64,
    /// Commands it refused.
    pub commands_refused: u64,
    /// Commands it dropped unread because a newer one had arrived.
    pub commands_superseded: u64,
    /// The newest trajectory job's identifier; `0` before the first.
    pub trajectory_id: u64,
    /// `STATUS_*` bits.
    pub flags: u32,
    /// The lease holder's class rank (`10` ambient, `50` policy, `90`
    /// operator); `0` while none is held.
    pub lease_class_rank: u32,
    /// The `arm_command` path's rate ceiling, commands per second; `0` while
    /// the path is not served.
    pub max_command_hz: u32,
    /// How old feedback may be before the daemon calls it stale,
    /// milliseconds; `0` when the daemon applies no such rule.
    pub feedback_stale_after_ms: u32,
    /// `TRAJECTORY_*`: the newest trajectory job's phase.
    pub trajectory_phase: u32,
    /// How often the daemon publishes this when nothing changes,
    /// milliseconds.
    pub period_ms: u32,
    /// The lease holder's self-declared name; empty while none is held.
    pub lease_holder: [u8; HOLDER_LEN],
    /// Arm A (index 0) and arm B (index 1).
    pub sides: [SideStatus; SIDES],
}

/// [`SideStatus::mode`]: a controller state the daemon does not know.
pub const MODE_UNKNOWN: u32 = 0;
/// Idle (servos off).
pub const MODE_IDLE: u32 = 1;
/// Joint position mode.
pub const MODE_POSITION: u32 = 2;
/// PVT mode.
pub const MODE_PVT: u32 = 3;
/// Torque, impedance or force-compliance mode.
pub const MODE_TORQUE: u32 = 4;
/// Released (floating).
pub const MODE_RELEASE: u32 = 5;
/// The controller reports an error: a protective stop.
pub const MODE_ERROR: u32 = 6;

/// [`SideStatus::flags`]: the daemon holds this arm's brakes released for
/// hand guiding.
pub const SIDE_BRAKES_RELEASED: u32 = 1 << 0;
/// The controller's firmware family contradicts the configured protocol.
pub const SIDE_PROTOCOL_MISMATCH: u32 = 1 << 1;
/// The controller reports the arm stationary.
pub const SIDE_STATIONARY: u32 = 1 << 2;
/// Every side bit schema 1 defines.
pub const SIDE_ALL: u32 = SIDE_BRAKES_RELEASED | SIDE_PROTOCOL_MISMATCH | SIDE_STATIONARY;

/// [`SideStatus::tool_source`]: nothing registered since the daemon
/// started.
pub const TOOL_NONE: u32 = 0;
/// The tool the side's configured end effector declares.
pub const TOOL_CONFIG: u32 = 1;
/// A client's `POST /v1/arm/{side}/tool`.
pub const TOOL_CLIENT: u32 = 2;

/// [`Status::flags`]: the arm soft-kill latch stands; every motion command
/// is refused.
pub const STATUS_SOFT_KILLED: u32 = 1 << 0;
/// A live lease is held.
pub const STATUS_LEASE_HELD: u32 = 1 << 1;
/// The `arm_command` path is served.
pub const STATUS_COMMAND_SERVED: u32 = 1 << 2;
/// The newest controller record is older than `feedback_stale_after_ms`.
pub const STATUS_FEEDBACK_STALE: u32 = 1 << 3;
/// The `arm_command` stream is stopped: its newest command was refused for
/// a reason that does not clear by itself (lease, soft kill, mode).
pub const STATUS_STREAM_STOPPED: u32 = 1 << 4;
/// Every status bit schema 1 defines.
pub const STATUS_ALL: u32 = STATUS_SOFT_KILLED
    | STATUS_LEASE_HELD
    | STATUS_COMMAND_SERVED
    | STATUS_FEEDBACK_STALE
    | STATUS_STREAM_STOPPED;

/// [`Status::trajectory_phase`]: no job since the daemon started.
pub const TRAJECTORY_NONE: u32 = 0;
/// Playback is in progress.
pub const TRAJECTORY_RUNNING: u32 = 1;
/// Playback ran to the end.
pub const TRAJECTORY_COMPLETED: u32 = 2;
/// Playback was cancelled.
pub const TRAJECTORY_CANCELLED: u32 = 3;
/// Playback stopped on an error.
pub const TRAJECTORY_FAILED: u32 = 4;

// --- arm_events.shm/1 -----------------------------------------------------

/// One thing that happened to the arms, on `omakase/arm/events` (daemon to
/// readers).
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_arm_events_v1_Event")]
pub struct Event {
    /// The common header; `instance_id` is the daemon's, `seq` counts the
    /// events published since it started.
    pub header: Header,
    /// `EVENT_*`: what happened.
    pub kind: u32,
    /// `SIDE_MASK_*`: which arms it concerns.
    pub sides: u32,
    /// A kind-specific code (the contract text lists them per kind).
    pub code: i32,
    /// The joint it concerns, `0` to `6`; `-1` for none.
    pub joint: i32,
    /// Kind-specific values (the contract text lists them per kind); unused
    /// entries are zero.
    pub values: [f64; EVENT_VALUES],
    /// A sentence for a human, NUL-padded; may be empty.
    pub text: [u8; TEXT_LEN],
}

/// [`Event::sides`]: arm A.
pub const SIDE_MASK_A: u32 = 1 << 0;
/// [`Event::sides`]: arm B.
pub const SIDE_MASK_B: u32 = 1 << 1;
/// Every side bit.
pub const SIDE_MASK_ALL: u32 = SIDE_MASK_A | SIDE_MASK_B;

/// [`Event::kind`]: the controller's state for one arm changed. `code`:
/// the new `cur_state`; `values[0]`: the previous one.
pub const EVENT_CONTROLLER_STATE: u32 = 1;
/// The controller's error code for one arm changed. `code`: the new code;
/// `values[0]`: the previous one.
pub const EVENT_CONTROLLER_ERROR: u32 = 2;
/// One arm's `frame_miss_count` reached the vendor's unhealthy band
/// (`>= 20`). `code`: the count.
pub const EVENT_FRAME_MISS_HIGH: u32 = 3;
/// It fell back below it. `code`: the count.
pub const EVENT_FRAME_MISS_NORMAL: u32 = 4;
/// The arm soft-kill latch rose.
pub const EVENT_SOFT_KILL_LATCHED: u32 = 5;
/// The arm soft-kill latch was released.
pub const EVENT_SOFT_KILL_RELEASED: u32 = 6;
/// The lease changed hands, expired or was released. `code`: the new
/// holder's class rank (`0` for none); `values[0]`: its epoch; `text`: its
/// holder (empty for none).
pub const EVENT_LEASE_CHANGED: u32 = 7;
/// The daemon released one arm's brakes for hand guiding.
pub const EVENT_BRAKES_RELEASED: u32 = 8;
/// The daemon engaged one arm's brakes again.
pub const EVENT_BRAKES_ENGAGED: u32 = 9;
/// The motion guard refused a joint move, on any lane. `text`: the refusal.
pub const EVENT_GUARD_REFUSED: u32 = 10;
/// The approach limiter wrote a streamed step short of its target.
/// `values[0]`, `values[1]`: arm A's and arm B's step scale (`0` to `1`).
pub const EVENT_APPROACH_SLOWED: u32 = 11;
/// The `arm_command` stream stopped: a command was refused after an
/// accepted one. `code`: the `RESULT_*` reason; `text`: the refusal. The
/// arms hold the last accepted target; the next command that passes every
/// check is written, with nothing to unlock.
pub const EVENT_STREAM_STOPPED: u32 = 12;
/// The `arm_command` stream resumed: a command was accepted after a
/// refused one.
pub const EVENT_STREAM_RESUMED: u32 = 13;
/// One arm was emergency-stopped.
pub const EVENT_EMERGENCY_STOP: u32 = 14;
/// A trajectory job started. `values[0]`: its identifier.
pub const EVENT_TRAJECTORY_STARTED: u32 = 15;
/// A trajectory job ended. `code`: its `TRAJECTORY_*` phase; `values[0]`:
/// its identifier.
pub const EVENT_TRAJECTORY_ENDED: u32 = 16;
/// The highest event kind schema 1 defines.
pub const EVENT_MAX: u32 = EVENT_TRAJECTORY_ENDED;

// --- arm_command.shm/1 ----------------------------------------------------

/// One joint target for one or both arms, on `omakase/arm/command` (client
/// to daemon).
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_arm_command_v1_Command")]
pub struct Command {
    /// The common header; `instance_id` is the client's, `seq` strictly
    /// increasing per client from 1, `mono_ns` when the client produced the
    /// command.
    pub header: Header,
    /// The arm lease's token, as `POST /v1/arm/lease` returned it to the
    /// holder.
    pub lease_token: u64,
    /// How old the command may be when the daemon takes it, nanoseconds;
    /// `0` for [`crate::DEFAULT_COMMAND_MAX_AGE_MS`].
    pub max_age_ns: u64,
    /// `SIDE_MASK_*`: which arms `target_deg` addresses (at least one).
    pub sides: u32,
    /// Zero (no flag is defined in schema 1).
    pub flags: u32,
    /// Joint targets, degrees: arm A's (index 0) and arm B's (index 1). An
    /// arm `sides` does not name is ignored.
    pub target_deg: [[f64; JOINTS]; SIDES],
}

/// What the daemon did with one command, on `omakase/arm/command_status`
/// (daemon to clients).
#[derive(Debug, Clone, Copy, PartialEq, ZeroCopySend)]
#[repr(C)]
#[type_name("omakase_arm_command_v1_CommandStatus")]
pub struct CommandStatus {
    /// The common header; `instance_id` is the daemon's.
    pub header: Header,
    /// The command's header `instance_id`.
    pub client_instance_id: u64,
    /// The command's header `seq`.
    pub client_seq: u64,
    /// The command's header `mono_ns`.
    pub client_mono_ns: u64,
    /// `CLOCK_MONOTONIC` nanoseconds when the daemon took the command.
    pub rx_mono_ns: u64,
    /// When the daemon's write to the controller returned; `0` when the
    /// command was refused.
    pub sent_mono_ns: u64,
    /// Commands accepted since the daemon started.
    pub accepted: u64,
    /// Commands refused since the daemon started.
    pub refused: u64,
    /// Commands dropped unread because a newer one had arrived.
    pub superseded: u64,
    /// `RESULT_*`.
    pub result: u32,
    /// `CMD_*` bits.
    pub flags: u32,
    /// The approach limiter's scale of arm A's and arm B's step (`1` when
    /// the target was written as sent).
    pub scale: [f64; SIDES],
    /// The targets written, degrees (the sent ones, or the approach
    /// limiter's shortened ones); zero when refused.
    pub written_deg: [[f64; JOINTS]; SIDES],
    /// Why the command was refused, NUL-padded; empty when accepted.
    pub message: [u8; MESSAGE_LEN],
}

/// [`CommandStatus::result`]: written to the controller.
pub const RESULT_ACCEPTED: u32 = 0;
/// Malformed: another schema version, no sender, no side, an undefined
/// flag, a non-finite target.
pub const RESULT_INVALID: u32 = 1;
/// Older than its `max_age_ns` when the daemon took it.
pub const RESULT_STALE: u32 = 2;
/// The lease token is not the live lease's: no lease is held, it expired,
/// or it belongs to another holder.
pub const RESULT_LEASE: u32 = 3;
/// The arm soft-kill latch stands.
pub const RESULT_SOFT_KILLED: u32 = 4;
/// An arm is not in a mode that acts on joint targets, or its feedback is
/// stale, or the controller's firmware family contradicts the protocol.
pub const RESULT_NOT_READY: u32 = 5;
/// The motion guard refused the move (a clearance or a joint limit).
pub const RESULT_GUARD: u32 = 6;
/// Another arm operation holds the arm (a recover, a trajectory).
pub const RESULT_BUSY: u32 = 7;
/// Above the path's rate ceiling.
pub const RESULT_RATE: u32 = 8;
/// The write to the controller failed.
pub const RESULT_DEVICE: u32 = 9;
/// Outside the joint limits.
pub const RESULT_LIMITS: u32 = 10;
/// The highest result code schema 1 defines.
pub const RESULT_MAX: u32 = RESULT_LIMITS;

/// [`CommandStatus::flags`]: the approach limiter shortened the step.
pub const CMD_SLOWED: u32 = 1 << 0;
/// Every command-status bit schema 1 defines.
pub const CMD_ALL: u32 = CMD_SLOWED;

macro_rules! zeroed {
    ($($ty:ident),* $(,)?) => {
        $(
            impl $ty {
                /// Every field zero.
                #[must_use]
                pub fn zeroed() -> Self {
                    // SAFETY: every field is an integer, a float or an array
                    // of them (the header too), for which the all-zero bit
                    // pattern is valid.
                    unsafe { core::mem::zeroed() }
                }
            }

            impl Default for $ty {
                fn default() -> Self {
                    Self::zeroed()
                }
            }
        )*
    };
}

zeroed!(
    SideFeedback,
    Feedback,
    SideStatus,
    Status,
    Event,
    Command,
    CommandStatus
);

/// Write `text` into a NUL-padded field, cut at a character boundary to
/// fit.
pub fn write_text<const N: usize>(field: &mut [u8; N], text: &str) {
    let mut end = text.len().min(N);
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    field.fill(0);
    field[..end].copy_from_slice(&text.as_bytes()[..end]);
}

/// The text of a NUL-padded field, up to its first NUL; `None` when it is
/// not UTF-8.
#[must_use]
pub fn read_text<const N: usize>(field: &[u8; N]) -> Option<&str> {
    let end = field.iter().position(|byte| *byte == 0).unwrap_or(N);
    core::str::from_utf8(&field[..end]).ok()
}
