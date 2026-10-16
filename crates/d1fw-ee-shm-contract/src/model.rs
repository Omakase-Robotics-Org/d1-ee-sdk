//! The owned, validated form of every message, and the conversions to and
//! from the wire.
//!
//! Encoding refuses what the receiver would refuse, so a provider learns of
//! a malformed declaration when it builds it rather than from a daemon that
//! silently ignores it. Decoding checks `schema_version`, every count
//! against its capacity, every string for UTF-8 and NUL padding, every enum
//! code, every flag word for bits schema 1 does not define, and every float
//! for finiteness.

use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use crate::{KEY_LEN, MAX_JOINTS, MAX_SET_EXTRA, NAME_LEN, SCHEMA_VERSION, clock, wire};

/// How long a `set` with `wait` may take when the descriptor names no
/// duration, milliseconds.
pub const DEFAULT_SET_TIMEOUT_MS: u32 = 10_000;

/// The longest `set` duration a descriptor may name, milliseconds.
pub const MAX_SET_TIMEOUT_MS: u32 = 60_000;

/// The highest `max_command_hz` a descriptor may name.
pub const MAX_COMMAND_HZ: u32 = 10_000;

/// The `set_extra` key of a hold (§4 rule 10): a provider that lists it
/// takes a `set` whose `extra` is exactly `{"hold": true}` as "keep the
/// actuators holding where they are now, with no command watchdog of your
/// own, until the next command", whether or not it declares `openness`.
pub const HOLD_EXTRA_KEY: &str = "hold";

/// The `set_extra` key of a home (§4 rule 11): a provider that lists it
/// takes a `set` whose `extra` is exactly `{"home": true}` as "drive to your
/// own home pose through your normal control, then hold there", whether or
/// not it declares `openness`.
pub const HOME_EXTRA_KEY: &str = "home";

/// The keyed `set`s: a `set` whose `extra` names one of these is that verb,
/// not an openness.
pub const KEYED_SETS: [&str; 2] = [HOLD_EXTRA_KEY, HOME_EXTRA_KEY];

/// Why a message could not be built, read or exchanged.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum ContractError {
    /// The message carries a `schema_version` other than
    /// [`SCHEMA_VERSION`].
    #[error("schema_version {got} is not the contract's {SCHEMA_VERSION}")]
    SchemaVersion {
        /// The version the message carried.
        got: u32,
    },
    /// A field is out of its range or malformed.
    #[error("{0}")]
    Invalid(String),
    /// iceoryx2 failed (a node, a service, a port, a send).
    #[error("{0}")]
    Transport(String),
}

fn invalid(message: impl Into<String>) -> ContractError {
    ContractError::Invalid(message.into())
}

/// A header for a message sent now by `instance_id` as its `seq`-th on one
/// service.
#[must_use]
pub fn header(instance_id: u64, seq: u64) -> wire::Header {
    wire::Header {
        schema_version: SCHEMA_VERSION,
        reserved: 0,
        instance_id,
        seq,
        mono_ns: clock::monotonic_ns(),
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
        return Err(invalid("instance_id 0 names no sender"));
    }
    Ok(())
}

/// A random non-zero instance id, drawn once per process start.
#[must_use]
pub fn new_instance_id() -> u64 {
    use std::hash::{BuildHasher, Hasher};
    let mut hasher = std::collections::hash_map::RandomState::new().build_hasher();
    hasher.write_u64(clock::monotonic_ns());
    hasher.write_u32(std::process::id());
    hasher.finish().max(1)
}

fn put_str(field: &mut [u8], value: &str, what: &str) -> Result<(), ContractError> {
    if value.len() > field.len() {
        return Err(invalid(format!(
            "{what} {value:?} is {} bytes; the field holds {}",
            value.len(),
            field.len()
        )));
    }
    if value.as_bytes().contains(&0) {
        return Err(invalid(format!("{what} {value:?} contains a NUL byte")));
    }
    field.fill(0);
    field[..value.len()].copy_from_slice(value.as_bytes());
    Ok(())
}

/// Write human-readable text, cut at a character boundary when it does not
/// fit (a fault or an error message loses its tail, never its meaning
/// entirely).
pub(crate) fn put_text(field: &mut [u8], value: &str) {
    let mut end = value.len().min(field.len());
    while !value.is_char_boundary(end) {
        end -= 1;
    }
    let text = value[..end].replace('\0', " ");
    field.fill(0);
    field[..text.len()].copy_from_slice(text.as_bytes());
}

fn get_str<'a>(field: &'a [u8], what: &str) -> Result<&'a str, ContractError> {
    let end = field
        .iter()
        .position(|byte| *byte == 0)
        .unwrap_or(field.len());
    if field[end..].iter().any(|byte| *byte != 0) {
        return Err(invalid(format!(
            "{what} has bytes after its terminating NUL"
        )));
    }
    std::str::from_utf8(&field[..end]).map_err(|_| invalid(format!("{what} is not UTF-8")))
}

pub(crate) fn get_optional_text(field: &[u8], what: &str) -> Result<Option<String>, ContractError> {
    let text = get_str(field, what)?;
    Ok((!text.is_empty()).then(|| text.to_owned()))
}

fn put_json(field: &mut [u8], value: &Value, what: &str) -> Result<u32, ContractError> {
    let bytes = serde_json::to_vec(value)
        .map_err(|error| invalid(format!("{what} does not serialize: {error}")))?;
    if bytes.len() > field.len() {
        return Err(invalid(format!(
            "{what} is {} bytes of JSON; the field holds {}",
            bytes.len(),
            field.len()
        )));
    }
    field.fill(0);
    field[..bytes.len()].copy_from_slice(&bytes);
    Ok(u32::try_from(bytes.len()).unwrap_or(u32::MAX))
}

fn get_json(field: &[u8], len: u32, what: &str) -> Result<Option<Value>, ContractError> {
    let len = usize::try_from(len).unwrap_or(usize::MAX);
    if len > field.len() {
        return Err(invalid(format!(
            "{what} length {len} exceeds its field's {}",
            field.len()
        )));
    }
    if len == 0 {
        return Ok(None);
    }
    serde_json::from_slice(&field[..len])
        .map(Some)
        .map_err(|error| invalid(format!("{what} is not JSON: {error}")))
}

fn count(value: u32, capacity: usize, what: &str) -> Result<usize, ContractError> {
    let value = usize::try_from(value).unwrap_or(usize::MAX);
    if value > capacity {
        return Err(invalid(format!(
            "{what} {value} exceeds the contract's {capacity}"
        )));
    }
    Ok(value)
}

fn optional_f64(value: f64, what: &str) -> Result<Option<f64>, ContractError> {
    if value.is_nan() {
        Ok(None)
    } else if value.is_finite() {
        Ok(Some(value))
    } else {
        Err(invalid(format!("{what} is infinite")))
    }
}

fn finite_all(values: &[f64], what: &str) -> Result<(), ContractError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(invalid(format!("{what} holds a non-finite value")))
    }
}

fn no_unknown_bits(value: u32, known: u32, what: &str) -> Result<(), ContractError> {
    if value & !known == 0 {
        Ok(())
    } else {
        Err(invalid(format!(
            "{what} sets bits {:#x} that schema {SCHEMA_VERSION} does not define",
            value & !known
        )))
    }
}

fn joints_into(
    field: &mut [f64; MAX_JOINTS],
    values: &[f64],
    what: &str,
) -> Result<u32, ContractError> {
    if values.len() > MAX_JOINTS {
        return Err(invalid(format!(
            "{what} has {} values; the contract holds {MAX_JOINTS}",
            values.len()
        )));
    }
    *field = [0.0; MAX_JOINTS];
    field[..values.len()].copy_from_slice(values);
    Ok(u32::try_from(values.len()).unwrap_or(u32::MAX))
}

/// The unit a joint is commanded and reported in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointUnit {
    /// Radians.
    Rad,
    /// A fraction of the axis's range, `0.0` … `1.0`.
    Fraction,
}

/// One joint of the `joints` command mode.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Joint {
    /// Unique on its end effector, at most [`KEY_LEN`] bytes.
    pub name: String,
    /// The unit targets and readings are in.
    pub unit: JointUnit,
    /// The smallest target accepted.
    pub min: f64,
    /// The largest target accepted.
    pub max: f64,
}

/// A named grip a `set` may ask for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum GripPreset {
    /// A light grip for crushable objects.
    Soft,
    /// The default grip.
    Firm,
    /// The hardest preset.
    Strong,
}

impl GripPreset {
    const ALL: [(Self, u32, u32); 3] = [
        (Self::Soft, wire::GRIP_SOFT, wire::GRIP_INDEX_SOFT),
        (Self::Firm, wire::GRIP_FIRM, wire::GRIP_INDEX_FIRM),
        (Self::Strong, wire::GRIP_STRONG, wire::GRIP_INDEX_STRONG),
    ];

    fn bit(self) -> u32 {
        Self::ALL
            .iter()
            .find(|(preset, ..)| *preset == self)
            .map_or(0, |(_, bit, _)| *bit)
    }

    fn index(self) -> u32 {
        Self::ALL
            .iter()
            .find(|(preset, ..)| *preset == self)
            .map_or(0, |(.., index)| *index)
    }

    fn from_index(index: u32) -> Result<Self, ContractError> {
        Self::ALL
            .iter()
            .find(|(.., known)| *known == index)
            .map(|(preset, ..)| *preset)
            .ok_or_else(|| invalid(format!("grip index {index} names no preset")))
    }
}

/// How an end effector senses the force it applies.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ForceSensing {
    /// It does not.
    #[default]
    None,
    /// Inferred from motor current or torque.
    Estimated,
    /// Measured by a force or torque sensor.
    Measured,
}

/// What an end effector can do.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Capabilities {
    /// `set` with an openness is accepted.
    pub openness: bool,
    /// `joints` is accepted.
    pub joints: bool,
    /// `set` with `wait` answers when the stroke concludes.
    pub stroke_completion: bool,
    /// The presets `set` accepts.
    pub grip_presets: Vec<GripPreset>,
    /// `set` accepts an explicit `torque_nm`.
    pub grip_torque: bool,
    /// The keys `set` accepts in its `extra` object, at most
    /// [`MAX_SET_EXTRA`] of at most [`KEY_LEN`] bytes.
    pub set_extra: Vec<String>,
    /// How the applied force is sensed.
    pub force_sensing: ForceSensing,
    /// The state reports whether a closing stroke stopped on an object.
    pub contact_sensing: bool,
    /// `clear_fault` is served.
    pub clear_fault: bool,
    /// `registers` is served.
    pub registers: bool,
    /// Control-loop ticks are published.
    pub ticks: bool,
    /// The state carries per-joint velocities ([`State::velocity`]).
    pub joint_velocity: bool,
    /// The state carries per-joint efforts ([`State::effort`]).
    pub joint_effort: bool,
    /// The vendor-rated ceiling on commands, hertz, when one is published.
    /// The daemon never sends `set` and `joints` faster than this.
    pub max_command_hz: Option<u32>,
}

/// The tool a provider declares for its arm's controller, in the flange
/// frame: the values a client registers with `POST /v1/arm/{side}/tool`, in
/// the same units and order.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Tool {
    /// The mass, kilograms; finite and not negative.
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

impl Tool {
    /// Refuse a tool the daemon would refuse.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for a non-finite value or a negative mass.
    pub fn validate(&self) -> Result<(), ContractError> {
        for (what, values) in [
            ("tool mass_kg", std::slice::from_ref(&self.mass_kg)),
            ("tool com_mm", self.com_mm.as_slice()),
            ("tool inertia", self.inertia.as_slice()),
            ("tool tcp_xyz_mm", self.tcp_xyz_mm.as_slice()),
            ("tool tcp_rpy_deg", self.tcp_rpy_deg.as_slice()),
        ] {
            finite_all(values, what)?;
        }
        if self.mass_kg < 0.0 {
            return Err(invalid(format!(
                "tool mass_kg {} must not be negative",
                self.mass_kg
            )));
        }
        Ok(())
    }

    fn to_wire(self) -> wire::Tool {
        wire::Tool {
            mass_kg: self.mass_kg,
            com_mm: self.com_mm,
            inertia: self.inertia,
            tcp_xyz_mm: self.tcp_xyz_mm,
            tcp_rpy_deg: self.tcp_rpy_deg,
        }
    }

    const fn from_wire(tool: &wire::Tool) -> Self {
        Self {
            mass_kg: tool.mass_kg,
            com_mm: tool.com_mm,
            inertia: tool.inertia,
            tcp_xyz_mm: tool.tcp_xyz_mm,
            tcp_rpy_deg: tool.tcp_rpy_deg,
        }
    }
}

/// What a provider drives.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Descriptor {
    /// The provider's driver name, at most [`NAME_LEN`] bytes.
    pub driver: String,
    /// The schema id (`"<id>/<major>"`) the state's `detail` follows, at
    /// most [`NAME_LEN`] bytes; `None` when the provider publishes no
    /// detail. The id is the provider's own; see [`valid_schema_id`].
    pub detail_schema: Option<String>,
    /// The model.
    pub model: String,
    /// The bus address the provider found its device at.
    pub node_id: Option<u16>,
    /// The joints, in command order; at most [`MAX_JOINTS`].
    pub joints: Vec<Joint>,
    /// What it can do.
    pub capabilities: Capabilities,
    /// How long a `set` with `wait` may take, milliseconds (at most
    /// [`MAX_SET_TIMEOUT_MS`]); `None` for [`DEFAULT_SET_TIMEOUT_MS`].
    pub set_timeout_ms: Option<u32>,
    /// The tool the daemon registers on the side's arm when it accepts this
    /// declaration; `None` registers nothing.
    pub tool: Option<Tool>,
    /// The id of the collision envelope the arm guard should check this end
    /// effector with, at most [`NAME_LEN`] bytes; `None` when the provider
    /// declares none. An id the daemon's guard does not know leaves the
    /// guard on its default envelope, and the daemon says so.
    pub guard_model: Option<String>,
}

impl Descriptor {
    /// Whether `self` declares the same end effector as `other` in
    /// everything but the bus address it found its device at. A provider
    /// whose device has not answered yet announces `node_id: None`, and the
    /// same declaration with the address once it answers (§5 of the
    /// contract); nothing else may change.
    #[must_use]
    pub fn same_but_node(&self, other: &Self) -> bool {
        Self {
            node_id: None,
            ..self.clone()
        } == Self {
            node_id: None,
            ..other.clone()
        }
    }

    /// The `set` duration the daemon waits for, milliseconds.
    #[must_use]
    pub fn set_timeout_ms(&self) -> u32 {
        self.set_timeout_ms.unwrap_or(DEFAULT_SET_TIMEOUT_MS)
    }

    /// Refuse a declaration the daemon would refuse.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] naming the first rule broken.
    pub fn validate(&self) -> Result<(), ContractError> {
        for (what, value) in [("driver", &self.driver), ("model", &self.model)] {
            if value.is_empty() || value.len() > NAME_LEN || value.contains('\0') {
                return Err(invalid(format!(
                    "descriptor {what} {value:?} must be 1 to {NAME_LEN} bytes without NUL"
                )));
            }
        }
        if let Some(schema) = &self.detail_schema
            && (schema.len() > NAME_LEN || !valid_schema_id(schema))
        {
            return Err(invalid(format!(
                "descriptor detail_schema {schema:?} must be \"<id>/<major>\" in at most \
                 {NAME_LEN} bytes"
            )));
        }
        if self.joints.len() > MAX_JOINTS {
            return Err(invalid(format!(
                "descriptor declares {} joints; the contract holds {MAX_JOINTS}",
                self.joints.len()
            )));
        }
        let mut names = std::collections::HashSet::new();
        for joint in &self.joints {
            if joint.name.is_empty() || joint.name.len() > KEY_LEN || joint.name.contains('\0') {
                return Err(invalid(format!(
                    "joint name {:?} must be 1 to {KEY_LEN} bytes without NUL",
                    joint.name
                )));
            }
            if !names.insert(joint.name.as_str()) {
                return Err(invalid(format!(
                    "joint name {:?} is declared twice",
                    joint.name
                )));
            }
            if !joint.min.is_finite() || !joint.max.is_finite() || joint.min >= joint.max {
                return Err(invalid(format!(
                    "joint {} limits [{}, {}] must be finite with min below max",
                    joint.name, joint.min, joint.max
                )));
            }
        }
        let capabilities = &self.capabilities;
        if capabilities.joints && self.joints.is_empty() {
            return Err(invalid("the joints capability needs at least one joint"));
        }
        if capabilities.set_extra.len() > MAX_SET_EXTRA {
            return Err(invalid(format!(
                "descriptor declares {} set_extra keys; the contract holds {MAX_SET_EXTRA}",
                capabilities.set_extra.len()
            )));
        }
        let mut keys = std::collections::HashSet::new();
        for key in &capabilities.set_extra {
            if key.is_empty() || key.len() > KEY_LEN || key.contains('\0') {
                return Err(invalid(format!(
                    "set_extra key {key:?} must be 1 to {KEY_LEN} bytes without NUL"
                )));
            }
            if !keys.insert(key.as_str()) {
                return Err(invalid(format!("set_extra key {key:?} is declared twice")));
            }
        }
        if let Some(hz) = capabilities.max_command_hz
            && !(1..=MAX_COMMAND_HZ).contains(&hz)
        {
            return Err(invalid(format!(
                "max_command_hz {hz} must be 1 to {MAX_COMMAND_HZ} (absent when the vendor \
                 publishes none)"
            )));
        }
        if let Some(timeout) = self.set_timeout_ms
            && !(1..=MAX_SET_TIMEOUT_MS).contains(&timeout)
        {
            return Err(invalid(format!(
                "set_timeout_ms {timeout} must be 1 to {MAX_SET_TIMEOUT_MS}"
            )));
        }
        if let Some(tool) = &self.tool {
            tool.validate()?;
        }
        if let Some(guard_model) = &self.guard_model
            && (guard_model.is_empty()
                || guard_model.len() > NAME_LEN
                || guard_model.contains('\0'))
        {
            return Err(invalid(format!(
                "descriptor guard_model {guard_model:?} must be 1 to {NAME_LEN} bytes without NUL"
            )));
        }
        Ok(())
    }

    /// The wire form, as generation `generation`.
    ///
    /// # Errors
    ///
    /// Whatever [`Self::validate`] refuses.
    pub fn to_wire(
        &self,
        generation: u64,
        header: wire::Header,
    ) -> Result<wire::Descriptor, ContractError> {
        self.validate()?;
        let capabilities = &self.capabilities;
        let mut out = wire::Descriptor::zeroed();
        out.header = header;
        out.generation = generation;
        put_str(&mut out.driver, &self.driver, "driver")?;
        put_str(
            &mut out.detail_schema,
            self.detail_schema.as_deref().unwrap_or(""),
            "detail_schema",
        )?;
        put_str(&mut out.model, &self.model, "model")?;
        out.node_id = self.node_id.map_or(-1, i32::from);
        out.joint_count = u32::try_from(self.joints.len()).unwrap_or(u32::MAX);
        for (slot, joint) in out.joints.iter_mut().zip(&self.joints) {
            put_str(&mut slot.name, &joint.name, "joint name")?;
            slot.unit = match joint.unit {
                JointUnit::Rad => wire::JOINT_UNIT_RAD,
                JointUnit::Fraction => wire::JOINT_UNIT_FRACTION,
            };
            slot.min = joint.min;
            slot.max = joint.max;
        }
        out.capabilities = [
            (capabilities.openness, wire::CAP_OPENNESS),
            (capabilities.joints, wire::CAP_JOINTS),
            (capabilities.stroke_completion, wire::CAP_STROKE_COMPLETION),
            (capabilities.grip_torque, wire::CAP_GRIP_TORQUE),
            (capabilities.contact_sensing, wire::CAP_CONTACT_SENSING),
            (capabilities.clear_fault, wire::CAP_CLEAR_FAULT),
            (capabilities.registers, wire::CAP_REGISTERS),
            (capabilities.ticks, wire::CAP_TICKS),
            (capabilities.joint_velocity, wire::CAP_JOINT_VELOCITY),
            (capabilities.joint_effort, wire::CAP_JOINT_EFFORT),
        ]
        .into_iter()
        .filter(|(set, _)| *set)
        .fold(0, |bits, (_, bit)| bits | bit);
        out.grip_presets = capabilities
            .grip_presets
            .iter()
            .fold(0, |bits, preset| bits | preset.bit());
        out.force_sensing = match capabilities.force_sensing {
            ForceSensing::None => wire::FORCE_SENSING_NONE,
            ForceSensing::Estimated => wire::FORCE_SENSING_ESTIMATED,
            ForceSensing::Measured => wire::FORCE_SENSING_MEASURED,
        };
        out.max_command_hz = capabilities.max_command_hz.unwrap_or(0);
        out.set_timeout_ms = self.set_timeout_ms.unwrap_or(0);
        out.set_extra_count = u32::try_from(capabilities.set_extra.len()).unwrap_or(u32::MAX);
        for (slot, key) in out.set_extra.iter_mut().zip(&capabilities.set_extra) {
            put_str(&mut slot.name, key, "set_extra key")?;
        }
        if let Some(tool) = self.tool {
            out.tool_declared = 1;
            out.tool = tool.to_wire();
        }
        put_str(
            &mut out.guard_model,
            self.guard_model.as_deref().unwrap_or(""),
            "guard_model",
        )?;
        Ok(out)
    }

    /// The declaration a wire descriptor carries, and its generation.
    ///
    /// # Errors
    ///
    /// [`ContractError::SchemaVersion`] or [`ContractError::Invalid`]
    /// naming the first field out of its range, and whatever
    /// [`Self::validate`] refuses.
    pub fn from_wire(message: &wire::Descriptor) -> Result<(Self, u64), ContractError> {
        check_header(&message.header)?;
        if message.generation == 0 {
            return Err(invalid("descriptor generation 0 is not a generation"));
        }
        let joint_count = count(message.joint_count, MAX_JOINTS, "joint_count")?;
        let mut joints = Vec::with_capacity(joint_count);
        for joint in &message.joints[..joint_count] {
            joints.push(Joint {
                name: get_str(&joint.name, "joint name")?.to_owned(),
                unit: match joint.unit {
                    wire::JOINT_UNIT_RAD => JointUnit::Rad,
                    wire::JOINT_UNIT_FRACTION => JointUnit::Fraction,
                    other => return Err(invalid(format!("joint unit {other} is not a unit"))),
                },
                min: joint.min,
                max: joint.max,
            });
        }
        no_unknown_bits(message.capabilities, wire::CAP_ALL, "capabilities")?;
        no_unknown_bits(message.grip_presets, wire::GRIP_ALL, "grip_presets")?;
        let has = |bit: u32| message.capabilities & bit != 0;
        let extra_count = count(message.set_extra_count, MAX_SET_EXTRA, "set_extra_count")?;
        let mut set_extra = Vec::with_capacity(extra_count);
        for key in &message.set_extra[..extra_count] {
            set_extra.push(get_str(&key.name, "set_extra key")?.to_owned());
        }
        let tool = match message.tool_declared {
            0 => None,
            1 => Some(Tool::from_wire(&message.tool)),
            other => {
                return Err(invalid(format!("tool_declared {other} is neither 0 nor 1")));
            }
        };
        let descriptor = Self {
            driver: get_str(&message.driver, "driver")?.to_owned(),
            detail_schema: Some(get_str(&message.detail_schema, "detail_schema")?)
                .filter(|schema| !schema.is_empty())
                .map(str::to_owned),
            model: get_str(&message.model, "model")?.to_owned(),
            node_id: match message.node_id {
                -1 => None,
                id => Some(
                    u16::try_from(id)
                        .map_err(|_| invalid(format!("node_id {id} is not -1 or 0..=65535")))?,
                ),
            },
            joints,
            capabilities: Capabilities {
                openness: has(wire::CAP_OPENNESS),
                joints: has(wire::CAP_JOINTS),
                stroke_completion: has(wire::CAP_STROKE_COMPLETION),
                grip_presets: GripPreset::ALL
                    .iter()
                    .filter(|(_, bit, _)| message.grip_presets & bit != 0)
                    .map(|(preset, ..)| *preset)
                    .collect(),
                grip_torque: has(wire::CAP_GRIP_TORQUE),
                set_extra,
                force_sensing: match message.force_sensing {
                    wire::FORCE_SENSING_NONE => ForceSensing::None,
                    wire::FORCE_SENSING_ESTIMATED => ForceSensing::Estimated,
                    wire::FORCE_SENSING_MEASURED => ForceSensing::Measured,
                    other => {
                        return Err(invalid(format!(
                            "force_sensing {other} is not a sensing kind"
                        )));
                    }
                },
                contact_sensing: has(wire::CAP_CONTACT_SENSING),
                clear_fault: has(wire::CAP_CLEAR_FAULT),
                registers: has(wire::CAP_REGISTERS),
                ticks: has(wire::CAP_TICKS),
                joint_velocity: has(wire::CAP_JOINT_VELOCITY),
                joint_effort: has(wire::CAP_JOINT_EFFORT),
                max_command_hz: (message.max_command_hz != 0).then_some(message.max_command_hz),
            },
            set_timeout_ms: (message.set_timeout_ms != 0).then_some(message.set_timeout_ms),
            tool,
            guard_model: get_optional_text(&message.guard_model, "guard_model")?,
        };
        descriptor.validate()?;
        Ok((descriptor, message.generation))
    }
}

/// The provider's liveness.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Heartbeat {
    /// The descriptor generation the provider declares.
    pub descriptor_generation: u64,
    /// How often it publishes its state, milliseconds.
    pub state_period_ms: u32,
}

impl Heartbeat {
    /// The wire form.
    #[must_use]
    pub fn to_wire(&self, header: wire::Header) -> wire::Heartbeat {
        wire::Heartbeat {
            header,
            descriptor_generation: self.descriptor_generation,
            state_period_ms: self.state_period_ms,
            reserved: 0,
        }
    }

    /// Read a wire heartbeat.
    ///
    /// # Errors
    ///
    /// Whatever [`check_header`] refuses.
    pub fn from_wire(message: &wire::Heartbeat) -> Result<Self, ContractError> {
        check_header(&message.header)?;
        Ok(Self {
            descriptor_generation: message.descriptor_generation,
            state_period_ms: message.state_period_ms,
        })
    }
}

/// Whether `schema` is a schema id: `"<id>/<major>"`, where `id` is
/// non-empty and holds no `/` or NUL, and `major` is a decimal integer from
/// 1 without leading zeros (`acme.report/1`, `damiao.report/1`).
#[must_use]
pub fn valid_schema_id(schema: &str) -> bool {
    let Some((id, major)) = schema.split_once('/') else {
        return false;
    };
    !id.is_empty()
        && !id.contains('\0')
        && !major.is_empty()
        && !major.starts_with('0')
        && major.bytes().all(|byte| byte.is_ascii_digit())
}

/// A driver's own report: `schema` is the schema id `data` follows, the
/// one the descriptor's `detail_schema` declares.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Detail {
    /// The schema id, `"<id>/<major>"`.
    pub schema: String,
    /// The report.
    pub data: Value,
}

/// One state sample.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct State {
    /// The openness estimate, `0.0` closed … `1.0` open.
    pub openness: Option<f64>,
    /// Measured joint positions in descriptor order; empty for no reading.
    pub joints: Vec<f64>,
    /// Measured joint velocities, in each joint's unit per second: one entry
    /// per entry of `joints`, `None` for a joint whose velocity is not
    /// known, or empty when no joint's is. Published by a provider that
    /// declares `joint_velocity`.
    pub velocity: Vec<Option<f64>>,
    /// The effort each joint's actuator reports, newton-metres for a rotary
    /// actuator and newtons for a linear one: one entry per entry of
    /// `joints`, `None` for a joint whose effort is not known, or empty when
    /// no joint's is. Published by a provider that declares `joint_effort`.
    pub effort: Vec<Option<f64>>,
    /// The reading was taken for this sample.
    pub live: bool,
    /// A streamed target is being driven toward.
    pub tracking: bool,
    /// Whether an object is held, when the provider can tell.
    pub holding: Option<bool>,
    /// Whether the newest closing stroke stopped on an object.
    pub contact: Option<bool>,
    /// Whether the newest closing stroke concluded at its cap.
    pub cap_reached: Option<bool>,
    /// The actuator torque, newton-metres.
    pub torque_nm: Option<f64>,
    /// The fault the device reports.
    pub fault: Option<String>,
    /// The provider's own report.
    pub detail: Option<Detail>,
    /// The provider had a detail too large for the contract and sent none.
    pub detail_dropped: bool,
}

/// A per-joint optional reading in its wire form: NaN for `None` and for
/// every entry past the reading. `values` is either empty or one entry per
/// joint of the reading.
fn per_joint_into(
    values: &[Option<f64>],
    joints: usize,
    what: &str,
) -> Result<[f64; MAX_JOINTS], ContractError> {
    if !values.is_empty() && values.len() != joints {
        return Err(invalid(format!(
            "{what} has {} values for {joints} joints",
            values.len()
        )));
    }
    let mut out = [f64::NAN; MAX_JOINTS];
    for (slot, value) in out.iter_mut().zip(values) {
        if let Some(value) = value {
            if !value.is_finite() {
                return Err(invalid(format!("{what} holds a non-finite value")));
            }
            *slot = *value;
        }
    }
    Ok(out)
}

/// The reading of a per-joint optional array: `None` for a NaN entry, and
/// empty when no entry is known.
fn per_joint_from(values: &[f64], what: &str) -> Result<Vec<Option<f64>>, ContractError> {
    let values = values
        .iter()
        .map(|value| optional_f64(*value, what))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(if values.iter().all(Option::is_none) {
        Vec::new()
    } else {
        values
    })
}

fn tri(known: u32, value: u32, field: Option<bool>) -> u32 {
    match field {
        None => 0,
        Some(false) => known,
        Some(true) => known | value,
    }
}

fn read_tri(flags: u32, known: u32, value: u32) -> Option<bool> {
    (flags & known != 0).then_some(flags & value != 0)
}

impl State {
    /// The wire form, in terms of descriptor generation `generation`. A
    /// detail that does not fit [`crate::DETAIL_LEN`] is left out and flagged
    /// rather than cut.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for more than [`MAX_JOINTS`] joints, a
    /// non-finite joint, or an openness outside `[0, 1]`.
    pub fn to_wire(
        &self,
        generation: u64,
        header: wire::Header,
    ) -> Result<wire::State, ContractError> {
        let mut out = wire::State::zeroed();
        out.header = header;
        out.descriptor_generation = generation;
        if let Some(openness) = self.openness
            && !(0.0..=1.0).contains(&openness)
        {
            return Err(invalid(format!(
                "state openness {openness} is outside [0, 1]"
            )));
        }
        out.openness = self.openness.unwrap_or(f64::NAN);
        out.torque_nm = self.torque_nm.unwrap_or(f64::NAN);
        finite_all(&self.joints, "state joints")?;
        out.joint_count = joints_into(&mut out.joints, &self.joints, "state joints")?;
        out.velocity = per_joint_into(&self.velocity, self.joints.len(), "state velocity")?;
        out.effort = per_joint_into(&self.effort, self.joints.len(), "state effort")?;
        put_text(&mut out.fault, self.fault.as_deref().unwrap_or(""));
        let mut flags = tri(wire::STATE_HOLDING_KNOWN, wire::STATE_HOLDING, self.holding)
            | tri(wire::STATE_CONTACT_KNOWN, wire::STATE_CONTACT, self.contact)
            | tri(
                wire::STATE_CAP_REACHED_KNOWN,
                wire::STATE_CAP_REACHED,
                self.cap_reached,
            );
        if self.live {
            flags |= wire::STATE_LIVE;
        }
        if self.tracking {
            flags |= wire::STATE_TRACKING;
        }
        if self.detail_dropped {
            flags |= wire::STATE_DETAIL_DROPPED;
        }
        if let Some(detail) = &self.detail {
            let value = serde_json::to_value(detail)
                .map_err(|error| invalid(format!("detail does not serialize: {error}")))?;
            match put_json(&mut out.detail, &value, "detail") {
                Ok(len) => out.detail_len = len,
                Err(_) => {
                    out.detail.fill(0);
                    out.detail_len = 0;
                    flags |= wire::STATE_DETAIL_DROPPED;
                }
            }
        }
        out.flags = flags;
        Ok(out)
    }

    /// Read a wire state taken against a descriptor of `dof` joints, and the
    /// generation it names.
    ///
    /// # Errors
    ///
    /// [`ContractError::SchemaVersion`], or [`ContractError::Invalid`] for a
    /// joint count other than `0` and `dof`, a non-finite reading, an
    /// openness outside `[0, 1]`, an undefined flag, or a detail that is not
    /// the `{schema, data}` object.
    pub fn from_wire(message: &wire::State, dof: usize) -> Result<(Self, u64), ContractError> {
        check_header(&message.header)?;
        no_unknown_bits(message.flags, wire::STATE_ALL, "state flags")?;
        let joint_count = count(message.joint_count, MAX_JOINTS, "state joint_count")?;
        if joint_count != 0 && joint_count != dof {
            return Err(invalid(format!(
                "state carries {joint_count} joints; the descriptor declares {dof}"
            )));
        }
        let joints = message.joints[..joint_count].to_vec();
        finite_all(&joints, "state joints")?;
        let velocity = per_joint_from(&message.velocity[..joint_count], "state velocity")?;
        let effort = per_joint_from(&message.effort[..joint_count], "state effort")?;
        let openness = optional_f64(message.openness, "state openness")?;
        if let Some(openness) = openness
            && !(0.0..=1.0).contains(&openness)
        {
            return Err(invalid(format!(
                "state openness {openness} is outside [0, 1]"
            )));
        }
        let detail = get_json(&message.detail, message.detail_len, "detail")?
            .map(|value| {
                serde_json::from_value::<Detail>(value).map_err(|error| {
                    invalid(format!("detail is not a {{schema, data}} object: {error}"))
                })
            })
            .transpose()?;
        let flags = message.flags;
        Ok((
            Self {
                openness,
                joints,
                velocity,
                effort,
                live: flags & wire::STATE_LIVE != 0,
                tracking: flags & wire::STATE_TRACKING != 0,
                holding: read_tri(flags, wire::STATE_HOLDING_KNOWN, wire::STATE_HOLDING),
                contact: read_tri(flags, wire::STATE_CONTACT_KNOWN, wire::STATE_CONTACT),
                cap_reached: read_tri(
                    flags,
                    wire::STATE_CAP_REACHED_KNOWN,
                    wire::STATE_CAP_REACHED,
                ),
                torque_nm: optional_f64(message.torque_nm, "state torque_nm")?,
                fault: get_optional_text(&message.fault, "state fault")?,
                detail,
                detail_dropped: flags & wire::STATE_DETAIL_DROPPED != 0,
            },
            message.descriptor_generation,
        ))
    }
}

/// What a control loop was doing at one tick.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum TickPhase {
    /// No command owns the actuators.
    #[default]
    Idle,
    /// Closing under a motion reference.
    Closing,
    /// Opening toward a destination commanded in one step.
    Opening,
    /// Holding what a closing stroke met.
    Hold,
}

/// One control-loop tick.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Tick {
    /// Ticks the loop has run, strictly increasing.
    pub seq: u64,
    /// Wall-clock microseconds since the Unix epoch.
    pub timestamp_us: u64,
    /// Measured joint positions.
    pub measured: Vec<f64>,
    /// Commanded joint positions, before the loop's first command `None`.
    pub commanded: Option<Vec<f64>>,
    /// The fault on this tick.
    pub fault: Option<String>,
    /// What the loop was doing.
    pub phase: TickPhase,
    /// Per-joint effort, newton-metres.
    pub effort: Option<Vec<f64>>,
}

impl Tick {
    /// The wire form.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for more than [`MAX_JOINTS`] values, a
    /// non-finite one, or `commanded` / `effort` of another length than
    /// `measured`.
    pub fn to_wire(
        &self,
        generation: u64,
        header: wire::Header,
    ) -> Result<wire::Tick, ContractError> {
        let mut out = wire::Tick::zeroed();
        out.header = header;
        out.descriptor_generation = generation;
        out.tick_seq = self.seq;
        out.timestamp_us = self.timestamp_us;
        out.phase = match self.phase {
            TickPhase::Idle => wire::PHASE_IDLE,
            TickPhase::Closing => wire::PHASE_CLOSING,
            TickPhase::Opening => wire::PHASE_OPENING,
            TickPhase::Hold => wire::PHASE_HOLD,
        };
        finite_all(&self.measured, "tick measured")?;
        out.joint_count = joints_into(&mut out.measured, &self.measured, "tick measured")?;
        for (values, field, bit, what) in [
            (
                &self.commanded,
                &mut out.commanded,
                wire::TICK_COMMANDED,
                "tick commanded",
            ),
            (
                &self.effort,
                &mut out.effort,
                wire::TICK_EFFORT,
                "tick effort",
            ),
        ] {
            if let Some(values) = values {
                if values.len() != self.measured.len() {
                    return Err(invalid(format!(
                        "{what} has {} values; measured has {}",
                        values.len(),
                        self.measured.len()
                    )));
                }
                finite_all(values, what)?;
                joints_into(field, values, what)?;
                out.flags |= bit;
            }
        }
        put_text(&mut out.fault, self.fault.as_deref().unwrap_or(""));
        Ok(out)
    }

    /// Read a wire tick, and the generation it names.
    ///
    /// # Errors
    ///
    /// [`ContractError::SchemaVersion`], or [`ContractError::Invalid`] for an
    /// undefined phase or flag, a count beyond the contract, or a
    /// non-finite value.
    pub fn from_wire(message: &wire::Tick) -> Result<(Self, u64), ContractError> {
        check_header(&message.header)?;
        no_unknown_bits(
            message.flags,
            wire::TICK_COMMANDED | wire::TICK_EFFORT,
            "tick flags",
        )?;
        let joint_count = count(message.joint_count, MAX_JOINTS, "tick joint_count")?;
        let take = |values: &[f64; MAX_JOINTS], what: &str| {
            let values = values[..joint_count].to_vec();
            finite_all(&values, what).map(|()| values)
        };
        Ok((
            Self {
                seq: message.tick_seq,
                timestamp_us: message.timestamp_us,
                measured: take(&message.measured, "tick measured")?,
                commanded: (message.flags & wire::TICK_COMMANDED != 0)
                    .then(|| take(&message.commanded, "tick commanded"))
                    .transpose()?,
                fault: get_optional_text(&message.fault, "tick fault")?,
                phase: match message.phase {
                    wire::PHASE_IDLE => TickPhase::Idle,
                    wire::PHASE_CLOSING => TickPhase::Closing,
                    wire::PHASE_OPENING => TickPhase::Opening,
                    wire::PHASE_HOLD => TickPhase::Hold,
                    other => return Err(invalid(format!("tick phase {other} is not a phase"))),
                },
                effort: (message.flags & wire::TICK_EFFORT != 0)
                    .then(|| take(&message.effort, "tick effort"))
                    .transpose()?,
            },
            message.descriptor_generation,
        ))
    }
}

/// Streamed joint targets, as a provider receives them.
#[derive(Debug, Clone, PartialEq)]
pub struct JointTargets {
    /// The provider instance they were validated for.
    pub target_instance_id: u64,
    /// The descriptor generation they were validated against.
    pub descriptor_generation: u64,
    /// When the daemon sent them, `CLOCK_MONOTONIC` nanoseconds.
    pub issued_mono_ns: u64,
    /// The client's wall-clock microseconds when it produced them.
    pub timestamp_us: u64,
    /// One target per declared joint.
    pub targets: Vec<f64>,
}

impl JointTargets {
    /// The wire form.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for more than [`MAX_JOINTS`] targets or a
    /// non-finite one.
    pub fn to_wire(&self, header: wire::Header) -> Result<wire::JointsCommand, ContractError> {
        let mut out = wire::JointsCommand::zeroed();
        out.header = header;
        out.target_instance_id = self.target_instance_id;
        out.descriptor_generation = self.descriptor_generation;
        out.timestamp_us = self.timestamp_us;
        finite_all(&self.targets, "joint targets")?;
        out.joint_count = joints_into(&mut out.targets, &self.targets, "joint targets")?;
        Ok(out)
    }

    /// Read a wire joints command.
    ///
    /// # Errors
    ///
    /// [`ContractError::SchemaVersion`], or [`ContractError::Invalid`] for a
    /// count beyond the contract or a non-finite target.
    pub fn from_wire(message: &wire::JointsCommand) -> Result<Self, ContractError> {
        check_header(&message.header)?;
        let joint_count = count(message.joint_count, MAX_JOINTS, "joints joint_count")?;
        let targets = message.targets[..joint_count].to_vec();
        finite_all(&targets, "joint targets")?;
        Ok(Self {
            target_instance_id: message.target_instance_id,
            descriptor_generation: message.descriptor_generation,
            issued_mono_ns: message.header.mono_ns,
            timestamp_us: message.timestamp_us,
            targets,
        })
    }
}

/// The daemon's control word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Control {
    /// Increases by one on every stop the daemon issues on this side.
    pub stop_generation: u64,
    /// How often the daemon publishes it, milliseconds.
    pub period_ms: u32,
}

impl Control {
    /// The wire form.
    #[must_use]
    pub fn to_wire(&self, header: wire::Header) -> wire::Control {
        wire::Control {
            header,
            stop_generation: self.stop_generation,
            period_ms: self.period_ms,
            reserved: 0,
        }
    }

    /// Read a wire control word.
    ///
    /// # Errors
    ///
    /// Whatever [`check_header`] refuses.
    pub fn from_wire(message: &wire::Control) -> Result<Self, ContractError> {
        check_header(&message.header)?;
        Ok(Self {
            stop_generation: message.stop_generation,
            period_ms: message.period_ms,
        })
    }
}

/// A validated `set`, as a provider receives it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SetCommand {
    /// `0.0` fully closed … `1.0` fully open.
    pub openness: f64,
    /// Answer when the stroke concludes.
    pub wait: bool,
    /// A grip preset the descriptor lists.
    pub grip: Option<GripPreset>,
    /// An explicit grip cap, newton-metres (descriptor `grip_torque`).
    pub torque_nm: Option<f64>,
    /// Driver-scoped parameters, only keys the descriptor lists.
    pub extra: Option<Map<String, Value>>,
}

impl SetCommand {
    /// The keyed `set` `key` (one of [`KEYED_SETS`]): `extra` `{key: true}`,
    /// no wait, no grip, no torque, and an openness of `0.0` that the
    /// provider ignores.
    #[must_use]
    pub fn keyed(key: &str) -> Self {
        let mut extra = Map::new();
        extra.insert(key.to_owned(), Value::Bool(true));
        Self {
            openness: 0.0,
            wait: false,
            grip: None,
            torque_nm: None,
            extra: Some(extra),
        }
    }

    /// A hold (§4 rule 10).
    #[must_use]
    pub fn hold() -> Self {
        Self::keyed(HOLD_EXTRA_KEY)
    }

    /// A home (§4 rule 11).
    #[must_use]
    pub fn home() -> Self {
        Self::keyed(HOME_EXTRA_KEY)
    }

    /// The keyed verb this `set` names, if its `extra` names one (a
    /// well-formed one is checked by [`SetCommand::check_keyed`]).
    #[must_use]
    pub fn keyed_verb(&self) -> Option<&'static str> {
        let extra = self.extra.as_ref()?;
        KEYED_SETS.into_iter().find(|key| extra.contains_key(*key))
    }

    /// Whether this `set` is a hold.
    #[must_use]
    pub fn is_hold(&self) -> bool {
        self.keyed_verb() == Some(HOLD_EXTRA_KEY)
    }

    /// Whether this `set` is a home.
    #[must_use]
    pub fn is_home(&self) -> bool {
        self.keyed_verb() == Some(HOME_EXTRA_KEY)
    }

    /// Check a keyed `set`: `extra` is exactly `{key: true}`, without wait,
    /// grip or torque.
    ///
    /// # Errors
    ///
    /// A sentence naming what else the request carries.
    pub fn check_keyed(&self) -> Result<(), String> {
        let Some(key) = self.keyed_verb() else {
            return Err("not a keyed set".to_owned());
        };
        let extra = self.extra.as_ref();
        if extra.and_then(|extra| extra.get(key)) != Some(&Value::Bool(true)) {
            return Err(format!("a {key} is extra {{\"{key}\": true}}"));
        }
        if extra.is_some_and(|extra| extra.len() != 1) {
            return Err(format!("a {key} carries no other extra key"));
        }
        if self.wait || self.grip.is_some() || self.torque_nm.is_some() {
            return Err(format!("a {key} takes no wait, grip or torque_nm"));
        }
        Ok(())
    }
}

/// What a `command` request asks for.
#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Drive to an openness.
    Set(SetCommand),
    /// Stop and take the actuators' authority away; never opens.
    Stop,
}

/// A `command` request, as a provider receives it.
#[derive(Debug, Clone, PartialEq)]
pub struct CommandEnvelope {
    /// The provider instance the request was validated for.
    pub target_instance_id: u64,
    /// The descriptor generation it was validated against.
    pub descriptor_generation: u64,
    /// After this `CLOCK_MONOTONIC` instant a `set` must not start.
    pub start_by_mono_ns: u64,
    /// What is asked.
    pub command: Command,
}

impl CommandEnvelope {
    /// The wire form.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for an openness outside `[0, 1]`, a
    /// non-finite torque, or an `extra` that does not fit [`crate::EXTRA_LEN`].
    pub fn to_wire(&self, header: wire::Header) -> Result<wire::CommandRequest, ContractError> {
        let mut out = wire::CommandRequest::zeroed();
        out.header = header;
        out.target_instance_id = self.target_instance_id;
        out.descriptor_generation = self.descriptor_generation;
        out.start_by_mono_ns = self.start_by_mono_ns;
        match &self.command {
            Command::Stop => out.op = wire::OP_STOP,
            Command::Set(set) => {
                out.op = wire::OP_SET;
                if !(0.0..=1.0).contains(&set.openness) {
                    return Err(invalid(format!(
                        "openness {} is outside [0, 1]",
                        set.openness
                    )));
                }
                out.openness = set.openness;
                if set.wait {
                    out.flags |= wire::SET_WAIT;
                }
                if let Some(grip) = set.grip {
                    out.flags |= wire::SET_GRIP;
                    out.grip = grip.index();
                }
                if let Some(torque) = set.torque_nm {
                    if !torque.is_finite() {
                        return Err(invalid("torque_nm is not finite"));
                    }
                    out.flags |= wire::SET_TORQUE;
                    out.torque_nm = torque;
                }
                if let Some(extra) = &set.extra {
                    out.extra_len =
                        put_json(&mut out.extra, &Value::Object(extra.clone()), "set extra")?;
                }
            }
        }
        Ok(out)
    }

    /// Read a wire command request.
    ///
    /// # Errors
    ///
    /// [`ContractError::SchemaVersion`], or [`ContractError::Invalid`] for an
    /// unknown op, flag or grip, an openness outside `[0, 1]`, a non-finite
    /// torque, or an `extra` that is not a JSON object.
    pub fn from_wire(message: &wire::CommandRequest) -> Result<Self, ContractError> {
        check_header(&message.header)?;
        let command = match message.op {
            wire::OP_STOP => Command::Stop,
            wire::OP_SET => {
                no_unknown_bits(
                    message.flags,
                    wire::SET_WAIT | wire::SET_GRIP | wire::SET_TORQUE,
                    "set flags",
                )?;
                if !(0.0..=1.0).contains(&message.openness) {
                    return Err(invalid(format!(
                        "openness {} is outside [0, 1]",
                        message.openness
                    )));
                }
                let torque_nm =
                    (message.flags & wire::SET_TORQUE != 0).then_some(message.torque_nm);
                if torque_nm.is_some_and(|torque| !torque.is_finite()) {
                    return Err(invalid("torque_nm is not finite"));
                }
                let extra = match get_json(&message.extra, message.extra_len, "set extra")? {
                    None => None,
                    Some(Value::Object(map)) => Some(map),
                    Some(_) => return Err(invalid("set extra is not a JSON object")),
                };
                Command::Set(SetCommand {
                    openness: message.openness,
                    wait: message.flags & wire::SET_WAIT != 0,
                    grip: (message.flags & wire::SET_GRIP != 0)
                        .then(|| GripPreset::from_index(message.grip))
                        .transpose()?,
                    torque_nm,
                    extra,
                })
            }
            other => return Err(invalid(format!("command op {other} is not an op"))),
        };
        Ok(Self {
            target_instance_id: message.target_instance_id,
            descriptor_generation: message.descriptor_generation,
            start_by_mono_ns: message.start_by_mono_ns,
            command,
        })
    }
}

/// Why a provider did not do what it was asked.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ErrorKind {
    /// A capability not declared; a stroke stopped or halted.
    Refused,
    /// A value the provider cannot take.
    Unprocessable,
    /// The device cannot be driven now.
    Unavailable,
    /// The device failed.
    Device,
    /// The device did not answer in time.
    Timeout,
    /// The request was for another instance or generation, or too late to
    /// start; it was not executed.
    StaleTarget,
    /// The request was malformed.
    Invalid,
}

impl ErrorKind {
    const ALL: [(Self, u32); 7] = [
        (Self::Refused, wire::STATUS_REFUSED),
        (Self::Unprocessable, wire::STATUS_UNPROCESSABLE),
        (Self::Unavailable, wire::STATUS_UNAVAILABLE),
        (Self::Device, wire::STATUS_DEVICE),
        (Self::Timeout, wire::STATUS_TIMEOUT),
        (Self::StaleTarget, wire::STATUS_STALE_TARGET),
        (Self::Invalid, wire::STATUS_INVALID),
    ];

    /// The wire status code.
    #[must_use]
    pub fn status(self) -> u32 {
        Self::ALL
            .iter()
            .find(|(kind, _)| *kind == self)
            .map_or(wire::STATUS_DEVICE, |(_, status)| *status)
    }

    fn from_status(status: u32) -> Result<Self, ContractError> {
        Self::ALL
            .iter()
            .find(|(_, known)| *known == status)
            .map(|(kind, _)| *kind)
            .ok_or_else(|| invalid(format!("status {status} is not a status")))
    }
}

/// A provider's refusal or failure.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{kind:?}: {message}")]
pub struct ProviderError {
    /// What kind of failure.
    pub kind: ErrorKind,
    /// Why, for a person.
    pub message: String,
}

impl ProviderError {
    /// A failure of `kind`.
    #[must_use]
    pub fn new(kind: ErrorKind, message: impl Into<String>) -> Self {
        Self {
            kind,
            message: message.into(),
        }
    }
}

/// The answer to a `command` request: done, or why not.
pub type CommandOutcome = Result<(), ProviderError>;

/// The wire form of a command outcome.
#[must_use]
pub fn command_response(outcome: &CommandOutcome, header: wire::Header) -> wire::CommandResponse {
    let mut out = wire::CommandResponse::zeroed();
    out.header = header;
    if let Err(error) = outcome {
        out.status = error.kind.status();
        put_text(&mut out.message, &error.message);
    }
    out
}

/// Read a wire command response.
///
/// # Errors
///
/// [`ContractError::SchemaVersion`], or [`ContractError::Invalid`] for an
/// unknown status or a malformed message.
pub fn read_command_response(
    message: &wire::CommandResponse,
) -> Result<CommandOutcome, ContractError> {
    check_header(&message.header)?;
    if message.status == wire::STATUS_OK {
        return Ok(Ok(()));
    }
    Ok(Err(ProviderError {
        kind: ErrorKind::from_status(message.status)?,
        message: get_str(&message.message, "response message")?.to_owned(),
    }))
}

/// What a `fault` request asks for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FaultOp {
    /// Clear the device's fault latch.
    ClearFault,
    /// Read registers: these ids, or the provider's default set.
    Registers(Option<Vec<String>>),
}

/// A `fault` request, as a provider receives it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FaultEnvelope {
    /// The provider instance the request is for.
    pub target_instance_id: u64,
    /// What is asked.
    pub op: FaultOp,
}

impl FaultEnvelope {
    /// The wire form.
    ///
    /// # Errors
    ///
    /// [`ContractError::Invalid`] for an id list that does not fit
    /// [`crate::IDS_LEN`].
    pub fn to_wire(&self, header: wire::Header) -> Result<wire::FaultRequest, ContractError> {
        let mut out = wire::FaultRequest::zeroed();
        out.header = header;
        out.target_instance_id = self.target_instance_id;
        match &self.op {
            FaultOp::ClearFault => out.op = wire::OP_CLEAR_FAULT,
            FaultOp::Registers(ids) => {
                out.op = wire::OP_REGISTERS;
                if let Some(ids) = ids {
                    out.ids_len = put_json(
                        &mut out.ids,
                        &Value::Array(ids.iter().cloned().map(Value::String).collect()),
                        "register ids",
                    )?;
                }
            }
        }
        Ok(out)
    }

    /// Read a wire fault request.
    ///
    /// # Errors
    ///
    /// [`ContractError::SchemaVersion`], or [`ContractError::Invalid`] for an
    /// unknown op or an id list that is not a JSON array of strings.
    pub fn from_wire(message: &wire::FaultRequest) -> Result<Self, ContractError> {
        check_header(&message.header)?;
        let op = match message.op {
            wire::OP_CLEAR_FAULT => FaultOp::ClearFault,
            wire::OP_REGISTERS => FaultOp::Registers(
                get_json(&message.ids, message.ids_len, "register ids")?
                    .map(|value| {
                        serde_json::from_value::<Vec<String>>(value)
                            .map_err(|_| invalid("register ids are not a JSON array of strings"))
                    })
                    .transpose()?,
            ),
            other => return Err(invalid(format!("fault op {other} is not an op"))),
        };
        Ok(Self {
            target_instance_id: message.target_instance_id,
            op,
        })
    }
}

/// The result of a fault clear.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FaultClear {
    /// The device no longer reports a fault.
    pub ok: bool,
    /// The fault still standing when `ok` is false.
    pub fault: Option<String>,
    /// The provider's own record of the attempt.
    pub details: Value,
}

/// One register of a dump.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Register {
    /// The register's id.
    pub id: String,
    /// The vendor's name for it.
    pub name: Option<String>,
    /// The decoded value, `null` when it did not answer.
    pub value: Value,
    /// The provider's own record of the read.
    pub details: Value,
}

/// A register dump.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Registers {
    /// One entry per register, in request order.
    pub entries: Vec<Register>,
    /// The provider's report about the dump as a whole.
    pub details: Value,
}

/// The answer to a `fault` request: the JSON body, or why not.
pub type FaultOutcome = Result<Value, ProviderError>;

/// The wire form of a fault outcome. A body larger than [`crate::BODY_LEN`] is
/// answered as a device failure saying so.
#[must_use]
pub fn fault_response(outcome: &FaultOutcome, header: wire::Header) -> wire::FaultResponse {
    let mut out = wire::FaultResponse::zeroed();
    out.header = header;
    match outcome {
        Ok(body) => match put_json(&mut out.body, body, "fault response body") {
            Ok(len) => out.body_len = len,
            Err(error) => {
                out.body.fill(0);
                out.status = wire::STATUS_DEVICE;
                put_text(&mut out.message, &error.to_string());
            }
        },
        Err(error) => {
            out.status = error.kind.status();
            put_text(&mut out.message, &error.message);
        }
    }
    out
}

/// Read a wire fault response.
///
/// # Errors
///
/// [`ContractError::SchemaVersion`], or [`ContractError::Invalid`] for an
/// unknown status or a body that is not JSON.
pub fn read_fault_response(message: &wire::FaultResponse) -> Result<FaultOutcome, ContractError> {
    check_header(&message.header)?;
    if message.status != wire::STATUS_OK {
        return Ok(Err(ProviderError {
            kind: ErrorKind::from_status(message.status)?,
            message: get_str(&message.message, "response message")?.to_owned(),
        }));
    }
    Ok(Ok(get_json(
        &message.body,
        message.body_len,
        "fault response body",
    )?
    .unwrap_or(Value::Null)))
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;
    use crate::{BODY_LEN, DETAIL_LEN};

    fn gripper() -> Descriptor {
        Descriptor {
            driver: "acme-gripper".to_owned(),
            detail_schema: Some("acme.report/1".to_owned()),
            model: "acme/g1".to_owned(),
            node_id: Some(3),
            joints: vec![Joint {
                name: "jaw".to_owned(),
                unit: JointUnit::Rad,
                min: 0.0,
                max: 1.2,
            }],
            capabilities: Capabilities {
                openness: true,
                joints: true,
                stroke_completion: true,
                grip_presets: vec![GripPreset::Soft, GripPreset::Strong],
                grip_torque: true,
                set_extra: vec!["speed".to_owned()],
                force_sensing: ForceSensing::Estimated,
                contact_sensing: true,
                clear_fault: true,
                registers: false,
                ticks: true,
                joint_velocity: true,
                joint_effort: false,
                max_command_hz: Some(200),
            },
            set_timeout_ms: Some(3000),
            tool: Some(Tool {
                mass_kg: 0.62,
                com_mm: [0.0, 1.5, 48.0],
                inertia: [0.0011, 0.0, 0.0, 0.0012, 0.0, 0.0004],
                tcp_xyz_mm: [0.0, 0.0, 121.0],
                tcp_rpy_deg: [0.0, 0.0, 90.0],
            }),
            guard_model: Some("acme/g1-envelope".to_owned()),
        }
    }

    #[test]
    fn a_descriptor_round_trips() {
        let descriptor = gripper();
        let message = descriptor.to_wire(7, header(42, 1)).unwrap();
        assert_eq!(message.header.schema_version, SCHEMA_VERSION);
        assert_eq!(Descriptor::from_wire(&message).unwrap(), (descriptor, 7));

        let without_detail = Descriptor {
            detail_schema: None,
            ..gripper()
        };
        let message = without_detail.to_wire(8, header(42, 2)).unwrap();
        assert!(message.detail_schema.iter().all(|byte| *byte == 0));
        assert_eq!(
            Descriptor::from_wire(&message).unwrap(),
            (without_detail, 8)
        );
    }

    #[test]
    fn the_tool_and_the_guard_model_are_optional() {
        let bare = Descriptor {
            tool: None,
            guard_model: None,
            ..gripper()
        };
        let message = bare.to_wire(1, header(42, 1)).unwrap();
        assert_eq!(message.tool_declared, 0);
        assert_eq!(message.tool, wire::Tool::zeroed());
        assert!(message.guard_model.iter().all(|byte| *byte == 0));
        assert_eq!(Descriptor::from_wire(&message).unwrap(), (bare, 1));

        let declared = gripper().to_wire(1, header(42, 1)).unwrap();
        assert_eq!(declared.tool_declared, 1);
        assert_eq!(declared.tool.tcp_xyz_mm, [0.0, 0.0, 121.0]);
        assert_eq!(declared.tool.inertia[3], 0.0012);
    }

    #[test]
    fn a_malformed_tool_or_guard_model_is_refused() {
        let with_tool = |change: fn(&mut Tool)| {
            let mut descriptor = gripper();
            change(descriptor.tool.as_mut().unwrap());
            descriptor
        };
        let mut empty_guard = gripper();
        empty_guard.guard_model = Some(String::new());
        let mut long_guard = gripper();
        long_guard.guard_model = Some("g".repeat(NAME_LEN + 1));
        for descriptor in [
            with_tool(|tool| tool.mass_kg = -0.1),
            with_tool(|tool| tool.mass_kg = f64::NAN),
            with_tool(|tool| tool.com_mm[2] = f64::INFINITY),
            with_tool(|tool| tool.inertia[0] = f64::NAN),
            with_tool(|tool| tool.tcp_rpy_deg[1] = f64::NEG_INFINITY),
            empty_guard,
            long_guard,
        ] {
            assert!(
                descriptor.to_wire(1, header(1, 1)).is_err(),
                "{descriptor:?}"
            );
        }

        let good = gripper().to_wire(1, header(1, 1)).unwrap();
        let mut flag = good;
        flag.tool_declared = 2;
        let mut nan_tool = good;
        nan_tool.tool.mass_kg = f64::NAN;
        let mut guard_garbage = good;
        guard_garbage.guard_model[63] = b'x';
        let mut unknown_capability = good;
        unknown_capability.capabilities |= 1 << 10;
        for message in [flag, nan_tool, guard_garbage, unknown_capability] {
            assert!(Descriptor::from_wire(&message).is_err());
        }
        // An undeclared tool's bytes are ignored, as a reserved field's are.
        let mut ignored = Descriptor {
            tool: None,
            ..gripper()
        }
        .to_wire(1, header(1, 1))
        .unwrap();
        ignored.tool.mass_kg = f64::NAN;
        assert_eq!(Descriptor::from_wire(&ignored).unwrap().0.tool, None);
    }

    #[test]
    fn a_schema_id_is_an_id_and_a_major_from_one() {
        for good in ["acme.report/1", "damiao.report/1", "hand.state/12", "x/1"] {
            assert!(valid_schema_id(good), "{good}");
        }
        for bad in [
            "",
            "acme.report",
            "/1",
            "acme/",
            "acme/0",
            "acme/01",
            "acme/1.2",
            "acme/v1",
            "acme/1/2",
            "a\0b/1",
        ] {
            assert!(!valid_schema_id(bad), "{bad:?}");
        }
    }

    #[test]
    fn another_schema_version_is_refused() {
        let mut message = gripper().to_wire(1, header(42, 1)).unwrap();
        message.header.schema_version = 2;
        assert_eq!(
            Descriptor::from_wire(&message),
            Err(ContractError::SchemaVersion { got: 2 })
        );
    }

    #[test]
    fn malformed_descriptors_are_refused() {
        let mut too_many = gripper();
        too_many.joints = (0..33)
            .map(|index| Joint {
                name: format!("j{index}"),
                unit: JointUnit::Fraction,
                min: 0.0,
                max: 1.0,
            })
            .collect();
        let mut twice = gripper();
        twice.joints.push(twice.joints[0].clone());
        let mut inverted = gripper();
        inverted.joints[0].max = -1.0;
        let mut long_name = gripper();
        long_name.model = "m".repeat(NAME_LEN + 1);
        let mut zero_hz = gripper();
        zero_hz.capabilities.max_command_hz = Some(0);
        let mut unversioned = gripper();
        unversioned.detail_schema = Some("acme.report".to_owned());
        for descriptor in [too_many, twice, inverted, long_name, zero_hz, unversioned] {
            assert!(
                descriptor.to_wire(1, header(1, 1)).is_err(),
                "{descriptor:?}"
            );
        }

        let good = gripper().to_wire(1, header(1, 1)).unwrap();
        let mut bits = good;
        bits.capabilities |= 1 << 20;
        let mut unit = good;
        unit.joints[0].unit = 9;
        let mut count = good;
        count.joint_count = 40;
        let mut garbage = good;
        garbage.model[60] = b'x';
        let mut no_sender = good;
        no_sender.header.instance_id = 0;
        for message in [bits, unit, count, garbage, no_sender] {
            assert!(Descriptor::from_wire(&message).is_err());
        }
    }

    #[test]
    fn a_state_round_trips_with_its_detail_and_tristates() {
        let state = State {
            openness: Some(0.25),
            joints: vec![0.9],
            live: true,
            tracking: false,
            velocity: vec![Some(-0.3)],
            effort: vec![None],
            holding: Some(true),
            contact: Some(false),
            cap_reached: None,
            torque_nm: Some(0.4),
            fault: Some("over temperature".to_owned()),
            detail: Some(Detail {
                schema: "acme.report/1".to_owned(),
                data: json!({"model": "acme/g1", "kind": "holding"}),
            }),
            detail_dropped: false,
        };
        let message = state.to_wire(3, header(9, 5)).unwrap();
        assert!(message.effort.iter().all(|value| value.is_nan()));
        let (read, generation) = State::from_wire(&message, 1).unwrap();
        assert_eq!(generation, 3);
        // An array no joint of which is known reads back empty.
        assert_eq!(
            read,
            State {
                effort: Vec::new(),
                ..state
            }
        );
    }

    #[test]
    fn velocity_and_effort_are_per_joint_and_optional() {
        let state = State {
            joints: vec![0.1, 0.2, 0.3],
            velocity: vec![Some(1.0), None, Some(-2.0)],
            effort: vec![Some(0.5), Some(0.0), None],
            ..State::default()
        };
        let message = state.to_wire(1, header(9, 1)).unwrap();
        assert!(message.velocity[1].is_nan());
        assert!(message.velocity[3..].iter().all(|value| value.is_nan()));
        assert_eq!(State::from_wire(&message, 3).unwrap().0, state);

        let short = State {
            velocity: vec![Some(1.0)],
            ..state.clone()
        };
        let infinite = State {
            effort: vec![Some(f64::INFINITY), None, None],
            ..state.clone()
        };
        let no_reading = State {
            joints: Vec::new(),
            ..state.clone()
        };
        for bad in [short, infinite, no_reading] {
            assert!(bad.to_wire(1, header(9, 1)).is_err(), "{bad:?}");
        }
        let mut wire = message;
        wire.velocity[0] = f64::INFINITY;
        assert!(State::from_wire(&wire, 3).is_err());
        // Entries past the reading are not read.
        let mut beyond = message;
        beyond.effort[3] = f64::INFINITY;
        assert!(State::from_wire(&beyond, 3).is_ok());
    }

    #[test]
    fn a_state_of_the_wrong_dof_or_with_an_oversize_detail() {
        let state = State {
            joints: vec![0.1, 0.2],
            ..State::default()
        };
        let message = state.to_wire(1, header(9, 1)).unwrap();
        assert!(State::from_wire(&message, 1).is_err());
        assert!(State::from_wire(&message, 2).is_ok());

        let big = State {
            detail: Some(Detail {
                schema: "f/1".to_owned(),
                data: json!("x".repeat(DETAIL_LEN)),
            }),
            ..State::default()
        };
        let (read, _) = State::from_wire(&big.to_wire(1, header(9, 1)).unwrap(), 0).unwrap();
        assert!(read.detail.is_none());
        assert!(read.detail_dropped);
    }

    #[test]
    fn commands_round_trip() {
        let mut extra = Map::new();
        extra.insert("speed".to_owned(), json!(0.5));
        for command in [
            Command::Stop,
            Command::Set(SetCommand {
                openness: 0.0,
                wait: true,
                grip: Some(GripPreset::Strong),
                torque_nm: Some(1.2),
                extra: Some(extra),
            }),
            Command::Set(SetCommand::default()),
        ] {
            let envelope = CommandEnvelope {
                target_instance_id: 5,
                descriptor_generation: 2,
                start_by_mono_ns: 99,
                command,
            };
            let message = envelope.to_wire(header(8, 1)).unwrap();
            assert_eq!(CommandEnvelope::from_wire(&message).unwrap(), envelope);
        }
    }

    #[test]
    fn responses_round_trip() {
        for outcome in [
            Ok(()),
            Err(ProviderError::new(
                ErrorKind::Unprocessable,
                "torque 9 Nm is above 2",
            )),
        ] {
            let message = command_response(&outcome, header(5, 1));
            assert_eq!(read_command_response(&message).unwrap(), outcome);
        }
        let body = json!({"ok": true, "fault": null, "details": {"attempts": 1}});
        let message = fault_response(&Ok(body.clone()), header(5, 2));
        assert_eq!(read_fault_response(&message).unwrap(), Ok(body));
        let oversize = fault_response(&Ok(json!("x".repeat(BODY_LEN))), header(5, 3));
        assert_eq!(
            read_fault_response(&oversize).unwrap().unwrap_err().kind,
            ErrorKind::Device
        );
    }

    #[test]
    fn ticks_and_joints_round_trip() {
        let tick = Tick {
            seq: 4,
            timestamp_us: 1_790_000_000_000_000,
            measured: vec![0.1, 0.2],
            commanded: Some(vec![0.15, 0.25]),
            fault: None,
            phase: TickPhase::Closing,
            effort: None,
        };
        let message = tick.to_wire(2, header(3, 4)).unwrap();
        assert_eq!(Tick::from_wire(&message).unwrap(), (tick, 2));

        let targets = JointTargets {
            target_instance_id: 3,
            descriptor_generation: 2,
            issued_mono_ns: 0,
            timestamp_us: 17,
            targets: vec![0.5],
        };
        let message = targets.to_wire(header(8, 1)).unwrap();
        let read = JointTargets::from_wire(&message).unwrap();
        assert_eq!(read.targets, vec![0.5]);
        assert_eq!(read.issued_mono_ns, message.header.mono_ns);

        let ids = FaultEnvelope {
            target_instance_id: 3,
            op: FaultOp::Registers(Some(vec!["0x0a".to_owned()])),
        };
        assert_eq!(
            FaultEnvelope::from_wire(&ids.to_wire(header(8, 1)).unwrap()).unwrap(),
            ids
        );
    }

    #[test]
    fn text_is_cut_at_a_character_boundary() {
        let mut field = [0_u8; 4];
        put_text(&mut field, "aé日本");
        assert_eq!(get_str(&field, "t").unwrap(), "aé");
    }
}
