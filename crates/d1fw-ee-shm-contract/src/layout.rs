//! The machine-readable layout of the contract.
//!
//! [`document`] describes every message (iceoryx2 type name, size,
//! alignment, and every field's offset, size and type), every bit and code
//! constant, every service's iceoryx2 settings and the timing rules, as
//! computed from this crate's own definitions. The committed
//! `spec/end_effector.shm/1.layout.json` must equal it; a provider in
//! another language checks its own message definitions against that file,
//! so a layout change on either side fails a test on that side.

use serde_json::{Value, json};

use crate::{
    BODY_LEN, CONTRACT, DETAIL_LEN, EXTRA_LEN, FAULT_LEN, ICEORYX2_VERSION, IDS_LEN, KEY_LEN,
    MAX_JOINTS, MAX_SET_EXTRA, MESSAGE_LEN, NAME_LEN, SCHEMA_VERSION,
    model::{DEFAULT_SET_TIMEOUT_MS, MAX_COMMAND_HZ, MAX_SET_TIMEOUT_MS},
    names, ports, wire,
};

/// The repository-relative path of the committed layout.
pub const COMMITTED_PATH: &str = "spec/end_effector.shm/1.layout.json";

/// The SHA-256 of the committed layout file.
///
/// Version 1 of the contract is frozen: a change to a layout, a code or a
/// service setting is version 2, in its own `2.layout.json`. Pinning the
/// file's hash here, and in every provider that vendors a copy of it, makes
/// a change to version 1 fail a test on both sides instead of drifting.
pub const LAYOUT_SHA256: &str = "30414fd5c321b72ee171a14edb56fca3ac2fa19b25881454383fc9a160f5dfee";

fn field_size<T, F>(_: fn(&T) -> &F) -> usize {
    core::mem::size_of::<F>()
}

macro_rules! message {
    ($ty:ident { $($field:ident : $kind:expr),* $(,)? }) => {{
        let fields: Vec<Value> = vec![$(
            json!({
                "name": stringify!($field),
                "offset": core::mem::offset_of!(wire::$ty, $field),
                "size": field_size(|message: &wire::$ty| &message.$field),
                "type": $kind,
            })
        ),*];
        json!({
            "name": stringify!($ty),
            "type_name": wire::type_name::<wire::$ty>(),
            "size": core::mem::size_of::<wire::$ty>(),
            "align": core::mem::align_of::<wire::$ty>(),
            "fields": fields,
        })
    }};
}

fn bytes(len: usize) -> String {
    format!("u8[{len}]")
}

fn floats() -> String {
    format!("f64[{MAX_JOINTS}]")
}

/// Every message, in the order the contract lists them.
#[must_use]
pub fn messages() -> Vec<Value> {
    vec![
        message!(Header {
            schema_version: "u32",
            reserved: "u32",
            instance_id: "u64",
            seq: "u64",
            mono_ns: "u64",
        }),
        message!(Joint {
            name: bytes(KEY_LEN),
            unit: "u32",
            reserved: "u32",
            min: "f64",
            max: "f64",
        }),
        message!(ExtraKey {
            name: bytes(KEY_LEN)
        }),
        message!(Tool {
            mass_kg: "f64",
            com_mm: "f64[3]",
            inertia: "f64[6]",
            tcp_xyz_mm: "f64[3]",
            tcp_rpy_deg: "f64[3]",
        }),
        message!(Descriptor {
            header: "Header",
            generation: "u64",
            driver: bytes(NAME_LEN),
            detail_schema: bytes(NAME_LEN),
            model: bytes(NAME_LEN),
            node_id: "i32",
            joint_count: "u32",
            joints: format!("Joint[{MAX_JOINTS}]"),
            capabilities: "u32",
            grip_presets: "u32",
            force_sensing: "u32",
            max_command_hz: "u32",
            set_timeout_ms: "u32",
            set_extra_count: "u32",
            set_extra: format!("ExtraKey[{MAX_SET_EXTRA}]"),
            tool_declared: "u32",
            reserved: "u32",
            tool: "Tool",
            guard_model: bytes(NAME_LEN),
        }),
        message!(Heartbeat {
            header: "Header",
            descriptor_generation: "u64",
            state_period_ms: "u32",
            reserved: "u32",
        }),
        message!(State {
            header: "Header",
            descriptor_generation: "u64",
            openness: "f64",
            torque_nm: "f64",
            flags: "u32",
            joint_count: "u32",
            joints: floats(),
            velocity: floats(),
            effort: floats(),
            fault: bytes(FAULT_LEN),
            detail_len: "u32",
            reserved: "u32",
            detail: bytes(DETAIL_LEN),
        }),
        message!(Tick {
            header: "Header",
            descriptor_generation: "u64",
            tick_seq: "u64",
            timestamp_us: "u64",
            phase: "u32",
            joint_count: "u32",
            flags: "u32",
            reserved: "u32",
            measured: floats(),
            commanded: floats(),
            effort: floats(),
            fault: bytes(FAULT_LEN),
        }),
        message!(JointsCommand {
            header: "Header",
            target_instance_id: "u64",
            descriptor_generation: "u64",
            timestamp_us: "u64",
            joint_count: "u32",
            reserved: "u32",
            targets: floats(),
        }),
        message!(Control {
            header: "Header",
            stop_generation: "u64",
            period_ms: "u32",
            reserved: "u32",
        }),
        message!(CommandRequest {
            header: "Header",
            target_instance_id: "u64",
            descriptor_generation: "u64",
            start_by_mono_ns: "u64",
            op: "u32",
            flags: "u32",
            grip: "u32",
            extra_len: "u32",
            openness: "f64",
            torque_nm: "f64",
            extra: bytes(EXTRA_LEN),
        }),
        message!(CommandResponse {
            header: "Header",
            status: "u32",
            reserved: "u32",
            message: bytes(MESSAGE_LEN),
        }),
        message!(FaultRequest {
            header: "Header",
            target_instance_id: "u64",
            op: "u32",
            ids_len: "u32",
            ids: bytes(IDS_LEN),
        }),
        message!(FaultResponse {
            header: "Header",
            status: "u32",
            body_len: "u32",
            message: bytes(MESSAGE_LEN),
            body: bytes(BODY_LEN),
        }),
    ]
}

fn topic(
    suffix: &str,
    payload: &str,
    publisher: &str,
    subscriber: &str,
    spec: ports::TopicSpec,
) -> Value {
    json!({
        "suffix": suffix,
        "pattern": "publish_subscribe",
        "payload": payload,
        "publisher": publisher,
        "subscriber": subscriber,
        "history_size": spec.history_size,
        "subscriber_max_buffer_size": spec.buffer_size,
        "enable_safe_overflow": spec.safe_overflow,
        "max_publishers": spec.max_publishers,
        "max_subscribers": spec.max_subscribers,
    })
}

fn rpc(suffix: &str, request: &str, response: &str) -> Value {
    let spec = ports::RPC;
    json!({
        "suffix": suffix,
        "pattern": "request_response",
        "request": request,
        "response": response,
        "server": "provider",
        "client": "daemon",
        "max_servers": spec.max_servers,
        "max_clients": spec.max_clients,
        "max_active_requests_per_client": spec.max_active_requests_per_client,
        "max_response_buffer_size": spec.max_response_buffer_size,
        "enable_safe_overflow_for_requests": false,
        "enable_safe_overflow_for_responses": false,
    })
}

/// Every bit and code constant of [`wire`], by name.
#[must_use]
pub fn codes() -> Value {
    let codes: [(&str, u32); 51] = [
        ("JOINT_UNIT_RAD", wire::JOINT_UNIT_RAD),
        ("JOINT_UNIT_FRACTION", wire::JOINT_UNIT_FRACTION),
        ("CAP_OPENNESS", wire::CAP_OPENNESS),
        ("CAP_JOINTS", wire::CAP_JOINTS),
        ("CAP_STROKE_COMPLETION", wire::CAP_STROKE_COMPLETION),
        ("CAP_GRIP_TORQUE", wire::CAP_GRIP_TORQUE),
        ("CAP_CONTACT_SENSING", wire::CAP_CONTACT_SENSING),
        ("CAP_CLEAR_FAULT", wire::CAP_CLEAR_FAULT),
        ("CAP_REGISTERS", wire::CAP_REGISTERS),
        ("CAP_TICKS", wire::CAP_TICKS),
        ("CAP_JOINT_VELOCITY", wire::CAP_JOINT_VELOCITY),
        ("CAP_JOINT_EFFORT", wire::CAP_JOINT_EFFORT),
        ("GRIP_SOFT", wire::GRIP_SOFT),
        ("GRIP_FIRM", wire::GRIP_FIRM),
        ("GRIP_STRONG", wire::GRIP_STRONG),
        ("FORCE_SENSING_NONE", wire::FORCE_SENSING_NONE),
        ("FORCE_SENSING_ESTIMATED", wire::FORCE_SENSING_ESTIMATED),
        ("FORCE_SENSING_MEASURED", wire::FORCE_SENSING_MEASURED),
        ("STATE_LIVE", wire::STATE_LIVE),
        ("STATE_TRACKING", wire::STATE_TRACKING),
        ("STATE_HOLDING_KNOWN", wire::STATE_HOLDING_KNOWN),
        ("STATE_HOLDING", wire::STATE_HOLDING),
        ("STATE_CONTACT_KNOWN", wire::STATE_CONTACT_KNOWN),
        ("STATE_CONTACT", wire::STATE_CONTACT),
        ("STATE_CAP_REACHED_KNOWN", wire::STATE_CAP_REACHED_KNOWN),
        ("STATE_CAP_REACHED", wire::STATE_CAP_REACHED),
        ("STATE_DETAIL_DROPPED", wire::STATE_DETAIL_DROPPED),
        ("PHASE_IDLE", wire::PHASE_IDLE),
        ("PHASE_CLOSING", wire::PHASE_CLOSING),
        ("PHASE_OPENING", wire::PHASE_OPENING),
        ("PHASE_HOLD", wire::PHASE_HOLD),
        ("TICK_COMMANDED", wire::TICK_COMMANDED),
        ("TICK_EFFORT", wire::TICK_EFFORT),
        ("OP_SET", wire::OP_SET),
        ("OP_STOP", wire::OP_STOP),
        ("OP_CLEAR_FAULT", wire::OP_CLEAR_FAULT),
        ("OP_REGISTERS", wire::OP_REGISTERS),
        ("SET_WAIT", wire::SET_WAIT),
        ("SET_GRIP", wire::SET_GRIP),
        ("SET_TORQUE", wire::SET_TORQUE),
        ("GRIP_INDEX_SOFT", wire::GRIP_INDEX_SOFT),
        ("GRIP_INDEX_FIRM", wire::GRIP_INDEX_FIRM),
        ("GRIP_INDEX_STRONG", wire::GRIP_INDEX_STRONG),
        ("STATUS_OK", wire::STATUS_OK),
        ("STATUS_REFUSED", wire::STATUS_REFUSED),
        ("STATUS_UNPROCESSABLE", wire::STATUS_UNPROCESSABLE),
        ("STATUS_UNAVAILABLE", wire::STATUS_UNAVAILABLE),
        ("STATUS_DEVICE", wire::STATUS_DEVICE),
        ("STATUS_TIMEOUT", wire::STATUS_TIMEOUT),
        ("STATUS_STALE_TARGET", wire::STATUS_STALE_TARGET),
        ("STATUS_INVALID", wire::STATUS_INVALID),
    ];
    Value::Object(
        codes
            .into_iter()
            .map(|(name, value)| (name.to_owned(), Value::from(value)))
            .collect(),
    )
}

/// The whole layout document.
#[must_use]
pub fn document() -> Value {
    json!({
        "contract": CONTRACT,
        "schema_version": SCHEMA_VERSION,
        "iceoryx2": ICEORYX2_VERSION,
        "byte_order": "little",
        "root": {
            "root_path": crate::root::DEFAULT_ROOT_PATH,
            "prefix": crate::root::DEFAULT_PREFIX,
            "default_base": names::default_base("<side>"),
        },
        "limits": {
            "MAX_JOINTS": MAX_JOINTS,
            "NAME_LEN": NAME_LEN,
            "KEY_LEN": KEY_LEN,
            "MAX_SET_EXTRA": MAX_SET_EXTRA,
            "FAULT_LEN": FAULT_LEN,
            "MESSAGE_LEN": MESSAGE_LEN,
            "DETAIL_LEN": DETAIL_LEN,
            "EXTRA_LEN": EXTRA_LEN,
            "IDS_LEN": IDS_LEN,
            "BODY_LEN": BODY_LEN,
            "MAX_BASE_LEN": names::MAX_BASE_LEN,
            "DEFAULT_SET_TIMEOUT_MS": DEFAULT_SET_TIMEOUT_MS,
            "MAX_SET_TIMEOUT_MS": MAX_SET_TIMEOUT_MS,
            "MAX_COMMAND_HZ": MAX_COMMAND_HZ,
        },
        "timing_ms": {
            "LATCHED_REPUBLISH_MS": ports::LATCHED_REPUBLISH_MS,
            "MAX_HEARTBEAT_PERIOD_MS": ports::MAX_HEARTBEAT_PERIOD_MS,
            "DEFAULT_STALE_AFTER_MS": ports::DEFAULT_STALE_AFTER_MS,
            "CONTROL_PERIOD_MS": ports::CONTROL_PERIOD_MS,
            "START_WINDOW_MS": ports::START_WINDOW_MS,
            "JOINTS_MAX_AGE_MS": ports::JOINTS_MAX_AGE_MS,
        },
        "codes": codes(),
        "messages": messages(),
        "services": [
            topic(names::DESCRIPTOR, "Descriptor", "provider", "daemon", ports::DESCRIPTOR_TOPIC),
            topic(names::HEARTBEAT, "Heartbeat", "provider", "daemon", ports::HEARTBEAT_TOPIC),
            topic(names::STATE, "State", "provider", "daemon", ports::STATE_TOPIC),
            topic(names::TICK, "Tick", "provider", "daemon", ports::TICK_TOPIC),
            topic(names::JOINTS, "JointsCommand", "daemon", "provider", ports::JOINTS_TOPIC),
            topic(names::CONTROL, "Control", "daemon", "provider", ports::CONTROL_TOPIC),
            rpc(names::COMMAND, "CommandRequest", "CommandResponse"),
            rpc(names::FAULT, "FaultRequest", "FaultResponse"),
        ],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// No message has implicit padding: its fields tile it from offset 0
    /// to its size, so a definition in another language that lists the
    /// same fields in the same order has the same layout.
    #[test]
    fn every_message_is_tiled_by_its_fields() {
        for message in messages() {
            let mut next = 0_u64;
            for field in message["fields"].as_array().unwrap() {
                assert_eq!(
                    field["offset"].as_u64().unwrap(),
                    next,
                    "{} field {} is not where the previous one ended",
                    message["name"],
                    field["name"]
                );
                next += field["size"].as_u64().unwrap();
            }
            assert_eq!(
                next,
                message["size"].as_u64().unwrap(),
                "{}",
                message["name"]
            );
            assert_eq!(
                message["size"].as_u64().unwrap() % message["align"].as_u64().unwrap(),
                0,
                "{}",
                message["name"]
            );
        }
    }

    /// The contract's text lists every field where the layout puts it, so
    /// the prose a provider author reads cannot drift from the bytes.
    #[test]
    fn the_contract_text_mirrors_the_layout() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../spec/end_effector.shm/1.md");
        let text = std::fs::read_to_string(&path).expect("read the contract text");
        for message in messages() {
            let heading = format!(
                "#### `{}` — `{}`, {} bytes, align {}",
                message["name"].as_str().unwrap(),
                message["type_name"].as_str().unwrap(),
                message["size"],
                message["align"]
            );
            assert!(text.contains(&heading), "missing: {heading}");
            for field in message["fields"].as_array().unwrap() {
                let row = format!(
                    "| {} | {} | `{}` | `{}` |",
                    field["offset"],
                    field["size"],
                    field["name"].as_str().unwrap(),
                    field["type"].as_str().unwrap()
                );
                assert!(
                    text.contains(&row),
                    "{}: missing row {row}",
                    message["name"]
                );
            }
        }
    }

    /// Version 1's layout file is the one whose hash is pinned.
    #[test]
    fn the_committed_layout_is_the_frozen_version_1() {
        use sha2::{Digest, Sha256};
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../spec/end_effector.shm/1.layout.json");
        let bytes = std::fs::read(&path).expect("read the committed layout");
        let digest = Sha256::digest(&bytes);
        let hex: String = digest.iter().map(|byte| format!("{byte:02x}")).collect();
        assert_eq!(
            hex, LAYOUT_SHA256,
            "{COMMITTED_PATH} changed: version 1 is frozen; a layout change is version 2"
        );
    }

    /// The committed layout is what these definitions produce. Regenerate it
    /// with `D1FW_EE_SHM_WRITE_LAYOUT=1 cargo test -p d1fw-ee-shm-contract
    /// layout_matches`.
    #[test]
    fn layout_matches_the_committed_spec() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../spec/end_effector.shm/1.layout.json");
        let generated = format!(
            "{}\n",
            serde_json::to_string_pretty(&document()).expect("serialize the layout")
        );
        if std::env::var_os("D1FW_EE_SHM_WRITE_LAYOUT").is_some() {
            std::fs::write(&path, &generated).expect("write the layout");
        }
        let committed = std::fs::read_to_string(&path).expect("read the committed layout");
        assert!(
            committed == generated,
            "{COMMITTED_PATH} differs from the definitions; regenerate it with \
             D1FW_EE_SHM_WRITE_LAYOUT=1 cargo test -p d1fw-ee-shm-contract layout_matches, \
             and treat any difference as a contract change"
        );
    }
}
