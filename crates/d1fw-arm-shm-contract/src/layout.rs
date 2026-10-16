//! The machine-readable layout of each arm contract, and its pin.
//!
//! Each `*_document` function describes one contract — every message, every
//! code, every service's iceoryx2 settings and the timing rules — as
//! computed from this crate's own definitions. The committed
//! `spec/<contract>/1.layout.json` must equal it, and its SHA-256 is pinned
//! in the matching `*_LAYOUT_SHA256`. Version 1 of each contract is frozen
//! once released: a change to a layout, a code or a service setting is that
//! contract's version 2, in its own `2.layout.json`.

use serde_json::{Value, json};

use crate::{
    CART_DOF, COMMAND_CONTRACT, DEFAULT_COMMAND_MAX_AGE_MS, DEFAULT_MAX_COMMAND_HZ, EVENT_VALUES,
    EVENTS_CONTRACT, FEEDBACK_CONTRACT, FRAME_MISS_HEALTHY_BELOW, HOLDER_LEN, ICEORYX2_VERSION,
    JOINTS, MAX_COMMAND_HZ, MAX_COMMAND_MAX_AGE_MS, MESSAGE_LEN, PARA_NAME_LEN, RAW_CAPACITY,
    SCHEMA_VERSION, SIDES, STATUS_CONTRACT, STATUS_PERIOD_MS, TEXT_LEN, TOOL_DYN_LEN, names,
    ports::{self, EventSpec, TopicSpec},
    wire,
};

/// The SHA-256 of `spec/arm_feedback.shm/1.layout.json`.
pub const FEEDBACK_LAYOUT_SHA256: &str =
    "f9bbac48eff4d24449d66f0219ee32960ecf58cf39aebc4d49b522909758b65a";
/// The SHA-256 of `spec/arm_status.shm/1.layout.json`.
pub const STATUS_LAYOUT_SHA256: &str =
    "c7d6685594206be6d58ed7d7f28fe5701eaf3818dc7d40d45156143c621868fb";
/// The SHA-256 of `spec/arm_events.shm/1.layout.json`.
pub const EVENTS_LAYOUT_SHA256: &str =
    "2377745c6660434a2b805afb5feb21b7299a38cbfa03928b3e53a1d87142d619";
/// The SHA-256 of `spec/arm_command.shm/1.layout.json`.
pub const COMMAND_LAYOUT_SHA256: &str =
    "987222e48bb079782e01a5d541b0bf19017a67fdf53dbacbc4172d278493c4be";

/// Each contract's name, its committed layout's repository-relative path,
/// its document and its pin.
#[must_use]
pub fn contracts() -> [(&'static str, &'static str, Value, &'static str); 4] {
    [
        (
            FEEDBACK_CONTRACT,
            "spec/arm_feedback.shm/1.layout.json",
            feedback_document(),
            FEEDBACK_LAYOUT_SHA256,
        ),
        (
            STATUS_CONTRACT,
            "spec/arm_status.shm/1.layout.json",
            status_document(),
            STATUS_LAYOUT_SHA256,
        ),
        (
            EVENTS_CONTRACT,
            "spec/arm_events.shm/1.layout.json",
            events_document(),
            EVENTS_LAYOUT_SHA256,
        ),
        (
            COMMAND_CONTRACT,
            "spec/arm_command.shm/1.layout.json",
            command_document(),
            COMMAND_LAYOUT_SHA256,
        ),
    ]
}

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
            "type_name": d1fw_ee_shm_contract::wire::type_name::<wire::$ty>(),
            "size": core::mem::size_of::<wire::$ty>(),
            "align": core::mem::align_of::<wire::$ty>(),
            "fields": fields,
        })
    }};
}

fn header() -> Value {
    message!(Header {
        schema_version: "u32",
        reserved: "u32",
        instance_id: "u64",
        seq: "u64",
        mono_ns: "u64",
    })
}

fn codes(entries: &[(&str, i64)]) -> Value {
    Value::Object(
        entries
            .iter()
            .map(|(name, value)| ((*name).to_owned(), Value::from(*value)))
            .collect(),
    )
}

fn topic(suffix: &str, payload: &str, publisher: &str, subscriber: &str, spec: TopicSpec) -> Value {
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

fn event_service(suffix: &str, notifier: &str, listener: &str, spec: EventSpec) -> Value {
    json!({
        "suffix": suffix,
        "pattern": "event",
        "notifier": notifier,
        "listener": listener,
        "max_notifiers": spec.max_notifiers,
        "max_listeners": spec.max_listeners,
    })
}

fn root() -> Value {
    json!({
        "root_path": crate::root::DEFAULT_ROOT_PATH,
        "prefix": crate::root::DEFAULT_PREFIX,
        "default_base": names::DEFAULT_BASE,
    })
}

fn document(contract: &str, rest: Value) -> Value {
    let mut document = json!({
        "contract": contract,
        "schema_version": SCHEMA_VERSION,
        "iceoryx2": ICEORYX2_VERSION,
        "byte_order": "little",
        "root": root(),
    });
    if let (Value::Object(document), Value::Object(rest)) = (&mut document, rest) {
        document.extend(rest);
    }
    document
}

/// The messages of `arm_feedback.shm/1`.
#[must_use]
pub fn feedback_messages() -> Vec<Value> {
    let joints = format!("f32[{JOINTS}]");
    let cart = format!("f32[{CART_DOF}]");
    vec![
        header(),
        message!(SideFeedback {
            cur_state: "i32",
            cmd_state: "i32",
            error_code: "i32",
            imp_type: "i32",
            out_frame_serial: "i32",
            in_frame_serial: "i32",
            sys_cyc: "i32",
            cart_kd_type: "i32",
            force_fb_type: "i32",
            force_type: "i32",
            frame_miss_count: "i16",
            max_frame_miss_count: "i16",
            sys_cyc_miss_count: "i16",
            max_sys_cyc_miss_count: "i16",
            vel_ratio: "i16",
            acc_ratio: "i16",
            tip_di: "u8",
            low_speed: "u8",
            traj_state: "u8",
            reserved: "u8",
            pos: joints.clone(),
            vel: joints.clone(),
            pos_e: joints.clone(),
            cmd_echo: joints.clone(),
            current: joints.clone(),
            torque: joints.clone(),
            temperature: joints.clone(),
            est_friction: joints.clone(),
            est_friction_dot: joints.clone(),
            est_force: joints.clone(),
            accepted_cmd: joints.clone(),
            joint_k: joints.clone(),
            joint_d: joints.clone(),
            force_pidul: joints,
            est_cart_fn: cart.clone(),
            cart_k: cart.clone(),
            cart_d: cart.clone(),
            force_dir: cart.clone(),
            tool_kine: cart,
            tool_dyn: format!("f32[{TOOL_DYN_LEN}]"),
            cart_kn: "f32",
            cart_dn: "f32",
            force_adj_lmt: "f32",
            force_cmd: "f32",
        }),
        message!(Feedback {
            header: "Header",
            rx_mono_ns: "u64",
            controller_version: "i32",
            protocol: "u32",
            flags: "u32",
            raw_format: "u32",
            raw_len: "u32",
            para_type: "u8",
            para_ins: "u8",
            para_cmd_serial: "i16",
            para_ret_serial: "i16",
            reserved: "u16",
            para_value_i: "i32",
            para_value_f: "f32",
            reserved2: "u32",
            para_name: format!("u8[{PARA_NAME_LEN}]"),
            sides: format!("SideFeedback[{SIDES}]"),
            raw: format!("u8[{RAW_CAPACITY}]"),
        }),
    ]
}

/// The whole layout document of `arm_feedback.shm/1`.
#[must_use]
pub fn feedback_document() -> Value {
    document(
        FEEDBACK_CONTRACT,
        json!({
            "limits": {
                "SIDES": SIDES,
                "JOINTS": JOINTS,
                "CART_DOF": CART_DOF,
                "TOOL_DYN_LEN": TOOL_DYN_LEN,
                "PARA_NAME_LEN": PARA_NAME_LEN,
                "RAW_CAPACITY": RAW_CAPACITY,
                "FRAME_MISS_HEALTHY_BELOW": FRAME_MISS_HEALTHY_BELOW,
            },
            "codes": codes(&[
                ("PROTOCOL_UNKNOWN", i64::from(wire::PROTOCOL_UNKNOWN)),
                ("PROTOCOL_FX", i64::from(wire::PROTOCOL_FX)),
                ("PROTOCOL_FY", i64::from(wire::PROTOCOL_FY)),
                ("FEEDBACK_PROTOCOL_MISMATCH", i64::from(wire::FEEDBACK_PROTOCOL_MISMATCH)),
                ("FEEDBACK_RAW_TRUNCATED", i64::from(wire::FEEDBACK_RAW_TRUNCATED)),
                ("RAW_DCSS_1428", i64::from(wire::RAW_DCSS_1428)),
                ("STATE_IDLE", i64::from(wire::STATE_IDLE)),
                ("STATE_POSITION", i64::from(wire::STATE_POSITION)),
                ("STATE_PVT", i64::from(wire::STATE_PVT)),
                ("STATE_TORQUE", i64::from(wire::STATE_TORQUE)),
                ("STATE_RELEASE", i64::from(wire::STATE_RELEASE)),
                ("STATE_ERROR", i64::from(wire::STATE_ERROR)),
                ("STATE_TO_POSITION", i64::from(wire::STATE_TO_POSITION)),
                ("STATE_TO_PVT", i64::from(wire::STATE_TO_PVT)),
                ("STATE_TO_TORQUE", i64::from(wire::STATE_TO_TORQUE)),
                ("STATE_TO_RELEASE", i64::from(wire::STATE_TO_RELEASE)),
                ("STATE_TO_IDLE", i64::from(wire::STATE_TO_IDLE)),
            ]),
            "messages": feedback_messages(),
            "services": [
                topic(names::FEEDBACK, "Feedback", "daemon", "reader", ports::FEEDBACK_TOPIC),
            ],
        }),
    )
}

/// The messages of `arm_status.shm/1`.
#[must_use]
pub fn status_messages() -> Vec<Value> {
    vec![
        header(),
        message!(SideStatus {
            mode: "u32",
            error_code: "i32",
            flags: "u32",
            tool_source: "u32",
            tool_mass_kg: "f64",
            frame_miss_count: "i32",
            max_frame_miss_count: "i32",
            sys_cyc_miss_count: "i32",
            reserved: "u32",
        }),
        message!(Status {
            header: "Header",
            feedback_seq: "u64",
            feedback_age_ns: "u64",
            lease_epoch: "u64",
            lease_expires_in_s: "u64",
            commands_accepted: "u64",
            commands_refused: "u64",
            commands_superseded: "u64",
            trajectory_id: "u64",
            flags: "u32",
            lease_class_rank: "u32",
            max_command_hz: "u32",
            feedback_stale_after_ms: "u32",
            trajectory_phase: "u32",
            period_ms: "u32",
            lease_holder: format!("u8[{HOLDER_LEN}]"),
            sides: format!("SideStatus[{SIDES}]"),
        }),
    ]
}

/// The whole layout document of `arm_status.shm/1`.
#[must_use]
pub fn status_document() -> Value {
    document(
        STATUS_CONTRACT,
        json!({
            "limits": { "SIDES": SIDES, "HOLDER_LEN": HOLDER_LEN },
            "timing_ms": { "STATUS_PERIOD_MS": STATUS_PERIOD_MS },
            "codes": codes(&[
                ("MODE_UNKNOWN", i64::from(wire::MODE_UNKNOWN)),
                ("MODE_IDLE", i64::from(wire::MODE_IDLE)),
                ("MODE_POSITION", i64::from(wire::MODE_POSITION)),
                ("MODE_PVT", i64::from(wire::MODE_PVT)),
                ("MODE_TORQUE", i64::from(wire::MODE_TORQUE)),
                ("MODE_RELEASE", i64::from(wire::MODE_RELEASE)),
                ("MODE_ERROR", i64::from(wire::MODE_ERROR)),
                ("SIDE_BRAKES_RELEASED", i64::from(wire::SIDE_BRAKES_RELEASED)),
                ("SIDE_PROTOCOL_MISMATCH", i64::from(wire::SIDE_PROTOCOL_MISMATCH)),
                ("SIDE_STATIONARY", i64::from(wire::SIDE_STATIONARY)),
                ("TOOL_NONE", i64::from(wire::TOOL_NONE)),
                ("TOOL_CONFIG", i64::from(wire::TOOL_CONFIG)),
                ("TOOL_CLIENT", i64::from(wire::TOOL_CLIENT)),
                ("STATUS_SOFT_KILLED", i64::from(wire::STATUS_SOFT_KILLED)),
                ("STATUS_LEASE_HELD", i64::from(wire::STATUS_LEASE_HELD)),
                ("STATUS_COMMAND_SERVED", i64::from(wire::STATUS_COMMAND_SERVED)),
                ("STATUS_FEEDBACK_STALE", i64::from(wire::STATUS_FEEDBACK_STALE)),
                ("STATUS_STREAM_STOPPED", i64::from(wire::STATUS_STREAM_STOPPED)),
                ("TRAJECTORY_NONE", i64::from(wire::TRAJECTORY_NONE)),
                ("TRAJECTORY_RUNNING", i64::from(wire::TRAJECTORY_RUNNING)),
                ("TRAJECTORY_COMPLETED", i64::from(wire::TRAJECTORY_COMPLETED)),
                ("TRAJECTORY_CANCELLED", i64::from(wire::TRAJECTORY_CANCELLED)),
                ("TRAJECTORY_FAILED", i64::from(wire::TRAJECTORY_FAILED)),
            ]),
            "messages": status_messages(),
            "services": [
                topic(names::STATUS, "Status", "daemon", "reader", ports::STATUS_TOPIC),
            ],
        }),
    )
}

/// The messages of `arm_events.shm/1`.
#[must_use]
pub fn events_messages() -> Vec<Value> {
    vec![
        header(),
        message!(Event {
            header: "Header",
            kind: "u32",
            sides: "u32",
            code: "i32",
            joint: "i32",
            values: format!("f64[{EVENT_VALUES}]"),
            text: format!("u8[{TEXT_LEN}]"),
        }),
    ]
}

/// The whole layout document of `arm_events.shm/1`.
#[must_use]
pub fn events_document() -> Value {
    document(
        EVENTS_CONTRACT,
        json!({
            "limits": { "EVENT_VALUES": EVENT_VALUES, "TEXT_LEN": TEXT_LEN },
            "codes": codes(&[
                ("SIDE_MASK_A", i64::from(wire::SIDE_MASK_A)),
                ("SIDE_MASK_B", i64::from(wire::SIDE_MASK_B)),
                ("EVENT_CONTROLLER_STATE", i64::from(wire::EVENT_CONTROLLER_STATE)),
                ("EVENT_CONTROLLER_ERROR", i64::from(wire::EVENT_CONTROLLER_ERROR)),
                ("EVENT_FRAME_MISS_HIGH", i64::from(wire::EVENT_FRAME_MISS_HIGH)),
                ("EVENT_FRAME_MISS_NORMAL", i64::from(wire::EVENT_FRAME_MISS_NORMAL)),
                ("EVENT_SOFT_KILL_LATCHED", i64::from(wire::EVENT_SOFT_KILL_LATCHED)),
                ("EVENT_SOFT_KILL_RELEASED", i64::from(wire::EVENT_SOFT_KILL_RELEASED)),
                ("EVENT_LEASE_CHANGED", i64::from(wire::EVENT_LEASE_CHANGED)),
                ("EVENT_BRAKES_RELEASED", i64::from(wire::EVENT_BRAKES_RELEASED)),
                ("EVENT_BRAKES_ENGAGED", i64::from(wire::EVENT_BRAKES_ENGAGED)),
                ("EVENT_GUARD_REFUSED", i64::from(wire::EVENT_GUARD_REFUSED)),
                ("EVENT_APPROACH_SLOWED", i64::from(wire::EVENT_APPROACH_SLOWED)),
                ("EVENT_STREAM_STOPPED", i64::from(wire::EVENT_STREAM_STOPPED)),
                ("EVENT_STREAM_RESUMED", i64::from(wire::EVENT_STREAM_RESUMED)),
                ("EVENT_EMERGENCY_STOP", i64::from(wire::EVENT_EMERGENCY_STOP)),
                ("EVENT_TRAJECTORY_STARTED", i64::from(wire::EVENT_TRAJECTORY_STARTED)),
                ("EVENT_TRAJECTORY_ENDED", i64::from(wire::EVENT_TRAJECTORY_ENDED)),
            ]),
            "messages": events_messages(),
            "services": [
                topic(names::EVENTS, "Event", "daemon", "reader", ports::EVENTS_TOPIC),
            ],
        }),
    )
}

/// The messages of `arm_command.shm/1`.
#[must_use]
pub fn command_messages() -> Vec<Value> {
    let targets = format!("f64[{SIDES}][{JOINTS}]");
    vec![
        header(),
        message!(Command {
            header: "Header",
            lease_token: "u64",
            max_age_ns: "u64",
            sides: "u32",
            flags: "u32",
            target_deg: targets.clone(),
        }),
        message!(CommandStatus {
            header: "Header",
            client_instance_id: "u64",
            client_seq: "u64",
            client_mono_ns: "u64",
            rx_mono_ns: "u64",
            sent_mono_ns: "u64",
            accepted: "u64",
            refused: "u64",
            superseded: "u64",
            result: "u32",
            flags: "u32",
            scale: format!("f64[{SIDES}]"),
            written_deg: targets,
            message: format!("u8[{MESSAGE_LEN}]"),
        }),
    ]
}

/// The whole layout document of `arm_command.shm/1`.
#[must_use]
pub fn command_document() -> Value {
    document(
        COMMAND_CONTRACT,
        json!({
            "limits": {
                "SIDES": SIDES,
                "JOINTS": JOINTS,
                "MESSAGE_LEN": MESSAGE_LEN,
                "DEFAULT_MAX_COMMAND_HZ": DEFAULT_MAX_COMMAND_HZ,
                "MAX_COMMAND_HZ": MAX_COMMAND_HZ,
            },
            "timing_ms": {
                "DEFAULT_COMMAND_MAX_AGE_MS": DEFAULT_COMMAND_MAX_AGE_MS,
                "MAX_COMMAND_MAX_AGE_MS": MAX_COMMAND_MAX_AGE_MS,
            },
            "codes": codes(&[
                ("SIDE_MASK_A", i64::from(wire::SIDE_MASK_A)),
                ("SIDE_MASK_B", i64::from(wire::SIDE_MASK_B)),
                ("RESULT_ACCEPTED", i64::from(wire::RESULT_ACCEPTED)),
                ("RESULT_INVALID", i64::from(wire::RESULT_INVALID)),
                ("RESULT_STALE", i64::from(wire::RESULT_STALE)),
                ("RESULT_LEASE", i64::from(wire::RESULT_LEASE)),
                ("RESULT_SOFT_KILLED", i64::from(wire::RESULT_SOFT_KILLED)),
                ("RESULT_NOT_READY", i64::from(wire::RESULT_NOT_READY)),
                ("RESULT_GUARD", i64::from(wire::RESULT_GUARD)),
                ("RESULT_BUSY", i64::from(wire::RESULT_BUSY)),
                ("RESULT_RATE", i64::from(wire::RESULT_RATE)),
                ("RESULT_DEVICE", i64::from(wire::RESULT_DEVICE)),
                ("RESULT_LIMITS", i64::from(wire::RESULT_LIMITS)),
                ("CMD_SLOWED", i64::from(wire::CMD_SLOWED)),
            ]),
            "messages": command_messages(),
            "services": [
                topic(names::COMMAND, "Command", "client", "daemon", ports::COMMAND_TOPIC),
                event_service(names::COMMAND, "client", "daemon", ports::COMMAND_EVENT),
                topic(
                    names::COMMAND_STATUS,
                    "CommandStatus",
                    "daemon",
                    "client",
                    ports::COMMAND_STATUS_TOPIC,
                ),
            ],
        }),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn repo_path(relative: &str) -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(relative)
    }

    #[test]
    fn every_message_is_tiled_by_its_fields() {
        for (contract, _, document, _) in contracts() {
            for message in document["messages"].as_array().unwrap() {
                let mut next = 0_u64;
                for field in message["fields"].as_array().unwrap() {
                    assert_eq!(
                        field["offset"].as_u64().unwrap(),
                        next,
                        "{contract} {} field {} is not where the previous one ended",
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
    }

    #[test]
    fn the_contract_texts_mirror_the_layouts() {
        for (contract, path, document, _) in contracts() {
            let text_path = repo_path(&path.replace("1.layout.json", "1.md"));
            let text = std::fs::read_to_string(&text_path)
                .unwrap_or_else(|error| panic!("read {}: {error}", text_path.display()));
            for message in document["messages"].as_array().unwrap() {
                let heading = format!(
                    "#### `{}` — `{}`, {} bytes, align {}",
                    message["name"].as_str().unwrap(),
                    message["type_name"].as_str().unwrap(),
                    message["size"],
                    message["align"]
                );
                assert!(text.contains(&heading), "{contract}: missing {heading}");
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
                        "{contract} {}: missing row {row}",
                        message["name"]
                    );
                }
            }
            for (name, value) in document["codes"].as_object().unwrap() {
                let row = format!("`{name}` | {value} |");
                assert!(text.contains(&row), "{contract}: missing code row {row}");
            }
        }
    }

    #[test]
    fn each_committed_layout_is_its_pinned_version_1() {
        use sha2::{Digest, Sha256};
        for (contract, path, _, pinned) in contracts() {
            let bytes = std::fs::read(repo_path(path)).expect("read the committed layout");
            let hex: String = Sha256::digest(&bytes)
                .iter()
                .map(|byte| format!("{byte:02x}"))
                .collect();
            assert_eq!(
                hex, pinned,
                "{path} changed: version 1 of {contract} is frozen; a layout change is version 2"
            );
            let text = std::fs::read_to_string(repo_path(&path.replace("1.layout.json", "1.md")))
                .expect("read the contract text");
            assert!(
                text.contains(pinned),
                "the text of {contract} must state its layout's SHA-256 {pinned}"
            );
        }
    }

    /// Regenerate the committed layouts with `D1FW_ARM_WRITE_LAYOUT=1 cargo
    /// test -p d1fw-arm-shm-contract layout`.
    #[test]
    fn each_layout_matches_its_committed_spec() {
        for (contract, path, document, _) in contracts() {
            let generated = format!(
                "{}\n",
                serde_json::to_string_pretty(&document).expect("serialize the layout")
            );
            if std::env::var_os("D1FW_ARM_WRITE_LAYOUT").is_some() {
                std::fs::write(repo_path(path), &generated).expect("write the layout");
            }
            let committed = std::fs::read_to_string(repo_path(path)).expect("read the layout");
            assert!(
                committed == generated,
                "{path} differs from the definitions of {contract}; regenerate it with \
                 D1FW_ARM_WRITE_LAYOUT=1 cargo test -p d1fw-arm-shm-contract layout, and treat \
                 any difference as a contract change"
            );
        }
    }
}
