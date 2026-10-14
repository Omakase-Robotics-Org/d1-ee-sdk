//! The machine-readable layout of `flange_bus.shm/1`.
//!
//! [`document`] describes every message, every bit and code, every
//! service's iceoryx2 settings and the timing rules, as computed from this
//! module's own definitions. The committed
//! `spec/flange_bus.shm/1.layout.json` must equal it, and its SHA-256 is
//! pinned in [`LAYOUT_SHA256`].

use serde_json::{Value, json};

use super::{
    CONTRACT, DEFAULT_MAX_TX_HZ, ERROR_LEN, EXTENDED_ID_MAX, HEARTBEAT, HEARTBEAT_TOPIC,
    MAX_FRAME_LEN, MAX_HEARTBEAT_PERIOD_MS, MAX_TX_HZ, RX, RX_TOPIC, SCHEMA_VERSION,
    STANDARD_ID_MAX, STATUS, STATUS_PERIOD_MS, STATUS_TOPIC, TX, TX_MAX_AGE_MS, TX_TOPIC,
    TX_WINDOW, default_base, wire,
};
use crate::{ICEORYX2_VERSION, ports::TopicSpec};

/// The repository-relative path of the committed layout.
pub const COMMITTED_PATH: &str = "spec/flange_bus.shm/1.layout.json";

/// The SHA-256 of the committed layout file.
///
/// Version 1 of the contract is frozen once released: a change to a
/// layout, a code or a service setting is version 2, in its own
/// `2.layout.json`. Every participant pins this hash (the daemon and the
/// Rust SDK through this crate, a provider in another language in its own
/// test), so a change fails a test on each side instead of drifting.
pub const LAYOUT_SHA256: &str = "43d74ebf4944cf21c40e368cf6816ef0bc71698a5b2fe34988d8dfe12bf766b7";

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
            "type_name": crate::wire::type_name::<wire::$ty>(),
            "size": core::mem::size_of::<wire::$ty>(),
            "align": core::mem::align_of::<wire::$ty>(),
            "fields": fields,
        })
    }};
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
        message!(Frame {
            id: "u32",
            flags: "u32",
            len: "u32",
            reserved: "u32",
            data: format!("u8[{MAX_FRAME_LEN}]"),
        }),
        message!(RxFrame {
            header: "Header",
            channel_mono_ns: "u64",
            frame: "Frame",
        }),
        message!(TxFrame {
            header: "Header",
            kind: "u32",
            reserved: "u32",
            frame: "Frame",
        }),
        message!(Heartbeat {
            header: "Header",
            period_ms: "u32",
            reserved: "u32",
        }),
        message!(Status {
            header: "Header",
            bound_instance_id: "u64",
            tx_seq_seen: "u64",
            rx_frames: "u64",
            tx_forwarded: "u64",
            tx_refused_latched: "u64",
            tx_refused_rate: "u64",
            tx_refused_invalid: "u64",
            tx_refused_unbound: "u64",
            tx_refused_stale: "u64",
            tx_lost: "u64",
            tx_failed: "u64",
            flags: "u32",
            channel_flags: "u32",
            max_tx_hz: "u32",
            max_frame_len: "u32",
            period_ms: "u32",
            reserved: "u32",
            channel_error: format!("u8[{ERROR_LEN}]"),
        }),
    ]
}

/// Every bit and code constant of [`wire`], by name.
#[must_use]
pub fn codes() -> Value {
    let codes: [(&str, u32); 9] = [
        ("FRAME_EXTENDED_ID", wire::FRAME_EXTENDED_ID),
        ("TX_MOTION", wire::TX_MOTION),
        ("TX_READ", wire::TX_READ),
        ("TX_STOP", wire::TX_STOP),
        ("STATUS_CHANNEL_UP", wire::STATUS_CHANNEL_UP),
        ("STATUS_LATCHED", wire::STATUS_LATCHED),
        ("STATUS_ARM_FAULTED", wire::STATUS_ARM_FAULTED),
        (
            "CHANNEL_SUPPRESSES_REPEATS",
            wire::CHANNEL_SUPPRESSES_REPEATS,
        ),
        ("CHANNEL_EXTENDED_IDS", wire::CHANNEL_EXTENDED_IDS),
    ];
    Value::Object(
        codes
            .into_iter()
            .map(|(name, value)| (name.to_owned(), Value::from(value)))
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
            "default_base": default_base("<side>"),
        },
        "limits": {
            "MAX_FRAME_LEN": MAX_FRAME_LEN,
            "STANDARD_ID_MAX": STANDARD_ID_MAX,
            "EXTENDED_ID_MAX": EXTENDED_ID_MAX,
            "ERROR_LEN": ERROR_LEN,
            "TX_WINDOW": TX_WINDOW,
            "DEFAULT_MAX_TX_HZ": DEFAULT_MAX_TX_HZ,
            "MAX_TX_HZ": MAX_TX_HZ,
            "MAX_BASE_LEN": crate::names::MAX_BASE_LEN,
        },
        "timing_ms": {
            "MAX_HEARTBEAT_PERIOD_MS": MAX_HEARTBEAT_PERIOD_MS,
            "STATUS_PERIOD_MS": STATUS_PERIOD_MS,
            "TX_MAX_AGE_MS": TX_MAX_AGE_MS,
            "DEFAULT_STALE_AFTER_MS": crate::ports::DEFAULT_STALE_AFTER_MS,
        },
        "codes": codes(),
        "messages": messages(),
        "services": [
            topic(RX, "RxFrame", "daemon", "provider", RX_TOPIC),
            topic(TX, "TxFrame", "provider", "daemon", TX_TOPIC),
            topic(STATUS, "Status", "daemon", "provider", STATUS_TOPIC),
            topic(HEARTBEAT, "Heartbeat", "provider", "daemon", HEARTBEAT_TOPIC),
        ],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec_path(file: &str) -> std::path::PathBuf {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../spec/flange_bus.shm")
            .join(file)
    }

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

    #[test]
    fn the_contract_text_mirrors_the_layout() {
        let text = std::fs::read_to_string(spec_path("1.md")).expect("read the contract text");
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

    #[test]
    fn the_committed_layout_is_the_pinned_version_1() {
        use sha2::{Digest, Sha256};
        let bytes = std::fs::read(spec_path("1.layout.json")).expect("read the committed layout");
        let hex: String = Sha256::digest(&bytes)
            .iter()
            .map(|byte| format!("{byte:02x}"))
            .collect();
        assert_eq!(
            hex, LAYOUT_SHA256,
            "{COMMITTED_PATH} changed: version 1 is frozen; a layout change is version 2"
        );
    }

    /// Regenerate the committed layout with `D1FW_FLANGE_WRITE_LAYOUT=1
    /// cargo test -p d1fw-ee-shm-contract flange`.
    #[test]
    fn layout_matches_the_committed_spec() {
        let path = spec_path("1.layout.json");
        let generated = format!(
            "{}\n",
            serde_json::to_string_pretty(&document()).expect("serialize the layout")
        );
        if std::env::var_os("D1FW_FLANGE_WRITE_LAYOUT").is_some() {
            std::fs::write(&path, &generated).expect("write the layout");
        }
        let committed = std::fs::read_to_string(&path).expect("read the committed layout");
        assert!(
            committed == generated,
            "{COMMITTED_PATH} differs from the definitions; regenerate it with \
             D1FW_FLANGE_WRITE_LAYOUT=1 cargo test -p d1fw-ee-shm-contract flange, and treat \
             any difference as a contract change"
        );
    }
}
