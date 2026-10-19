//! `d1fw-ee-manifest check`: exit 0 for a valid manifest (and a descriptor
//! that agrees), 1 for an invalid one or a disagreeing descriptor, 2 for a
//! file it cannot read.

use std::{path::Path, process::Command};

fn fixture(name: &str) -> String {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
        .display()
        .to_string()
}

fn run(args: &[&str]) -> (i32, String, String) {
    let output = Command::new(env!("CARGO_BIN_EXE_d1fw-ee-manifest"))
        .args(args)
        .output()
        .expect("run d1fw-ee-manifest");
    (
        output.status.code().expect("an exit code"),
        String::from_utf8_lossy(&output.stdout).into_owned(),
        String::from_utf8_lossy(&output.stderr).into_owned(),
    )
}

#[test]
fn a_valid_manifest_and_its_descriptor_pass() {
    let manifest = fixture("d1-ee-dh-ag100100.manifest.json");
    let descriptor = fixture("d1-ee-dh-ag100100.descriptor.json");
    let (code, out, err) = run(&["check", &manifest]);
    assert_eq!(code, 0, "{err}");
    assert!(
        out.contains("valid d1-firmware.end-effector.dh.ag100100 0.2.0"),
        "{out}"
    );
    let (code, out, err) = run(&["check", &manifest, "--descriptor", &descriptor]);
    assert_eq!(code, 0, "{err}");
    assert!(out.contains("posture open: [1.0]"), "{out}");
}

#[test]
fn an_invalid_manifest_exits_1_naming_each_problem() {
    let dir = tempfile::tempdir().expect("a directory");
    let path = dir.path().join("manifest.json");
    let mut value: serde_json::Value =
        serde_json::from_str(include_str!("fixtures/d1-ee-dh-ag100100.manifest.json"))
            .expect("JSON");
    value["models"][0]["postures"]["open"] = serde_json::json!([1.0, 1.0]);
    std::fs::write(&path, value.to_string()).expect("write");
    let (code, _, err) = run(&["check", &path.display().to_string()]);
    assert_eq!(code, 1);
    assert!(err.contains("/models/0/postures/open"), "{err}");
}

#[test]
fn a_disagreeing_descriptor_exits_1() {
    let dir = tempfile::tempdir().expect("a directory");
    let path = dir.path().join("descriptor.json");
    std::fs::write(
        &path,
        r#"{"descriptor": {"driver": "acme-g1", "model": "acme/g1", "joints": []}}"#,
    )
    .expect("write");
    let (code, _, err) = run(&[
        "check",
        &fixture("d1-ee-dh-ag100100.manifest.json"),
        "--descriptor",
        &path.display().to_string(),
    ]);
    assert_eq!(code, 1);
    assert!(err.contains("\"acme/g1\" is not listed"), "{err}");
}

#[test]
fn an_unreadable_file_exits_2() {
    let (code, _, _) = run(&["check", "/nonexistent/manifest.json"]);
    assert_eq!(code, 2);
}

#[test]
fn the_schema_command_prints_the_committed_schema() {
    let (code, out, _) = run(&["schema"]);
    assert_eq!(code, 0);
    assert_eq!(out, d1fw_ee_manifest::SCHEMA_JSON);
}

#[test]
fn a_provider_without_a_configuration_file_is_valid() {
    let (code, out, err) = run(&[
        "check",
        &fixture("published/d1-firmware.end-effector.leadshine.dh116-0.2.0.json"),
    ]);
    assert_eq!(code, 0, "{err}");
    assert!(
        out.contains("config none (the provider reads no file)"),
        "{out}"
    );
}

#[test]
fn what_the_on_robot_reader_refuses_is_a_warning_with_exit_0() {
    let dir = tempfile::tempdir().expect("a directory");
    let path = dir.path().join("manifest.json");
    let text = include_str!("fixtures/d1-ee-dh-ag100100.manifest.json")
        .replace("side configuration", "side configur\\u0061tion");
    std::fs::write(&path, text).expect("write");
    let (code, out, err) = run(&["check", &path.display().to_string()]);
    assert_eq!(code, 0, "{err}");
    assert!(
        out.contains("valid d1-firmware.end-effector.dh.ag100100 0.2.0"),
        "{out}"
    );
    assert!(
        err.contains("warning: /config_schema/title: the string escape \\u"),
        "{err}"
    );
}
