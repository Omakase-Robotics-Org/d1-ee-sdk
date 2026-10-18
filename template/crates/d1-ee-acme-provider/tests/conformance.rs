//! The provider process (`--fake`) against the SDK's host, the daemon's side
//! of `end_effector.shm/1`: the lifecycle every provider must follow
//! (announcement, every declared verb, the stop, SIGKILL and restart,
//! SIGTERM), and the accepted declaration as the manifest describes it.
//!
//! The accepted descriptor is also written to
//! `target/conformance/descriptor-a.json`, the file
//! `d1fw-ee-manifest check manifest.json --descriptor` reads in CI.

use std::{path::Path, process::Command};

use d1_ee_acme_provider::{DRIVER, GUARD_MODEL, MODEL};
use d1fw_ee_harness::{Host, HostSettings, conformance};
use d1fw_ee_provider::ShmRoot;

fn command(config: &Path) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_d1-ee-acme-provider"));
    command
        .args([
            "--side",
            "a",
            "--fake",
            "--state-period-ms",
            "50",
            "--config",
        ])
        .arg(config)
        .env("RUST_LOG", "warn");
    command
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_provider_process_conforms() {
    let root = ShmRoot::scratch();
    let dir = tempfile::tempdir().expect("a config directory");
    // No file: the built-in configuration.
    let config = dir.path().join("a.toml");
    let mut settings = HostSettings::new("a", root.clone());
    settings.expect_model = Some(MODEL.to_owned());
    let host = Host::start(settings);

    let lifecycle = conformance::lifecycle(&host, || command(&config)).await;
    let descriptor = lifecycle.descriptor;
    assert_eq!(descriptor.driver, DRIVER);
    assert_eq!(descriptor.model, MODEL);
    assert_eq!(descriptor.guard_model.as_deref(), Some(GUARD_MODEL));
    assert_eq!(lifecycle.verbs, vec!["set", "joints", "stop"]);

    let out = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/conformance");
    std::fs::create_dir_all(&out).expect("create target/conformance");
    std::fs::write(
        out.join("descriptor-a.json"),
        serde_json::to_string_pretty(&serde_json::json!({ "descriptor": descriptor }))
            .expect("a descriptor serializes"),
    )
    .expect("write the accepted descriptor");
}
