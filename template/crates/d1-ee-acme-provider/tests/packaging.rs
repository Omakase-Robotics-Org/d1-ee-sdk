//! The package and the manifest agree with the SDK's provider conventions:
//! the program's path, the `.env` the unit reads, and the manifest's launch
//! command.

use d1fw_ee_provider::unit::program_path;

const CARGO_TOML: &str = include_str!("../Cargo.toml");
const WORKSPACE_TOML: &str = include_str!("../../../Cargo.toml");
const ENV_EXAMPLE: &str = include_str!("../../../deploy/ee-provider.env.example");
const MANIFEST: &str = include_str!("../../../manifest.json");

#[test]
fn the_program_is_installed_where_the_manifest_runs_it() {
    let path = program_path("d1-ee-acme", "d1-ee-acme-provider");
    assert!(
        CARGO_TOML
            .contains(r#"["target/release/d1-ee-acme-provider", "opt/d1-ee-acme/bin/", "755"]"#),
        "the package installs the program under /opt/d1-ee-acme/bin/"
    );
    assert!(
        ENV_EXAMPLE
            .lines()
            .any(|line| line == format!("EE_PROVIDER_COMMAND={path} --side a")),
        "the .env example runs the installed program"
    );
    let manifest: serde_json::Value = serde_json::from_str(MANIFEST).expect("manifest.json");
    assert_eq!(
        manifest["launch"]["command"].as_str(),
        Some(format!("{path} --side {{side}}").as_str())
    );
    assert_eq!(manifest["package"]["name"].as_str(), Some("d1-ee-acme"));
}

#[test]
fn the_manifest_version_is_the_workspace_version() {
    let manifest: serde_json::Value = serde_json::from_str(MANIFEST).expect("manifest.json");
    let version = manifest["version"].as_str().expect("a version");
    assert!(
        WORKSPACE_TOML
            .lines()
            .any(|line| line == format!("version = \"{version}\"")),
        "manifest.json version {version} is the workspace version"
    );
}
