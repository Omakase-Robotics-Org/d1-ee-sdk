//! The manifest `omakase-ee-provider/1`: the committed schema, the golden
//! manifest of d1-ee-ag 0.1.1, the template's manifest, every rule refused
//! on a mutation of the golden, and the check against a descriptor.

use d1fw_ee_manifest::{DescriptorView, Link, Plane, ProviderManifest, SCHEMA_JSON, schema};
use serde_json::{Value, json};

const AG_MANIFEST: &str = include_str!("fixtures/d1-ee-ag.manifest.json");
const AG_DESCRIPTOR: &str = include_str!("fixtures/d1-ee-ag.descriptor.json");
const TEMPLATE_MANIFEST: &str = include_str!("../../../template/manifest.json");

fn golden() -> Value {
    serde_json::from_str(AG_MANIFEST).expect("the golden manifest is JSON")
}

fn ag_descriptor() -> DescriptorView {
    DescriptorView::from_json(&serde_json::from_str(AG_DESCRIPTOR).expect("JSON"))
        .expect("a descriptor")
}

/// The problems of the golden with `edit` applied, as `at: message` lines.
fn refused(edit: impl FnOnce(&mut Value)) -> Vec<String> {
    let mut value = golden();
    edit(&mut value);
    match ProviderManifest::from_value(&value) {
        Ok(_) => panic!("the edited manifest was accepted: {value}"),
        Err(invalid) => invalid.0.iter().map(ToString::to_string).collect(),
    }
}

fn assert_refused_at(edit: impl FnOnce(&mut Value), at: &str) {
    let problems = refused(edit);
    assert!(
        problems.iter().any(|problem| problem.starts_with(at)),
        "no problem at {at}: {problems:#?}"
    );
}

#[test]
fn the_committed_schema_is_a_draft_2020_12_schema() {
    let schema = schema();
    jsonschema::draft202012::meta::validate(&schema).expect("valid against its meta-schema");
    jsonschema::draft202012::new(&schema).expect("compiles");
    assert_eq!(schema["title"], "omakase-ee-provider/1");
    assert!(SCHEMA_JSON.ends_with('\n'));
}

#[test]
fn the_golden_manifest_of_d1_ee_ag_is_valid() {
    let manifest = ProviderManifest::parse(AG_MANIFEST).expect("valid");
    assert_eq!(manifest.id, "omakase.ag");
    assert_eq!(manifest.vendor(), "omakase");
    assert_eq!(manifest.version, "0.1.1");
    assert_eq!(manifest.package.name, "d1-ee-ag");
    assert_eq!(
        manifest.command_for("a").as_deref(),
        Ok("/opt/d1-ee-ag/bin/d1-ee-ag-provider --side a"),
        "the command d1-ee-ag's own .env example names"
    );
    assert_eq!(
        manifest.config_path_for("b").as_deref(),
        Ok("/etc/d1-ee-ag/b.toml")
    );
    assert!(manifest.command_for("c").is_err());
    let model = manifest.model("ag/ag-100-100").expect("the model");
    assert_eq!(model.driver, "ag-100-100");
    assert!(model.links.contains(&Link::Flange));
    assert!(model.links.contains(&Link::UsbSerial));
    assert_eq!(model.postures["open"], vec![1.0]);
    assert!(
        manifest
            .sdk_range()
            .expect("a range")
            .matches(&semver_version("0.1.0"))
    );
    manifest.validate().expect("valid when built in code too");
    assert_eq!(
        ProviderManifest::parse(&manifest.to_json_pretty()).expect("round trip"),
        manifest
    );
}

fn semver_version(text: &str) -> semver::Version {
    semver::Version::parse(text).expect("a version")
}

#[test]
fn the_golden_config_schema_takes_the_example_configuration_and_refuses_a_band() {
    let manifest = ProviderManifest::parse(AG_MANIFEST).expect("valid");
    let validator = jsonschema::draft202012::new(&Value::Object(manifest.config_schema))
        .expect("the config schema compiles");
    // d1-ee-ag's deploy/d1-ee-ag.toml.example, read as JSON.
    let example = json!({
        "model": "ag/ag-100-100",
        "revision": "standard-fingertip",
        "bus": {"kind": "flange", "device_id": 1, "reply_timeout_ms": 300, "fd_min_len": 0},
        "stroke": {"closed_permille": 0, "open_permille": 1000},
        "motion": {"speed_percent": 50, "home_on_first_command": true},
        "grip": {"soft": {"force_percent": 20}, "firm": {"force_percent": 50},
                 "strong": {"force_percent": 100}}
    });
    assert!(validator.is_valid(&example));
    assert!(
        validator.is_valid(&json!({})),
        "a missing file is every default"
    );
    assert!(!validator.is_valid(&json!({"motion": {"speed_percent": 3}})));
    assert!(!validator.is_valid(&json!({"bus": {"kind": "usb"}})));
    assert!(!validator.is_valid(&json!({"colour": "red"})));
}

#[test]
fn the_templates_manifest_is_valid() {
    let manifest = ProviderManifest::parse(TEMPLATE_MANIFEST).expect("valid");
    assert_eq!(manifest.id, "acme.g1");
    assert_eq!(manifest.package.plane, Plane::Apt);
    assert_eq!(manifest.conformance.run_url, None);
}

#[test]
fn a_manifest_is_complete_offline() {
    // A local conformance run: no run reference, a log path, or a digest.
    for run in [
        None,
        Some(json!("/home/dev/d1-ee-acme/target/conformance.log")),
        Some(json!("d96c509245d5ba387ddb47a80c1ed59bfa229818")),
    ] {
        let mut value = golden();
        match run {
            None => {
                value["conformance"]
                    .as_object_mut()
                    .expect("conformance")
                    .remove("run_url");
            }
            Some(run) => value["conformance"]["run_url"] = run,
        }
        ProviderManifest::from_value(&value).expect("valid without a hub or a CI service");
    }
}

#[test]
fn the_document_type_and_unknown_fields_are_refused() {
    assert_refused_at(|m| m["schema"] = json!("omakase-ee-provider/2"), "/schema");
    assert_refused_at(|m| m["notes"] = json!("x"), "/:");
    assert_refused_at(|m| m["package"]["url"] = json!("x"), "/package");
    assert_refused_at(
        |m| {
            m.as_object_mut().expect("an object").remove("guard_models");
        },
        "/:",
    );
}

#[test]
fn the_id_is_a_lowercase_vendor_and_name() {
    assert_refused_at(|m| m["id"] = json!("Omakase.AG"), "/id");
    assert_refused_at(|m| m["id"] = json!("omakase-ag"), "/id");
    assert_refused_at(|m| m["id"] = json!("omakase.ag.v2"), "/id");
    assert_refused_at(|m| m["id"] = json!("omakase.ag_100"), "/id");
}

#[test]
fn the_version_is_a_plain_release() {
    assert_refused_at(|m| m["version"] = json!("0.1.1-rc.1"), "/version");
    assert_refused_at(|m| m["version"] = json!("0.1"), "/version");
    assert_refused_at(|m| m["version"] = json!("v0.1.1"), "/version");
}

#[test]
fn the_package_fields_are_typed() {
    assert_refused_at(
        |m| m["package"]["plane"] = json!("manifest"),
        "/package/plane",
    );
    assert_refused_at(|m| m["package"]["arch"] = json!("aarch64"), "/package/arch");
    assert_refused_at(
        |m| m["package"]["name"] = json!("D1-EE-AG"),
        "/package/name",
    );
}

fn registry_package(m: &mut Value) {
    m["package"] = json!({
        "name": "d1-ee-ag",
        "arch": "arm64",
        "plane": "registry",
        "key": "apt-deb/d1-ee-ag/0.1.1/arm64/d1-ee-ag_0.1.1_arm64.deb",
        "sha256": "a".repeat(64),
        "sizeBytes": 1_234_567
    });
}

#[test]
fn a_registry_package_carries_its_entrys_key_digest_and_size() {
    let mut value = golden();
    registry_package(&mut value);
    let manifest = ProviderManifest::from_value(&value).expect("a registry package");
    assert_eq!(manifest.package.plane, Plane::Registry);
    assert_eq!(manifest.package.size_bytes, Some(1_234_567));
    manifest.validate().expect("valid when built in code too");

    for field in ["key", "sha256", "sizeBytes"] {
        assert_refused_at(
            |m| {
                registry_package(m);
                m["package"].as_object_mut().expect("package").remove(field);
            },
            "/package",
        );
    }
    assert_refused_at(
        |m| {
            registry_package(m);
            m["package"]["sha256"] = json!("A".repeat(64));
        },
        "/package/sha256",
    );
    assert_refused_at(
        |m| {
            registry_package(m);
            m["package"]["sizeBytes"] = json!(0);
        },
        "/package/sizeBytes",
    );
    assert_refused_at(
        |m| {
            registry_package(m);
            m["package"]["key"] = json!("/apt-deb/d1-ee-ag/0.1.1/arm64/d1-ee-ag_0.1.1_arm64.deb");
        },
        "/package/key",
    );
    assert_refused_at(
        |m| {
            registry_package(m);
            m["package"]["key"] = json!("apt-deb/d1-ee-ag/0.1.1/arm64/d1-ee-ag_0.1.1_arm64.tar.gz");
        },
        "/package/key",
    );
    // A key that names another release's file.
    assert_refused_at(
        |m| {
            registry_package(m);
            m["package"]["key"] = json!("apt-deb/d1-ee-ag/0.1.0/arm64/d1-ee-ag_0.1.0_arm64.deb");
        },
        "/package/key",
    );
}

#[test]
fn an_apt_package_carries_no_registry_fields() {
    for (field, value) in [
        (
            "key",
            json!("apt-deb/d1-ee-ag/0.1.1/arm64/d1-ee-ag_0.1.1_arm64.deb"),
        ),
        ("sha256", json!("a".repeat(64))),
        ("sizeBytes", json!(1)),
    ] {
        assert_refused_at(|m| m["package"][field] = value, "/package");
    }
}

#[test]
fn the_sdk_range_holds_the_conformance_harness() {
    assert_refused_at(
        |m| m["sdk"]["contract"] = json!("end_effector.shm/2"),
        "/sdk/contract",
    );
    assert_refused_at(
        |m| m["sdk"]["d1_ee_sdk"] = json!("~> 0.1"),
        "/sdk/d1_ee_sdk",
    );
    assert_refused_at(
        |m| m["conformance"]["version"] = json!("0.2.0"),
        "/conformance/version",
    );
    assert_refused_at(
        |m| m["conformance"]["harness"] = json!("my-harness"),
        "/conformance/harness",
    );
    assert_refused_at(
        |m| m["conformance"]["run_url"] = json!("http://ci.example/1"),
        "/conformance/run_url",
    );
}

#[test]
fn the_launch_command_is_a_side_template_under_the_packages_bin() {
    assert_refused_at(
        |m| m["launch"]["command"] = json!("/opt/d1-ee-ag/bin/d1-ee-ag-provider --side {arm}"),
        "/launch/command",
    );
    assert_refused_at(
        |m| m["launch"]["command"] = json!("/opt/d1-ee-ag/bin/d1-ee-ag-provider --side a"),
        "/launch/command",
    );
    assert_refused_at(
        |m| m["launch"]["command"] = json!("/opt/d1-ee-ag/bin/d1-ee-ag-provider --side {side"),
        "/launch/command",
    );
    assert_refused_at(
        |m| {
            m["launch"]["command"] =
                json!("/opt/d1-ee-ag/bin/d1-ee-ag-provider --side {side}; reboot")
        },
        "/launch/command",
    );
    assert_refused_at(
        |m| m["launch"]["command"] = json!("/opt/d1-ee-ag/bin/d1-ee-ag-provider  --side {side}"),
        "/launch/command",
    );
    assert_refused_at(
        |m| m["launch"]["command"] = json!("/usr/bin/d1-ee-ag-provider --side {side}"),
        "/launch/command",
    );
    assert_refused_at(
        |m| m["launch"]["command"] = json!("/opt/other/bin/d1-ee-ag-provider --side {side}"),
        "/launch/command",
    );
    assert_refused_at(
        |m| m["launch"]["config_file"] = json!("ag.toml"),
        "/launch/config_file",
    );
    assert_refused_at(
        |m| m["launch"]["config_file"] = json!("{side}/ag.toml"),
        "/launch/config_file",
    );
    assert_refused_at(
        |m| m["launch"]["config_dir"] = json!("/etc/d1-ee-ag/"),
        "/launch/config_dir",
    );
    assert_refused_at(
        |m| m["launch"]["config_dir"] = json!("/etc/../tmp"),
        "/launch/config_dir",
    );
    assert_refused_at(
        |m| m["launch"]["config_dir"] = json!("etc/d1-ee-ag"),
        "/launch/config_dir",
    );
}

#[test]
fn the_config_schema_is_a_usable_draft_2020_12_schema() {
    assert_refused_at(
        |m| m["config_schema"]["$schema"] = json!("http://json-schema.org/draft-07/schema#"),
        "/config_schema/$schema",
    );
    assert_refused_at(
        |m| m["config_schema"]["type"] = json!("nope"),
        "/config_schema",
    );
    assert_refused_at(
        |m| m["config_schema"]["$ref"] = json!("https://schemas.example/ag.json"),
        "/config_schema",
    );
    assert_refused_at(|m| m["config_schema"] = json!(true), "/config_schema");
}

#[test]
fn models_name_their_links_joints_and_postures() {
    assert_refused_at(|m| m["models"] = json!([]), "/models");
    assert_refused_at(
        |m| m["models"][0]["links"] = json!(["bluetooth"]),
        "/models/0/links/0",
    );
    assert_refused_at(|m| m["models"][0]["links"] = json!([]), "/models/0/links");
    assert_refused_at(
        |m| m["models"][0]["links"] = json!(["can", "can"]),
        "/models/0/links",
    );
    assert_refused_at(|m| m["models"][0]["joints"] = json!(33), "/models/0/joints");
    assert_refused_at(
        |m| m["models"][0]["postures"]["open"] = json!([1.0, 0.0]),
        "/models/0/postures/open",
    );
    assert_refused_at(
        |m| m["models"][0]["postures"]["open"] = json!([1.5]),
        "/models/0/postures/open/0",
    );
    assert_refused_at(
        |m| {
            m["models"][0]["postures"]
                .as_object_mut()
                .expect("postures")
                .remove("closed");
        },
        "/models/0/postures",
    );
    assert_refused_at(
        |m| m["models"][0]["postures"]["Pinch"] = json!([0.5]),
        "/models/0/postures",
    );
    assert_refused_at(
        |m| {
            m["models"][0]["joints"] = json!(0);
        },
        "/models/0/postures",
    );
    assert_refused_at(
        |m| {
            let model = m["models"][0].clone();
            m["models"].as_array_mut().expect("models").push(model);
        },
        "/models/1/model",
    );
    assert_refused_at(
        |m| m["models"][0]["model"] = json!("ag 100"),
        "/models/0/model",
    );
}

#[test]
fn a_model_with_no_joints_takes_no_postures() {
    let mut value = golden();
    value["models"][0]["joints"] = json!(0);
    value["models"][0]["postures"] = json!({});
    ProviderManifest::from_value(&value).expect("a model driven by openness only");
}

#[test]
fn guard_models_and_calibration_are_typed() {
    assert_refused_at(
        |m| m["guard_models"] = json!(["ag/ag-100-100", "ag/ag-100-100"]),
        "/guard_models",
    );
    assert_refused_at(
        |m| m["calibration"]["required"] = json!("no"),
        "/calibration/required",
    );
}

#[test]
fn every_problem_is_reported_at_once() {
    let problems = refused(|m| {
        m["sdk"]["d1_ee_sdk"] = json!(">=0.3.0, <0.4.0");
        m["models"][0]["postures"]["open"] = json!([1.0, 1.0]);
    });
    assert_eq!(problems.len(), 2, "{problems:#?}");
}

#[test]
fn the_golden_matches_the_providers_descriptor() {
    let manifest = ProviderManifest::parse(AG_MANIFEST).expect("valid");
    let matched = manifest
        .check_descriptor(&ag_descriptor())
        .expect("the descriptor agrees");
    assert_eq!(matched.model, "ag/ag-100-100");
    assert_eq!(matched.postures["open"], vec![1.0]);
    assert_eq!(matched.postures["closed"], vec![0.0]);
}

#[test]
fn a_posture_resolves_in_the_joints_own_range() {
    let manifest = ProviderManifest::parse(AG_MANIFEST).expect("valid");
    let mut descriptor = ag_descriptor();
    // A configured stroke band of 100..900 per mille.
    descriptor.joints[0].min = 0.1;
    descriptor.joints[0].max = 0.9;
    let matched = manifest.check_descriptor(&descriptor).expect("agrees");
    assert!((matched.postures["open"][0] - 0.9).abs() < 1e-12);
    assert!((matched.postures["closed"][0] - 0.1).abs() < 1e-12);
}

#[test]
fn a_descriptor_that_disagrees_is_refused() {
    let manifest = ProviderManifest::parse(AG_MANIFEST).expect("valid");
    let problems = |edit: &dyn Fn(&mut DescriptorView)| {
        let mut descriptor = ag_descriptor();
        edit(&mut descriptor);
        manifest
            .check_descriptor(&descriptor)
            .expect_err("refused")
            .to_string()
    };
    assert!(problems(&|d| d.model = "ag/ag-160-95".to_owned()).starts_with("/models:"));
    assert!(problems(&|d| d.driver = "ag".to_owned()).starts_with("/models/0/driver"));
    assert!(problems(&|d| d.joints.clear()).starts_with("/models/0/joints"));
    assert!(
        problems(&|d| d.guard_model = Some("d1/parallel_gripper".to_owned()))
            .starts_with("/guard_models")
    );
    assert!(problems(&|d| d.joints[0].max = 0.0).starts_with("descriptor"));
}

#[test]
fn a_descriptor_is_read_from_a_sides_answer_or_bare() {
    let answer: Value = serde_json::from_str(AG_DESCRIPTOR).expect("JSON");
    let bare = answer["descriptor"].clone();
    assert_eq!(
        DescriptorView::from_json(&bare).expect("bare"),
        ag_descriptor()
    );
    let none = json!({"side": "a", "fitted": true, "available": false, "descriptor": null});
    assert!(
        DescriptorView::from_json(&none)
            .expect_err("no descriptor")
            .contains("no accepted descriptor")
    );
    assert!(DescriptorView::from_json(&json!({"side": "a"})).is_err());
}
