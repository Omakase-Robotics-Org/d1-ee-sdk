//! The provider manifest `omakase-ee-provider/1`: what one release of an
//! end-effector provider is, in a document a robot's fit references by
//! `{id, version}`. It is complete offline: a developer applies it from a
//! local file, and the artifacts hub serves the same document for the
//! providers it distributes.
//!
//! A manifest is checked in two passes, both by [`ProviderManifest::parse`]:
//!
//! 1. the JSON Schema committed as `schema/omakase-ee-provider-1.schema.json`
//!    ([`SCHEMA_JSON`]), the shape every reader can check with any validator;
//! 2. the rules a JSON Schema cannot state: the SemVer range of
//!    `sdk.d1_ee_sdk` and the conformance harness inside it, `{side}` as the
//!    only placeholder, the program under `/opt/<package>/bin/`, a registry
//!    key naming this release's `.deb`, one posture
//!    value per joint, distinct models, and `config_schema` itself a usable
//!    draft 2020-12 schema.
//!
//! [`ProviderManifest::check_descriptor`] holds a manifest to what a
//! provider actually announced (`GET /v1/end_effectors/{side}`): the model
//! is listed, its driver and joint count agree, its guard envelope id is
//! listed, and each posture resolves to joint targets inside the declared
//! limits.

use std::{collections::BTreeMap, fmt};

use semver::{Version, VersionReq};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

/// The document type and major version a manifest carries in `schema`.
pub const SCHEMA_ID: &str = "omakase-ee-provider/1";

/// The JSON Schema of [`SCHEMA_ID`], as committed in this repository's
/// `schema/`.
pub const SCHEMA_JSON: &str = include_str!("../../../schema/omakase-ee-provider-1.schema.json");

/// The contract every provider serves, in `sdk.contract`.
pub const CONTRACT: &str = "end_effector.shm/1";

/// The conformance harness, in `conformance.harness`.
pub const HARNESS: &str = "d1fw-ee-harness";

/// The arm sides a `{side}` placeholder takes: `a` is the physical left arm,
/// `b` the right.
pub const SIDES: [&str; 2] = ["a", "b"];

/// The only placeholder of `launch.command` and `launch.config_file`.
pub const SIDE_PLACEHOLDER: &str = "{side}";

/// The meta-schema a `config_schema` is written in.
pub const CONFIG_SCHEMA_DRAFT: &str = "https://json-schema.org/draft/2020-12/schema";

/// One release of a provider.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProviderManifest {
    /// [`SCHEMA_ID`].
    pub schema: String,
    /// `<vendor>.<name>`: the name a fit references, and the registry entry
    /// name of a provider the hub distributes.
    pub id: String,
    /// The release; equal to the Debian package's version.
    pub version: String,
    /// The package the release installs from.
    pub package: Package,
    /// What the provider was built against.
    pub sdk: Sdk,
    /// How a side's unit runs it.
    pub launch: Launch,
    /// The JSON Schema (draft 2020-12) of a side's configuration file.
    pub config_schema: Map<String, Value>,
    /// Every model the release drives.
    pub models: Vec<Model>,
    /// Every guard envelope id a descriptor of the release declares.
    pub guard_models: Vec<String>,
    /// Whether a per-robot calibration is needed.
    pub calibration: Calibration,
    /// The conformance run the release passed.
    pub conformance: Conformance,
}

/// `package`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Package {
    /// The Debian package name.
    pub name: String,
    /// The Debian architecture.
    pub arch: String,
    /// Where the package comes from.
    pub plane: Plane,
    /// [`Plane::Registry`] only: the .deb's key relative to the hub's
    /// `baseUrl`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub key: Option<String>,
    /// [`Plane::Registry`] only: the .deb's SHA-256, lowercase hex.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sha256: Option<String>,
    /// [`Plane::Registry`] only: the .deb's size, bytes.
    #[serde(default, rename = "sizeBytes", skip_serializing_if = "Option::is_none")]
    pub size_bytes: Option<u64>,
}

impl Package {
    /// The file name Debian's pool spelling gives this package's .deb.
    #[must_use]
    pub fn deb_file_name(&self, version: &str) -> String {
        format!("{}_{version}_{}.deb", self.name, self.arch)
    }
}

/// Where a provider's package comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Plane {
    /// Installed by name and version from an apt source (the hub's apt
    /// suites, a vendor's own source) or already installed with dpkg; the
    /// manifest names no bytes and needs no hub.
    Apt,
    /// A content-addressed artifact on the hub's manifest plane, beside the
    /// manifest: downloaded by `key`, checked against `sha256` and
    /// `sizeBytes`, installed with dpkg.
    Registry,
}

/// `sdk`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sdk {
    /// [`CONTRACT`].
    pub contract: String,
    /// A Cargo-syntax SemVer range of d1-ee-sdk releases.
    pub d1_ee_sdk: String,
}

/// `launch`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Launch {
    /// The `EE_PROVIDER_COMMAND` template.
    pub command: String,
    /// The directory of the per-side configuration file.
    pub config_dir: String,
    /// The per-side configuration file name template.
    pub config_file: String,
}

/// One model of `models`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Model {
    /// The descriptor's `model`.
    pub model: String,
    /// The descriptor's `driver`.
    pub driver: String,
    /// How the device can be reached.
    pub links: Vec<Link>,
    /// The descriptor's joint count.
    pub joints: usize,
    /// Named postures: per joint, a fraction of its range (0 `min`, 1 `max`).
    pub postures: BTreeMap<String, Vec<f64>>,
}

/// How a device is reached.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Link {
    /// The arm's flange CAN channel, relayed by the daemon
    /// (`flange_bus.shm/1`).
    Flange,
    /// Classic CAN on an interface of the robot's computer.
    Can,
    /// CAN FD on an interface of the robot's computer.
    Canfd,
    /// A USB serial adapter (RS485, RS232, TTL) on the robot's computer.
    UsbSerial,
    /// Ethernet.
    Ethernet,
}

/// `calibration`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Calibration {
    /// The fit supplies a calibration file, appended to the side's
    /// configuration file.
    pub required: bool,
}

/// `conformance`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Conformance {
    /// [`HARNESS`].
    pub harness: String,
    /// The harness release the run used.
    pub version: String,
    /// Where the run is recorded: an https URL of a CI run, an absolute path
    /// of a local run log, or a hex digest (the commit, or the log's
    /// SHA-256).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub run_url: Option<String>,
}

/// One rule a manifest (or a manifest against a descriptor) breaks.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Problem {
    /// Where: a JSON pointer into the manifest (`/models/0/postures/open`),
    /// or `descriptor` for the descriptor's side of a comparison.
    pub at: String,
    /// What is wrong.
    pub message: String,
}

impl Problem {
    fn new(at: impl Into<String>, message: impl Into<String>) -> Self {
        Self {
            at: at.into(),
            message: message.into(),
        }
    }
}

impl fmt::Display for Problem {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let at = if self.at.is_empty() { "/" } else { &self.at };
        write!(formatter, "{at}: {}", self.message)
    }
}

/// Every rule broken, in the order found.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub struct Invalid(pub Vec<Problem>);

impl fmt::Display for Invalid {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (index, problem) in self.0.iter().enumerate() {
            if index > 0 {
                formatter.write_str("\n")?;
            }
            write!(formatter, "{problem}")?;
        }
        Ok(())
    }
}

impl Invalid {
    fn one(at: impl Into<String>, message: impl Into<String>) -> Self {
        Self(vec![Problem::new(at, message)])
    }
}

/// The committed JSON Schema, parsed.
///
/// # Panics
///
/// Never for the committed file: a test holds it to be JSON and a usable
/// draft 2020-12 schema.
#[must_use]
pub fn schema() -> Value {
    serde_json::from_str(SCHEMA_JSON).expect("the committed manifest schema is JSON")
}

/// Problems of `value` against the committed JSON Schema.
fn schema_problems(value: &Value) -> Vec<Problem> {
    let validator =
        jsonschema::draft202012::new(&schema()).expect("the committed manifest schema compiles");
    validator
        .iter_errors(value)
        .map(|error| Problem::new(error.instance_path().to_string(), error.to_string()))
        .collect()
}

impl ProviderManifest {
    /// Read and check a manifest's JSON text.
    ///
    /// # Errors
    ///
    /// [`Invalid`] with every problem found: the JSON, the schema, then the
    /// rules (the rules run only on a document the schema accepts).
    pub fn parse(text: &str) -> Result<Self, Invalid> {
        let value: Value = serde_json::from_str(text)
            .map_err(|error| Invalid::one("", format!("not JSON: {error}")))?;
        Self::from_value(&value)
    }

    /// Check a manifest already read as JSON.
    ///
    /// # Errors
    ///
    /// As [`Self::parse`].
    pub fn from_value(value: &Value) -> Result<Self, Invalid> {
        let problems = schema_problems(value);
        if !problems.is_empty() {
            return Err(Invalid(problems));
        }
        let manifest: Self = serde_json::from_value(value.clone())
            .map_err(|error| Invalid::one("", format!("does not deserialize: {error}")))?;
        let problems = manifest.rule_problems();
        if problems.is_empty() {
            Ok(manifest)
        } else {
            Err(Invalid(problems))
        }
    }

    /// Check a manifest built in code, by the same two passes.
    ///
    /// # Errors
    ///
    /// As [`Self::parse`].
    pub fn validate(&self) -> Result<(), Invalid> {
        let value = serde_json::to_value(self)
            .map_err(|error| Invalid::one("", format!("does not serialize: {error}")))?;
        Self::from_value(&value).map(|_| ())
    }

    /// The manifest as pretty-printed JSON.
    ///
    /// # Panics
    ///
    /// Never: every field serializes.
    #[must_use]
    pub fn to_json_pretty(&self) -> String {
        serde_json::to_string_pretty(self).expect("a manifest serializes")
    }

    /// The vendor segment of `id`, before the dot.
    #[must_use]
    pub fn vendor(&self) -> &str {
        self.id
            .split_once('.')
            .map_or(self.id.as_str(), |(vendor, _)| vendor)
    }

    /// The `sdk.d1_ee_sdk` range.
    ///
    /// # Errors
    ///
    /// The parse error of a range that is not Cargo SemVer syntax.
    pub fn sdk_range(&self) -> Result<VersionReq, semver::Error> {
        VersionReq::parse(&self.sdk.d1_ee_sdk)
    }

    /// The `EE_PROVIDER_COMMAND` of `side`.
    ///
    /// # Errors
    ///
    /// A side other than [`SIDES`].
    pub fn command_for(&self, side: &str) -> Result<String, String> {
        check_side(side)?;
        Ok(self.launch.command.replace(SIDE_PLACEHOLDER, side))
    }

    /// The configuration file of `side`.
    ///
    /// # Errors
    ///
    /// A side other than [`SIDES`].
    pub fn config_path_for(&self, side: &str) -> Result<String, String> {
        check_side(side)?;
        let file = self.launch.config_file.replace(SIDE_PLACEHOLDER, side);
        let dir = self.launch.config_dir.trim_end_matches('/');
        Ok(format!("{dir}/{file}"))
    }

    /// The entry of `model`.
    #[must_use]
    pub fn model(&self, model: &str) -> Option<&Model> {
        self.models.iter().find(|entry| entry.model == model)
    }

    /// The rules the JSON Schema cannot state.
    fn rule_problems(&self) -> Vec<Problem> {
        let mut problems = Vec::new();
        let mut problem = |at: &str, message: String| problems.push(Problem::new(at, message));

        if let Err(error) = Version::parse(&self.version) {
            problem("/version", format!("not a SemVer version: {error}"));
        }
        // The key is version-pinned: one that names another release's file
        // would install bytes this manifest does not describe.
        if let Some(key) = &self.package.key {
            let file = key.rsplit('/').next().unwrap_or_default();
            let expected = self.package.deb_file_name(&self.version);
            if file != expected {
                problem(
                    "/package/key",
                    format!("the file {file:?} is not this release's .deb {expected:?}"),
                );
            }
        }
        match self.sdk_range() {
            Err(error) => problem(
                "/sdk/d1_ee_sdk",
                format!(
                    "{:?} is not a Cargo SemVer range: {error}",
                    self.sdk.d1_ee_sdk
                ),
            ),
            Ok(range) => match Version::parse(&self.conformance.version) {
                Err(error) => problem(
                    "/conformance/version",
                    format!("not a SemVer version: {error}"),
                ),
                Ok(harness) if !range.matches(&harness) => problem(
                    "/conformance/version",
                    format!(
                        "the conformance run used d1fw-ee-harness {harness}, outside \
                         sdk.d1_ee_sdk {range}"
                    ),
                ),
                Ok(_) => {}
            },
        }

        if let Err(message) = template_with_side(&self.launch.command) {
            problem("/launch/command", message);
        }
        let program = self.launch.command.split(' ').next().unwrap_or_default();
        let prefix = format!("/opt/{}/bin/", self.package.name);
        match program.strip_prefix(&prefix) {
            Some(name) if !name.is_empty() && !name.contains(['/', '{', '}']) => {}
            _ => problem(
                "/launch/command",
                format!(
                    "the program {program:?} is not a file directly under {prefix} (the \
                     package installs its program as /opt/<package>/bin/<program>)"
                ),
            ),
        }
        if let Err(message) = template_with_side(&self.launch.config_file) {
            problem("/launch/config_file", message);
        }
        if self.launch.config_file.contains('/') {
            problem(
                "/launch/config_file",
                "is a file name inside config_dir, without a directory".to_owned(),
            );
        }
        if self
            .launch
            .config_dir
            .split('/')
            .skip(1)
            .any(|segment| matches!(segment, "" | "." | ".."))
            && self.launch.config_dir != "/"
        {
            problem(
                "/launch/config_dir",
                format!(
                    "{:?} must be a normal absolute path: no empty, \".\" or \"..\" segment and \
                     no trailing slash",
                    self.launch.config_dir
                ),
            );
        }

        for (at, message) in config_schema_problems(&self.config_schema) {
            problem(&format!("/config_schema{at}"), message);
        }

        let mut seen = std::collections::HashSet::new();
        for (index, model) in self.models.iter().enumerate() {
            let at = format!("/models/{index}");
            if !seen.insert(model.model.as_str()) {
                problem(
                    &format!("{at}/model"),
                    format!("model {:?} is listed twice", model.model),
                );
            }
            if model.joints == 0 && !model.postures.is_empty() {
                problem(
                    &format!("{at}/postures"),
                    "a model with no joints names no posture".to_owned(),
                );
            }
            for (name, values) in &model.postures {
                if values.len() != model.joints {
                    problem(
                        &format!("{at}/postures/{name}"),
                        format!(
                            "{} values for {} joints (one per joint, in descriptor order)",
                            values.len(),
                            model.joints
                        ),
                    );
                }
            }
        }
        problems
    }

    /// Hold the manifest to a provider's announced descriptor.
    ///
    /// # Errors
    ///
    /// [`Invalid`] naming each disagreement.
    pub fn check_descriptor(
        &self,
        descriptor: &DescriptorView,
    ) -> Result<DescriptorMatch, Invalid> {
        let Some((index, model)) = self
            .models
            .iter()
            .enumerate()
            .find(|(_, entry)| entry.model == descriptor.model)
        else {
            let listed: Vec<&str> = self
                .models
                .iter()
                .map(|entry| entry.model.as_str())
                .collect();
            return Err(Invalid::one(
                "/models",
                format!(
                    "the descriptor's model {:?} is not listed (listed: {listed:?})",
                    descriptor.model
                ),
            ));
        };
        let at = format!("/models/{index}");
        let mut problems = Vec::new();
        if model.driver != descriptor.driver {
            problems.push(Problem::new(
                format!("{at}/driver"),
                format!(
                    "{:?}, the descriptor's driver is {:?}",
                    model.driver, descriptor.driver
                ),
            ));
        }
        if model.joints != descriptor.joints.len() {
            problems.push(Problem::new(
                format!("{at}/joints"),
                format!(
                    "{}, the descriptor declares {} joints",
                    model.joints,
                    descriptor.joints.len()
                ),
            ));
        }
        for joint in &descriptor.joints {
            if !joint.min.is_finite() || !joint.max.is_finite() || joint.min >= joint.max {
                problems.push(Problem::new(
                    "descriptor",
                    format!(
                        "joint {} limits [{}, {}] are not a range a posture can resolve in",
                        joint.name, joint.min, joint.max
                    ),
                ));
            }
        }
        if let Some(guard) = &descriptor.guard_model
            && !self.guard_models.contains(guard)
        {
            problems.push(Problem::new(
                "/guard_models",
                format!("the descriptor's guard_model {guard:?} is not listed"),
            ));
        }
        if !problems.is_empty() {
            return Err(Invalid(problems));
        }
        let postures = model
            .postures
            .iter()
            .map(|(name, fractions)| {
                let targets = descriptor
                    .joints
                    .iter()
                    .zip(fractions)
                    .map(|(joint, fraction)| joint.target(*fraction))
                    .collect();
                (name.clone(), targets)
            })
            .collect();
        Ok(DescriptorMatch {
            model: model.model.clone(),
            postures,
        })
    }
}

fn check_side(side: &str) -> Result<(), String> {
    if SIDES.contains(&side) {
        Ok(())
    } else {
        Err(format!("a side is \"a\" or \"b\", got {side:?}"))
    }
}

/// `text` holds `{side}` and no other placeholder or stray brace.
fn template_with_side(text: &str) -> Result<(), String> {
    let mut rest = text;
    let mut sides = 0;
    while let Some(open) = rest.find(['{', '}']) {
        if rest[open..].starts_with('}') {
            return Err(format!("{text:?} has a '}}' that closes no placeholder"));
        }
        let after = &rest[open + 1..];
        let Some(close) = after.find('}') else {
            return Err(format!("{text:?} has a '{{' that is never closed"));
        };
        let name = &after[..close];
        if name != "side" {
            return Err(format!(
                "{text:?} has the placeholder {{{name}}}; {SIDE_PLACEHOLDER} is the only one"
            ));
        }
        sides += 1;
        rest = &after[close + 1..];
    }
    if sides == 0 {
        return Err(format!(
            "{text:?} has no {SIDE_PLACEHOLDER}: each side runs its own instance"
        ));
    }
    Ok(())
}

/// Problems of a `config_schema`, as (JSON pointer below it, message).
fn config_schema_problems(config_schema: &Map<String, Value>) -> Vec<(String, String)> {
    if let Some(draft) = config_schema.get("$schema")
        && draft.as_str() != Some(CONFIG_SCHEMA_DRAFT)
    {
        return vec![(
            "/$schema".to_owned(),
            format!("{draft} is not {CONFIG_SCHEMA_DRAFT:?}"),
        )];
    }
    let value = Value::Object(config_schema.clone());
    if let Err(error) = jsonschema::draft202012::meta::validate(&value) {
        return vec![(
            error.instance_path().to_string(),
            format!("not a draft 2020-12 schema: {error}"),
        )];
    }
    // Compiling resolves every `$ref`; with no network or file resolver
    // built in, a reference outside the document fails here rather than on a
    // reader's machine.
    match jsonschema::draft202012::new(&value) {
        Ok(_) => Vec::new(),
        Err(error) => vec![(String::new(), format!("does not compile: {error}"))],
    }
}

/// The part of a provider's descriptor a manifest is held to, as
/// `GET /v1/end_effectors/{side}` serves it (the response's `descriptor`) or
/// as the SDK's `Descriptor` serializes. Every other field is ignored.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct DescriptorView {
    /// `model`.
    pub model: String,
    /// `driver`.
    pub driver: String,
    /// `joints`, in command order.
    #[serde(default)]
    pub joints: Vec<JointView>,
    /// `guard_model`.
    #[serde(default)]
    pub guard_model: Option<String>,
}

/// One descriptor joint.
#[derive(Debug, Clone, PartialEq, Deserialize)]
pub struct JointView {
    /// `name`.
    pub name: String,
    /// `unit`: `rad` or `fraction`.
    pub unit: String,
    /// `min`.
    pub min: f64,
    /// `max`.
    pub max: f64,
}

impl JointView {
    /// The target a posture fraction names: `min + fraction × (max − min)`.
    #[must_use]
    pub fn target(&self, fraction: f64) -> f64 {
        fraction.mul_add(self.max - self.min, self.min)
    }
}

impl DescriptorView {
    /// Read a descriptor from a side's REST answer (its `descriptor` field)
    /// or from a bare descriptor.
    ///
    /// # Errors
    ///
    /// A side with no accepted descriptor (`"descriptor": null`), or JSON
    /// that is neither form.
    pub fn from_json(value: &Value) -> Result<Self, String> {
        let descriptor = match value.get("descriptor") {
            Some(Value::Null) => {
                return Err("the side has no accepted descriptor (\"descriptor\": null)".to_owned());
            }
            Some(inner) => inner,
            None => value,
        };
        serde_json::from_value(descriptor.clone())
            .map_err(|error| format!("not a descriptor: {error}"))
    }
}

/// What a manifest resolved against a descriptor.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DescriptorMatch {
    /// The model entry the descriptor matched.
    pub model: String,
    /// Each posture as joint targets, in the joints' own units.
    pub postures: BTreeMap<String, Vec<f64>>,
}
