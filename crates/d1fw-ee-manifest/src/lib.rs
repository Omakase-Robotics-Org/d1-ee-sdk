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
//! 2. the rules a JSON Schema cannot state: the conformance harness inside
//!    the `sdk.d1_ee_sdk` range, `{side}` as the only placeholder, the
//!    program under `/opt/<package>/bin/`, a registry key naming this
//!    release's `.deb`, one posture value per joint, distinct models, and
//!    `config_schema` itself a usable draft 2020-12 schema.
//!
//! d1-provisioning's on-robot reader (`/usr/lib/d1-host/end-effectors`, a
//! shell program) holds a manifest to the same rules. Its JSON tokenizer
//! takes less than JSON: [`reader_warnings`] names what it refuses, so that
//! a manifest valid here is not refused on the robot.
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

/// `launch.command`: an absolute program and its arguments, words of ASCII
/// letters, digits and `_./{}=:,@+-` separated by single spaces. The unit
/// word-splits the value without a shell parse; every quoting, expansion,
/// redirection, glob and comment character is outside the set. The schema's
/// pattern and d1-provisioning's reader use this expression verbatim.
pub const COMMAND_PATTERN: &str = "^/[A-Za-z0-9_./{}=:,@+-]+( [A-Za-z0-9_./{}=:,@+-]+)*$";

/// A version range (`sdk.d1_ee_sdk`, a fit's `provider.version`):
/// comparators joined by commas, each an optional operator (`>=`, `<=`, `>`,
/// `<`, `=`; none is `=`) and a plain `MAJOR.MINOR.PATCH`, spaces around the
/// operator and the commas. The schema's pattern and d1-provisioning's
/// reader use this expression verbatim.
pub const RANGE_PATTERN: &str = "^ *(>=|<=|>|<|=)? *(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*) *(, *(>=|<=|>|<|=)? *(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*) *)*$";

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
    /// The JSON Schema (draft 2020-12) of a side's configuration file;
    /// `None` for a provider that reads none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub config_schema: Option<Map<String, Value>>,
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
    /// The d1-ee-sdk releases, a comparator list ([`RANGE_PATTERN`]).
    pub d1_ee_sdk: String,
}

/// `launch`.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Launch {
    /// The `EE_PROVIDER_COMMAND` template.
    pub command: String,
    /// The directory of the per-side configuration file; `None` for a
    /// provider that reads none.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub config_dir: Option<String>,
    /// The per-side configuration file name template; `None` with
    /// `config_dir`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub config_file: Option<String>,
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

/// The schema's verdicts that a validator words as a raw pattern or
/// subschema, said in the manifest's terms.
fn plain_words(value: &Value) -> Vec<Problem> {
    let mut problems = Vec::new();
    if let Some(Err(error)) = value
        .pointer("/sdk/d1_ee_sdk")
        .and_then(Value::as_str)
        .map(version_range)
    {
        problems.push(Problem::new("/sdk/d1_ee_sdk", error));
    }
    let given: Vec<&str> = [
        ("/launch/config_dir", "launch.config_dir"),
        ("/launch/config_file", "launch.config_file"),
        ("/config_schema", "config_schema"),
    ]
    .into_iter()
    .filter(|(pointer, _)| value.pointer(pointer).is_some())
    .map(|(_, name)| name)
    .collect();
    if !given.is_empty() && given.len() < 3 {
        problems.push(Problem::new(
            "/launch",
            format!(
                "launch.config_dir, launch.config_file and config_schema are given together or \
                 not at all (a provider that reads no configuration file names none); given: {}",
                given.join(", ")
            ),
        ));
    }
    if given.is_empty() && value.pointer("/calibration/required") == Some(&Value::Bool(true)) {
        problems.push(Problem::new(
            "/calibration/required",
            "true, but the provider reads no configuration file to append a calibration to"
                .to_owned(),
        ));
    }
    problems
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
        let mut problems = schema_problems(value);
        if !problems.is_empty() {
            problems.extend(plain_words(value));
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
    /// As [`version_range`].
    pub fn sdk_range(&self) -> Result<VersionReq, String> {
        version_range(&self.sdk.d1_ee_sdk)
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

    /// The configuration file of `side`, or `None` for a provider that
    /// reads none.
    ///
    /// # Errors
    ///
    /// A side other than [`SIDES`].
    pub fn config_path_for(&self, side: &str) -> Result<Option<String>, String> {
        check_side(side)?;
        let (Some(dir), Some(file)) = (&self.launch.config_dir, &self.launch.config_file) else {
            return Ok(None);
        };
        let file = file.replace(SIDE_PLACEHOLDER, side);
        Ok(Some(format!("{dir}/{file}")))
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
            Err(error) => problem("/sdk/d1_ee_sdk", error),
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
        // The schema holds config_file to a name without a directory and
        // config_dir to a normal path under /etc; the group of the two and
        // config_schema is all-or-none.
        if let Some(file) = &self.launch.config_file
            && let Err(message) = template_with_side(file)
        {
            problem("/launch/config_file", message);
        }
        if let Some(config_schema) = &self.config_schema {
            for (at, message) in config_schema_problems(config_schema) {
                problem(&format!("/config_schema{at}"), message);
            }
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

/// Read a version range: comparators joined by commas, all of which a
/// version satisfies, each an operator (`>=`, `>`, `<=`, `<`, `=`, or none
/// for `=`) and a plain `MAJOR.MINOR.PATCH` ([`RANGE_PATTERN`]). Cargo's
/// other forms (`^`, `~`, wildcards, partial versions, pre-releases) are
/// refused, and a bare version is exact, as d1-provisioning reads it, not
/// Cargo's caret.
///
/// # Errors
///
/// What is wrong with the first comparator that is not one.
pub fn version_range(text: &str) -> Result<VersionReq, String> {
    let mut comparators = Vec::new();
    for part in text.split(',') {
        let part = part.trim_matches(' ');
        let (operator, rest) = [">=", "<=", ">", "<", "="]
            .iter()
            .find_map(|op| part.strip_prefix(op).map(|rest| (*op, rest)))
            .unwrap_or(("=", part));
        let operand = rest.trim_start_matches(' ');
        let plain = !operand.is_empty()
            && operand
                .bytes()
                .all(|byte| byte.is_ascii_digit() || byte == b'.')
            && operand.split('.').count() == 3;
        let version = Version::parse(operand)
            .ok()
            .filter(|_| plain)
            .ok_or_else(|| {
                format!(
                    "{text:?} is not a version range: {part:?} is not a comparator (an operator \
                 >=, >, <=, < or = and a plain MAJOR.MINOR.PATCH; no ^, ~, wildcard, partial \
                 version or pre-release)"
                )
            })?;
        comparators.push(format!("{operator}{version}"));
    }
    VersionReq::parse(&comparators.join(", "))
        .map_err(|error| format!("{text:?} is not a version range: {error}"))
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

/// What d1-provisioning's on-robot reader refuses in a manifest that is
/// valid JSON, as warnings at JSON pointers: its tokenizer (a shell program,
/// no JSON library on the robot) takes no `\u`, `\b` or `\f` string escape,
/// no object key that is empty or holds a `.` (its keys are dotted paths),
/// and no key given twice in one object (which JSON leaves undefined and
/// this crate reads as the last). A manifest with a warning passes
/// [`ProviderManifest::parse`] and is refused by that reader.
///
/// `text` that is not JSON yields no warning: [`ProviderManifest::parse`]
/// refuses it.
#[must_use]
pub fn reader_warnings(text: &str) -> Vec<Problem> {
    if serde_json::from_str::<Value>(text).is_err() {
        return Vec::new();
    }
    let mut scan = Scan {
        bytes: text.as_bytes(),
        at: 0,
        warnings: Vec::new(),
    };
    scan.value("");
    scan.warnings
}

/// A walk over JSON text already known to parse.
struct Scan<'a> {
    bytes: &'a [u8],
    at: usize,
    warnings: Vec<Problem>,
}

impl Scan<'_> {
    fn peek(&self) -> u8 {
        self.bytes.get(self.at).copied().unwrap_or(0)
    }

    fn space(&mut self) {
        while matches!(self.peek(), b' ' | b'\t' | b'\n' | b'\r') {
            self.at += 1;
        }
    }

    fn warn(&mut self, at: &str, message: String) {
        self.warnings.push(Problem::new(at, message));
    }

    fn value(&mut self, at: &str) {
        self.space();
        match self.peek() {
            b'{' => self.object(at),
            b'[' => self.array(at),
            b'"' => {
                self.string(at);
            }
            _ => {
                while !matches!(
                    self.peek(),
                    b',' | b'}' | b']' | b' ' | b'\t' | b'\n' | b'\r' | 0
                ) {
                    self.at += 1;
                }
            }
        }
    }

    /// A string from its opening quote; its text with escapes kept as
    /// written.
    fn string(&mut self, at: &str) -> String {
        self.at += 1;
        let start = self.at;
        let mut escapes = std::collections::BTreeSet::new();
        while self.peek() != b'"' {
            if self.peek() == b'\\' {
                let escape = self.bytes.get(self.at + 1).copied().unwrap_or(0);
                if matches!(escape, b'u' | b'b' | b'f') {
                    escapes.insert(escape as char);
                }
                self.at += 2;
            } else {
                self.at += 1;
            }
        }
        let raw = String::from_utf8_lossy(&self.bytes[start..self.at]).into_owned();
        self.at += 1;
        for escape in escapes {
            self.warn(
                at,
                format!(
                    "the string escape \\{escape}: d1-provisioning's reader takes only \\\", \\\\, \
                     \\/, \\n, \\t and \\r (write the character itself)"
                ),
            );
        }
        raw
    }

    fn object(&mut self, at: &str) {
        self.at += 1;
        let mut keys = std::collections::HashSet::new();
        loop {
            self.space();
            if self.peek() == b'}' {
                self.at += 1;
                return;
            }
            let key = self.string(at);
            let child = format!("{at}/{}", key.replace('~', "~0").replace('/', "~1"));
            if key.is_empty() || key.contains('.') {
                self.warn(
                    &child,
                    format!(
                        "the key {key:?}: d1-provisioning's reader takes no key that is empty or \
                         holds a '.' (it reads keys as dotted paths)"
                    ),
                );
            }
            if !keys.insert(key.clone()) {
                self.warn(
                    &child,
                    format!("the key {key:?} is given twice; d1-provisioning's reader refuses it"),
                );
            }
            self.space();
            self.at += 1; // ':'
            self.value(&child);
            self.space();
            if self.peek() == b',' {
                self.at += 1;
            }
        }
    }

    fn array(&mut self, at: &str) {
        self.at += 1;
        let mut index = 0;
        loop {
            self.space();
            if self.peek() == b']' {
                self.at += 1;
                return;
            }
            self.value(&format!("{at}/{index}"));
            index += 1;
            self.space();
            if self.peek() == b',' {
                self.at += 1;
            }
        }
    }
}
