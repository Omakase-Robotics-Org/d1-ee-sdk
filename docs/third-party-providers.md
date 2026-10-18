# Third-party end-effector providers

This guide is the path from an end effector on the bench to that end
effector working on a D1, for a developer outside Omakase Robotics, for a
partner, and for in-house development alike. Every step runs on the
developer's machine and on the robot: no Omakase service, account or
approval is involved.

`d1-firmwared` names no end effector. A side is served by a **provider**, a
process of its own speaking `end_effector.shm/1` through this SDK
(`docs/end-effector-shm-interface.md`). What the robot needs to run one is
described by two documents:

- the **provider manifest** `omakase-ee-provider/1`
  (`schema/omakase-ee-provider-1.schema.json`): one release of a provider —
  its package, the contract and SDK range it was built for, the command a
  side's unit runs, the JSON Schema of its configuration file, the models it
  drives with their joints and named postures, its guard envelope ids,
  whether it needs a calibration, and its conformance run;
- the robot's **fit** `omakase-robot-fit/1` (defined by d1-provisioning):
  which provider, by `{id, version}`, is on which side, with the side's
  configuration and, when the manifest requires one, its calibration.

d1-provisioning applies the two: it writes the side's
`/etc/d1-firmwared/ee-provider/<side>.env` from `launch.command`, the side's
configuration file at `launch.config_dir`/`launch.config_file` (the fit's
calibration appended when `calibration.required` is true), and enables
`d1-firmwared-ee-provider@<side>`. A teleoperation client reads the model's
`postures` (`open`, `closed`, `pinch`, …) instead of carrying code for the
hand. Neither d1-provisioning nor the teleoperation stack changes for a new
provider.

## 1. Write the provider

Start from `template/` (its README has the copy-and-rename commands): a
crate implementing `d1fw_ee_provider::Provider` over a fake device
transport, with its conformance test, its Debian packaging, its CI and
release workflows, and its `manifest.json`. Replace the fake transport with
the device's bus and declare what the device is in the descriptor: joints
(name, unit `rad` or `fraction`, min, max), capabilities, command ceiling,
tool, guard envelope id. The rules a provider must follow are
`spec/end_effector.shm/1.md` (and `spec/flange_bus.shm/1.md` for a device on
the arm's flange CAN channel).

## 2. Pass conformance locally

```sh
cargo test --workspace --all-features
d1fw-ee-manifest check manifest.json --descriptor target/conformance/descriptor-a.json
```

The template's conformance test runs `d1fw_ee_harness::conformance::lifecycle`
against the provider binary — the declaration accepted, every declared verb
answered, the stop, SIGKILL and restart with the same declaration, SIGTERM
ending it cleanly — on any Linux machine, with no robot, and writes the
descriptor the harness accepted. `d1fw-ee-manifest check` then validates the
manifest (exit 0 valid, 1 invalid, 2 unreadable) and holds it to that
descriptor: the model is listed, its driver and joint count agree, its
guard envelope id is listed, and every posture resolves to joint targets
(printed in the joints' own units).

Install the command from this repository:

```sh
cargo install --locked --git https://github.com/Omakase-Robotics-Org/d1-ee-sdk --tag <tag> d1fw-ee-manifest
```

## 3. Package it

`cargo deb -p <crate>` builds `<package>_<version>_<arch>.deb`, which
installs the program as `/opt/<package>/bin/<program>` and its references
under `/opt/<package>/share/`, writes no configuration and enables nothing.
The robots are arm64 with glibc 2.35: build the robot's package on arm64
inside `ubuntu:22.04` (the template's `release.yml` does, on a `v*` tag).

## 4. Write the manifest

`manifest.json` beside the code, one per release. Its `version` is the
package's version; the template's packaging test holds it equal to the
workspace version.

| Field | Rule |
| --- | --- |
| `schema` | `omakase-ee-provider/1` |
| `id` | `<vendor>.<name>`, lowercase `[a-z0-9-]` in each segment: the name a fit references |
| `version` | `MAJOR.MINOR.PATCH`, equal to the package's version (no pre-release suffix, which Debian orders differently) |
| `package` | `{name, arch, plane}`; `arch` `arm64` or `amd64`; `plane` `apt` for a package installed with dpkg or from an apt source of your own |
| `sdk` | `contract` `end_effector.shm/1`; `d1_ee_sdk` a Cargo SemVer range such as `">=0.2.0, <0.3.0"` |
| `launch.command` | the `.env` command: `/opt/<package>/bin/<program>` and its arguments, single spaces, `{side}` the only placeholder and required, no quoting or shell syntax (the unit word-splits it) |
| `launch.config_dir`, `launch.config_file` | the side's configuration file: an absolute directory, and a file name with `{side}` (e.g. `/etc/d1-ee-acme` and `{side}.toml`) |
| `config_schema` | JSON Schema draft 2020-12 of the configuration file, its TOML read as JSON; self-contained (no `$ref` outside it) |
| `models[]` | `model` and `driver` as the descriptor announces them; `links` from `flange`, `can`, `canfd`, `usb-serial`, `ethernet`; `joints` the descriptor's joint count; `postures` named vectors, one value per joint in descriptor order, each a fraction of that joint's range (`0` its `min`, `1` its `max`), `open` and `closed` required when the model has joints |
| `guard_models` | every `guard_model` a descriptor of the release declares |
| `calibration.required` | `true` when the fit must supply a calibration file, appended to the side's configuration file |
| `conformance` | `harness` `d1fw-ee-harness`; `version` the harness release the run used, inside `sdk.d1_ee_sdk`; `run_url` optional: an https URL of a CI run, an absolute path of a local run log, or a hex digest (the commit, or the log's SHA-256) |

Nothing in a manifest needs a network to be read or checked.

## 5. Install and apply on the robot

```sh
sudo dpkg -i d1-ee-acme_0.1.0_arm64.deb          # or apt from your own source
d1-provisioning apply --ledger fit.json --provider-manifest acme.g1=manifest.json
```

`--ledger` is the robot's fit document, here a local file naming
`{"id": "acme.g1", "version": "0.1.0"}` on the side; `--provider-manifest
<id>=<file>` gives d1-provisioning the manifest for that `id` from a local
file. This is a supported path for development, partners and customers, not
an error state: the robot reports the side's provider as an **unregistered
provider** (applied from a local manifest rather than resolved from a
registry), and runs it like any other.

Check the side once it is applied:

```sh
curl -s http://<robot>/v1/end_effectors/a > descriptor.json
d1fw-ee-manifest check manifest.json --descriptor descriptor.json
```

`GET /v1/end_effectors/{side}` answers `available: true` and the accepted
descriptor once the provider has announced itself; the check accepts that
answer as it is.

A new release is the same loop: raise the version, pass conformance, build,
install, and apply the fit naming the new `{id, version}`.

## What is trusted, and what checks what

- **The provider is code the robot runs.** The unit runs it as the daemon's
  account, with access to the daemon's shared-memory root. On the local
  path, what vouches for the package and the manifest is the person who
  installed and applied them; the robot's "unregistered provider" fact keeps
  that visible.
- **Conformance** is evidence the developer produces: the harness's
  lifecycle on the released commit, referenced by `conformance`. The robot
  does not re-run it. What the daemon enforces at run time regardless is the
  contract itself: it accepts one valid declaration per provider instance,
  judges liveness by the heartbeat, validates and paces every command
  against the declaration, and raises the stop generation on a soft kill
  (`spec/end_effector.shm/1.md` §5).
- **The guard envelope** is not delivered by a provider. A descriptor's
  `guard_model` selects an envelope the daemon's arm guard already has; the
  guard knows `d1/parallel_gripper` only, checks every other id with that
  default envelope, and says so (`guard_model_applied`). An envelope for a
  new end effector is added to the daemon.

## Providers the artifacts hub distributes

Omakase Robotics distributes its own providers (d1-ee-damiao, d1-ee-ag, …)
through the artifacts hub (omakaseos-artifacts). That route is for those
providers only; nothing in sections 1 to 5 depends on it, and the manifest
is the same document either way.

- **The registry.** Each release's manifest is an entry selected by the hub
  registry node `d1/ee-providers`. A registry node is a path with a `match`
  list of rules `{type, name}` or `{type, namePrefix}`; a `public` node is
  projected to the signed document `_registry/<path>/index.json` with its
  detached signature at `_registry/sig/<sha256 of the bytes>.asc`
  (omakaseos-artifacts `PUBLISHING.md`, "The registry plane"). A robot's fit
  can then name `{id, version}` without a local manifest; robots' fits are
  themselves registry documents (`omakase-robot-fit/1`, node `d1/fits`).
- **The package's plane.** `package.plane` is `apt` for a package that is a
  member of the hub's signed apt suites (installed by apt, promoted between
  suites by the hub), or `registry` for a `.deb` published as a
  content-addressed artifact on the hub's manifest plane next to the
  manifest. A `registry` package carries its entry's `key` (relative to the
  hub's `baseUrl`, file name `<name>_<version>_<arch>.deb`), `sha256` and
  `sizeBytes`; a consumer downloads `baseUrl + "/" + key`, checks the digest
  and the size, and installs it with dpkg. `apt` carries none of the three.
- **Publishing** goes through the artifacts control plane's release ingest
  with GitHub OIDC (the `publish-release` action of omakase-artifacts-service,
  planes `apt` and `manifest`), admitted per repository by a publisher rule
  that declares the planes and names it may publish.
- **Trust on this route** is split three ways: the hub's signature and the
  node's `namePrefix` rule say who published the manifest under which
  vendor namespace (and the signed manifest's `sha256` and `sizeBytes` pin a
  `registry` package's bytes); `conformance` says the release passed the
  harness; the guard envelope stays the daemon's, as above.
