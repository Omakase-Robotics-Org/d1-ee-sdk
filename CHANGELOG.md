# Changelog

All notable changes to `d1-ee-sdk` are recorded here, newest first. The
format follows [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/);
versions follow `0.x` SemVer (README, "Versions").

## [Unreleased]

### Changed

- A provider manifest's `id` is a dotted namespace of two to six lowercase
  segments (`ID_PATTERN`, at most `ID_MAX_LENGTH` = 64 characters), by
  convention `d1-firmware.end-effector.<vendor>.<model>`; it was exactly
  `<vendor>.<name>`. The template's id is
  `d1-firmware.end-effector.acme.g1`. The providers the hub distributes are
  republished under `d1-firmware.end-effector.dh.ag`, `….damiao.motor`,
  `….linkerbot.o30`, `….leadshine.dh116` and `….robotera.xhand-lite` (same
  versions); the `omakase.*` entries stay on the hub, retired
  (`docs/third-party-providers.md`). `ProviderManifest::vendor` is the
  segment before the last. The id's accept and refuse cases are a shared
  rules file (`tests/fixtures/rules/provider-id.txt`), held by
  d1-provisioning's reader as well.

### Added

- The provider manifest `omakase-ee-provider/1`: its JSON Schema
  (`schema/omakase-ee-provider-1.schema.json`) and the crate
  `d1fw-ee-manifest` — the `ProviderManifest` type, the rules the schema
  cannot state (the SDK range holding the conformance harness, `{side}` as
  the only placeholder, the program under `/opt/<package>/bin/`, a registry
  package's key naming the release's `.deb`, one posture value per joint, a
  self-contained draft 2020-12 `config_schema`), the check against a
  provider's descriptor, and the command `d1fw-ee-manifest check <file>
  [--descriptor <json>]`. The manifest of d1-ee-ag 0.1.1 is its golden test.
- The manifest's rules are those of d1-provisioning's on-robot reader, case
  for case (`crates/d1fw-ee-manifest/tests/fixtures/rules/`, shared with
  d1-provisioning's tests): a provider that reads no configuration file
  names none of `launch.config_dir`, `launch.config_file` and
  `config_schema` (all three or none; `calibration.required` then false);
  `package.arch` takes `all`; `launch.command` is one character allow-list,
  `COMMAND_PATTERN`; `launch.config_dir` is a directory of its own under
  `/etc`, not the daemon's layer directory nor under `/etc/systemd*` or
  `/etc/apt*`; `launch.config_file` ends in `.toml`; a version range is a
  comparator list with plain `MAJOR.MINOR.PATCH` operands, a bare version
  exact (`version_range`, `RANGE_PATTERN`), no Cargo caret, tilde or
  wildcard. `d1fw-ee-manifest check` warns about valid JSON the reader
  refuses (`\u`, `\b`, `\f` escapes, empty or dotted keys, a key twice).
  The published documents (the five provider manifests and the fit of d1-3
  revision 2) are test fixtures.
- `template/`: a provider repository to copy — a crate implementing
  `Provider` over a fake transport, its conformance test against
  `d1fw-ee-harness`, Debian packaging, CI and release workflows, and its
  `manifest.json` — built and tested against this checkout by
  `tools/check-template.sh`, a step of CI.
- `docs/third-party-providers.md`: the developer path from a device to a
  robot without an Omakase service, what is trusted and what checks what,
  and the hub route of the providers the artifacts hub distributes.

- `d1-arm-loopbench` reports the daemon's own record of the command path
  after a stream, when the daemon offers it (`GET /v1/arm/command_path`):
  the time of each command per stage inside the daemon over the last second
  (p50 / p99 / max) and the controller's uptake of the writes (echo lag,
  consumption and feedback rates, targets in flight, saturation, pacing);
  `daemon_command_path` in the JSON report (`null` without it).

## [0.2.0] - 2026-10-16

### Added

- The arm contracts `arm_feedback.shm/1` (layout SHA-256
  `f9bbac48eff4d24449d66f0219ee32960ecf58cf39aebc4d49b522909758b65a`),
  `arm_status.shm/1` (`c7d6685594206be6d58ed7d7f28fe5701eaf3818dc7d40d45156143c621868fb`),
  `arm_events.shm/1` (`2377745c6660434a2b805afb5feb21b7299a38cbfa03928b3e53a1d87142d619`)
  and `arm_command.shm/1` (`987222e48bb079782e01a5d541b0bf19017a67fdf53dbacbc4172d278493c4be`),
  as d1-firmwared serves them, with their Rust crate `d1fw-arm-shm-contract`
  (iceoryx2 `=0.9.3`, the end-effector contracts' root and header).
- `d1-arm-loopbench`: a client of the arm contracts that streams
  `arm_command.shm/1` at a chosen rate and reports the written rate, the
  daemon-added and end-to-end latency, the controller's frame-miss counters
  and every refusal, with gates for a robot run.
- `tools/check-layout-sha.sh` checks the four arm layouts' pins too.

## [0.1.0]

### Added

- The contracts `end_effector.shm/1` (layout SHA-256
  `30414fd5c321b72ee171a14edb56fca3ac2fa19b25881454383fc9a160f5dfee`) and
  `flange_bus.shm/1` (layout SHA-256
  `43d74ebf4944cf21c40e368cf6816ef0bc71698a5b2fe34988d8dfe12bf766b7`), as
  d1-firmwared serves them, with their Rust crate `d1fw-ee-shm-contract`
  (iceoryx2 `=0.9.3`).
- `d1fw-ee-provider`: the provider SDK (`Provider`, `serve`, `FlangeBus`,
  `TestProvider`) and the unit and packaging conventions (`unit`:
  `PROVIDER_UNIT`, `PROVIDER_CONFIG_DIR`, `program_path`).
- `d1fw-canfd`: the CAN FD frame, frame kinds and bus seam.
- `d1fw-ee-hand`: the multi-finger hand framework and the `hand.state/1`
  detail with its JSON Schema (`schema/hand.state-1.json`).
- `d1fw-ee-harness`: `Host` (the daemon's side of both contracts, with the
  flange relay over a simulated device and the daemon's HTTP statuses),
  `Recorder`, golden-frame comparison, `ProviderProcess`,
  `conformance::lifecycle`, and the binary `d1-ee-test-provider`.
- The template unit `deploy/systemd/d1-firmwared-ee-provider@.service`.
- The provider author's guide `docs/end-effector-shm-interface.md`.
