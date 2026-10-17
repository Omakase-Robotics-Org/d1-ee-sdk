# Changelog

All notable changes to `d1-ee-sdk` are recorded here, newest first. The
format follows [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/);
versions follow `0.x` SemVer (README, "Versions").

## [Unreleased]

### Added

- `d1-arm-loopbench` reports the daemon's own record of the command path
  after a stream, when the daemon offers it (`GET /v1/arm/command_path`):
  the time of each command per stage inside the daemon over the last second
  (p50 / p99 / max) and the controller's uptake of the writes (echo lag,
  consumption and feedback rates, targets in flight, saturation, pacing);
  `daemon_command_path` in the JSON report (`null` without it).
- A hold, within `end_effector.shm/1` (layout SHA-256 unchanged): a `set`
  whose `extra` is `{"hold": true}`, taken by a provider that lists `hold`
  in `set_extra` whether or not it declares `openness`. The provider keeps
  the last set-point its control sent, with its own command watchdog
  disarmed, until the next `joints`, `set`, `stop`, fault or cleared fault;
  a hold gives no authority back to actuators that are off (spec §4 rule 3
  addendum and rule 10). `model::HOLD_EXTRA_KEY`, `SetCommand::hold` and
  `is_hold`; `check_set` and the SDK runtime accept and
  check it; `Host::hold`; `conformance::lifecycle` drives a hold when the
  declaration lists it; `TestRecord::holds`.
- A home, the same way: a `set` whose `extra` is `{"home": true}`, taken by
  a provider that lists `home` in `set_extra`, drives to the provider's own
  home pose through its normal control and then holds (spec §4 rule 11).
  `model::HOME_EXTRA_KEY`, `KEYED_SETS`, `SetCommand::home`, `keyed`,
  `keyed_verb`, `is_home`, `check_keyed` (which replaces `check_hold`);
  `Host::home`; the lifecycle drives a home when listed;
  `TestRecord::homes`.
- A hold's lifetime and a stop's pose (spec §4 rules 4 and 10), with no
  layout change: a provider that lists `hold` declares `hold_timeout_ms`
  (cap 300000 ms) and treats an expired hold as a stop; a back-drivable
  hand declares `stop_hold_ms` (cap 3000 ms) to keep its pose at reduced
  effort after a stop before it relaxes; `hold_expiring` and `stop_relaxing`
  notices precede each end and the events are recorded in the `hold` object
  of the state's `detail` and in the provider's log. A settle definition
  (2 % of range, 5 % of range per second, 100 ms) says when `tracking`
  clears. `d1fw_ee_shm_contract::hold` (`HoldLifetime`, `HoldReport`,
  `HoldEvent`, `Joint::settled`, the caps and leads); the test provider
  implements it (`TestRecord::lifetime`, `hold_events`, `hold_phase`) and
  `conformance::lifecycle` checks the declared report after a hold.

### Changed

- `State.tracking` is defined (spec §3.4): a streamed target taken and not
  yet reached, cleared once the end effector has settled (a settle is defined in §4
  rule 10) and only holds.

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
