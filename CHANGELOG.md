# Changelog

All notable changes to `d1-ee-sdk` are recorded here, newest first. The
format follows [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/);
versions follow `0.x` SemVer (README, "Versions").

## [Unreleased]

### Added

- A hold, within `end_effector.shm/1` (layout SHA-256 unchanged): a `set`
  whose `extra` is `{"hold": true}`, taken by a provider that lists `hold`
  in `set_extra` whether or not it declares `openness`. The provider keeps
  the last set-point its control sent, with its own command watchdog
  disarmed, until the next `joints`, `set`, `stop`, fault or cleared fault;
  a hold gives no authority back to actuators that are off (spec §4 rule 3
  addendum and rule 10). `model::HOLD_EXTRA_KEY`, `SetCommand::hold`,
  `is_hold` and `check_hold`; `check_set` and the SDK runtime accept and
  check it; `Host::hold`; `conformance::lifecycle` drives a hold when the
  declaration lists it; `TestRecord::holds`.

### Changed

- `State.tracking` is defined (spec §3.4): a streamed target taken and not
  yet reached, cleared once the end effector has arrived and only holds.

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
