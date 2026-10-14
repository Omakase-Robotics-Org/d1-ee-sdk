# d1fw-ee-hand

What the multi-finger hand providers share. A hand on a D1 flange talks over
the arm's end-module CAN channel, which only `d1-firmwared` reaches; its
driver is a provider process of its own that serves the side over
`end_effector.shm/1` and reaches the hand through the daemon's relay of that
channel, `flange_bus.shm/1`. Each hand model's provider, in a repository of
its own, supplies its wire and its facts; this crate is the rest. The
daemon links none of it.

## What this crate is

- `link` — the per-side link serialization with command priority (a read in
  flight yields the bus to a waiting command, `CommandsWaiting`,
  `SerializedLink`), the stale-sample cache, the probe band (ids 1–15, the
  factory id first) and the reply timings.
- `state` — `HandState`, the `hand.state/1` detail every hand publishes, and
  its JSON Schema `schema/hand.state-1.json` (`HAND_STATE_SCHEMA`). The
  daemon's telemetry decodes the detail by that id;
  `tests/hand_state_schema.rs` validates a serialized state against the
  committed schema.
- `provider` — `HandProvider<L: HandLink>`, the `end_effector.shm/1` provider
  of any hand link: openness through the model's posture mapping, joints as
  fractions of each axis's wire range, the first command after a stop
  enabling the joints, a state read that never interrupts a command, and a
  stop that disables without opening (a repeated stop with no command in
  between sends nothing, since the daemon delivers one stop twice).
- `channel` — `FlangeChannel`, the relayed channel as a `d1fw_canfd::CanFdBus`:
  a link written against the bus seam runs over the relay unchanged and
  labels its frames with `FrameKind` (motion, read, stop); `flush` returns
  once the relay has taken what was sent.
- `tool` — the tool a hand declares, read from its compiled-in
  `manipulation-kit` tool configuration.
- `sim` — `SimBus` (a device model on the bus seam) and `SuppressRepeats` (the
  end module's repeat suppression around any bus).
- `app` — the provider process: `--side`, `--root`, `--prefix`, `--service`,
  `--flange`, `--state-period-ms`, `--sim`; startup opens the flange channel,
  waits for the relay to admit it, probes for the hand (again every 2 s while
  none answers) and serves it until SIGTERM or SIGINT.
- `harness` (feature `test-support`) — the scenario a hand crate's tests
  drive its provider with, on the SDK's host (`d1fw-ee-harness`: the
  daemon's side of both contracts, the flange relayed to a simulated hand),
  and the cross-process conformance run of a provider binary.

## Scope

The contracts are `spec/end_effector.shm/1.md` and `spec/flange_bus.shm/1.md`;
the SDK is `d1fw-ee-provider`. Without `test-support` this crate depends on
the SDK and `d1fw-canfd` only.
