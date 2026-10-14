# d1fw-ee-shm-contract

The `end_effector.shm/1` contract in Rust: how an end effector driven by its
own process talks to `d1-firmwared` over iceoryx2 shared memory. Both sides
build on this crate — d1-firmwared's shared-memory driver and the provider
SDK (`d1fw-ee-provider`) — so they can disagree only by being built from
different versions of it.

## What this crate is

- `wire` — the thirteen fixed-size `#[repr(C)]` messages (`Header`,
  `Descriptor`, `Heartbeat`, `State`, `Tick`, `JointsCommand`, `Control`,
  `CommandRequest`/`Response`, `FaultRequest`/`Response`, and the `Joint` /
  `ExtraKey` entries) with their iceoryx2 type names (`omakase_ee_v1_*`) and
  every bit and code constant.
- `model` — the owned, validated form of each message and the conversions
  both ways. Decoding refuses another `schema_version`, an undefined bit, an
  over-capacity count, malformed text and non-finite numbers; encoding
  refuses what the receiver would.
- `names` — a side's eight service names under its base
  (`omakase/ee/<side>` by default).
- `ports` — the iceoryx2 settings of every service (one writer per topic,
  latched descriptor/state/control, newest-wins joints), opened identically
  by both sides, and the contract's timing rules.
- `root` — `ShmRoot`, the root directory (`/run/d1-firmwared/iceoryx2/`) and
  prefix (`omakase_ee_`) every participant builds its node from;
  `ShmRoot::scratch` gives a test a root of its own.
- `layout` — the machine-readable layout; it must equal the committed
  `spec/end_effector.shm/1.layout.json`, and the spec's tables must list
  every field where the layout puts it.
- `flange` — the sibling contract `flange_bus.shm/1`
  (`spec/flange_bus.shm/1.md`): an arm side's flange CAN channel relayed
  between the daemon and the provider of the side's end effector. Its own
  messages (`omakase_flange_v1_*`: `Frame`, `RxFrame`, `TxFrame`,
  `Heartbeat`, `Status`), their validated models, the four topics' settings
  and its own layout file, pinned by `flange::layout::LAYOUT_SHA256`.

## Scope

The normative texts are `spec/end_effector.shm/1.md` and
`spec/flange_bus.shm/1.md`; the provider author's guide is
`docs/end-effector-shm-interface.md`. This crate depends on iceoryx2
(`=0.9.3`), serde and rustix only, never on d1-firmwared's crates.
