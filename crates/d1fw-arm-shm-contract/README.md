# d1fw-arm-shm-contract

The D1 arm contracts in Rust: how `d1-firmwared` publishes everything it
reads from the arm controller and takes joint commands over iceoryx2 shared
memory. The daemon and every client build on this crate, so they can
disagree only by being built from different versions of it.

## What this crate is

- `wire` — the fixed-size `#[repr(C)]` messages of the four contracts
  (`omakase_arm_feedback_v1_*`, `omakase_arm_status_v1_*`,
  `omakase_arm_events_v1_*`, `omakase_arm_command_v1_*`) and every code and
  bit constant; NUL-padded text helpers.
- `names` — the service names under the base (`omakase/arm` by default).
- `ports` — every service's iceoryx2 settings, generic over the service
  variant (`ipc` or `ipc_threadsafe`), and the `command` event service's
  notifier and listener.
- `model` — headers, the checks the contract makes of a command (header,
  sides, flags, finite targets, age), and the lease token's text form.
- `layout` — the machine-readable layout of each contract; each must equal
  the committed `spec/arm_*.shm/1.layout.json`, whose SHA-256 is pinned in
  `FEEDBACK_LAYOUT_SHA256`, `STATUS_LAYOUT_SHA256`, `EVENTS_LAYOUT_SHA256`
  and `COMMAND_LAYOUT_SHA256`, and each spec's tables must list every field
  and code where the layout puts it.

The shared-memory root, the clock and the 32-byte header are the
end-effector contracts' (`d1fw-ee-shm-contract`), re-exported as `root` and
`clock`.

## Scope

The normative texts are `spec/arm_feedback.shm/1.md`,
`spec/arm_status.shm/1.md`, `spec/arm_events.shm/1.md` and
`spec/arm_command.shm/1.md`. Regenerate the layouts after a deliberate
contract change with `D1FW_ARM_WRITE_LAYOUT=1 cargo test -p
d1fw-arm-shm-contract layout`; a version-1 layout never changes.
