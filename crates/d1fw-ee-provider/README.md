# d1fw-ee-provider

The provider SDK of `end_effector.shm/1`: what a process that drives its own
end effector links to serve it to `d1-firmwared` over iceoryx2 shared
memory.

## What this crate is

- `Provider` — the trait a provider implements: its `Descriptor`, `set`
  (an openness), `joints` (timestamped targets), `read`, `stop` (never
  opens, idempotent), and optionally `clear_fault`, `registers`, `ticks` and
  a daemon-presence hook.
- `serve` — owns every port on a thread of its own and applies the
  contract's rules before the provider is called: a `set` for another
  instance or generation or past its start deadline is answered
  `stale_target`; undeclared grips, torques and `extra` keys are refused;
  stale or out-of-limit joint targets are dropped and the newest delivered
  one at a time; a rise of the daemon's stop generation calls `stop`; the
  heartbeat, the descriptor republish and the state read run on their
  periods. Dropping the returned handle closes every port.
- `check_set`, `check_joints` — those checks, for a provider's own use.
- `FlangeBus` — the provider's side of `flange_bus.shm/1`, for a device on
  the arm's flange CAN channel: a port thread of its own publishes the
  heartbeat and keeps the relay's newest `status`; `recv` takes the
  channel's frames in order (gaps counted in `stats().rx_lost`); `send`
  publishes a frame labelled `TxKind::Motion`, `Read` or `Stop`, refusing at
  once what the relay would refuse (not admitted, channel down, motion under
  the soft-kill latch, a frame the channel does not carry) and holding a
  frame while more than `TX_WINDOW` are untaken or the rate ceiling is
  reached (a stop is never held for the ceiling). `wait_admitted` waits for
  the relay to bind this provider.
- `TestProvider` (feature `test-support`) — a simulated one-joint gripper
  that records every call, for the daemon's tests.

## Scope

It depends on the contract and tokio, never on d1-firmwared's crates. The contracts'
texts are `spec/end_effector.shm/1.md` and `spec/flange_bus.shm/1.md`; a
provider in another language implements the same messages from their
`1.layout.json` files.
