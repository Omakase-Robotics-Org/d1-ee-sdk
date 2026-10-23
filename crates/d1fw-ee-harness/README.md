# d1fw-ee-harness

The daemon's side of `end_effector.shm/1` and `flange_bus.shm/1` for a
provider's tests, so a provider is proven without d1-firmwared or a robot:

- `Host`: one arm side as d1-firmwared serves it — the declaration accepted
  and kept, liveness from the heartbeat, `set` and `joints` validated against
  the declaration and paced at its `max_command_hz`, the stop generation,
  the side's HTTP status for every refusal (`HostError::http_status`). With
  `Host::start_with_flange_channel`, the flange relay over any `CanFdBus`
  (a simulated device) with the relay's gates; `latch` and
  `set_arm_faulted` drive the soft-kill latch and the `arm_faulted` bit.
  A `SetRequest`'s `force_percent` follows the daemon's grip-force rules
  (spec §9): refused outside `[0, 100]`, beside a grip or a torque, or
  inside `extra`; sent as `extra.force_percent` to a provider that declares
  the key and dropped for one that does not; `Host::grip_force` is the
  side's report as the daemon serves it.
- `Recorder` and `golden`: every frame the relay puts on the channel, with
  its kind, compared step by step with a recording within timing
  tolerances.
- `ProviderProcess` and `conformance::lifecycle`: a provider binary as a
  child process through the lifecycle every provider must follow
  (declaration, every declared verb, SIGKILL, restart, SIGTERM).
- `d1-ee-test-provider`: the SDK's `TestProvider` as a process.

The host is a copy of the daemon's behaviour, not the daemon: a provider that
passes here is then run once against `d1-firmwared run --backend sim`.
