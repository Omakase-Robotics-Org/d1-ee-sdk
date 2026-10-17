# Writing an end-effector provider for the D1

An end effector on a D1 arm is driven by a process of its own — a
**provider** — that `d1-firmwared` (the **daemon**) serves to every client
through its one `/v1/end_effectors` resource. The wire contract is
`spec/end_effector.shm/1.md` with its byte layout
`spec/end_effector.shm/1.layout.json`; a device on the arm's flange CAN
channel also uses the sibling contract `spec/flange_bus.shm/1.md` (the
flange relay). This guide explains why the contracts are shaped as they are,
what the daemon does with a provider, how a provider is written with this
SDK, tested without a robot, packaged, and run.

## Requirement and scope

Anyone must be able to implement an end effector as their own process, talk
to the daemon over shared memory, and have the daemon drive it through
`/v1/end_effectors` — with no change to the daemon for a new device, and
without access to the daemon's source. The daemon keeps every safety duty
for every end effector: validation, the timestamp rule, the command
ceiling, the soft-kill latch and the stop, tool registration from a
declaration, the arm guard, and the REST/WebSocket/ROS 2 surfaces. Every end
effector is a provider, the D1's stock gripper included, and every provider
is a package of its own built on this SDK; one vendor's provider never
depends on another's. The vendor provider packages for the end effectors
d1-firmwared knows ship with the robot; their source is not public.

A provider's device is either on a bus the daemon does not own (a USB
adapter, an EtherCAT master, a vendor SDK), which the provider opens itself,
or on the arm's flange CAN channel — the arm controller's end-module
passthrough, a UDP link only the daemon reaches. For the second case the
daemon relays that channel to the side's provider over shared memory
(`flange_bus.shm/1`, below) and keeps the channel's gates: the transport has
one owner, and the provider never holds it.

`end_effector.shm/1` is the one published boundary for every end effector.
An end effector driven over ROS 2 is no exception: it reaches the daemon
through a bridge process that is a `shm/1` provider, so the daemon has a
single external driver to keep safe and a client sees one resource whatever
carries the device.

## Shape

```
 client ──REST/WS──▶ d1-firmwared  (validation, timestamp lanes, kill latch,
                      one shared-memory driver per side, the flange relay)
                 /run/d1-firmwared/iceoryx2/, prefix omakase_ee_
                      omakase/ee/<side>/{descriptor, heartbeat, state, tick,
                        joints, control, command, fault}
                      omakase/flange/<side>/{rx, tx, status, heartbeat}
                                                        (flange_bus only)
                      provider process (this SDK in Rust, or any iceoryx2
                        0.9.3 binding, e.g. Python)
                        └─ the device: on the relayed flange channel, or on
                           a bus the provider opens itself
```

This repository carries:

| Crate | Holds | Depends on |
| --- | --- | --- |
| `d1fw-ee-shm-contract` | both contracts: the service names, the fixed-size messages, their validation and conversions, the per-service iceoryx2 settings, the shared-memory root, the layout documents (each pinned by SHA-256) | iceoryx2 `=0.9.3`, serde, rustix |
| `d1fw-canfd` | the CAN FD frame, the frame kinds a relayed frame carries (`motion`, `read`, `stop`) and the synchronous bus seam a device driver is written against | thiserror |
| `d1fw-ee-provider` | the provider SDK: the `Provider` trait and `serve`; `FlangeBus`, the provider's side of the flange relay; the unit and packaging conventions (`unit`); `TestProvider` behind the `test-support` feature | the contract, tokio |
| `d1fw-ee-hand` | what a multi-finger hand provider is built from: the serialized link with command priority, the `hand.state/1` detail and its JSON Schema, the relayed channel as a `CanFdBus`, a simulated device and the end module's repeat suppression, the provider process around a hand model | the SDK, `d1fw-canfd` |
| `d1fw-ee-harness` | the daemon's side of both contracts for a provider's tests (`Host`), frame recording, golden-frame comparison, the lifecycle conformance run of a provider process, and `d1-ee-test-provider` | the SDK |

d1-firmwared builds its side from the same contract crate at the same
release tag, and pins both layouts' SHA-256 in its own tests, so the daemon
and every provider compile the same frozen layout.

## Ownership and permissions

iceoryx2 0.9.3 creates every shared-memory segment, socket and lock file
owner-only, from compile-time constants, and offers no configuration of it:

| What | Where in the crate sources (0.9.3) | Mode |
| --- | --- | --- |
| shared-memory segments (`/dev/shm/<prefix>…`) | `iceoryx2-cal/src/dynamic_storage/posix_shared_memory.rs:76-82` (`INIT_PERMISSIONS`, `FINAL_PERMISSIONS`), applied with `shm.set_permission(FINAL_PERMISSIONS)` after creation at `:375` | `Permission::OWNER_ALL` |
| event and channel sockets | `iceoryx2-cal/src/event/unix_datagram_socket.rs:35` (`SOCKET_PERMISSIONS`, set at `:423` with the process umask overridden for the call, `iceoryx2-bb-posix/src/unix_datagram_socket.rs:433`), `iceoryx2-cal/src/communication_channel/unix_datagram.rs:37` | `Permission::OWNER_ALL` |
| node monitoring locks | `iceoryx2-cal/src/monitoring/file_lock.rs:49` (`GUARD_PERMISSIONS`) | `Permission::OWNER_ALL` |
| static service files | `iceoryx2-cal/src/static_storage/file.rs:68` (`FINAL_PERMISSIONS`) | `Permission::OWNER_READ` |

Every one of them has a `#[cfg(feature = "dev_permissions")]` twin set to
`Permission::ALL` (the feature is `dev_permissions =
["iceoryx2-cal/dev_permissions"]`, `iceoryx2/Cargo.toml:44`), which makes
every segment readable and writable by every local account, the `control`
and `command` services included; it is not enabled here and is not an
option on a robot. `iceoryx2/src/config.rs` has no permission field. Because
the mode is set explicitly after creation, neither a umask nor a setgid root
directory can widen it to a group. Two processes therefore see each other's
services only when they run as the same account and resolve the same root
directory and prefix.

This is the precedent the fleet already follows for cameras: the sensor HAL
(`seiryu-robot-sensor-hal`) runs as its own `seiryu` account with root
`/run/seiryu/iceoryx2` and prefix `seiryu_` (its account's iceoryx2 file
`/var/lib/seiryu/.config/iceoryx2/iceoryx2.toml`), and its consumers on a D1
run as `seiryu` (`d1-host`'s drop-in `omakase-core.service.d/
10-seiryu-account.conf`), because nothing else can open those segments.

The end-effector services follow the same rule with the daemon as the
owner:

- **The root is the daemon's.** `/run/d1-firmwared/iceoryx2/`, inside the
  daemon's `RuntimeDirectory=d1-firmwared` (mode 0750, owned by the unit's
  account, kept across restarts by `RuntimeDirectoryPreserve=yes`). The
  daemon creates the `iceoryx2/` directory when it opens its services; no
  package file, tmpfiles entry or unit change is needed, and the daemon's
  `PrivateTmp=true` does not matter because the root is not under `/tmp`.
- **A provider runs as the daemon's account.** On a D1 provisioned by
  `d1-provisioning`, `d1-firmwared.service` runs as `User=doraemon` (the
  robot's login account, created by `d1-host`'s postinst), so a provider
  runs as `doraemon`. The contract states it as "the account the daemon runs
  as", so a host whose daemon runs elsewhere moves both together.
- **The configuration is explicit, never iceoryx2's global file.** Both
  sides build their node from iceoryx2's default configuration with the root
  and prefix set in code (`ShmRoot::config`). A global file would differ per
  account (`$HOME/.config/iceoryx2/`), and the daemon's unit has
  `ProtectHome=true`; the existing `/d1` iceoryx2 frontend
  (`d1fw-front-iox`) keeps its own default configuration and is unaffected.

### Running a provider from your own home

A provider's program does not have to be installed system-wide: it may live
in a developer's home directory, as long as it runs as the daemon's account.
The d1-firmwared package installs the template unit
`/lib/systemd/system/d1-firmwared-ee-provider@.service` (its text is this
repository's `deploy/systemd/d1-firmwared-ee-provider@.service`,
`d1fw_ee_provider::unit::PROVIDER_UNIT`), one instance per arm side, which
runs a configured command as `doraemon`:

1. Let the daemon's account reach the program (home directories are usually
   0750): `setfacl -m u:doraemon:x /home/<user>` and
   `setfacl -R -m u:doraemon:rX /home/<user>/<provider>`.
2. Name the command in `/etc/d1-firmwared/ee-provider/<side>.env` (create
   the directory, mode 0755, owned by root):

   ```sh
   EE_PROVIDER_COMMAND=/home/<user>/acme/bin/acme-provider --service omakase/ee/b
   ```

   For the kit's reference provider in a virtual environment of your own:
   `EE_PROVIDER_COMMAND=/home/<user>/venv/bin/python -m
   manipulation_kit.end_effectors.shm_provider --model gripper --service
   omakase/ee/b`.
3. `systemctl enable --now d1-firmwared-ee-provider@b`. The unit requires and
   follows `d1-firmwared.service`, may write only under `/run/d1-firmwared`
   (and `/dev/shm`), and sees home directories read-only.

While developing, the same account is reached by hand:
`sudo -u doraemon /home/<user>/acme/bin/acme-provider --service
omakase/ee/b`.

What running as the daemon's account means is stated plainly in the
contract: a provider can open everything the daemon's account can. A
provider is equipment software that drives an actuator on the robot, and is
trusted as such; the shared-memory boundary is a process boundary, not a
security boundary. A group-scoped root (a dedicated group both accounts
join) needs a configurable segment mode, which iceoryx2 0.9.3 does not
offer.

## The contract in brief

The full text is the spec; the decisions behind it:

- **Fixed-size messages, `#[repr(C)]`, no implicit padding.** Thirteen
  messages from 32 bytes (`Header`) to 16 680 bytes (`FaultResponse`), each
  with a header carrying `schema_version`, the sender's random
  `instance_id`, a per-service `seq` and a `CLOCK_MONOTONIC` stamp. Fixed
  sizes keep every message a single iceoryx2 sample with no allocation
  negotiation, and they are what the Python bindings (ctypes structures)
  express directly. Strings are NUL-padded UTF-8; optional floats are NaN.
- **Opaque payloads stay JSON, inside fixed buffers.** The `detail`
  (8 KiB), `set`'s `extra` (1 KiB), the register ids (1 KiB) and the fault
  lane's bodies (16 KiB) carry the same JSON objects the REST surface
  serves, so the daemon forwards them without knowing their schema. A detail
  too large is dropped and flagged, never cut.
- **Eight services per side.** Provider to daemon: `descriptor` (latched),
  `heartbeat`, `state` (latched), `tick`. Daemon to provider: `joints`
  (newest wins, no history), `control` (latched: the daemon's liveness and
  stop generation). Request-response, provider as server: `command` (`set`,
  `stop`) and `fault` (`clear_fault`, `registers`). Every topic admits one
  writer, which is what keeps a second provider off a bound side.
- **Two command lanes, two delivery rules.** `set` is a request: it can
  wait for its stroke and its error comes back to the REST caller (409, 422,
  502, 504 as for any driver). `joints` is a stream: the REST call returns
  once the target is published, a newer target supersedes an older one
  anywhere on the way, and the provider takes only the newest per wake-up.
- **Nothing executes late.** A `set` carries a start deadline (issue + 250
  ms) and the instance and descriptor generation it was validated against; a
  provider answers `stale_target` instead of executing one that misses any
  of them. A `joints` sample older than 200 ms when the provider takes it is
  dropped. A `stop` always executes.

## What the daemon does with a provider

The contract's §5 is normative; in short:

- **The declaration.** The first valid descriptor is accepted and kept for
  the daemon's lifetime. A restarted provider announcing the same
  declaration is taken back at once; a different one is refused, naming
  both, until the daemon restarts. The one field a later declaration may
  change is the bus address, from `null` to one value, once. A `model` the
  side's configuration pins must be the one announced.
- **Liveness.** The side is available while the accepted instance's newest
  heartbeat is younger than the side's stale threshold (default 500 ms) and
  names the accepted generation; on a flange side, also while the
  declaration names a node.
- **Validation and pacing.** Every command is validated against the
  declaration (openness range, declared grips, torques and `extra` keys,
  joint count and limits) before it reaches the provider, and `set` and
  `joints` never leave faster than the declared `max_command_hz` (a `set`
  waits for its slot, a newer `joints` target replaces a waiting one).
- **The stop.** A stop raises the side's stop generation on `control` at
  once — which the SDK acts on even if the request never arrives — drops a
  waiting `joints` target, then asks the provider to stop. A soft kill stops
  both sides and refuses every command until it is released; under it the
  relay passes only a provider's reads and stops.
- **The tool and the guard.** A declared tool is registered on the side's
  arm when the declaration is accepted (a client's `POST
  /v1/arm/{side}/tool` outranks it); a declared `guard_model` (the model id) is resolved through the hand catalog
  of the robot's description bundle; an id it does not resolve, or resolves
  to an unverified envelope, is an unknown hand, checked with the default
  envelope widened by a margin (or, on a robot that opted in, refused), and
  the side's descriptor says so. A provider that disappears leaves its side
  unknown, not back on the default.
- **The detail.** A state's `detail` is published only under the
  `detail_schema` the declaration names; the id is the provider's own
  (`<id>/<major>`).

## The flange relay (`flange_bus.shm/1`)

**Why it exists.** An end effector on a D1 flange (the stock gripper, a
hand) talks over the arm's end-module CAN channel: the arm controller relays
CAN frames in datagrams over the UDP link the daemon owns. No other
process can reach it, and a second owner of the arm controller's transport
is not acceptable. So the daemon keeps the channel and relays it, per side,
to the side's provider over shared memory: the contract
`spec/flange_bus.shm/1.md`, layout `spec/flange_bus.shm/1.layout.json`
(pinned by SHA-256 in `flange::layout::LAYOUT_SHA256`), type names
`omakase_flange_v1_*`, the same 32-byte header as `end_effector.shm/1`.

**Services.** Under `omakase/flange/<side>`, in the same root and prefix as
the side's `end_effector.shm/1` services:

| Service | Writer | Payload | Settings |
| --- | --- | --- | --- |
| `rx` | daemon | `RxFrame`: a frame taken off the channel, with its `CLOCK_MONOTONIC` time | history 0, buffer 64, newest kept; a gap in `seq` counts what a slow reader lost |
| `tx` | provider | `TxFrame`: a frame to send, labelled `motion`, `read` or `stop` | history 0, buffer 64, read by the daemon alone |
| `status` | daemon | `Status`: the bound provider, `tx_seq_seen`, the channel's state and facts, the latch, the ceilings, a counter per refusal reason | latched; every 100 ms and on any change |
| `heartbeat` | provider | `Heartbeat` | at least every 100 ms |

Every topic admits one writer: one relay (one daemon) and one provider per
side.

**Binding.** The relay takes frames from the sender of the newest
heartbeat younger than the side's `stale_after_ms` (the threshold the side's
`end_effector.shm/1` liveness uses), and unbinds it when its heartbeat gets
older. A restarted provider is a new instance and is bound on its first
heartbeat.

**The gates, in order.** Each refused frame is counted in `status` and the
first refusal of each reason per bound provider is logged:

1. `invalid` — a frame that does not decode, longer than 64 bytes, or with
   a 29-bit identifier (the channel, a `CanFdBus`, carries 11-bit ones);
2. `unbound` — a sender that is not the bound provider;
3. `stale` — a frame whose header `mono_ns` is more than 100 ms old when the
   relay takes it: a frame delayed that long is not sent late;
4. `latched` — a `motion` frame while the end effectors' soft-kill latch
   stands (the relay refuses every motion frame until the daemon has wired
   its latch);
5. `rate` — a `motion` or `read` frame beyond `flange_max_tx_hz` (a token
   bucket holding 50 ms of frames). A `stop` frame is never refused by the
   ceiling.

What passes is put on the channel in `tx` order. A failed send turns
`channel_up` off until a send or a receive succeeds again; a gap in the bound
provider's `seq` is counted as `tx_lost`. The relay never sends a frame of
its own and never alters one.

**Why reads and stops pass the latch.** A soft kill stops the side through
`end_effector.shm/1` (the stop generation and the `stop` request), and a
provider's stop is a frame on this channel — the gripper motor's or the
hand's joints' disable — which must reach the device. A read moves nothing, and `end_effector.shm/1`
§5 keeps reads (and `clear_fault`, `registers`) reachable under the latch;
for a polled device every read is a frame. The relay cannot tell a frame's
effect without device knowledge, so the provider labels each frame, and the
label is trusted: the relay is a process boundary, not a security boundary,
because the provider runs as the daemon's account and could forge anything
the contract carries. The gates contain a provider that is wrong (stale, too
fast, still streaming when the latch rises), not a hostile one.

**When a provider dies.** The relay does not stop the device: it has no
device knowledge, and the device keeps its last command. The side's
`end_effector.shm/1` resource turns unavailable, every command is refused,
and the next provider instance takes the current stop generation as its
baseline.

**The SDK's side: `FlangeBus`.** A port thread of its own publishes the
heartbeat and keeps the newest status. `recv` takes received frames in
order (gaps counted in `stats().rx_lost`). `send` refuses at once what the
relay would refuse — not admitted, the channel down, a motion frame under
the latch, a frame the channel does not carry — so a device driver fails a
command immediately instead of waiting out an answer that cannot come; it
keeps at most 32 frames published beyond the status's `tx_seq_seen` (half the
`tx` buffer, so a frame is never overwritten unread) and paces frames to the
relay's ceiling, waiting up to 100 ms before it reports the relay busy.
`flush` returns once the relay has taken every frame sent, which is how a
write a device does not acknowledge returns only once it has left for the
channel. `d1fw-ee-hand`'s `FlangeChannel` presents all of this as a
`d1fw_canfd::CanFdBus`, so a link written against the bus seam runs over the
relay unchanged; `CanFdBus::send_as` carries the frame's kind.

**Latency.** The relay takes `tx` frames at least every millisecond and
publishes a received frame as soon as the channel hands it over; the SDK
reads `rx` at least every half millisecond. On an idle host a frame reaches
the channel within about 1 ms of the provider's send and an answer reaches
the provider within about 1 ms of the channel delivering it: a
request-answer exchange costs about 2 ms more than an in-process driver
paid, on top of the arm controller's own relay. It is not a real-time path.

**What the Python kit needs for a flange client** (not implemented): ctypes
definitions of the five `omakase_flange_v1_*` messages (`Frame`, `RxFrame`,
`TxFrame`, `Heartbeat`, `Status`) checked against
`spec/flange_bus.shm/1.layout.json` with its SHA-256 pinned; a port thread
that publishes `tx` and `heartbeat` and subscribes `rx` and `status` with the
settings of the contract's §2; the window of 32 frames beyond `tx_seq_seen`
and pacing to `max_tx_hz`; and the same labelling of every frame as
`motion`, `read` or `stop`.

## The provider SDK: `d1fw-ee-provider`

A provider implements:

```rust
#[async_trait]
pub trait Provider: Send + Sync + 'static {
    fn descriptor(&self) -> Descriptor;
    async fn set(&self, command: SetCommand) -> Result<(), ProviderError>;
    async fn joints(&self, targets: Vec<f64>, timestamp_us: u64) -> Result<(), ProviderError>;
    async fn read(&self) -> Result<State, ProviderError>;
    async fn stop(&self) -> Result<(), ProviderError>;
    async fn clear_fault(&self) -> Result<FaultClear, ProviderError>;          // default: refused
    async fn registers(&self, ids: Option<Vec<String>>) -> Result<Registers, ProviderError>; // default: refused
    fn ticks(&self) -> Option<watch::Receiver<Tick>>;                           // default: none
    fn daemon_presence(&self, present: bool);                                   // default: nothing
}
```

and calls `serve(provider, ServeOptions::new(ShmRoot::default(), names))`
from a tokio runtime. The SDK applies every rule of the contract's §4 before
the provider is called: the instance, generation and start-deadline checks
on `set`; the declared-capability checks (`check_set`, `check_joints`); the
200 ms age limit and newest-first delivery on `joints` (one call at a time,
a superseded target skipped); a stop on every rise of the daemon's stop
generation, with the first value per daemon instance taken as the baseline;
the heartbeat (20 Hz), the descriptor republish (2 Hz) and the state read
(50 Hz by default, an error published as a state with no reading and the
error as its fault). Dropping the returned handle closes every port before
it returns, so a provider can be restarted inside one process.

`FlangeBus` is the provider's side of the flange relay (above).

`TestProvider` (feature `test-support`) is a simulated one-joint gripper
that records every call: what a test of the daemon's side runs against, and
what `d1-ee-test-provider` (crate `d1fw-ee-harness`) serves as a process.
Both are public examples of a provider. A reference provider in Python, on
the iceoryx2 0.9.3 bindings, lives in the public manipulation-kit repository
(module `manipulation_kit.end_effectors`, guide
`docs/end-effector-providers.md`; on its open pull request #48, branch
`feat/end-effector-shm-provider`, until merged) and checks its message
definitions against `1.layout.json`; an end effector driven over ROS 2
attaches through the bridge `omakase_ee_bridge` of the D1's ROS 2 stack (not
a public repository), itself a provider of this contract.

## Testing a provider without a robot: `d1fw-ee-harness`

`Host` stands the daemon's side of one arm side up in a test's process,
under the rules above: it accepts and keeps the declaration, judges
liveness, validates and paces `set` and `joints` the way the daemon's REST
surface does, raises the stop generation, and answers with the HTTP status
the daemon would (`HostError::http_status`). `Host::start_with_flange_channel`
adds the flange relay over any `CanFdBus` — a simulated device — with the
relay's gates; `Host::latch` raises the soft-kill latch and
`Host::set_arm_faulted` the status bit `arm_faulted`.

- `Recorder` records every frame the relay puts on the channel and the kind
  it was relayed as; `golden` compares a run's steps (status, frames,
  timings within tolerances) with a recording, which is how a provider holds
  its wire behaviour to frames captured before a change.
- `ProviderProcess` and `conformance::lifecycle` run a provider binary as a
  child process and check what every provider must do: announce a valid
  declaration, answer every verb it declares, turn unavailable for want of
  a heartbeat when killed with SIGKILL, be taken back when restarted, and
  exit 0 on SIGTERM.
- `d1-ee-test-provider` serves `TestProvider` as a process, for a host-side
  check that needs a provider to talk to.

The harness is a copy of the daemon's behaviour, not the daemon: a provider
proven against it is then run once against d1-firmwared in its simulator
(`d1-firmwared run --backend sim` with the side declared `driver = "shm"`)
before it reaches a robot.

## Packaging and running a provider

A provider is a process of its own, started by the template unit
`d1-firmwared-ee-provider@<side>` that the d1-firmwared package installs
(its text: `deploy/systemd/d1-firmwared-ee-provider@.service` here,
`d1fw_ee_provider::unit::PROVIDER_UNIT`), never spawned by the daemon:

- **The unit.** It runs `EE_PROVIDER_COMMAND` from
  `/etc/d1-firmwared/ee-provider/<side>.env` as the daemon's account,
  requires and follows `d1-firmwared.service`, may write only under
  `/run/d1-firmwared`, and is skipped while the command's program is not
  installed, so it can be enabled before its package is.
- **A provider package** installs its program as
  `/opt/<package>/bin/<program>` (`unit::program_path`) and, when the
  provider reads a configuration of its own, a commented reference under
  `/opt/<package>/share/`; the configuration itself is
  `/etc/d1-firmwared/ee-provider/<side>.toml`. The package writes no
  configuration and enables nothing: which side carries which end effector
  is the robot's commissioning. For a model d1-firmwared knows (its
  `d1fw_backends::ee_packages` table), `d1-firmwared config ee-providers`
  writes the side's `<side>.env` naming the package's program, and the
  daemon's startup notice names the package to install; any other provider
  is named in `<side>.env` by an operator.
- **The command line** every provider package shipped with the robot takes:
  `--side <a|b>`, `--root` and `--prefix` (the daemon's
  `[end_effector] shm_root_path` / `shm_prefix`), `--service` and, for a
  flange device, `--flange` (the services' bases), and `--state-period-ms`;
  a provider that can simulate its device takes `--sim`.
- **Versions.** A provider depends on this SDK by a git dependency pinned to
  a release tag; the iceoryx2 pin (`=0.9.3`) comes with it and must equal
  d1-firmwared's. A release of this repository that changed a layout would
  be a new contract major (spec §6), never a silent change.
- **Restarting a provider does not restart the daemon,** and order does not
  matter: a provider started first retries until the daemon's runtime
  directory exists; a daemon started first reports the side unavailable
  until the provider announces itself.

## Failure modes

| Situation | What the side reports | What a command does |
| --- | --- | --- |
| no provider ever started | `available: false`, "no end-effector provider has announced itself on omakase/ee/b (root …, prefix …)"; the daemon also logs a notice naming the unit (and, for a model it knows, the package) 15 s after it starts | `set`/`joints` 502 `unavailable`; `stop` 502 naming the control topic |
| flange side, provider has not found its device | `available: false`, the provider's reason; `descriptor.node_id: null` | `set`/`joints` 502 `unavailable`; `stop` passes to the provider |
| provider crashed or hung | after the stale threshold: "no heartbeat for N ms" | as above; a `set` in flight answers 502 (went away) or 504 (no answer) |
| provider restarted, same declaration | available again within one heartbeat | normal |
| provider restarted, changed declaration | "differs from the one this daemon accepted …; restart d1-firmwared" | 502 until the daemon restarts |
| provider of another schema version | "its descriptor is malformed: schema_version 2 is not the contract's 1" | 502 |
| provider of another major | the services do not open on the provider's side (iceoryx2 type mismatch); the side reads as never announced | 502 |
| second provider on a bound side | its publisher cannot be created; `serve` fails naming "another provider" | the first provider keeps the side |
| shared-memory root not openable | "the shared-memory services of … cannot be opened: …", retried every second | 502 |
| daemon restarted | the provider keeps serving; the new daemon accepts its declaration | normal within one descriptor period |
| daemon gone | the provider's `daemon_presence(false)`; it holds and starts nothing | — |
| `flange_bus` side, provider not bound by the relay | its `FlangeBus` reports "not admitted"; the relay counts its frames `tx_refused_unbound` | a hand provider does not announce itself until admitted, so the side reads as never announced |
| `flange_bus` side under the soft-kill latch | `status.latched`; motion frames counted `tx_refused_latched` | `set`/`joints` 409 `kill_latched` before the wire; the provider's stop and reads still reach the device |
| `flange_bus` side, channel send failing | `status` `channel_up: false` with the transport's error | the provider's commands fail with the device error |

## Open questions

1. **Group-scoped access.** A provider runs as the daemon's account because
   iceoryx2 0.9.3 segments are owner-only and their mode is not
   configurable (the table under "Ownership and permissions"). When the
   iceoryx2 pin moves to a release with a configurable segment mode, a
   dedicated group holding both the daemon's and a provider's account would
   let a provider run as an account of its own; the contract's layout would
   not change.
2. **A changed declaration needs a daemon restart.** Re-fitting a side at
   runtime would spare a provider's developer the restart; it is a change to
   the daemon, not to the contract.
