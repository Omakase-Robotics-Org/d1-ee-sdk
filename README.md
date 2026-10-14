# d1-ee-sdk

The end-effector SDK of the Omakase D1: the shared-memory contracts through
which `d1-firmwared` (the robot's daemon) serves an end effector driven by a
process of its own — a **provider** — and the Rust crates a provider is built
from. Every D1 end effector is a provider, the stock gripper included, and
every provider is a package of its own built from a repository of its own on
this SDK; the daemon builds and ships none of them.

| Path | Holds |
| --- | --- |
| `spec/end_effector.shm/1.md`, `1.layout.json` | the end-effector contract: services, messages, byte layout, the rules a provider must follow, what the daemon does |
| `spec/flange_bus.shm/1.md`, `1.layout.json` | the flange relay: the arm's end-module CAN channel, relayed by the daemon to a side's provider |
| `crates/d1fw-ee-shm-contract` | both contracts in Rust: names, messages, validation, iceoryx2 settings, the layout documents and their SHA-256 pins |
| `crates/d1fw-canfd` | the CAN FD frame, the frame kinds (`motion`, `read`, `stop`) and the synchronous bus seam a device driver is written against |
| `crates/d1fw-ee-provider` | the provider SDK: `Provider`, `serve`, `FlangeBus`, the unit and packaging conventions (`unit`), `TestProvider` (feature `test-support`) |
| `crates/d1fw-ee-hand` | what a multi-finger hand provider is built from: the serialized link, the `hand.state/1` detail and its JSON Schema, the relayed channel as a bus, a simulated device, the provider process |
| `crates/d1fw-ee-harness` | the daemon's side of both contracts for a provider's tests (`Host`), frame recording and golden comparison, the lifecycle conformance run, and `d1-ee-test-provider` |
| `deploy/systemd/d1-firmwared-ee-provider@.service` | the template unit that runs a side's provider (installed by the d1-firmwared package) |
| `docs/end-effector-shm-interface.md` | the provider author's guide: why the contracts are shaped as they are, writing, testing, packaging and running a provider |

## Getting started

The D1's daemon, `d1-firmwared`, ships installed on the robot; its source is
not public, and a provider does not need it. Everything a provider author
needs is in this repository: the two contracts under `spec/`, the Rust crates
under `crates/`, the author's guide in `docs/end-effector-shm-interface.md`,
and `d1fw-ee-harness`, which plays the daemon's side of both contracts so a
provider is built and tested on any Linux machine without a robot.

```sh
git clone https://github.com/Omakase-Robotics-Org/d1-ee-sdk
cd d1-ee-sdk
cargo test --workspace --all-features --locked
```

Then:

1. Read `docs/end-effector-shm-interface.md`, and `spec/end_effector.shm/1.md`
   for the rules a provider must follow (`spec/flange_bus.shm/1.md` too for a
   device on the arm's flange CAN channel).
2. Start a provider crate that depends on this repository at a release tag
   (see [Depending on the SDK](#depending-on-the-sdk)), implement
   `d1fw_ee_provider::Provider` and call `d1fw_ee_provider::serve`.
   [d1-ee-damiao](https://github.com/Omakase-Robotics-Org/d1-ee-damiao), the
   provider of the D1's stock gripper, is a complete example of a flange
   device provider with golden-frame and conformance tests.
3. Test it against `d1fw_ee_harness::Host` and `conformance::lifecycle`.
   Running it on a robot needs the robot's `d1-firmwared` configuration to
   declare the side. Both are steps of the next section.

## Attaching an end effector to a D1

1. **Pick the side and the model.** Each arm side (`a` is the physical left
   arm, `b` the right) carries one end effector. The model is the provider's
   own id (`<maker>/<model>`), announced in its declaration.
2. **Write the provider.** In Rust, implement `d1fw_ee_provider::Provider`
   (`descriptor`, `set`, `joints`, `read`, `stop`; optionally
   `clear_fault`, `registers`, `ticks`) and call `d1fw_ee_provider::serve`.
   The descriptor declares the joints, the capabilities, the command rate,
   the detail schema id, the tool the daemon registers on the arm and the
   guard envelope id. A device on the arm's flange CAN channel opens
   `FlangeBus` and labels each frame `motion`, `read` or `stop`; any other
   device is opened by the provider itself. A hand built on `d1fw-ee-hand`
   implements its wire link and reuses the rest. In another language,
   implement the contract on the iceoryx2 0.9.3 bindings: the reference
   provider in Python is `manipulation_kit.end_effectors.shm_provider` in the
   public manipulation-kit repository (pull request
   Omakase-Robotics-Org/manipulation-kit#48), and a ROS 2 end effector
   attaches through the bridge `omakase_ee_bridge` of the D1's ROS 2 stack
   (not a public repository), itself a provider of this contract.
3. **Test it without a robot.** Stand the daemon's side up with
   `d1fw_ee_harness::Host` (over a simulated device for a flange provider),
   hold the wire behaviour to recorded frames with `golden`, and run
   `conformance::lifecycle` against the provider binary. Then run it once
   against `d1-firmwared run --backend sim` with the side declared
   `driver = "shm"`.
4. **Package it.** Install the program as `/opt/<package>/bin/<program>`
   (`d1fw_ee_provider::unit::program_path`), and a commented reference of
   its own configuration, if any, under `/opt/<package>/share/`. The package
   writes no configuration and enables nothing.
5. **Declare the side** in the robot's d1-firmwared configuration:

   ```toml
   [end_effector.a]
   driver = "shm"
   model = "acme/g2"      # optional pin: the daemon refuses another model
   flange_bus = true      # only for a device on the arm's flange CAN channel
   ```

6. **Run it.** Write `/etc/d1-firmwared/ee-provider/<side>.env`:

   ```sh
   EE_PROVIDER_COMMAND=/opt/<package>/bin/<program> --side a
   ```

   and `systemctl enable --now d1-firmwared-ee-provider@a`. The unit runs the
   command as the daemon's account, starts and stops with the daemon, and is
   skipped while the program is not installed. For a model d1-firmwared knows
   (the stock gripper, the LinkerHand O30, the Leadshine DH116S),
   `d1-firmwared config ee-providers` writes the `.env` file. The side
   appears in `GET /v1/end_effectors` once the declaration is accepted.

The provider packages built on this SDK:

| Repository | Package | Model |
| --- | --- | --- |
| `d1-ee-damiao` | `d1-ee-damiao` | `damiao-v1.1`, `damiao-v1.2` (the D1's stock parallel gripper) |
| `d1-ee-linkerbot` | `d1-ee-linkerbot` | `linkerbot/o30` (LinkerHand O30) |
| `d1-ee-leadshine` | `d1-ee-leadshine` | `leadshine/dh116s` (Leadshine DH116S) |

`d1-ee-damiao` is public; the other provider repositories are not.

## Depending on the SDK

A provider takes the crates as a git dependency pinned to a release tag:

```toml
[workspace.dependencies]
d1fw-ee-provider = { git = "https://github.com/Omakase-Robotics-Org/d1-ee-sdk", tag = "v0.1.0" }
d1fw-ee-harness = { git = "https://github.com/Omakase-Robotics-Org/d1-ee-sdk", tag = "v0.1.0" }
iceoryx2 = "=0.9.3"
```

The repository is public: cargo fetches it over HTTPS without credentials,
on a developer's machine and in CI alike. The iceoryx2 pin must equal the
SDK's and d1-firmwared's: iceoryx2's shared-memory layout is not guaranteed
compatible across releases.

## Versions

- **The contracts.** Version 1 of each is frozen. A change to a layout, a
  code, a bit or a service setting is a new major (`end_effector.shm/2`,
  type names `omakase_ee_v2_*`, a sibling `2.layout.json`), never an edit of
  a version-1 file (spec §6). Each `1.layout.json` is pinned by SHA-256 in
  the contract crate (`LAYOUT_SHA256`), in `tools/check-layout-sha.sh`
  (CI) and in d1-firmwared's own tests, so a change fails on every side.
- **The crates.** `vMAJOR.MINOR.PATCH` tags, `0.x` SemVer: a breaking API
  change or a new feature raises the minor, a fix the patch. A provider and
  d1-firmwared each pin one tag; a new tag that keeps the contract major
  needs no change on the other side. A tag is cut only from a commit the CI
  gate passed.
- **A detail** (`detail.schema`, `<id>/<major>`) is versioned by its own
  schema id, owned by the provider that publishes it; this repository does
  not version it.

## Developing

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features --locked -- -D warnings
cargo test --workspace --all-features --locked
tools/check-layout-sha.sh
```

The CI workflow (`.github/workflows/ci.yml`) runs the same four steps on
every pull request, on `main` and on a `v*` tag.
