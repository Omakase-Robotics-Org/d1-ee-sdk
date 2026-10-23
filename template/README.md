# d1-ee-acme (provider template)

A starting point for a D1 end-effector provider: a Rust crate that serves a
one-joint gripper to `d1-firmwared` over `end_effector.shm/1` with the
d1-ee-sdk, behind a fake device transport, with its conformance test, its
Debian packaging, its release workflow and its manifest
(`omakase-ee-provider/1`). The developer path from here to a robot is
`docs/third-party-providers.md` in d1-ee-sdk.

## Start

Copy the directory out of the SDK and rename the placeholders:

```sh
cp -r d1-ee-sdk/template d1-ee-yourvendor
cd d1-ee-yourvendor
git init
# acme -> your vendor, g1 -> your model, ACME/G1 in prose likewise.
grep -rl --exclude-dir=target -e acme -e ACME . | xargs sed -i \
  -e 's/d1-ee-acme/d1-ee-yourvendor/g' -e 's/d1_ee_acme/d1_ee_yourvendor/g' \
  -e 's/acme\/g1/yourvendor\/yourmodel/g' -e 's/acme-g1/yourvendor-yourmodel/g' \
  -e 's/acme\.g1/yourvendor.yourmodel/g' -e 's/AcmeProvider/YourProvider/g' \
  -e 's/ACME G1/Your Model/g' -e 's/acme-robotics/your-org/g' -e 's/ACME/Your Vendor/g'
mv crates/d1-ee-acme-provider crates/d1-ee-yourvendor-provider
mv deploy/d1-ee-acme.toml.example deploy/d1-ee-yourvendor.toml.example
cargo test --workspace --all-features
```

Commit the `Cargo.lock` the first build writes.

| Name | In the template | What it is |
| --- | --- | --- |
| package | `d1-ee-acme` | the Debian package; installs `/opt/<package>/bin/<program>` |
| program | `d1-ee-acme-provider` | the binary and its crate |
| model | `acme/g1` | the descriptor's `model`, `<maker>/<model>` |
| driver | `acme-g1` | the descriptor's `driver` |
| id | `d1-firmware.end-effector.acme.g1` | the manifest's `id`, `d1-firmware.end-effector.<vendor>.<model>`, lowercase: the name a fit document references the provider by, with its `version` |

## Layout

| Path | Holds |
| --- | --- |
| `crates/d1-ee-acme-provider/src/lib.rs` | the descriptor and the `Provider` implementation over a `Transport` |
| `crates/d1-ee-acme-provider/src/transport.rs` | the device seam and `FakeTransport` |
| `crates/d1-ee-acme-provider/src/config.rs` | the per-side configuration file (`/etc/d1-ee-acme/<side>.toml`) |
| `crates/d1-ee-acme-provider/src/app.rs` | the command line and the service until SIGTERM |
| `crates/d1-ee-acme-provider/tests/conformance.rs` | `d1fw_ee_harness::conformance::lifecycle` against the process with `--fake`; writes the accepted descriptor to `target/conformance/descriptor-a.json` |
| `crates/d1-ee-acme-provider/tests/packaging.rs` | the program path, the `.env` example and the manifest's launch command agree |
| `deploy/` | the commented configuration and `.env` examples the package installs under `/opt/<package>/share/` |
| `manifest.json` | the release's manifest (`omakase-ee-provider/1`); the release workflow fills `conformance.run_url` with its own run |
| `.github/workflows/ci.yml` | fmt, clippy, tests, the manifest check against the conformance descriptor, the package build |
| `.github/workflows/release.yml` | on a `v*` tag: the arm64 package, the conformance run, the release's manifest, kept as the run's artifacts |

## Make it drive a device

1. Implement `Transport` over the device's bus in `transport.rs`: the arm's
   flange channel (`d1fw_ee_provider::FlangeBus`, with the side declared
   `flange_bus = true`), a SocketCAN interface, a serial port or a socket.
   Open it in `app.rs` (`open_device`), which refuses to start without
   `--fake` until then.
2. Declare what the device is in `descriptor()`: its joints with their unit
   and limits, its capabilities, its command ceiling, its tool (mass, centre
   of mass, inertia, tool centre point) and its guard envelope id. The SDK
   refuses a command the descriptor does not admit before it reaches the
   provider.
   To take a client's grip force (`force_percent`, `0` gentlest … `100`
   strongest over the device's own range), list the key `force_percent` in
   `set_extra` (`d1fw_ee_provider::FORCE_PERCENT`), read it in `set` with
   `d1fw_ee_provider::force_percent`, map it with `ForceRange::at_percent`,
   and publish the `GripForce` report at `detail.data.grip_force` of every
   state; a device without an adjustable force declares it too and reports
   `fixed`. The convention is `spec/end_effector.shm/1.md` §9. Without the
   key the daemon drops a client's `force_percent` and reports the side
   `not_taken`.
3. Describe every configuration key in `manifest.json`'s `config_schema`
   (JSON Schema draft 2020-12 of the TOML file read as JSON). A provider
   that reads no configuration file removes `launch.config_dir`,
   `launch.config_file` and `config_schema` together (and keeps
   `calibration.required` false).
4. Keep `manifest.json`'s `models` equal to what the descriptor announces:
   `model`, `driver`, the joint count, the links the device can be reached
   by, and the postures (`open`, `closed`, and any other such as `pinch`),
   one fraction of each joint's range per joint in descriptor order. CI
   holds the manifest to the descriptor the conformance run accepted.

## Test

```sh
cargo test --workspace --all-features    # conformance included, no robot needed
d1fw-ee-manifest check manifest.json --descriptor target/conformance/descriptor-a.json
```

`d1fw-ee-manifest` is installed from the SDK:
`cargo install --locked --git https://github.com/Omakase-Robotics-Org/d1-ee-sdk --tag <tag> d1fw-ee-manifest`,
where `<tag>` is the first d1-ee-sdk release that carries the crate (the
workflows name it in `D1FW_EE_MANIFEST_TAG`).

## Release

Raise `[workspace.package] version` and `manifest.json`'s `version` together,
merge, and push the tag `v<version>`. The release workflow builds the arm64
package in `ubuntu:22.04` (the robots' glibc), runs the conformance test,
writes `dist/manifest.json` with the run's URL as `conformance.run_url`,
checks it against the conformance descriptor, and keeps the `.deb`, the
manifest and their `SHA256SUMS` as the run's artifacts. A robot installs the
`.deb` with `dpkg -i` (or from your own apt source) and applies the manifest
locally; nothing is published to an Omakase service
(`docs/third-party-providers.md` in d1-ee-sdk).
