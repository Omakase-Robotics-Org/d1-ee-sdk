#!/usr/bin/env bash
# The provider template builds, is lint-clean and conforms against THIS
# checkout of the SDK (its git dependency on a release tag is patched to the
# workspace's crates), and its manifest agrees with the descriptor its
# conformance run accepted. A template that only compiles against an old tag
# would hide an SDK change that breaks every new provider.
set -euo pipefail
cd "$(dirname "$0")/../template"
sdk=https://github.com/Omakase-Robotics-Org/d1-ee-sdk
patch=(
  --config "patch.\"$sdk\".d1fw-ee-provider.path=\"../crates/d1fw-ee-provider\""
  --config "patch.\"$sdk\".d1fw-ee-harness.path=\"../crates/d1fw-ee-harness\""
)
# The SDK's target directory: the template shares every dependency with it.
export CARGO_TARGET_DIR="$PWD/../target/template"
rm -f target/conformance/descriptor-a.json
cargo fmt --all -- --check
cargo clippy "${patch[@]}" --workspace --all-targets -- -D warnings
cargo test "${patch[@]}" --workspace
cargo run --manifest-path ../Cargo.toml --locked --quiet -p d1fw-ee-manifest -- \
  check manifest.json --descriptor target/conformance/descriptor-a.json
