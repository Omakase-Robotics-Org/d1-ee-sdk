#!/usr/bin/env bash
# The committed layout files are the frozen version 1 of each contract: their
# SHA-256 must equal the hash the contract crate pins (LAYOUT_SHA256), which
# d1-firmwared and every provider compile against. A layout change is a new
# contract major (spec §6), never an edit of a version-1 file.
set -euo pipefail
cd "$(dirname "$0")/.."
status=0
check() {
  local file=$1 source=$2
  local pinned actual
  pinned=$(sed -n 's/^pub const LAYOUT_SHA256: &str = "\([0-9a-f]\{64\}\)";$/\1/p' "$source")
  actual=$(sha256sum "$file" | cut -d' ' -f1)
  if [[ -z "$pinned" ]]; then
    echo "FAIL $source pins no LAYOUT_SHA256"
    status=1
  elif [[ "$pinned" != "$actual" ]]; then
    echo "FAIL $file sha256 $actual, $source pins $pinned"
    status=1
  else
    echo "ok   $file $actual"
  fi
}
check spec/end_effector.shm/1.layout.json crates/d1fw-ee-shm-contract/src/layout.rs
check spec/flange_bus.shm/1.layout.json crates/d1fw-ee-shm-contract/src/flange/layout.rs
# The arm contracts pin their four layouts in one module, one constant each.
check_arm() {
  local file=$1 name=$2 source=crates/d1fw-arm-shm-contract/src/layout.rs
  local pinned actual
  pinned=$(sed -n "/^pub const ${name}_LAYOUT_SHA256: &str =\$/{n;s/^ *\"\([0-9a-f]\{64\}\)\";\$/\1/p}" "$source")
  actual=$(sha256sum "$file" | cut -d' ' -f1)
  if [[ -z "$pinned" ]]; then
    echo "FAIL $source pins no ${name}_LAYOUT_SHA256"
    status=1
  elif [[ "$pinned" != "$actual" ]]; then
    echo "FAIL $file sha256 $actual, $source pins $pinned"
    status=1
  else
    echo "ok   $file $actual"
  fi
}
check_arm spec/arm_feedback.shm/1.layout.json FEEDBACK
check_arm spec/arm_status.shm/1.layout.json STATUS
check_arm spec/arm_events.shm/1.layout.json EVENTS
check_arm spec/arm_command.shm/1.layout.json COMMAND
exit "$status"
