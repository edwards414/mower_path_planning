#!/usr/bin/env bash
# What the oracle workflow checks, in the oracle image, from the repository
# root:
#   docker run --rm -v "$PWD":/repo mower-oracle:jazzy tools/oracle/check.sh
# 1. regen.sh regenerates every vector;
# 2. the deterministic ones must come out byte for byte as committed;
# 3. the two node scenarios (tests/data/controller_server.json and
#    navigator.json) record the real controller_server and bt_navigator on
#    the wall clock. Two runs in the same image differ by a few ms of
#    scheduling in a thousand lines, so they are checked the way their Rust
#    tests read them: the port's tests must pass against the fresh
#    recording, which is then put back. A node starved of CPU aborts goals
#    it otherwise reaches (seen on a loaded Mac), so a failing recording is
#    made once more before the check fails.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo"
git config --global --add safe.directory "$repo"
data=src/mower_rs/crates/mower_nav_core/tests/data
scenarios=("$data/controller_server.json" "$data/navigator.json")

tools/oracle/regen.sh

scenario_tests() {
  (cd src/mower_rs && CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-/tmp/oracle-target}" \
    cargo test --locked -q -p mower_nav_core --test controller_server_oracle --test navigator_oracle)
}
if ! scenario_tests; then
  echo "the port disagrees with this recording of the node scenarios; recording them again"
  for c in controller_server navigator; do
    bash src/mower_rs/crates/mower_nav_core/tests/oracle/run_oracle.sh "$c"
  done
  scenario_tests
fi
git checkout -- "${scenarios[@]}"

if [ -n "$(git status --porcelain)" ]; then
  git status --porcelain
  git diff --stat
  git diff | head -200
  echo "regenerating the oracle vectors changed them (diff above)" >&2
  exit 1
fi
echo "every oracle vector reproduced; the port agrees with a fresh recording of the node scenarios"
