#!/usr/bin/env bash
# What the oracle workflow checks, in the oracle image, from the repository
# root:
#   docker run --rm -v "$PWD":/repo mower-oracle:jazzy tools/oracle/check.sh
# 1. regen.sh regenerates every vector;
# 2. the deterministic ones must come out byte for byte as committed;
# 3. the two node scenarios (tests/data/controller_server.json and
#    navigator.json) record the real controller_server and bt_navigator on
#    the wall clock, so no two runs agree line for line, and the times and
#    plan / follow_path counts of the wall scenario vary by several rounds
#    on nav2 itself. compare_scenarios.py holds the fresh recording to the
#    committed one only where a run does not vary: the goals' outcomes and
#    error codes, and where the robot ends. A node starved of CPU aborts
#    goals it otherwise reaches (seen on a loaded Mac), so a recording that
#    fails that is made once more before the check fails. The port's tests
#    then run against the fresh recording as a report only: they hold the
#    port to the committed recording, with its counts, in build.yml's
#    mower_rs tests. The fresh recording is then put back.
set -euo pipefail
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo"
git config --global --add safe.directory "$repo"
data=src/mower_rs/crates/mower_nav_core/tests/data
scenarios=("$data/controller_server.json" "$data/navigator.json")

tools/oracle/regen.sh

committed="$(mktemp -d)"
for c in controller_server navigator; do
  git show "HEAD:$data/$c.json" > "$committed/$c.json"
done
if ! python3 tools/oracle/compare_scenarios.py "$committed" "$data"; then
  echo "this recording of the node scenarios lost an outcome of the committed one; recording them again"
  for c in controller_server navigator; do
    bash src/mower_rs/crates/mower_nav_core/tests/oracle/run_oracle.sh "$c"
  done
  python3 tools/oracle/compare_scenarios.py "$committed" "$data"
fi

# report only (see 3.): the port against this recording, counts included
if ! (cd src/mower_rs && CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-/tmp/oracle-target}" \
    cargo test --locked -q -p mower_nav_core --test controller_server_oracle --test navigator_oracle); then
  note="the port's scenario tests disagree with this recording of the real nodes (report only: they hold the port to the committed recording in mower_rs tests)"
  echo "$note"
  if [ -n "${GITHUB_ACTIONS:-}" ]; then echo "::warning title=Node scenarios::$note"; fi
fi
git checkout -- "${scenarios[@]}"

if [ -n "$(git status --porcelain)" ]; then
  git status --porcelain
  git diff --stat
  git diff | head -200
  echo "regenerating the oracle vectors changed them (diff above)" >&2
  exit 1
fi
echo "every deterministic oracle vector reproduced, and every outcome of the node scenarios"
