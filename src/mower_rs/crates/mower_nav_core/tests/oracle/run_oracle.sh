#!/usr/bin/env bash
# Regenerate tests/data/<component>.json from the real nav2 (Jazzy 1.3.13).
#
# Run it in the oracle image (tools/oracle/), from the repository root:
#   docker buildx build -f tools/oracle/Dockerfile -t mower-oracle:jazzy .
#   docker run --rm -v "$PWD":/repo mower-oracle:jazzy \
#       bash src/mower_rs/crates/mower_nav_core/tests/oracle/run_oracle.sh costmap
# (tools/oracle/regen.sh runs every oracle of the repository.)
#
# With no argument every component directory under tests/oracle is run.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
here="$(cd "$(dirname "$0")" && pwd)"
components=("$@")
if [ ${#components[@]} -eq 0 ]; then
  for d in "$here"/*/; do components+=("$(basename "$d")"); done
fi
mkdir -p "$here/../data"
for c in "${components[@]}"; do
  if [ -f "$here/$c/scenarios.py" ]; then
    # the real node, black box, on the wall clock (about 1.5 min)
    python3 "$here/$c/scenarios.py" "$here/../../../../../mower_nav2/config/nav2_no_map_params.yaml" \
      "$here/$c/scenarios.json" "$here/../data/$c.json"
  else
    build="/tmp/nav-oracle-build/$c"
    cmake -S "$here/$c" -B "$build" -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build "$build" -j4 >/dev/null
    "$build/oracle" "$here/../data/$c.json"
  fi
  ls -l "$here/../data/$c.json"
done
