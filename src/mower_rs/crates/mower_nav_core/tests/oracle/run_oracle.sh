#!/usr/bin/env bash
# Regenerate tests/data/<component>.json from the real nav2 (Jazzy 1.3.13).
#
#   docker build -t mower-nav2-oracle:jazzy - < tests/oracle/Dockerfile
#   docker run -d --name rf-d-oracle -v <repo>:/repo mower-nav2-oracle:jazzy sleep infinity
#   docker exec rf-d-oracle bash /repo/src/mower_rs/crates/mower_nav_core/tests/oracle/run_oracle.sh costmap
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
  build="/tmp/nav-oracle-build/$c"
  cmake -S "$here/$c" -B "$build" -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build "$build" -j4 >/dev/null
  "$build/oracle" "$here/../data/$c.json"
  ls -l "$here/../data/$c.json"
done
