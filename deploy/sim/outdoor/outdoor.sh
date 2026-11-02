#!/bin/bash
# The outdoor stress harness (I1): the robot's production stack on mower_sim,
# throttled to the robot's 408-1992 MHz load and behind an LTE-like link.
# Development only: no robot, no real pairing, no fleet backend.
#
#   export MOWER_PAIR_FILE=$PWD/tools/ab/fixtures/pair.json   # required
#   OD=od1 deploy/sim/outdoor/outdoor.sh calibrate            # once per machine
#   OD=od1 deploy/sim/outdoor/outdoor.sh up T50 typ [navcore]
#   OD=od1 deploy/sim/outdoor/outdoor.sh up-cold T20          # throttled boot
#   OD=od1 deploy/sim/outdoor/outdoor.sh net bad | cpu T20 | blk | unblk | ipchange
#   OD=od1 deploy/sim/outdoor/outdoor.sh collect DIR
#   OD=od1 deploy/sim/outdoor/outdoor.sh down
#
# OD is the compose project (one stack each; several can run side by side,
# OUTDOOR_CPUSET=4-7 for a second one). CPU levels: T100 / T50 / T20 are the
# robot at 1992 / 1008 / 408 MHz (PLAN.md), X = D / L cores from
# `calibrate`'s calibration.json; FULL is 4 unthrottled cores. NET: clean,
# good, typ, bad (netem.sh). navcore = RUST_NAV_CORE=true. MOWER_RS_TARGET
# = a directory with a Linux mower_rsd to run instead of the image's
# (../docker-compose.sim.local.yaml); MOWER_SIM_BIN another mower_sim.
# Guard overrides: MANUAL_GUARD_TIMEOUT_S, FINAL_GUARD_TIMEOUT_S. SIM_SEED,
# SIM_ARGS go to mower_sim.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$here/../../.." && pwd)
OD=${OD:-od1}
RUNS=${OUTDOOR_RUNS:-$HOME/work/outdoor-runs}
CALIB=${OUTDOOR_CALIB:-$RUNS/calibration/calibration.json}
export E2V_SCRIPTS=${E2V_SCRIPTS:-$HOME/work/e2-verify/scripts}
# the stack starts unthrottled (FULL) and `cpu` sets its level live (docker update)
export OUTDOOR_CPU_QUOTA=${OUTDOOR_CPU_QUOTA:-40000}
log() { echo "[outdoor $OD] $*" >&2; }
die() { log "$*"; exit 1; }

# --- never the real robot ------------------------------------------------------
[ -n "${MOWER_PAIR_FILE:-}" ] || die "MOWER_PAIR_FILE is unset: export MOWER_PAIR_FILE=$repo/tools/ab/fixtures/pair.json"
python3 "$repo/tools/outdoor/safety.py" >/dev/null || exit 3
python3 - "$MOWER_PAIR_FILE" "$repo/tools/ab/fixtures/identity.json" <<'EOF' || die "MOWER_PAIR_FILE does not pair with the sim's identity"
import json, sys
p, i = (json.load(open(f)) for f in sys.argv[1:3])
sys.exit(0 if (p["robot_id"], p["secret"]) == (i["robot_id"], i["secret"]) else 1)
EOF
case "${MOWER_BACKEND_URL:-}" in "" | http://127.0.0.1* | http://localhost*) ;; *) die "MOWER_BACKEND_URL is not local" ;; esac
export MOWER_PAIR_FILE MOWER_BACKEND_URL=

compose() {
  local files=(-f "$repo/deploy/sim/docker-compose.sim.yaml" -f "$here/docker-compose.outdoor.yaml")
  [ -n "${MOWER_RS_TARGET:-}" ] && files+=(-f "$repo/deploy/sim/docker-compose.sim.local.yaml")
  docker compose -p "$OD" "${files[@]}" "$@"
}
cid() { docker compose -p "$OD" -f "$repo/deploy/sim/docker-compose.sim.yaml" -f "$here/docker-compose.outdoor.yaml" ps -q "$1"; }

quota_of() {  # CPU level -> cpu_quota in us per 10 ms period
  case "$1" in
    FULL) echo 40000 ;;
    T100|T50|T20)
      [ -f "$CALIB" ] || die "no $CALIB: run calibrate first"
      python3 -c "import json,sys; c=json.load(open('$CALIB')); print(int(round(c['quota_us']['$1'])))" ;;
    *) die "CPU level $1: FULL, T100, T50 or T20" ;;
  esac
}

wait_ready() {  # the app contract through the gate, unshaped, up to $1 s
  local deadline=$((SECONDS + ${1:-240})) base=()
  [ -f "$repo/tools/outdoor/fixtures/sim_app_baseline.json" ] && base=(--baseline /to/fixtures/sim_app_baseline.json)
  while [ $SECONDS -lt $deadline ]; do
    if docker exec "$(cid client)" python3 /rt/probe_app.py ws://sim:9090 8 ${base[@]+"${base[@]}"} >/tmp/outdoor-ready-$OD.txt 2>&1; then
      log "ready after $SECONDS s (probe_app OK)"; return 0
    fi
    sleep 5
  done
  tail -5 /tmp/outdoor-ready-$OD.txt >&2
  die "not ready"
}

KEEP=$RUNS/cpukeeper
keep() {  # the CPU keeper puts "$2 10000" back into container $1's cgroup
  mkdir -p "$KEEP/targets"
  echo "$2 10000" >"$KEEP/targets/$1"
  if [ -z "$(docker ps -q -f name=^outdoor-cpukeeper$)" ]; then
    docker rm -f outdoor-cpukeeper >/dev/null 2>&1 || true
    docker run -d --name outdoor-cpukeeper --privileged --cgroupns=host -v /sys/fs/cgroup:/host_cg \
      -v "$KEEP/targets:/targets:ro" -v "$here/cpukeeper.sh:/cpukeeper.sh:ro" --entrypoint sh \
      mower-outdoor-netem:local /cpukeeper.sh >/dev/null
  fi
}

net() {  # profile on both ends: sim eth0 = uplink, client eth0 = downlink
  local p=$1
  [ "$p" = clean ] && p=clear
  docker exec "$(cid netem_sim)" sh /netem.sh "$p" up >/dev/null
  docker exec "$(cid client)" sh /netem.sh "$p" down >/dev/null
  log "net $1"
}

cmd=${1:-}
shift || true
case "$cmd" in
  up)
    cpu=${1:?CPU level}; netp=${2:-clean}; nav=${3:-}
    [ "$nav" = navcore ] && export RUST_NAV_CORE=true
    df_free=$(df -g "$HOME" | awk 'NR==2{print $4}')
    [ "$df_free" -ge 16 ] || die "only ${df_free} GB free (floor 16)"
    docker image inspect mower-outdoor-netem:local >/dev/null 2>&1 \
      || docker build -q -t mower-outdoor-netem:local -f "$here/netem.Dockerfile" "$here" >/dev/null
    compose up -d --quiet-pull sim observer netem_sim client >&2
    docker exec -d "$(cid observer)" python3 /to/vmload.py --out /tmp/obs/vmload.jsonl
    log "up: image $(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$(cid sim)" | grep '^MOWER_GIT_SHA=' | cut -c15-21)," \
        "RUST_NAV_CORE=${RUST_NAV_CORE:-false} MOWER_RS_TARGET=${MOWER_RS_TARGET:-image} guards=${MANUAL_GUARD_TIMEOUT_S:-default}/${FINAL_GUARD_TIMEOUT_S:-default}"
    wait_ready 300
    "$0" cpu "$cpu"
    net "$netp"
    ;;
  up-cold)  # throttled from the first instruction (cold starts, rows 3/4); no wait
    cpu=${1:?CPU level}; nav=${2:-}
    [ "$nav" = navcore ] && export RUST_NAV_CORE=true
    OUTDOOR_CPU_QUOTA=$(quota_of "$cpu"); export OUTDOOR_CPU_QUOTA
    compose up -d --quiet-pull sim observer netem_sim client >&2
    docker exec -d "$(cid observer)" python3 /to/vmload.py --out /tmp/obs/vmload.jsonl
    keep "$(docker inspect -f '{{.Id}}' "$(cid sim)")" "$OUTDOOR_CPU_QUOTA"
    got=$(docker exec "$(cid sim)" cat /sys/fs/cgroup/cpu.max)
    [ "$got" = "$OUTDOOR_CPU_QUOTA 10000" ] || die "cpu.max is '$got'"
    log "up-cold $cpu: quota $OUTDOOR_CPU_QUOTA us / 10 ms, SIM_ARGS=${SIM_ARGS:-}" ;;
  cpu)
    # docker update alone is not enough: on Docker Desktop it rewrites the
    # cgroup's period (cpu.max came out "22120 40000" for 5530/10000), so
    # the quota would come as 40-80 ms freezes instead of a slow clock. The
    # value goes straight into the container's cgroup as well, and is
    # checked from inside.
    q=$(quota_of "${1:?level}")
    sim=$(cid sim); id=$(docker inspect -f '{{.Id}}' "$sim")
    docker update --cpu-period 10000 --cpu-quota "$q" "$sim" >/dev/null
    keep "$id" "$q"
    sleep 1.2
    got=$(docker exec "$sim" cat /sys/fs/cgroup/cpu.max)
    [ "$got" = "$q 10000" ] || die "cpu.max is '$got', wanted '$q 10000'"
    log "cpu $1: cpu.max $got = $(python3 -c "print($q/10000)") cores on cpuset ${OUTDOOR_CPUSET:-0-3}"
    ;;
  net) net "${1:?profile}" ;;
  blk)    # both ends 100 % loss; prints the VM clock at the client's switch
    docker exec "$(cid netem_sim)" sh /netem.sh blk up >/dev/null &
    docker exec "$(cid client)" sh -c 'date +%s.%N; sh /netem.sh blk down >/dev/null'
    wait ;;
  unblk) net "${1:-clean}" ;;
  ipchange)  # the client gets a new address, as an LTE modem re-attaching: a
    # placeholder takes the old one (Docker hands out the lowest free address)
    netw=$(docker inspect -f '{{range $k, $v := .NetworkSettings.Networks}}{{$k}}{{end}}' "$(cid client)")
    old=$(docker inspect -f "{{(index .NetworkSettings.Networks \"$netw\").IPAddress}}" "$(cid client)")
    docker network disconnect "$netw" "$(cid client)"
    docker rm -f "$OD-ipholder" >/dev/null 2>&1 || true
    docker run -d --rm --name "$OD-ipholder" --network "$netw" --entrypoint sleep mower-outdoor-netem:local 600 >/dev/null
    docker network connect "$netw" "$(cid client)"
    new=$(docker inspect -f "{{(index .NetworkSettings.Networks \"$netw\").IPAddress}}" "$(cid client)")
    docker rm -f "$OD-ipholder" >/dev/null 2>&1 || true
    docker exec "$(cid client)" date +%s.%N
    log "client $old -> $new" ;;
  ready) wait_ready "${1:-240}" ;;
  sh) docker exec -i "$(cid "${1:?service}")" "${@:2}" ;;
  collect)
    out=${1:?dir}; mkdir -p "$out"
    docker logs -t "$(cid sim)" >"$out/stack.log" 2>&1 || true
    docker cp "$(cid sim):/tmp/outdoor/." "$out/sim/" >/dev/null 2>&1 || true
    docker cp "$(cid observer):/tmp/obs/." "$out/obs/" >/dev/null 2>&1 || true
    docker cp "$(cid client):/tmp/client/." "$out/client/" >/dev/null 2>&1 || true
    docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$(cid sim)" | grep -E '^(MOWER_GIT_SHA|RUST_|SIM_|MANUAL_|FINAL_)' >"$out/env.txt" || true
    log "collected into $out" ;;
  down)
    s=$(cid sim 2>/dev/null || true)
    [ -n "$s" ] && rm -f "$KEEP/targets/$(docker inspect -f '{{.Id}}' "$s")"
    compose --profile camera down -v -t 5 >/dev/null 2>&1 || true
    log "down (containers, network and volumes of project $OD removed)" ;;
  calibrate)
    # D: the stack's idle CPU in cores on this machine (cpu_window.py, 2 x 60 s,
    # unthrottled), split into the stack and mower_sim (the "hardware", which
    # the robot does not run). X = D_stack / L + D_sim, L = 0.30 / 0.51 / 1.25.
    out=$RUNS/calibration; mkdir -p "$out"
    "$0" up FULL clean
    sleep 20
    for i in 1 2; do
      docker exec "$(cid observer)" python3 /v/cpu_window.py --seconds 60 --out /tmp/cpu_idle_$i.json --label idle$i >&2
      docker cp "$(cid observer):/tmp/cpu_idle_$i.json" "$out/cpu_idle_$i.json" >/dev/null
    done
    python3 "$repo/tools/outdoor/calibrate.py" "$out/cpu_idle_1.json" "$out/cpu_idle_2.json" >"$CALIB"
    cat "$CALIB"
    "$0" collect "$out/run"
    "$0" down ;;
  *) sed -n 2,22p "$0"; exit 2 ;;
esac
