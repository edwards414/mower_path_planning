#!/bin/bash
# Test switch for the ROS-free branch image on the robot (procedure: README.md
# next to this file).
#
#   [SWITCH_EXPECT_SHA=<sha>] switch.sh <IMAGE_TAG> <NAV_COMPOSITION> <RUST_DAEMON> <branch-compose-file> [KEY=VAL ...]
#   switch.sh restore   the pre-test .env and compose back, verified, then the backups go
#   switch.sh status    what the running container was started with and which
#                       base / localization processes it runs; changes nothing
#
# A switch is absolute, never cumulative: .env is rebuilt from the pre-test
# backup each time and NAV_COMPOSITION, RUST_DAEMON, RUST_BASE and
# RUST_LOCALIZE are always written (the last two false unless given as
# KEY=VAL), so a flag from an earlier call cannot stick. The branch compose is
# required because main's does not pass those four through and compose
# silently ignores .env keys it does not reference. After `up` the container's
# launch arguments are checked and the script waits until the expected base
# and localization processes are the ones running (docker top; never the ros2
# CLI on the robot, it causes controller overruns), with the same PIDs for
# SWITCH_STABLE_S and no base/localization restart in the log. Any mismatch
# exits 1; from `compose up` on, a failure or an interrupt also stops
# lawan_node (fail-closed).
#
# RUST_BASE / RUST_LOCALIZE / RUST_DAEMON need SWITCH_EXPECT_SHA=<the commit
# the image was built from>, checked against the image's MOWER_GIT_SHA before
# anything is changed, and RUST_BASE refuses a driver binary without the
# feedback-loss arm latch: an old test image must not pass for the reviewed one.
#
# The test config outlives a reboot or power-off (mower.service brings up
# whatever .env and the compose say) until `restore` runs, so keep this script
# somewhere that survives a boot, not /tmp.
set -euo pipefail

MOWER_DIR=${MOWER_DIR:-/opt/mower}
REPO=ghcr.io/edwards414/mower_path_planning
WAIT_S=${SWITCH_WAIT_S:-180}
# ...then the drivers must keep their PIDs, with no restart logged, this long.
STABLE_S=${SWITCH_STABLE_S:-30}
POLL_S=${SWITCH_POLL_S:-5}
EXPECT_SHA=${SWITCH_EXPECT_SHA:-}
ENV_BAK=.env.bak-rosfree-test
CF_BAK=docker-compose.yaml.bak-rosfree-test
# Present exactly while a test is active; lists the .env keys the test wrote.
MARKER=ROSFREE_TEST_ACTIVE
# shellcheck disable=SC2016  # the literal compose text, not an expansion
PASSTHROUGH=('nav_composition:=${NAV_COMPOSITION' 'rust_daemon:=${RUST_DAEMON'
             'rust_base:=${RUST_BASE' 'rust_localize:=${RUST_LOCALIZE')
DRIVERS='ros2_control_node|mower_base|mower_localize|mower_rsd|ekf_node|navsat_transform_node'
# A launch respawn, or a mower_rsd base/localize module failing ("[mower_rsd]
# base/mower_base: <why>; restarting in 2.0 s") and coming back in the same PID.
RESTART_RE="\\[mower_rsd\\] (restarting (base|localize)/|(base|localize)/[^ :]*: )|process has died.*($DRIVERS)"
RS_BIN=/mower_ws/install/mower_rs/lib/mower_rs
# mower_base's disarm on 0x85 loss (fix/rf-base-preflight 27bbfaa). Without it
# re-seating the STM32 lead resumes whatever command is live.
LATCH_TEXT='arm latch: wheel feedback lost'

die() { echo "SWITCH_FAIL: $*" >&2; exit 1; }
usage() { sed -n '5,8p' "$0" >&2; exit 2; }
env_of() { sed -n "s/^$1=//p" <<<"$2" | tail -1; }
or_none() { local v; v=$(cat); echo "${v:-$1}"; }
md5() { sudo md5sum "$1" | cut -d' ' -f1; }
truthy() { case "${1,,}" in true|1|yes|on) return 0 ;; *) return 1 ;; esac; }
bool() { [[ $2 == true || $2 == false ]] || die "$1 must be exactly true or false, got '$2'"; }

# none | active | broken (some of the three files without the others)
test_state() {
  local n=0 f
  for f in "$ENV_BAK" "$CF_BAK" "$MARKER"; do [ -e "$f" ] && n=$((n + 1)); done
  case $n in 0) echo none ;; 3) echo active ;; *) echo broken ;; esac
}
refuse_broken() {
  local f
  for f in "$ENV_BAK" "$CF_BAK" "$MARKER"; do
    [ -e "$f" ] && echo "  present: $f" >&2 || echo "  missing: $f" >&2
  done
  die "backups and $MARKER do not match up. Backups without the marker are" \
      "leftovers of the old switch.sh, which never deleted them, or of a switch" \
      "killed while taking them, and may hold a stale .env (an old" \
      "ROSBRIDGE_ADDRESS). Compare with" \
      "'sudo diff $MOWER_DIR/$ENV_BAK $MOWER_DIR/.env' and clean up by hand."
}

compose_up() {
  local out
  if ! out=$(sudo docker compose up -d lawan_node 2>&1); then
    echo "$out" >&2
    die "docker compose up failed"
  fi
  echo "$out" | tail -3
}

# The lawan_node container (a stopped one too): id, image, state, launch
# command, and the env it runs with.
inspect_container() {
  CID=$({ sudo docker compose ps -a -q lawan_node 2>/dev/null || true; } | head -1)
  [ -n "$CID" ] || die "no lawan_node container"
  IMAGE=$(sudo docker inspect -f '{{.Config.Image}}' "$CID")
  CMD=$(sudo docker inspect -f '{{join .Config.Cmd " "}}' "$CID")
  STATE=$(sudo docker inspect -f '{{.State.Status}} since {{.State.StartedAt}}' "$CID")
  CENV=$(sudo docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$CID")
}
# A launch argument's value as the container got it; empty when not passed.
cmd_arg() { { grep -o "\(^\| \)$1:=[^ ]*" <<<"$CMD" || true; } | tail -1 | sed 's/.*:=//'; }
show_effective() {
  local k
  echo "== container ${CID:0:12} $IMAGE ($STATE)"
  echo "== build: MOWER_VERSION=$(env_of MOWER_VERSION "$CENV" | or_none '<unset>')" \
    "MOWER_GIT_SHA=$(env_of MOWER_GIT_SHA "$CENV" | or_none '<unset>')"
  printf '== launch arguments:'
  for k in nav_composition rust_base rust_localize rust_daemon; do
    printf ' %s=%s' "$k" "$(cmd_arg $k | or_none '<not passed>')"
  done
  echo " MOWER_FIRMWARE_SYNC=$(env_of MOWER_FIRMWARE_SYNC "$CENV" | or_none '<unset>')"
  echo "== .env: $(sudo grep -E '^(IMAGE_TAG|NAV_COMPOSITION|RUST_BASE|RUST_LOCALIZE|RUST_DAEMON|MOWER_FIRMWARE_SYNC)=' .env | tr '\n' ' ')"
}

# "<pid> <executable> [<mower_rsd modules>]" for every process in the container.
procs() {
  { sudo docker top "$CID" -o pid,args 2>/dev/null || true; } | awk 'NR > 1 {
    n = $2; sub(".*/", "", n); m = ""
    for (i = 3; i < NF; i++) if ($i == "--modules") m = " " $(i + 1)
    print $1, n m }'
}
# What is wrong with one snapshot against the expected base / localization
# drivers, one problem per line; empty = as expected.
proc_problems() {
  local snap=$1 base=$2 loc=$3 daemon=$4
  n() { awk -v x="$1" '$2 == x' <<<"$snap" | wc -l; }
  rsd() { awk -v m="$1" '$2 == "mower_rsd" && ("," $3 ",") ~ ("," m ",") { f = 1 } END { exit !f }' <<<"$snap"; }
  if truthy "$base"; then
    [ "$(n ros2_control_node)" -eq 0 ] || echo "ros2_control_node still running"
    if truthy "$daemon"; then rsd base || echo "no mower_rsd running the base module"
    else [ "$(n mower_base)" -ge 1 ] || echo "mower_base not running"; fi
  else
    [ "$(n ros2_control_node)" -ge 1 ] || echo "ros2_control_node not running"
    [ "$(n mower_base)" -eq 0 ] || echo "mower_base running"
    ! rsd base || echo "mower_rsd runs the base module"
  fi
  if truthy "$loc"; then
    [ "$(n ekf_node)" -eq 0 ] || echo "C++ ekf_node still running"
    [ "$(n navsat_transform_node)" -eq 0 ] || echo "C++ navsat_transform_node still running"
    if truthy "$daemon"; then rsd localize || echo "no mower_rsd running the localize module"
    else [ "$(n mower_localize)" -ge 1 ] || echo "mower_localize not running"; fi
  else
    [ "$(n ekf_node)" -ge 2 ] || echo "fewer than two C++ ekf_node"
    [ "$(n navsat_transform_node)" -ge 1 ] || echo "navsat_transform_node not running"
    [ "$(n mower_localize)" -eq 0 ] || echo "mower_localize running"
    ! rsd localize || echo "mower_rsd runs the localize module"
  fi
  true
}
show_procs() {
  echo "== base / localization processes:"
  awk -v re="^($DRIVERS)\$" '$2 ~ re' <<<"$1" | sed 's/^/  /'
  echo "  (nav2 component_container_isolated: $(awk '$2 == "component_container_isolated"' <<<"$1" | wc -l))"
}
log_restarts() { { sudo docker logs "$CID" 2>&1 || true; } | { grep -E "$RESTART_RE" || true; }; }
# Wait until the expected drivers are the ones running, then require the same
# PIDs for STABLE_S and no restart in the log, checked on every poll: a driver
# that dies or is respawned once it was up, or a mower_rsd module that fails
# and restarts inside the same PID, fails the switch at once. $4 = warn
# (restore) waits out a flapping driver and only reports logged restarts:
# main's config is what it is, and the files are already back.
wait_for_drivers() {
  local base=$1 loc=$2 daemon=$3 mode=${4:-strict} deadline=$((SECONDS + WAIT_S + STABLE_S))
  local snap problems pids prev='' since=0 restarts
  while :; do
    snap=$(procs)
    problems=$(proc_problems "$snap" "$base" "$loc" "$daemon")
    pids=$(awk -v re="^($DRIVERS)\$" '$2 ~ re { print $1, $2 }' <<<"$snap" | sort)
    restarts=$(log_restarts)
    if [ "$mode" = strict ] && [ -n "$restarts" ]; then
      show_procs "$snap"
      die "$(wc -l <<<"$restarts") base/localization restart(s) in the container log, last:" \
        "$(tail -1 <<<"$restarts")"
    fi
    if [ "$mode" = strict ] && [ -n "$prev" ] && { [ -n "$problems" ] || [ "$pids" != "$prev" ]; }; then
      show_procs "$snap"
      die "a driver died or restarted after it was up: $(tr '\n' ';' <<<"$problems")" \
        "$(diff <(echo "$prev") <(echo "$pids") | sed -n 's/^< /gone /p; s/^> /new /p' | tr '\n' ';')"
    fi
    if [ -n "$problems" ]; then prev=''
    elif [ "$pids" != "$prev" ]; then prev=$pids since=$SECONDS
    elif [ $((SECONDS - since)) -ge "$STABLE_S" ]; then break
    fi
    if [ $SECONDS -ge $deadline ]; then
      show_procs "$snap"
      die "after $((WAIT_S + STABLE_S))s: $(tr '\n' ';' <<<"${problems:-drivers not stable for ${STABLE_S}s}")"
    fi
    sleep "$POLL_S"
  done
  show_procs "$snap"
  echo "== same driver PIDs for ${STABLE_S}s"
  if [ -n "$restarts" ]; then
    echo "WARNING: $(wc -l <<<"$restarts") base/localization restart(s) in the container log:" >&2
    tail -3 <<<"$restarts" | sed 's/^/  /' >&2
  fi
}

# ---- status -----------------------------------------------------------------
if [ "${1:-}" = status ]; then
  cd "$MOWER_DIR"
  echo "== test: $(test_state)$([ -e "$MARKER" ] && sudo sed -n '1s/^# / /p' "$MARKER")"
  inspect_container
  show_effective
  show_procs "$(procs)"
  restarts=$(log_restarts)
  echo "== base/localization restarts in this container's log: $(grep -c . <<<"$restarts" || true)"
  tail -3 <<<"$restarts" | sed '/^$/d; s/^/  /'
  exit 0
fi

# ---- restore ----------------------------------------------------------------
if [ "${1:-}" = restore ]; then
  cd "$MOWER_DIR"
  case $(test_state) in
    none)
      echo "no test active (no backups, no $MARKER): nothing to restore"
      (inspect_container && show_effective) || true
      echo RESTORE_OK
      exit 0 ;;
    broken) refuse_broken ;;
  esac
  sudo cp "$ENV_BAK" .env
  sudo cp "$CF_BAK" docker-compose.yaml
  compose_up
  [ "$(md5 .env)" = "$(md5 "$ENV_BAK")" ] || die ".env differs from $ENV_BAK"
  [ "$(md5 docker-compose.yaml)" = "$(md5 "$CF_BAK")" ] || die "docker-compose.yaml differs from $CF_BAK"
  tag=$(sudo sed -n 's/^IMAGE_TAG=//p' .env | tail -1)
  inspect_container
  show_effective
  [ "$IMAGE" = "$REPO:${tag:-stable}" ] || die "container runs $IMAGE, the restored .env says $REPO:${tag:-stable}"
  for k in rust_base rust_localize rust_daemon; do
    v=$(cmd_arg $k)
    if truthy "$v" && ! sudo grep -qx "${k^^}=$v" "$ENV_BAK"; then
      die "container still has $k:=$v after the restore"
    fi
  done
  wait_for_drivers "$(cmd_arg rust_base)" "$(cmd_arg rust_localize)" "$(cmd_arg rust_daemon)" warn
  sudo rm -f "$ENV_BAK" "$CF_BAK" "$MARKER"
  echo "pre-test .env and docker-compose.yaml back (md5 verified), backups removed"
  echo RESTORE_OK
  exit 0
fi

# ---- switch -----------------------------------------------------------------
[ $# -ge 4 ] || usage
TAG=$1 NC=$2 RD=$3 CF=$4
shift 4
[[ $TAG =~ ^[A-Za-z0-9._-]+$ ]] || die "bad IMAGE_TAG '$TAG'"
bool NAV_COMPOSITION "$NC"
bool RUST_DAEMON "$RD"
[ "$CF" != keep ] || die "'keep' is not accepted any more: the installed compose may be" \
  "main's, which ignores RUST_BASE / RUST_LOCALIZE / RUST_DAEMON / NAV_COMPOSITION." \
  "Pass the branch deploy/docker-compose.yaml."
[ -f "$CF" ] || die "no compose file '$CF'"
CF=$(readlink -f "$CF")
for p in "${PASSTHROUGH[@]}"; do
  grep -qF -- "$p" "$CF" || die "$CF does not pass ${p%%:=*} through (not the branch compose?)"
done
grep -qF "image: $REPO:\${IMAGE_TAG" "$CF" || die "$CF does not run $REPO:\${IMAGE_TAG}"

declare -A SET=([IMAGE_TAG]=$TAG [NAV_COMPOSITION]=$NC [RUST_DAEMON]=$RD
                [RUST_BASE]=false [RUST_LOCALIZE]=false)
ORDER=(IMAGE_TAG NAV_COMPOSITION RUST_DAEMON RUST_BASE RUST_LOCALIZE)
for kv in "$@"; do
  [[ $kv =~ ^([A-Z][A-Z0-9_]*)=(.*)$ && $kv != *$'\n'* ]] || die "not KEY=VAL: '$kv'"
  k=${BASH_REMATCH[1]} v=${BASH_REMATCH[2]}
  case $k in
    IMAGE_TAG|NAV_COMPOSITION|RUST_DAEMON) die "$k is a positional argument" ;;
    RUST_*|NAV_*) bool "$k" "$v" ;;
    MOWER_FIRMWARE_SYNC) [[ $v == 0 || $v == 1 ]] || die "MOWER_FIRMWARE_SYNC must be 0 or 1" ;;
  esac
  [ -n "${SET[$k]+x}" ] || ORDER+=("$k")
  SET[$k]=$v
done

[ -z "$EXPECT_SHA" ] || [[ $EXPECT_SHA =~ ^[0-9a-f]{7,40}$ ]] ||
  die "SWITCH_EXPECT_SHA must be 7 to 40 lowercase hex digits, got '$EXPECT_SHA'"

# A failure says what state it left: PHASE prep = nothing touched, backup =
# the first backups being taken, nothing live touched (they are removed
# again), applied = the test config is (partly) in place, the old container
# still runs, up = from `compose up` on, whatever runs is unverified (it is
# stopped).
PHASE=prep tmp=''
on_exit() {
  local rc=$?
  rm -f "$tmp"
  [ "$rc" -ne 0 ] || return 0
  case $PHASE in
    prep) echo "this run changed nothing" >&2 ;;
    backup)
      sudo rm -f "$ENV_BAK" "$CF_BAK" "$MARKER"
      echo "this run changed nothing" >&2 ;;
    applied)
      echo "TEST CONFIG (PARTLY) APPLIED, no test container verified: run '$0 restore' now" >&2 ;;
    up)
      if sudo docker compose stop lawan_node >/dev/null 2>&1; then
        echo "lawan_node STOPPED (fail-closed: nothing drives the wheels). Run '$0 restore'" \
          "now; until then 'sudo docker logs ${CID:-<container>}' still has this run's log" >&2
      else
        echo "could not stop lawan_node: run '$0 restore' now" >&2
      fi ;;
  esac
}
trap on_exit EXIT
# Ctrl-C, a kill or a dropped ssh mid-switch is a failure too, not a clean exit
# 0. PIPE: after `ssh robot 'switch.sh ...'` loses its client, the first write
# would otherwise kill the shell untrapped, leaving the unverified container up.
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
trap 'exit 141' PIPE

cd "$MOWER_DIR"
sudo docker image inspect "$REPO:$TAG" >/dev/null 2>&1 ||
  die "image $REPO:$TAG is not on this machine (docker load, then docker tag; README.md)"
# Which build this is, before anything changes: the tag alone says nothing
# (rosfree-test has named more than one build).
IENV=$(sudo docker image inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$REPO:$TAG")
ISHA=$(env_of MOWER_GIT_SHA "$IENV")
echo "== image $REPO:$TAG: MOWER_VERSION=$(env_of MOWER_VERSION "$IENV" | or_none '<unset>')" \
  "MOWER_GIT_SHA=${ISHA:-<unset>}"
if [ "${SET[RUST_BASE]}" = true ] || [ "${SET[RUST_LOCALIZE]}" = true ] || [ "$RD" = true ]; then
  [ -n "$EXPECT_SHA" ] || die "RUST_BASE / RUST_LOCALIZE / RUST_DAEMON need SWITCH_EXPECT_SHA=<the" \
    "commit the image was built from, containing the pre-flight fixes> (README.md)"
fi
if [ -n "$EXPECT_SHA" ] && [[ $ISHA != "$EXPECT_SHA"* ]]; then
  die "$REPO:$TAG was built from ${ISHA:-<no MOWER_GIT_SHA>}, not $EXPECT_SHA"
fi
if [ "${SET[RUST_BASE]}" = true ]; then
  bin=$RS_BIN/$([ "$RD" = true ] && echo mower_rsd || echo mower_base)
  # --entrypoint: only grep runs, never the image's firmware-sync.
  sudo docker run --rm --network none --entrypoint grep "$REPO:$TAG" -qaF "$LATCH_TEXT" "$bin" ||
    die "$bin in $REPO:$TAG has no feedback-loss arm latch ('$LATCH_TEXT'):" \
      "built without fix/rf-base-preflight"
fi

state=$(test_state)
[ "$state" != broken ] || refuse_broken
if [ "$state" = active ]; then
  # Rebuilding from the backup would silently drop an edit made to the live
  # .env during the test (e.g. ROSBRIDGE_ADDRESS after a DHCP move).
  live=$(sudo cat .env) bak=$(sudo cat "$ENV_BAK") drift=''
  for k in $(printf '%s\n%s\n' "$live" "$bak" | sed -n 's/^\([A-Za-z_][A-Za-z0-9_]*\)=.*/\1/p' | sort -u); do
    sudo grep -qx "$k" "$MARKER" && continue
    [ "$(grep "^$k=" <<<"$live" || true)" = "$(grep "^$k=" <<<"$bak" || true)" ] || drift+=" $k"
  done
  [ -z "$drift" ] || die ".env was edited during the test (${drift# }); make the same edit in" \
    "$MOWER_DIR/$ENV_BAK (restore puts that file back), or restore first"
fi

# The test's keys, one per line after a comment; the drift check skips them.
write_marker() { printf '%s\n' "# $(date -Is) tag=$TAG" "$@" | sudo tee "$MARKER" >/dev/null; }
if [ "$state" = none ]; then
  PHASE=backup
  sudo cp -p .env "$ENV_BAK"
  sudo cp -p docker-compose.yaml "$CF_BAK"
  write_marker "${ORDER[@]}"
else
  PHASE=applied
  # Keys of the previous switch too, until the new .env is in place.
  mapfile -t prev_keys < <(sudo sed 1d "$MARKER")
  write_marker "${prev_keys[@]}" "${ORDER[@]}"
fi
PHASE=applied

tmp=$(mktemp)
# shellcheck disable=SC2024  # $tmp is ours; only the read needs root
sudo cat "$ENV_BAK" >"$tmp"
for k in "${ORDER[@]}"; do
  sed -i "/^$k=/d" "$tmp"
  printf '%s=%s\n' "$k" "${SET[$k]}" >>"$tmp"
done
echo "== .env against the pre-test backup:"
diff <(sudo cat "$ENV_BAK") "$tmp" | grep '^[<>]' | sed 's/^/  /' || true
sudo cp "$tmp" .env
# Equal when the installed compose already is the branch one (or the same file).
sudo cmp -s "$CF" docker-compose.yaml || sudo cp "$CF" docker-compose.yaml
write_marker "${ORDER[@]}"
if command -v systemctl >/dev/null && systemctl is-active --quiet mower-update.timer; then
  echo "WARNING: mower-update.timer is active; stop it for the test (README.md)" >&2
fi
echo "TEST CONFIG ACTIVE: it survives a reboot until '$0 restore'"

PHASE=up
compose_up
inspect_container
show_effective
[ "$IMAGE" = "$REPO:$TAG" ] || die "container runs $IMAGE, not $REPO:$TAG"
for k in NAV_COMPOSITION RUST_BASE RUST_LOCALIZE RUST_DAEMON; do
  v=$(cmd_arg "${k,,}")
  [ "$v" = "${SET[$k]}" ] || die "container got ${k,,}:=${v:-<not passed>}, asked for ${SET[$k]}"
done
wait_for_drivers "${SET[RUST_BASE]}" "${SET[RUST_LOCALIZE]}" "$RD"
PHASE=verified
echo SWITCH_OK
