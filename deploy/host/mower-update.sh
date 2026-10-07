#!/bin/bash
# Pull the configured robot image and restart the stack if it changed.
#
#   mower-update.sh              pull IMAGE_TAG from /opt/mower/.env
#   mower-update.sh --tag v0.6.0 switch the channel/version first (also how
#                                you roll back: --tag v0.5.0)
#   mower-update.sh --force      restart even if the digest did not change
#   mower-update.sh --auto       scheduled run (mower-update.timer): does
#                                nothing while MOWER_AUTO_UPDATE=0, while the
#                                robot is busy (robot_status.json from
#                                robot_info_node) or while another update is
#                                already in progress
#   mower-update.sh --check      only ask the registry whether the channel
#                                moved (mower-update-check.timer, every 5 min,
#                                and /system/check_update): writes
#                                update_check.json, never pulls or restarts
#
# Progress goes to <MOWER_STATE_DIR>/update_status.json (relayed to the app
# by /robot/info; the base shows the amber orbit while it is pulling or
# restarting), the running image identity to image.json and the last
# registry check to update_check.json (/robot/info update.available). Runs
# as root (systemd, or sudo); called by mower-host-request.sh for
# /system/update and /system/check_update.
set -uo pipefail

MOWER_DIR=${MOWER_DIR:-/opt/mower}
ENV_FILE="$MOWER_DIR/.env"
SERVICE=lawan_node
ROBOT_STATUS_MAX_AGE_S=30

tag=""
force=0
auto=0
check=0
while [ $# -gt 0 ]; do
  case "$1" in
    --tag) tag="$2"; shift 2 ;;
    --force) force=1; shift ;;
    --auto) auto=1; shift ;;
    --check) check=1; shift ;;
    -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

cd "$MOWER_DIR" || { echo "no $MOWER_DIR (run deploy/install.sh first)" >&2; exit 1; }
touch "$ENV_FILE"
if [ -n "$tag" ]; then
  if grep -q '^IMAGE_TAG=' "$ENV_FILE"; then
    sed -i "s|^IMAGE_TAG=.*|IMAGE_TAG=$tag|" "$ENV_FILE"
  else
    echo "IMAGE_TAG=$tag" >> "$ENV_FILE"
  fi
fi
# shellcheck disable=SC1090
set -a; . "$ENV_FILE"; set +a
IMAGE_TAG=${IMAGE_TAG:-stable}
STATE_DIR=${MOWER_STATE_DIR:-/home/cat/.mower}
# Registry + repository the robot pulls from (.env IMAGE_REPO). CI pushes
# every tag to both ghcr.io and Docker Hub; Docker Hub is the default because
# from the home ISP ghcr.io's CDN delivers ~70 kB/s against ~9 MB/s
# (deploy/README.md). Root must be logged in to that registry when the
# repository is private: sudo docker login <registry>.
IMAGE_REPO=${IMAGE_REPO:-docker.io/edwards414/mower_path_planning}
REGISTRY=${IMAGE_REPO%%/*}
IMAGE="${IMAGE_REPO}:${IMAGE_TAG}"
mkdir -p "$STATE_DIR"

# "An update is in progress" is this lock, not the state file: a pull cut
# short by a reboot or a power cut left update_status.json at "pulling"
# forever, and every later run (timer, app) refused to start (lubancat,
# 2026-10-07: stuck from 17:57 across two reboots). The file only tells the
# app what is going on; the lock dies with the process.
exec 9>"$STATE_DIR/.update.lock"
if ! flock -n 9; then
  echo "[mower-update] an update is already in progress (another mower-update.sh holds the lock)"
  exit 0
fi

status() {  # state message
  local tmp="$STATE_DIR/update_status.json.tmp"
  printf '{"state":"%s","message":"%s","tag":"%s","time":%s}\n' "$1" "${2//\"/\\\"}" "$IMAGE_TAG" "$(date +%s)" > "$tmp" \
    && mv -f "$tmp" "$STATE_DIR/update_status.json"
  chown --reference="$STATE_DIR" "$STATE_DIR/update_status.json" 2>/dev/null || true
  echo "[mower-update] $1: $2"
}

image_digest() {
  docker image inspect --format '{{if .RepoDigests}}{{index .RepoDigests 0}}{{end}}' "$IMAGE" 2>/dev/null || true
}

write_image_json() {
  local id digest tmp="$STATE_DIR/image.json.tmp"
  id=$(docker image inspect --format '{{.Id}}' "$IMAGE" 2>/dev/null || true)
  digest=$(image_digest)
  printf '{"image":"%s","tag":"%s","digest":"%s","id":"%s","pulled_at":%s}\n' \
    "$IMAGE" "$IMAGE_TAG" "${digest#*@}" "$id" "$(date +%s)" > "$tmp" && mv -f "$tmp" "$STATE_DIR/image.json"
  chown --reference="$STATE_DIR" "$STATE_DIR/image.json" 2>/dev/null || true
}

write_check_json() {  # remote_digest error
  local tmp="$STATE_DIR/update_check.json.tmp"
  printf '{"time":%s,"tag":"%s","image":"%s","remote_digest":"%s","local_digest":"%s","error":"%s"}\n' \
    "$(date +%s)" "$IMAGE_TAG" "$IMAGE" "$1" "$(image_digest | sed 's/.*@//')" "${2//\"/\\\"}" > "$tmp" \
    && mv -f "$tmp" "$STATE_DIR/update_check.json"
  chown --reference="$STATE_DIR" "$STATE_DIR/update_check.json" 2>/dev/null || true
}

json_field() {  # file key -> value or empty
  python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); v=d.get(sys.argv[2]); print("" if v is None else (str(v).lower() if isinstance(v,bool) else v))' "$1" "$2" 2>/dev/null
}

if [ "$auto" -eq 1 ]; then
  if [ "${MOWER_AUTO_UPDATE:-1}" = "0" ]; then
    echo "[mower-update] auto update disabled (MOWER_AUTO_UPDATE=0)"
    exit 0
  fi
  rs="$STATE_DIR/robot_status.json"
  if [ -f "$rs" ]; then
    age=$(( $(date +%s) - $(json_field "$rs" time || echo 0) ))
    if [ "$age" -le "$ROBOT_STATUS_MAX_AGE_S" ] && [ "$(json_field "$rs" busy)" = "true" ]; then
      echo "[mower-update] robot is busy (moving or navigating), trying again next time"
      exit 0
    fi
  fi
fi

remote_digest() {
  # manifest digest on the registry (no download); empty if unknown
  docker manifest inspect -v "$IMAGE" 2>/dev/null | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
except ValueError:
    sys.exit(0)
if isinstance(d, list):
    d = d[0] if d else {}
print((d.get("Descriptor") or {}).get("digest", ""))
' 2>/dev/null
}

if [ "$check" -eq 1 ]; then
  # Registry lookup only (one HTTPS request, no download, no state change):
  # robot_status turns remote_digest != software.digest into
  # /robot/info update.available for the apps.
  remote=$(remote_digest)
  if [ -z "$remote" ]; then
    write_check_json "" "registry lookup failed (network or $REGISTRY login)"
    echo "[mower-update] check: registry lookup failed"
    exit 1
  fi
  write_check_json "$remote" ""
  local_digest=$(image_digest)
  if [ "${local_digest#*@}" = "$remote" ]; then
    echo "[mower-update] check: up to date ($remote)"
  else
    echo "[mower-update] check: $IMAGE moved to $remote (running ${local_digest#*@})"
  fi
  exit 0
fi

cur_state=$(json_field "$STATE_DIR/update_status.json" state)
case "$cur_state" in
  pulling|restarting|rebooting)
    # nobody holds the lock, so that update died: say so, then carry on
    status failed "earlier update interrupted while $cur_state (reboot or power loss)" ;;
esac

before=$(image_digest)
remote=$(remote_digest)
[ -n "$remote" ] && write_check_json "$remote" ""
if [ -n "$remote" ] && [ "${before#*@}" = "$remote" ] && [ "$force" -eq 0 ] \
   && docker compose ps --status running "$SERVICE" 2>/dev/null | grep -q "$SERVICE"; then
  # nothing new on the channel: say so without entering the pulling state
  # (which lights the update effect on the robot)
  write_image_json
  status up_to_date "already running $remote"
  exit 0
fi

status pulling "pulling $IMAGE"
# plain docker pull: compose pull happily "skips" when a stale local image
# exists and the registry says denied
if ! pull_out=$(docker pull "$IMAGE" 2>&1); then
  case "$pull_out" in
    *denied*|*unauthorized*) hint="$REGISTRY login for root is missing or expired: sudo docker login $REGISTRY" ;;
    *"no such host"*|*"timeout"*|*"connection refused"*) hint="no network to $REGISTRY" ;;
    *) hint="${pull_out##*$'\n'}" ;;
  esac
  status failed "pull of $IMAGE failed: $hint"
  exit 1
fi
after=$(image_digest)

if [ "$before" = "$after" ] && [ "$force" -eq 0 ] && docker compose ps --status running "$SERVICE" 2>/dev/null | grep -q "$SERVICE"; then
  write_image_json
  status up_to_date "already running ${after#*@}"
  exit 0
fi

status restarting "starting ${after#*@}"
if ! docker compose up -d --remove-orphans; then
  status failed "docker compose up failed"
  exit 1
fi
write_image_json
write_check_json "${after#*@}" ""
docker image prune -f >/dev/null 2>&1 || true
status idle "updated to $IMAGE (${after#*@})"
