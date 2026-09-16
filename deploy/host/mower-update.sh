#!/bin/bash
# Pull the configured robot image and restart the stack if it changed.
#
#   mower-update.sh              pull IMAGE_TAG from /opt/mower/.env
#   mower-update.sh --tag v0.6.0 switch the channel/version first (also how
#                                you roll back: --tag v0.5.0)
#   mower-update.sh --force      restart even if the digest did not change
#
# Progress goes to <MOWER_STATE_DIR>/update_status.json (relayed to the app
# by /robot/info) and the running image identity to image.json. Runs as root
# (systemd, or sudo); called by mower-host-request.sh for /system/update.
set -uo pipefail

MOWER_DIR=${MOWER_DIR:-/opt/mower}
ENV_FILE="$MOWER_DIR/.env"
SERVICE=lawan_node

tag=""
force=0
while [ $# -gt 0 ]; do
  case "$1" in
    --tag) tag="$2"; shift 2 ;;
    --force) force=1; shift ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
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
IMAGE="ghcr.io/edwards414/mower_path_planning:${IMAGE_TAG}"
mkdir -p "$STATE_DIR"

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

before=$(image_digest)
status pulling "pulling $IMAGE"
if ! docker compose pull --quiet "$SERVICE"; then
  status failed "pull of $IMAGE failed (network? docker login ghcr.io?)"
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
docker image prune -f >/dev/null 2>&1 || true
status idle "updated to $IMAGE (${after#*@})"
