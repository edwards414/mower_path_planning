#!/bin/bash
# Serve one request from <state_dir>/host.request, written by the robot
# container (mower_mission/host_request.py, utils/mower-host-request):
#   {"action": "update|restart|reboot|poweroff", "time": ..., "requested_by": "..."}
# Triggered by mower-host-request.path whenever the file changes.
set -uo pipefail

STATE_DIR=${1:-/home/cat/.mower}
REQ="$STATE_DIR/host.request"
MOWER_DIR=${MOWER_DIR:-/opt/mower}

[ -f "$REQ" ] || exit 0
action=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("action",""))' "$REQ" 2>/dev/null || echo "")
by=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("requested_by",""))' "$REQ" 2>/dev/null || echo "")
rm -f "$REQ"
logger -t mower-host-request "action=$action by=$by"

status() {
  local tmp="$STATE_DIR/update_status.json.tmp"
  printf '{"state":"%s","message":"%s","time":%s}\n' "$1" "$2" "$(date +%s)" > "$tmp" && mv -f "$tmp" "$STATE_DIR/update_status.json"
  chown --reference="$STATE_DIR" "$STATE_DIR/update_status.json" 2>/dev/null || true
}

case "$action" in
  update)
    exec "$MOWER_DIR/host/mower-update.sh"
    ;;
  restart)
    status restarting "restarting robot software"
    cd "$MOWER_DIR" && docker compose restart lawan_node
    status idle "restarted"
    ;;
  reboot)
    status rebooting "host reboot requested by $by"
    systemctl reboot
    ;;
  poweroff)
    status powering_off "host power-off requested by $by"
    # the STM32 cuts the rail after its grace period once the host is down
    systemctl poweroff
    ;;
  "")
    ;;
  *)
    logger -t mower-host-request "ignoring unknown action '$action'"
    ;;
esac
