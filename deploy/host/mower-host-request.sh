#!/bin/bash
# Serve one request from <state_dir>/host.request, written by the robot
# container (mower_mission/host_request.py, utils/mower-host-request):
#   {"action": "update|check|restart|reboot|poweroff", "time": ..., "requested_by": "..."}
# Triggered by mower-host-request.path whenever the file changes.
set -uo pipefail

STATE_DIR=${1:-/home/cat/.mower}
REQ="$STATE_DIR/host.request"
MOWER_DIR=${MOWER_DIR:-/opt/mower}

# The state dir belongs to the robot user and the container writes it, so a root
# script treats what is there as input and does not touch it as root: a read or
# write there runs as the directory's owner, with none of root's environment,
# and a file written that way needs no chown. A directory root itself owns has
# no other writer, and a person's run as that owner has nothing to give up.
as_state_owner() {
  local who
  if [ "$EUID" -ne 0 ]; then "$@"; return; fi
  # The owner of the name first; of what it points at only if root made the name.
  who=$(stat -c '%u:%g' "$STATE_DIR") || return 1
  [ "${who%%:*}" != 0 ] || who=$(stat -Lc '%u:%g' "$STATE_DIR") || return 1
  if [ "${who%%:*}" = 0 ]; then "$@"; return; fi
  setpriv --reuid="${who%%:*}" --regid="${who##*:}" --clear-groups env -i PATH="$PATH" "$@"
}
state_write() {  # name, the contents on stdin: a temporary file, then a rename
  as_state_owner sh -c 'umask 022; set -C; rm -f "$1/$2.tmp" && cat > "$1/$2.tmp" && mv -f "$1/$2.tmp" "$1/$2"' sh "$STATE_DIR" "$1"
}

[ -f "$REQ" ] || exit 0
action=$(as_state_owner python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("action",""))' "$REQ" 2>/dev/null || echo "")
by=$(as_state_owner python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("requested_by",""))' "$REQ" 2>/dev/null || echo "")
as_state_owner rm -f "$REQ"
logger -t mower-host-request "action=$action by=$by"

status() {
  printf '{"state":"%s","message":"%s","time":%s}\n' "$1" "$2" "$(date +%s)" | state_write update_status.json
}

case "$action" in
  update)
    exec "$MOWER_DIR/host/mower-update.sh"
    ;;
  check)
    exec "$MOWER_DIR/host/mower-update.sh" --check
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
