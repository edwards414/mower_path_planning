#!/bin/bash
# One side of the image A/B (tools/ab/README.md), inside the image under test.
#
#   docker run -u 0 --cap-add NET_ADMIN --entrypoint bash IMAGE /ab/side.sh
#
# Mounts: /ab = tools/ab, /rs-tools = src/mower_rs/tools (fake_base.py), both
# read-only; /run-in = this side's stack.argv (the image's entrypoint and
# compose's lawan_node command, one argument per line) and stack.env
# (compose's lawan_node environment, KEY=VALUE lines).
#
# As root, it sets up what the robot has around the container (devices, the
# state dir, loopback), then execs the stack as uid 1000: the stack is PID 1
# and its output is the container's log, as under compose. `docker stop`
# with compose's grace period stops it. The fakes' and this script's files
# stay in /tmp/ab, inside the container (a bind mount can block a write for
# seconds, and a blocked log write stalls the fake STM32); ab.py copies them
# out once the container has stopped.
set -u
w=/tmp/ab
export HOME=/root

mkdir -p "$w"
log() { echo "[side] $*" >>"$w/side.log"; }
log "$(date -u +%FT%TZ) MOWER_GIT_SHA=${MOWER_GIT_SHA:-?} MOWER_VERSION=${MOWER_VERSION:-?}"

# the robot's VPN address (compose's ROSBRIDGE_ADDRESS default) on lo, so
# ws_bridge binds its --address as there; lo multicast for DDS
python3 /ab/incontainer.py net 10.77.0.2 >>"$w/side.log" 2>&1 || log "network setup failed"

# --- state dir: what compose bind-mounts at /home/mower/.mower ----------------
state=/home/mower/.mower
rm -rf "$state"
mkdir -p "$state/zone_record" "$state/sites" "$state/bags"
cp /ab/fixtures/identity.json "$state/"
cp /ab/fixtures/zone_record/*.json "$state/zone_record/"
chown -R 1000:1000 "$state"

# --- fake devices under the robot's udev names, root:dialout 0660 like the real ones
python3 /rs-tools/fake_base.py --pty-link /dev/stmcom --log "$w/fake_base.jsonl" --seconds 3600 \
  >"$w/fake_base.log" 2>&1 &
python3 /ab/fake_sensors.py --seconds 3600 >"$w/fake_sensors.log" 2>&1 &
for dev in /dev/stmcom /dev/imu_usb /dev/gps_rtk; do
  for _ in $(seq 100); do [ -e "$dev" ] && break; sleep 0.1; done
  tty=$(readlink -f "$dev")
  { chgrp dialout "$tty" && chmod 660 "$tty"; } || log "no $dev"
done

# --- the ports ab.py's probes use: the pairing gate and the loopback port -----
python3 /ab/incontainer.py forward 19090=127.0.0.1:9090 19091=127.0.0.1:9091 >"$w/forward.log" 2>&1 &

# --- the stack, as PID 1 ---------------------------------------------------------
mapfile -t argv </run-in/stack.argv
while IFS= read -r kv; do [ -n "$kv" ] && export "${kv?}"; done </run-in/stack.env
log "stack as uid 1000: ${argv[*]}"
exec setpriv --reuid=1000 --regid=1000 --init-groups env HOME=/home/mower "${argv[@]}"
