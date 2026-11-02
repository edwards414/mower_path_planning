#!/bin/bash
# Entry point of the outdoor stress harness's `sim` service
# (docker-compose.outdoor.yaml, outdoor.sh). Development only, never on a
# robot. It is ../sim-entry.sh with what the stress rows need:
#
#   - the pairing gate on, with tools/ab's dev identity (MW-E2TEST), as on the
#     robot; the harness's clients use the matching tools/ab/fixtures/pair.json;
#   - a fresh state dir on the container's first start (the sim's sample site);
#   - mower_sim with a truth log (/tmp/outdoor/truth.<start>.jsonl, 10 Hz), its seed
#     (SIM_SEED) and its output in /tmp/outdoor/sim.<start>.log, all inside the
#     container (a live bind-mounted log can stall a writer; outdoor.sh
#     collect copies them out);
#   - the robot's .env switches sim-entry.sh leaves out: RUST_RECORDER (true on
#     the robot since 2026-10-31), RUST_MUX, RUST_STATIC_TF;
#   - guard overrides for row 2: MANUAL_GUARD_TIMEOUT_S / FINAL_GUARD_TIMEOUT_S
#     put command_timeout_s into a copy of mower_rsd.yaml that
#     rust_daemon_params_file points at.
set -euo pipefail
log() { echo "[outdoor] $*"; }
STATE=/home/mower/.mower
CONTROL=/tmp/mower_sim
OUT=/tmp/outdoor
SIM=/usr/local/bin/mower_sim

# --- never on a robot (as sim-entry.sh) --------------------------------------
for dev in /dev/stmcom /dev/imu_usb /dev/gps_rtk; do
  if [ -e "$dev" ] || [ -L "$dev" ]; then
    target=$(readlink "$dev" || true)
    case "$target" in
      /dev/pts/*) ;;
      *) log "refusing to start: $dev exists (${target:-not a link}); this is the robot's /dev"; exit 1 ;;
    esac
  fi
done
for dev in /dev/ttyS3 /dev/ttyUSB* /dev/ttyACM*; do
  if [ -e "$dev" ]; then
    log "refusing to start: $dev is visible, the host's serial devices are mounted; is this the robot?"
    exit 1
  fi
done
case "${MOWER_BACKEND_URL:-}" in
  "" | http://127.0.0.1* | http://localhost*) ;;
  *) log "refusing to start: MOWER_BACKEND_URL=$MOWER_BACKEND_URL is not local"; exit 1 ;;
esac
[ -x "$SIM" ] || { log "no mower_sim at $SIM: mount a Linux build of it"; exit 1; }

# --- loopback multicast for DDS ----------------------------------------------
python3 - <<'EOF' || { log "cannot turn on multicast on lo (cap_add NET_ADMIN)"; exit 1; }
import fcntl, socket, struct
SIOCGIFFLAGS, SIOCSIFFLAGS, IFF_MULTICAST = 0x8913, 0x8914, 0x1000
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
flags = struct.unpack("16sH", fcntl.ioctl(s, SIOCGIFFLAGS, struct.pack("16sH", b"lo", 0))[:18])[1]
fcntl.ioctl(s, SIOCSIFFLAGS, struct.pack("16sH", b"lo", flags | IFF_MULTICAST))
EOF

# --- state: fresh, gate on ---------------------------------------------------
# A container's first start wipes the state volume; a restart of the same
# container (row 5's reload check, a power kill) keeps it. Each start gets its
# own truth and sim log, numbered.
mkdir -p "$OUT"
START=$(( $(cat "$OUT/starts" 2>/dev/null || echo 0) + 1 ))
echo "$START" >"$OUT/starts"
if [ "$START" = 1 ]; then
  rm -rf "${STATE:?}"/*
fi
mkdir -p "$STATE/zone_record" "$STATE/sites" "$STATE/bags"
if [ -z "$(ls -A "$STATE/zone_record")" ]; then
  cp /sim/fixtures/zone_record/*.json "$STATE/zone_record/"
fi
cp /fixtures/identity.json "$STATE/identity.json"
chown -R 1000:1000 "$STATE"

# --- parameters --------------------------------------------------------------
SHARE=${WORKSPACE:-/mower_ws}/install
PARAMS="$SHARE/mower_bringup/share/mower_bringup/config/mower_rsd.yaml"
if [ -n "${MANUAL_GUARD_TIMEOUT_S:-}${FINAL_GUARD_TIMEOUT_S:-}" ]; then
  python3 - "$PARAMS" "$OUT/mower_rsd.yaml" "${MANUAL_GUARD_TIMEOUT_S:-}" "${FINAL_GUARD_TIMEOUT_S:-}" <<'EOF'
import sys, yaml
src, dst, manual, final = sys.argv[1:5]
doc = yaml.safe_load(open(src))
for node, value in (("manual_velocity_guard", manual), ("velocity_command_guard", final)):
    if value:
        doc.setdefault(node, {}).setdefault("ros__parameters", {})["command_timeout_s"] = float(value)
yaml.safe_dump(doc, open(dst, "w"), sort_keys=False)
print("[outdoor] guard overrides:", {n: doc[n]["ros__parameters"] for n in ("manual_velocity_guard", "velocity_command_guard")})
EOF
  PARAMS="$OUT/mower_rsd.yaml"
fi

# --- the devices -------------------------------------------------------------
mkdir -p "$CONTROL"
chown 1000:1000 "$CONTROL"
# shellcheck disable=SC2086
"$SIM" --owner 1000:1000 --control-dir "$CONTROL" \
  --urdf "$SHARE/mower_description/share/mower_description/mower_robot/robot.urdf.xacro" \
  --params "$SHARE/mower_bringup/share/mower_bringup/config/mower_rsd.yaml" \
  --truth-log "$OUT/truth.$START.jsonl" --seed "${SIM_SEED:-1}" \
  ${SIM_ARGS:-} >"$OUT/sim.$START.log" 2>&1 &
for _ in $(seq 50); do
  [ -L /dev/stmcom ] && [ -L /dev/imu_usb ] && [ -L /dev/gps_rtk ] && break
  kill -0 $! 2>/dev/null || { log "mower_sim did not start"; cat "$OUT/sim.$START.log"; exit 1; }
  sleep 0.1
done
log "start $START: mower_sim up (seed ${SIM_SEED:-1}, args: ${SIM_ARGS:-none}); params $PARAMS; RUST_NAV_CORE=${RUST_NAV_CORE:-false}"

# --- the stack (sim-entry.sh's, plus the switches above) ---------------------
exec setpriv --reuid=1000 --regid=1000 --init-groups \
  /usr/local/bin/docker-entrypoint.sh \
  ros2 launch mower_bringup robot.launch.py \
  use_sim_time:=false \
  rosbridge_address:=0.0.0.0 \
  zone_record_dir:=$STATE/zone_record \
  sites_dir:=$STATE/sites \
  output_root:=$STATE/bags \
  record:=false \
  rust_daemon_params_file:="$PARAMS" \
  enable_gps:="${GPS:-true}" \
  nav_composition:="${NAV_COMPOSITION:-true}" \
  rust_status:="${RUST_STATUS:-true}" \
  rust_adapter:="${RUST_ADAPTER:-true}" \
  rust_record:="${RUST_RECORD:-true}" \
  rust_nav:="${RUST_NAV:-true}" \
  rust_battery:="${RUST_BATTERY:-true}" \
  rust_pid_autotune:="${RUST_PID_AUTOTUNE:-true}" \
  rust_map:="${RUST_MAP:-true}" \
  rust_agent:="${RUST_AGENT:-true}" \
  rust_guards:="${RUST_GUARDS:-true}" \
  rust_imu:="${RUST_IMU:-true}" \
  rust_bridge:="${RUST_BRIDGE:-true}" \
  rust_base:="${RUST_BASE:-true}" \
  rust_localize:="${RUST_LOCALIZE:-true}" \
  rust_daemon:="${RUST_DAEMON:-true}" \
  rust_nav_core:="${RUST_NAV_CORE:-false}" \
  rust_recorder:="${RUST_RECORDER:-true}" \
  rust_mux:="${RUST_MUX:-false}" \
  rust_static_tf:="${RUST_STATIC_TF:-false}"
