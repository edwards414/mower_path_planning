#!/bin/bash
# Entry point of docker-compose.sim.yaml's `sim` service, run as root in the
# robot's runtime image. Development only (README.md):
#
#   1. refuse to run where the robot's devices are;
#   2. multicast on the container's loopback, for CycloneDDS discovery (the
#      robot's mower.service does `ip link set lo multicast on`);
#   3. the state dir: a sample site on the first start, a dev pairing
#      identity with SIM_PAIRED=1;
#   4. mower_sim (a local build, mounted; not in the image): the STM32, the
#      IMU and the F9P on ptys at /dev/stmcom, /dev/imu_usb and /dev/gps_rtk;
#   5. the production stack, as the robot's compose starts it, with the
#      robot's .env switches, as the user mower (uid 1000).
set -euo pipefail
log() { echo "[sim] $*"; }
STATE=/home/mower/.mower
CONTROL=/tmp/mower_sim
SIM=/usr/local/bin/mower_sim

# --- 1. never on a robot ----------------------------------------------------------
# The service mounts no host /dev: the container's own has none of these. A
# device link that is not a pty, or any host serial port, means someone ran
# this where the hardware is.
for dev in /dev/stmcom /dev/imu_usb /dev/gps_rtk; do
  if [ -e "$dev" ] || [ -L "$dev" ]; then
    target=$(readlink "$dev" || true)
    case "$target" in
      /dev/pts/*) ;;
      *) log "refusing to start: $dev exists (${target:-not a link}); this is the robot's /dev, not a simulation"; exit 1 ;;
    esac
  fi
done
for dev in /dev/ttyS3 /dev/ttyUSB* /dev/ttyACM*; do
  if [ -e "$dev" ]; then
    log "refusing to start: $dev is visible, the host's serial devices are mounted; is this the robot?"
    exit 1
  fi
done
if [ ! -x "$SIM" ]; then
  log "no mower_sim at $SIM: mount a Linux build of it (README.md, Start)"
  exit 1
fi

# --- 2. loopback multicast ----------------------------------------------------------
python3 - <<'EOF' || { log "cannot turn on multicast on lo: the service needs cap_add NET_ADMIN"; exit 1; }
import fcntl, socket, struct
SIOCGIFFLAGS, SIOCSIFFLAGS, IFF_MULTICAST = 0x8913, 0x8914, 0x1000
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
flags = struct.unpack("16sH", fcntl.ioctl(s, SIOCGIFFLAGS, struct.pack("16sH", b"lo", 0))[:18])[1]
fcntl.ioctl(s, SIOCSIFFLAGS, struct.pack("16sH", b"lo", flags | IFF_MULTICAST))
EOF

# --- 3. state ------------------------------------------------------------------------
mkdir -p "$STATE/zone_record" "$STATE/sites" "$STATE/bags"
if [ -z "$(ls -A "$STATE/zone_record")" ]; then
  cp /sim/fixtures/zone_record/*.json "$STATE/zone_record/"
  log "sample site in $STATE/zone_record: two zones (8 x 6 m around the robot, 7.5 x 6 m east of it) joined by a channel, one risk zone"
fi
# The gate is open without identity.json (the app's direct-IP mode). With
# SIM_PAIRED=1 a dev identity is made once and kept aside while unpaired.
if [ "${SIM_PAIRED:-0}" = 1 ]; then
  [ -s "$STATE/identity.json" ] || { [ -s "$STATE/identity.sim.json" ] && mv "$STATE/identity.sim.json" "$STATE/identity.json"; } || true
  python3 - "$STATE/identity.json" "${SIM_HOST_IP:-}" <<'EOF'
import base64, json, os, secrets, sys, urllib.parse
path, lan = sys.argv[1], sys.argv[2]
if not (os.path.exists(path) and os.path.getsize(path)):
    with open(path, "w") as f:
        json.dump({"robot_id": "MW-SIM001", "name": "mower-sim",
                   "secret": base64.b32encode(secrets.token_bytes(20)).decode().rstrip("=")}, f, indent=1)
d = json.load(open(path))
params = {"v": "1", "id": d["robot_id"], "s": d["secret"], "n": d.get("name", "")}
if lan:
    params["l"] = lan
print("[sim] paired: robot", d["robot_id"], "- pairing URL (the app's QR payload, as mower-pair prints it):")
print("[sim]   https://mower.fxrbindi.com/pair?" + urllib.parse.urlencode(params))
EOF
elif [ -e "$STATE/identity.json" ]; then
  mv "$STATE/identity.json" "$STATE/identity.sim.json"
  log "SIM_PAIRED is off: identity.json kept aside as identity.sim.json, the gate is open"
fi
chown -R 1000:1000 "$STATE"

# --- 4. the devices ----------------------------------------------------------------
mkdir -p "$CONTROL"
chown 1000:1000 "$CONTROL"
# The stack's own geometry: the URDF robot_state_publisher reads and
# mower_base's parameters, from the image (named here: the ROS environment
# is not sourced yet).
SHARE=${WORKSPACE:-/mower_ws}/install
# shellcheck disable=SC2086
"$SIM" --owner 1000:1000 --control-dir "$CONTROL" \
  --urdf "$SHARE/mower_description/share/mower_description/mower_robot/robot.urdf.xacro" \
  --params "$SHARE/mower_bringup/share/mower_bringup/config/mower_rsd.yaml" \
  ${SIM_ARGS:-} &
for _ in $(seq 50); do
  [ -L /dev/stmcom ] && [ -L /dev/imu_usb ] && [ -L /dev/gps_rtk ] && break
  kill -0 $! 2>/dev/null || { log "mower_sim did not start"; exit 1; }
  sleep 0.1
done
log "faults: touch $CONTROL/base_mute, base_deaf or sensors_mute; echo float > $CONTROL/gps_mode"

# --- 5. the stack -------------------------------------------------------------------
# deploy/docker-compose.yaml's command with the robot's .env (every RUST_*
# on, nav2 for navigation), listening on all of the container's addresses so
# the published port reaches it.
exec setpriv --reuid=1000 --regid=1000 --init-groups \
  /usr/local/bin/docker-entrypoint.sh \
  ros2 launch mower_bringup robot.launch.py \
  use_sim_time:=false \
  rosbridge_address:=0.0.0.0 \
  zone_record_dir:=$STATE/zone_record \
  sites_dir:=$STATE/sites \
  output_root:=$STATE/bags \
  record:=false \
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
  rust_nav_core:="${RUST_NAV_CORE:-false}"
