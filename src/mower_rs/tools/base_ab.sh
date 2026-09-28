#!/bin/bash
# One side of the Phase B differential test, under the robot's RMW.
#
# Runs inside the runtime image (it has rmw_cyclonedds_cpp, the robot's
# /etc/mower/cyclonedds.xml and the installed C++ chain) with the repo
# mounted at /repo:
#
#   docker run --rm -u 0 --cap-add NET_ADMIN --cap-add SYS_NICE -e ROS_DOMAIN_ID=61 \
#     -v "$PWD":/repo -v /tmp/ab:/out --entrypoint bash \
#     mower_path_planning:ros-free-test \
#     /repo/src/mower_rs/tools/base_ab.sh A /out/a            # ros2_control chain
#   ... base_ab.sh B /out/b [/path/to/mower_base]              # mower_base
#   ... base_ab.sh B /out/b_pull "" --scenario pull            # the cable pull
#   python3 src/mower_rs/tools/base_compare.py --a /tmp/ab/a --b /tmp/ab/b
#
# A fake STM32 (fake_base.py) owns a pty linked at /dev/stmcom, the driver
# opens it, base_harness.py drives and records. B defaults to the image's
# own mower_base; pass a freshly built binary to test a change. Root (-u 0)
# for the /dev/stmcom link the C++ chain's URDF hard-codes; NET_ADMIN for the
# one thing the robot's mower.service does and a container does not:
# multicast on lo, which the cyclonedds.xml discovery needs. SYS_NICE (optional)
# lets the fake STM32 and the driver run ahead of the harness: on a loaded
# host a Python fake that stalls for half a second is a feedback loss, and
# mower_base's arm latch then does exactly what it should.
side=$1
out=$2
bin=${3:-}
shift 3 2>/dev/null || shift $#
tools=$(cd "$(dirname "$0")" && pwd)
repo=$(cd "$tools/../../.." && pwd)

source /opt/ros/jazzy/setup.bash
source /mower_ws/install/setup.bash
python3 - <<'EOF'
import fcntl, socket, struct
SIOCGIFFLAGS, SIOCSIFFLAGS, IFF_MULTICAST = 0x8913, 0x8914, 0x1000
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
flags = struct.unpack("16sH", fcntl.ioctl(s, SIOCGIFFLAGS, struct.pack("16sH", b"lo", 0))[:18])[1]
fcntl.ioctl(s, SIOCSIFFLAGS, struct.pack("16sH", b"lo", flags | IFF_MULTICAST))
EOF
echo "[base_ab] $side: RMW=$RMW_IMPLEMENTATION CYCLONEDDS_URI=${CYCLONEDDS_URI:-} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

mkdir -p "$out"
rm -f "$out/mute"
nice -n -5 python3 "$tools/fake_base.py" --pty-link /dev/stmcom --log "$out/fake.jsonl" \
  --seconds 120 --mute-file "$out/mute" &
fake=$!
sleep 1

# robot_state_publisher runs beside either driver, as on the robot; it is
# also where controller_manager 4.48 gets robot_description from.
python3 - "$(ros2 pkg prefix mower_description)/share/mower_description/mower_robot/real_robot.xacro" \
  "$out/rsp.yaml" <<'EOF'
import sys, xacro, yaml
urdf = xacro.process_file(sys.argv[1]).toxml()
with open(sys.argv[2], "w") as f:
    yaml.safe_dump({"robot_state_publisher": {"ros__parameters": {"robot_description": urdf}}}, f)
EOF
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  --params-file "$out/rsp.yaml" >"$out/rsp.log" 2>&1 &
rsp=$!

if [ "$side" = A ]; then
  nice -n -5 ros2 launch mower_controller controller_test.launch.py \
    publish_robot_state_publisher:=false >"$out/driver.log" 2>&1 &
else
  nice -n -5 "${bin:-/mower_ws/install/mower_rs/lib/mower_rs/mower_base}" --ros-args \
    --params-file "$repo/src/mower_bringup/config/mower_rsd.yaml" >"$out/driver.log" 2>&1 &
fi
driver=$!

python3 "$tools/base_harness.py" --out "$out/run.json" --label "$side" \
  --mute-file "$out/mute" "$@"
status=$?

# SIGTERM: a non-interactive shell starts background jobs with SIGINT ignored.
stop() {
  kill -TERM "$1" 2>/dev/null
  for _ in $(seq 50); do kill -0 "$1" 2>/dev/null || return; sleep 0.1; done
  kill -KILL "$1" 2>/dev/null
}
stop "$driver"
stop "$rsp"
stop "$fake"
exit $status
