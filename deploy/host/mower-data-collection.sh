#!/bin/bash
# Data collection helper: mower_rsd's bag module with the data_collection
# profile (procedure: docs/資料收集錄製程序.md).
#
#   sudo mower-data-collection.sh start        stop the app camera (mower-camera.service) and the
#                                              update timer, hand the module the profile and the camera
#   mower-data-collection.sh status            camera rate + recorder status
#   mower-data-collection.sh record-start      /mower_recorder/start
#   mower-data-collection.sh record-stop       /mower_recorder/stop (finalizes both mcaps)
#   mower-data-collection.sh smoke [SECONDS]   30 s smoke bag, then check it
#   mower-data-collection.sh check [RUN_ID] [mower-check-run options]
#   sudo mower-data-collection.sh stop         record-stop, take the profile back,
#                                              give the camera back to the app
#   mower-data-collection.sh upload            upload every pending run (manifest last)
#   mower-data-collection.sh verify RUN_ID     R2 manifest vs objects
#
# Settings (optional, environment): ENV_FILE (default /opt/mower/.env),
# MAIN_COMPOSE_FILE (the robot stack, default /opt/mower/docker-compose.yaml).
#
# `start` writes the module's request file, recorder_profile.json in
# MOWER_STATE_DIR (the data_collection profile, the camera arguments, the
# session file, the upload policy): the module applies it while idle and runs
# the camera itself (its camera_front instance). `stop` deletes the file and
# the module goes back to the default profile. The collection runs while the
# file is there. No container is started: the ROS commands run in the robot
# stack's lawan_node (sourcing ROS themselves, not through its entrypoint,
# which would sync the STM32 firmware). RUST_RECORDER=false in ENV_FILE means
# no bag module, so `start` refuses.
#
# upload and verify run mower_bag in the image (crates/mower_recorder_core),
# in lawan_node or, with the stack down, in a one-off container of it.
# smoke and check run the Python mower-check-run there (design D9).
set -euo pipefail

env_file=${ENV_FILE:-/opt/mower/.env}
env_args=()
if [ -f "$env_file" ]; then env_args=(--env-file "$env_file"); fi
state=/run/mower-data-collection.state   # timer state to restore on stop
bags=/home/mower/.mower/bags            # path inside the container
r2_env=/home/mower/.mower/r2.env        # path inside the container

main_compose=${MAIN_COMPOSE_FILE:-/opt/mower/docker-compose.yaml}
stack() { docker compose "${env_args[@]}" -f "$main_compose" "$@"; }
stack_up() { [ -n "$(stack ps -q --status running lawan_node 2>/dev/null)" ]; }
# A setting as compose sees it: the environment, else the last line of
# $env_file (a trailing comment and quotes dropped), else $2.
setting() {
  local name=$1 v=
  if [ -n "${!name+x}" ]; then
    v=${!name}
  elif [ -f "$env_file" ]; then
    v=$(sed -n "s/^[[:space:]]*$name[[:space:]]*=//p" "$env_file" | tail -n 1)
  fi
  v=${v%%[[:space:]]#*}
  v=${v//\"/}
  v=${v//\'/}
  read -r v <<< "$v" || true
  printf '%s' "${v:-${2:-}}"
}
# RUST_RECORDER as compose sees it (unset = true, compose's default), true in
# robot.launch.py's _truthy spellings.
rust_recorder_on() {
  case "$(setting RUST_RECORDER true | tr '[:upper:]' '[:lower:]')" in true|1|yes|on) return 0 ;; *) return 1 ;; esac
}
# The bag module's request file, on the host (the state dir compose mounts at
# /home/mower/.mower) and what goes in it (paths as the container sees them).
request=$(setting MOWER_STATE_DIR /home/cat/.mower)/recorder_profile.json
json_str() { local v=$1; v=${v//\\/\\\\}; v=${v//\"/\\\"}; printf '"%s"' "$v"; }
write_request() {
  local tmp="$request.tmp"
  {
    printf '{"profile": "data_collection", "camera": {'
    printf '"camera_device": %s, ' "$(json_str "$(setting CAMERA_DEVICE /dev/video0)")"
    printf '"camera_size": %s, ' "$(json_str "$(setting CAMERA_SIZE 1280x720)")"
    printf '"camera_fps": %s, ' "$(json_str "$(setting CAMERA_FPS 25)")"
    printf '"record_fps": %s, ' "$(json_str "$(setting CAMERA_RECORD_FPS 10)")"
    printf '"calib_dir": "/home/mower/.mower/calib", '
    printf '"driver": "gstreamer"}, '
    printf '"session_file": "/home/mower/.mower/data_collection_session.yaml", '
    printf '"upload_policy": %s}\n' "$(json_str "$(setting DATA_COLLECTION_UPLOAD_POLICY wifi_or_dock)")"
  } > "$tmp"
  chmod 644 "$tmp"
  mv -f "$tmp" "$request"
}
# lawan_node's environment by hand: its entrypoint would sync the firmware
ros() {
  stack exec -T lawan_node bash -c \
    'source "/opt/ros/$ROS_DISTRO/setup.bash" && source "$WORKSPACE/install/local_setup.bash" && exec "$@"' ros "$@"
}
running() { [ -f "$request" ] && stack_up; }
need_root() { [ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }; }
need_running() { running || { echo "data_collection is not running (sudo $0 start)" >&2; exit 1; }; }
latest_run() { ros bash -c "ls -td $bags/*/ 2>/dev/null | head -1" | tr -d '\r' | sed 's#/$##'; }

case "${1:-}" in
  start)
    need_root
    if ! rust_recorder_on; then
      echo "RUST_RECORDER is off ($env_file): there is no bag module to record." \
        "Remove the line there and run 'docker compose up --detach' in $(dirname "$main_compose")," \
        "then start the data collection again. Nothing was changed." >&2
      exit 1
    fi
    if systemctl is-active --quiet mower-update.timer; then
      echo "timer=active" > "$state"
      systemctl stop mower-update.timer     # no image update / restart mid-run
    else
      echo "timer=inactive" > "$state"
    fi
    systemctl stop mower-camera.service     # it holds /dev/video0 (app has no video now)
    write_request                           # the bag module takes data_collection and runs the camera
    echo "waiting for /camera/front/image_raw/compressed ..."
    for _ in $(seq 1 30); do
      if ros ros2 topic list 2>/dev/null | grep -q '^/camera/front/image_raw/compressed'; then
        echo "camera is up; next: $0 status, then $0 smoke"
        exit 0
      fi
      sleep 2
    done
    echo "camera topic did not appear: docker compose -f $main_compose logs lawan_node | grep camera_front" >&2
    exit 1
    ;;
  status)
    need_running
    ros timeout 6 ros2 topic hz /camera/front/image_raw/compressed 2>/dev/null | tail -2 || true
    ros ros2 topic echo --once /mower_recorder/status 2>/dev/null | sed -n 's/^data: //p' || true
    ;;
  record-start)
    need_running
    ros ros2 service call /mower_recorder/start std_srvs/srv/Trigger | tail -1
    ;;
  record-stop)
    need_running
    ros ros2 service call /mower_recorder/stop std_srvs/srv/Trigger | tail -1
    ;;
  smoke)
    need_running
    secs=${2:-30}
    ros ros2 service call /mower_recorder/start std_srvs/srv/Trigger | tail -1
    echo "smoke bag: recording ${secs} s ..."
    sleep "$secs"
    ros ros2 service call /mower_recorder/stop std_srvs/srv/Trigger | tail -1
    run=$(latest_run)
    ros ros2 run mower_recorder mower-check-run "$run"
    ;;
  check)
    need_running
    shift
    if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then run="$bags/$1"; shift; else run=$(latest_run); fi
    ros ros2 run mower_recorder mower-check-run "$run" "$@"
    ;;
  stop)
    need_root
    if running; then
      ros ros2 service call /mower_recorder/stop std_srvs/srv/Trigger | tail -1 || true
    fi
    rm -f "$request"                        # the module's default profile again
    sleep 3                                 # camera_front sees that within 2 s and frees /dev/video0
    systemctl start mower-camera.service
    if grep -q '^timer=active' "$state" 2>/dev/null; then
      systemctl start mower-update.timer
    fi
    rm -f "$state"
    ;;
  upload|verify)
    cmd=$1; shift
    if [ "$cmd" = upload ]; then args=(upload --all --root "$bags" "$@"); else args=(verify "$@"); fi
    if stack_up; then
      ros mower_bag --env-file "$r2_env" "${args[@]}"
    else
      # no STM32 here: this is the image's entrypoint, so keep it off the firmware
      stack run --rm --no-deps -T -e MOWER_FIRMWARE_SYNC=0 lawan_node mower_bag --env-file "$r2_env" "${args[@]}"
    fi
    ;;
  *)
    sed -n '2,16p' "$0"
    exit 2
    ;;
esac
