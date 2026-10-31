#!/bin/bash
# Data collection helper for the mower_recorder data_collection profile
# (docker-compose.data-collection.yaml; procedure: docs/資料收集錄製程序.md).
#
#   sudo mower-data-collection.sh start        stop the app camera (mower-camera.service),
#                                              the update timer and the app's recorder
#                                              (docker-compose.yaml `recorder`), start the container
#   mower-data-collection.sh status            camera rate + recorder status
#   mower-data-collection.sh record-start      /mower_recorder/start
#   mower-data-collection.sh record-stop       /mower_recorder/stop (finalizes both mcaps)
#   mower-data-collection.sh smoke [SECONDS]   30 s smoke bag, then check it
#   mower-data-collection.sh check [RUN_ID] [mower-check-run options]
#   sudo mower-data-collection.sh stop         record-stop, stop the container,
#                                              give the camera and the recorder back to the app
#   mower-data-collection.sh upload            upload every pending run (manifest last)
#   mower-data-collection.sh verify RUN_ID     R2 manifest vs objects
#
# Settings (optional, environment): DC_COMPOSE_FILE (default
# /opt/mower/docker-compose.data-collection.yaml, else next to this script's
# parent), ENV_FILE (default /opt/mower/.env), MAIN_COMPOSE_FILE (the robot
# stack, default /opt/mower/docker-compose.yaml).
#
# With RUST_RECORDER true (ENV_FILE, or the environment as compose reads it)
# mower_rsd's bag module owns /mower_recorder/* (E5-5): `start` writes its
# request file, recorder_profile.json in MOWER_STATE_DIR (the data_collection
# profile, the camera arguments, the session file, the upload policy), and
# starts only the camera (the compose file's data_collection_camera:
# gscam and its static TFs); `stop` deletes the file and the module goes back
# to the default profile. The commands, their output and exit codes are the
# same either way.
#
# E5-6: with CAMERA_DRIVER=gstreamer (the default) the module runs the camera
# itself (its camera_front instance, asked by the request file's
# camera.driver), so no container is started: the ROS commands run in the
# robot stack's lawan_node (sourcing ROS themselves, not through its
# entrypoint, which would sync the STM32 firmware), and the collection runs
# while the request file is there. CAMERA_DRIVER=gscam is E5-5's camera-only
# service (the roll-back).
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
compose_file=${DC_COMPOSE_FILE:-}
if [ -z "$compose_file" ]; then
  for c in /opt/mower/docker-compose.data-collection.yaml \
           "$here/../docker-compose.data-collection.yaml"; do
    if [ -f "$c" ]; then compose_file=$c; break; fi
  done
fi
if [ -z "$compose_file" ]; then
  echo "docker-compose.data-collection.yaml not found (set DC_COMPOSE_FILE)" >&2
  exit 2
fi
env_file=${ENV_FILE:-/opt/mower/.env}
env_args=()
if [ -f "$env_file" ]; then env_args=(--env-file "$env_file"); fi
state=/run/mower-data-collection.state   # timer / recorder state to restore on stop
svc=data_collection
bags=/home/mower/.mower/bags            # path inside the container
profile_args=()                         # data_collection_camera's profile (RUST_RECORDER)

compose() { docker compose "${env_args[@]}" -f "$compose_file" ${profile_args[@]+"${profile_args[@]}"} "$@"; }
# The robot stack's idle recorder (the app's 錄話題 button) owns
# /mower_recorder/* too; it steps aside for the run and comes back after it.
main_compose=${MAIN_COMPOSE_FILE:-/opt/mower/docker-compose.yaml}
stack() { docker compose "${env_args[@]}" -f "$main_compose" "$@"; }
app_recorder_running() {
  [ -f "$main_compose" ] && [ -n "$(stack ps -q --status running recorder 2>/dev/null)" ]
}
running() { [ -n "$(compose ps -q --status running "$svc" 2>/dev/null)" ]; }
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
# RUST_RECORDER as compose sees it, true in robot.launch.py's _truthy spellings.
rust_recorder_on() {
  case "$(setting RUST_RECORDER | tr '[:upper:]' '[:lower:]')" in true|1|yes|on) return 0 ;; *) return 1 ;; esac
}
# E5-5: the bag module's request file, on the host (the state dir compose
# mounts at /home/mower/.mower) and what goes in it (paths as the container
# sees them; the camera arguments as compose passes them to the camera).
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
    printf '"driver": %s}, ' "$(json_str "$camera_driver")"
    printf '"session_file": "/home/mower/.mower/data_collection_session.yaml", '
    printf '"upload_policy": %s}\n' "$(json_str "$(setting DATA_COLLECTION_UPLOAD_POLICY wifi_or_dock)")"
  } > "$tmp"
  chmod 644 "$tmp"
  mv -f "$tmp" "$request"
}
camera_driver=$(setting CAMERA_DRIVER gstreamer)
if rust_recorder_on; then
  svc=data_collection_camera
  profile_args=(--profile camera_only)
fi
# E5-6: the module's own camera (RUST_RECORDER and CAMERA_DRIVER=gstreamer)
module_camera() { rust_recorder_on && [ "$camera_driver" = gstreamer ]; }
# docker-entrypoint.sh sources ROS + the workspace (firmware sync is off here)
ros() { compose exec -T "$svc" /usr/local/bin/docker-entrypoint.sh "$@"; }
if module_camera; then
  # lawan_node's environment by hand: its entrypoint would sync the firmware
  ros() {
    stack exec -T lawan_node bash -c \
      'source "/opt/ros/$ROS_DISTRO/setup.bash" && source "$WORKSPACE/install/local_setup.bash" && exec "$@"' ros "$@"
  }
  running() { [ -f "$request" ] && [ -n "$(stack ps -q --status running lawan_node 2>/dev/null)" ]; }
fi
need_root() { [ "$(id -u)" -eq 0 ] || { echo "run with sudo" >&2; exit 1; }; }
need_running() { running || { echo "data_collection is not running (sudo $0 start)" >&2; exit 1; }; }
latest_run() { ros bash -c "ls -td $bags/*/ 2>/dev/null | head -1" | tr -d '\r' | sed 's#/$##'; }

case "${1:-}" in
  start)
    need_root
    if systemctl is-active --quiet mower-update.timer; then
      echo "timer=active" > "$state"
      systemctl stop mower-update.timer     # no image update / restart mid-run
    else
      echo "timer=inactive" > "$state"
    fi
    systemctl stop mower-camera.service     # it holds /dev/video0 (app has no video now)
    if module_camera; then
      write_request                         # the bag module takes data_collection and runs the camera
    elif rust_recorder_on; then
      write_request                         # the bag module takes data_collection while idle
      compose up -d "$svc"                  # the camera only
    else
      if app_recorder_running; then
        stack stop recorder                 # SIGINT: a bag it was recording is finalized
        echo "recorder=running" >> "$state"
      fi
      compose up -d
    fi
    echo "waiting for /camera/front/image_raw/compressed ..."
    for _ in $(seq 1 30); do
      if ros ros2 topic list 2>/dev/null | grep -q '^/camera/front/image_raw/compressed'; then
        echo "camera is up; next: $0 status, then $0 smoke"
        exit 0
      fi
      sleep 2
    done
    echo "camera topic did not appear: docker compose -f $compose_file logs $svc" >&2
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
      module_camera || compose down         # stop_signal SIGINT: clean shutdown
    fi
    rm -f "$request"                        # RUST_RECORDER: the module's default profile again
    if module_camera; then
      sleep 3                               # camera_front sees that within 2 s and frees /dev/video0
    fi
    systemctl start mower-camera.service
    if grep -q '^recorder=running' "$state" 2>/dev/null; then
      stack up -d recorder
    fi
    if grep -q '^timer=active' "$state" 2>/dev/null; then
      systemctl start mower-update.timer
    fi
    rm -f "$state"
    ;;
  upload|verify)
    cmd=$1; shift
    if [ "$cmd" = upload ]; then args=(upload --all --root "$bags" "$@"); else args=(verify "$@"); fi
    if running; then
      ros ros2 run mower_recorder mower-bag --env-file /home/mower/.mower/r2.env "${args[@]}"
    else
      compose run --rm --no-deps -T "$svc" /usr/local/bin/docker-entrypoint.sh \
        ros2 run mower_recorder mower-bag --env-file /home/mower/.mower/r2.env "${args[@]}"
    fi
    ;;
  *)
    sed -n '2,20p' "$0"
    exit 2
    ;;
esac
