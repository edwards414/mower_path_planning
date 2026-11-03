#!/bin/bash
# Front camera publisher: USB MJPG camera -> Rockchip MPP (hardware JPEG
# decode + H.264 encode, ~2 % of one core) -> RTSP publish into the mediamtx
# container, which serves it to the app over WebRTC/WHEP. Runs on the host
# (mower-camera.service) because the MPP libraries and /dev/mpp_service live
# here, not in the container. GStreamer: gstreamer1.0-rockchip1 (vendor
# image) + gstreamer1.0-{tools,plugins-base,plugins-good,plugins-bad,rtsp}.
#
# Settings in /opt/mower/.env (all optional):
#   CAMERA_DEVICE    /dev/video0
#   CAMERA_SIZE      1280x720          MJPG mode of the camera
#   CAMERA_FPS       25
#   CAMERA_BPS       2000000           H.264 bit rate, bit/s (CBR)
#   CAMERA_GOP       10                frames between key frames
#   CAMERA_RTSP_URL  where mediamtx accepts the publish (default the local
#                    mediamtx, rtsp://127.0.0.1:8554/front)
#   CAMERA_ON_DEMAND 1                 1: the camera runs only while it is
#                                      wanted (below); 0: all the time
#
# The camera's auto exposure lowers the frame rate in dim light (about 5 fps
# indoors at close range, 20+ fps in daylight); that is the camera, not the
# encoder.
#
# On demand: an idle robot does not capture, decode and encode 25 frames a
# second for nobody (2026-11-03: the SoC sat at 69-73 °C idle, near its
# 85 °C throttling point). The camera runs while it is wanted:
#   - a reader asked MediaMTX for the path (the app's WHEP, an RTSP client):
#     mediamtx.yml's runOnDemand touches DEMAND_FILE once a second while the
#     path is requested and removes it when MediaMTX stops it
#     (runOnDemandCloseAfter after the last reader left); a file older than
#     5 s counts as gone (MediaMTX was killed or restarted);
#   - the path has readers (one that came while a recording had the camera
#     on: MediaMTX runs no runOnDemand for a path that is already live);
#   - MediaMTX records the path (a recorder run, `record: true` in the path's
#     config): a recording is no reader, so it would not start the camera.
# The last two come from MediaMTX's loopback API, once a second. A first
# viewer waits while the camera starts (about 2 s): MediaMTX holds its WHEP
# answer up to runOnDemandStartTimeout (8 s; mower_agent relays WHEP with a
# 10 s timeout, the app waits 12 s). A recording's video starts when the
# camera does. The data-collection helper stops this service while it holds
# /dev/video0 and starts it again afterwards.
set -u
ENV_FILE=${ENV_FILE:-/opt/mower/.env}
GST_LAUNCH=${GST_LAUNCH:-gst-launch-1.0}              # the tests put a fake here
DEMAND_FILE=${DEMAND_FILE:-/run/mower-camera/demand}  # compose's bind mount
MEDIAMTX_API=${MEDIAMTX_API:-http://127.0.0.1:9997}

env_get() {  # env_get KEY DEFAULT — reads one key without exporting the rest
  local v
  v=$(sed -nE "s/^$1=([^#]*).*/\1/p" "$ENV_FILE" 2>/dev/null | tail -1 | tr -d '"'"'"' ')
  printf '%s' "${v:-$2}"
}

dev=$(env_get CAMERA_DEVICE /dev/video0)
size=$(env_get CAMERA_SIZE 1280x720)
w=${size%x*}; h=${size#*x}
fps=$(env_get CAMERA_FPS 25)
bps=$(env_get CAMERA_BPS 2000000)
url=$(env_get CAMERA_RTSP_URL rtsp://127.0.0.1:8554/front)
gop=$(env_get CAMERA_GOP 10)
on_demand=$(env_get CAMERA_ON_DEMAND 1)
path=${url##*/}

# gop = 10 frames: a key frame every 0.4 s at 25 fps and still every 2 s
# when auto exposure drops the camera to 5 fps, so a new viewer joins fast;
# SPS/PPS repeated at every IDR (config-interval=1) for mid-stream joins.
camera() {
  exec "$GST_LAUNCH" -e \
    v4l2src device="$dev" \
    ! "image/jpeg,width=$w,height=$h,framerate=$fps/1" \
    ! mppjpegdec ! "video/x-raw,format=NV12" \
    ! mpph264enc bps="$bps" bps-max=$((bps * 5 / 4)) gop="$gop" rc-mode=cbr profile=baseline \
    ! h264parse config-interval=1 \
    ! rtspclientsink location="$url" protocols=tcp
}

if [ "$on_demand" = 0 ]; then
  echo "mower-camera: $dev ${w}x${h}@${fps} MJPG -> mpph264enc ${bps} bit/s -> $url (always on)"
  camera
fi
echo "mower-camera: $dev ${w}x${h}@${fps} MJPG -> mpph264enc ${bps} bit/s -> $url (on demand)"

api() {  # api PATH: MediaMTX's answer, or nothing (MediaMTX down, the path not live)
  curl -fsS --max-time 1 "$MEDIAMTX_API$1" 2>/dev/null
}

why() {  # prints why the camera is wanted, or fails
  local m body
  if m=$(stat -c %Y "$DEMAND_FILE" 2>/dev/null) && [ $(($(date +%s) - m)) -le 5 ]; then
    echo "a reader asked for $path"; return 0
  fi
  body=$(api "/v3/paths/get/$path")
  if grep -q '"readers"' <<<"$body" && ! grep -Eq '"readers": *\[ *\]' <<<"$body"; then
    echo "$path has readers"; return 0
  fi
  if api "/v3/config/paths/get/$path" | grep -Eq '"record": *true'; then
    echo "MediaMTX records $path"; return 0
  fi
  return 1
}

pid=
stop_camera() {
  [ -n "$pid" ] || return 0
  kill -INT "$pid" 2>/dev/null   # -e: EOS first, so the RTSP session closes cleanly
  for _ in $(seq 30); do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
  kill -KILL "$pid" 2>/dev/null
  wait "$pid" 2>/dev/null
  pid=
}
trap 'stop_camera; exit 0' TERM INT

retry_at=0
while :; do
  if reason=$(why); then
    if [ -n "$pid" ] && ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid"; rc=$?; pid=
      echo "mower-camera: the pipeline exited ($rc) while wanted; again in 3 s"
      retry_at=$((SECONDS + 3))
    fi
    if [ -z "$pid" ] && [ "$SECONDS" -ge "$retry_at" ]; then
      echo "mower-camera: on ($reason)"
      camera &
      pid=$!
    fi
  elif [ -n "$pid" ]; then
    echo "mower-camera: off (no reader, no recording)"
    stop_camera
  fi
  sleep 1
done
