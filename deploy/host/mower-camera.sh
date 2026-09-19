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
#
# The camera's auto exposure lowers the frame rate in dim light (about 5 fps
# indoors at close range, 20+ fps in daylight); that is the camera, not the
# encoder.
set -u
ENV_FILE=${ENV_FILE:-/opt/mower/.env}

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

echo "mower-camera: $dev ${w}x${h}@${fps} MJPG -> mpph264enc ${bps} bit/s -> $url"
# gop = 10 frames: a key frame every 0.4 s at 25 fps and still every 2 s
# when auto exposure drops the camera to 5 fps, so a new viewer joins fast;
# SPS/PPS repeated at every IDR (config-interval=1) for mid-stream joins.
exec gst-launch-1.0 -e \
  v4l2src device="$dev" \
  ! "image/jpeg,width=$w,height=$h,framerate=$fps/1" \
  ! mppjpegdec ! "video/x-raw,format=NV12" \
  ! mpph264enc bps="$bps" bps-max=$((bps * 5 / 4)) gop="$(env_get CAMERA_GOP 10)" rc-mode=cbr profile=baseline \
  ! h264parse config-interval=1 \
  ! rtspclientsink location="$url" protocols=tcp
