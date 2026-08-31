# WebRTC front-camera streaming

The manual-control page shows the mower's single front camera as a low-latency
WebRTC stream. MediaMTX runs on both the mower and the public Ubuntu relay.
Video is H.264 and is kept separate from rosbridge control/status traffic.

## Production data flow

```text
Camera device (/dev/video* or a temporary lavfi test pattern)
  │ ffmpeg H.264 baseline, started on demand
  ▼
Mower MediaMTX: rtsp://10.77.0.2:8554/front
  │ RTSP/TCP over WireGuard through the mower's 4G connection
  ▼
Ubuntu relay MediaMTX: 127.0.0.1:8889/front/whep
  │ Cloudflare Tunnel for HTTPS WHEP signaling
  ▼
Flutter app: https://camera.fxrbindi.com/front/whep
  │
  └─ WebRTC media to the relay on 8189/udp or 8189/tcp
```

Control follows a separate path:

```text
Flutter app
  -> wss://control.fxrbindi.com
  -> Cloudflare Tunnel
  -> ws://10.77.0.2:9090 over WireGuard
  -> rosbridge on the mower
```

The mower is `10.77.0.2` and the relay is `10.77.0.1` inside WireGuard. The
mower initiates the VPN connection, so it does not need a public 4G address.

## Components

- `mediamtx.yml` runs on the mower. Its one `front` path starts ffmpeg only
  while a downstream reader exists.
- `docker-compose.yaml` runs the mower MediaMTX/ffmpeg container.
- `deploy/server/` runs the public relay MediaMTX. Its WHEP signaling listener
  is loopback-only; Cloudflare Tunnel publishes it.
- `mower_lawer_app` reads `front/whep` with `flutter_webrtc`.

## Router forwarding

The router in front of the Ubuntu relay must forward:

- `51820/udp` to `192.168.10.200:51820` for WireGuard.
- `8189/udp` to `192.168.10.200:8189` for WebRTC media.
- `8189/tcp` to `192.168.10.200:8189` as a WebRTC fallback.

Ports `8554`, `8889`, and `9090` must not be exposed publicly.

## Attaching the physical front camera

Find the V4L2 color device:

```bash
v4l2-ctl --list-devices
```

Replace the temporary `lavfi` input in the mower's `mediamtx.yml`, for example:

```yaml
paths:
  front:
    runOnDemand: >
      ffmpeg -hide_banner
      -f v4l2 -framerate 30 -video_size 1280x720 -i /dev/video0
      -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p
      -profile:v baseline -level 3.1 -bf 0 -g 50
      -f rtsp rtsp://localhost:$RTSP_PORT/$MTX_PATH
    runOnDemandRestart: true
    runOnDemandCloseAfter: 10s
```

Expose the selected device in `docker-compose.yaml`:

```yaml
services:
  mediamtx:
    devices:
      - /dev/video0:/dev/video0
```

For a RealSense-class depth camera, WHEP uses its RGB/color V4L2 stream. Depth
images and camera calibration continue to travel through ROS 2 topics.

## LAN fallback

The on-mower player remains available on the LAN:

```text
http://<mower-lan-ip>:8889/front
```

Build the Flutter app with an empty `CAMERA_BASE_URL` and
`USE_SAVED_ROBOT_IP=true` to derive the LAN WHEP URL from the configured mower
IP.
