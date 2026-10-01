# WebRTC front-camera streaming

The manual-control page shows the mower's single front camera as a low-latency
WebRTC stream served by MediaMTX on the mower. Video is H.264 and is kept
separate from rosbridge control/status traffic.

> **2026-09-30:** this page used to describe the WireGuard + home Ubuntu relay
> (`camera.fxrbindi.com`, `control.fxrbindi.com → 10.77.0.2`). That setup is
> retired (docs/BACKEND_ARCHITECTURE.md §8–9, stage 3); `deploy/server/` is
> legacy. The current path is below.

## Production data flow

```text
USB camera (/dev/video0, MJPG)
  │ host service mower-camera.service (deploy/host/mower-camera.sh):
  │ Rockchip MPP hardware JPEG decode + H.264 encode, published continuously
  ▼
Mower MediaMTX (deploy/mediamtx.yml): rtsp://127.0.0.1:8554/front, path
`front` with `source: publisher`, re-served over WebRTC/WHEP on :8889
  │
  ├─ same Wi-Fi: app -> http://<robot-ip>:8889/front/whep,
  │              media direct on UDP 8189
  │
  └─ elsewhere:  app -> fleet backend /v1/robots/<id>/http/front/whep
                 -> RobotHub -> mower_agent -> 127.0.0.1:8889;
                 media direct if ICE finds a path, else Cloudflare TURN
                 (mower_agent writes the TURN credentials into MediaMTX's
                 webrtcICEServers2 through the loopback API :9997)
```

Control traffic (rosbridge / ws_bridge) takes the same two routes: LAN
directly, otherwise the backend relay (docs/BACKEND_ARCHITECTURE.md).

## Components

- `deploy/host/mower-camera.sh` + `mower-camera.service` (host, not the
  container: the MPP libraries and `/dev/mpp_service` live there) publish the
  feed. The MediaMTX container needs no camera device.
- `deploy/mediamtx.yml` is the one MediaMTX config for every robot (WHEP :8889,
  WebRTC media UDP 8189, API bound to 127.0.0.1:9997).
- `mower_agent` forwards only `/<path>/whep` signaling and refreshes the TURN
  credentials; see BACKEND_ARCHITECTURE.md §8 for the security boundary.
- `mower_lawer_app` reads `front/whep` with `flutter_webrtc`.

Ports `8554` and `9997` must never be exposed; nothing needs forwarding on the
robot's router (the relay connection is outbound).

## Camera settings

On the robot, set any of these in `/opt/mower/.env` and restart
`mower-camera.service`:

| Key | Default | |
|---|---|---|
| `CAMERA_DEVICE` | `/dev/video0` | V4L2 device (`v4l2-ctl --list-devices`) |
| `CAMERA_SIZE` | `1280x720` | MJPG mode of the camera |
| `CAMERA_FPS` | `25` | |
| `CAMERA_BPS` | `2000000` | H.264 bit rate (CBR) |
| `CAMERA_GOP` | `10` | frames between key frames |
| `CAMERA_RTSP_URL` | `rtsp://127.0.0.1:8554/front` | where MediaMTX accepts the publish |

Without a camera, publish a test pattern from the host (the command is in the
comment of `deploy/mediamtx.yml`). The `lavfi` `runOnDemand` recipe in the
repository-root `mediamtx.yml` / `docker-compose.yaml` is for development only.

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
