# WebRTC Camera Streaming (WHEP)

The app's manual-control page shows the robot's cameras as live **WebRTC**
streams pulled from an on-robot [MediaMTX](https://github.com/bluenviron/mediamtx)
media server using **WHEP** (WebRTC-HTTP Egress Protocol).

This replaces the previous approach of shipping raw `sensor_msgs/msg/Image`
frames over the rosbridge WebSocket, which was bandwidth-heavy (uncompressed
pixels) and high-latency. Video is now H.264-encoded and decoupled from
rosbridge — rosbridge (`ws://<robot-ip>:9090`) still carries control and status
only.

## Data flow

```
Camera device (/dev/video*  or  lavfi test pattern while no camera)
        │  ffmpeg (H.264 baseline, low-latency)  ── runOnDemand
        ▼
   MediaMTX  (on the robot, docker service `mediamtx`)
   ├─ path front  →  WHEP: http://<robot-ip>:8889/front/whep
   └─ path rear   →  WHEP: http://<robot-ip>:8889/rear/whep
        │  WebRTC (H.264)
        ▼
   Flutter app  (flutter_webrtc + RTCVideoView)
```

Signaling is a single HTTP POST of the SDP offer to the WHEP endpoint; MediaMTX
replies with the SDP answer (HTTP 201) and embeds its ICE candidates, so no
separate signaling server or trickle ICE is needed on a LAN.

## Configuration

- `mediamtx.yml` — MediaMTX config: two paths (`front`, `rear`), each with a
  `runOnDemand` ffmpeg producer (ffmpeg starts when the app connects, stops
  shortly after it disconnects).
- `docker-compose.yaml` — the `mediamtx` service (`bluenviron/mediamtx:latest-ffmpeg`,
  `network_mode: host`).

The app derives the WHEP URLs from the same robot IP it uses for rosbridge (port
`8889`); see `whepUrl()` in
`mower_lawer_app/lib/providers/mission_mock_provider.dart`.

## Running

Start just the camera server on the robot:

```bash
docker compose up -d mediamtx
```

Then in the app, set the robot IP to the robot's LAN address and open the
manual-control page. Front/rear toggle switches feeds.

Quick check without the app — MediaMTX serves a built-in player page:

```
http://<robot-ip>:8889/front
http://<robot-ip>:8889/rear
```

## No camera yet: synthetic test sources

Until a USB/v4l2 camera is wired, both feeds are ffmpeg **synthetic test
patterns** so the full WebRTC path can be verified end-to-end:

- `front` → `testsrc2` (animated test pattern)
- `rear`  → `smptebars` (SMPTE colour bars)

They are visually distinct so front/rear switching is obvious.

## Attaching a real camera

In `mediamtx.yml`, replace the `lavfi` input of the relevant path with the
camera device, e.g. a USB/v4l2 camera on `/dev/video0`:

```yaml
paths:
  front:
    runOnDemand: >
      ffmpeg -hide_banner
      -f v4l2 -framerate 30 -video_size 1280x720 -i /dev/video0
      -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p
      -profile:v baseline -level 3.1 -bf 0 -g 50
      -f rtsp rtsp://localhost:$RTSP_PORT/$MTX_PATH
    runOnDemandRestart: yes
```

Then expose the device to the container in `docker-compose.yaml`:

```yaml
  mediamtx:
    devices:
      - /dev/video0:/dev/video0
```

A RealSense-class front "depth camera" exposes its **colour** stream as a plain
v4l2 device (some `/dev/videoN`); capture that directly rather than going
through the ROS depth node. Confirm the index with `v4l2-ctl --list-devices`.

## Notes / gotchas

- `network_mode: host` is used so WebRTC ICE host candidates are reachable on
  the LAN with no port mapping. On Docker Desktop for macOS host networking runs
  inside a VM, so the WebRTC media flow is best verified on the robot (Linux) on
  the same LAN as the phone.
- The app talks to the WHEP endpoint over `http://` (cleartext). Android needs
  `usesCleartextTraffic="true"` + `INTERNET`; iOS needs the local-network ATS
  key. These are already set in the app's `AndroidManifest.xml` / `Info.plist`.
- `flutter_webrtc` is a native plugin: after adding it the app needs a full
  rebuild (and `pod install` on iOS), not hot reload.
