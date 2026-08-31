# Mower public relay

This stack runs on the Ubuntu 22.04 server and exposes the mower's single front
camera through MediaMTX/WHEP.

The mower joins the WireGuard network as `10.77.0.2`. MediaMTX pulls
`rtsp://10.77.0.2:8554/front` only while a viewer is connected, so the 4G video
uplink is not continuously active.

## Required router forwarding

- `51820/udp` -> `192.168.10.200:51820` (WireGuard)
- `8189/udp` -> `192.168.10.200:8189` (WebRTC media)
- `8189/tcp` -> `192.168.10.200:8189` (WebRTC TCP fallback)

WHEP signaling on `127.0.0.1:8889` is intentionally not exposed directly. It
must be published as `https://camera.fxrbindi.com` through the authenticated
reverse proxy or Cloudflare Tunnel.

The Cloudflare Tunnel publishes:

- `camera.fxrbindi.com` -> `http://127.0.0.1:8889`
- `control.fxrbindi.com` -> `http://10.77.0.2:9090`

Protect both hostnames with a Cloudflare Access service-auth policy before the
mower joins the VPN. Native app builds send `CF-Access-Client-Id` and
`CF-Access-Client-Secret` headers supplied with Dart defines.

Do not treat a long-lived service token compiled into an APK/IPA as production
user authentication: it is extractable and not independently revocable per
operator/device. Replace it with user login or short-lived per-device
credentials before field release, and rotate any token already distributed.

Copy `cloudflared-config.example.yml` to the gitignored
`cloudflared-config.yml`, and copy only the tunnel UUID credential JSON to the
gitignored `tunnel-credentials.json`. The container intentionally mounts those
two files rather than the whole `~/.cloudflared` directory (which can also
contain the account-wide `cert.pem`). Override `CLOUDFLARED_CONFIG_PATH` and
`CLOUDFLARED_CREDENTIALS_PATH` if they live elsewhere.

## Operations

```bash
docker compose pull
docker compose up -d
docker compose logs --tail=100 mediamtx
```
