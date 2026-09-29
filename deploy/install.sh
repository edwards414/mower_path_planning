#!/bin/bash
# Install or refresh the robot deployment on the LubanCat.
#
#   git clone / copy this deploy/ folder to the robot, then:
#   sudo ./install.sh              # user defaults to the sudo caller (cat)
#
# Puts the stack in /opt/mower, the udev names in /etc/udev/rules.d, the
# systemd units in /etc/systemd/system, creates ~/.mower for the container
# and enables everything on boot. Re-run after changing anything in deploy/.
# It does not pull the image: run  sudo /opt/mower/host/mower-update.sh  next
# (after `docker login ghcr.io` if the package is private).
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
user=${MOWER_USER:-${SUDO_USER:-cat}}
home=$(getent passwd "$user" | cut -d: -f6)
if [ -z "$home" ]; then
  echo "user $user not found (set MOWER_USER)" >&2
  exit 1
fi
if [ "$(id -u)" -ne 0 ]; then
  echo "run with sudo" >&2
  exit 1
fi
state_dir="$home/.mower"

echo "== /opt/mower (user=$user state=$state_dir)"
mkdir -p /opt/mower/host
install -m 644 "$here/docker-compose.yaml" "$here/mediamtx.yml" /opt/mower/
# The u-blox driver runs inside the main image now (config in
# mower_bringup); the old `--profile gps` service kept its yaml here.
rm -rf /opt/mower/gps
install -m 755 "$here/host/mower-update.sh" "$here/host/mower-host-request.sh" "$here/host/mower-link-status.py" "$here/host/mower-camera.sh" /opt/mower/host/
install -m 755 "$here/host/mower-pair" /opt/mower/host/
ln -sf /opt/mower/host/mower-pair /usr/local/bin/mower-pair
if [ ! -f /opt/mower/.env ]; then
  sed "s#/home/cat/.mower#$state_dir#" "$here/.env.example" > /opt/mower/.env
  echo "   wrote /opt/mower/.env (edit IMAGE_TAG / ROSBRIDGE_ADDRESS there)"
fi
mkdir -p "$state_dir"/zone_record "$state_dir"/sites "$state_dir"/bags
chown -R "$user:$user" "$state_dir"

echo "== identity / pairing"
if ! command -v qrencode >/dev/null 2>&1; then
  apt-get install -y -qq qrencode >/dev/null 2>&1 || echo "   (qrencode not installed; mower-pair prints the URL only)"
fi
/opt/mower/host/mower-pair --state-dir "$state_dir" --owner "$user" --env /opt/mower/.env --no-qr | sed 's/^/   /'
echo "   sudo mower-pair   # prints the pairing QR code for the app"

echo "== camera (USB MJPG -> Rockchip MPP H.264 -> mediamtx)"
# gstreamer1.0-rockchip1 (mpph264enc/mppjpegdec) ships with the vendor image;
# the generic GStreamer pieces around it do not.
if ! command -v gst-launch-1.0 >/dev/null 2>&1 || ! gst-inspect-1.0 rtspclientsink >/dev/null 2>&1; then
  apt-get install -y -qq --no-install-recommends gstreamer1.0-tools gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-rtsp >/dev/null 2>&1 \
    || echo "   (GStreamer install failed; mower-camera.service will keep retrying)"
fi
gst-inspect-1.0 mpph264enc >/dev/null 2>&1 && echo "   mpph264enc available (hardware H.264)" \
  || echo "   WARNING: mpph264enc missing (gstreamer1.0-rockchip1); the front camera will not publish"

echo "== kernel (DDS)"
install -m 644 "$here/host/99-mower-dds.conf" /etc/sysctl.d/99-mower-dds.conf
sysctl -q -p /etc/sysctl.d/99-mower-dds.conf || true
ip link set lo multicast on || true

echo "== udev"
install -m 644 "$here/udev/99-mower.rules" /etc/udev/rules.d/99-mower.rules
udevadm control --reload-rules && udevadm trigger
sleep 1
for dev in stmcom imu_usb gps_rtk lte_at; do
  if [ -e "/dev/$dev" ]; then echo "   /dev/$dev -> $(readlink -f /dev/$dev)"; else echo "   /dev/$dev: not present"; fi
done

echo "== systemd"
for unit in mower.service mower-host-request.path mower-host-request.service mower-update.service mower-update.timer mower-update-check.service mower-update-check.timer mower-link-status.service mower-camera.service; do
  sed "s#/home/cat/.mower#$state_dir#g; s#/home/cat#$home#g; s#^User=cat#User=$user#" "$here/host/$unit" > "/etc/systemd/system/$unit"
done
# no RTC battery: ntpd is the only thing that sets the clock, so it must not
# be rate-limited to death by the DHCP hook's restarts at boot
install -D -m 644 "$here/host/ntpsec-mower.conf" /etc/systemd/system/ntpsec.service.d/10-mower.conf
systemctl daemon-reload
systemctl is-failed -q ntpsec.service && systemctl restart ntpsec.service || true
systemctl enable mower.service >/dev/null
systemctl enable --now mower-host-request.path >/dev/null
systemctl enable --now mower-update.timer >/dev/null
systemctl enable --now mower-update-check.timer >/dev/null
systemctl enable --now mower-link-status.service >/dev/null
systemctl enable --now mower-camera.service >/dev/null
echo "   enabled mower.service (stack on boot), mower-host-request.path (app-triggered update/restart/poweroff),"
echo "           mower-update.timer (hourly channel check, MOWER_AUTO_UPDATE=0 in .env disables),"
echo "           mower-update-check.timer (registry check every 5 min -> /robot/info update.available),"
echo "           mower-link-status.service (LTE/Wi-Fi signal -> link_status.json for /robot/telemetry),"
echo "           mower-camera.service (front USB camera -> MPP H.264 -> mediamtx, CAMERA_* in .env)"

echo "== docker"
if ! docker info >/dev/null 2>&1; then
  echo "   docker is not running or not installed" >&2
  exit 1
fi
if ! grep -q ghcr.io /root/.docker/config.json 2>/dev/null; then
  echo "   no ghcr.io login for root: run  sudo docker login ghcr.io  if the image is private"
fi
echo
echo "next:  sudo /opt/mower/host/mower-update.sh     # pull \$IMAGE_TAG and start"
echo "       sudo systemctl status mower"
