#!/bin/bash
# Test switch for the ROS-free branch image. Backs up .env and the compose file once,
# then sets IMAGE_TAG / NAV_COMPOSITION / RUST_DAEMON and recreates only lawan_node.
# usage: switch.sh <IMAGE_TAG> <NAV_COMPOSITION> <RUST_DAEMON> <compose-file-to-install|keep>
# restore: switch.sh restore
set -e
cd /opt/mower
if [ "$1" = restore ]; then
  sudo cp .env.bak-rosfree-test .env
  sudo cp docker-compose.yaml.bak-rosfree-test docker-compose.yaml
  sudo docker compose up -d lawan_node 2>&1 | tail -2
  sudo docker ps --format "{{.Names}} {{.Image}} {{.Status}}" | grep lawan
  exit 0
fi
TAG=$1; NC=$2; RD=$3; CF=$4
[ -f .env.bak-rosfree-test ] || sudo cp .env .env.bak-rosfree-test
[ -f docker-compose.yaml.bak-rosfree-test ] || sudo cp docker-compose.yaml docker-compose.yaml.bak-rosfree-test
[ "$CF" = keep ] || sudo cp "$CF" docker-compose.yaml
sudo sed -i "s/^IMAGE_TAG=.*/IMAGE_TAG=$TAG/" .env
sudo sed -i "/^NAV_COMPOSITION=/d; /^RUST_DAEMON=/d" .env
echo "NAV_COMPOSITION=$NC" | sudo tee -a .env >/dev/null
echo "RUST_DAEMON=$RD" | sudo tee -a .env >/dev/null
shift 4 2>/dev/null || true
for kv in "$@"; do key=${kv%%=*}; sudo sed -i "/^$key=/d" .env; echo "$kv" | sudo tee -a .env >/dev/null; done
sudo docker compose up -d lawan_node 2>&1 | tail -2
sudo docker ps --format "{{.Names}} {{.Image}} {{.Status}}" | grep lawan
