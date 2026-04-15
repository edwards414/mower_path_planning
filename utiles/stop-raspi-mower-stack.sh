#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_DIR}"

export USER_NAME="${USER_NAME:-$(id -un)}"
export USER_ID="${USER_ID:-$(id -u)}"
export GROUP_NAME="${GROUP_NAME:-$(id -gn)}"
export GROUP_ID="${GROUP_ID:-$(id -g)}"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

COMPOSE_ARGS=(
  -f .devcontainer/docker-compose.raspi.dev.yaml
  -f .devcontainer/docker-compose.raspi.zenoh.yaml
)

docker_compose() {
  docker compose "${COMPOSE_ARGS[@]}" "$@"
}

docker_compose down
