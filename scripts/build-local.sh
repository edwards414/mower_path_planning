#!/usr/bin/env bash
#
# Build the runtime image locally while reusing the GHCR build cache that CI
# (.github/workflows/build.yml) publishes for the `runtime` target. This makes
# expensive builder layers (rustup toolchain, rosdep install) cache-hit instead
# of re-running on every local build.
#
# Quick start:
#   ./scripts/build-local.sh                 # pull CI cache, build, load locally
#   CACHE_PUSH=1 ./scripts/build-local.sh     # also push updated cache to GHCR
#
# Auth:
#   - Pulling the (private) cache needs a GHCR login with `read:packages`.
#   - Pushing it back (CACHE_PUSH=1) additionally needs `write:packages`.
#   Either run `docker login ghcr.io` beforehand, or export CR_PAT and this
#   script logs in for you. If the cache package is public, no login is needed.
#
# Common overrides (env vars):
#   GHCR_OWNER   GitHub owner/org          (default: edwards414)
#   CACHE_REPO   cache package base name   (default: mower_path_planning)
#   IMAGE        local image tag to load   (default: mower:opt-test)
#   TARGET       Dockerfile stage          (default: runtime)
#   BUILDER      buildx builder name       (default: mower)
#   CACHE_PUSH   1 to also --cache-to      (default: 0, pull-only)
#   CR_PAT       PAT used to auto-login to ghcr.io (optional)
#
# Any extra args are forwarded to `docker buildx build`, e.g.:
#   ./scripts/build-local.sh --platform linux/arm64 --no-cache
set -euo pipefail

GHCR_OWNER="${GHCR_OWNER:-edwards414}"
CACHE_REPO="${CACHE_REPO:-mower_path_planning}"
IMAGE="${IMAGE:-mower:opt-test}"
TARGET="${TARGET:-runtime}"
BUILDER="${BUILDER:-mower}"
CACHE_PUSH="${CACHE_PUSH:-0}"

CACHE_REF="ghcr.io/${GHCR_OWNER}/${CACHE_REPO}-buildcache:${TARGET}"

# Build context = the repo root (parent of this scripts/ directory), where the
# Dockerfile and .dockerignore live.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$ROOT_DIR"

log() { printf '\033[1;34m[build-local]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[build-local]\033[0m %s\n' "$*" >&2; }

# --- GHCR login -------------------------------------------------------------
if [[ -n "${CR_PAT:-}" ]]; then
  log "Logging in to ghcr.io as ${GHCR_OWNER} (via CR_PAT)..."
  echo "${CR_PAT}" | docker login ghcr.io -u "${GHCR_OWNER}" --password-stdin >/dev/null
elif ! grep -q '"ghcr.io"' "${HOME}/.docker/config.json" 2>/dev/null; then
  warn "Not logged in to ghcr.io and CR_PAT is unset."
  warn "If the cache package is private, the cache pull will simply miss."
  warn "To use it: run 'docker login ghcr.io -u ${GHCR_OWNER}' (PAT with read:packages),"
  warn "or export CR_PAT=<your-PAT> and re-run."
fi

# --- buildx builder (docker-container driver, required for registry cache) ---
if ! docker buildx inspect "${BUILDER}" >/dev/null 2>&1; then
  log "Creating buildx builder '${BUILDER}' (docker-container driver)..."
  docker buildx create --name "${BUILDER}" --driver docker-container >/dev/null
fi

# --- assemble cache args -----------------------------------------------------
CACHE_ARGS=(--cache-from "type=registry,ref=${CACHE_REF}")
if [[ "${CACHE_PUSH}" == "1" ]]; then
  log "CACHE_PUSH=1: will also push updated cache to ${CACHE_REF} (needs write:packages)."
  CACHE_ARGS+=(--cache-to "type=registry,mode=max,image-manifest=true,oci-mediatypes=true,ref=${CACHE_REF}")
fi

log "Building target '${TARGET}' -> ${IMAGE}"
log "  cache-from: ${CACHE_REF}"
set -x
docker buildx build \
  --builder "${BUILDER}" \
  --target "${TARGET}" \
  "${CACHE_ARGS[@]}" \
  -t "${IMAGE}" \
  --load \
  "$@" \
  .
