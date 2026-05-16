#!/usr/bin/env bash
#
# Cross-platform build for kinekit via Docker buildx.
#
# Usage:
#   docker/build.sh                          # default: linux/arm64 (Pi Zero 2 W, Pi 4, Pi 5)
#   PLATFORM=linux/arm/v7 docker/build.sh    # 32-bit ARMv7
#   PLATFORM=linux/amd64  docker/build.sh    # x86_64 for testing
#
# Artefacts land in ./out/<platform>/ on the host.

set -euo pipefail

PLATFORM="${PLATFORM:-linux/arm64}"
OUT_DIR="${OUT_DIR:-./out/${PLATFORM//\//-}}"

cd "$(dirname "$0")/.."

mkdir -p "${OUT_DIR}"

echo "==> Building kinekit for ${PLATFORM} -> ${OUT_DIR}"

docker buildx build \
    --platform "${PLATFORM}" \
    --target export \
    --file docker/Dockerfile \
    --output "type=local,dest=${OUT_DIR}" \
    .

echo
echo "==> Artefacts:"
ls -la "${OUT_DIR}"
echo
echo "==> File types:"
file "${OUT_DIR}"/* 2>/dev/null || true
