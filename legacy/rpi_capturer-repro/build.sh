#!/usr/bin/env bash
# Rebuild the Pi Zero rpi_capturer binary from tag rpi_capturer/zero-deployed
# and compare it with the deployed one.
set -euo pipefail
cd "$(dirname "$0")"
REF="${REF:-rpi_capturer/zero-deployed}"
rm -rf src out && mkdir -p src
git -C "$(git rev-parse --show-toplevel)" archive "$REF" rpi_capturer | tar -x -C src
docker buildx build --platform linux/arm64 --target out --output type=local,dest=out .
echo
sha256sum out/rpi_capturer
if (cd out && sha256sum -c ../expected.sha256); then
  echo "OK: bit-identical to the binary deployed on the Pi Zero"
else
  echo "MISMATCH: differs from the deployed binary" >&2; exit 1
fi
