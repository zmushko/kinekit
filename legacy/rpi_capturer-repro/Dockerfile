# syntax=docker/dockerfile:1
#
# Bit-for-bit rebuild of the rpi_capturer binary deployed on the Pi Zero 2 W
# (tag rpi_capturer/zero-deployed, built natively on a Pi 5 on 2025-11-01 19:12 +03).
#
# The original build environment was Raspberry Pi OS (trixie, arm64) with the
# package versions below, reconstructed from the Pi 5 dpkg logs. Debian packages
# come from snapshot.debian.org as of the build date; Raspberry Pi specific
# (+rpt) packages are fetched by exact version from archive.raspberrypi.com.
#
# Expected output sha256: see expected.sha256

ARG SNAPSHOT=20251101T160000Z

FROM --platform=$BUILDPLATFORM alpine:3.20 AS fetch
RUN apk add --no-cache curl
COPY rpi-debs.txt rpi-debs.sha256 /tmp/
RUN mkdir /debs && cd /debs \
 && xargs -n1 curl -fsSLO < /tmp/rpi-debs.txt \
 && sha256sum -c /tmp/rpi-debs.sha256

FROM debian:trixie-20251020 AS build
ARG SNAPSHOT
RUN rm -f /etc/apt/sources.list.d/debian.sources \
 && printf '%s\n' \
      "deb [check-valid-until=no] http://snapshot.debian.org/archive/debian/${SNAPSHOT} trixie main" \
      "deb [check-valid-until=no] http://snapshot.debian.org/archive/debian/${SNAPSHOT} trixie-updates main" \
      "deb [check-valid-until=no] http://snapshot.debian.org/archive/debian-security/${SNAPSHOT} trixie-security main" \
      > /etc/apt/sources.list \
 && apt-get -o Acquire::Check-Valid-Until=false update
COPY --from=fetch /debs /debs
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      /debs/*.deb \
      gcc-14=14.2.0-19 g++-14=14.2.0-19 cpp-14=14.2.0-19 libstdc++-14-dev=14.2.0-19 \
      g++=4:14.2.0-1 binutils=2.44-3 make cmake=3.31.6-2 pkgconf=1.8.1-4 \
      libcurl4-openssl-dev=8.14.1-2 libturbojpeg0-dev=1:2.1.5-4 libcpptoml-dev=0.1.1-3 \
      libpisp1=1.2.1-2 libpisp-common=1.2.1-2 \
 && dpkg-query -W -f='${Package} ${Version}\n' > /build-env-packages.txt

# Same paths as on the Pi 5: main_v2.cpp includes the generated libcamera.h
# from the local libcamera build tree via "../../build/include/libcamera/libcamera.h".
WORKDIR /home/pi/Main/libcamera
COPY libcamera.h build/include/libcamera/libcamera.h
COPY src/rpi_capturer rpi_capturer
RUN mkdir rpi_capturer/build2 && cd rpi_capturer/build2 && cmake .. && make VERBOSE=1

FROM scratch AS out
COPY --from=build /home/pi/Main/libcamera/rpi_capturer/build2/rpi_capturer /rpi_capturer
COPY --from=build /build-env-packages.txt /build-env-packages.txt
