# syntax=docker/dockerfile:1.7
#
# Multi-arch build for kinekit. The same Dockerfile produces ARM64 binaries
# (Raspberry Pi Zero 2 W, Pi 4, Pi 5) and others — controlled by buildx
# `--platform` flag. On Apple Silicon hosts, linux/arm64 runs natively;
# on x86_64 hosts, QEMU emulation is used transparently.

FROM debian:trixie-slim AS build

ARG DEBIAN_FRONTEND=noninteractive

# Add the Raspberry Pi APT archive so we pull the same libcamera 0.5.x
# that Raspberry Pi OS ships, not Debian's stock 0.4.x. Without this the
# build produces binaries linked against libcamera.so.0.4 which the Pi
# has no chance of loading at runtime (it has libcamera.so.0.5).
#
# `trusted=yes` skips signature verification — the Raspberry Pi archive
# key is still SHA1-signed, which Debian Trixie's apt rejects since
# 2026-02-01. This is acceptable here because the container is
# ephemeral and only used to link against headers + .so stubs; the
# actual runtime libcamera on the Pi is whatever the user has vetted
# and installed via the Pi's own apt. Revisit when Raspberry Pi
# re-signs their archive key (track upstream).
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
    && echo "deb [trusted=yes] http://archive.raspberrypi.com/debian/ trixie main" \
        > /etc/apt/sources.list.d/raspi.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        g++ \
        meson \
        ninja-build \
        pkg-config \
        libtomlplusplus-dev \
        libspdlog-dev \
        libfmt-dev \
        libcamera-dev \
        libavformat-dev \
        libavcodec-dev \
        libavutil-dev \
        libswscale-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY meson.build ./
COPY kinecore ./kinecore
COPY kinegram ./kinegram
COPY kinemetry ./kinemetry

RUN meson setup --buildtype=release --prefix=/usr /build /src \
    && ninja -C /build

# Export stage: just the artefacts, nothing else. Used with
# `docker buildx build --target export --output type=local,dest=./out`.
FROM scratch AS export
COPY --from=build /build/kinegram/kinegram        /kinegram
COPY --from=build /build/kinemetry/kinemetry      /kinemetry
COPY --from=build /build/kinecore/libkinecore.so  /libkinecore.so
