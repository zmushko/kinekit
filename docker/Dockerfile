# syntax=docker/dockerfile:1.7
#
# Multi-arch build for kinekit. The same Dockerfile produces ARM64 binaries
# (Raspberry Pi Zero 2 W, Pi 4, Pi 5) and others — controlled by buildx
# `--platform` flag. On Apple Silicon hosts, linux/arm64 runs natively;
# on x86_64 hosts, QEMU emulation is used transparently.

FROM debian:trixie-slim AS build

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        g++ \
        meson \
        ninja-build \
        pkg-config \
        libtomlplusplus-dev \
        libspdlog-dev \
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
