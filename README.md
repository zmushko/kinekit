# kinekit

A modular toolkit for smart Raspberry Pi cameras. Common root: *kine-* (kinesis Greek for "motion").

## Components

```
kinekit/
├── kinecore/    shared C++ library
├── kinegram/    application: observation + Telegram
└── kinemetry/   application: scientific motion-data collection
```

### kinecore
The foundation C++ library. Camera capture via `libcamera`, motion detection, H.264 and MJPEG encoding, MP4 muxing via `libav`, transports (files, TCP, Telegram), and TOML-based configuration. Consumed by both applications.

### kinegram
Standalone surveillance app: motion-triggered capture with events, photos, and videos published to Telegram. Two-way control via a Telegram bot (toggle detection, tune sensitivity, set minimum object size, etc.). The showcase product.

### kinemetry
Kinematic data collection on a thin client: object counts, motion patterns, vectors, velocities, accelerations. Captured data is stored in formats suitable for downstream analysis by large language models or a human operator. Potential applications include biology, ethology, and population monitoring.

## Status

Early stage. The project is being carved out of experiments in `main.cpp` (~4500-line monolith) that currently runs on a live Raspberry Pi Zero. Next step: incremental extraction of the monolith into `kinecore` + the two applications.

## Platform

Raspberry Pi (Pi Zero confirmed; Pi 4 / Pi 5 and other modules with `libcamera` support expected to work). ARM Linux. Built with [Meson](https://mesonbuild.com/) + [Ninja](https://ninja-build.org/).

```sh
meson setup build
ninja -C build
./build/kinegram/kinegram   # or kinemetry/kinemetry
```

### Cross-platform builds via Docker

Reproducible builds for any Raspberry Pi target are driven by `docker/build.sh`,
which wraps `docker buildx`. The same `docker/Dockerfile` (Debian Trixie base)
produces binaries for any architecture the host can run — natively on Apple
Silicon for `linux/arm64`, via QEMU emulation on x86_64 hosts.

```sh
docker/build.sh                          # default: linux/arm64 (Pi Zero 2 W, Pi 4, Pi 5)
PLATFORM=linux/arm/v7 docker/build.sh    # 32-bit ARMv7
PLATFORM=linux/amd64  docker/build.sh    # x86_64 for local testing
```

Artefacts land in `./out/<platform>/` on the host.

### Makefile shortcuts

A top-level `Makefile` wraps the common loops: build, deploy to a Pi over SSH,
run a smoke test, and so on. Defaults target `pi@zero.local`; override via
environment.

```sh
make help          # list all targets
make ssh-key       # install local SSH public key on the Pi (one-time)
make build         # cross-build for linux/arm64
make deploy        # build then scp artefacts to pi@zero.local
make test          # deploy and verify kinegram runs on the Pi
make run           # just run kinegram on the Pi
make ssh           # interactive shell on the Pi
make clean         # wipe ./out
```

## Co-authorship

Developed jointly with [Claude](https://www.anthropic.com/claude) by Anthropic.

## License

TBD.
