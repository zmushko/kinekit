# kinekit

A modular toolkit for smart Raspberry Pi cameras. Common root: *kine-* (Greek for "motion").

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

Early stage. The project is being carved out of experiments in `rpi_capturer/src/main_v2.cpp` (~4500-line monolith) that currently runs on a live Raspberry Pi Zero. Next step: incremental extraction of the monolith into `kinecore` + the two applications.

## Platform

Raspberry Pi (Pi Zero confirmed; Pi 4 / Pi 5 and other modules with `libcamera` support expected to work). ARM Linux. Built with CMake.

## Co-authorship

Developed jointly with [Claude](https://www.anthropic.com/claude) by Anthropic. Architecture, naming, and refactoring decisions are made in dialogue between the human author and the model.

## License

TBD.
