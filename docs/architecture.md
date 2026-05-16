# kinekit — Architecture (DRAFT)

> Status: **draft / living document**. Updated as decisions are made. The
> structure here drives the migration of `rpi_capturer/src/main_v2.cpp`
> (~4500 lines, single file) into the kinekit monorepo.

## Goals

1. **Two products from shared modules.** `kinegram` (Telegram surveillance)
   and `kinemetry` (scientific motion analytics) must reuse the heavy
   parts: camera capture, encoding, motion detection, transport.
2. **Modular, but not micro.** Prefer a handful of meaningful libraries
   over a swarm of tiny ones. Split where the contract is clear; keep
   things together when the seam is fuzzy.
3. **No app-specifics in shared libs.** No Telegram in `kinecore`. No
   science analytics in `kinegram`. Cross-product coupling is the bug we
   are fixing by leaving the monolith.
4. **Buildable on Raspberry Pi via Docker** (already wired). New
   libraries should not require extra runtime install on the Pi beyond
   what `apt install` already provides (libcamera-dev, libav*, libcurl,
   turbojpeg, libtomlplusplus-dev — its headers only).

## Source inventory (`main_v2.cpp`)

| # | Class / unit | Lines | Approx LOC | Concern |
|---|---|---|---|---|
| 1 | `V4L2Controls` namespace | 52–60 | 9 | constants for V4L2 H.264 encoder |
| 2 | `Config` | 66–280 | 215 | TOML config, all sections |
| 3 | `HttpClient` | 283–503 | 220 | libcurl wrapper, multipart POST |
| 4 | `TelegramBotApi` | 505–704 | 200 | Telegram REST endpoints |
| 5 | `TelegramCommandHandler` | 706–831 | 126 | poll Telegram for `/commands` |
| 6 | `MotionDetector` | 833–1062 | 230 | frame-diff motion (NEON SIMD) |
| 7 | `TcpClient` | 1064–1126 | 63 | outbound TCP client |
| 8 | `JpegEncoder` | 1127–1200 | 74 | turbojpeg wrapper |
| 9 | `CircularVideoBuffer` | 1205–1270 | 66 | ring buffer of H.264 frames (preroll) |
| 10 | `mkdir_p` helper | 1274–1301 | 28 | `mkdir -p` |
| 11 | `VideoFileWriter` | 1305–1652 | 348 | MP4 muxing via libav |
| 12 | `VideoUploader` | 1657–1964 | 308 | async upload queue with retries → Telegram |
| 13 | `VideoRecordingManager` | 1969–2172 | 204 | state machine: idle → recording → tail |
| 14 | `H264Encoder` | 2174–2690 | 517 | V4L2 hardware H.264 encoder |
| 15 | `TcpBroadcaster` | 2692–2933 | 242 | TCP server, multi-client broadcast |
| 16 | `ISender` / `IH264Sender` / `IMjpegSender` | 2940–2958 | 19 | sender interfaces |
| 17 | `CompositeH264Sender` / `CompositeMjpegSender` | 2965–3070 | 106 | composite pattern |
| 18 | `TcpBroadcastSender` | 3077–3110 | 34 | sender → TcpBroadcaster |
| 19 | `TelegramSender` | 3112–3240 | 129 | sender → Telegram (burst photos) |
| 20 | `TcpSender` | 3242–3378 | 137 | sender → TcpClient |
| 21 | `H264FileSender` / `MjpegFileSender` | 3380–3421 | 42 | sender → disk file |
| 22 | `MjpegFrameHandler` / `H264FrameHandler` | 3428–3488 | 61 | dispatch frames to senders |
| 23 | `CapturerV2` | 3489–4400ish | ~900 | orchestrator: libcamera + main loop |
| 24 | `main()` | 4400ish–4519 | ~100 | CLI entry, glue |

Total: ~4500 lines, ~24 logical units.

## Proposed module layout

```
kinekit/
├── kinecore/        capture + frame primitives + common config
├── kineencode/      JPEG, H.264, MP4 mux, preroll buffer
├── kinemotion/      motion detection algorithms
├── kinetransport/   TCP server/client + sender interfaces + file senders
├── kinegram/        APP: Telegram subsystem + surveillance loop
└── kinemetry/       APP: scientific analytics + data export
```

Four libraries + two apps. Each library is independent; apps compose
them. No library knows about the apps. No app knows about another app's
internals.

### Dependency graph

```
         ┌────────────┐
         │  kinecore  │  (camera capture, frame types, common config)
         └─────┬──────┘
               │
   ┌───────────┼─────────────┐
   │           │             │
┌──┴───────┐ ┌─┴──────────┐ ┌┴────────────┐
│kineencode│ │ kinemotion │ │kinetransport│
└──┬───────┘ └─┬──────────┘ └┬────────────┘
   │           │             │
   └───────────┴─────┬───────┘
                    ┌┴─────────┐
               ┌────┴─────┐┌───┴─────┐
               │ kinegram ││kinemetry│
               └──────────┘└─────────┘
```

`kineencode`, `kinemotion`, `kinetransport` each depend only on
`kinecore`. Apps depend on whichever subset they need.

---

## Module contracts

### `kinecore`
**Owns:** camera capture (libcamera glue), frame data types, autofocus
controls, V4L2 control constants, common config types, filesystem
helpers.

**Classes from `main_v2.cpp`:**
- `V4L2Controls` (1)
- `CapturerV2` → split into `kinecore::Camera` (libcamera lifecycle) and
  per-app main loops (the orchestration part is app-specific glue, not
  core)
- `mkdir_p` helper (10) → `kinecore::fs::ensure_dir`

**Public config types (`kinecore::config::*`):**
- `Camera` (width/height/fps)
- `Autofocus` (enabled/mode/speed/range)

**Public frame types:**
- `RawFrame` — pointer + width/height/stride/pts (zero-copy view into
  libcamera buffer)
- `EncodedPacket` — bytes + pts + keyframe flag

**External deps:** libcamera, std threads, libtomlplusplus-dev (headers).

---

### `kineencode`
**Owns:** turning raw frames into bytes (MJPEG, H.264) and bytes into
MP4 files. Includes the preroll ring buffer because it stores encoded
H.264 frames (intimately tied to the encoder's output).

**Classes from `main_v2.cpp`:**
- `JpegEncoder` (8) → `kineencode::JpegEncoder`
- `H264Encoder` (14) → `kineencode::H264Encoder` (V4L2-based; only
  works on Raspberry Pi)
- `CircularVideoBuffer` (9) → `kineencode::PrerollBuffer`
- `VideoFileWriter` (11) → `kineencode::Mp4Writer`

**Public config types (`kineencode::config::*`):**
- `Mjpeg` (encode_interval, burst, etc.)
- `H264` (bitrate, gop, cbr, sps_pps_repeat)

**External deps:** turbojpeg, libav (avformat/avcodec/avutil), V4L2.

---

### `kinemotion`
**Owns:** motion detection algorithms. Starts with the existing
frame-diff approach. Will grow with `kinemetry` needs (optical flow,
contour tracking, kinematics).

**Classes from `main_v2.cpp`:**
- `MotionDetector` (6) → `kinemotion::FrameDiff` (the existing
  pixel-delta algorithm with NEON SIMD)

**Public config types (`kinemotion::config::*`):**
- `FrameDiff` (frame_skip, pixel_change_sensitivity, min_object_size)

**Planned (not in `main_v2.cpp` yet):**
- `kinemotion::OpticalFlow` (for kinemetry — vectors, velocities)
- `kinemotion::Tracker` (object tracking across frames)

**External deps:** ARM NEON intrinsics (optional, runtime-detected).

---

### `kinetransport`
**Owns:** moving bytes from inside the process to the outside world.
Generic — does not know what those bytes mean (JPEG? H.264? metrics?).

**Classes from `main_v2.cpp`:**
- `TcpBroadcaster` (15) → `kinetransport::TcpBroadcaster` (server)
- `TcpClient` (7) → `kinetransport::TcpClient` (outbound)
- `ISender`, `IH264Sender`, `IMjpegSender` (16) → `kinetransport::*`
  abstract sinks
- `CompositeH264Sender`, `CompositeMjpegSender` (17) → fan-out
- `TcpBroadcastSender`, `TcpSender` (18, 20) → TCP-backed senders
- `H264FileSender`, `MjpegFileSender` (21) → file-backed senders

**Public config types (`kinetransport::config::*`):**
- `Tcp` (format, broadcast{enabled,port,max_clients}, client{enabled,remote_ip,remote_port,reconnect_interval_sec})

**Excluded:** `TelegramSender` (19) — moves to `kinegram` because it
depends on `TelegramBotApi`.

**External deps:** BSD sockets (POSIX).

---

### `kinerecord` — *deferred*

The recording state machine (`VideoRecordingManager`) sits awkwardly:
half encoder glue, half app policy. **Initial decision: keep it inside
`kineencode` as `kineencode::Recorder`** rather than spinning a 5th
library. Revisit if `kinemetry` ends up needing a meaningfully
different recording policy.

**Critical seam to fix during migration:** the current
`VideoRecordingManager` constructor takes `VideoUploader*` directly,
leaking Telegram into the recording layer. The migrated version takes
an abstract callback / interface (e.g.
`std::function<void(RecordedFile)>`) so the recorder is unaware of
where the file goes afterwards.

**Public config types:**
- `kineencode::config::Recording` (duration_sec, preroll_sec,
  tail_duration, max_memory_mb, output_dir). The current
  `failed_videos_dir`, `max_failed_files`, `retry_interval_sec`,
  `max_retries`, `send_to_telegram` move to **kinegram** — they are
  upload-queue concerns, not recording concerns.

---

### `kinegram` (app)
**Owns:** Telegram bot, surveillance pipeline.

**Classes from `main_v2.cpp`:**
- `HttpClient` (3) — generic libcurl wrapper; kept inside kinegram for
  now (no other consumer). If a second app needs HTTP later, promote
  to `kinecore` or a new `kinenet` lib.
- `TelegramBotApi` (4)
- `TelegramCommandHandler` (5)
- `TelegramSender` (19)
- `VideoUploader` (12) → kinegram-specific: it's Telegram-flavoured
  upload queue with retries and `failed_videos_dir`
- Surveillance main loop (extracted from `CapturerV2`'s post-config
  logic + `main()`'s wiring)

**Owns its config layer:** wraps the kinecore/kineencode/etc.
configs and adds:
- `Telegram` (bot_token, chat_id, max_queue_size)
- Upload-queue fields (failed_videos_dir, max_failed_files,
  retry_interval_sec, max_retries, send_videos_to_telegram)

**External deps:** libcurl, all four libraries.

---

### `kinemetry` (app)
**Owns:** scientific motion-data collection and export.

**Classes from `main_v2.cpp`:** none yet — this is greenfield. It
borrows kinemotion for analysis primitives.

**Planned classes:**
- `kinemetry::DataExporter` — serialise per-frame analytics to CSV /
  JSON-Lines / MQTT / HTTP (TBD)
- `kinemetry::Analyser` — orchestrates frame-by-frame analysis pipeline
- Main loop (analogous to `CapturerV2` but science-oriented)

**Owns its config layer:** wraps kinecore/kineencode/kinemotion configs
and adds science-specific fields (output format, sink URL,
analysis-tuning parameters).

**External deps:** TBD (depends on what data sink we settle on).

---

## Migration order

A pragmatic order that keeps `kinegram` runnable on the Pi at every
step. Each step is one PR-sized chunk.

1. ✅ **Config skeleton** in kinecore — *done in current branch*.
   Split this Config into per-module configs as part of the
   subsequent steps (rather than as a separate pass).
2. **`kinecore::Camera`** — extract libcamera lifecycle from
   `CapturerV2`. Frame types (`RawFrame`). Keep a thin orchestrator in
   kinegram that wires it up.
3. **`kineencode::JpegEncoder`** + **`H264Encoder`** — relatively
   self-contained units. Move and wire.
4. **`kineencode::Mp4Writer`** + **`PrerollBuffer`** + **`Recorder`** —
   together because they're tightly coupled. **Critical: replace
   `VideoUploader*` dep with an abstract callback.**
5. **`kinemotion::FrameDiff`** — small, isolated. Easy move.
6. **`kinetransport`** — `TcpBroadcaster`, `TcpClient`, sender
   interfaces, file senders. Bring composite senders too.
7. **`kinegram`** internals — `HttpClient`, `TelegramBotApi`,
   `TelegramCommandHandler`, `TelegramSender`, `VideoUploader`,
   surveillance main loop. At this point `main_v2.cpp` is empty and
   the `rpi_capturer/` directory in the libcamera repo can be
   archived.
8. **`kinemetry`** main loop and a minimal data sink — first
   greenfield work.

Steps 2–7 are mechanical refactors with full Pi-side smoke tests at
each step. Step 8 is design work.

---

## Decisions

- ✅ **Filesystem helpers — `kinecore::fs`.** `mkdir_p` → `kinecore::fs::ensure_dir`.
  Lives in kinecore because multiple downstream modules need it (recorder
  for output dir, kinegram for failed-upload dir).
- ✅ **Logging — adopt `spdlog` during migration.** As each class moves
  into its target module, replace its `std::cout` / `std::cerr` lines
  with spdlog calls (per-module logger by module name). `libspdlog-dev`
  added to the Docker image; the library is used in header-only mode so
  it inlines into each module's `.so` and the Pi needs nothing extra
  installed.

## Open questions

- **HttpClient placement.** Currently planned inside kinegram. If
  kinemetry's data sink turns out to be HTTP-based, we should promote
  HttpClient to a shared lib (maybe a new `kinenet`). Revisit at task #7.
- **Software H.264 fallback (libx264) — deferred / NOT included.**
  The V4L2 H.264 encoder works only on Raspberry Pi. Adding a libx264
  fallback would let `kinegram`/`kinemetry` run on macOS without a Pi,
  but libcamera-based capture also doesn't exist on macOS, so the
  fallback would only help if we also added a file-based or webcam-based
  capture source. Not worth the parallel code path right now —
  `make deploy` to the Pi closes the loop in ~5 seconds. Revisit only if
  contributors without a Pi materialise.

---

## Living document

When module decisions change, update this file and reference the commit
in the relevant PR description. Keep the source inventory up to date as
classes are moved out of `main_v2.cpp`.
