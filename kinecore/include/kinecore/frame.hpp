#pragma once

// Raw camera frame types. Designed for zero-copy: a RawFrame is a view
// into a libcamera-owned mmap region whose lifetime is bounded by the
// frame callback. Consumers MUST NOT store the pointers — copy out
// whatever bytes they need before returning.

#include <cstddef>
#include <cstdint>

namespace kinecore {

struct Plane {
    const uint8_t* data = nullptr;
    size_t size = 0;
    size_t stride = 0;
};

// YUV420 planar frame. Format is locked at this stage of kinekit —
// every downstream consumer (encoders, motion detection) consumes Y
// for analysis and Y/U/V for encoding. When we eventually support
// other pixel formats, this struct will gain a `format` enum and the
// invariants will be relaxed; for now keeping it concrete keeps the
// API blunt and easy to read.
struct RawFrame {
    int width = 0;
    int height = 0;

    Plane y;   // luma plane,   width * height bytes
    Plane u;   // chroma U,     (width/2) * (height/2) bytes
    Plane v;   // chroma V,     (width/2) * (height/2) bytes

    int64_t pts_us = 0;     // libcamera SensorTimestamp, converted ns -> µs
    uint32_t sequence = 0;  // libcamera Request::sequence()
};

}  // namespace kinecore
