#pragma once

// Thin, libcamera-free public API for capturing frames from a Raspberry
// Pi camera. The libcamera dependency is hidden behind a pimpl in
// camera.cpp — consumers (kinegram, kinemetry, future apps) include
// only this header and never see the libcamera namespace.

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "kinecore/config.hpp"
#include "kinecore/frame.hpp"

namespace kinecore {

struct CameraInfo {
    std::string id;     // libcamera-assigned opaque identifier
    std::string model;  // human-readable sensor model (e.g. "imx708")
};

class Camera {
public:
    // Free-function-shaped enumeration. Spins up a short-lived
    // CameraManager internally; safe to call without an open Camera.
    static std::vector<CameraInfo> enumerate();

    Camera();
    ~Camera();

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    Camera(Camera&&) noexcept;
    Camera& operator=(Camera&&) noexcept;

    // Acquire the first available camera, or one by libcamera id.
    // Returns false on failure (no cameras, already acquired by
    // another process, etc.).
    bool open();
    bool open(std::string_view id);

    // Apply resolution / fps. Pixel format is currently locked to
    // YUV420 — that is what the rest of kinekit knows how to consume.
    // Must be called after open() and before start(). Returns false on
    // libcamera configuration / validation errors.
    bool configure(const config::Camera& cam);

    // Re-applied to every queued request — changes take effect on the
    // next captured frame, not retroactively. Safe to call mid-capture.
    void set_autofocus(const config::Autofocus& af);

    using FrameCallback = std::function<void(const RawFrame&)>;

    // Begin capture. The callback is invoked on libcamera's internal
    // thread — DO NOT BLOCK or do heavy work in it. Slow consumers
    // (encoders, network, disk) should hand the data off to their own
    // worker threads.
    bool start(FrameCallback cb);

    // Cancel any in-flight requests and stop the capture thread. Safe
    // to call multiple times.
    void stop();

    // Available after a successful open().
    const CameraInfo& info() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace kinecore
