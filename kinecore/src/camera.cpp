#include "kinecore/camera.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <sys/mman.h>
#include <vector>

#include <libcamera/libcamera.h>

#include <spdlog/spdlog.h>

namespace kinecore {

// =============================================================================
// Camera::Impl — owns libcamera state and translates each completed Request
// into a kinecore::RawFrame for the user's callback.
// =============================================================================
class Camera::Impl {
public:
    Impl() : cm_(std::make_unique<libcamera::CameraManager>()) {}

    ~Impl() {
        stop();
        if (camera_) {
            camera_->release();
            camera_.reset();
        }
        if (cm_) {
            cm_->stop();
        }
    }

    bool open_first() {
        if (!ensure_manager()) return false;
        if (cm_->cameras().empty()) {
            spdlog::error("kinecore::Camera: no cameras available");
            return false;
        }
        return acquire(cm_->cameras()[0]);
    }

    bool open_by_id(std::string_view id) {
        if (!ensure_manager()) return false;
        for (const auto& c : cm_->cameras()) {
            if (c->id() == id) {
                return acquire(c);
            }
        }
        spdlog::error("kinecore::Camera: no camera matched id '{}'", id);
        return false;
    }

    bool configure(const config::Camera& cam) {
        if (!camera_) {
            spdlog::error("kinecore::Camera: configure() called before open()");
            return false;
        }

        config_ = camera_->generateConfiguration({libcamera::StreamRole::VideoRecording});
        if (!config_) {
            spdlog::error("kinecore::Camera: generateConfiguration() returned null");
            return false;
        }

        libcamera::StreamConfiguration& sc = config_->at(0);
        sc.size = libcamera::Size(cam.width, cam.height);
        sc.pixelFormat = libcamera::formats::YUV420;

        switch (config_->validate()) {
            case libcamera::CameraConfiguration::Invalid:
                spdlog::error("kinecore::Camera: configuration invalid");
                return false;
            case libcamera::CameraConfiguration::Adjusted:
                spdlog::warn("kinecore::Camera: configuration adjusted to {}x{} ({})",
                             sc.size.width, sc.size.height, sc.pixelFormat.toString());
                break;
            case libcamera::CameraConfiguration::Valid:
                break;
        }

        if (camera_->configure(config_.get()) < 0) {
            spdlog::error("kinecore::Camera: configure() failed");
            return false;
        }

        fps_ = cam.fps;
        configured_width_ = sc.size.width;
        configured_height_ = sc.size.height;

        // Pre-allocate buffers and per-buffer Requests up front.
        allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
        libcamera::Stream* stream = config_->at(0).stream();
        if (allocator_->allocate(stream) < 0) {
            spdlog::error("kinecore::Camera: buffer allocation failed");
            return false;
        }

        requests_.clear();
        for (const auto& buf : allocator_->buffers(stream)) {
            auto req = camera_->createRequest();
            if (!req) {
                spdlog::error("kinecore::Camera: createRequest() failed");
                return false;
            }
            if (req->addBuffer(stream, buf.get()) < 0) {
                spdlog::error("kinecore::Camera: addBuffer() failed");
                return false;
            }
            // mmap each plane[0] once and remember the mapping — the
            // fd-to-pointer translation is invariant across requests.
            map_buffer(buf.get());
            requests_.push_back(std::move(req));
        }

        spdlog::info("kinecore::Camera: configured {}x{} @ {} fps, {} buffers",
                     configured_width_, configured_height_, fps_, requests_.size());
        return true;
    }

    void set_autofocus(const config::Autofocus& af) {
        std::lock_guard lock(af_mu_);
        af_ = af;
    }

    bool start(Camera::FrameCallback cb) {
        if (!camera_) {
            spdlog::error("kinecore::Camera: start() called before open()");
            return false;
        }
        if (requests_.empty()) {
            spdlog::error("kinecore::Camera: start() called before configure()");
            return false;
        }
        cb_ = std::move(cb);
        running_.store(true);

        camera_->requestCompleted.connect(this, &Impl::on_request_completed);

        if (camera_->start() < 0) {
            spdlog::error("kinecore::Camera: camera->start() failed");
            running_.store(false);
            return false;
        }

        for (auto& req : requests_) {
            req->controls() = build_controls();
            if (camera_->queueRequest(req.get()) < 0) {
                spdlog::error("kinecore::Camera: initial queueRequest() failed");
                running_.store(false);
                camera_->stop();
                return false;
            }
        }
        spdlog::info("kinecore::Camera: capture started ({} fps target)", fps_);
        return true;
    }

    void stop() {
        if (!camera_ || !running_.exchange(false)) return;
        camera_->stop();
        camera_->requestCompleted.disconnect(this, &Impl::on_request_completed);
        spdlog::info("kinecore::Camera: capture stopped");
    }

    const CameraInfo& info() const { return info_; }

    static std::vector<CameraInfo> enumerate_all() {
        std::vector<CameraInfo> out;
        libcamera::CameraManager cm;
        if (cm.start() < 0) {
            spdlog::error("kinecore::Camera::enumerate: CameraManager::start failed");
            return out;
        }
        for (const auto& c : cm.cameras()) {
            CameraInfo ci;
            ci.id = c->id();
            if (auto m = c->properties().get(libcamera::properties::Model)) {
                ci.model = *m;
            }
            out.push_back(std::move(ci));
        }
        cm.stop();
        return out;
    }

private:
    bool ensure_manager() {
        if (manager_started_) return true;
        if (cm_->start() < 0) {
            spdlog::error("kinecore::Camera: CameraManager::start failed");
            return false;
        }
        manager_started_ = true;
        return true;
    }

    bool acquire(std::shared_ptr<libcamera::Camera> c) {
        camera_ = std::move(c);
        if (camera_->acquire() < 0) {
            spdlog::error("kinecore::Camera: acquire() failed for id={}", camera_->id());
            camera_.reset();
            return false;
        }
        info_.id = camera_->id();
        if (auto m = camera_->properties().get(libcamera::properties::Model)) {
            info_.model = *m;
        }
        spdlog::info("kinecore::Camera: opened id='{}' model='{}'", info_.id, info_.model);
        return true;
    }

    // Pi DMA buffers expose YUV420 as a single contiguous mapping through
    // plane[0]. Map once and stash the pointer keyed by the buffer.
    void map_buffer(libcamera::FrameBuffer* buf) {
        const auto& plane0 = buf->planes()[0];
        size_t total = static_cast<size_t>(configured_width_) * configured_height_ * 3 / 2;
        void* m = ::mmap(nullptr, total, PROT_READ, MAP_SHARED, plane0.fd.get(), 0);
        if (m == MAP_FAILED) {
            spdlog::error("kinecore::Camera: mmap failed for buffer fd={} : {}",
                          plane0.fd.get(), std::strerror(errno));
            return;
        }
        buffer_maps_[buf] = {static_cast<uint8_t*>(m), total};
    }

    libcamera::ControlList build_controls() {
        libcamera::ControlList ctrls;

        config::Autofocus af_snapshot;
        {
            std::lock_guard lock(af_mu_);
            af_snapshot = af_;
        }

        if (af_snapshot.enabled) {
            switch (af_snapshot.mode) {
                case 0:
                    ctrls.set(libcamera::controls::AfMode, libcamera::controls::AfModeAuto);
                    break;
                case 1:
                    ctrls.set(libcamera::controls::AfMode, libcamera::controls::AfModeManual);
                    break;
                case 2:
                    ctrls.set(libcamera::controls::AfMode, libcamera::controls::AfModeContinuous);
                    break;
            }
            ctrls.set(libcamera::controls::AfSpeed,
                      af_snapshot.speed == 1 ? libcamera::controls::AfSpeedFast
                                             : libcamera::controls::AfSpeedNormal);
            switch (af_snapshot.range) {
                case 0:
                    ctrls.set(libcamera::controls::AfRange, libcamera::controls::AfRangeNormal);
                    break;
                case 1:
                    ctrls.set(libcamera::controls::AfRange, libcamera::controls::AfRangeMacro);
                    break;
                case 2:
                    ctrls.set(libcamera::controls::AfRange, libcamera::controls::AfRangeFull);
                    break;
            }
        }

        if (fps_ > 0) {
            int64_t frame_us = 1'000'000 / fps_;
            ctrls.set(libcamera::controls::FrameDurationLimits,
                      libcamera::Span<const int64_t, 2>({frame_us, frame_us}));
        }

        return ctrls;
    }

    void on_request_completed(libcamera::Request* req) {
        if (!running_.load(std::memory_order_relaxed)) return;
        if (req->status() != libcamera::Request::RequestComplete) {
            spdlog::warn("kinecore::Camera: request status {}", static_cast<int>(req->status()));
            requeue(req);
            return;
        }

        libcamera::FrameBuffer* buf = req->buffers().begin()->second;
        auto it = buffer_maps_.find(buf);
        if (it == buffer_maps_.end()) {
            spdlog::error("kinecore::Camera: unmapped buffer in callback");
            requeue(req);
            return;
        }
        const auto& [base, size] = it->second;

        const int w = configured_width_;
        const int h = configured_height_;
        const size_t y_size = static_cast<size_t>(w) * h;
        const size_t uv_size = y_size / 4;

        RawFrame frame;
        frame.width = w;
        frame.height = h;
        frame.y = {base,                 y_size,  static_cast<size_t>(w)};
        frame.u = {base + y_size,        uv_size, static_cast<size_t>(w / 2)};
        frame.v = {base + y_size + uv_size, uv_size, static_cast<size_t>(w / 2)};

        if (auto ts = req->metadata().get(libcamera::controls::SensorTimestamp)) {
            frame.pts_us = static_cast<int64_t>(*ts) / 1000;  // ns -> us
        }
        frame.sequence = req->sequence();

        if (cb_) cb_(frame);

        requeue(req);
    }

    void requeue(libcamera::Request* req) {
        if (!running_.load(std::memory_order_relaxed)) return;
        req->reuse(libcamera::Request::ReuseBuffers);
        req->controls() = build_controls();
        if (camera_->queueRequest(req) < 0) {
            spdlog::error("kinecore::Camera: queueRequest() failed during reuse");
        }
    }

    std::unique_ptr<libcamera::CameraManager> cm_;
    bool manager_started_ = false;

    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    struct Mapping { uint8_t* base; size_t size; };
    std::map<libcamera::FrameBuffer*, Mapping> buffer_maps_;

    int configured_width_ = 0;
    int configured_height_ = 0;
    int fps_ = 30;

    Camera::FrameCallback cb_;
    std::atomic<bool> running_{false};

    std::mutex af_mu_;
    config::Autofocus af_;

    CameraInfo info_;
};

// =============================================================================
// Camera — thin forwarders to Impl. Public type is libcamera-free.
// =============================================================================

Camera::Camera() : impl_(std::make_unique<Impl>()) {}
Camera::~Camera() = default;
Camera::Camera(Camera&&) noexcept = default;
Camera& Camera::operator=(Camera&&) noexcept = default;

std::vector<CameraInfo> Camera::enumerate() {
    return Impl::enumerate_all();
}

bool Camera::open()                                       { return impl_->open_first(); }
bool Camera::open(std::string_view id)                    { return impl_->open_by_id(id); }
bool Camera::configure(const config::Camera& cam)         { return impl_->configure(cam); }
void Camera::set_autofocus(const config::Autofocus& af)   { impl_->set_autofocus(af); }
bool Camera::start(FrameCallback cb)                      { return impl_->start(std::move(cb)); }
void Camera::stop()                                       { impl_->stop(); }
const CameraInfo& Camera::info() const                    { return impl_->info(); }

}  // namespace kinecore
