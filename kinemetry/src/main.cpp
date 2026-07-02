#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include "kinecore/camera.hpp"
#include "kinecore/version.hpp"
#include "kinemetry/config.hpp"

namespace {

constexpr std::chrono::seconds kProbeDuration{2};

int probe_camera(const kinemetry::config::Config& cfg) {
    std::cout << "\n=== kinemetry --probe-camera ===\n";
    auto cams = kinecore::Camera::enumerate();
    std::cout << "Detected " << cams.size() << " camera(s):\n";
    for (const auto& c : cams) {
        std::cout << "  - id='" << c.id << "'  model='" << c.model << "'\n";
    }
    if (cams.empty()) {
        std::cerr << "kinemetry: no cameras to probe\n";
        return 2;
    }

    kinecore::Camera cam;
    if (!cam.open()) return 3;
    if (!cam.configure(cfg.camera)) return 4;
    cam.set_autofocus(cfg.autofocus);

    std::atomic<int> frames{0};
    auto callback = [&frames](const kinecore::RawFrame& /*frame*/) {
        frames.fetch_add(1, std::memory_order_relaxed);
    };

    const auto t0 = std::chrono::steady_clock::now();
    if (!cam.start(callback)) return 5;
    std::this_thread::sleep_for(kProbeDuration);
    cam.stop();
    const auto t1 = std::chrono::steady_clock::now();

    const int n = frames.load();
    const double secs = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "Captured " << n << " frames in " << secs
              << " s -> avg " << (secs > 0 ? n / secs : 0.0) << " fps\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::cout << "kinemetry (kinecore " << kinecore::version() << ")\n";

    bool probe = false;
    const char* config_path = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--probe-camera") == 0) {
            probe = true;
        } else {
            config_path = argv[i];
        }
    }

    kinemetry::config::Config cfg;
    if (config_path) {
        auto result = kinemetry::config::load_from_file(config_path);
        if (!result.ok) {
            std::cerr << "kinemetry: " << result.error << "\n";
            std::cerr << "kinemetry: falling back to defaults\n";
        } else {
            std::cout << "kinemetry: loaded config from " << config_path << "\n";
        }
        cfg = std::move(result.config);
    } else {
        std::cout << "kinemetry: no config path given — using defaults\n";
    }

    if (probe) {
        return probe_camera(cfg);
    }

    std::cout << cfg;
    return 0;
}
