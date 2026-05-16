#include "kinemetry/config.hpp"

#include <toml++/toml.h>

#include <ostream>

namespace kinemetry::config {

LoadResult load_from_file(std::string_view path) {
    LoadResult result;
    try {
        auto root = toml::parse_file(path);

        if (auto t = root["camera"].as_table())            kinecore::config::load(*t, result.config.camera);
        if (auto t = root["autofocus"].as_table())         kinecore::config::load(*t, result.config.autofocus);
        if (auto t = root["mjpeg"].as_table())             kineencode::config::load(*t, result.config.mjpeg);
        if (auto t = root["h264"].as_table())              kineencode::config::load(*t, result.config.h264);
        if (auto t = root["motion_detection"].as_table())  kinemotion::config::load(*t, result.config.motion);
        if (auto t = root["tcp"].as_table())               kinetransport::config::load(*t, result.config.tcp);
        if (auto t = root["video_recording"].as_table())   kineencode::config::load(*t, result.config.recording);

        // The [telegram] section, if present in the TOML, is silently
        // ignored — kinemetry doesn't speak Telegram.

        result.ok = true;
    } catch (const toml::parse_error& e) {
        result.error = std::string("TOML parse error: ") + std::string(e.description());
    } catch (const std::exception& e) {
        result.error = std::string("Config load error: ") + e.what();
    }
    return result;
}

std::ostream& operator<<(std::ostream& os, const Config& c) {
    os << "========== kinemetry Config ==========\n"
       << "Camera:    " << c.camera << "\n"
       << "Autofocus: " << c.autofocus << "\n"
       << "MJPEG:     " << c.mjpeg << "\n"
       << "H.264:     " << c.h264 << "\n"
       << "Motion:    " << c.motion << "\n"
       << "Recording: " << c.recording << "\n"
       << "TCP:       " << c.tcp << "\n"
       << "======================================\n";
    return os;
}

}  // namespace kinemetry::config
