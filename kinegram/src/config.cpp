#include "kinegram/config.hpp"

#include <toml++/toml.h>

#include <ostream>

namespace kinegram::config {

namespace {

template <typename T>
void assign(const toml::table& t, std::string_view key, T& out) {
    if (auto v = t[key].value<T>()) {
        out = *v;
    }
}

}  // namespace

void load(const toml::table& t, Telegram& out) {
    assign(t, "enabled", out.enabled);
    assign(t, "bot_token", out.bot_token);
    assign(t, "chat_id", out.chat_id);
    assign(t, "max_queue_size", out.max_queue_size);
}

void load(const toml::table& t, UploadQueue& out) {
    assign(t, "failed_videos_dir", out.failed_videos_dir);
    assign(t, "max_failed_files", out.max_failed_files);
    assign(t, "retry_interval_sec", out.retry_interval_sec);
    assign(t, "max_retries", out.max_retries);
    assign(t, "send_to_telegram", out.send_to_telegram);
}

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

        // The legacy [video_recording] TOML section feeds two destinations:
        // kineencode::Recording for the recording-proper fields, and
        // kinegram::UploadQueue for the upload-retry fields. Unknown keys
        // are silently ignored on both sides, so the same table is safe to
        // pass through both loaders.
        if (auto t = root["video_recording"].as_table()) {
            kineencode::config::load(*t, result.config.recording);
            load(*t, result.config.upload);
        }

        if (auto t = root["telegram"].as_table())          load(*t, result.config.telegram);

        result.ok = true;
    } catch (const toml::parse_error& e) {
        result.error = std::string("TOML parse error: ") + std::string(e.description());
    } catch (const std::exception& e) {
        result.error = std::string("Config load error: ") + e.what();
    }
    return result;
}

std::ostream& operator<<(std::ostream& os, const Telegram& t) {
    return os << (t.enabled ? "on" : "off")
              << " (token=" << (t.bot_token.empty() ? "unset" : "set")
              << ", chat=" << (t.chat_id.empty() ? "unset" : "set")
              << ", queue=" << t.max_queue_size << ")";
}

std::ostream& operator<<(std::ostream& os, const UploadQueue& q) {
    return os << "dir=" << q.failed_videos_dir
              << ", max_files=" << q.max_failed_files
              << ", retry=" << q.retry_interval_sec << "s"
              << ", max_retries=" << q.max_retries
              << ", send=" << (q.send_to_telegram ? "yes" : "no");
}

std::ostream& operator<<(std::ostream& os, const Config& c) {
    os << "========== kinegram Config ==========\n"
       << "Camera:    " << c.camera << "\n"
       << "Autofocus: " << c.autofocus << "\n"
       << "MJPEG:     " << c.mjpeg << "\n"
       << "H.264:     " << c.h264 << "\n"
       << "Motion:    " << c.motion << "\n"
       << "Recording: " << c.recording << "\n"
       << "TCP:       " << c.tcp << "\n"
       << "Telegram:  " << c.telegram << "\n"
       << "Upload:    " << c.upload << "\n"
       << "=====================================\n";
    return os;
}

}  // namespace kinegram::config
