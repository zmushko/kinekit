#pragma once

// App-level configuration for kinegram.
//
// Aggregates every relevant kinecore / kineencode / kinemotion /
// kinetransport section, plus the kinegram-specific Telegram and
// upload-queue settings.

#include <iosfwd>
#include <string>
#include <string_view>

#include "kinecore/config.hpp"

namespace toml {
inline namespace v3 {
class table;
}
}

namespace kinegram::config {

struct Telegram {
    bool enabled = true;
    std::string bot_token;
    std::string chat_id;
    int max_queue_size = 5;
};

// Upload-queue concerns split out of the old [video_recording] section:
// they belong here, not in kineencode, because they only make sense in a
// Telegram-oriented surveillance pipeline.
struct UploadQueue {
    std::string failed_videos_dir = "/home/pi/recordings/failed";
    int max_failed_files = 50;
    int retry_interval_sec = 2;
    int max_retries = 2;
    bool send_to_telegram = true;
};

struct Config {
    kinecore::config::Camera        camera;
    kinecore::config::Autofocus     autofocus;
    kineencode::config::Mjpeg       mjpeg;
    kineencode::config::H264        h264;
    kineencode::config::Recording   recording;
    kinemotion::config::FrameDiff   motion;
    kinetransport::config::Tcp      tcp;
    Telegram                        telegram;
    UploadQueue                     upload;
};

struct LoadResult {
    Config config;
    bool ok = false;
    std::string error;
};

void load(const toml::table& t, Telegram& out);
void load(const toml::table& t, UploadQueue& out);

LoadResult load_from_file(std::string_view path);

std::ostream& operator<<(std::ostream& os, const Telegram& t);
std::ostream& operator<<(std::ostream& os, const UploadQueue& q);
std::ostream& operator<<(std::ostream& os, const Config& c);

}  // namespace kinegram::config
