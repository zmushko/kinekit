#pragma once

// Configuration types for kinecore and its sibling libraries
// (kineencode, kinemotion, kinetransport). During the migration of
// main_v2.cpp into kinekit, the sibling .so files don't exist yet, so
// all of these types currently compile into libkinecore.so. They live
// in their *target* namespaces so that the eventual move is a pure
// file-shuffle — no API change for consumers.
//
// App-level umbrella configs (kinegram::config::Config,
// kinemetry::config::Config) live in their respective apps and
// aggregate the types declared here plus any app-specific bits
// (Telegram, upload queue, data sink, etc.).

#include <iosfwd>
#include <string>

// Forward-declare toml::table so this header is cheap to include — the
// full <toml++/toml.h> only gets pulled into config.cpp.
namespace toml {
inline namespace v3 {
class table;
}
}

// -----------------------------------------------------------------------------
// kinecore — base camera primitives shared by every consumer
// -----------------------------------------------------------------------------
namespace kinecore::config {

struct Camera {
    int width = 1920;
    int height = 1080;
    int fps = 30;
};

struct Autofocus {
    bool enabled = true;
    int mode = 2;   // 0=Auto, 1=Manual, 2=Continuous
    int speed = 1;  // 0=Normal, 1=Fast
    int range = 2;  // 0=Normal, 1=Macro, 2=Full
};

void load(const toml::table& t, Camera& out);
void load(const toml::table& t, Autofocus& out);

std::ostream& operator<<(std::ostream& os, const Camera& c);
std::ostream& operator<<(std::ostream& os, const Autofocus& a);

}  // namespace kinecore::config

// -----------------------------------------------------------------------------
// kineencode — encoder + recorder configs
// -----------------------------------------------------------------------------
namespace kineencode::config {

struct Mjpeg {
    bool enabled = true;
    bool output_enabled = true;
    int encode_interval_ms = 1000;
    int burst_photo_count = 1;
    int max_burst_packets_during_recording = 3;
};

struct H264 {
    bool enabled = true;
    bool output_enabled = true;
    int gop_size = 60;
    int bitrate = 5'000'000;
    bool cbr = false;
    bool sps_pps_repeat = true;
};

// Pure recording concerns. Upload-queue fields (failed_videos_dir,
// max_failed_files, retry_interval_sec, max_retries, send_to_telegram)
// are kinegram-specific and live there.
struct Recording {
    bool enabled = true;
    int duration_sec = 30;
    int preroll_sec = 3;
    int tail_duration = 5;
    int max_memory_mb = 20;
    std::string output_dir = "/tmp/recordings";
};

void load(const toml::table& t, Mjpeg& out);
void load(const toml::table& t, H264& out);
void load(const toml::table& t, Recording& out);

std::ostream& operator<<(std::ostream& os, const Mjpeg& m);
std::ostream& operator<<(std::ostream& os, const H264& h);
std::ostream& operator<<(std::ostream& os, const Recording& r);

}  // namespace kineencode::config

// -----------------------------------------------------------------------------
// kinemotion — motion-detection algorithm configs
// -----------------------------------------------------------------------------
namespace kinemotion::config {

struct FrameDiff {
    bool enabled = true;
    int frame_skip = 7;
    float pixel_change_sensitivity = 25.0f;
    float min_object_size = 0.05f;
};

void load(const toml::table& t, FrameDiff& out);
std::ostream& operator<<(std::ostream& os, const FrameDiff& f);

}  // namespace kinemotion::config

// -----------------------------------------------------------------------------
// kinetransport — TCP and sender configs
// -----------------------------------------------------------------------------
namespace kinetransport::config {

struct Tcp {
    std::string format = "h264";  // "h264" | "mjpeg"

    struct Broadcast {
        bool enabled = true;
        int port = 8554;
        int max_clients = 5;
    } broadcast;

    struct Client {
        bool enabled = false;
        std::string remote_ip = "10.0.0.2";
        int remote_port = 9999;
        int reconnect_interval_sec = 1;
    } client;
};

void load(const toml::table& t, Tcp& out);
std::ostream& operator<<(std::ostream& os, const Tcp& t);

}  // namespace kinetransport::config
