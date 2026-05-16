#include "kinecore/config.hpp"

#include <toml++/toml.h>

#include <ostream>

namespace {

template <typename T>
void assign(const toml::table& t, std::string_view key, T& out) {
    if (auto v = t[key].value<T>()) {
        out = *v;
    }
}

void assign_float(const toml::table& t, std::string_view key, float& out) {
    if (auto v = t[key].value<double>()) {
        out = static_cast<float>(*v);
    }
}

}  // namespace

// -----------------------------------------------------------------------------
// kinecore::config
// -----------------------------------------------------------------------------
namespace kinecore::config {

void load(const toml::table& t, Camera& out) {
    assign(t, "width", out.width);
    assign(t, "height", out.height);
    assign(t, "fps", out.fps);
}

void load(const toml::table& t, Autofocus& out) {
    assign(t, "enabled", out.enabled);
    assign(t, "mode", out.mode);
    assign(t, "speed", out.speed);
    assign(t, "range", out.range);
}

std::ostream& operator<<(std::ostream& os, const Camera& c) {
    return os << c.width << "x" << c.height << " @ " << c.fps << " fps";
}

std::ostream& operator<<(std::ostream& os, const Autofocus& a) {
    return os << (a.enabled ? "on" : "off")
              << " (mode=" << a.mode
              << ", speed=" << a.speed
              << ", range=" << a.range << ")";
}

}  // namespace kinecore::config

// -----------------------------------------------------------------------------
// kineencode::config
// -----------------------------------------------------------------------------
namespace kineencode::config {

void load(const toml::table& t, Mjpeg& out) {
    assign(t, "enabled", out.enabled);
    assign(t, "output_enabled", out.output_enabled);
    assign(t, "encode_interval_ms", out.encode_interval_ms);
    assign(t, "burst_photo_count", out.burst_photo_count);
    assign(t, "max_burst_packets_during_recording", out.max_burst_packets_during_recording);
}

void load(const toml::table& t, H264& out) {
    assign(t, "enabled", out.enabled);
    assign(t, "output_enabled", out.output_enabled);
    assign(t, "gop_size", out.gop_size);
    assign(t, "bitrate", out.bitrate);
    assign(t, "cbr", out.cbr);
    assign(t, "sps_pps_repeat", out.sps_pps_repeat);
}

void load(const toml::table& t, Recording& out) {
    assign(t, "enabled", out.enabled);
    assign(t, "duration_sec", out.duration_sec);
    assign(t, "preroll_sec", out.preroll_sec);
    assign(t, "tail_duration", out.tail_duration);
    assign(t, "max_memory_mb", out.max_memory_mb);
    assign(t, "output_dir", out.output_dir);
}

std::ostream& operator<<(std::ostream& os, const Mjpeg& m) {
    return os << (m.enabled ? "on" : "off")
              << " (burst=" << m.burst_photo_count
              << ", interval=" << m.encode_interval_ms << "ms)";
}

std::ostream& operator<<(std::ostream& os, const H264& h) {
    return os << (h.enabled ? "on" : "off")
              << " (bitrate=" << h.bitrate
              << ", gop=" << h.gop_size
              << (h.cbr ? ", CBR" : ", VBR") << ")";
}

std::ostream& operator<<(std::ostream& os, const Recording& r) {
    return os << (r.enabled ? "on" : "off")
              << " (duration=" << r.duration_sec
              << "s, preroll=" << r.preroll_sec
              << "s, tail=" << r.tail_duration << "s)";
}

}  // namespace kineencode::config

// -----------------------------------------------------------------------------
// kinemotion::config
// -----------------------------------------------------------------------------
namespace kinemotion::config {

void load(const toml::table& t, FrameDiff& out) {
    assign(t, "enabled", out.enabled);
    assign(t, "frame_skip", out.frame_skip);
    assign_float(t, "pixel_change_sensitivity", out.pixel_change_sensitivity);
    assign_float(t, "min_object_size", out.min_object_size);
}

std::ostream& operator<<(std::ostream& os, const FrameDiff& f) {
    return os << (f.enabled ? "on" : "off")
              << " (skip=" << f.frame_skip
              << ", sens=" << f.pixel_change_sensitivity
              << ", min_obj=" << f.min_object_size << ")";
}

}  // namespace kinemotion::config

// -----------------------------------------------------------------------------
// kinetransport::config
// -----------------------------------------------------------------------------
namespace kinetransport::config {

void load(const toml::table& t, Tcp& out) {
    assign(t, "format", out.format);
    if (auto b = t["broadcast"].as_table()) {
        assign(*b, "enabled", out.broadcast.enabled);
        assign(*b, "port", out.broadcast.port);
        assign(*b, "max_clients", out.broadcast.max_clients);
    }
    if (auto c = t["client"].as_table()) {
        assign(*c, "enabled", out.client.enabled);
        assign(*c, "remote_ip", out.client.remote_ip);
        assign(*c, "remote_port", out.client.remote_port);
        assign(*c, "reconnect_interval_sec", out.client.reconnect_interval_sec);
    }
}

std::ostream& operator<<(std::ostream& os, const Tcp& t) {
    os << "format=" << t.format << "; ";
    os << "broadcast=" << (t.broadcast.enabled ? "on" : "off")
       << " (port=" << t.broadcast.port << "); ";
    os << "client=" << (t.client.enabled ? "on" : "off");
    if (t.client.enabled) {
        os << " (" << t.client.remote_ip << ":" << t.client.remote_port << ")";
    }
    return os;
}

}  // namespace kinetransport::config
