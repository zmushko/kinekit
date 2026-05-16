#pragma once

// App-level configuration for kinemetry.
//
// Aggregates kinecore / kineencode / kinemotion / kinetransport
// sections that the scientific pipeline needs. Telegram is intentionally
// absent — kinemetry exports data to its own sinks (file, MQTT, HTTP,
// TBD), not to a chat platform.

#include <iosfwd>
#include <string>
#include <string_view>

#include "kinecore/config.hpp"

namespace toml {
inline namespace v3 {
class table;
}
}

namespace kinemetry::config {

struct Config {
    kinecore::config::Camera        camera;
    kinecore::config::Autofocus     autofocus;
    kineencode::config::Mjpeg       mjpeg;
    kineencode::config::H264        h264;
    kineencode::config::Recording   recording;
    kinemotion::config::FrameDiff   motion;
    kinetransport::config::Tcp      tcp;
    // TODO: kinemetry-specific fields (data sink, output format, analyser
    // tuning) will land here when task #7 starts.
};

struct LoadResult {
    Config config;
    bool ok = false;
    std::string error;
};

LoadResult load_from_file(std::string_view path);

std::ostream& operator<<(std::ostream& os, const Config& c);

}  // namespace kinemetry::config
