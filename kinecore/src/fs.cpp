#include "kinecore/fs.hpp"

#include <cerrno>
#include <cstring>
#include <string>
#include <sys/stat.h>

#include <spdlog/spdlog.h>

namespace kinecore::fs {

bool ensure_dir(std::string_view path) {
    if (path.empty()) return false;

    struct stat st;
    if (::stat(std::string(path).c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }

    if (auto pos = path.find_last_of('/'); pos != std::string_view::npos && pos > 0) {
        if (!ensure_dir(path.substr(0, pos))) {
            return false;
        }
    }

    if (::mkdir(std::string(path).c_str(), 0755) != 0 && errno != EEXIST) {
        spdlog::error("kinecore::fs: cannot create '{}': {}", path, std::strerror(errno));
        return false;
    }
    return true;
}

}  // namespace kinecore::fs
