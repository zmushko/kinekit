#pragma once

#include <string_view>

namespace kinecore::fs {

// Recursive directory creation, analogous to `mkdir -p`. Returns true
// if the directory exists (or was created) at the end of the call.
// Logs the failure via spdlog if a path component cannot be created.
bool ensure_dir(std::string_view path);

}  // namespace kinecore::fs
