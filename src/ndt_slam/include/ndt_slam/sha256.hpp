#pragma once

#include <string>

namespace ndt_slam {

// Portable SHA-256 used by the archive worker and offline identity checks.
// Returns an empty string when the file cannot be read.
std::string sha256File(const std::string& path);

}  // namespace ndt_slam
