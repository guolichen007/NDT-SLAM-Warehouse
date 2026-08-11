#pragma once

#include "ndt_slam/map_session_snapshot.hpp"

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class DurableMapLoadStatus : std::uint8_t {
  LOADED = 0,
  FIRST_BOOT,
  REFERENCE_CORRUPTED,
};

struct DurableMapLoadResult {
  DurableMapLoadStatus status = DurableMapLoadStatus::FIRST_BOOT;
  MapSessionLoadResult session;
  std::string pointer;
  std::string generation;
  std::string reason;
};

// Immutable, generational wrapper around MapSessionSnapshot. CURRENT is the
// only publication point; generations that are not referenced by a pointer
// are harmless staging remnants and are never selected at startup.
class DurableMapStore {
 public:
  explicit DurableMapStore(std::string root_directory);

  bool initialize(std::string* reason = nullptr) const;
  bool save(MapSessionSaveRequest request, std::string* reason = nullptr) const;
  DurableMapLoadResult loadBest() const;

  const std::string& rootDirectory() const { return root_directory_; }

 private:
  std::string root_directory_;
};

}  // namespace ndt_slam
