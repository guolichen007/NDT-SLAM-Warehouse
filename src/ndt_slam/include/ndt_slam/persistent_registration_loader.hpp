#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <string>

namespace ndt_slam {

enum class PersistentRegistrationLoadStatus {
  NEW_MAP_BOOTSTRAP = 0,
  RESTORED = 1,
  INVALID_EXISTING_MAP = 2,
};

struct PersistentRegistrationLoadResult {
  PersistentRegistrationLoadStatus status =
      PersistentRegistrationLoadStatus::INVALID_EXISTING_MAP;
  std::string reason;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::size_t tile_count = 0U;
};

// Loads only the authoritative registration layer. An absent manifest and an
// absent registration layer mean a genuinely new map. Any inconsistency once
// registration tiles exist is reported as INVALID_EXISTING_MAP so callers
// cannot silently bootstrap over an existing deployment.
PersistentRegistrationLoadResult loadPersistentRegistrationLayer(
    const std::string& root_directory,
    const std::string& expected_map_uuid,
    double expected_tile_size_m);

}  // namespace ndt_slam
