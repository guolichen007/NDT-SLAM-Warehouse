#pragma once

#include "ndt_slam/rail_localization_authority.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <string>

namespace ndt_slam {

struct RegistrationTargetSnapshot {
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
  RegistrationTargetSource source = RegistrationTargetSource::UNKNOWN;
  std::uint64_t target_snapshot_id = 0U;
  std::uint64_t content_version = 0U;
  std::uint64_t map_rebuild_generation = 0U;
  std::string map_frame_uuid;
  std::string crop_identity;

  bool valid() const noexcept {
    return cloud && !cloud->empty() &&
        source != RegistrationTargetSource::UNKNOWN &&
        target_snapshot_id != 0U;
  }
};

RegistrationTargetSource registrationTargetSourceFromName(
    const std::string& source) noexcept;

RegistrationTargetSnapshot makeRegistrationTargetSnapshot(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    RegistrationTargetSource source,
    std::uint64_t content_version,
    std::uint64_t map_rebuild_generation,
    const std::string& map_frame_uuid,
    const std::string& crop_identity);

}  // namespace ndt_slam
