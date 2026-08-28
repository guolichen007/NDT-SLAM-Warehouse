#pragma once

#include "ndt_slam/pose_authority_identity.hpp"

#include <pcl/point_types.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <utility>

namespace ndt_slam {

struct PointOwnershipVoxel {
  std::int32_t x = 0;
  std::int32_t y = 0;
  std::int32_t z = 0;

  bool operator<(const PointOwnershipVoxel& other) const noexcept {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
  }
};

inline bool makePointOwnershipVoxel(const pcl::PointXYZ& point,
                                    float voxel_size_m,
                                    PointOwnershipVoxel* voxel) {
  if (voxel == nullptr || !std::isfinite(voxel_size_m) ||
      voxel_size_m <= 0.0F || !std::isfinite(point.x) ||
      !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return false;
  }
  const double x = std::floor(point.x / voxel_size_m);
  const double y = std::floor(point.y / voxel_size_m);
  const double z = std::floor(point.z / voxel_size_m);
  if (x < std::numeric_limits<std::int32_t>::lowest() ||
      x > std::numeric_limits<std::int32_t>::max() ||
      y < std::numeric_limits<std::int32_t>::lowest() ||
      y > std::numeric_limits<std::int32_t>::max() ||
      z < std::numeric_limits<std::int32_t>::lowest() ||
      z > std::numeric_limits<std::int32_t>::max()) {
    return false;
  }
  voxel->x = static_cast<std::int32_t>(x);
  voxel->y = static_cast<std::int32_t>(y);
  voxel->z = static_cast<std::int32_t>(z);
  return true;
}

struct CurrentFramePointOwnership {
  bool valid = false;
  float voxel_size_m = 0.0F;
  std::set<PointOwnershipVoxel> voxels;

  bool owns(const pcl::PointXYZ& point) const {
    PointOwnershipVoxel voxel;
    return valid && makePointOwnershipVoxel(point, voxel_size_m, &voxel) &&
        voxels.find(voxel) != voxels.end();
  }
};

using HumanCurrentFramePointOwnership = CurrentFramePointOwnership;

struct CargoMapMutationSnapshot {
  bool authorized = false;
  CurrentFramePointOwnership owner_points;
  bool tight_geometry_valid = false;
  float center_x = 0.0F;
  float center_y = 0.0F;
  float min_z = 0.0F;
  float max_z = 0.0F;
  float half_length = 0.0F;
  float half_width = 0.0F;
  float yaw_rad = 0.0F;

  bool owns(const pcl::PointXYZ& point) const {
    if (!authorized || !tight_geometry_valid || !owner_points.owns(point)) {
      return false;
    }
    const float dx = point.x - center_x;
    const float dy = point.y - center_y;
    const float cosine = std::cos(yaw_rad);
    const float sine = std::sin(yaw_rad);
    const float local_x = cosine * dx + sine * dy;
    const float local_y = -sine * dx + cosine * dy;
    return std::abs(local_x) <= half_length &&
        std::abs(local_y) <= half_width && point.z >= min_z &&
        point.z <= max_z;
  }
};

struct StaticLearningBlockCells {
  float cell_size_m = 0.15F;
  std::set<std::pair<int, int>> human_cells;
  std::set<std::pair<int, int>> cargo_cells;
};

enum class MapMutationReason : std::uint8_t {
  AUTHORIZED = 0,
  LOCALIZATION_NOT_AUTHORIZED = 1,
  INVALID_SOURCE = 2,
  INVALID_POSE_IDENTITY = 3,
  STALE_SNAPSHOT = 4,
};

struct AvoidanceMapMutationSnapshot {
  double source_stamp_sec = 0.0;
  std::uint64_t source_cloud_instance_id = 0U;
  PoseAuthorityIdentity pose_identity;
  HumanCurrentFramePointOwnership human_points;
  CargoMapMutationSnapshot cargo_points;
  StaticLearningBlockCells static_learning_blocks;
  bool localization_map_write_authorized = false;
  bool avoidance_static_write_authorized = false;
  bool display_cleanup_authorized = false;
  MapMutationReason reason = MapMutationReason::INVALID_SOURCE;

  bool validFor(const PoseAuthorityIdentity& expected_identity,
                double expected_stamp_sec,
                std::uint64_t expected_cloud_instance_id,
                double stamp_epsilon_sec = 1.0e-4) const {
    return localization_map_write_authorized &&
        source_cloud_instance_id != 0U &&
        source_cloud_instance_id == expected_cloud_instance_id &&
        std::isfinite(source_stamp_sec) &&
        std::isfinite(expected_stamp_sec) &&
        std::abs(source_stamp_sec - expected_stamp_sec) <= stamp_epsilon_sec &&
        samePoseAuthorityIdentity(pose_identity, expected_identity);
  }
};

}  // namespace ndt_slam
