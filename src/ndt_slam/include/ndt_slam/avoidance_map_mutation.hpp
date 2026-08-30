#pragma once

#include "ndt_slam/pose_authority_identity.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <string>
#include <utility>

namespace ndt_slam {

struct SourceFrameIdentity {
  std::uint64_t processing_frame_index = 0U;
  double sensor_source_stamp_sec = 0.0;
  std::uint64_t time_epoch_id = 0U;
  std::uint64_t source_cloud_size = 0U;
  std::uint64_t source_cloud_signature_hash = 0U;

  bool valid() const noexcept {
    return processing_frame_index != 0U &&
        std::isfinite(sensor_source_stamp_sec) &&
        sensor_source_stamp_sec > 0.0 && source_cloud_signature_hash != 0U;
  }
};

inline bool sameSourceFrameIdentity(
    const SourceFrameIdentity& lhs,
    const SourceFrameIdentity& rhs,
    double stamp_epsilon_sec = 1.0e-6) noexcept {
  return lhs.valid() && rhs.valid() &&
      lhs.processing_frame_index == rhs.processing_frame_index &&
      std::abs(lhs.sensor_source_stamp_sec - rhs.sensor_source_stamp_sec) <=
          stamp_epsilon_sec &&
      lhs.time_epoch_id == rhs.time_epoch_id &&
      lhs.source_cloud_size == rhs.source_cloud_size &&
      lhs.source_cloud_signature_hash == rhs.source_cloud_signature_hash;
}

inline std::uint32_t canonicalPointFloatBits(float value) noexcept {
  if (value == 0.0F) value = 0.0F;
  std::uint32_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value), "float width changed");
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

inline std::uint64_t hashSourceCloudExact(
    const pcl::PointCloud<pcl::PointXYZ>& cloud) noexcept {
  constexpr std::uint64_t kOffset = 1469598103934665603ULL;
  constexpr std::uint64_t kPrime = 1099511628211ULL;
  std::uint64_t hash = kOffset;
  const auto add = [&](std::uint32_t value, std::uint64_t* state) {
    for (unsigned int shift = 0U; shift < 32U; shift += 8U) {
      *state ^= static_cast<std::uint8_t>(value >> shift);
      *state *= kPrime;
    }
  };
  for (const pcl::PointXYZ& point : cloud.points) {
    add(canonicalPointFloatBits(point.x), &hash);
    add(canonicalPointFloatBits(point.y), &hash);
    add(canonicalPointFloatBits(point.z), &hash);
  }
  const std::uint64_t cloud_size = static_cast<std::uint64_t>(cloud.size());
  add(static_cast<std::uint32_t>(cloud_size), &hash);
  add(static_cast<std::uint32_t>(cloud_size >> 32U), &hash);
  return hash == 0U ? 1U : hash;
}

inline SourceFrameIdentity makeSourceFrameIdentity(
    std::uint64_t processing_frame_index,
    double sensor_source_stamp_sec,
    std::uint64_t time_epoch_id,
    const pcl::PointCloud<pcl::PointXYZ>& cloud) noexcept {
  SourceFrameIdentity identity;
  identity.processing_frame_index = processing_frame_index;
  identity.sensor_source_stamp_sec = sensor_source_stamp_sec;
  identity.time_epoch_id = time_epoch_id;
  identity.source_cloud_size = cloud.size();
  identity.source_cloud_signature_hash = hashSourceCloudExact(cloud);
  return identity;
}

inline bool sourceFrameIdentityMatchesCloud(
    const SourceFrameIdentity& identity,
    const pcl::PointCloud<pcl::PointXYZ>& cloud) noexcept {
  return identity.valid() && identity.source_cloud_size == cloud.size() &&
      identity.source_cloud_signature_hash == hashSourceCloudExact(cloud);
}

struct SourcePointKey {
  std::uint32_t x = 0U;
  std::uint32_t y = 0U;
  std::uint32_t z = 0U;

  bool operator<(const SourcePointKey& other) const noexcept {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
  }
};

inline bool makeSourcePointKey(const pcl::PointXYZ& point,
                               SourcePointKey* key) noexcept {
  if (key == nullptr || !std::isfinite(point.x) ||
      !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return false;
  }
  key->x = canonicalPointFloatBits(point.x);
  key->y = canonicalPointFloatBits(point.y);
  key->z = canonicalPointFloatBits(point.z);
  return true;
}

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
  SourceFrameIdentity source_frame_identity;
  std::set<SourcePointKey> exact_points;
  float voxel_size_m = 0.0F;
  std::set<PointOwnershipVoxel> voxels;

  bool owns(const pcl::PointXYZ& point) const {
    SourcePointKey key;
    return valid && source_frame_identity.valid() &&
        makeSourcePointKey(point, &key) &&
        exact_points.find(key) != exact_points.end();
  }

  bool owns(const SourceFrameIdentity& frame,
            const pcl::PointXYZ& point) const {
    return sameSourceFrameIdentity(source_frame_identity, frame) &&
        owns(point);
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

  bool ownsCurrentPoint(const pcl::PointXYZ& point) const {
    if (!tight_geometry_valid || !owner_points.owns(point)) {
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

  bool owns(const pcl::PointXYZ& point) const {
    return authorized && ownsCurrentPoint(point);
  }
};

// V6 never substitutes an odom-anchor rectangle for missing current-point
// ownership. A loaded frame without exact Cargo map authority is omitted from
// the persistent map; Legacy behavior is intentionally outside this helper.
inline bool shouldDropV6MapCommitWithoutExactCargoOwnership(
    bool v6_authority_mode,
    bool hook_loaded,
    bool canonical_safety_authorized,
    const CargoMapMutationSnapshot& cargo,
    const CurrentFramePointOwnership& candidate_points) noexcept {
  const bool exact_current_authority = cargo.authorized &&
      cargo.tight_geometry_valid && cargo.owner_points.valid &&
      !cargo.owner_points.exact_points.empty();
  const bool exact_current_candidate = !canonical_safety_authorized &&
      candidate_points.valid &&
      candidate_points.source_frame_identity.valid() &&
      !candidate_points.exact_points.empty();
  return v6_authority_mode && hook_loaded && !exact_current_authority &&
      !exact_current_candidate;
}

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
  SourceFrameIdentity source_frame_identity;
  PoseAuthorityIdentity pose_identity;
  HumanCurrentFramePointOwnership human_points;
  CargoMapMutationSnapshot cargo_points;
  CurrentFramePointOwnership cargo_candidate_points;
  bool cargo_canonical_safety_authorized = false;
  StaticLearningBlockCells static_learning_blocks;
  bool localization_map_write_authorized = false;
  bool avoidance_static_write_authorized = false;
  bool display_cleanup_authorized = false;
  MapMutationReason reason = MapMutationReason::INVALID_SOURCE;

  bool validFor(const PoseAuthorityIdentity& expected_identity,
                const SourceFrameIdentity& expected_source_frame,
                double expected_stamp_sec,
                std::uint64_t expected_cloud_instance_id,
                double stamp_epsilon_sec = 1.0e-4) const {
    const bool human_owner_current = human_points.valid &&
        sameSourceFrameIdentity(
            human_points.source_frame_identity, source_frame_identity);
    const bool cargo_owner_current = !cargo_points.authorized ||
        (cargo_points.tight_geometry_valid &&
         cargo_points.owner_points.valid &&
         sameSourceFrameIdentity(
             cargo_points.owner_points.source_frame_identity,
             source_frame_identity));
    const bool cargo_candidate_current = !cargo_candidate_points.valid ||
        sameSourceFrameIdentity(
            cargo_candidate_points.source_frame_identity,
            source_frame_identity);
    return localization_map_write_authorized &&
        source_cloud_instance_id != 0U &&
        source_cloud_instance_id == expected_cloud_instance_id &&
        std::isfinite(source_stamp_sec) &&
        std::isfinite(expected_stamp_sec) &&
        std::abs(source_stamp_sec - expected_stamp_sec) <= stamp_epsilon_sec &&
        sameSourceFrameIdentity(
            source_frame_identity, expected_source_frame) &&
        samePoseAuthorityIdentity(pose_identity, expected_identity) &&
        human_owner_current && cargo_owner_current &&
        cargo_candidate_current;
  }
};

}  // namespace ndt_slam
