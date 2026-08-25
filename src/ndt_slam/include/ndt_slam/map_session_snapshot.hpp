#pragma once

#include "ndt_slam/static_obstacle_evidence_index.hpp"
#include "ndt_slam/rail_localization_authority.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace ndt_slam {

struct MapSessionLayers {
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr registration;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr display;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr ground;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr objects_raw;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr objects_clean;
};

struct MapSessionMetadata {
  std::string map_uuid;
  std::string map_frame_uuid;
  std::string frame_id = "map";
  std::string base_frame_id = "base_link";
  std::string map_frame_convention_id;
  std::string sensor_rig_calibration_id;
  RailYawReference yaw_reference;
  bool rail_write_authorized = false;
  std::string source_git_sha = "unknown";
  std::string source_git_branch = "unknown";
  std::uint64_t map_generation = 0U;
  std::uint64_t objects_content_version = 0U;
  std::uint64_t clean_build_version = 0U;
  std::uint64_t keyframe_count = 0U;
  double source_stamp_sec = 0.0;
  bool active_only = false;
  float objects_voxel_size_m = 0.0F;
};

struct MapSessionSaveRequest {
  std::string target_directory;
  MapSessionLayers layers;
  MapSessionMetadata metadata;
  std::shared_ptr<const StaticEvidenceSnapshot> static_evidence;
  // Called while writing the sibling temporary directory. Returning false
  // aborts the entire transaction and removes the temporary directory.
  std::function<bool(const std::string&, std::string*)> write_extras;
};

struct MapSessionLoadResult {
  bool valid = false;
  std::string reason;
  std::string session_directory;
  MapSessionMetadata metadata;
  MapSessionLayers layers;
  std::string static_evidence_path;
  StaticEvidenceAuthority static_authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  std::uint64_t static_evidence_revision = 0U;
  std::uint64_t static_evidence_source_generation = 0U;
};

std::vector<Eigen::Vector3f> selectStaticHeightPointsForAuthority(
    const pcl::PointCloud<pcl::PointXYZ>& objects_clean,
    const StaticEvidenceSnapshot& evidence);

class MapSessionSnapshot {
 public:
  static constexpr std::uint32_t kLegacySchemaVersion = 1U;
  static constexpr std::uint32_t kSchemaVersion = 2U;

  // Atomic for reader visibility through a same-filesystem directory rename.
  // This API does not fsync every file/directory and therefore does not claim
  // crash durability across sudden power loss.
  static bool saveAtomic(const MapSessionSaveRequest& request,
                         std::string* reason);
  static MapSessionLoadResult loadVerified(const std::string& directory);
  static std::string sha256File(const std::string& path,
                                std::string* reason = nullptr);
  static std::string generateUuid();
};

}  // namespace ndt_slam
