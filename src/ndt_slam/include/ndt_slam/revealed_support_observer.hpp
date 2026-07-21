#pragma once

#include "ndt_slam/static_height_component_extractor.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ndt_slam {

struct RevealedSupportObservationInput {
  double stamp_sec = 0.0;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t map_generation = 0U;
  StaticHeightComponent origin_component;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr observation_cloud_base;
  Sophus::SE3d T_map_base;
  float cell_size_m = 0.25F;
  float support_height_tolerance_m = 0.12F;
  std::size_t visibility_min_points_per_cell = 2U;
};

struct RevealedSupportObservation {
  bool valid = false;
  double evidence_stamp_sec = 0.0;
  std::size_t origin_total_cells = 0U;
  std::size_t origin_observable_cells = 0U;
  std::size_t origin_revealed_cells = 0U;
  float coverage = 0.0F;
  float robust_support_z_map = 0.0F;
  float support_residual_m = 0.0F;
  float uncertainty_m = 0.0F;
  std::string reason = "invalid";
};

class RevealedSupportObserver {
 public:
  void reset();
  RevealedSupportObservation update(
      const RevealedSupportObservationInput& input);
  const RevealedSupportObservation& result() const noexcept { return result_; }

 private:
  RevealedSupportObservation result_;
  std::uint64_t cargo_lifecycle_id_ = 0U;
  std::uint64_t map_generation_ = 0U;
  std::uint64_t component_id_ = 0U;
  double last_stamp_sec_ = 0.0;
  double last_revealed_stamp_sec_ = 0.0;
  std::set<StaticHeightLayerNodeId> observable_members_;
  std::set<StaticHeightLayerNodeId> revealed_members_;
  std::map<StaticHeightLayerNodeId, std::vector<float>> support_samples_;
};

}  // namespace ndt_slam
