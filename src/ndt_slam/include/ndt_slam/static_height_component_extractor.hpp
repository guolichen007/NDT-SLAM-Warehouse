#pragma once

#include "ndt_slam/static_height_field.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ndt_slam {

struct StaticHeightComponentExtractorConfig {
  float neighbor_height_center_tolerance_m = 0.18F;
  float neighbor_interval_gap_tolerance_m = 0.12F;
  float maximum_support_height_difference_m = 0.18F;
  std::size_t maximum_component_cells = 4096U;
  std::size_t minimum_component_cells = 4U;
  float maximum_anchor_distance_m = 2.0F;
  float minimum_candidate_overlap = 0.15F;
};

struct StaticHeightComponentQuery {
  Eigen::Vector2f hook_anchor_map = Eigen::Vector2f::Zero();
  bool candidate_valid = false;
  Eigen::Vector2f candidate_center_map = Eigen::Vector2f::Zero();
  float candidate_length_m = 0.0F;
  float candidate_width_m = 0.0F;
  float candidate_yaw_map_rad = 0.0F;
  bool expected_top_valid = false;
  float expected_top_z_map = 0.0F;
  std::uint64_t map_generation = 0U;
  std::uint64_t previous_component_id = 0U;
};

struct StaticHeightComponent {
  bool valid = false;
  std::uint64_t component_id = 0U;
  std::uint64_t map_generation = 0U;
  StaticEvidenceAuthority authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  std::vector<StaticHeightLayerNodeId> members;
  Eigen::Vector2f center_map = Eigen::Vector2f::Zero();
  float length_m = 0.0F;
  float width_m = 0.0F;
  float yaw_map_rad = 0.0F;
  float top_z95_map = 0.0F;
  float support_z_map = 0.0F;
  float uncertainty_m = 0.0F;
  float hook_anchor_distance_m = 0.0F;
  float candidate_overlap = 0.0F;
  float anchor_overlap = 0.0F;
  std::size_t point_count = 0U;
  std::string reason = "invalid";
};

class StaticHeightComponentExtractor {
 public:
  explicit StaticHeightComponentExtractor(
      const StaticHeightComponentExtractorConfig& config =
          StaticHeightComponentExtractorConfig{});

  void setConfig(const StaticHeightComponentExtractorConfig& config);
  const StaticHeightComponentExtractorConfig& config() const noexcept {
    return config_;
  }

  std::vector<StaticHeightComponent> extract(
      const StaticHeightField& field,
      const StaticHeightComponentQuery& query) const;

 private:
  StaticHeightComponentExtractorConfig config_;
};

}  // namespace ndt_slam
