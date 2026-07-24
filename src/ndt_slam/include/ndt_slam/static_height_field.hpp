#pragma once

#include "ndt_slam/static_obstacle_evidence_index.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ndt_slam {

struct StaticHeightLayerNodeId {
  std::int64_t cell_key = 0;
  std::uint16_t layer_index = 0U;

  bool operator<(const StaticHeightLayerNodeId& rhs) const noexcept {
    return cell_key < rhs.cell_key ||
        (cell_key == rhs.cell_key && layer_index < rhs.layer_index);
  }

  bool operator==(const StaticHeightLayerNodeId& rhs) const noexcept {
    return cell_key == rhs.cell_key && layer_index == rhs.layer_index;
  }
};

struct StaticHeightFieldConfig {
  float cell_size_m = 0.25F;
  float z_bin_m = 0.10F;
  std::size_t maximum_layers_per_cell = 3U;
  std::size_t minimum_points_per_layer = 6U;
  float maximum_merge_gap_m = 0.18F;
  float ground_fit_min_z_m = -0.25F;
  float ground_fit_max_z_m = 0.25F;
  float support_neighbor_inner_m = 0.30F;
  float support_neighbor_outer_m = 0.80F;
  float minimum_support_uncertainty_m = 0.08F;
  float default_support_uncertainty_m = 0.12F;
  float maximum_support_uncertainty_m = 0.25F;
  std::size_t maximum_query_area_cells = 4096U;
};

struct StaticHeightLayer {
  float z_low = 0.0F;
  float z_high = 0.0F;
  float z05 = 0.0F;
  float z50 = 0.0F;
  float z95 = 0.0F;
  float roughness_m = 0.0F;
  float uncertainty_m = 0.0F;
  std::uint32_t point_count = 0U;
  std::uint32_t observation_count = 0U;
  StaticEvidenceAuthority authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
};

struct StaticSupportSurfaceCell {
  bool valid = false;
  bool interpolated = false;
  bool elevated_or_misclassified = false;
  float z = 0.0F;
  float variance_m2 = 0.0F;
  float uncertainty_m = 0.12F;
  std::uint32_t point_count = 0U;
};

struct StaticHeightCell {
  std::int64_t key = 0;
  std::vector<StaticHeightLayer> layers;
  StaticSupportSurfaceCell support;
};

struct StaticHeightFieldBuildResult {
  bool valid = false;
  std::string reason = "not_built";
  std::size_t finite_object_points = 0U;
  std::size_t finite_ground_points = 0U;
  std::size_t occupied_cells = 0U;
  std::size_t layer_count = 0U;
  std::size_t support_cells = 0U;
  std::size_t elevated_ground_points = 0U;
  Eigen::Vector3f support_plane = Eigen::Vector3f::Zero();
  float support_residual_std_m = 0.0F;
};

struct StaticHeightQuery {
  Eigen::Vector2f center_map = Eigen::Vector2f::Zero();
  float length_m = 0.0F;
  float width_m = 0.0F;
  float yaw_map_rad = 0.0F;
  float shell_m = 0.0F;
  float minimum_z = -std::numeric_limits<float>::infinity();
  float maximum_z = std::numeric_limits<float>::infinity();
  std::size_t maximum_cells = 0U;
  std::set<StaticHeightLayerNodeId> excluded_members;
  std::uint64_t excluded_component_id = 0U;
  std::uint64_t excluded_component_generation = 0U;
  bool exclusion_authorized = false;
  // A current formal cargo can already exist in a previously accumulated
  // clean map. Exclude only layers spatially inside that current rigid body;
  // excluded cells cannot contribute CLEAR coverage.
  bool cargo_self_exclusion_authorized = false;
  float cargo_self_length_m = 0.0F;
  float cargo_self_width_m = 0.0F;
  float cargo_self_minimum_z = 0.0F;
  float cargo_self_maximum_z = 0.0F;
};

struct StaticHeightQueryResult {
  bool valid = false;
  bool bounded = true;
  std::string reason = "not_queried";
  std::size_t queried_cells = 0U;
  std::size_t matched_cells = 0U;
  std::size_t matched_layers = 0U;
  std::size_t covered_cells = 0U;
  std::size_t raw_covered_cells = 0U;
  std::size_t effective_external_covered_cells = 0U;
  std::size_t excluded_origin_cells = 0U;
  std::size_t excluded_cargo_self_cells = 0U;
  std::size_t excluded_layer_count = 0U;
  std::size_t excluded_cargo_self_layer_count = 0U;
  std::size_t clear_shell_queried_cells = 0U;
  std::size_t clear_shell_covered_cells = 0U;
  float coverage_ratio = 0.0F;
  float effective_coverage_ratio = 0.0F;
  float clear_shell_coverage_ratio = 0.0F;
  float nearest_horizontal_distance_m =
      std::numeric_limits<float>::infinity();
  float highest_z95_m = -std::numeric_limits<float>::infinity();
  float highest_uncertainty_m = 0.0F;
  StaticEvidenceAuthority strongest_authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  std::vector<std::int64_t> matched_cell_keys;
};

// Sparse 0.25 m XY height field. It reuses the existing packed-cell contract
// and keeps at most three vertical intervals per cell; no OctoMap/Voxblox
// dependency or full-PCD safety scan is introduced.
class StaticHeightField {
 public:
  explicit StaticHeightField(
      const StaticHeightFieldConfig& config = StaticHeightFieldConfig{});

  void setConfig(const StaticHeightFieldConfig& config);
  const StaticHeightFieldConfig& config() const noexcept { return config_; }
  void clear();

  StaticHeightFieldBuildResult build(
      const std::vector<Eigen::Vector3f>& object_points,
      const std::vector<Eigen::Vector3f>& ground_points,
      StaticEvidenceAuthority authority,
      std::uint32_t observation_count = 1U,
      std::uint64_t map_generation = 0U);

  const StaticHeightCell* cell(std::int64_t key) const;
  StaticSupportSurfaceCell supportAt(const Eigen::Vector2f& xy) const;
  StaticHeightQueryResult query(const StaticHeightQuery& query) const;
  std::size_t cellCount() const noexcept { return cells_.size(); }
  std::size_t layerCount() const noexcept;
  std::uint64_t mapGeneration() const noexcept { return map_generation_; }
  const std::map<std::int64_t, StaticHeightCell>& cells() const noexcept {
    return cells_;
  }

 private:
  StaticHeightFieldConfig config_;
  std::map<std::int64_t, StaticHeightCell> cells_;
  Eigen::Vector3f support_plane_ = Eigen::Vector3f::Zero();
  bool support_plane_valid_ = false;
  std::uint64_t map_generation_ = 0U;
};

}  // namespace ndt_slam
