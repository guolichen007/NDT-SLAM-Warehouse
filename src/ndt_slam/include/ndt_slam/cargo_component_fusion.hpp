#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace ndt_slam {

struct CargoComponentFusionConfig {
  float maximum_axial_yaw_difference_rad = 0.261799F;  // 15 degrees
  float maximum_longitudinal_gap_m = 0.60F;
  float maximum_lateral_gap_m = 0.35F;
  float minimum_z_overlap_ratio = 0.30F;
  float maximum_combined_long_side_m = 4.0F;
  float maximum_combined_short_side_m = 3.0F;
  std::size_t maximum_components = 3U;
};

struct CargoComponentFragment {
  Eigen::Vector2f center = Eigen::Vector2f::Zero();
  float length_m = 0.0F;
  float width_m = 0.0F;
  float yaw_rad = 0.0F;
  float min_z = 0.0F;
  float max_z = 0.0F;
  std::size_t point_count = 0U;
};

struct CargoComponentHypothesis {
  std::vector<std::size_t> component_indices;
  bool merged = false;
  std::string reason = "single_component";
};

// Produces singleton and conservative 2/3-component hypotheses. Components
// must be axially aligned, vertically overlapping and separated mainly along
// their long axis. This joins sparse returns from one long cargo without
// increasing the Euclidean clustering tolerance and gluing nearby racks.
std::vector<CargoComponentHypothesis> buildCargoComponentHypotheses(
    const std::vector<CargoComponentFragment>& fragments,
    const CargoComponentFusionConfig& config);

}  // namespace ndt_slam
