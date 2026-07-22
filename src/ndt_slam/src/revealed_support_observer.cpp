#include "ndt_slam/revealed_support_observer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace ndt_slam {
namespace {

float median(std::vector<float> values) {
  if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
  const std::size_t middle = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  float result = values[middle];
  if (values.size() % 2U == 0U) {
    result = 0.5F * (result + *std::max_element(
        values.begin(), values.begin() + middle));
  }
  return result;
}

}  // namespace

void RevealedSupportObserver::reset() {
  result_ = RevealedSupportObservation{};
  cargo_lifecycle_id_ = 0U;
  map_generation_ = 0U;
  component_id_ = 0U;
  last_stamp_sec_ = 0.0;
  last_revealed_stamp_sec_ = 0.0;
}

RevealedSupportObservation RevealedSupportObserver::update(
    const RevealedSupportObservationInput& input) {
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0 ||
      input.cargo_lifecycle_id == 0U || input.map_generation == 0U ||
      !input.origin_component.valid ||
      input.origin_component.component_id == 0U ||
      input.origin_component.map_generation != input.map_generation ||
      input.origin_component.members.empty() ||
      !input.observation_cloud_base ||
      !input.T_map_base.matrix().allFinite() ||
      !std::isfinite(input.cell_size_m) || input.cell_size_m <= 0.0F ||
      !std::isfinite(input.support_height_tolerance_m) ||
      input.support_height_tolerance_m <= 0.0F ||
      input.visibility_min_points_per_cell == 0U) {
    result_.valid = false;
    result_.reason = "invalid_observation_contract";
    return result_;
  }
  const bool identity_changed =
      cargo_lifecycle_id_ != input.cargo_lifecycle_id ||
      map_generation_ != input.map_generation ||
      component_id_ != input.origin_component.component_id;
  if (identity_changed ||
      (last_stamp_sec_ > 0.0 && input.stamp_sec <= last_stamp_sec_)) {
    reset();
  }
  cargo_lifecycle_id_ = input.cargo_lifecycle_id;
  map_generation_ = input.map_generation;
  component_id_ = input.origin_component.component_id;
  last_stamp_sec_ = input.stamp_sec;

  std::map<std::int64_t, std::vector<float>> observed_z;
  for (const pcl::PointXYZ& point : input.observation_cloud_base->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const Eigen::Vector3d point_map = input.T_map_base *
        Eigen::Vector3d(point.x, point.y, point.z);
    if (!point_map.allFinite()) continue;
    const std::int64_t key = packStaticEvidenceCell(
        static_cast<std::int32_t>(std::floor(
            point_map.x() / input.cell_size_m)),
        static_cast<std::int32_t>(std::floor(
            point_map.y() / input.cell_size_m)));
    observed_z[key].push_back(static_cast<float>(point_map.z()));
  }

  std::set<std::int64_t> counted_cells;
  std::set<std::int64_t> current_observable_cells;
  std::set<std::int64_t> current_revealed_cells;
  std::vector<float> current_support_samples;
  std::size_t current_observable = 0U;
  for (const StaticHeightLayerNodeId& member :
       input.origin_component.members) {
    if (!counted_cells.insert(member.cell_key).second) continue;
    const std::map<std::int64_t, std::vector<float>>::const_iterator found =
        observed_z.find(member.cell_key);
    if (found == observed_z.end() ||
        found->second.size() < input.visibility_min_points_per_cell) {
      // NOT_IN_VIEW/UNKNOWN never clears accumulated evidence.
      continue;
    }
    ++current_observable;
    current_observable_cells.insert(member.cell_key);
    std::vector<float> support_points;
    std::size_t origin_layer_points = 0U;
    for (const float z : found->second) {
      if (std::abs(z - input.origin_component.support_z_map) <=
          input.support_height_tolerance_m) {
        support_points.push_back(z);
      }
      if (z > input.origin_component.support_z_map +
                  input.support_height_tolerance_m &&
          z <= input.origin_component.top_z95_map +
                  input.support_height_tolerance_m) {
        ++origin_layer_points;
      }
    }
    // A cell that still contains the original object-height layer is
    // observable but not revealed. Sparse ground leakage must not confirm a
    // lift while the source object remains in place.
    const bool origin_layer_still_present =
        origin_layer_points >= input.visibility_min_points_per_cell;
    if (!origin_layer_still_present &&
        support_points.size() >= input.visibility_min_points_per_cell) {
      current_revealed_cells.insert(member.cell_key);
      last_revealed_stamp_sec_ = input.stamp_sec;
      current_support_samples.insert(
          current_support_samples.end(), support_points.begin(),
          support_points.end());
    }
  }

  std::set<std::int64_t> total_cells;
  for (const StaticHeightLayerNodeId& member :
       input.origin_component.members) {
    total_cells.insert(member.cell_key);
  }
  const float robust_support = median(current_support_samples);
  std::vector<float> residuals;
  residuals.reserve(current_support_samples.size());
  if (std::isfinite(robust_support)) {
    for (const float value : current_support_samples) {
      residuals.push_back(std::abs(value - robust_support));
    }
  }
  const float residual = median(residuals);
  // Visibility without a new support return must not refresh source
  // freshness. Formal coverage and height use this frame only; old evidence
  // is retained solely as a diagnostic timestamp.
  result_.evidence_stamp_sec = last_revealed_stamp_sec_;
  result_.origin_total_cells = total_cells.size();
  result_.origin_observable_cells = current_observable_cells.size();
  result_.origin_revealed_cells = current_revealed_cells.size();
  result_.coverage = result_.origin_observable_cells > 0U
      ? static_cast<float>(result_.origin_revealed_cells) /
            static_cast<float>(result_.origin_observable_cells)
      : 0.0F;
  result_.robust_support_z_map = robust_support;
  result_.support_residual_m = std::isfinite(residual) ? residual : 0.0F;
  result_.uncertainty_m = std::max(
      0.5F * input.support_height_tolerance_m,
      std::isfinite(residual) ? 1.4826F * residual
                              : input.support_height_tolerance_m);
  result_.valid = current_observable > 0U &&
      result_.origin_revealed_cells > 0U &&
      std::isfinite(result_.robust_support_z_map);
  result_.reason = result_.valid
      ? "origin_support_cells_revealed"
      : (current_observable == 0U
             ? "origin_not_observable"
             : "support_not_revealed");
  return result_;
}

}  // namespace ndt_slam
