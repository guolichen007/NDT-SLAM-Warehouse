#include "ndt_slam/cargo_component_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

constexpr float kPi = 3.14159265358979323846F;

float axialDifference(float lhs, float rhs) {
  float difference = std::remainder(lhs - rhs, kPi);
  return std::abs(difference);
}

bool validConfig(const CargoComponentFusionConfig& config) {
  return std::isfinite(config.maximum_axial_yaw_difference_rad) &&
      config.maximum_axial_yaw_difference_rad >= 0.0F &&
      config.maximum_axial_yaw_difference_rad <= 0.785399F &&
      std::isfinite(config.maximum_longitudinal_gap_m) &&
      config.maximum_longitudinal_gap_m >= 0.0F &&
      std::isfinite(config.maximum_lateral_gap_m) &&
      config.maximum_lateral_gap_m >= 0.0F &&
      std::isfinite(config.minimum_z_overlap_ratio) &&
      config.minimum_z_overlap_ratio >= 0.0F &&
      config.minimum_z_overlap_ratio <= 1.0F &&
      std::isfinite(config.maximum_combined_long_side_m) &&
      config.maximum_combined_long_side_m > 0.0F &&
      std::isfinite(config.maximum_combined_short_side_m) &&
      config.maximum_combined_short_side_m > 0.0F &&
      config.maximum_components >= 1U && config.maximum_components <= 3U;
}

bool validFragment(const CargoComponentFragment& fragment) {
  return fragment.center.allFinite() && std::isfinite(fragment.length_m) &&
      fragment.length_m > 0.0F && std::isfinite(fragment.width_m) &&
      fragment.width_m > 0.0F && fragment.length_m >= fragment.width_m &&
      std::isfinite(fragment.yaw_rad) && std::isfinite(fragment.min_z) &&
      std::isfinite(fragment.max_z) && fragment.max_z > fragment.min_z &&
      fragment.point_count > 0U;
}

bool compatible(const CargoComponentFragment& lhs,
                const CargoComponentFragment& rhs,
                const CargoComponentFusionConfig& config) {
  if (!validFragment(lhs) || !validFragment(rhs) ||
      axialDifference(lhs.yaw_rad, rhs.yaw_rad) >
          config.maximum_axial_yaw_difference_rad) {
    return false;
  }
  const float overlap_z = std::max(
      0.0F, std::min(lhs.max_z, rhs.max_z) -
                std::max(lhs.min_z, rhs.min_z));
  const float smaller_height = std::min(
      lhs.max_z - lhs.min_z, rhs.max_z - rhs.min_z);
  if (smaller_height <= 1.0e-4F ||
      overlap_z / smaller_height < config.minimum_z_overlap_ratio) {
    return false;
  }

  const float cosine = std::cos(lhs.yaw_rad);
  const float sine = std::sin(lhs.yaw_rad);
  const Eigen::Vector2f delta = rhs.center - lhs.center;
  const float along = std::abs(cosine * delta.x() + sine * delta.y());
  const float lateral = std::abs(-sine * delta.x() + cosine * delta.y());
  const float longitudinal_gap = std::max(
      0.0F, along - 0.5F * (lhs.length_m + rhs.length_m));
  const float lateral_gap = std::max(
      0.0F, lateral - 0.5F * (lhs.width_m + rhs.width_m));
  return longitudinal_gap <= config.maximum_longitudinal_gap_m &&
      lateral_gap <= config.maximum_lateral_gap_m;
}

bool combinedBoundsValid(
    const std::vector<std::size_t>& indices,
    const std::vector<CargoComponentFragment>& fragments,
    const CargoComponentFusionConfig& config) {
  if (indices.empty()) return false;
  const float yaw = fragments[indices.front()].yaw_rad;
  const Eigen::Vector2f axis(std::cos(yaw), std::sin(yaw));
  const Eigen::Vector2f normal(-axis.y(), axis.x());
  float min_long = std::numeric_limits<float>::infinity();
  float max_long = -std::numeric_limits<float>::infinity();
  float min_short = std::numeric_limits<float>::infinity();
  float max_short = -std::numeric_limits<float>::infinity();
  for (std::size_t index : indices) {
    const CargoComponentFragment& fragment = fragments[index];
    const float center_long = fragment.center.dot(axis);
    const float center_short = fragment.center.dot(normal);
    min_long = std::min(min_long, center_long - 0.5F * fragment.length_m);
    max_long = std::max(max_long, center_long + 0.5F * fragment.length_m);
    min_short = std::min(min_short, center_short - 0.5F * fragment.width_m);
    max_short = std::max(max_short, center_short + 0.5F * fragment.width_m);
  }
  return max_long - min_long <= config.maximum_combined_long_side_m &&
      max_short - min_short <= config.maximum_combined_short_side_m;
}

}  // namespace

std::vector<CargoComponentHypothesis> buildCargoComponentHypotheses(
    const std::vector<CargoComponentFragment>& fragments,
    const CargoComponentFusionConfig& config) {
  std::vector<CargoComponentHypothesis> hypotheses;
  if (!validConfig(config)) return hypotheses;
  for (std::size_t i = 0U; i < fragments.size(); ++i) {
    if (!validFragment(fragments[i])) continue;
    hypotheses.push_back({{i}, false, "single_component"});
  }
  if (config.maximum_components < 2U) return hypotheses;

  std::vector<std::vector<bool>> edges(
      fragments.size(), std::vector<bool>(fragments.size(), false));
  for (std::size_t i = 0U; i < fragments.size(); ++i) {
    for (std::size_t j = i + 1U; j < fragments.size(); ++j) {
      edges[i][j] = edges[j][i] = compatible(fragments[i], fragments[j], config);
      if (edges[i][j] && combinedBoundsValid({i, j}, fragments, config)) {
        hypotheses.push_back({{i, j}, true, "collinear_pair"});
      }
    }
  }
  if (config.maximum_components < 3U) return hypotheses;
  for (std::size_t i = 0U; i < fragments.size(); ++i) {
    for (std::size_t j = i + 1U; j < fragments.size(); ++j) {
      for (std::size_t k = j + 1U; k < fragments.size(); ++k) {
        const int edge_count = static_cast<int>(edges[i][j]) +
            static_cast<int>(edges[i][k]) + static_cast<int>(edges[j][k]);
        if (edge_count < 2 ||
            !combinedBoundsValid({i, j, k}, fragments, config)) {
          continue;
        }
        hypotheses.push_back({{i, j, k}, true, "collinear_triple"});
      }
    }
  }
  return hypotheses;
}

}  // namespace ndt_slam
