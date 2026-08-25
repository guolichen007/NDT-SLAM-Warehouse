#include "ndt_slam/cargo_vertical_evidence.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <utility>

namespace ndt_slam {
namespace {

bool finitePositive(float value) {
  return std::isfinite(value) && value > 0.0F;
}

Eigen::Vector2f toFootprintFrame(
    const Eigen::Vector3f& point,
    const CargoVerticalEvidenceInput& input) {
  const Eigen::Vector2f delta = point.head<2>() -
      input.footprint_center_base;
  const float cosine = std::cos(input.footprint_yaw_base_rad);
  const float sine = std::sin(input.footprint_yaw_base_rad);
  return Eigen::Vector2f(
      cosine * delta.x() + sine * delta.y(),
      -sine * delta.x() + cosine * delta.y());
}

float median(std::vector<float> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  return values.size() % 2U == 0U
      ? 0.5F * (values[middle - 1U] + values[middle])
      : values[middle];
}

}  // namespace

bool cargoPointInsideFootprint(
    const Eigen::Vector3f& point,
    const CargoVerticalEvidenceInput& input,
    float margin_m) {
  const Eigen::Vector2f local = toFootprintFrame(point, input);
  return std::abs(local.x()) <=
             0.5F * input.footprint_size_xy.x() + margin_m &&
         std::abs(local.y()) <=
             0.5F * input.footprint_size_xy.y() + margin_m;
}

CargoFootprintGridIndex makeCargoFootprintGridIndex(
    const Eigen::Vector3f& point,
    const CargoVerticalEvidenceInput& input,
    float cell_size_m) {
  const Eigen::Vector2f local = toFootprintFrame(point, input);
  return {
      static_cast<int>(std::floor(local.x() / cell_size_m)),
      static_cast<int>(std::floor(local.y() / cell_size_m))};
}

CargoVerticalEvidence extractCargoVerticalEvidence(
    const CargoVerticalEvidenceInput& input,
    const CargoVerticalEvidenceConfig& config) {
  CargoVerticalEvidence result;
  if (!input.footprint_valid ||
      !input.footprint_center_base.allFinite() ||
      !input.footprint_size_xy.allFinite() ||
      !std::isfinite(input.footprint_yaw_base_rad) ||
      !finitePositive(input.footprint_size_xy.x()) ||
      !finitePositive(input.footprint_size_xy.y())) {
    result.reject_reason = "footprint_invalid";
    return result;
  }
  if (!finitePositive(config.surface_band_height_m) ||
      !finitePositive(config.xy_cell_size_m) ||
      config.minimum_surface_points == 0U ||
      config.minimum_surface_cells == 0U ||
      !std::isfinite(config.minimum_surface_coverage_ratio) ||
      config.minimum_surface_coverage_ratio < 0.0F ||
      config.minimum_surface_coverage_ratio > 1.0F ||
      !std::isfinite(config.footprint_margin_m) ||
      config.footprint_margin_m < 0.0F ||
      !std::isfinite(config.thickness_slab_margin_m) ||
      config.thickness_slab_margin_m < 0.0F) {
    result.reject_reason = "config_invalid";
    return result;
  }

  if (!input.selected_points_base.empty() && input.selected_cloud_base) {
    result.reject_reason = "multiple_point_sources";
    return result;
  }
  const std::size_t selected_point_count = input.selected_cloud_base
      ? input.selected_cloud_base->size() : input.selected_points_base.size();
  std::vector<Eigen::Vector3f> footprint_points;
  footprint_points.reserve(selected_point_count);
  const auto append_point = [&](const Eigen::Vector3f& point) {
    if (!point.allFinite() || !cargoPointInsideFootprint(
            point, input, config.footprint_margin_m)) {
      return;
    }
    footprint_points.push_back(point);
  };
  if (input.selected_cloud_base) {
    for (const pcl::PointXYZ& point : input.selected_cloud_base->points) {
      append_point(Eigen::Vector3f(point.x, point.y, point.z));
    }
  } else {
    for (const Eigen::Vector3f& point : input.selected_points_base) {
      append_point(point);
    }
  }
  result.footprint_points = footprint_points.size();
  if (footprint_points.empty()) {
    result.reject_reason = "no_finite_points_in_footprint";
    return result;
  }

  std::vector<Eigen::Vector3f> candidate_points;
  candidate_points.reserve(footprint_points.size());
  const bool ground_usable = input.ground_reference_valid &&
      std::isfinite(input.ground_z_base) &&
      std::isfinite(config.ground_hag_min_m) &&
      std::isfinite(config.ground_hag_max_m) &&
      config.ground_hag_max_m >= config.ground_hag_min_m;
  if (ground_usable) {
    result.ground_filter_used = true;
    for (const Eigen::Vector3f& point : footprint_points) {
      const float hag = point.z() - input.ground_z_base;
      if (hag >= config.ground_hag_min_m &&
          hag <= config.ground_hag_max_m) {
        candidate_points.push_back(point);
      }
    }
  } else {
    candidate_points = footprint_points;
  }
  if (candidate_points.empty()) {
    result.reject_reason = "ground_filter_removed_all_points";
    return result;
  }
  result.filtered_vertical_points_base = candidate_points;

  std::vector<float> descending_z;
  descending_z.reserve(candidate_points.size());
  for (const Eigen::Vector3f& point : candidate_points) {
    descending_z.push_back(point.z());
  }
  std::sort(descending_z.begin(), descending_z.end(), std::greater<float>());

  const float cell_size = config.xy_cell_size_m;
  const std::size_t expected_cells = static_cast<std::size_t>(
      std::max(1.0, std::ceil(
          static_cast<double>(input.footprint_size_xy.x() / cell_size)))) *
      static_cast<std::size_t>(std::max(1.0, std::ceil(
          static_cast<double>(input.footprint_size_xy.y() / cell_size))));

  std::vector<Eigen::Vector3f> supported_band;
  std::set<CargoFootprintGridIndex> supported_cells;
  for (float band_top : descending_z) {
    supported_band.clear();
    supported_cells.clear();
    const float band_bottom = band_top - config.surface_band_height_m;
    for (const Eigen::Vector3f& point : candidate_points) {
      if (point.z() < band_bottom || point.z() > band_top + 1.0e-4F) {
        continue;
      }
      supported_band.push_back(point);
      supported_cells.insert(
          makeCargoFootprintGridIndex(point, input, cell_size));
    }
    const float coverage = static_cast<float>(supported_cells.size()) /
        static_cast<float>(std::max<std::size_t>(1U, expected_cells));
    if (supported_band.size() >= config.minimum_surface_points &&
        supported_cells.size() >= config.minimum_surface_cells &&
        coverage >= config.minimum_surface_coverage_ratio) {
      result.top_support_points = supported_band.size();
      result.top_surface_cells = supported_cells.size();
      result.top_surface_coverage = std::min(1.0F, coverage);
      result.top_support_points_base = supported_band;
      result.top_surface_cell_indices.assign(
          supported_cells.begin(), supported_cells.end());
      std::vector<float> supported_z;
      supported_z.reserve(supported_band.size());
      for (const Eigen::Vector3f& point : supported_band) {
        supported_z.push_back(point.z());
      }
      // A median of the highest supported band prevents one hook/rope return
      // from becoming the top while retaining a physically observed surface.
      result.top_z_base = median(std::move(supported_z));
      break;
    }
  }

  if (!std::isfinite(result.top_z_base)) {
    result.reject_reason = "no_supported_upper_surface";
    return result;
  }

  const bool thickness_usable = input.frozen_thickness_valid &&
      input.frozen_thickness_matches_lifecycle &&
      finitePositive(input.frozen_thickness_m);
  result.thickness_slab_used = thickness_usable;
  float slab_min_z = result.top_z_base - config.surface_band_height_m;
  if (thickness_usable) {
    slab_min_z = result.top_z_base - input.frozen_thickness_m -
        config.thickness_slab_margin_m;
  } else if (ground_usable) {
    slab_min_z = input.ground_z_base + config.ground_hag_min_m;
  }
  const float slab_max_z = result.top_z_base +
      config.surface_band_height_m;
  result.clean_vertical_points_base.reserve(candidate_points.size());
  for (const Eigen::Vector3f& point : candidate_points) {
    if (point.z() < slab_min_z) {
      ++result.removed_low_points;
      continue;
    }
    if (point.z() <= slab_max_z) {
      result.clean_vertical_points_base.push_back(point);
    }
  }
  if (result.clean_vertical_points_base.empty()) {
    result.reject_reason = "supported_surface_has_no_clean_points";
    return result;
  }

  result.valid = true;
  result.reject_reason = "supported_upper_surface";
  return result;
}

}  // namespace ndt_slam
