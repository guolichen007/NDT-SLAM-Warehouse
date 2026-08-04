#include "ndt_slam/cargo_preload_baseline_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ndt_slam {
namespace {

float median(std::vector<float> values) {
  if (values.empty()) return 0.0F;
  const auto middle = values.begin() +
      static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

bool validConfig(const CargoPreloadBaselineConfig& config) {
  return config.minimum_confirm_frames >= 3 &&
      config.window_frames >= config.minimum_confirm_frames &&
      std::isfinite(config.maximum_observation_gap_sec) &&
      config.maximum_observation_gap_sec > 0.0 &&
      std::isfinite(config.maximum_center_step_m) &&
      config.maximum_center_step_m > 0.0F &&
      std::isfinite(config.maximum_thickness_step_m) &&
      config.maximum_thickness_step_m > 0.0F &&
      std::isfinite(config.maximum_thickness_mad_m) &&
      config.maximum_thickness_mad_m >= 0.0F &&
      std::isfinite(config.maximum_size_relative_step) &&
      config.maximum_size_relative_step >= 0.0F &&
      std::isfinite(config.maximum_anchor_component_distance_m) &&
      config.maximum_anchor_component_distance_m > 0.0F &&
      config.minimum_occupied_cells > 0U &&
      std::isfinite(config.maximum_component_uncertainty_m) &&
      config.maximum_component_uncertainty_m > 0.0F;
}

bool finiteComponent(const StaticHeightComponent& component) {
  return component.valid && component.center_map.allFinite() &&
      std::isfinite(component.length_m) && component.length_m > 0.0F &&
      std::isfinite(component.width_m) && component.width_m > 0.0F &&
      std::isfinite(component.top_z95_map) &&
      std::isfinite(component.support_z_map) &&
      component.top_z95_map > component.support_z_map &&
      std::isfinite(component.uncertainty_m) &&
      component.uncertainty_m >= 0.0F &&
      component.authority !=
          StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN &&
      !component.members.empty();
}

}  // namespace

CargoPreloadBaselineTracker::CargoPreloadBaselineTracker(
    const CargoPreloadBaselineConfig& config) {
  setConfig(config);
}

void CargoPreloadBaselineTracker::setConfig(
    const CargoPreloadBaselineConfig& config) {
  config_ = validConfig(config) ? config : CargoPreloadBaselineConfig{};
  reset();
}

void CargoPreloadBaselineTracker::reset() {
  result_ = CargoPreloadBaselineResult{};
  window_.clear();
  last_stamp_sec_ = 0.0;
}

CargoPreloadBaselineResult CargoPreloadBaselineTracker::update(
    const CargoPreloadBaselineInput& input) {
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0 ||
      (last_stamp_sec_ > 0.0 && input.stamp_sec <= last_stamp_sec_)) {
    reset();
    result_.reason = "source_time_invalid_or_rollback";
    return result_;
  }
  if (last_stamp_sec_ > 0.0 &&
      input.stamp_sec - last_stamp_sec_ >
          config_.maximum_observation_gap_sec) {
    window_.clear();
  }
  last_stamp_sec_ = input.stamp_sec;
  if (!input.hook_empty || !input.localization_valid || !input.stationary) {
    window_.clear();
    result_ = CargoPreloadBaselineResult{};
    result_.reason = !input.hook_empty
        ? "hook_not_empty"
        : (!input.localization_valid ? "localization_invalid"
                                     : "base_not_stationary");
    return result_;
  }
  if (!finiteComponent(input.component)) {
    window_.clear();
    result_ = CargoPreloadBaselineResult{};
    result_.reason = "authoritative_component_invalid";
    return result_;
  }
  if (!std::isfinite(input.component.hook_anchor_distance_m) ||
      input.component.hook_anchor_distance_m >
          config_.maximum_anchor_component_distance_m) {
    window_.clear();
    result_ = CargoPreloadBaselineResult{};
    result_.reason = "anchor_component_spatially_uncertain";
    return result_;
  }
  if (input.component.members.size() < config_.minimum_occupied_cells) {
    window_.clear();
    result_ = CargoPreloadBaselineResult{};
    result_.reason = "baseline_spatial_coverage_insufficient";
    return result_;
  }
  if (input.component.uncertainty_m >
      config_.maximum_component_uncertainty_m) {
    window_.clear();
    result_ = CargoPreloadBaselineResult{};
    result_.reason = "baseline_uncertainty_exceeded";
    return result_;
  }

  if (!window_.empty()) {
    const StaticHeightComponent& previous = window_.back();
    const float previous_thickness =
        previous.top_z95_map - previous.support_z_map;
    const float current_thickness =
        input.component.top_z95_map - input.component.support_z_map;
    const Eigen::Array2f relative_size =
        (Eigen::Vector2f(input.component.length_m, input.component.width_m) -
         Eigen::Vector2f(previous.length_m, previous.width_m))
            .cwiseAbs().array() /
        Eigen::Vector2f(previous.length_m, previous.width_m)
            .cwiseMax(Eigen::Vector2f::Constant(0.05F)).array();
    const bool same_physical_component =
        input.component.map_generation == previous.map_generation &&
        input.component.component_id == previous.component_id &&
        (input.component.center_map - previous.center_map).norm() <=
            config_.maximum_center_step_m &&
        std::abs(current_thickness - previous_thickness) <=
            config_.maximum_thickness_step_m &&
        relative_size.maxCoeff() <= config_.maximum_size_relative_step;
    if (!same_physical_component) window_.clear();
  }
  window_.push_back(input.component);
  while (window_.size() >
         static_cast<std::size_t>(config_.window_frames)) {
    window_.pop_front();
  }

  std::vector<float> thicknesses;
  std::vector<float> tops;
  std::vector<float> supports;
  std::vector<float> centers_x;
  std::vector<float> centers_y;
  std::vector<float> lengths;
  std::vector<float> widths;
  thicknesses.reserve(window_.size());
  tops.reserve(window_.size());
  supports.reserve(window_.size());
  centers_x.reserve(window_.size());
  centers_y.reserve(window_.size());
  lengths.reserve(window_.size());
  widths.reserve(window_.size());
  for (const StaticHeightComponent& component : window_) {
    thicknesses.push_back(component.top_z95_map - component.support_z_map);
    tops.push_back(component.top_z95_map);
    supports.push_back(component.support_z_map);
    centers_x.push_back(component.center_map.x());
    centers_y.push_back(component.center_map.y());
    lengths.push_back(component.length_m);
    widths.push_back(component.width_m);
  }
  const float thickness = median(thicknesses);
  std::vector<float> deviations;
  deviations.reserve(thicknesses.size());
  for (const float value : thicknesses) {
    deviations.push_back(std::abs(value - thickness));
  }
  const float mad = median(deviations);

  result_.valid = true;
  result_.spatially_consistent = true;
  result_.window_valid_frames = static_cast<int>(window_.size());
  result_.confirm_frames = mad <= config_.maximum_thickness_mad_m
      ? static_cast<int>(window_.size()) : 0;
  result_.ready = result_.confirm_frames >= config_.minimum_confirm_frames;
  result_.thickness_m = thickness;
  result_.thickness_mad_m = mad;
  result_.component = window_.back();
  result_.component.top_z95_map = median(tops);
  result_.component.support_z_map = median(supports);
  result_.component.center_map = Eigen::Vector2f(
      median(centers_x), median(centers_y));
  result_.component.length_m = median(lengths);
  result_.component.width_m = median(widths);
  result_.component.uncertainty_m = std::max(
      result_.component.uncertainty_m, mad);
  result_.occupied_cells = result_.component.members.size();
  result_.reason = result_.ready
      ? "preload_baseline_ready"
      : (mad <= config_.maximum_thickness_mad_m
             ? "preload_baseline_confirmation_pending"
             : "preload_baseline_thickness_unstable");
  return result_;
}

}  // namespace ndt_slam
