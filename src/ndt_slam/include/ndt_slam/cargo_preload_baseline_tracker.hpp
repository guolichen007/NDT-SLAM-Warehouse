#pragma once

#include "ndt_slam/static_height_component_extractor.hpp"

#include <cstddef>
#include <deque>
#include <string>

namespace ndt_slam {

struct CargoPreloadBaselineConfig {
  int minimum_confirm_frames = 5;
  int window_frames = 8;
  double maximum_observation_gap_sec = 0.50;
  float maximum_center_step_m = 0.35F;
  float maximum_thickness_step_m = 0.25F;
  float maximum_thickness_mad_m = 0.10F;
  float maximum_size_relative_step = 0.30F;
  float maximum_anchor_component_distance_m = 0.50F;
  std::size_t minimum_occupied_cells = 6U;
  float maximum_component_uncertainty_m = 0.20F;
};

struct CargoPreloadBaselineInput {
  double stamp_sec = 0.0;
  bool hook_empty = false;
  bool localization_valid = false;
  bool stationary = false;
  StaticHeightComponent component;
};

struct CargoPreloadBaselineResult {
  bool valid = false;
  bool ready = false;
  int confirm_frames = 0;
  int window_valid_frames = 0;
  float thickness_m = 0.0F;
  float thickness_mad_m = 0.0F;
  std::size_t occupied_cells = 0U;
  bool spatially_consistent = false;
  StaticHeightComponent component;
  std::string reason = "not_initialized";
};

// Builds an immutable pre-lift top/support baseline while gravity is EMPTY.
// It consumes the authoritative map-frame height component and never derives
// thickness from base/odom Z, so terrain slope and localization Z corrections
// cannot be mistaken for cargo height.
class CargoPreloadBaselineTracker {
 public:
  explicit CargoPreloadBaselineTracker(
      const CargoPreloadBaselineConfig& config = {});

  void setConfig(const CargoPreloadBaselineConfig& config);
  const CargoPreloadBaselineConfig& config() const noexcept { return config_; }
  void reset();
  CargoPreloadBaselineResult update(
      const CargoPreloadBaselineInput& input);
  const CargoPreloadBaselineResult& result() const noexcept { return result_; }

 private:
  CargoPreloadBaselineConfig config_;
  CargoPreloadBaselineResult result_;
  std::deque<StaticHeightComponent> window_;
  double last_stamp_sec_ = 0.0;
};

}  // namespace ndt_slam
