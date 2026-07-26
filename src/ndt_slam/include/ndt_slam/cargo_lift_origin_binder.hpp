#pragma once

#include "ndt_slam/static_height_field.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ndt_slam {

enum class CargoLiftEventState : std::uint8_t {
  IDLE = 0,
  PRELOAD_BASELINE_READY,
  LOAD_DETECTED,
  ORIGIN_BINDING,
  LIFT_CONFIRMING,
  THICKNESS_CONFIRMING,
  GEOMETRY_FROZEN,
  TRANSPORT,
  PLACEMENT_CONFIRMING,
  LOADED_REACQUIRE,
  INVALID,
};

const char* cargoLiftEventStateName(CargoLiftEventState state) noexcept;

enum class CargoOriginCandidateSource : std::uint8_t {
  RETIRED_FORMAL_SHAPE = 0,
  OPERATOR_APPROVED_BASELINE = 1,
  RUNTIME_MATURE_STATIC = 2,
  CONFIGURED_ENVELOPE = 3,
};

struct CargoLiftOriginConfig {
  float maximum_anchor_distance_m = 2.0F;
  float minimum_significant_change_m = 0.15F;
  float significance_sigma = 3.0F;
  float minimum_source_coverage = 0.35F;
  float minimum_top_coverage = 0.20F;
  float minimum_revealed_support_coverage = 0.25F;
  int lift_confirm_frames = 4;
  int thickness_confirm_frames = 5;
  double maximum_observation_gap_sec = 0.50;
  double maximum_source_age_sec = 0.50;
  float minimum_height_m = 0.30F;
  float maximum_height_m = 5.00F;
};

struct CargoOriginCandidate {
  std::uint64_t component_id = 0U;
  CargoOriginCandidateSource source =
      CargoOriginCandidateSource::CONFIGURED_ENVELOPE;
  StaticEvidenceAuthority authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  bool predates_cargo_lifecycle = false;
  Eigen::Vector2f center_map = Eigen::Vector2f::Zero();
  float length_m = 0.0F;
  float width_m = 0.0F;
  float yaw_map_rad = 0.0F;
  float top_z95_map = std::numeric_limits<float>::quiet_NaN();
  float support_z_map = std::numeric_limits<float>::quiet_NaN();
  float uncertainty_m = 0.20F;
  std::size_t occupied_cells = 0U;
  std::size_t point_count = 0U;
  std::uint64_t map_generation = 0U;
  std::vector<StaticHeightLayerNodeId> members;
  float hook_anchor_distance_m = 0.0F;
  float candidate_overlap = 0.0F;
  float anchor_overlap = 0.0F;
};

struct CargoLiftOriginInput {
  double stamp_sec = 0.0;
  bool hook_signal_valid = false;
  bool hook_loaded = false;
  bool hook_was_empty = false;
  bool node_started_loaded = false;
  bool anchor_valid = false;
  Eigen::Vector2f hook_anchor_map = Eigen::Vector2f::Zero();
  std::vector<CargoOriginCandidate> candidates;
  bool current_top_valid = false;
  double current_top_stamp_sec = 0.0;
  float current_top_z_map = std::numeric_limits<float>::quiet_NaN();
  float current_top_uncertainty_m = 0.20F;
  float source_coverage = 0.0F;
  float top_coverage = 0.0F;
  bool revealed_support_valid = false;
  double revealed_support_stamp_sec = 0.0;
  float revealed_support_z_map = std::numeric_limits<float>::quiet_NaN();
  float revealed_support_coverage = 0.0F;
};

struct CargoLiftOriginResult {
  bool valid = false;
  bool lift_confirmed = false;
  bool thickness_ready = false;
  CargoLiftEventState state = CargoLiftEventState::IDLE;
  CargoOriginCandidate origin;
  float lift_delta_m = 0.0F;
  float change_threshold_m = 0.15F;
  float static_thickness_m = std::numeric_limits<float>::quiet_NaN();
  float revealed_thickness_m = std::numeric_limits<float>::quiet_NaN();
  int lift_confirm_count = 0;
  int thickness_confirm_count = 0;
  std::string reason = "idle";
};

class CargoLiftOriginBinder {
 public:
  explicit CargoLiftOriginBinder(
      const CargoLiftOriginConfig& config = CargoLiftOriginConfig{});
  void setConfig(const CargoLiftOriginConfig& config);
  void reset();
  CargoLiftOriginResult update(const CargoLiftOriginInput& input);
  const CargoLiftOriginResult& result() const noexcept { return result_; }

 private:
  CargoLiftOriginConfig config_;
  CargoLiftOriginResult result_;
  bool previous_loaded_ = false;
  double last_stamp_sec_ = 0.0;
  double last_valid_lift_stamp_sec_ = 0.0;
  double last_valid_thickness_stamp_sec_ = 0.0;
  double last_consumed_support_stamp_sec_ = 0.0;
  double last_consumed_top_stamp_sec_ = 0.0;
  std::uint64_t bound_component_id_ = 0U;
};

}  // namespace ndt_slam
