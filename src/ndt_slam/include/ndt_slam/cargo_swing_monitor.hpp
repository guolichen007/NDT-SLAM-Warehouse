#pragma once

#include "ndt_slam/stationary_motion_policy.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <deque>
#include <string>

namespace ndt_slam {

enum class HoistMotionState : std::uint8_t {
  UNKNOWN = 0,
  STOPPED,
  UP,
  DOWN,
};

enum class CargoSwingObservationState : std::uint8_t {
  INVALID = 0,
  CURRENT_MEASUREMENT,
  SHORT_GAP_HOLD,
  STALE,
  TRACK_CHANGED,
  TIMESTAMP_ROLLBACK,
};

enum class CargoSwayState : std::uint8_t {
  NOT_EVALUATED = 0,
  NORMAL,
  SWAY_DETECTED,
  SWAY_WARNING,
  SWAY_ALARM,
  SETTLING,
};

enum class CargoSkewPullState : std::uint8_t {
  NOT_EVALUATED = 0,
  NO_SKEW_PULL,
  SKEW_PULL_SUSPECTED,
  SKEW_PULL_ALARM,
};

enum class CargoTorsionState : std::uint8_t {
  NOT_EVALUATED = 0,
  NORMAL,
  TORSION_DETECTED,
  TORSION_WARNING,
  TORSION_ALARM,
};

enum class CargoSwingRecommendedAction : std::uint8_t {
  NONE = 0,
  WATCH,
  REDUCE_TRAVEL_SPEED,
  STOP_TRAVEL,
  INHIBIT_HOIST_UP,
  STOP_AND_SETTLE,
};

enum class CargoRopeLengthSource : std::uint8_t {
  MEASURED = 0,
  CONFIG_FALLBACK,
  INVALID,
};

struct CargoSwingConfig {
  bool enabled = true;
  float configured_sling_length_m = 1.00F;
  float minimum_rope_length_m = 0.30F;
  double minimum_valid_observation_sec = 0.50;
  double maximum_observation_gap_sec = 0.30;
  double history_window_sec = 4.00;
  double stationary_settle_delay_sec = 0.30;
  float measurement_filter_alpha = 0.35F;
  float normal_angle_deg = 2.0F;
  float warning_angle_deg = 3.0F;
  float alarm_angle_deg = 5.0F;
  float warning_offset_m = 0.20F;
  float alarm_offset_m = 0.50F;
  float normal_speed_mps = 0.15F;
  float warning_speed_mps = 0.30F;
  float sway_end_angle_deg = 1.0F;
  float sway_end_speed_mps = 0.10F;
  double sway_end_confirm_sec = 2.0;
  float skew_suspect_angle_deg = 3.0F;
  float skew_alarm_angle_deg = 5.0F;
  float skew_direction_consistency = 0.80F;
  double skew_min_duration_sec = 1.00;
  double skew_alarm_confirm_sec = 0.50;
  int skew_max_zero_crossings = 1;
  float torsion_detect_deg = 3.0F;
  float torsion_warning_deg = 5.0F;
  float torsion_alarm_deg = 10.0F;
  bool allow_skew_alarm_without_hoist_up = false;
};

struct CargoSwingInput {
  double stamp_sec = 0.0;
  bool localization_valid = false;
  bool hook_loaded = false;
  bool hook_anchor_valid = false;
  Eigen::Vector3f hook_anchor_base = Eigen::Vector3f::Zero();
  std::string hook_anchor_source = "config";
  bool track_retained = false;
  bool track_locked = false;
  bool observation_associated_current = false;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t track_segment_id = 0U;
  bool measured_center_valid = false;
  Eigen::Vector3f measured_center_base = Eigen::Vector3f::Zero();
  bool filtered_center_valid = false;
  Eigen::Vector3f filtered_center_base = Eigen::Vector3f::Zero();
  float horizontal_tracking_residual_m = 0.0F;
  float identity_confidence = 0.0F;
  float shape_confidence = 0.0F;
  bool locked_yaw_valid = false;
  float locked_yaw_base_rad = 0.0F;
  bool measured_yaw_valid = false;
  float measured_yaw_base_rad = 0.0F;
  RuntimeMotionState crane_motion_state = RuntimeMotionState::MOVING;
  HoistMotionState hoist_motion_state = HoistMotionState::UNKNOWN;
  bool hoist_state_fresh = false;
  float hoist_speed_mps = 0.0F;
};

struct CargoSwingResult {
  bool valid = false;
  CargoSwingObservationState observation_state =
      CargoSwingObservationState::INVALID;
  CargoSwayState sway_state = CargoSwayState::NOT_EVALUATED;
  CargoSkewPullState skew_pull_state =
      CargoSkewPullState::NOT_EVALUATED;
  CargoTorsionState torsion_state =
      CargoTorsionState::NOT_EVALUATED;
  Eigen::Vector2f offset_xy_m = Eigen::Vector2f::Zero();
  float offset_m = 0.0F;
  float rope_length_m = 0.0F;
  CargoRopeLengthSource rope_length_source = CargoRopeLengthSource::INVALID;
  float angle_deg = 0.0F;
  float horizontal_speed_mps = 0.0F;
  float radial_speed_mps = 0.0F;
  float oscillation_amplitude_m = 0.0F;
  float direction_consistency = 0.0F;
  int zero_crossings = 0;
  float yaw_error_deg = 0.0F;
  float observation_age_sec = 0.0F;
  float state_duration_sec = 0.0F;
  bool hoist_up_confirmed = false;
  bool alarm_inhibited = false;
  CargoSwingRecommendedAction recommended_action =
      CargoSwingRecommendedAction::NONE;
  std::string reason = "not_evaluated";
};

class CargoSwingMonitor {
 public:
  explicit CargoSwingMonitor(
      const CargoSwingConfig& config = CargoSwingConfig{});
  void setConfig(const CargoSwingConfig& config);
  void reset();
  CargoSwingResult update(const CargoSwingInput& input);
  const CargoSwingResult& result() const noexcept { return result_; }

 private:
  struct Sample {
    double stamp_sec = 0.0;
    Eigen::Vector2f offset = Eigen::Vector2f::Zero();
    float angle_deg = 0.0F;
    float yaw_error_deg = 0.0F;
  };

  CargoSwingConfig config_;
  CargoSwingResult result_;
  std::deque<Sample> history_;
  std::uint64_t cargo_lifecycle_id_ = 0U;
  std::uint64_t track_segment_id_ = 0U;
  std::string hook_anchor_source_;
  double last_input_stamp_sec_ = 0.0;
  double last_measurement_stamp_sec_ = 0.0;
  double state_change_stamp_sec_ = 0.0;
  double sway_below_end_stamp_sec_ = 0.0;
  double skew_alarm_candidate_stamp_sec_ = 0.0;
  bool filtered_offset_valid_ = false;
  Eigen::Vector2f filtered_offset_ = Eigen::Vector2f::Zero();
};

float shortestAxialAngle(float lhs_rad, float rhs_rad);

}  // namespace ndt_slam
