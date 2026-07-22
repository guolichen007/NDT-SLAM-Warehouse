#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class CargoPhysicalMotionState : std::uint8_t {
  UNKNOWN = 0,
  MOVING,
  STOPPING_SETTLE,
  STATIONARY,
};

enum class CargoPhysicalMotionSource : std::uint8_t {
  NONE = 0,
  EXTERNAL_CONTROLLER,
  RAW_POSE_DELTA,
};

struct CargoPhysicalMotionConfig {
  bool enabled = true;
  float enter_stationary_speed_mps = 0.03F;
  float exit_stationary_speed_mps = 0.08F;
  double enter_stationary_confirm_sec = 0.50;
  double exit_stationary_confirm_sec = 0.20;
  double maximum_sample_gap_sec = 0.50;
  double confidence_decay_tau_sec = 0.25;
  float minimum_valid_confidence = 0.35F;
  float maximum_physical_speed_mps = 3.0F;
  float maximum_raw_pose_innovation_m = 0.75F;
  float velocity_filter_alpha = 0.30F;
};

struct CargoPhysicalMotionInput {
  double stamp_sec = 0.0;
  bool external_state_valid = false;
  CargoPhysicalMotionState external_state =
      CargoPhysicalMotionState::UNKNOWN;
  bool raw_position_valid = false;
  Eigen::Vector2d raw_position = Eigen::Vector2d::Zero();
  bool raw_pose_quality_valid = true;
  float raw_pose_innovation_m = 0.0F;
  float raw_pose_step_m = 0.0F;
  bool localization_degenerate = false;
};

struct CargoPhysicalMotionResult {
  bool valid = false;
  CargoPhysicalMotionState state = CargoPhysicalMotionState::UNKNOWN;
  CargoPhysicalMotionSource source = CargoPhysicalMotionSource::NONE;
  float filtered_speed_mps = 0.0F;
  float confidence = 0.0F;
  double state_duration_sec = 0.0;
  std::uint64_t source_epoch = 0U;
  std::string reason = "not_evaluated";
};

class CargoPhysicalMotionEstimator {
 public:
  explicit CargoPhysicalMotionEstimator(
      const CargoPhysicalMotionConfig& config =
          CargoPhysicalMotionConfig{});

  void setConfig(const CargoPhysicalMotionConfig& config);
  void reset();
  CargoPhysicalMotionResult update(const CargoPhysicalMotionInput& input);
  const CargoPhysicalMotionResult& result() const noexcept { return result_; }

 private:
  void setState(CargoPhysicalMotionState state, double stamp_sec);
  void resetKinematics(bool preserve_epoch);

  CargoPhysicalMotionConfig config_;
  CargoPhysicalMotionResult result_;
  double last_stamp_sec_ = 0.0;
  double last_valid_sample_stamp_sec_ = 0.0;
  double state_since_sec_ = 0.0;
  double transition_candidate_since_sec_ = 0.0;
  bool previous_position_valid_ = false;
  Eigen::Vector2d previous_position_ = Eigen::Vector2d::Zero();
  bool filtered_speed_valid_ = false;
};

const char* cargoPhysicalMotionStateName(
    CargoPhysicalMotionState state) noexcept;
const char* cargoPhysicalMotionSourceName(
    CargoPhysicalMotionSource source) noexcept;

}  // namespace ndt_slam
