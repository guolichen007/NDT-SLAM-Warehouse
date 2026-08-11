#include "ndt_slam/local_map_update_policy.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {

const char* localMapPoseAuthorityName(
    LocalMapPoseAuthority authority) noexcept {
  switch (authority) {
    case LocalMapPoseAuthority::NDT_MEASURED:
      return "NDT_MEASURED";
    case LocalMapPoseAuthority::EKF_PREDICTED:
      return "EKF_PREDICTED";
    case LocalMapPoseAuthority::RECOVERY:
      return "RECOVERY";
    case LocalMapPoseAuthority::INVALID:
      return "INVALID";
  }
  return "INVALID";
}

const char* localMapUpdateModeName(LocalMapUpdateMode mode) noexcept {
  switch (mode) {
    case LocalMapUpdateMode::NONE:
      return "NONE";
    case LocalMapUpdateMode::NORMAL_UPDATE:
      return "NORMAL_UPDATE";
    case LocalMapUpdateMode::MOTION_ESCAPE_REFRESH:
      return "MOTION_ESCAPE_REFRESH";
  }
  return "NONE";
}

const char* localMapHealthStateName(LocalMapHealthState state) noexcept {
  switch (state) {
    case LocalMapHealthState::HEALTHY_UPDATING:
      return "HEALTHY_UPDATING";
    case LocalMapHealthState::IDLE_STATIONARY:
      return "IDLE_STATIONARY";
    case LocalMapHealthState::WAITING_TRIGGER:
      return "WAITING_TRIGGER";
    case LocalMapHealthState::QUARANTINED:
      return "QUARANTINED";
    case LocalMapHealthState::STARVED_MOVING:
      return "STARVED_MOVING";
  }
  return "QUARANTINED";
}

const char* localMapUpdateBlockReasonName(
    LocalMapUpdateBlockReason reason) noexcept {
  switch (reason) {
    case LocalMapUpdateBlockReason::READY:
      return "READY";
    case LocalMapUpdateBlockReason::NO_REGISTRATION:
      return "NO_REGISTRATION";
    case LocalMapUpdateBlockReason::CLOUD_INVALID:
      return "CLOUD_INVALID";
    case LocalMapUpdateBlockReason::RUNTIME_POSE_NONFINITE:
      return "RUNTIME_POSE_NONFINITE";
    case LocalMapUpdateBlockReason::MOTION_STATE_BLOCKED:
      return "MOTION_STATE_BLOCKED";
    case LocalMapUpdateBlockReason::RELOCALIZATION_UNRELIABLE:
      return "RELOCALIZATION_UNRELIABLE";
    case LocalMapUpdateBlockReason::WAITING_UPDATE_TRIGGER:
      return "WAITING_UPDATE_TRIGGER";
  }
  return "UNKNOWN";
}

LocalMapUpdateDecision evaluateLocalMapUpdate(
    const LocalMapUpdateInput& input) noexcept {
  LocalMapUpdateDecision decision;
  if (!input.registration_success) {
    decision.block_reason = LocalMapUpdateBlockReason::NO_REGISTRATION;
    return decision;
  }
  if (!input.registration_cloud_valid) {
    decision.block_reason = LocalMapUpdateBlockReason::CLOUD_INVALID;
    return decision;
  }
  if (!input.runtime_pose_finite) {
    decision.block_reason =
        LocalMapUpdateBlockReason::RUNTIME_POSE_NONFINITE;
    return decision;
  }
  if (!input.relocalization_pose_reliable) {
    decision.block_reason =
        LocalMapUpdateBlockReason::RELOCALIZATION_UNRELIABLE;
    return decision;
  }

  if (!input.normal_motion_update_allowed &&
      !input.motion_escape_refresh_allowed) {
    decision.block_reason =
        LocalMapUpdateBlockReason::MOTION_STATE_BLOCKED;
    return decision;
  }

  decision.eligible = true;
  decision.mode = input.normal_motion_update_allowed
      ? LocalMapUpdateMode::NORMAL_UPDATE
      : LocalMapUpdateMode::MOTION_ESCAPE_REFRESH;
  const bool valid_delta =
      std::isfinite(input.translation_since_update_m) &&
      std::isfinite(input.rotation_since_update_rad) &&
      input.translation_since_update_m >= 0.0 &&
      input.rotation_since_update_rad >= 0.0;
  const bool valid_triggers =
      std::isfinite(input.translation_trigger_m) &&
      std::isfinite(input.rotation_trigger_rad) &&
      input.translation_trigger_m >= 0.0 &&
      input.rotation_trigger_rad >= 0.0 && input.frame_trigger >= 1;
  if (!valid_delta || !valid_triggers) {
    decision.eligible = false;
    decision.mode = LocalMapUpdateMode::NONE;
    decision.block_reason =
        LocalMapUpdateBlockReason::RUNTIME_POSE_NONFINITE;
    return decision;
  }

  decision.due =
      input.translation_since_update_m > input.translation_trigger_m ||
      input.rotation_since_update_rad > input.rotation_trigger_rad ||
      input.frames_since_update > input.frame_trigger;
  decision.block_reason = decision.due
      ? LocalMapUpdateBlockReason::READY
      : LocalMapUpdateBlockReason::WAITING_UPDATE_TRIGGER;
  return decision;
}

void LocalMapPoseAuthorityTracker::reset() noexcept {
  metrics_ = LocalMapPoseAuthorityMetrics{};
  prediction_only_started_sec_ = 0.0;
  trusted_stamp_sec_ = 0.0;
  trusted_x_m_ = 0.0;
  trusted_y_m_ = 0.0;
  trusted_yaw_rad_ = 0.0;
}

LocalMapPoseAuthorityMetrics LocalMapPoseAuthorityTracker::observe(
    const LocalMapPoseSample& sample) noexcept {
  const bool sample_finite = std::isfinite(sample.stamp_sec) &&
      std::isfinite(sample.x_m) && std::isfinite(sample.y_m) &&
      std::isfinite(sample.yaw_rad);
  metrics_.authority = sample_finite
      ? sample.authority : LocalMapPoseAuthority::INVALID;
  if (!sample_finite) {
    metrics_.consecutive_prediction_only_frames = 0;
    metrics_.prediction_only_duration_sec = 0.0;
    prediction_only_started_sec_ = 0.0;
    return metrics_;
  }

  if (metrics_.authority == LocalMapPoseAuthority::NDT_MEASURED) {
    metrics_.trusted_ndt_reference_valid = true;
    trusted_stamp_sec_ = sample.stamp_sec;
    trusted_x_m_ = sample.x_m;
    trusted_y_m_ = sample.y_m;
    trusted_yaw_rad_ = sample.yaw_rad;
    metrics_.consecutive_prediction_only_frames = 0;
    metrics_.prediction_only_duration_sec = 0.0;
    metrics_.frames_since_last_trusted_ndt = 0;
    metrics_.time_since_last_trusted_ndt_sec = 0.0;
    metrics_.distance_since_last_trusted_ndt_m = 0.0;
    metrics_.yaw_since_last_trusted_ndt_rad = 0.0;
    prediction_only_started_sec_ = 0.0;
    return metrics_;
  }

  if (metrics_.trusted_ndt_reference_valid) {
    ++metrics_.frames_since_last_trusted_ndt;
    metrics_.time_since_last_trusted_ndt_sec =
        std::max(0.0, sample.stamp_sec - trusted_stamp_sec_);
    metrics_.distance_since_last_trusted_ndt_m = std::hypot(
        sample.x_m - trusted_x_m_, sample.y_m - trusted_y_m_);
    metrics_.yaw_since_last_trusted_ndt_rad = std::abs(std::atan2(
        std::sin(sample.yaw_rad - trusted_yaw_rad_),
        std::cos(sample.yaw_rad - trusted_yaw_rad_)));
  }

  if (metrics_.authority == LocalMapPoseAuthority::EKF_PREDICTED) {
    if (metrics_.consecutive_prediction_only_frames == 0U) {
      prediction_only_started_sec_ = sample.stamp_sec;
    }
    ++metrics_.consecutive_prediction_only_frames;
    metrics_.prediction_only_duration_sec = std::max(
        0.0, sample.stamp_sec - prediction_only_started_sec_);
  } else {
    metrics_.consecutive_prediction_only_frames = 0;
    metrics_.prediction_only_duration_sec = 0.0;
    prediction_only_started_sec_ = 0.0;
  }
  return metrics_;
}

LocalMapPoseAuthorityMetrics
LocalMapPoseAuthorityTracker::recordLocalMapUpdate(
    LocalMapPoseAuthority authority) noexcept {
  if (authority == LocalMapPoseAuthority::NDT_MEASURED) {
    ++metrics_.local_map_updates_from_measured_pose;
  } else if (authority == LocalMapPoseAuthority::EKF_PREDICTED) {
    ++metrics_.local_map_updates_from_predicted_pose;
  }
  return metrics_;
}

LocalMapHealthState classifyLocalMapHealth(
    const LocalMapHealthInput& input) noexcept {
  if (!input.relocalization_pose_reliable) {
    return LocalMapHealthState::QUARANTINED;
  }
  if (input.stationary_idle && !input.motion_update_expected) {
    return LocalMapHealthState::IDLE_STATIONARY;
  }
  if (input.motion_update_expected && !input.committed &&
      std::isfinite(input.expected_update_age_sec) &&
      std::isfinite(input.starvation_warning_sec) &&
      input.expected_update_age_sec >
          std::max(0.0, input.starvation_warning_sec)) {
    return LocalMapHealthState::STARVED_MOVING;
  }
  if (input.eligible && !input.due) {
    return LocalMapHealthState::WAITING_TRIGGER;
  }
  return LocalMapHealthState::HEALTHY_UPDATING;
}

}  // namespace ndt_slam
