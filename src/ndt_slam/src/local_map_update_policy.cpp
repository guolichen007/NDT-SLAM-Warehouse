#include "ndt_slam/local_map_update_policy.hpp"

#include <cmath>

namespace ndt_slam {

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
  decision.attempted = input.registration_success;
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
  if (!input.motion_state_allows_update) {
    decision.block_reason =
        LocalMapUpdateBlockReason::MOTION_STATE_BLOCKED;
    return decision;
  }
  if (!input.relocalization_pose_reliable) {
    decision.block_reason =
        LocalMapUpdateBlockReason::RELOCALIZATION_UNRELIABLE;
    return decision;
  }

  decision.allowed = true;
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
    decision.allowed = false;
    decision.block_reason =
        LocalMapUpdateBlockReason::RUNTIME_POSE_NONFINITE;
    return decision;
  }

  decision.should_update =
      input.translation_since_update_m > input.translation_trigger_m ||
      input.rotation_since_update_rad > input.rotation_trigger_rad ||
      input.frames_since_update > input.frame_trigger;
  decision.block_reason = decision.should_update
      ? LocalMapUpdateBlockReason::READY
      : LocalMapUpdateBlockReason::WAITING_UPDATE_TRIGGER;
  return decision;
}

}  // namespace ndt_slam
