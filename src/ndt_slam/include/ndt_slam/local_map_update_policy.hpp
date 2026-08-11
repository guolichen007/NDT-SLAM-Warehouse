#pragma once

#include <cstdint>

namespace ndt_slam {

// Runtime local-map authority is deliberately narrower than persistent-map
// authority.  It owns only the short-lived NDT target lifecycle and therefore
// must not consume AcceptedLocalizationSnapshot, MapWriteAuthority, fitness
// measurement authority, or CleanWorkerLineage.
enum class LocalMapUpdateBlockReason : std::uint8_t {
  READY = 0,
  NO_REGISTRATION,
  CLOUD_INVALID,
  RUNTIME_POSE_NONFINITE,
  MOTION_STATE_BLOCKED,
  RELOCALIZATION_UNRELIABLE,
  WAITING_UPDATE_TRIGGER,
};

const char* localMapUpdateBlockReasonName(
    LocalMapUpdateBlockReason reason) noexcept;

struct LocalMapUpdateInput {
  bool registration_success = false;
  bool registration_cloud_valid = false;
  bool runtime_pose_finite = false;
  bool motion_state_allows_update = false;
  bool relocalization_pose_reliable = false;

  double translation_since_update_m = 0.0;
  double rotation_since_update_rad = 0.0;
  int frames_since_update = 0;

  double translation_trigger_m = 0.50;
  double rotation_trigger_rad = 0.08;
  int frame_trigger = 15;
};

struct LocalMapUpdateDecision {
  bool attempted = false;
  bool allowed = false;
  bool should_update = false;
  LocalMapUpdateBlockReason block_reason =
      LocalMapUpdateBlockReason::NO_REGISTRATION;
};

LocalMapUpdateDecision evaluateLocalMapUpdate(
    const LocalMapUpdateInput& input) noexcept;

}  // namespace ndt_slam
