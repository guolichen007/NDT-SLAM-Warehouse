#pragma once

#include <cstdint>

namespace ndt_slam {

enum class LocalMapPoseAuthority : std::uint8_t {
  NDT_MEASURED = 0,
  EKF_PREDICTED,
  RECOVERY,
  INVALID,
};

const char* localMapPoseAuthorityName(
    LocalMapPoseAuthority authority) noexcept;

enum class LocalMapUpdateMode : std::uint8_t {
  NONE = 0,
  NORMAL_UPDATE,
  MOTION_ESCAPE_REFRESH,
};

const char* localMapUpdateModeName(LocalMapUpdateMode mode) noexcept;

enum class LocalMapHealthState : std::uint8_t {
  HEALTHY_UPDATING = 0,
  IDLE_STATIONARY,
  WAITING_TRIGGER,
  QUARANTINED,
  STARVED_MOVING,
};

const char* localMapHealthStateName(LocalMapHealthState state) noexcept;

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
  bool normal_motion_update_allowed = false;
  bool motion_escape_refresh_allowed = false;
  bool relocalization_pose_reliable = false;

  double translation_since_update_m = 0.0;
  double rotation_since_update_rad = 0.0;
  int frames_since_update = 0;

  double translation_trigger_m = 0.50;
  double rotation_trigger_rad = 0.08;
  int frame_trigger = 15;
};

struct LocalMapUpdateDecision {
  bool eligible = false;
  bool due = false;
  LocalMapUpdateMode mode = LocalMapUpdateMode::NONE;
  LocalMapUpdateBlockReason block_reason =
      LocalMapUpdateBlockReason::NO_REGISTRATION;
};

LocalMapUpdateDecision evaluateLocalMapUpdate(
    const LocalMapUpdateInput& input) noexcept;

struct LocalMapPoseSample {
  LocalMapPoseAuthority authority = LocalMapPoseAuthority::INVALID;
  double stamp_sec = 0.0;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
};

struct LocalMapPoseAuthorityMetrics {
  LocalMapPoseAuthority authority = LocalMapPoseAuthority::INVALID;
  std::uint64_t consecutive_prediction_only_frames = 0;
  double prediction_only_duration_sec = 0.0;
  std::uint64_t frames_since_last_trusted_ndt = 0;
  double time_since_last_trusted_ndt_sec = 0.0;
  double distance_since_last_trusted_ndt_m = 0.0;
  double yaw_since_last_trusted_ndt_rad = 0.0;
  std::uint64_t local_map_updates_from_measured_pose = 0;
  std::uint64_t local_map_updates_from_predicted_pose = 0;
  bool trusted_ndt_reference_valid = false;
};

// Frame-local observability only. It never authorizes a map write and carries
// no production cutoff for prediction-only duration or distance.
class LocalMapPoseAuthorityTracker {
 public:
  void reset() noexcept;
  LocalMapPoseAuthorityMetrics observe(
      const LocalMapPoseSample& sample) noexcept;
  LocalMapPoseAuthorityMetrics recordLocalMapUpdate(
      LocalMapPoseAuthority authority) noexcept;
  const LocalMapPoseAuthorityMetrics& metrics() const noexcept {
    return metrics_;
  }

 private:
  LocalMapPoseAuthorityMetrics metrics_;
  double prediction_only_started_sec_ = 0.0;
  double trusted_stamp_sec_ = 0.0;
  double trusted_x_m_ = 0.0;
  double trusted_y_m_ = 0.0;
  double trusted_yaw_rad_ = 0.0;
};

struct LocalMapHealthInput {
  bool relocalization_pose_reliable = false;
  bool stationary_idle = false;
  bool motion_update_expected = false;
  bool eligible = false;
  bool due = false;
  bool committed = false;
  double expected_update_age_sec = 0.0;
  double starvation_warning_sec = 5.0;
};

LocalMapHealthState classifyLocalMapHealth(
    const LocalMapHealthInput& input) noexcept;

}  // namespace ndt_slam
