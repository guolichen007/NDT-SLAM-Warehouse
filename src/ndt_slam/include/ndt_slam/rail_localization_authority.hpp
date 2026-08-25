#pragma once

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class YawAuthorityMode : std::uint8_t {
  LEGACY = 0,
  SHADOW,
  RAIL_AUTHORITY,
};

enum class YawReferenceSource : std::uint8_t {
  NEW_MAP_BOOTSTRAP_REFERENCE = 0,
  CONFIG_SITE_REFERENCE,
  VERIFIED_MAP_SESSION,
};

enum class YawAuthorityTransitionReason : std::uint8_t {
  INITIALIZE_FRESH_MAP = 0,
  LOAD_VERIFIED_SESSION,
  EXPLICIT_MAP_FRAME_MIGRATION,
  RESET,
};

enum class LocalizationFailureClass : std::uint8_t {
  NONE = 0,
  RECOVERABLE_TRACKING_DEGRADATION,
  TEMPORARY_OBSERVABILITY_LOSS,
  TARGET_DATA_UNAVAILABLE,
  NONRECOVERABLE_REFERENCE_CONFIG,
  NONRECOVERABLE_MAP_IDENTITY,
  INTERNAL_CONTRACT_ERROR,
};

enum class RegistrationTargetSource : std::uint8_t {
  UNKNOWN = 0,
  GLOBAL_MAP,
  CROPPED_ACTIVE_MAP,
  LOCALIZATION_MAP,
  PERSISTENT_MAP,
};

const char* yawAuthorityModeName(YawAuthorityMode mode) noexcept;
const char* yawReferenceSourceName(YawReferenceSource source) noexcept;
const char* yawAuthorityTransitionReasonName(
    YawAuthorityTransitionReason reason) noexcept;
const char* localizationFailureClassName(
    LocalizationFailureClass failure) noexcept;
const char* registrationTargetSourceName(
    RegistrationTargetSource source) noexcept;

struct RailYawReference {
  std::uint32_t schema_version = 1U;
  bool verified = false;
  double rail_yaw_in_map_rad = 0.0;
  YawReferenceSource source =
      YawReferenceSource::NEW_MAP_BOOTSTRAP_REFERENCE;
  std::string map_frame_uuid;
  std::string map_frame_id = "map";
  std::string base_frame_id = "base_link";
  std::string map_frame_convention_id;
  std::string sensor_rig_calibration_id;
  std::string reference_uuid;
  std::string reference_hash;
};

// Hashes canonical semantic identity only. Host paths are not part of this
// contract and therefore cannot change a verified map-frame reference.
std::string semanticYawReferenceHash(const RailYawReference& reference);
bool validateRailYawReference(const RailYawReference& reference,
                              std::string* reason = nullptr);

class RailYawAuthority {
 public:
  bool initialize(const RailYawReference& reference,
                  YawAuthorityTransitionReason reason);
  bool explicitMapFrameMigration(const RailYawReference& reference);
  void resetForFrameSession();

  // Proposal writers are diagnostic-only by type and cannot mutate the
  // authority value or generation.
  void observeProposalYaw(double raw_yaw_rad, double stamp_sec) noexcept;
  void observeRelocalizationProposalYaw(
      double raw_yaw_rad, double stamp_sec) noexcept;
  void handleTimestampRollback(double stamp_sec) noexcept;

  bool valid() const noexcept { return valid_; }
  double yawRad() const noexcept { return reference_.rail_yaw_in_map_rad; }
  std::uint64_t generation() const noexcept { return generation_; }
  const RailYawReference& reference() const noexcept { return reference_; }
  double lastProposalYawRad() const noexcept { return last_proposal_yaw_rad_; }
  double lastProposalStampSec() const noexcept {
    return last_proposal_stamp_sec_;
  }
  YawAuthorityTransitionReason lastTransitionReason() const noexcept {
    return last_transition_reason_;
  }

 private:
  bool assignReference(const RailYawReference& reference,
                       YawAuthorityTransitionReason reason);

  bool valid_ = false;
  RailYawReference reference_;
  std::uint64_t generation_ = 0U;
  double last_proposal_yaw_rad_ = 0.0;
  double last_proposal_stamp_sec_ = 0.0;
  YawAuthorityTransitionReason last_transition_reason_ =
      YawAuthorityTransitionReason::RESET;
};

class YawAuthorityModeLatch {
 public:
  // A different mode is accepted only with a new explicit frame-session id.
  bool initialize(YawAuthorityMode mode, std::uint64_t frame_session_id);
  bool initialized() const noexcept { return initialized_; }
  YawAuthorityMode mode() const noexcept { return mode_; }
  std::uint64_t frameSessionId() const noexcept { return frame_session_id_; }

 private:
  bool initialized_ = false;
  YawAuthorityMode mode_ = YawAuthorityMode::LEGACY;
  std::uint64_t frame_session_id_ = 0U;
};

struct RegistrationTargetIdentityInput {
  RegistrationTargetSource source = RegistrationTargetSource::UNKNOWN;
  std::uint64_t content_version = 0U;
  std::uint64_t map_rebuild_generation = 0U;
  std::string map_frame_uuid;
  std::string crop_identity;
};

std::uint64_t makeRegistrationTargetSnapshotId(
    const RegistrationTargetIdentityInput& input) noexcept;

struct RailLocalizationHealthInput {
  YawAuthorityMode mode = YawAuthorityMode::LEGACY;
  bool raw_ndt_proposal_healthy = false;
  bool yaw_reference_valid = false;
  bool target_identity_valid = false;
  bool fixed_xy_valid = false;
  bool ekf_measurement_accepted = false;
  bool rail_fitness_allow_measurement = false;
  bool rail_fitness_baseline_ready = false;
  bool reference_failure_nonrecoverable = false;
  bool map_identity_failure_nonrecoverable = false;
  bool internal_contract_error = false;
  bool prediction_continuity_valid = false;
};

struct LocalizationAuthorityHealth {
  bool authoritative_frame_healthy = false;
  bool odom_continuity_valid = false;
  bool safety_localization_authorized = false;
  bool map_mutation_authorized = false;
  bool increment_relocalization_bad_frames = false;
  bool request_relocalization = false;
  bool watchdog_restart_authorized = false;
  LocalizationFailureClass failure_class = LocalizationFailureClass::NONE;
  std::string reason = "not_evaluated";
};

LocalizationAuthorityHealth evaluateRailLocalizationHealth(
    const RailLocalizationHealthInput& input);

}  // namespace ndt_slam
