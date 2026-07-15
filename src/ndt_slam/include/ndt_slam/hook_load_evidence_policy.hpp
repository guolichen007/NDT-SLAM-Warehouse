#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "ndt_slam/hook_load_state_filter.hpp"

namespace ndt_slam {

enum class HookLoadSignalRole : std::uint8_t {
    DISABLED = 0,
    REQUIRED = 1,
    AUXILIARY = 2
};

struct HookLoadRoleParseInput {
    bool enabled = true;
    bool role_present = false;
    std::string role;
    bool legacy_required = true;
};

struct HookLoadRoleParseResult {
    HookLoadSignalRole role = HookLoadSignalRole::REQUIRED;
    bool valid = true;
    bool legacy_mapping_used = false;
    std::string reason = "explicit_required";
};

HookLoadRoleParseResult parseHookLoadSignalRole(
    const HookLoadRoleParseInput& input);
const char* hookLoadSignalRoleName(HookLoadSignalRole role);

struct HookLoadEvidenceInput {
    HookLoadSignalRole role = HookLoadSignalRole::REQUIRED;
    bool gravity_valid = false;
    HookLoadState gravity_state = HookLoadState::UNKNOWN;
    bool lidar_cargo_valid = false;
    bool lidar_no_cargo_confirmed = false;
    bool lidar_track_locked = false;
    bool lidar_geometry_valid = false;
    bool lidar_height_valid = false;
};

struct HookLoadEvidenceDecision {
    bool gravity_required_fault = false;
    bool gravity_conflict = false;
    bool lidar_cargo_accepted = false;
    bool lidar_empty_accepted = false;
    std::string reason = "lidar_evidence_incomplete";
};

HookLoadEvidenceDecision evaluateHookLoadEvidence(
    const HookLoadEvidenceInput& input);

struct SuspendedCargoLockInput {
    HookLoadSignalRole role = HookLoadSignalRole::REQUIRED;
    bool gravity_valid = false;
    HookLoadState gravity_state = HookLoadState::UNKNOWN;
    int base_confirm_frames = 1;
    bool lidar_lift_evidence = false;
};

struct SuspendedCargoLockDecision {
    bool allow_candidate = false;
    bool allow_lock = false;
    bool gravity_conflict = false;
    int required_confirm_frames = 1;
    std::string reason = "required_gravity_unavailable";
};

SuspendedCargoLockDecision evaluateSuspendedCargoLock(
    const SuspendedCargoLockInput& input);

enum class CargoObservationOutcome : std::uint8_t {
    UNKNOWN = 0,
    CARGO_DETECTED = 1,
    EMPTY_CONFIRMED = 2
};

struct CargoObservationClassificationInput {
    bool detection_executed = false;
    bool ground_reference_valid = false;
    bool roi_coverage_valid = false;
    std::size_t hag_candidate_points = 0U;
    std::size_t maximum_empty_noise_points = 0U;
    bool cargo_detected = false;
};

CargoObservationOutcome classifyCargoObservationOutcome(
    const CargoObservationClassificationInput& input);

struct LidarNoCargoEvidenceConfig {
    std::uint32_t confirm_frames = 3U;
};

struct LidarNoCargoEvidenceInput {
    bool detection_executed = false;
    CargoObservationOutcome outcome = CargoObservationOutcome::UNKNOWN;
    bool localization_valid = false;
    bool cargo_lock_active = false;
    double source_time_sec = 0.0;
};

struct LidarNoCargoEvidenceResult {
    bool confirmed = false;
    std::uint32_t confirm_count = 0U;
    std::string reason = "startup_unconfirmed";
};

class LidarNoCargoEvidenceTracker {
public:
    explicit LidarNoCargoEvidenceTracker(
        const LidarNoCargoEvidenceConfig& config =
            LidarNoCargoEvidenceConfig());

    void setConfig(const LidarNoCargoEvidenceConfig& config);
    LidarNoCargoEvidenceResult update(
        const LidarNoCargoEvidenceInput& input);
    LidarNoCargoEvidenceResult result() const;
    void reset(const std::string& reason = "reset");

private:
    LidarNoCargoEvidenceConfig config_;
    bool confirmed_ = false;
    std::uint32_t confirm_count_ = 0U;
    bool has_seen_source_time_ = false;
    double last_seen_source_time_sec_ = 0.0;
    std::string reason_ = "startup_unconfirmed";
};

struct HookLoadMapCommitInput {
    HookLoadSignalRole role = HookLoadSignalRole::REQUIRED;
    bool gravity_valid = false;
    HookLoadState gravity_state = HookLoadState::UNKNOWN;
    bool lidar_removal_authorized = false;
    bool lidar_cargo_candidate = false;
};

struct HookLoadMapCommitDecision {
    bool allow_commit = false;
    bool use_formal_remove_box = false;
    bool exclude_candidate_region = false;
    bool required_fault = false;
    std::string reason = "required_signal_invalid";
};

HookLoadMapCommitDecision evaluateHookLoadMapCommit(
    const HookLoadMapCommitInput& input);

}  // namespace ndt_slam
