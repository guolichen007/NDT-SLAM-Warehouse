#include "ndt_slam/hook_load_evidence_policy.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

namespace ndt_slam {

const char* hookLoadSignalRoleName(HookLoadSignalRole role) {
    switch (role) {
        case HookLoadSignalRole::DISABLED: return "disabled";
        case HookLoadSignalRole::REQUIRED: return "required";
        case HookLoadSignalRole::AUXILIARY: return "auxiliary";
    }
    return "invalid";
}

HookLoadRoleParseResult parseHookLoadSignalRole(
    const HookLoadRoleParseInput& input) {
    HookLoadRoleParseResult result;
    if (!input.enabled) {
        result.role = HookLoadSignalRole::DISABLED;
        result.reason = "enabled_false_override";
        return result;
    }
    if (!input.role_present) {
        result.role = input.legacy_required
            ? HookLoadSignalRole::REQUIRED
            : HookLoadSignalRole::AUXILIARY;
        result.legacy_mapping_used = true;
        result.reason = input.legacy_required
            ? "legacy_required_true" : "legacy_required_false";
        return result;
    }

    std::string role = input.role;
    std::transform(role.begin(), role.end(), role.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    if (role == "required") {
        result.role = HookLoadSignalRole::REQUIRED;
        result.reason = "explicit_required";
    } else if (role == "auxiliary") {
        result.role = HookLoadSignalRole::AUXILIARY;
        result.reason = "explicit_auxiliary";
    } else if (role == "disabled") {
        result.role = HookLoadSignalRole::DISABLED;
        result.reason = "explicit_disabled";
    } else {
        result.role = HookLoadSignalRole::REQUIRED;
        result.valid = false;
        result.reason = "invalid_role_fail_safe_required";
    }
    return result;
}

HookLoadEvidenceDecision evaluateHookLoadEvidence(
    const HookLoadEvidenceInput& input) {
    HookLoadEvidenceDecision decision;
    const bool gravity_loaded = input.gravity_valid &&
        input.gravity_state == HookLoadState::LOADED;
    const bool gravity_empty = input.gravity_valid &&
        input.gravity_state == HookLoadState::EMPTY;
    const bool reliable_cargo = input.lidar_cargo_valid &&
        input.lidar_track_locked && input.lidar_geometry_valid &&
        input.lidar_height_valid;
    const bool reliable_empty = input.lidar_no_cargo_confirmed &&
        !input.lidar_cargo_valid;

    if (input.role == HookLoadSignalRole::REQUIRED) {
        decision.gravity_required_fault =
            !gravity_loaded && !gravity_empty;
        decision.lidar_cargo_accepted = reliable_cargo && gravity_loaded;
        decision.lidar_empty_accepted = reliable_empty && gravity_empty;
    } else {
        decision.lidar_cargo_accepted = reliable_cargo;
        decision.lidar_empty_accepted = reliable_empty;
    }

    if (input.role != HookLoadSignalRole::DISABLED && input.gravity_valid) {
        decision.gravity_conflict =
            (gravity_empty && reliable_cargo) ||
            (gravity_loaded && reliable_empty);
    }

    if (decision.gravity_required_fault) {
        decision.reason = "required_gravity_unavailable";
    } else if (decision.gravity_conflict) {
        decision.reason = "gravity_lidar_conflict";
    } else if (decision.lidar_cargo_accepted) {
        decision.reason = gravity_loaded
            ? "lidar_cargo_gravity_support" : "lidar_cargo_primary";
    } else if (decision.lidar_empty_accepted) {
        decision.reason = gravity_empty
            ? "lidar_empty_gravity_support" : "lidar_empty_primary";
    }
    return decision;
}

SuspendedCargoLockDecision evaluateSuspendedCargoLock(
    const SuspendedCargoLockInput& input) {
    SuspendedCargoLockDecision decision;
    const int base_frames = std::max(1, input.base_confirm_frames);
    decision.required_confirm_frames = base_frames;
    const bool gravity_loaded = input.gravity_valid &&
        input.gravity_state == HookLoadState::LOADED;
    const bool gravity_empty = input.gravity_valid &&
        input.gravity_state == HookLoadState::EMPTY;

    if (input.role == HookLoadSignalRole::REQUIRED) {
        decision.allow_candidate = gravity_loaded;
        decision.allow_lock = gravity_loaded;
        decision.reason = gravity_loaded
            ? "required_gravity_loaded" : "required_gravity_unavailable";
        return decision;
    }

    decision.allow_candidate = true;
    if (input.role == HookLoadSignalRole::DISABLED) {
        decision.allow_lock = true;
        decision.reason = "lidar_only";
        return decision;
    }
    if (gravity_loaded) {
        decision.allow_lock = true;
        decision.reason = "auxiliary_gravity_support";
        return decision;
    }
    if (gravity_empty) {
        decision.gravity_conflict = true;
        decision.allow_lock = input.lidar_lift_evidence;
        decision.required_confirm_frames = base_frames + 2;
        decision.reason = "auxiliary_empty_delayed_confirmation";
        return decision;
    }
    decision.allow_lock = true;
    decision.required_confirm_frames = base_frames + 1;
    decision.reason = "auxiliary_gravity_unavailable_strict_lidar";
    return decision;
}

LockedCargoHeightAction evaluateLockedCargoHeightAction(
    bool freeze_geometry_after_lock,
    bool has_good_height,
    bool observation_valid) {
    if (!observation_valid) {
        return LockedCargoHeightAction::IGNORE_INVALID;
    }
    if (!freeze_geometry_after_lock) {
        return LockedCargoHeightAction::UPDATE_ADAPTIVE;
    }
    return has_good_height
        ? LockedCargoHeightAction::REFRESH_FROZEN
        : LockedCargoHeightAction::INITIALIZE_ONCE;
}

CargoObservationOutcome classifyCargoObservationOutcome(
    const CargoObservationClassificationInput& input) {
    if (!input.detection_executed) {
        return CargoObservationOutcome::UNKNOWN;
    }
    if (input.cargo_detected) {
        return CargoObservationOutcome::CARGO_DETECTED;
    }
    if (input.ground_reference_valid && input.roi_coverage_valid &&
        input.hag_candidate_points <= input.maximum_empty_noise_points) {
        return CargoObservationOutcome::EMPTY_CONFIRMED;
    }
    return CargoObservationOutcome::UNKNOWN;
}

LidarNoCargoEvidenceTracker::LidarNoCargoEvidenceTracker(
    const LidarNoCargoEvidenceConfig& config) : config_(config) {
    config_.confirm_frames = std::max<std::uint32_t>(1U, config_.confirm_frames);
}

void LidarNoCargoEvidenceTracker::setConfig(
    const LidarNoCargoEvidenceConfig& config) {
    config_ = config;
    config_.confirm_frames = std::max<std::uint32_t>(1U, config_.confirm_frames);
    reset("config_changed");
}

LidarNoCargoEvidenceResult LidarNoCargoEvidenceTracker::update(
    const LidarNoCargoEvidenceInput& input) {
    if (!input.detection_executed) {
        if (!confirmed_) reason_ = "detection_not_executed";
        return result();
    }
    if (!std::isfinite(input.source_time_sec) || input.source_time_sec < 0.0) {
        confirmed_ = false;
        confirm_count_ = 0U;
        reason_ = "invalid_source_time";
        return result();
    }
    if (has_seen_source_time_ &&
        input.source_time_sec + 1.0e-6 < last_seen_source_time_sec_) {
        confirmed_ = false;
        confirm_count_ = 0U;
        last_seen_source_time_sec_ = input.source_time_sec;
        reason_ = "source_time_rollback";
        return result();
    }
    if (has_seen_source_time_ &&
        std::abs(input.source_time_sec - last_seen_source_time_sec_) <= 1.0e-6) {
        reason_ = "duplicate_detection_ignored";
        return result();
    }
    has_seen_source_time_ = true;
    last_seen_source_time_sec_ = input.source_time_sec;

    if (!input.localization_valid) {
        confirmed_ = false;
        confirm_count_ = 0U;
        reason_ = "localization_invalid";
    } else if (input.outcome == CargoObservationOutcome::UNKNOWN) {
        confirmed_ = false;
        confirm_count_ = 0U;
        reason_ = "observation_unknown";
    } else if (input.outcome == CargoObservationOutcome::CARGO_DETECTED ||
               input.cargo_lock_active) {
        confirmed_ = false;
        confirm_count_ = 0U;
        reason_ = input.outcome == CargoObservationOutcome::CARGO_DETECTED
            ? "cargo_detected" : "cargo_lock_active";
    } else {
        confirm_count_ = std::min<std::uint32_t>(
            confirm_count_ + 1U, std::numeric_limits<std::uint32_t>::max());
        confirmed_ = confirm_count_ >= config_.confirm_frames;
        reason_ = confirmed_ ? "no_cargo_confirmed" : "no_cargo_pending";
    }
    return result();
}

LidarNoCargoEvidenceResult LidarNoCargoEvidenceTracker::result() const {
    return {confirmed_, confirm_count_, reason_};
}

void LidarNoCargoEvidenceTracker::reset(const std::string& reason) {
    confirmed_ = false;
    confirm_count_ = 0U;
    has_seen_source_time_ = false;
    last_seen_source_time_sec_ = 0.0;
    reason_ = reason;
}

HookLoadMapCommitDecision evaluateHookLoadMapCommit(
    const HookLoadMapCommitInput& input) {
    HookLoadMapCommitDecision decision;
    const bool gravity_loaded = input.gravity_valid &&
        input.gravity_state == HookLoadState::LOADED;
    const bool gravity_empty = input.gravity_valid &&
        input.gravity_state == HookLoadState::EMPTY;

    if (input.role == HookLoadSignalRole::REQUIRED) {
        if (!gravity_loaded && !gravity_empty) {
            decision.required_fault = true;
            return decision;
        }
        if (gravity_loaded && !input.lidar_removal_authorized) {
            decision.reason = "required_loaded_without_lidar_authorization";
            return decision;
        }
    }

    decision.allow_commit = true;
    decision.use_formal_remove_box = input.lidar_removal_authorized;
    decision.exclude_candidate_region =
        input.lidar_cargo_candidate && !input.lidar_removal_authorized;
    if (decision.use_formal_remove_box) {
        decision.reason = "lidar_formal_remove_box";
    } else if (decision.exclude_candidate_region) {
        decision.reason = "conservative_candidate_exclusion";
    } else {
        decision.reason = "static_regions_allowed";
    }
    return decision;
}

}  // namespace ndt_slam
