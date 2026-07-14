#include "ndt_slam/hook_load_evidence_policy.hpp"

#include <algorithm>
#include <cctype>

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
