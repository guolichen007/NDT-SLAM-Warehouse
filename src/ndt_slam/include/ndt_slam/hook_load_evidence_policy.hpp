#pragma once

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
