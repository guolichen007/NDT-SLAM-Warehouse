#include <gtest/gtest.h>

#include <utility>

#include "ndt_slam/hook_load_evidence_policy.hpp"

namespace ndt_slam {
namespace {

HookLoadEvidenceInput lockedCargo(HookLoadSignalRole role,
                                  bool gravity_valid,
                                  HookLoadState gravity_state) {
    HookLoadEvidenceInput input;
    input.role = role;
    input.gravity_valid = gravity_valid;
    input.gravity_state = gravity_state;
    input.lidar_cargo_valid = true;
    input.lidar_track_locked = true;
    input.lidar_geometry_valid = true;
    input.lidar_height_valid = true;
    return input;
}

TEST(HookLoadRole, ParsesExplicitLegacyAndDisabledForms) {
    for (const auto& item : {
             std::pair<const char*, HookLoadSignalRole>{"required", HookLoadSignalRole::REQUIRED},
             {"auxiliary", HookLoadSignalRole::AUXILIARY},
             {"disabled", HookLoadSignalRole::DISABLED}}) {
        const auto result = parseHookLoadSignalRole({true, true, item.first, true});
        EXPECT_TRUE(result.valid);
        EXPECT_EQ(result.role, item.second);
        EXPECT_FALSE(result.legacy_mapping_used);
    }
    EXPECT_EQ(parseHookLoadSignalRole({true, false, "", true}).role,
              HookLoadSignalRole::REQUIRED);
    EXPECT_EQ(parseHookLoadSignalRole({true, false, "", false}).role,
              HookLoadSignalRole::AUXILIARY);
    EXPECT_EQ(parseHookLoadSignalRole({false, true, "required", true}).role,
              HookLoadSignalRole::DISABLED);
    const auto invalid = parseHookLoadSignalRole({true, true, "unsafe", false});
    EXPECT_FALSE(invalid.valid);
    EXPECT_EQ(invalid.role, HookLoadSignalRole::REQUIRED);
}

TEST(HookLoadEvidence, AuxiliaryGravityNeverCreatesOrClearsCargo) {
    const auto loaded = evaluateHookLoadEvidence(
        lockedCargo(HookLoadSignalRole::AUXILIARY, true, HookLoadState::LOADED));
    EXPECT_TRUE(loaded.lidar_cargo_accepted);
    EXPECT_FALSE(loaded.gravity_conflict);

    const auto empty_conflict = evaluateHookLoadEvidence(
        lockedCargo(HookLoadSignalRole::AUXILIARY, true, HookLoadState::EMPTY));
    EXPECT_TRUE(empty_conflict.lidar_cargo_accepted);
    EXPECT_TRUE(empty_conflict.gravity_conflict);

    const auto stale = evaluateHookLoadEvidence(
        lockedCargo(HookLoadSignalRole::AUXILIARY, false, HookLoadState::UNKNOWN));
    EXPECT_TRUE(stale.lidar_cargo_accepted);
    EXPECT_FALSE(stale.gravity_required_fault);

    HookLoadEvidenceInput gravity_only;
    gravity_only.role = HookLoadSignalRole::AUXILIARY;
    gravity_only.gravity_valid = true;
    gravity_only.gravity_state = HookLoadState::LOADED;
    const auto no_lidar = evaluateHookLoadEvidence(gravity_only);
    EXPECT_FALSE(no_lidar.lidar_cargo_accepted);
    EXPECT_FALSE(no_lidar.lidar_empty_accepted);
}

TEST(HookLoadEvidence, LidarEmptyRemainsPrimaryAndConflictsAreDiagnostic) {
    HookLoadEvidenceInput input;
    input.role = HookLoadSignalRole::AUXILIARY;
    input.lidar_no_cargo_confirmed = true;
    input.gravity_valid = true;
    input.gravity_state = HookLoadState::LOADED;
    const auto conflict = evaluateHookLoadEvidence(input);
    EXPECT_TRUE(conflict.lidar_empty_accepted);
    EXPECT_TRUE(conflict.gravity_conflict);

    input.gravity_valid = false;
    input.gravity_state = HookLoadState::UNKNOWN;
    const auto stale = evaluateHookLoadEvidence(input);
    EXPECT_TRUE(stale.lidar_empty_accepted);
    EXPECT_FALSE(stale.gravity_required_fault);
}

TEST(HookLoadEvidence, RequiredAndDisabledCompatibilityIsExplicit) {
    const auto required_invalid = evaluateHookLoadEvidence(
        lockedCargo(HookLoadSignalRole::REQUIRED, false, HookLoadState::UNKNOWN));
    EXPECT_TRUE(required_invalid.gravity_required_fault);
    EXPECT_FALSE(required_invalid.lidar_cargo_accepted);

    const auto required_loaded = evaluateHookLoadEvidence(
        lockedCargo(HookLoadSignalRole::REQUIRED, true, HookLoadState::LOADED));
    EXPECT_TRUE(required_loaded.lidar_cargo_accepted);

    const auto disabled = evaluateHookLoadEvidence(
        lockedCargo(HookLoadSignalRole::DISABLED, false, HookLoadState::UNKNOWN));
    EXPECT_TRUE(disabled.lidar_cargo_accepted);
    EXPECT_FALSE(disabled.gravity_conflict);
}

TEST(HookLoadMapCommit, UsesLidarAuthorizationAndConservativeCandidateExclusion) {
    const auto formal = evaluateHookLoadMapCommit(
        {HookLoadSignalRole::AUXILIARY, false, HookLoadState::UNKNOWN, true, true});
    EXPECT_TRUE(formal.allow_commit);
    EXPECT_TRUE(formal.use_formal_remove_box);
    EXPECT_FALSE(formal.exclude_candidate_region);

    const auto empty_conflict = evaluateHookLoadMapCommit(
        {HookLoadSignalRole::AUXILIARY, true, HookLoadState::EMPTY, true, true});
    EXPECT_TRUE(empty_conflict.allow_commit);
    EXPECT_TRUE(empty_conflict.use_formal_remove_box);

    const auto candidate = evaluateHookLoadMapCommit(
        {HookLoadSignalRole::AUXILIARY, false, HookLoadState::UNKNOWN, false, true});
    EXPECT_TRUE(candidate.allow_commit);
    EXPECT_FALSE(candidate.use_formal_remove_box);
    EXPECT_TRUE(candidate.exclude_candidate_region);

    const auto required_invalid = evaluateHookLoadMapCommit(
        {HookLoadSignalRole::REQUIRED, false, HookLoadState::UNKNOWN, true, true});
    EXPECT_FALSE(required_invalid.allow_commit);
    EXPECT_TRUE(required_invalid.required_fault);

    const auto gravity_cannot_authorize = evaluateHookLoadMapCommit(
        {HookLoadSignalRole::AUXILIARY, true, HookLoadState::LOADED, false, true});
    EXPECT_FALSE(gravity_cannot_authorize.use_formal_remove_box);
    EXPECT_TRUE(gravity_cannot_authorize.exclude_candidate_region);
}

}  // namespace
}  // namespace ndt_slam
