#include <gtest/gtest.h>

#define CARGO_ALARM_HEARTBEAT_STATE_MACHINE_ONLY
#include "../src/cargo_alarm_heartbeat_node.cpp"

namespace {

using cargo_alarm::AlarmStateMachine;
using cargo_alarm::StatusContractInput;
using cargo_alarm::validateStatusContract;

StatusContractInput loadedWarning(int code) {
    StatusContractInput input;
    input.schema_valid = true;
    input.valid = true;
    input.warning_valid = true;
    input.requested_code = code;
    input.warning_code = code;
    input.localization_valid = true;
    input.hook_signal_valid = true;
    input.hook_load_state = AlarmStateMachine::kHookLoaded;
    input.cargo_valid = true;
    input.obstacle_valid = true;
    if (code == AlarmStateMachine::kLevel1Warning ||
        code == AlarmStateMachine::kLevel2Warning) {
        input.obstacle_count = 1U;
        input.nearest_obstacle_distance_m =
            code == AlarmStateMachine::kLevel1Warning ? 2.0 : 4.0;
        input.obstacle_top_z_map = 1.0;
        input.obstacle_uncertainty_m = 0.05;
        input.conservative_vertical_clearance_m = 0.70;
    }
    return input;
}

StatusContractInput loadedClusterClear(double distance, double clearance) {
    StatusContractInput input = loadedWarning(AlarmStateMachine::kClear);
    input.obstacle_count = 1U;
    input.nearest_obstacle_distance_m = distance;
    input.obstacle_top_z_map = 1.0;
    input.obstacle_uncertainty_m = 0.05;
    input.conservative_vertical_clearance_m = clearance;
    return input;
}

StatusContractInput pendingWarning(int code) {
    StatusContractInput input = loadedWarning(code);
    input.cargo_valid = false;
    input.evidence_state =
        AlarmStateMachine::kEvidenceHazardCandidate;
    return input;
}

StatusContractInput faultStatus(int code, std::uint32_t mask) {
    StatusContractInput input;
    input.schema_valid = true;
    input.requested_code = code;
    input.fault_code = code;
    input.fault_mask = mask;
    return input;
}

TEST(CargoAlarmHeartbeat, StartupAndStaleUseSystemNotReady) {
    AlarmStateMachine state(0.5);
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady, state.currentCode());
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady, state.tick(1.0).code);

    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(AlarmStateMachine::kClear, 2.0, 20.0).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(2.1).code);
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady, state.tick(2.6).code);
}

TEST(CargoAlarmHeartbeat, PendingEnvelopeCanOnlyPublishPositiveWarning) {
    const auto warning = validateStatusContract(
        pendingWarning(AlarmStateMachine::kLevel1Warning));
    EXPECT_TRUE(warning.valid);
    EXPECT_EQ(warning.code, AlarmStateMachine::kLevel1Warning);

    auto clear = pendingWarning(AlarmStateMachine::kClear);
    clear.obstacle_count = 0U;
    const auto rejected = validateStatusContract(clear);
    EXPECT_FALSE(rejected.valid);
    EXPECT_EQ(rejected.code, AlarmStateMachine::kInternalError);
}

TEST(CargoAlarmHeartbeat, SourceRollbackStartsRecoverableNewEpoch) {
    AlarmStateMachine state(0.8);
    state.ingest(AlarmStateMachine::kCargoInvalid, 10.0, 100.0);
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady,
              state.ingest(AlarmStateMachine::kClear,
                           10.1, 1.0,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(), true).code);
    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(AlarmStateMachine::kClear,
                           10.2, 1.1,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(), true).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(10.2).code);
}

TEST(CargoAlarmHeartbeat, SecondBagPlaybackCanRecoverWithoutNodeRestart) {
    AlarmStateMachine state(2.0);
    state.ingest(AlarmStateMachine::kCargoInvalid, 1.0, 50.0);
    state.ingest(AlarmStateMachine::kCargoInvalid, 1.1, 50.1);

    EXPECT_EQ(AlarmStateMachine::kSystemNotReady,
              state.ingest(AlarmStateMachine::kClear, 2.0, 5.0,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(), true).code);
    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(AlarmStateMachine::kClear, 2.1, 5.1).code);
}

TEST(CargoAlarmHeartbeat, DuplicateSourceStampCannotChangeFormalCode) {
    AlarmStateMachine stalled(0.5);
    stalled.ingest(AlarmStateMachine::kCargoInvalid, 20.0, 200.0);
    EXPECT_EQ(AlarmStateMachine::kCargoInvalid,
              stalled.ingest(AlarmStateMachine::kClear, 20.1, 200.0).code);
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady, stalled.tick(20.5).code);
}

TEST(CargoAlarmHeartbeat, InvalidTimestampAndUnknownCodeUseInternalError) {
    AlarmStateMachine state(1.0);
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              state.ingest(AlarmStateMachine::kClear, 1.0, 0.0).code);
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              state.ingest(99, 1.1, 1.1).code);
}

TEST(CargoAlarmHeartbeat, EveryFreshFormalTransitionIsImmediate) {
    AlarmStateMachine state(2.0);
    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(AlarmStateMachine::kClear,
                           1.0, 10.0, 6.0, 1.0, true).code);
    EXPECT_EQ(AlarmStateMachine::kLevel2Warning,
              state.ingest(AlarmStateMachine::kLevel2Warning,
                           1.1, 10.1, 4.0, 0.70).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kLevel1Warning,
                           1.2, 10.2, 2.0, 0.70).code);
    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(AlarmStateMachine::kClear,
                           1.3, 10.3, 3.01, 0.80).code);
}

TEST(CargoAlarmHeartbeat, FaultCodesPassThroughImmediately) {
    AlarmStateMachine state(2.0);
    for (int code = AlarmStateMachine::kSystemNotReady;
         code <= AlarmStateMachine::kInternalError; ++code) {
        EXPECT_EQ(code, state.ingest(code, 1.0 + code, 10.0 + code).code);
    }
}

TEST(CargoAlarmHeartbeat, ContractAcceptsOnlyStructuredWarningsAndFaults) {
    for (int code : {AlarmStateMachine::kClear,
                     AlarmStateMachine::kLevel1Warning,
                     AlarmStateMachine::kLevel2Warning}) {
        EXPECT_EQ(code, validateStatusContract(loadedWarning(code)).code);
    }
    for (int code = AlarmStateMachine::kSystemNotReady;
         code <= AlarmStateMachine::kInternalError; ++code) {
        EXPECT_EQ(code,
                  validateStatusContract(faultStatus(
                      code, 1U << static_cast<unsigned>(
                          code - AlarmStateMachine::kSystemNotReady))).code);
    }
}

TEST(CargoAlarmHeartbeat, ContractRejectsSchemaUnknownAndInvalidHazardEvidence) {
    StatusContractInput schema = loadedWarning(AlarmStateMachine::kClear);
    schema.schema_valid = false;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(schema).code);

    StatusContractInput unknown = loadedWarning(AlarmStateMachine::kClear);
    unknown.requested_code = 99;
    unknown.warning_code = 99;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(unknown).code);

    StatusContractInput invalid17 = loadedWarning(
        AlarmStateMachine::kLevel1Warning);
    invalid17.localization_valid = false;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(invalid17).code);

    StatusContractInput invalid18 = loadedWarning(
        AlarmStateMachine::kLevel2Warning);
    invalid18.obstacle_valid = false;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(invalid18).code);
}

TEST(CargoAlarmHeartbeat, AuxiliaryRoleUsesLidarGeometryWhenGravityIsInvalid) {
    StatusContractInput hazard = loadedWarning(
        AlarmStateMachine::kLevel1Warning);
    hazard.hook_signal_role = AlarmStateMachine::kHookRoleAuxiliary;
    hazard.hook_signal_valid = false;
    hazard.hook_load_state = 0;
    hazard.hook_signal_conflict = true;
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              validateStatusContract(hazard).code);

    StatusContractInput empty;
    empty.schema_valid = true;
    empty.valid = true;
    empty.warning_valid = true;
    empty.requested_code = AlarmStateMachine::kClear;
    empty.warning_code = AlarmStateMachine::kClear;
    empty.localization_valid = true;
    empty.hook_signal_role = AlarmStateMachine::kHookRoleAuxiliary;
    empty.hook_signal_valid = false;
    empty.no_cargo_confirmed = true;
    EXPECT_EQ(AlarmStateMachine::kClear,
              validateStatusContract(empty).code);

    hazard.obstacle_valid = false;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(hazard).code);
}

TEST(CargoAlarmHeartbeat, Code32IsReservedForRequiredGravity) {
    StatusContractInput gravity = faultStatus(
        AlarmStateMachine::kGravityInvalid, 4U);
    gravity.hook_signal_role = AlarmStateMachine::kHookRoleRequired;
    EXPECT_EQ(AlarmStateMachine::kGravityInvalid,
              validateStatusContract(gravity).code);

    gravity.hook_signal_role = AlarmStateMachine::kHookRoleAuxiliary;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(gravity).code);
    gravity.hook_signal_role = AlarmStateMachine::kHookRoleDisabled;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(gravity).code);
}

TEST(CargoAlarmHeartbeat, EmptyHookClearIsExplicitlyAllowed) {
    StatusContractInput empty = loadedWarning(AlarmStateMachine::kClear);
    empty.hook_load_state = AlarmStateMachine::kHookEmpty;
    empty.no_cargo_confirmed = true;
    empty.cargo_valid = false;
    empty.obstacle_valid = false;
    const auto result = validateStatusContract(empty);
    EXPECT_EQ(AlarmStateMachine::kClear, result.code);
    EXPECT_TRUE(result.clear_without_obstacle_geometry);
}

TEST(CargoAlarmHeartbeat, LoadedClearAllowsValidObservationWithNoObstacle) {
    StatusContractInput clear = loadedWarning(AlarmStateMachine::kClear);
    clear.obstacle_count = 0U;
    const auto result = validateStatusContract(clear);
    EXPECT_TRUE(result.valid);
    EXPECT_EQ(AlarmStateMachine::kClear, result.code);
    EXPECT_TRUE(result.clear_without_obstacle_geometry);
}

TEST(CargoAlarmHeartbeat, FreshClearImmediatelyReleasesWarning) {
    AlarmStateMachine state(0.8);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.79);
    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(AlarmStateMachine::kClear,
                           1.1, 10.1, 3.01, 0.80).code);
}

TEST(CargoAlarmHeartbeat, FreshClearImmediatelyRecoversFault) {
    AlarmStateMachine state(0.8);
    state.ingest(AlarmStateMachine::kObstacleInvalid, 1.0, 10.0);
    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(AlarmStateMachine::kClear,
                           1.1, 10.1,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(), true).code);
}

TEST(CargoAlarmHeartbeat, DuplicateStampCannotReleaseWarning) {
    AlarmStateMachine state(0.8);
    state.ingest(AlarmStateMachine::kLevel2Warning,
                 1.0, 10.0, 4.0, 0.79);
    EXPECT_EQ(AlarmStateMachine::kLevel2Warning,
              state.ingest(AlarmStateMachine::kClear,
                           1.1, 10.0, 5.01, 0.80).code);
    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(AlarmStateMachine::kClear,
                           1.2, 10.1, 5.01, 0.80).code);
}

TEST(CargoAlarmHeartbeat, HeartbeatOnlyRepeatsAndNeverTransitions) {
    AlarmStateMachine state(0.8);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.79);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.2).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.7).code);
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady, state.tick(1.801).code);
}

TEST(CargoAlarmHeartbeat, Level1GeometryContractUsesEntryThresholds) {
    StatusContractInput valid = loadedWarning(
        AlarmStateMachine::kLevel1Warning);
    valid.nearest_obstacle_distance_m = 3.0;
    valid.conservative_vertical_clearance_m = 0.79;
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              validateStatusContract(valid).code);

    StatusContractInput too_far = valid;
    too_far.nearest_obstacle_distance_m = 3.5;
    const auto distance_result = validateStatusContract(too_far);
    EXPECT_EQ(AlarmStateMachine::kInternalError, distance_result.code);
    EXPECT_STREQ("warning_geometry_mismatch", distance_result.reason);

    StatusContractInput enough_clearance = valid;
    enough_clearance.nearest_obstacle_distance_m = 2.0;
    enough_clearance.conservative_vertical_clearance_m = 0.80;
    const auto clearance_result = validateStatusContract(enough_clearance);
    EXPECT_EQ(AlarmStateMachine::kInternalError, clearance_result.code);
    EXPECT_STREQ("warning_geometry_mismatch", clearance_result.reason);
}

TEST(CargoAlarmHeartbeat, Level2GeometryContractUsesEntryThresholds) {
    StatusContractInput valid = loadedWarning(
        AlarmStateMachine::kLevel2Warning);
    valid.nearest_obstacle_distance_m = 4.0;
    valid.conservative_vertical_clearance_m = 0.70;
    EXPECT_EQ(AlarmStateMachine::kLevel2Warning,
              validateStatusContract(valid).code);

    StatusContractInput too_near = valid;
    too_near.nearest_obstacle_distance_m = 2.0;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(too_near).code);

    StatusContractInput too_far = valid;
    too_far.nearest_obstacle_distance_m = 5.1;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(too_far).code);
}

TEST(CargoAlarmHeartbeat, HazardsRequireAtLeastOneObstacle) {
    for (int code : {AlarmStateMachine::kLevel1Warning,
                     AlarmStateMachine::kLevel2Warning}) {
        StatusContractInput input = loadedWarning(code);
        input.obstacle_count = 0U;
        const auto result = validateStatusContract(input);
        EXPECT_EQ(AlarmStateMachine::kInternalError, result.code);
        EXPECT_STREQ("warning_geometry_mismatch", result.reason);
    }
}

TEST(CargoAlarmHeartbeat, HazardsRejectEveryNonFiniteGeometryField) {
    for (int code : {AlarmStateMachine::kLevel1Warning,
                     AlarmStateMachine::kLevel2Warning}) {
        StatusContractInput distance = loadedWarning(code);
        distance.nearest_obstacle_distance_m =
            std::numeric_limits<double>::quiet_NaN();
        EXPECT_EQ(AlarmStateMachine::kInternalError,
                  validateStatusContract(distance).code);

        StatusContractInput top = loadedWarning(code);
        top.obstacle_top_z_map = std::numeric_limits<double>::quiet_NaN();
        EXPECT_EQ(AlarmStateMachine::kInternalError,
                  validateStatusContract(top).code);

        StatusContractInput uncertainty = loadedWarning(code);
        uncertainty.obstacle_uncertainty_m =
            std::numeric_limits<double>::quiet_NaN();
        EXPECT_EQ(AlarmStateMachine::kInternalError,
                  validateStatusContract(uncertainty).code);

        StatusContractInput clearance = loadedWarning(code);
        clearance.conservative_vertical_clearance_m =
            std::numeric_limits<double>::quiet_NaN();
        EXPECT_EQ(AlarmStateMachine::kInternalError,
                  validateStatusContract(clearance).code);
    }
}

TEST(CargoAlarmHeartbeat, HazardGeometryRejectsNegativePhysicalValues) {
    StatusContractInput distance = loadedWarning(
        AlarmStateMachine::kLevel1Warning);
    distance.nearest_obstacle_distance_m = -0.01;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(distance).code);

    StatusContractInput uncertainty = loadedWarning(
        AlarmStateMachine::kLevel2Warning);
    uncertainty.obstacle_uncertainty_m = -0.01;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(uncertainty).code);
}

TEST(CargoAlarmHeartbeat, ClearWithClusterRejectsDangerousGeometry) {
    const auto result = validateStatusContract(loadedClusterClear(2.0, 0.20));
    EXPECT_EQ(AlarmStateMachine::kInternalError, result.code);
    EXPECT_STREQ("clear_geometry_mismatch", result.reason);
}

TEST(CargoAlarmHeartbeat, ClearWithClusterAcceptsSafeGeometry) {
    EXPECT_EQ(AlarmStateMachine::kClear,
              validateStatusContract(loadedClusterClear(2.0, 1.0)).code);
    EXPECT_EQ(AlarmStateMachine::kClear,
              validateStatusContract(loadedClusterClear(6.0, 0.50)).code);
}

TEST(CargoAlarmHeartbeat, NoObstacleClearStillAllowsNanGeometry) {
    StatusContractInput clear = loadedWarning(AlarmStateMachine::kClear);
    clear.obstacle_count = 0U;
    clear.nearest_obstacle_distance_m =
        std::numeric_limits<double>::quiet_NaN();
    clear.obstacle_top_z_map = std::numeric_limits<double>::quiet_NaN();
    clear.obstacle_uncertainty_m =
        std::numeric_limits<double>::quiet_NaN();
    clear.conservative_vertical_clearance_m =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(AlarmStateMachine::kClear,
              validateStatusContract(clear).code);
}

TEST(CargoAlarmHeartbeat, InvalidStatusContractConfigIsRejected) {
    cargo_alarm::StatusContractConfig config;
    config.level2_distance_m = config.level1_distance_m;
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              validateStatusContract(
                  loadedWarning(AlarmStateMachine::kLevel1Warning),
                  config).code);
}

}  // namespace
