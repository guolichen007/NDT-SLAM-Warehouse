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

StatusContractInput faultStatus(int code, std::uint32_t mask) {
    StatusContractInput input;
    input.schema_valid = true;
    input.requested_code = code;
    input.fault_code = code;
    input.fault_mask = mask;
    return input;
}

TEST(CargoAlarmHeartbeat, StartupAndStaleUseSystemNotReady) {
    AlarmStateMachine::Config config;
    config.clear_delay_sec = 0.0;
    AlarmStateMachine state(0.5, config);
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady, state.currentCode());
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady, state.tick(1.0).code);

    state.ingest(AlarmStateMachine::kClear, 2.0, 20.0,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(2.0).code);
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady, state.tick(2.5).code);
}

TEST(CargoAlarmHeartbeat, SourceRollbackAndNonAdvancingStampUseCode30) {
    AlarmStateMachine state(0.5);
    state.ingest(AlarmStateMachine::kCargoInvalid, 10.0, 100.0);
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady,
              state.ingest(AlarmStateMachine::kCargoInvalid,
                           10.1, 99.0).code);

    AlarmStateMachine stalled(0.5);
    stalled.ingest(AlarmStateMachine::kCargoInvalid, 20.0, 200.0);
    stalled.ingest(AlarmStateMachine::kCargoInvalid, 20.4, 200.0);
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady,
              stalled.ingest(AlarmStateMachine::kCargoInvalid,
                             20.5, 200.0).code);
}

TEST(CargoAlarmHeartbeat, InvalidTimestampAndUnknownCodeUseInternalError) {
    AlarmStateMachine state(1.0);
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              state.ingest(AlarmStateMachine::kClear, 1.0, 0.0).code);
    EXPECT_EQ(AlarmStateMachine::kInternalError,
              state.ingest(99, 1.1, 1.1).code);
}

TEST(CargoAlarmHeartbeat, WarningEntryIsConfirmedButSevereClearanceIsImmediate) {
    AlarmStateMachine::Config config;
    config.clear_delay_sec = 0.0;
    AlarmStateMachine state(2.0, config);
    state.ingest(AlarmStateMachine::kClear, 1.0, 10.0, 6.0, 1.0, true);
    ASSERT_EQ(AlarmStateMachine::kClear, state.tick(1.0).code);

    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(AlarmStateMachine::kLevel2Warning,
                           1.1, 10.1, 4.0, 0.70).code);
    EXPECT_EQ(AlarmStateMachine::kLevel2Warning,
              state.ingest(AlarmStateMachine::kLevel2Warning,
                           1.2, 10.2, 4.0, 0.70).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kLevel1Warning,
                           1.3, 10.3, 2.0, 0.70).code);

    AlarmStateMachine severe(2.0, config);
    severe.ingest(AlarmStateMachine::kClear, 2.0, 20.0, 6.0, 1.0, true);
    severe.tick(2.0);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              severe.ingest(AlarmStateMachine::kLevel1Warning,
                            2.1, 20.1, 2.0, 0.49).code);
}

TEST(CargoAlarmHeartbeat, WarningExitUsesHysteresisAndClearDelay) {
    AlarmStateMachine::Config config;
    config.clear_delay_sec = 0.5;
    AlarmStateMachine state(2.0, config);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.49);
    ASSERT_EQ(AlarmStateMachine::kLevel1Warning, state.currentCode());

    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear,
                           1.1, 10.1, 3.10, 0.85).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear,
                           1.2, 10.2, 3.21, 0.85).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear,
                           1.3, 10.3, 3.21, 0.85).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.799).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(1.8).code);
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

TEST(CargoAlarmHeartbeat, FirstNoObstacleClearCannotReleaseLevel1Warning) {
    AlarmStateMachine state(0.8);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.49);
    ASSERT_EQ(AlarmStateMachine::kLevel1Warning, state.currentCode());

    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear, 1.1, 10.1,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(), true).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.3).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.6).code);
}

TEST(CargoAlarmHeartbeat, TwoFreshNoObstacleClearsStartDelay) {
    AlarmStateMachine state(2.0);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.49);

    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear, 1.1, 10.1,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(), true).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.2).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear, 1.3, 10.2,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(), true).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.799).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(1.8).code);
}

TEST(CargoAlarmHeartbeat, DuplicateSourceStampIsNotFreshClearEvidence) {
    AlarmStateMachine state(2.0);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.49);

    state.ingest(AlarmStateMachine::kClear, 1.1, 10.1,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear, 1.2, 10.1,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(), true).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.3).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear, 1.4, 10.2,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(), true).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.899).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(1.9).code);
}

TEST(CargoAlarmHeartbeat, OneClearThenStreamLossGoesDirectlyToCode30) {
    AlarmStateMachine state(0.8);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.49);
    state.ingest(AlarmStateMachine::kClear, 1.1, 10.1,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);

    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.6).code);
    EXPECT_EQ(AlarmStateMachine::kSystemNotReady, state.tick(1.901).code);
}

TEST(CargoAlarmHeartbeat, Level2AlsoRequiresTwoFreshClears) {
    AlarmStateMachine state(2.0);
    state.ingest(AlarmStateMachine::kLevel2Warning,
                 1.0, 10.0, 4.0, 0.49);

    state.ingest(AlarmStateMachine::kClear, 1.1, 10.1,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    EXPECT_EQ(AlarmStateMachine::kLevel2Warning, state.tick(1.2).code);
    EXPECT_EQ(AlarmStateMachine::kLevel2Warning,
              state.ingest(AlarmStateMachine::kClear, 1.3, 10.2,
                           std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::infinity(), true).code);
    EXPECT_EQ(AlarmStateMachine::kLevel2Warning, state.tick(1.799).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(1.8).code);
}

TEST(CargoAlarmHeartbeat, GeometryClearAlsoRequiresTwoFreshMessages) {
    AlarmStateMachine state(2.0);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.49);

    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear,
                           1.1, 10.1, 3.21, 0.85).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.2).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear,
                           1.3, 10.2, 3.21, 0.85).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(1.8).code);
}

TEST(CargoAlarmHeartbeat, FailedClearGeometryResetsConfirmation) {
    AlarmStateMachine state(2.0);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.49);
    state.ingest(AlarmStateMachine::kClear, 1.1, 10.1, 3.21, 0.85);

    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear,
                           1.2, 10.2, 3.10, 0.85).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning,
              state.ingest(AlarmStateMachine::kClear,
                           1.3, 10.3, 3.21, 0.85).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.4).code);
    state.ingest(AlarmStateMachine::kClear, 1.5, 10.4, 3.21, 0.85);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(2.0).code);
}

TEST(CargoAlarmHeartbeat, NewRiskCancelsPendingClearConfirmation) {
    AlarmStateMachine state(2.0);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.49);
    state.ingest(AlarmStateMachine::kClear, 1.1, 10.1,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.2, 10.2, 2.0, 0.70);

    state.ingest(AlarmStateMachine::kClear, 1.3, 10.3,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.4).code);
    state.ingest(AlarmStateMachine::kClear, 1.5, 10.4,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(2.0).code);

    AlarmStateMachine escalation(2.0);
    escalation.ingest(AlarmStateMachine::kLevel1Warning,
                      3.0, 30.0, 2.0, 0.49);
    escalation.ingest(AlarmStateMachine::kClear, 3.1, 30.1,
                      std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity(), true);
    escalation.ingest(AlarmStateMachine::kLevel2Warning,
                      3.2, 30.2, 3.10, 0.70);
    escalation.ingest(AlarmStateMachine::kClear, 3.3, 30.3,
                      std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::infinity(), true);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, escalation.tick(3.4).code);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, escalation.tick(3.8).code);
}

TEST(CargoAlarmHeartbeat, FaultCancelsPendingClearConfirmation) {
    AlarmStateMachine state(2.0);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.49);
    state.ingest(AlarmStateMachine::kClear, 1.1, 10.1,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    state.ingest(AlarmStateMachine::kGravityInvalid, 1.2, 10.2);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.3, 10.3, 2.0, 0.49);

    state.ingest(AlarmStateMachine::kClear, 1.4, 10.4,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.5).code);
}

TEST(CargoAlarmHeartbeat, ConfirmFramesThreeRequiresThreeFreshClears) {
    AlarmStateMachine::Config config;
    config.confirm_frames = 3;
    AlarmStateMachine state(2.0, config);
    state.ingest(AlarmStateMachine::kLevel1Warning,
                 1.0, 10.0, 2.0, 0.49);

    state.ingest(AlarmStateMachine::kClear, 1.1, 10.1,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    state.ingest(AlarmStateMachine::kClear, 1.2, 10.2,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    state.ingest(AlarmStateMachine::kClear, 1.3, 10.2,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.4).code);
    state.ingest(AlarmStateMachine::kClear, 1.5, 10.3,
                 std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(), true);
    EXPECT_EQ(AlarmStateMachine::kLevel1Warning, state.tick(1.999).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(2.0).code);
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
