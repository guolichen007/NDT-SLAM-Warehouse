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
    EXPECT_TRUE(result.confirmed_empty);
}

}  // namespace
