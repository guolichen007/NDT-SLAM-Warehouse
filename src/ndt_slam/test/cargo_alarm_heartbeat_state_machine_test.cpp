#include <gtest/gtest.h>

#define CARGO_ALARM_HEARTBEAT_STATE_MACHINE_ONLY
#include "../src/cargo_alarm_heartbeat_node.cpp"

namespace {

using cargo_alarm::AlarmStateMachine;

TEST(CargoAlarmHeartbeat, StartsFailSafeAndRequiresDelayedClear) {
    AlarmStateMachine state(0.5, 0.5);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid, state.currentCode());

    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid,
              state.ingest(true, AlarmStateMachine::kClear, 10.0, 100.0).code);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid, state.tick(10.499).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(10.5).code);
}

TEST(CargoAlarmHeartbeat, EscalatesImmediatelyAndRestartsClearDelay) {
    AlarmStateMachine state(2.0, 0.5);
    state.ingest(true, AlarmStateMachine::kClear, 1.0, 11.0);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(1.5).code);

    EXPECT_EQ(AlarmStateMachine::kInnerWarning,
              state.ingest(true, AlarmStateMachine::kInnerWarning, 1.6, 11.1).code);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid,
              state.ingest(true, AlarmStateMachine::kOuterOrInvalid, 1.7, 11.2).code);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid,
              state.ingest(true, AlarmStateMachine::kClear, 1.8, 11.3).code);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid, state.tick(2.299).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(2.3).code);
}

TEST(CargoAlarmHeartbeat, StaleBoundaryIsImmediatelyFailSafe) {
    AlarmStateMachine state(0.5, 0.0);
    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(true, AlarmStateMachine::kClear, 5.0, 50.0).code);
    EXPECT_EQ(AlarmStateMachine::kClear, state.tick(5.499).code);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid, state.tick(5.5).code);
}

TEST(CargoAlarmHeartbeat, RejectsInvalidEvidenceCodeAndTimestamp) {
    AlarmStateMachine state(1.0, 0.0);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid,
              state.ingest(false, AlarmStateMachine::kClear, 1.0, 1.0).code);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid,
              state.ingest(true, 0, 1.1, 1.1).code);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid,
              state.ingest(true, AlarmStateMachine::kClear, 1.2, 0.0).code);
}

TEST(CargoAlarmHeartbeat, RejectsSourceAndWallClockRollback) {
    AlarmStateMachine state(1.0, 0.0);
    EXPECT_EQ(AlarmStateMachine::kClear,
              state.ingest(true, AlarmStateMachine::kClear, 10.0, 100.0).code);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid,
              state.ingest(true, AlarmStateMachine::kClear, 10.1, 99.0).code);

    AlarmStateMachine wall_state(1.0, 0.0);
    wall_state.ingest(true, AlarmStateMachine::kClear, 20.0, 200.0);
    EXPECT_EQ(AlarmStateMachine::kOuterOrInvalid, wall_state.tick(19.0).code);
}

}  // namespace
