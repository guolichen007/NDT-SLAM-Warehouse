#include <gtest/gtest.h>

#include <vector>

#include "ndt_slam/local_map_update_policy.hpp"
#include "ndt_slam/stationary_motion_policy.hpp"

namespace ndt_slam {
namespace {

StationaryMotionInput reliableInput(double stamp,
                                    const Eigen::Vector2d& raw,
                                    const Eigen::Vector2d& filtered,
                                    double raw_increment,
                                    double speed = 0.0) {
    StationaryMotionInput input;
    input.stamp_sec = stamp;
    input.ndt_converged = true;
    input.ndt_accepted = true;
    input.registration_quality_valid = true;
    input.persistent_map_quality_valid = true;
    input.raw_position = raw;
    input.filtered_position = filtered;
    input.filtered_velocity = Eigen::Vector2d(speed, 0.0);
    input.raw_increment_m = raw_increment;
    input.allowed_physical_step_m = 0.25;
    return input;
}

StationaryMotionInput rejectedButGeometricInput(
    double stamp, const Eigen::Vector2d& raw,
    const Eigen::Vector2d& filtered, double raw_increment) {
    StationaryMotionInput input = reliableInput(
        stamp, raw, filtered, raw_increment, 0.0);
    input.ndt_accepted = false;
    input.prediction_only = true;
    input.registration_quality_valid = false;
    input.persistent_map_quality_valid = false;
    input.raw_motion_observation_valid = true;
    return input;
}

void enterStationary(StationaryMotionPolicy& policy,
                     double* stamp,
                     Eigen::Vector2d* raw) {
    for (int i = 0; i < 20; ++i) {
        *stamp += 0.1;
        raw->x() += 0.001;
        policy.update(reliableInput(*stamp, *raw, Eigen::Vector2d::Zero(),
                                    0.001));
    }
    ASSERT_EQ(policy.state(), RuntimeMotionState::STATIONARY_HOLD);
}

TEST(StationaryMotionPolicyTest, DuplicateTimestampDoesNotEnterStationary) {
    StationaryMotionPolicy policy;
    double stamp = 1.0;
    for (int i = 0; i < 40; ++i) {
        policy.update(reliableInput(stamp, Eigen::Vector2d::Zero(),
                                    Eigen::Vector2d::Zero(), 0.0));
    }
    EXPECT_EQ(policy.state(), RuntimeMotionState::MOVING);
}

TEST(StationaryMotionPolicyTest,
     BoundedUnreliableGapsDoNotEraseStationaryEvidence) {
    StationaryMotionPolicy policy;
    StationaryMotionPolicyConfig config;
    config.enter_confirm_frames = 4;
    config.enter_max_consecutive_unreliable_frames = 2;
    policy.setConfig(config);

    double stamp = 0.0;
    Eigen::Vector2d raw = Eigen::Vector2d::Zero();
    StationaryMotionDecision decision;
    for (int i = 0; i < 3; ++i) {
        stamp += 0.1;
        raw.x() += 0.001;
        decision = policy.update(
            reliableInput(stamp, raw, Eigen::Vector2d::Zero(), 0.001));
    }
    EXPECT_EQ(3, decision.stationary_entry_confirm_count);

    stamp += 0.1;
    raw.x() += 0.001;
    decision = policy.update(rejectedButGeometricInput(
        stamp, raw, Eigen::Vector2d::Zero(), 0.001));
    EXPECT_EQ(RuntimeMotionState::MOVING, decision.state);
    EXPECT_EQ("STATIONARY_ENTRY_EVIDENCE_GAP", decision.reason);
    EXPECT_EQ(3, decision.stationary_entry_confirm_count);
    EXPECT_EQ(1, decision.stationary_entry_unreliable_count);

    stamp += 0.1;
    raw.x() += 0.001;
    decision = policy.update(
        reliableInput(stamp, raw, Eigen::Vector2d::Zero(), 0.001));
    EXPECT_EQ(RuntimeMotionState::STATIONARY_HOLD, decision.state);
    EXPECT_EQ("STATIONARY_CONFIRMED", decision.reason);
}

TEST(StationaryMotionPolicyTest,
     ExcessiveUnreliableGapResetsStationaryEvidence) {
    StationaryMotionPolicy policy;
    StationaryMotionPolicyConfig config;
    config.enter_confirm_frames = 4;
    config.enter_max_consecutive_unreliable_frames = 2;
    policy.setConfig(config);

    double stamp = 0.0;
    Eigen::Vector2d raw = Eigen::Vector2d::Zero();
    for (int i = 0; i < 3; ++i) {
        stamp += 0.1;
        raw.x() += 0.001;
        policy.update(
            reliableInput(stamp, raw, Eigen::Vector2d::Zero(), 0.001));
    }

    StationaryMotionDecision decision;
    for (int i = 0; i < 3; ++i) {
        stamp += 0.1;
        raw.x() += 0.001;
        decision = policy.update(rejectedButGeometricInput(
            stamp, raw, Eigen::Vector2d::Zero(), 0.001));
    }
    EXPECT_EQ(RuntimeMotionState::MOVING, decision.state);
    EXPECT_EQ(0, decision.stationary_entry_confirm_count);
    EXPECT_EQ(0, decision.stationary_entry_unreliable_count);
}

TEST(StationaryMotionPolicyTest,
     RejectedButExplicitRawMotionResetsStationaryEvidence) {
    StationaryMotionPolicy policy;
    StationaryMotionPolicyConfig config;
    config.enter_confirm_frames = 4;
    config.enter_max_consecutive_unreliable_frames = 2;
    policy.setConfig(config);

    double stamp = 0.0;
    Eigen::Vector2d raw = Eigen::Vector2d::Zero();
    for (int i = 0; i < 3; ++i) {
        stamp += 0.1;
        raw.x() += 0.001;
        policy.update(
            reliableInput(stamp, raw, Eigen::Vector2d::Zero(), 0.001));
    }

    stamp += 0.1;
    raw.x() += 0.05;
    const auto decision = policy.update(rejectedButGeometricInput(
        stamp, raw, Eigen::Vector2d::Zero(), 0.05));
    EXPECT_EQ(RuntimeMotionState::MOVING, decision.state);
    EXPECT_EQ("MOVING", decision.reason);
    EXPECT_EQ(0, decision.stationary_entry_confirm_count);
    EXPECT_EQ(0, decision.stationary_entry_unreliable_count);
}

TEST(StationaryMotionPolicyTest,
     AcceptedSoftFrameUpdatesLocalButNotPersistentMap) {
    StationaryMotionPolicy policy;
    auto input = reliableInput(
        1.0, Eigen::Vector2d(0.10, 0.0),
        Eigen::Vector2d(0.08, 0.0), 0.10, 0.8);
    input.persistent_map_quality_valid = false;

    const auto decision = policy.update(input);

    EXPECT_EQ(decision.state, RuntimeMotionState::MOVING);
    EXPECT_TRUE(decision.allow_local_map_update);
    EXPECT_FALSE(decision.allow_persistent_map_commit);
}

TEST(StationaryMotionPolicyTest, InconsistentDriftCannotExitHold) {
    StationaryMotionPolicy policy;
    double stamp = 0.0;
    Eigen::Vector2d raw = Eigen::Vector2d::Zero();
    enterStationary(policy, &stamp, &raw);

    double accumulated_path = 0.0;
    bool saw_drift_rejection = false;
    for (int i = 0; i < 28; ++i) {
        const Eigen::Vector2d delta = (i % 4 == 0)
            ? Eigen::Vector2d(0.025, 0.0)
            : (i % 4 == 1)
                ? Eigen::Vector2d(0.0, 0.025)
                : (i % 4 == 2)
                    ? Eigen::Vector2d(-0.025, 0.0)
                    : Eigen::Vector2d(0.0, -0.025);
        raw += delta;
        accumulated_path += delta.norm();
        stamp += 0.1;
        const auto decision = policy.update(
            reliableInput(stamp, raw, Eigen::Vector2d::Zero(), delta.norm()));
        saw_drift_rejection = saw_drift_rejection ||
            decision.reason == "DRIFT_ONLY_REJECTED";
        EXPECT_NE(decision.state, RuntimeMotionState::CATCH_UP);
        EXPECT_FALSE(decision.allow_local_map_update);
        EXPECT_FALSE(decision.allow_persistent_map_commit);
    }
    EXPECT_NEAR(accumulated_path, 0.70, 1.0e-9);
    // Let the bounded evidence window age out. Random cyclic drift must not
    // escape through the anchor-drift failsafe.
    for (int i = 0; i < 18; ++i) {
        stamp += 0.1;
        const auto decision = policy.update(
            reliableInput(stamp, raw, Eigen::Vector2d::Zero(), 0.0));
        saw_drift_rejection = saw_drift_rejection ||
            decision.reason == "DRIFT_ONLY_REJECTED";
        EXPECT_NE(decision.state, RuntimeMotionState::CATCH_UP);
    }
    EXPECT_EQ(policy.state(), RuntimeMotionState::STATIONARY_HOLD);
    EXPECT_TRUE(saw_drift_rejection);
}

TEST(StationaryMotionPolicyTest,
     DroppedFrameLowIncrementsAccumulateBySensorTime) {
    StationaryMotionPolicy policy;
    double stamp = 0.0;
    Eigen::Vector2d raw = Eigen::Vector2d::Zero();
    enterStationary(policy, &stamp, &raw);

    StationaryMotionDecision decision;
    bool saw_movement_confirmed = false;
    for (int i = 0; i < 10; ++i) {
        // A 0.2 s processed-frame interval is expected when the latest-only
        // queue drops a frame. No individual displacement reaches the legacy
        // 0.02 m threshold, but the time-window motion is coherent and real.
        stamp += 0.2;
        raw.x() += 0.019;
        decision = policy.update(
            reliableInput(stamp, raw, Eigen::Vector2d::Zero(), 0.019, 0.095));
        saw_movement_confirmed =
            saw_movement_confirmed || decision.movement_confirmed;
        if (decision.state == RuntimeMotionState::MOVING_CONFIRM) {
            EXPECT_FALSE(decision.apply_position_constraint);
            EXPECT_FALSE(decision.allow_local_map_update);
            EXPECT_FALSE(decision.allow_persistent_map_commit);
        }
    }
    EXPECT_EQ(decision.state, RuntimeMotionState::CATCH_UP);
    EXPECT_TRUE(saw_movement_confirmed);
}

TEST(StationaryMotionPolicyTest,
     OneDirectionOutlierDoesNotEraseMotionWindow) {
    StationaryMotionPolicy policy;
    double stamp = 0.0;
    Eigen::Vector2d raw = Eigen::Vector2d::Zero();
    enterStationary(policy, &stamp, &raw);

    const std::vector<Eigen::Vector2d> deltas = {
        Eigen::Vector2d(0.05, 0.00),
        Eigen::Vector2d(0.05, 0.00),
        Eigen::Vector2d(-0.01, 0.01),
        Eigen::Vector2d(0.05, 0.00),
        Eigen::Vector2d(0.05, 0.00)};
    StationaryMotionDecision decision;
    for (const auto& delta : deltas) {
        stamp += 0.1;
        raw += delta;
        decision = policy.update(
            reliableInput(stamp, raw, Eigen::Vector2d::Zero(),
                          delta.norm(), 0.10));
    }
    EXPECT_EQ(decision.state, RuntimeMotionState::CATCH_UP);
    EXPECT_TRUE(decision.start_catch_up);
}

TEST(StationaryMotionPolicyTest, RealMotionUsesBoundedCatchUpBeforeMoving) {
    StationaryMotionPolicy policy;
    double stamp = 0.0;
    Eigen::Vector2d raw = Eigen::Vector2d::Zero();
    enterStationary(policy, &stamp, &raw);
    const Eigen::Vector2d anchor = Eigen::Vector2d::Zero();

    StationaryMotionDecision decision;
    bool saw_movement_confirmed = false;
    for (int i = 0; i < 3; ++i) {
        raw.x() += 0.05;
        stamp += 0.1;
        decision = policy.update(
            reliableInput(stamp, raw, anchor, 0.05, 0.05));
        saw_movement_confirmed =
            saw_movement_confirmed || decision.movement_confirmed;
    }
    ASSERT_EQ(decision.state, RuntimeMotionState::CATCH_UP);
    EXPECT_TRUE(saw_movement_confirmed);
    EXPECT_TRUE(decision.start_catch_up);
    EXPECT_FALSE(decision.allow_local_map_update);
    EXPECT_FALSE(decision.allow_persistent_map_commit);

    stamp += 0.1;
    decision = policy.update(reliableInput(stamp, raw, anchor, 0.0, 0.05));
    ASSERT_EQ(decision.state, RuntimeMotionState::CATCH_UP);
    EXPECT_LE(decision.catch_up_step_m, 0.08 + 1.0e-12);
    EXPECT_FALSE(decision.allow_local_map_update);
    EXPECT_FALSE(decision.allow_persistent_map_commit);

    Eigen::Vector2d filtered = decision.constrained_position;
    stamp += 0.1;
    decision = policy.update(reliableInput(stamp, raw, filtered, 0.0, 0.05));
    ASSERT_EQ(decision.state, RuntimeMotionState::CATCH_UP);
    EXPECT_LE(decision.catch_up_step_m, 0.08 + 1.0e-12);

    filtered = decision.constrained_position;
    stamp += 0.1;
    decision = policy.update(reliableInput(stamp, raw, filtered, 0.0, 0.05));
    EXPECT_EQ(decision.state, RuntimeMotionState::MOVING);
    EXPECT_EQ(decision.reason, "CATCH_UP_COMPLETE_RELEASE_GUARD");
    EXPECT_FALSE(decision.allow_local_map_update);
    EXPECT_FALSE(decision.allow_persistent_map_commit);

    stamp += 0.1;
    decision = policy.update(reliableInput(stamp, raw, raw, 0.0, 0.05));
    EXPECT_EQ(decision.state, RuntimeMotionState::MOVING);
    EXPECT_TRUE(decision.allow_local_map_update);
    EXPECT_TRUE(decision.allow_persistent_map_commit);
}

TEST(StationaryMotionPolicyTest, VeryLowSpeedCoherentMotionCanExit) {
    StationaryMotionPolicy policy;
    StationaryMotionPolicyConfig config;
    config.exit_evidence_window_sec = 30.0;
    policy.setConfig(config);
    double stamp = 0.0;
    Eigen::Vector2d raw = Eigen::Vector2d::Zero();
    enterStationary(policy, &stamp, &raw);

    StationaryMotionDecision decision;
    bool saw_movement_confirmed = false;
    for (int i = 0; i < 16; ++i) {
        stamp += 1.0;
        raw.x() += 0.01;
        decision = policy.update(
            reliableInput(stamp, raw, Eigen::Vector2d::Zero(), 0.01, 0.01));
        saw_movement_confirmed =
            saw_movement_confirmed || decision.movement_confirmed;
    }
    EXPECT_EQ(decision.state, RuntimeMotionState::CATCH_UP);
    EXPECT_TRUE(saw_movement_confirmed);
}

TEST(StationaryMotionPolicyTest, PredictionAndNonphysicalStepsNeverConfirmExit) {
    StationaryMotionPolicy policy;
    double stamp = 0.0;
    Eigen::Vector2d raw = Eigen::Vector2d::Zero();
    enterStationary(policy, &stamp, &raw);

    for (int i = 0; i < 5; ++i) {
        stamp += 0.1;
        raw.x() += 0.30;
        auto input = reliableInput(stamp, raw, Eigen::Vector2d::Zero(), 0.30);
        input.prediction_only = true;
        const auto decision = policy.update(input);
        EXPECT_EQ(decision.state, RuntimeMotionState::STATIONARY_HOLD);
        EXPECT_FALSE(decision.allow_local_map_update);
        EXPECT_FALSE(decision.allow_persistent_map_commit);
    }
}

TEST(StationaryMotionPolicyTest,
     StopToMoveRejectedNdtGetsTargetOnlyEscapeAndThenRecovers) {
    StationaryMotionPolicy motion_policy;
    double stamp = 0.0;
    Eigen::Vector2d raw = Eigen::Vector2d::Zero();
    enterStationary(motion_policy, &stamp, &raw);

    StationaryMotionDecision motion_decision;
    LocalMapUpdateDecision local_map_decision;
    for (int i = 0; i < 3; ++i) {
        stamp += 0.1;
        raw.x() += 0.05;
        motion_decision = motion_policy.update(
            rejectedButGeometricInput(
                stamp, raw, Eigen::Vector2d::Zero(), 0.05));

        LocalMapUpdateInput local_input;
        local_input.registration_success = true;
        local_input.registration_cloud_valid = true;
        local_input.runtime_pose_finite = true;
        local_input.normal_motion_update_allowed =
            motion_decision.state == RuntimeMotionState::MOVING;
        local_input.motion_escape_refresh_allowed =
            motion_decision.allow_local_map_motion_escape_refresh;
        local_input.relocalization_pose_reliable = true;
        local_input.frames_since_update = 16;
        local_map_decision = evaluateLocalMapUpdate(local_input);
    }

    EXPECT_EQ(RuntimeMotionState::STATIONARY_HOLD, motion_decision.state);
    EXPECT_TRUE(
        motion_decision.allow_local_map_motion_escape_refresh);
    EXPECT_FALSE(motion_decision.allow_persistent_map_commit);
    EXPECT_TRUE(local_map_decision.eligible);
    EXPECT_TRUE(local_map_decision.due);
    EXPECT_EQ(LocalMapUpdateMode::MOTION_ESCAPE_REFRESH,
              local_map_decision.mode);

    // Once accepted geometry returns, the original bounded confirmation and
    // CATCH_UP path remains responsible for returning to MOVING.
    for (int i = 0; i < 3; ++i) {
        stamp += 0.1;
        raw.x() += 0.05;
        motion_decision = motion_policy.update(
            reliableInput(
                stamp, raw, Eigen::Vector2d::Zero(), 0.05, 0.5));
    }
    ASSERT_EQ(RuntimeMotionState::CATCH_UP, motion_decision.state);

    Eigen::Vector2d filtered = motion_decision.constrained_position;
    for (int i = 0;
         i < 10 && motion_policy.state() != RuntimeMotionState::MOVING;
         ++i) {
        stamp += 0.1;
        motion_decision = motion_policy.update(
            reliableInput(stamp, raw, filtered, 0.0, 0.0));
        filtered = motion_decision.constrained_position;
    }
    EXPECT_EQ(RuntimeMotionState::MOVING, motion_policy.state());
    EXPECT_FALSE(
        motion_decision.allow_local_map_motion_escape_refresh);

    stamp += 0.1;
    motion_decision = motion_policy.update(
        reliableInput(stamp, raw, raw, 0.0, 0.0));
    EXPECT_TRUE(motion_decision.allow_local_map_update);
    EXPECT_TRUE(motion_decision.allow_persistent_map_commit);
}

}  // namespace
}  // namespace ndt_slam
