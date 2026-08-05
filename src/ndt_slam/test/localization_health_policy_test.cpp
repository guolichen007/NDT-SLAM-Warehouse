#include <gtest/gtest.h>

#include "ndt_slam/localization_health_policy.hpp"

namespace ndt_slam {
namespace {

LocalizationHealthEvidence qualifiedEvidence(double stamp) {
    LocalizationHealthEvidence evidence;
    evidence.stamp_sec = stamp;
    evidence.ndt_converged = true;
    evidence.ndt_accepted = true;
    evidence.prediction_only = false;
    evidence.observability_valid = true;
    evidence.pose_finite = true;
    evidence.fitness = 0.20;
    evidence.raw_step_m = 0.05;
    evidence.maximum_allowed_step_m = 0.20;
    evidence.innovation_m = 0.10;
    return evidence;
}

TEST(LocalizationHealthPolicyTest, RequiresCompleteEightFrameWindow) {
    LocalizationHealthPolicy policy;
    for (int frame = 1; frame < 8; ++frame) {
        const auto decision = policy.update(qualifiedEvidence(frame * 0.1));
        EXPECT_FALSE(decision.localization_verified);
        EXPECT_EQ(frame, decision.consecutive_qualified_frames);
    }
    EXPECT_TRUE(policy.update(qualifiedEvidence(0.8)).localization_verified);
}

TEST(LocalizationHealthPolicyTest, ThreeConsecutiveFailuresRevokeVerification) {
    LocalizationHealthPolicy policy;
    for (int frame = 1; frame <= 8; ++frame) {
        policy.update(qualifiedEvidence(frame * 0.1));
    }
    for (int failure = 1; failure <= 2; ++failure) {
        auto bad = qualifiedEvidence(0.8 + failure * 0.1);
        bad.prediction_only = true;
        const auto decision = policy.update(bad);
        EXPECT_TRUE(decision.localization_verified);
        EXPECT_EQ(8 - failure, decision.consecutive_qualified_frames);
    }
    auto third_bad = qualifiedEvidence(1.1);
    third_bad.prediction_only = true;
    const auto revoked = policy.update(third_bad);
    EXPECT_FALSE(revoked.localization_verified);
    EXPECT_EQ(5, revoked.consecutive_qualified_frames);
    EXPECT_EQ("prediction_only", revoked.reason);
}

TEST(LocalizationHealthPolicyTest, TwoIsolatedFailuresDoNotBlockStartup) {
    LocalizationHealthPolicy policy;
    for (int frame = 1; frame <= 8; ++frame) {
        auto evidence = qualifiedEvidence(frame * 0.1);
        if (frame == 3 || frame == 7) evidence.ndt_accepted = false;
        const auto decision = policy.update(evidence);
        if (frame < 8) EXPECT_FALSE(decision.localization_verified);
    }
    const auto decision = policy.decision();
    EXPECT_TRUE(decision.localization_verified);
    EXPECT_EQ(6, decision.consecutive_qualified_frames);
    EXPECT_EQ(8, decision.evaluated_window_frames);
}

TEST(LocalizationHealthPolicyTest, TimeRollbackAndGapsResetEvidence) {
    LocalizationHealthPolicy policy;
    policy.update(qualifiedEvidence(1.0));
    EXPECT_EQ("time_not_monotonic",
              policy.update(qualifiedEvidence(0.9)).reason);
    EXPECT_EQ("frame_gap", policy.update(qualifiedEvidence(2.0)).reason);
}

TEST(LocalizationHealthPolicyTest, IdleLabelCannotBypassStrictEvidence) {
    LocalizationHealthPolicy policy;
    auto evidence = qualifiedEvidence(1.0);
    evidence.ekf_recovered = true;
    EXPECT_FALSE(policy.update(evidence).localization_verified);
    evidence = qualifiedEvidence(1.1);
    evidence.output_step_limited = true;
    EXPECT_FALSE(policy.update(evidence).localization_verified);
    evidence = qualifiedEvidence(1.2);
    evidence.fitness = 0.36;
    EXPECT_FALSE(policy.update(evidence).localization_verified);
}

}  // namespace
}  // namespace ndt_slam
