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

TEST(LocalizationHealthPolicyTest, RequiresTwentyStrictFrames) {
    LocalizationHealthPolicy policy;
    for (int frame = 1; frame < 20; ++frame) {
        const auto decision = policy.update(qualifiedEvidence(frame * 0.1));
        EXPECT_FALSE(decision.localization_verified);
        EXPECT_EQ(frame, decision.consecutive_qualified_frames);
    }
    EXPECT_TRUE(policy.update(qualifiedEvidence(2.0)).localization_verified);
}

TEST(LocalizationHealthPolicyTest, AnyStrictFailureRevokesVerification) {
    LocalizationHealthPolicy policy;
    for (int frame = 1; frame <= 20; ++frame) {
        policy.update(qualifiedEvidence(frame * 0.1));
    }
    auto bad = qualifiedEvidence(2.1);
    bad.prediction_only = true;
    const auto decision = policy.update(bad);
    EXPECT_FALSE(decision.localization_verified);
    EXPECT_EQ(0, decision.consecutive_qualified_frames);
    EXPECT_EQ("prediction_only", decision.reason);
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
