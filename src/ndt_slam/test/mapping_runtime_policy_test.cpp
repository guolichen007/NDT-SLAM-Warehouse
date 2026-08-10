#include "ndt_slam/mapping_runtime_policy.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

TEST(MappingRuntimePolicyTest, HighFitnessAloneOnlyPausesQuality) {
  MappingRuntimePolicy policy({3, 0.4});
  MappingRuntimeDecision decision;
  for (int index = 0; index < 20; ++index) {
    MappingRuntimeEvidence evidence;
    evidence.stamp_sec = 1.0 + 0.2 * index;
    evidence.high_fitness = true;
    decision = policy.update(evidence);
  }
  EXPECT_EQ(decision.state, MappingAuthorityState::PAUSED_QUALITY);
  EXPECT_FALSE(decision.trusted_writes_allowed);
  EXPECT_TRUE(decision.formal_warning_authority_allowed);
  EXPECT_FALSE(decision.fail_closed_latched);
}

TEST(MappingRuntimePolicyTest, SingleNonConvergenceRecovers) {
  MappingRuntimePolicy policy({3, 0.4});
  MappingRuntimeEvidence bad;
  bad.stamp_sec = 1.0;
  bad.ndt_converged = false;
  EXPECT_EQ(policy.update(bad).state,
            MappingAuthorityState::PAUSED_QUALITY);

  MappingRuntimeEvidence healthy;
  healthy.stamp_sec = 1.2;
  const auto decision = policy.update(healthy);
  EXPECT_EQ(decision.state, MappingAuthorityState::ACTIVE);
  EXPECT_TRUE(decision.trusted_writes_allowed);
}

TEST(MappingRuntimePolicyTest, GeometryDegradationPausesWithoutFailingClosed) {
  MappingRuntimePolicy policy({3, 0.4});
  MappingRuntimeEvidence evidence;
  evidence.stamp_sec = 1.0;
  evidence.geometry_invalid = true;
  const auto decision = policy.update(evidence);
  EXPECT_EQ(decision.state, MappingAuthorityState::PAUSED_QUALITY);
  EXPECT_FALSE(decision.trusted_writes_allowed);
  EXPECT_TRUE(decision.formal_warning_authority_allowed);
  EXPECT_FALSE(decision.fail_closed_latched);
}

TEST(MappingRuntimePolicyTest, ContinuousHardFailureLatchesFailClosed) {
  MappingRuntimePolicy policy({3, 0.4});
  MappingRuntimeDecision decision;
  for (int index = 0; index < 3; ++index) {
    MappingRuntimeEvidence evidence;
    evidence.stamp_sec = 1.0 + 0.2 * index;
    evidence.ndt_converged = false;
    decision = policy.update(evidence);
  }
  EXPECT_EQ(decision.state, MappingAuthorityState::FAIL_CLOSED);
  EXPECT_TRUE(decision.fail_closed_latched);
  EXPECT_FALSE(decision.trusted_writes_allowed);
  EXPECT_FALSE(decision.formal_warning_authority_allowed);
}

TEST(MappingRuntimePolicyTest, NonfiniteFailsImmediatelyAndStaysLatched) {
  MappingRuntimePolicy policy({8, 2.0});
  MappingRuntimeEvidence invalid;
  invalid.stamp_sec = 1.0;
  invalid.nonfinite = true;
  EXPECT_EQ(policy.update(invalid).state,
            MappingAuthorityState::FAIL_CLOSED);

  MappingRuntimeEvidence healthy;
  healthy.stamp_sec = 2.0;
  EXPECT_EQ(policy.update(healthy).state,
            MappingAuthorityState::FAIL_CLOSED);
}

TEST(MappingRuntimePolicyTest, IoPauseDoesNotBecomeLocalizationFailure) {
  MappingRuntimePolicy policy;
  MappingRuntimeEvidence evidence;
  evidence.stamp_sec = 1.0;
  evidence.io_paused = true;
  const auto decision = policy.update(evidence);
  EXPECT_EQ(decision.state, MappingAuthorityState::PAUSED_IO);
  EXPECT_TRUE(decision.formal_warning_authority_allowed);
}

}  // namespace
}  // namespace ndt_slam
