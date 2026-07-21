#include <gtest/gtest.h>

#include "ndt_slam/pending_cargo_envelope.hpp"

namespace ndt_slam {
namespace {

PendingCargoEnvelopeCandidate candidate(double stamp, float length) {
  PendingCargoEnvelopeCandidate value;
  value.valid = true;
  value.center_base = Eigen::Vector3f(0.0F, -2.0F, 1.5F);
  value.length_m = length;
  value.width_m = 1.0F;
  value.height_m = 1.2F;
  value.evidence_stamp_sec = stamp;
  return value;
}

TEST(PendingCargoEnvelopeTest, CurrentCandidateHasHighestPriority) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_loaded = true;
  input.current_candidate = candidate(10.0, 1.5F);
  input.retired_formal_shape = candidate(9.0, 2.5F);
  const auto result = buildPendingCargoEnvelope(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.source, PendingCargoEnvelopeSource::CURRENT_CANDIDATE);
  EXPECT_GT(result.length_m, 1.5F);
}

TEST(PendingCargoEnvelopeTest, StaleCandidateFallsBackToRetiredShape) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_loaded = true;
  input.current_candidate = candidate(8.0, 1.5F);
  input.retired_formal_shape = candidate(9.0, 2.5F);
  const auto result = buildPendingCargoEnvelope(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(
      result.source, PendingCargoEnvelopeSource::RETIRED_FORMAL_SHAPE);
}

TEST(PendingCargoEnvelopeTest, LoadedHookAlwaysGetsConfiguredFallback) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_loaded = true;
  input.hook_anchor_valid = true;
  input.hook_anchor_base = Eigen::Vector3f(0.0F, -2.0F, 1.5F);
  const auto result = buildPendingCargoEnvelope(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.source,
            PendingCargoEnvelopeSource::CONFIGURED_CONSERVATIVE);
  EXPECT_TRUE(toCargoObbFootprint(result).valid);
}

TEST(PendingCargoEnvelopeTest, EmptyHookCannotCreateEnvelope) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_anchor_valid = true;
  const auto result = buildPendingCargoEnvelope(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "hook_not_loaded");
}

}  // namespace
}  // namespace ndt_slam
