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

TEST(PendingCargoEnvelopeTest, CurrentCandidateBeatsRetiredAndOrigin) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_loaded = true;
  input.current_candidate = candidate(10.0, 1.5F);
  input.retired_formal_shape = candidate(9.0, 2.5F);
  input.lift_origin_candidate = candidate(9.0, 3.5F);
  const auto result = buildPendingCargoEnvelope(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.source, PendingCargoEnvelopeSource::CURRENT_CANDIDATE);
  EXPECT_GT(result.length_m, 1.5F);
}

TEST(PendingCargoEnvelopeTest, RetiredBeatsOriginAndConfigured) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_loaded = true;
  input.current_candidate = candidate(8.0, 1.5F);
  input.retired_formal_shape = candidate(9.0, 2.5F);
  input.lift_origin_candidate = candidate(9.0, 3.5F);
  input.hook_anchor_valid = true;
  const auto result = buildPendingCargoEnvelope(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(
      result.source, PendingCargoEnvelopeSource::RETIRED_FORMAL_SHAPE);
}

TEST(PendingCargoEnvelopeTest, ConfiguredFallbackIsBelowHook) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_loaded = true;
  input.hook_anchor_valid = true;
  input.hook_anchor_base = Eigen::Vector3f(0.0F, -2.0F, 1.5F);
  const auto result = buildPendingCargoEnvelope(input);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.source,
            PendingCargoEnvelopeSource::CONFIGURED_CONSERVATIVE);
  EXPECT_FLOAT_EQ(result.center_base.z(), 0.0F);
  EXPECT_TRUE(toCargoObbFootprint(result).valid);
}

TEST(PendingCargoEnvelopeTest, ExpandedHeightMatchesMinMax) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_loaded = true;
  input.current_candidate = candidate(10.0, 1.5F);
  const auto result = buildPendingCargoEnvelope(input);
  ASSERT_TRUE(result.valid);
  EXPECT_FLOAT_EQ(result.height_m,
                  result.top_z_base - result.bottom_z_base);
}

TEST(PendingCargoEnvelopeTest, CandidateUncertaintyExpandsHeightExactlyOnce) {
  PendingCargoEnvelopeConfig config;
  config.vertical_margin_m = 0.10F;
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_loaded = true;
  input.current_candidate = candidate(10.0, 1.5F);
  input.current_candidate.vertical_uncertainty_m = 0.25F;
  const auto result = buildPendingCargoEnvelope(input, config);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.height_m, 1.2F + 2.0F * (0.25F + 0.10F), 1.0e-6F);
  EXPECT_NEAR(result.top_z_base - result.bottom_z_base,
              result.height_m, 1.0e-6F);
}

TEST(PendingCargoEnvelopeTest, InvalidOrFutureTimestampRejected) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_loaded = true;
  input.current_candidate = candidate(10.1, 1.5F);
  EXPECT_FALSE(buildPendingCargoEnvelope(input).valid);
  input.current_candidate.evidence_stamp_sec =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(buildPendingCargoEnvelope(input).valid);
}

TEST(PendingCargoEnvelopeTest, PendingFootprintMinMaxMatchesEnvelope) {
  PendingCargoEnvelopeInput input;
  input.stamp_sec = 10.0;
  input.hook_loaded = true;
  input.current_candidate = candidate(10.0, 1.5F);
  input.current_candidate.horizontal_uncertainty_m = 0.12F;
  const auto envelope = buildPendingCargoEnvelope(input);
  const auto footprint = toCargoObbFootprint(envelope);
  ASSERT_TRUE(footprint.valid);
  EXPECT_FLOAT_EQ(footprint.length_m, envelope.length_m);
  EXPECT_FLOAT_EQ(footprint.width_m, envelope.width_m);
  EXPECT_FLOAT_EQ(footprint.min_z, envelope.bottom_z_base);
  EXPECT_FLOAT_EQ(footprint.max_z, envelope.top_z_base);
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
