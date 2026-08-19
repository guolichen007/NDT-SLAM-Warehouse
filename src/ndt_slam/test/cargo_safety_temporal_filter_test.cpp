#include <gtest/gtest.h>

#include "ndt_slam/cargo_safety_temporal_filter.hpp"

namespace ndt_slam {
namespace {

CargoSafetyTemporalInput hazard(
    double stamp, float x, std::size_t points = 30U,
    std::uint16_t code = 17U) {
  CargoSafetyTemporalInput input;
  input.stamp_sec = stamp;
  input.raw_valid = true;
  input.raw_code = code;
  input.cluster_points = points;
  input.cluster_centroid = Eigen::Vector3f(x, 1.0F, 1.0F);
  input.footprint_distance_m = 1.5F + x;
  input.conservative_clearance_m = 0.30F;
  return input;
}

CargoSafetyTemporalInput clear(double stamp) {
  CargoSafetyTemporalInput input;
  input.stamp_sec = stamp;
  input.raw_valid = true;
  input.raw_code = 14U;
  return input;
}

TEST(CargoSafetyTemporalFilter, SparseOrJumpingPointsCannotCreateLevelOne) {
  CargoSafetyTemporalFilter filter;
  EXPECT_FALSE(filter.update(hazard(1.0, 0.0F, 8U)).stable);
  EXPECT_FALSE(filter.update(hazard(1.2, 1.5F)).stable);
  EXPECT_FALSE(filter.update(hazard(1.4, -1.5F)).stable);
  EXPECT_FALSE(filter.update(hazard(1.6, 1.5F)).stable);
}

TEST(CargoSafetyTemporalFilter,
     InvalidConfigIsExplicitAndNeverRestoredToDefaults) {
  CargoSafetyTemporalConfig config;
  config.hazard_confirm_frames = 1;
  config.maximum_evidence_gap_sec = -1.0;
  CargoSafetyTemporalFilter filter;
  const CargoConfigValidationResult validation = filter.setConfig(config);
  EXPECT_FALSE(validation.valid);
  EXPECT_NE(validation.summary().find("hazard_confirm_frames"),
            std::string::npos);
  EXPECT_NE(validation.summary().find("maximum_evidence_gap_sec"),
            std::string::npos);
  EXPECT_EQ(filter.config().hazard_confirm_frames, 1);
  EXPECT_DOUBLE_EQ(filter.config().maximum_evidence_gap_sec, -1.0);
  EXPECT_FALSE(filter.update(hazard(1.0, 0.0F)).stable);
}

TEST(CargoSafetyTemporalFilter, ThreeContinuousFreshClustersConfirmHazard) {
  CargoSafetyTemporalFilter filter;
  EXPECT_FALSE(filter.update(hazard(1.0, 0.00F)).stable);
  EXPECT_FALSE(filter.update(hazard(1.2, 0.05F)).stable);
  const CargoSafetyTemporalDecision decision =
      filter.update(hazard(1.4, 0.09F));
  EXPECT_TRUE(decision.stable);
  EXPECT_TRUE(decision.newly_confirmed);
  EXPECT_EQ(decision.code, 17U);
}

TEST(CargoSafetyTemporalFilter, RepeatedStampDoesNotAdvanceEvidence) {
  CargoSafetyTemporalFilter filter;
  EXPECT_FALSE(filter.update(hazard(1.0, 0.0F)).stable);
  EXPECT_FALSE(filter.update(hazard(1.0, 0.0F)).stable);
  EXPECT_FALSE(filter.update(hazard(1.2, 0.05F)).stable);
  EXPECT_TRUE(filter.update(hazard(1.4, 0.08F)).stable);
}

TEST(CargoSafetyTemporalFilter, ConfirmedHazardRepeatedStampIsNotCurrent) {
  CargoSafetyTemporalFilter filter;
  filter.update(hazard(1.0, 0.0F));
  filter.update(hazard(1.2, 0.05F));
  ASSERT_EQ(filter.update(hazard(1.4, 0.08F)).code, 17U);
  const CargoSafetyTemporalDecision repeated =
      filter.update(hazard(1.4, 0.08F));
  EXPECT_FALSE(repeated.stable);
  EXPECT_TRUE(repeated.pending);
  EXPECT_EQ(repeated.code, 0U);
}

TEST(CargoSafetyTemporalFilter, SourceRollbackStartsRecoverableNewEpoch) {
  CargoSafetyTemporalFilter filter;
  filter.update(hazard(100.0, 0.0F));
  filter.update(hazard(100.2, 0.05F));
  ASSERT_TRUE(filter.update(hazard(100.4, 0.08F)).stable);
  EXPECT_FALSE(filter.update(hazard(1.0, 0.0F)).stable);
  EXPECT_FALSE(filter.update(hazard(1.2, 0.05F)).stable);
  EXPECT_TRUE(filter.update(hazard(1.4, 0.08F)).stable);
}

TEST(CargoSafetyTemporalFilter, TwoFreshClearFramesReleaseHazardToClear) {
  CargoSafetyTemporalFilter filter;
  filter.update(hazard(1.0, 0.0F));
  filter.update(hazard(1.2, 0.05F));
  ASSERT_EQ(filter.update(hazard(1.4, 0.08F)).code, 17U);
  const CargoSafetyTemporalDecision first_clear = filter.update(clear(1.6));
  EXPECT_FALSE(first_clear.stable);
  EXPECT_TRUE(first_clear.pending);
  EXPECT_EQ(first_clear.code, 0U);
  const CargoSafetyTemporalDecision second_clear = filter.update(clear(1.8));
  EXPECT_TRUE(second_clear.stable);
  EXPECT_EQ(second_clear.code, 14U);
}

TEST(CargoSafetyTemporalFilter, ConfirmedHazardCannotBeHeldIndefinitely) {
  CargoSafetyTemporalFilter filter;
  filter.update(hazard(1.0, 0.0F));
  filter.update(hazard(1.2, 0.05F));
  ASSERT_EQ(filter.update(hazard(1.4, 0.08F)).code, 17U);
  for (int index = 0; index < 6; ++index) {
    const CargoSafetyTemporalDecision decision = filter.update(
        hazard(1.6 + 0.2 * static_cast<double>(index), 0.10F, 8U));
    EXPECT_FALSE(decision.stable);
    EXPECT_TRUE(decision.pending);
    EXPECT_EQ(decision.code, 0U);
  }
}

TEST(CargoSafetyTemporalFilter, LevelTransitionAlsoRequiresConfirmation) {
  CargoSafetyTemporalFilter filter;
  filter.update(hazard(1.0, 0.0F));
  filter.update(hazard(1.2, 0.05F));
  ASSERT_EQ(filter.update(hazard(1.4, 0.08F)).code, 17U);
  EXPECT_FALSE(filter.update(hazard(1.6, 0.10F, 30U, 18U)).stable);
  EXPECT_FALSE(filter.update(hazard(1.8, 0.12F, 30U, 18U)).stable);
  EXPECT_EQ(filter.update(hazard(2.0, 0.14F, 30U, 18U)).code, 18U);
}

TEST(CargoSafetyTemporalFilter, ConfirmedHazardThenSparseClusterIsPending) {
  CargoSafetyTemporalFilter filter;
  filter.update(hazard(1.0, 0.0F));
  filter.update(hazard(1.2, 0.05F));
  ASSERT_EQ(filter.update(hazard(1.4, 0.08F)).code, 17U);
  const CargoSafetyTemporalDecision sparse =
      filter.update(hazard(1.6, 0.10F, 8U));
  EXPECT_FALSE(sparse.stable);
  EXPECT_TRUE(sparse.pending);
  EXPECT_EQ(sparse.code, 0U);
  EXPECT_EQ(sparse.reason, "hazard_cluster_too_sparse");
}

TEST(CargoSafetyTemporalFilter, ConfirmedHazardThenDiscontinuityIsPending) {
  CargoSafetyTemporalFilter filter;
  filter.update(hazard(1.0, 0.0F));
  filter.update(hazard(1.2, 0.05F));
  ASSERT_EQ(filter.update(hazard(1.4, 0.08F)).code, 17U);
  const CargoSafetyTemporalDecision jump =
      filter.update(hazard(1.6, 1.50F));
  EXPECT_FALSE(jump.stable);
  EXPECT_TRUE(jump.pending);
  EXPECT_EQ(jump.code, 0U);
  EXPECT_EQ(jump.reason, "hazard_spatial_discontinuity");
}

TEST(CargoSafetyTemporalFilter, ConfirmedHazardThenEvidenceGapIsPending) {
  CargoSafetyTemporalFilter filter;
  filter.update(hazard(1.0, 0.0F));
  filter.update(hazard(1.2, 0.05F));
  ASSERT_EQ(filter.update(hazard(1.4, 0.08F)).code, 17U);
  const CargoSafetyTemporalDecision gap =
      filter.update(hazard(2.2, 0.10F));
  EXPECT_FALSE(gap.stable);
  EXPECT_TRUE(gap.pending);
  EXPECT_EQ(gap.code, 0U);
  EXPECT_EQ(gap.reason, "hazard_evidence_gap");
}

TEST(CargoSafetyTemporalFilter, UnconfirmedHazardNeverFallsBackToClear) {
  CargoSafetyTemporalFilter filter;
  filter.update(clear(1.0));
  ASSERT_EQ(filter.update(clear(1.2)).code, 14U);
  const CargoSafetyTemporalDecision sparse =
      filter.update(hazard(1.4, 0.0F, 8U));
  EXPECT_FALSE(sparse.stable);
  EXPECT_TRUE(sparse.pending);
  const CargoSafetyTemporalDecision first_robust =
      filter.update(hazard(1.6, 0.0F));
  EXPECT_FALSE(first_robust.stable);
  EXPECT_TRUE(first_robust.pending);
}

TEST(CargoSafetyTemporalFilter, H1UnresolvedEvidenceNeverConfirmsClear) {
  CargoSafetyTemporalFilter filter;
  filter.update(clear(1.0));
  ASSERT_EQ(filter.update(clear(1.2)).code, 14U);

  // H1 fail-closed raw evidence (raw_valid=false, non-protocol code) must not
  // enter CLEAR confirmation.
  CargoSafetyTemporalInput unresolved;
  unresolved.stamp_sec = 1.4;
  unresolved.raw_valid = false;
  unresolved.raw_code = 0U;
  const CargoSafetyTemporalDecision invalid = filter.update(unresolved);
  EXPECT_FALSE(invalid.stable);
  EXPECT_TRUE(invalid.pending);
  EXPECT_EQ(invalid.code, 0U);
  EXPECT_EQ(invalid.reason, "invalid_raw_evidence");

  // A subsequent CLEAR must re-confirm from scratch, not resume the prior one.
  EXPECT_FALSE(filter.update(clear(1.6)).stable);
  EXPECT_TRUE(filter.update(clear(1.8)).stable);
}

TEST(CargoSafetyTemporalFilter, H1UnresolvedFrameDoesNotLockOutRecovery) {
  CargoSafetyTemporalFilter filter;
  CargoSafetyTemporalInput unresolved;
  unresolved.stamp_sec = 1.0;
  unresolved.raw_valid = false;
  unresolved.raw_code = 0U;
  EXPECT_FALSE(filter.update(unresolved).stable);

  // The immediately following valid hazard frames confirm normally.
  EXPECT_FALSE(filter.update(hazard(1.2, 0.00F)).stable);
  EXPECT_FALSE(filter.update(hazard(1.4, 0.05F)).stable);
  const CargoSafetyTemporalDecision confirmed =
      filter.update(hazard(1.6, 0.08F));
  EXPECT_TRUE(confirmed.stable);
  EXPECT_EQ(confirmed.code, 17U);
}

}  // namespace
}  // namespace ndt_slam
