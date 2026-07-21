#include "ndt_slam/cargo_lift_origin_binder.hpp"

#include <gtest/gtest.h>

namespace ndt_slam {
namespace {

CargoOriginCandidate approvedCandidate() {
  CargoOriginCandidate candidate;
  candidate.component_id = 7U;
  candidate.source =
      CargoOriginCandidateSource::OPERATOR_APPROVED_BASELINE;
  candidate.authority =
      StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE;
  candidate.center_map = Eigen::Vector2f(1.0F, 2.0F);
  candidate.length_m = 4.0F;
  candidate.width_m = 1.6F;
  candidate.top_z95_map = 1.5F;
  candidate.support_z_map = 0.0F;
  candidate.uncertainty_m = 0.05F;
  candidate.point_count = 1000U;
  return candidate;
}

CargoLiftOriginInput loadedInput(double stamp) {
  CargoLiftOriginInput input;
  input.stamp_sec = stamp;
  input.hook_signal_valid = true;
  input.hook_loaded = true;
  input.anchor_valid = true;
  input.hook_anchor_map = Eigen::Vector2f(1.1F, 2.0F);
  input.candidates = {approvedCandidate()};
  input.current_top_valid = true;
  input.current_top_stamp_sec = stamp;
  input.current_top_z_map = 2.0F;
  input.current_top_uncertainty_m = 0.02F;
  input.source_coverage = 0.8F;
  input.revealed_support_valid = true;
  input.revealed_support_stamp_sec = stamp;
  input.revealed_support_z_map = 0.0F;
  input.revealed_support_coverage = 0.8F;
  return input;
}

TEST(CargoLiftOriginBinderTest, BindsLocalApprovedOriginAndConfirmsLift) {
  CargoLiftOriginConfig config;
  config.lift_confirm_frames = 2;
  config.thickness_confirm_frames = 2;
  CargoLiftOriginBinder binder(config);
  auto first = loadedInput(1.0);
  first.hook_was_empty = true;
  auto result = binder.update(first);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.origin.component_id, 7U);

  result = binder.update(loadedInput(1.1));
  EXPECT_TRUE(result.lift_confirmed);
  EXPECT_FALSE(result.thickness_ready);
  result = binder.update(loadedInput(1.2));
  EXPECT_TRUE(result.thickness_ready);
  EXPECT_EQ(result.state, CargoLiftEventState::GEOMETRY_FROZEN);
}

TEST(CargoLiftOriginBinderTest, NoCoverageIsNotDisappearanceEvidence) {
  CargoLiftOriginConfig config;
  config.lift_confirm_frames = 1;
  CargoLiftOriginBinder binder(config);
  auto input = loadedInput(1.0);
  input.hook_was_empty = true;
  input.source_coverage = 0.0F;
  input.revealed_support_coverage = 0.0F;
  const auto result = binder.update(input);
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.lift_confirmed);
  EXPECT_EQ(result.lift_confirm_count, 0);
}

TEST(CargoLiftOriginBinderTest, LoadedStartupReacquiresWithoutEmptyEdge) {
  CargoLiftOriginBinder binder;
  auto input = loadedInput(1.0);
  input.node_started_loaded = true;
  input.hook_was_empty = false;
  const auto result = binder.update(input);
  EXPECT_TRUE(result.valid);
  EXPECT_NE(result.reason, "waiting_for_preload_baseline");
}

TEST(CargoLiftOriginBinderTest, UnverifiedStaticCannotBindFormalOrigin) {
  CargoLiftOriginBinder binder;
  auto input = loadedInput(1.0);
  input.hook_was_empty = true;
  input.candidates.front().authority =
      StaticEvidenceAuthority::UNVERIFIED_LOADED_CLEAN;
  const auto result = binder.update(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "no_local_authorized_origin_candidate");
}

TEST(CargoLiftOriginBinderTest, InvalidFrameBreaksThicknessConfirmation) {
  CargoLiftOriginConfig config;
  config.lift_confirm_frames = 1;
  config.thickness_confirm_frames = 2;
  CargoLiftOriginBinder binder(config);
  auto first = loadedInput(1.0);
  first.hook_was_empty = true;
  ASSERT_EQ(binder.update(first).thickness_confirm_count, 1);
  auto invalid = loadedInput(1.1);
  invalid.revealed_support_valid = false;
  EXPECT_EQ(binder.update(invalid).thickness_confirm_count, 0);
  const auto restarted = binder.update(loadedInput(1.2));
  EXPECT_EQ(restarted.thickness_confirm_count, 1);
  EXPECT_FALSE(restarted.thickness_ready);
}

TEST(CargoLiftOriginBinderTest, ObservationGapBreaksLiftConfirmation) {
  CargoLiftOriginConfig config;
  config.lift_confirm_frames = 2;
  config.maximum_observation_gap_sec = 0.5;
  CargoLiftOriginBinder binder(config);
  auto first = loadedInput(1.0);
  first.hook_was_empty = true;
  EXPECT_EQ(binder.update(first).lift_confirm_count, 1);
  const auto after_gap = binder.update(loadedInput(2.0));
  EXPECT_EQ(after_gap.lift_confirm_count, 1);
  EXPECT_FALSE(after_gap.lift_confirmed);
}

TEST(CargoLiftOriginBinderTest, DuplicateStampDoesNotAdvanceConfirmation) {
  CargoLiftOriginConfig config;
  config.lift_confirm_frames = 2;
  CargoLiftOriginBinder binder(config);
  auto first = loadedInput(1.0);
  first.hook_was_empty = true;
  EXPECT_EQ(binder.update(first).lift_confirm_count, 1);
  const auto duplicate = binder.update(loadedInput(1.0));
  EXPECT_FALSE(duplicate.valid);
  EXPECT_EQ(duplicate.reason, "source_time_invalid_or_rollback");
}

TEST(CargoLiftOriginBinderTest, StaleSourceDoesNotAdvanceConfirmation) {
  CargoLiftOriginConfig config;
  config.lift_confirm_frames = 1;
  CargoLiftOriginBinder binder(config);
  auto input = loadedInput(1.0);
  input.hook_was_empty = true;
  input.current_top_stamp_sec = 0.1;
  const auto result = binder.update(input);
  EXPECT_EQ(result.lift_confirm_count, 0);
  EXPECT_FALSE(result.lift_confirmed);
}

TEST(CargoLiftOriginBinderTest, OriginIdentityChangeResetsConfirmation) {
  CargoLiftOriginConfig config;
  config.lift_confirm_frames = 2;
  CargoLiftOriginBinder binder(config);
  auto first = loadedInput(1.0);
  first.hook_was_empty = true;
  EXPECT_EQ(binder.update(first).lift_confirm_count, 1);
  auto changed = loadedInput(1.1);
  changed.candidates.front().component_id = 8U;
  const auto result = binder.update(changed);
  EXPECT_EQ(result.origin.component_id, 8U);
  EXPECT_EQ(result.lift_confirm_count, 1);
  EXPECT_FALSE(result.lift_confirmed);
}

}  // namespace
}  // namespace ndt_slam
