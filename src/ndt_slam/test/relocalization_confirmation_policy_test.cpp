#include "ndt_slam/relocalization_confirmation_policy.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <Eigen/Geometry>

namespace ndt_slam {
namespace {

RelocalizationConfirmationInput candidate(
    std::uint64_t frame, double stamp, double x = 1.0,
    double yaw_deg = 0.0) {
  RelocalizationConfirmationInput input;
  input.current_frame_index = frame + 1U;
  input.current_stamp_sec = stamp + 0.1;
  input.expected_map_generation = 3U;
  input.expected_pose_version = 4U;
  input.result.valid = true;
  input.result.frame_index = frame;
  input.result.map_generation = 3U;
  input.result.pose_version = 4U;
  input.result.stamp_sec = stamp;
  input.result.fitness = 0.1;
  input.result.probability = 0.5;
  input.result.reference_pose = Sophus::SE3d();
  input.result.pose = Sophus::SE3d(
      Eigen::AngleAxisd(
          yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix(),
      Eigen::Vector3d(x, 0.0, 0.0));
  return input;
}

TEST(RelocalizationConfirmationPolicy, RejectsWrongMapIdentity) {
  auto input = candidate(10U, 1.0);
  input.result.map_generation = 2U;
  const auto decision = evaluateRelocalizationConfirmation(input);
  EXPECT_EQ(
      decision.outcome,
      RelocalizationConfirmationOutcome::DISCARD_IDENTITY);
  EXPECT_FALSE(decision.update_last_result_frame);
}

TEST(RelocalizationConfirmationPolicy, RejectsFutureAndStaleResults) {
  auto future = candidate(10U, 1.0);
  future.current_frame_index = 9U;
  EXPECT_EQ(
      evaluateRelocalizationConfirmation(future).outcome,
      RelocalizationConfirmationOutcome::DISCARD_STALE);

  auto stale = candidate(10U, 1.0);
  stale.current_frame_index = 30U;
  stale.current_stamp_sec = 3.0;
  EXPECT_EQ(
      evaluateRelocalizationConfirmation(stale).outcome,
      RelocalizationConfirmationOutcome::DISCARD_STALE);
}

TEST(RelocalizationConfirmationPolicy, RequiresConsistentCorrections) {
  const auto first = evaluateRelocalizationConfirmation(
      candidate(10U, 1.0));
  ASSERT_EQ(
      first.outcome,
      RelocalizationConfirmationOutcome::CONFIRMING);

  auto second_input = candidate(11U, 1.1, 1.1);
  second_input.last_result_frame = 10U;
  second_input.previous_confirmation_count = first.confirmation_count;
  second_input.previous_correction = first.correction;
  const auto second =
      evaluateRelocalizationConfirmation(second_input);
  EXPECT_EQ(
      second.outcome,
      RelocalizationConfirmationOutcome::CONFIRMED);
  EXPECT_EQ(second.confirmation_count, 2);
}

TEST(RelocalizationConfirmationPolicy, InconsistentResultRestartsCount) {
  const auto first = evaluateRelocalizationConfirmation(
      candidate(10U, 1.0));
  auto second_input = candidate(11U, 1.1, 3.0);
  second_input.last_result_frame = 10U;
  second_input.previous_confirmation_count = first.confirmation_count;
  second_input.previous_correction = first.correction;
  const auto second =
      evaluateRelocalizationConfirmation(second_input);
  EXPECT_EQ(
      second.outcome,
      RelocalizationConfirmationOutcome::CONFIRMING);
  EXPECT_EQ(second.confirmation_count, 1);
}

TEST(RelocalizationConfirmationPolicy, InvalidResultNeverConfirms) {
  auto input = candidate(10U, 1.0);
  input.result.valid = false;
  input.result.reason = "no_candidate_passed_gates";
  const auto decision = evaluateRelocalizationConfirmation(input);
  EXPECT_EQ(
      decision.outcome,
      RelocalizationConfirmationOutcome::DISCARD_INVALID);
  EXPECT_EQ(decision.reason, "no_candidate_passed_gates");
}

TEST(RelocalizationConfirmationPolicy, DuplicateResultDoesNotAdvanceState) {
  auto input = candidate(10U, 1.0);
  input.last_result_frame = 10U;
  input.previous_confirmation_count = 1;
  const auto decision = evaluateRelocalizationConfirmation(input);
  EXPECT_EQ(
      decision.outcome,
      RelocalizationConfirmationOutcome::DISCARD_DUPLICATE);
  EXPECT_FALSE(decision.update_last_result_frame);
  EXPECT_EQ(decision.confirmation_count, 0);
}

TEST(RelocalizationConfirmationPolicy, NonfiniteQualityNeverConfirms) {
  auto input = candidate(10U, 1.0);
  input.result.fitness =
      std::numeric_limits<double>::quiet_NaN();
  const auto decision = evaluateRelocalizationConfirmation(input);
  EXPECT_EQ(
      decision.outcome,
      RelocalizationConfirmationOutcome::DISCARD_INVALID);
  EXPECT_EQ(
      decision.reason, "nonfinite_relocalization_candidate");
}

TEST(RelocalizationConfirmationPolicy,
     RailConfirmationNeverConsumesFreeYawDifference) {
  const auto first = evaluateRailRelocalizationConfirmation(
      candidate(10U, 1.0, 1.0, -45.0));
  ASSERT_EQ(first.outcome,
            RelocalizationConfirmationOutcome::CONFIRMING);
  auto second_input = candidate(11U, 1.1, 1.1, 70.0);
  second_input.last_result_frame = 10U;
  second_input.previous_confirmation_count = first.confirmation_count;
  second_input.previous_correction = first.correction;
  const auto second = evaluateRailRelocalizationConfirmation(second_input);
  EXPECT_EQ(second.outcome,
            RelocalizationConfirmationOutcome::CONFIRMED);
  EXPECT_NEAR(second.correction.so3().log().norm(), 0.0, 1.0e-12);
}

}  // namespace
}  // namespace ndt_slam
