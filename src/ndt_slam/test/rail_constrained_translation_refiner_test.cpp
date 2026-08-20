#include "ndt_slam/crane_pose_constraint.hpp"
#include "ndt_slam/rail_constrained_translation_refiner.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

namespace ndt_slam {
namespace {

Sophus::SE3d makePose(double x, double y, double yaw) {
  return Sophus::SE3d(
      Sophus::SO3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                       .toRotationMatrix()),
      Eigen::Vector3d(x, y, 0.5));
}

TEST(RailConstrainedTranslationRefinerTest, KeepsConfiguredYawAndRefinesXY) {
  RailTranslationRefinerConfig config;
  config.enabled = true;
  config.configured_yaw_rad = 0.25;
  config.initial_step_m = 0.5;
  config.minimum_step_m = 0.01;
  config.trust_radius_m = 3.0;
  config.deadline_ms = 1000.0;
  config.maximum_evaluations = 500U;
  RailConstrainedTranslationRefiner refiner(config);
  RailTranslationRefinerInput input;
  input.free_pose = makePose(0.0, 0.0, 0.8);
  input.predicted_pose = makePose(0.2, 0.2, 0.1);
  input.captured_target_version = 12U;
  input.current_target_version = 12U;
  const auto objective = [](const Sophus::SE3d& pose) {
    const Eigen::Vector2d error =
        pose.translation().head<2>() - Eigen::Vector2d(1.0, -0.5);
    return error.squaredNorm();
  };
  const auto result = refiner.refine(input, objective);
  ASSERT_TRUE(result.valid) << result.reason;
  const auto rpy = cranePoseRpy(result.rail_pose.so3());
  ASSERT_TRUE(rpy.valid);
  EXPECT_NEAR(rpy.yaw, config.configured_yaw_rad, 1.0e-9);
  EXPECT_NEAR(result.rail_pose.translation().x(), 1.0, 0.03);
  EXPECT_NEAR(result.rail_pose.translation().y(), -0.5, 0.03);
  EXPECT_NEAR(result.free_fitness, 1.25, 1.0e-9);
}

TEST(RailConstrainedTranslationRefinerTest, RejectsStaleTargetVersion) {
  RailTranslationRefinerConfig config;
  config.enabled = true;
  config.configured_yaw_rad = 0.0;
  RailConstrainedTranslationRefiner refiner(config);
  RailTranslationRefinerInput input;
  input.free_pose = makePose(0.0, 0.0, 0.0);
  input.predicted_pose = input.free_pose;
  input.captured_target_version = 1U;
  input.current_target_version = 2U;
  const auto result = refiner.refine(
      input, [](const Sophus::SE3d&) { return 0.0; });
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "target_version_mismatch");
}

TEST(RailConstrainedTranslationRefinerTest, RejectsMissingObjective) {
  RailTranslationRefinerConfig config;
  config.enabled = true;
  config.configured_yaw_rad = 0.0;
  RailConstrainedTranslationRefiner refiner(config);
  RailTranslationRefinerInput input;
  input.free_pose = makePose(0.0, 0.0, 0.0);
  input.predicted_pose = input.free_pose;
  const auto result = refiner.refine(input, {});
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "objective_missing");
}

TEST(RailConstrainedTranslationRefinerTest, RejectsNonfiniteYaw) {
  RailTranslationRefinerConfig config;
  config.enabled = true;
  RailConstrainedTranslationRefiner refiner(config);
  RailTranslationRefinerInput input;
  input.free_pose = makePose(0.0, 0.0, 0.0);
  input.predicted_pose = input.free_pose;
  const auto result = refiner.refine(
      input, [](const Sophus::SE3d&) { return 0.0; });
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "nonfinite_configuration_or_pose");
}

TEST(RailConstrainedTranslationRefinerTest, EvaluationsAreBounded) {
  RailTranslationRefinerConfig config;
  config.enabled = true;
  config.configured_yaw_rad = 0.0;
  config.maximum_evaluations = 3U;
  config.deadline_ms = 1000.0;
  RailConstrainedTranslationRefiner refiner(config);
  RailTranslationRefinerInput input;
  input.free_pose = makePose(0.0, 0.0, 0.0);
  input.predicted_pose = input.free_pose;
  const auto result = refiner.refine(
      input, [](const Sophus::SE3d& pose) {
        return pose.translation().head<2>().squaredNorm();
      });
  EXPECT_LE(result.evaluations, config.maximum_evaluations);
}

}  // namespace
}  // namespace ndt_slam
