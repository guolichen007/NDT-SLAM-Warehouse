#include "ndt_slam/rail_constrained_translation_refiner.hpp"

#include "ndt_slam/crane_pose_constraint.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

namespace ndt_slam {
namespace {

Sophus::SE3d poseWithYawAndTranslation(const Sophus::SE3d& source,
                                       double yaw,
                                       const Eigen::Vector2d& xy) {
  const CranePoseRpy rpy = cranePoseRpy(source.so3());
  if (!rpy.valid || !std::isfinite(yaw) || !xy.allFinite()) {
    return Sophus::SE3d();
  }
  Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy.pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy.roll, Eigen::Vector3d::UnitX());
  rotation.normalize();
  Eigen::Vector3d translation = source.translation();
  translation.x() = xy.x();
  translation.y() = xy.y();
  return Sophus::SE3d(Sophus::SO3d(rotation.toRotationMatrix()), translation);
}

}  // namespace

RailConstrainedTranslationRefiner::RailConstrainedTranslationRefiner(
    RailTranslationRefinerConfig config)
    : config_(std::move(config)) {}

void RailConstrainedTranslationRefiner::configure(
    const RailTranslationRefinerConfig& config) {
  config_ = config;
}

RailTranslationRefinerResult RailConstrainedTranslationRefiner::refine(
    const RailTranslationRefinerInput& input,
    const RailPoseObjective& objective) const {
  RailTranslationRefinerResult result;
  result.free_pose = input.free_pose;
  result.rail_pose = input.free_pose;
  result.free_ndt_reported_fitness = input.free_ndt_reported_fitness;
  const auto started = std::chrono::steady_clock::now();
  const auto finish = [&result, &started]() {
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
  };

  if (!config_.enabled) {
    result.reason = "disabled";
    finish();
    return result;
  }
  if (!config_.shadow_only) {
    result.reason = "PRODUCT_MODE_NOT_IMPLEMENTED_IN_SHADOW_BUILD";
    finish();
    return result;
  }
  if (!std::isfinite(config_.configured_yaw_rad) ||
      !input.free_pose.translation().allFinite() ||
      !input.free_pose.so3().matrix().allFinite() ||
      !input.predicted_pose.translation().allFinite() ||
      !input.predicted_pose.so3().matrix().allFinite()) {
    result.reason = "nonfinite_configuration_or_pose";
    finish();
    return result;
  }
  if (input.captured_target_version != input.current_target_version) {
    result.reason = "target_version_mismatch";
    finish();
    return result;
  }
  if (!objective) {
    result.reason = "objective_missing";
    finish();
    return result;
  }

  const double initial_step = std::max(
      config_.minimum_step_m, config_.initial_step_m);
  const double minimum_step = std::max(1.0e-6, config_.minimum_step_m);
  const double trust_radius = std::max(initial_step, config_.trust_radius_m);
  const double deadline_ms = std::max(0.1, config_.deadline_ms);
  const std::size_t maximum_evaluations =
      std::max<std::size_t>(1U, config_.maximum_evaluations);
  const Eigen::Vector2d trust_center =
      input.predicted_pose.translation().head<2>();

  const auto deadlineReached = [&]() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - started).count() >
           deadline_ms;
  };
  const auto evaluate = [&](const Sophus::SE3d& pose, double* score) {
    if (result.evaluations >= maximum_evaluations || deadlineReached()) {
      return false;
    }
    ++result.evaluations;
    *score = objective(pose);
    return std::isfinite(*score);
  };

  if (!evaluate(input.free_pose, &result.free_fitness)) {
    result.reason = deadlineReached() ? "deadline_before_free_score"
                                      : "free_score_invalid";
    finish();
    return result;
  }

  struct Candidate {
    Sophus::SE3d pose;
    double score = std::numeric_limits<double>::infinity();
  };
  Candidate global_best;
  const std::array<Eigen::Vector2d, 2> seeds{{
      input.free_pose.translation().head<2>(),
      input.predicted_pose.translation().head<2>()}};
  const std::array<Eigen::Vector2d, 8> directions{{
      {1.0, 0.0}, {-1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0},
      {1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0}}};

  for (const auto& seed : seeds) {
    Eigen::Vector2d center = seed;
    const Eigen::Vector2d from_trust_center = center - trust_center;
    if (from_trust_center.norm() > trust_radius) {
      center = trust_center +
          from_trust_center.normalized() * trust_radius;
    }
    Candidate local_best;
    local_best.pose = poseWithYawAndTranslation(
        input.free_pose, config_.configured_yaw_rad, center);
    if (!evaluate(local_best.pose, &local_best.score)) continue;

    double step = initial_step;
    while (step >= minimum_step &&
           result.evaluations < maximum_evaluations &&
           !deadlineReached()) {
      bool improved = false;
      Candidate iteration_best = local_best;
      for (const auto& direction : directions) {
        Eigen::Vector2d candidate_xy =
            local_best.pose.translation().head<2>() +
            step * direction.normalized();
        const Eigen::Vector2d offset = candidate_xy - trust_center;
        if (offset.norm() > trust_radius) continue;
        Candidate candidate;
        candidate.pose = poseWithYawAndTranslation(
            input.free_pose, config_.configured_yaw_rad, candidate_xy);
        if (!evaluate(candidate.pose, &candidate.score)) continue;
        if (candidate.score < iteration_best.score) {
          iteration_best = candidate;
          improved = true;
        }
      }
      if (improved) {
        local_best = iteration_best;
      } else {
        step *= 0.5;
      }
    }
    if (local_best.score < global_best.score) global_best = local_best;
  }

  if (deadlineReached()) {
    result.reason = "deadline_exceeded";
    finish();
    return result;
  }
  if (!std::isfinite(global_best.score) ||
      !global_best.pose.translation().allFinite() ||
      !global_best.pose.so3().matrix().allFinite()) {
    result.reason = "rail_score_invalid";
    finish();
    return result;
  }

  result.rail_pose = global_best.pose;
  result.rail_fitness = global_best.score;
  result.translation_delta_m =
      (result.rail_pose.translation().head<2>() -
       input.free_pose.translation().head<2>()).norm();
  result.fitness_delta = result.rail_fitness - result.free_fitness;
  result.valid = true;
  result.reason = "shadow_refinement_valid";
  finish();
  return result;
}

}  // namespace ndt_slam
