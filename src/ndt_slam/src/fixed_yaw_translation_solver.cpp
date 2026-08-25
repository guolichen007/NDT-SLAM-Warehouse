#include "ndt_slam/fixed_yaw_translation_solver.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

constexpr double kEpsilon = 1.0e-12;

FixedYawTranslationSolverConfig sanitize(
    FixedYawTranslationSolverConfig config) {
  config.maximum_iterations = std::max(1, config.maximum_iterations);
  config.target_normal_neighbor_count = std::max(
      3, config.target_normal_neighbor_count);
  config.minimum_inliers = std::max<std::size_t>(3U, config.minimum_inliers);
  config.maximum_correspondence_distance_m = std::max(
      1.0e-3, config.maximum_correspondence_distance_m);
  config.convergence_translation_m = std::max(
      1.0e-9, config.convergence_translation_m);
  return config;
}

Eigen::Matrix3d fixedYawRotation(double yaw) {
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation(0, 0) = cosine;
  rotation(0, 1) = -sine;
  rotation(1, 0) = sine;
  rotation(1, 1) = cosine;
  return rotation;
}

}  // namespace

const char* fixedYawSeedSourceName(FixedYawSeedSource source) noexcept {
  switch (source) {
    case FixedYawSeedSource::EKF_PREDICTION: return "EKF_PREDICTION";
    case FixedYawSeedSource::FREE_NDT_BASIN: return "FREE_NDT_BASIN";
    case FixedYawSeedSource::RELOCALIZATION_BASIN:
      return "RELOCALIZATION_BASIN";
    case FixedYawSeedSource::LOOP_PAIR: return "LOOP_PAIR";
  }
  return "INVALID";
}

FixedYawTranslationSolver::FixedYawTranslationSolver(
    const FixedYawTranslationSolverConfig& config)
    : config_(sanitize(config)) {}

void FixedYawTranslationSolver::setConfig(
    const FixedYawTranslationSolverConfig& config) {
  config_ = sanitize(config);
  resetTargetCache();
}

void FixedYawTranslationSolver::resetTargetCache() {
  cached_target_ = RegistrationTargetSnapshot{};
  target_normals_.clear();
  target_normal_valid_.clear();
}

bool FixedYawTranslationSolver::ensureTargetCache(
    const RegistrationTargetSnapshot& target, double* build_ms) {
  if (build_ms) *build_ms = 0.0;
  if (!target.valid()) return false;
  if (cached_target_.target_snapshot_id == target.target_snapshot_id &&
      target_normals_.size() == target.cloud->size()) {
    return true;
  }
  const auto start = std::chrono::steady_clock::now();
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target.cloud);
  target_normals_.assign(target.cloud->size(), Eigen::Vector2d::Zero());
  target_normal_valid_.assign(target.cloud->size(), false);
  const int neighbors = std::min<int>(
      config_.target_normal_neighbor_count,
      static_cast<int>(target.cloud->size()));
  std::vector<int> indices(static_cast<std::size_t>(neighbors));
  std::vector<float> distances(static_cast<std::size_t>(neighbors));
  for (std::size_t index = 0U; index < target.cloud->size(); ++index) {
    const pcl::PointXYZ& query = target.cloud->points[index];
    if (!std::isfinite(query.x) || !std::isfinite(query.y)) continue;
    const int found = tree.nearestKSearch(
        query, neighbors, indices, distances);
    if (found < 3) continue;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (int neighbor = 0; neighbor < found; ++neighbor) {
      const auto& point = target.cloud->points[
          static_cast<std::size_t>(indices[neighbor])];
      center += Eigen::Vector2d(point.x, point.y);
    }
    center /= static_cast<double>(found);
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (int neighbor = 0; neighbor < found; ++neighbor) {
      const auto& point = target.cloud->points[
          static_cast<std::size_t>(indices[neighbor])];
      const Eigen::Vector2d delta(point.x - center.x(),
                                  point.y - center.y());
      covariance += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(covariance);
    if (eigensolver.info() != Eigen::Success) continue;
    const Eigen::Vector2d normal = eigensolver.eigenvectors().col(0);
    if (!normal.allFinite() || normal.norm() <= kEpsilon) continue;
    target_normals_[index] = normal.normalized();
    target_normal_valid_[index] = true;
  }
  cached_target_ = target;
  if (build_ms) {
    *build_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
  }
  return true;
}

FixedYawTranslationResult FixedYawTranslationSolver::solve(
    const FixedYawTranslationInput& input) {
  FixedYawTranslationResult result;
  result.seed_source = input.seed_source;
  result.target_snapshot_id = input.target.target_snapshot_id;
  const auto start = std::chrono::steady_clock::now();
  if (!input.source_cloud_base || input.source_cloud_base->empty() ||
      !input.target.valid() || !std::isfinite(input.authoritative_yaw_rad) ||
      !input.seed_pose_map_base.translation().allFinite()) {
    result.reason = "invalid_fixed_yaw_solver_input";
    return result;
  }
  if (!ensureTargetCache(input.target,
                         &result.target_normal_cache_build_ms)) {
    result.reason = "target_normal_cache_unavailable";
    return result;
  }
  result.target_normal_cache_rebuilt =
      result.target_normal_cache_build_ms > 0.0;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(input.target.cloud);
  const Eigen::Matrix3d rotation = fixedYawRotation(
      input.authoritative_yaw_rad);
  Eigen::Vector3d translation = input.seed_pose_map_base.translation();
  const double maximum_distance_squared =
      config_.maximum_correspondence_distance_m *
      config_.maximum_correspondence_distance_m;
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_distance(1);

  for (int iteration = 0; iteration < config_.maximum_iterations;
       ++iteration) {
    Eigen::Matrix2d hessian = Eigen::Matrix2d::Zero();
    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
    double squared_residual_sum = 0.0;
    double squared_correspondence_sum = 0.0;
    std::size_t inliers = 0U;
    for (const pcl::PointXYZ& source : input.source_cloud_base->points) {
      if (!std::isfinite(source.x) || !std::isfinite(source.y) ||
          !std::isfinite(source.z)) {
        continue;
      }
      const Eigen::Vector3d source_vector(source.x, source.y, source.z);
      const Eigen::Vector3d transformed = rotation * source_vector +
          translation;
      pcl::PointXYZ query;
      query.x = static_cast<float>(transformed.x());
      query.y = static_cast<float>(transformed.y());
      query.z = static_cast<float>(transformed.z());
      if (tree.nearestKSearch(query, 1, nearest_index, nearest_distance) != 1 ||
          nearest_distance.front() > maximum_distance_squared) {
        continue;
      }
      const std::size_t target_index = static_cast<std::size_t>(
          nearest_index.front());
      if (!target_normal_valid_[target_index]) continue;
      const auto& target = input.target.cloud->points[target_index];
      const Eigen::Vector2d normal = target_normals_[target_index];
      const Eigen::Vector2d delta(transformed.x() - target.x,
                                  transformed.y() - target.y);
      const double residual = normal.dot(delta);
      hessian += normal * normal.transpose();
      gradient += normal * residual;
      squared_residual_sum += residual * residual;
      // PCL's NDT fitness is a mean squared nearest-target distance.  Keep
      // the point-to-normal residual for the optimizer, but expose the same
      // physical scale to the existing fitness circuit breaker.
      squared_correspondence_sum += nearest_distance.front();
      ++inliers;
    }
    result.iterations = iteration + 1;
    result.inliers = inliers;
    result.hessian = hessian;
    result.residual = inliers > 0U
        ? std::sqrt(squared_residual_sum / static_cast<double>(inliers))
        : std::numeric_limits<double>::infinity();
    result.fitness = inliers > 0U
        ? squared_correspondence_sum / static_cast<double>(inliers)
        : std::numeric_limits<double>::infinity();
    if (inliers < config_.minimum_inliers || !hessian.allFinite() ||
        std::abs(hessian.determinant()) <= kEpsilon) {
      result.reason = "fixed_yaw_translation_degenerate";
      break;
    }
    const Eigen::Vector2d step = -hessian.ldlt().solve(gradient);
    if (!step.allFinite()) {
      result.reason = "fixed_yaw_translation_nonfinite_step";
      break;
    }
    translation.head<2>() += step;
    if (step.norm() <= config_.convergence_translation_m) {
      result.valid = true;
      result.reason = "fixed_yaw_translation_converged";
      break;
    }
    if (iteration + 1 == config_.maximum_iterations) {
      result.valid = true;
      result.reason = "fixed_yaw_translation_iteration_bound";
    }
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(result.hessian);
  if (eigensolver.info() == Eigen::Success) {
    result.weak_eigenvalue = eigensolver.eigenvalues()[0];
    result.strong_eigenvalue = eigensolver.eigenvalues()[1];
    result.weak_direction = eigensolver.eigenvectors().col(0);
    result.strong_direction = eigensolver.eigenvectors().col(1);
    result.condition = result.weak_eigenvalue > kEpsilon
        ? result.strong_eigenvalue / result.weak_eigenvalue
        : std::numeric_limits<double>::infinity();
  }
  result.xy = translation.head<2>();
  result.pose_map_base = Sophus::SE3d(
      Sophus::SO3d(rotation), translation);
  // Evaluate the authoritative rail pose once after optimization. This is a
  // single health score at the final pose, never an XY grid search.
  const auto fitness_start = std::chrono::steady_clock::now();
  double final_squared_distance_sum = 0.0;
  std::size_t final_correspondences = 0U;
  for (const pcl::PointXYZ& source : input.source_cloud_base->points) {
    if (!std::isfinite(source.x) || !std::isfinite(source.y) ||
        !std::isfinite(source.z)) {
      continue;
    }
    const Eigen::Vector3d transformed = rotation *
        Eigen::Vector3d(source.x, source.y, source.z) + translation;
    pcl::PointXYZ query;
    query.x = static_cast<float>(transformed.x());
    query.y = static_cast<float>(transformed.y());
    query.z = static_cast<float>(transformed.z());
    if (tree.nearestKSearch(query, 1, nearest_index, nearest_distance) == 1 &&
        nearest_distance.front() <= maximum_distance_squared) {
      final_squared_distance_sum += nearest_distance.front();
      ++final_correspondences;
    }
  }
  result.fitness = final_correspondences > 0U
      ? final_squared_distance_sum /
            static_cast<double>(final_correspondences)
      : std::numeric_limits<double>::infinity();
  result.fitness_elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - fitness_start).count();
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return result;
}

FixedYawDualSeedDecision selectFixedYawDualSeed(
    const FixedYawTranslationResult& predicted,
    const FixedYawTranslationResult& free_ndt,
    double maximum_consistent_translation_m) {
  FixedYawDualSeedDecision decision;
  if (predicted.valid && free_ndt.valid) {
    if ((predicted.xy - free_ndt.xy).norm() >
        std::max(0.0, maximum_consistent_translation_m)) {
      decision.outcome = FixedYawDualSeedOutcome::SEED_BASIN_AMBIGUOUS;
      decision.reason = "fixed_yaw_seed_basin_ambiguous";
      return decision;
    }
    decision.selected = predicted.fitness <= free_ndt.fitness
        ? predicted : free_ndt;
    decision.outcome = FixedYawDualSeedOutcome::CONSISTENT_BEST_SELECTED;
    decision.authoritative_measurement_valid = true;
    decision.reason = "consistent_fixed_yaw_seed_selected";
    return decision;
  }
  if (predicted.valid) {
    decision.selected = predicted;
    decision.outcome = FixedYawDualSeedOutcome::PREDICTED_SEED_SELECTED;
    decision.authoritative_measurement_valid = true;
    decision.reason = "predicted_fixed_yaw_seed_selected";
    return decision;
  }
  if (free_ndt.valid) {
    decision.selected = free_ndt;
    decision.outcome = FixedYawDualSeedOutcome::FREE_SEED_RELOCALIZATION_ONLY;
    decision.reason = "free_ndt_seed_requires_relocalization_confirmation";
    return decision;
  }
  decision.reason = "no_valid_fixed_yaw_seed";
  return decision;
}

}  // namespace ndt_slam
