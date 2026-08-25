#pragma once

#include "ndt_slam/registration_target_snapshot.hpp"

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ndt_slam {

enum class FixedYawSeedSource : std::uint8_t {
  EKF_PREDICTION = 0,
  FREE_NDT_BASIN,
  RELOCALIZATION_BASIN,
  LOOP_PAIR,
};

const char* fixedYawSeedSourceName(FixedYawSeedSource source) noexcept;

struct FixedYawTranslationSolverConfig {
  int maximum_iterations = 10;
  int target_normal_neighbor_count = 10;
  std::size_t minimum_inliers = 30U;
  double maximum_correspondence_distance_m = 1.50;
  double convergence_translation_m = 1.0e-4;
};

struct FixedYawTranslationInput {
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr source_cloud_base;
  RegistrationTargetSnapshot target;
  Sophus::SE3d seed_pose_map_base;
  double authoritative_yaw_rad = 0.0;
  FixedYawSeedSource seed_source = FixedYawSeedSource::EKF_PREDICTION;
};

struct FixedYawTranslationResult {
  bool valid = false;
  Sophus::SE3d pose_map_base;
  Eigen::Vector2d xy = Eigen::Vector2d::Zero();
  double residual = 0.0;
  double fitness = 0.0;
  std::size_t inliers = 0U;
  Eigen::Matrix2d hessian = Eigen::Matrix2d::Zero();
  double strong_eigenvalue = 0.0;
  double weak_eigenvalue = 0.0;
  double condition = 0.0;
  Eigen::Vector2d strong_direction = Eigen::Vector2d::UnitX();
  Eigen::Vector2d weak_direction = Eigen::Vector2d::UnitY();
  int iterations = 0;
  double elapsed_ms = 0.0;
  double fitness_elapsed_ms = 0.0;
  double target_normal_cache_build_ms = 0.0;
  bool target_normal_cache_rebuilt = false;
  std::uint64_t target_snapshot_id = 0U;
  FixedYawSeedSource seed_source = FixedYawSeedSource::EKF_PREDICTION;
  std::string reason = "not_evaluated";
};

class FixedYawTranslationSolver {
 public:
  explicit FixedYawTranslationSolver(
      const FixedYawTranslationSolverConfig& config =
          FixedYawTranslationSolverConfig{});

  void setConfig(const FixedYawTranslationSolverConfig& config);
  const FixedYawTranslationSolverConfig& config() const noexcept {
    return config_;
  }
  void resetTargetCache();
  FixedYawTranslationResult solve(const FixedYawTranslationInput& input);

 private:
  bool ensureTargetCache(const RegistrationTargetSnapshot& target,
                         double* build_ms);

  FixedYawTranslationSolverConfig config_;
  RegistrationTargetSnapshot cached_target_;
  std::vector<Eigen::Vector2d> target_normals_;
  std::vector<bool> target_normal_valid_;
};

enum class FixedYawDualSeedOutcome : std::uint8_t {
  NO_AUTHORITATIVE_MEASUREMENT = 0,
  PREDICTED_SEED_SELECTED,
  CONSISTENT_BEST_SELECTED,
  FREE_SEED_RELOCALIZATION_ONLY,
  SEED_BASIN_AMBIGUOUS,
};

struct FixedYawDualSeedDecision {
  FixedYawDualSeedOutcome outcome =
      FixedYawDualSeedOutcome::NO_AUTHORITATIVE_MEASUREMENT;
  bool authoritative_measurement_valid = false;
  FixedYawTranslationResult selected;
  std::string reason = "not_evaluated";
};

FixedYawDualSeedDecision selectFixedYawDualSeed(
    const FixedYawTranslationResult& predicted,
    const FixedYawTranslationResult& free_ndt,
    double maximum_consistent_translation_m);

}  // namespace ndt_slam
