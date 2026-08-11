#include "ndt_slam/crane_startup_relocalizer.hpp"

#include "ndt_slam/rigid_transform_conversion.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>

#include <algorithm>
#include <climits>
#include <cmath>

namespace ndt_slam {
namespace {

double wrapAngle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

double yawOf(const Sophus::SE3d& pose) {
  const auto rotation = pose.so3().matrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Sophus::SE3d constrainedPose(double x, double y, double z, double yaw) {
  return Sophus::SE3d(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
      Eigen::Vector3d(x, y, z));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelized(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, double leaf) {
  auto output = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (!input || input->empty()) return output;
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  const float size = static_cast<float>(std::max(0.05, leaf));
  filter.setLeafSize(size, size, size);
  filter.filter(*output);
  return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cropTarget(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Sophus::SE3d& seed, double radius) {
  auto cropped = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (!target) return cropped;
  const double radius_squared = radius * radius;
  for (const auto& point : target->points) {
    const double dx = point.x - seed.translation().x();
    const double dy = point.y - seed.translation().y();
    if (dx * dx + dy * dy <= radius_squared) cropped->push_back(point);
  }
  return cropped;
}

struct NdtStageResult {
  bool valid = false;
  Sophus::SE3d pose;
  double fitness = std::numeric_limits<double>::infinity();
};

NdtStageResult runStage(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& full_target,
    const Sophus::SE3d& seed, const RelocalizationConfig& config,
    const CraneStartupRelocalizerConfig& recovery_config) {
  NdtStageResult result;
  auto source_filtered = voxelized(source, config.source_voxel_m);
  auto target_filtered = voxelized(
      cropTarget(full_target, seed, config.target_crop_radius_m),
      config.target_voxel_m);
  if (static_cast<int>(source_filtered->size()) < config.min_source_points ||
      static_cast<int>(target_filtered->size()) < config.min_target_points) {
    return result;
  }
  pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setResolution(config.resolution);
  ndt.setStepSize(config.step_size);
  ndt.setTransformationEpsilon(config.transformation_epsilon);
  ndt.setMaximumIterations(config.max_iterations);
  ndt.setNumThreads(std::clamp(config.num_threads, 1, 4));
  ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
  ndt.setInputSource(source_filtered);
  ndt.setInputTarget(target_filtered);
  pcl::PointCloud<pcl::PointXYZ> aligned;
  ndt.align(aligned, seed.matrix().cast<float>());
  if (!ndt.hasConverged()) return result;
  const SafeSE3Result converted =
      makeSafeSE3FromMatrix(ndt.getFinalTransformation().cast<double>());
  const double fitness = ndt.getFitnessScore();
  if (!converted.valid || !std::isfinite(fitness) ||
      fitness > std::min(config.max_fitness, recovery_config.maximum_fitness)) {
    return result;
  }
  const double fixed_z = std::isfinite(recovery_config.fixed_z_m)
      ? recovery_config.fixed_z_m : seed.translation().z();
  const double rail_yaw = std::isfinite(recovery_config.fixed_rail_yaw_rad)
      ? recovery_config.fixed_rail_yaw_rad : yawOf(seed);
  const double candidate_yaw = yawOf(converted.pose);
  const double yaw_limit = recovery_config.yaw_tolerance_deg * M_PI / 180.0;
  if (std::abs(wrapAngle(candidate_yaw - rail_yaw)) > yaw_limit + 1.0e-9) {
    return result;
  }
  // Explicitly project every accepted stage back to the crane manifold. Z,
  // roll and pitch never propagate from the generic NDT optimizer.
  result.pose = constrainedPose(converted.pose.translation().x(),
                                converted.pose.translation().y(), fixed_z,
                                candidate_yaw);
  result.fitness = fitness;
  result.valid = true;
  return result;
}

}  // namespace

CraneRegistrationCandidate CraneNdtRegistrationBackend::coarseCandidate(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const RelocalizationSeed& seed,
    const CraneStartupRelocalizerConfig& config) const {
  CraneRegistrationCandidate result;
  result.seed_source = seed.source;
  result.place_id = seed.keyframe_id > 0
      ? static_cast<std::uint64_t>(seed.keyframe_id) : 0U;
  const auto coarse =
      runStage(source, target, seed.pose, config.coarse_ndt, config);
  if (!coarse.valid) {
    result.reason = "coarse_ndt_failed";
    return result;
  }
  result.valid = true;
  result.pose = coarse.pose;
  result.fitness = coarse.fitness;
  result.observability_valid = true;
  result.reason = "coarse_constrained_ndt";
  return result;
}

CraneRegistrationCandidate CraneNdtRegistrationBackend::refineCandidate(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const CraneRegistrationCandidate& coarse,
    const CraneStartupRelocalizerConfig& config) const {
  CraneRegistrationCandidate result = coarse;
  const auto fine =
      runStage(source, target, coarse.pose, config.fine_ndt, config);
  if (!fine.valid) {
    result.valid = false;
    result.reason = "fine_ndt_failed";
    return result;
  }
  result.valid = true;
  result.pose = fine.pose;
  result.fitness = fine.fitness;
  result.observability_valid = true;
  result.reason = "fine_constrained_ndt";
  return result;
}

CraneStartupRelocalizer::CraneStartupRelocalizer(
    const CraneStartupRelocalizerConfig& config) {
  configure(config);
}

void CraneStartupRelocalizer::configure(
    const CraneStartupRelocalizerConfig& config) {
  config_ = config;
  config_.local_x_radius_m = std::max(0.0, config_.local_x_radius_m);
  config_.local_y_radius_m = std::max(0.0, config_.local_y_radius_m);
  config_.local_xy_step_m = std::max(0.05, config_.local_xy_step_m);
  config_.yaw_tolerance_deg =
      std::clamp(config_.yaw_tolerance_deg, 0.0, 5.0);
  config_.yaw_step_deg = std::max(0.1, config_.yaw_step_deg);
  config_.top_k_fine = std::max<std::size_t>(1U, config_.top_k_fine);
  config_.maximum_fitness = std::max(0.0, config_.maximum_fitness);
  config_.minimum_fitness_margin =
      std::max(0.0, config_.minimum_fitness_margin);
}

std::vector<RelocalizationSeed> CraneStartupRelocalizer::checkpointSeeds(
    const RecoveryCheckpointData& checkpoint) const {
  std::vector<RelocalizationSeed> seeds;
  if (!std::isfinite(checkpoint.x) || !std::isfinite(checkpoint.y) ||
      !std::isfinite(checkpoint.z) || !std::isfinite(checkpoint.yaw)) {
    return seeds;
  }
  const int x_steps = static_cast<int>(std::floor(
      config_.local_x_radius_m / config_.local_xy_step_m));
  const int y_steps = static_cast<int>(std::floor(
      config_.local_y_radius_m / config_.local_xy_step_m));
  const int yaw_steps = static_cast<int>(std::floor(
      config_.yaw_tolerance_deg / config_.yaw_step_deg));
  const double z = std::isfinite(config_.fixed_z_m)
      ? config_.fixed_z_m : checkpoint.z;
  const double rail_yaw = std::isfinite(config_.fixed_rail_yaw_rad)
      ? config_.fixed_rail_yaw_rad : checkpoint.yaw;
  for (int yaw_index = -yaw_steps; yaw_index <= yaw_steps; ++yaw_index) {
    for (int x_index = -x_steps; x_index <= x_steps; ++x_index) {
      for (int y_index = -y_steps; y_index <= y_steps; ++y_index) {
        RelocalizationSeed seed;
        seed.pose = constrainedPose(
            checkpoint.x + x_index * config_.local_xy_step_m,
            checkpoint.y + y_index * config_.local_xy_step_m, z,
            rail_yaw + yaw_index * config_.yaw_step_deg * M_PI / 180.0);
        seed.source = "checkpoint_grid";
        seeds.push_back(std::move(seed));
      }
    }
  }
  return seeds;
}

std::vector<RelocalizationSeed> CraneStartupRelocalizer::placeSeeds(
    const std::vector<CranePlaceCandidate>& candidates) const {
  std::vector<RelocalizationSeed> seeds;
  seeds.reserve(candidates.size());
  for (const auto& candidate : candidates) {
    const double z = std::isfinite(config_.fixed_z_m)
        ? config_.fixed_z_m : candidate.prior_pose.translation().z();
    const double yaw = std::isfinite(config_.fixed_rail_yaw_rad)
        ? config_.fixed_rail_yaw_rad : yawOf(candidate.prior_pose);
    RelocalizationSeed seed;
    seed.pose = constrainedPose(candidate.prior_pose.translation().x(),
                                candidate.prior_pose.translation().y(), z, yaw);
    seed.source = "crane_place_descriptor";
    seed.keyframe_id = candidate.id > static_cast<std::uint64_t>(INT_MAX)
        ? -1 : static_cast<int>(candidate.id);
    seed.descriptor_similarity = candidate.similarity;
    seeds.push_back(std::move(seed));
  }
  return seeds;
}

CraneRecoveryResult CraneStartupRelocalizer::recover(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const std::vector<RelocalizationSeed>& seeds,
    const IGlobalRegistrationBackend& backend) const {
  CraneRecoveryResult result;
  if (!source || !target || seeds.empty()) {
    result.reason = "recovery_input_invalid";
    return result;
  }
  std::vector<CraneRegistrationCandidate> coarse_candidates;
  coarse_candidates.reserve(seeds.size());
  for (const auto& seed : seeds) {
    auto candidate = backend.coarseCandidate(source, target, seed, config_);
    ++result.candidates_tested;
    if (candidate.valid && candidate.observability_valid &&
        std::isfinite(candidate.fitness) &&
        candidate.fitness <= config_.maximum_fitness) {
      coarse_candidates.push_back(std::move(candidate));
    }
  }
  std::sort(coarse_candidates.begin(), coarse_candidates.end(),
            [](const auto& lhs, const auto& rhs) {
    return lhs.fitness < rhs.fitness;
  });
  if (coarse_candidates.empty()) {
    result.reason = "no_constrained_candidate";
    return result;
  }
  if (coarse_candidates.size() > config_.top_k_fine)
    coarse_candidates.resize(config_.top_k_fine);
  std::vector<CraneRegistrationCandidate> accepted;
  accepted.reserve(coarse_candidates.size());
  for (const auto& coarse : coarse_candidates) {
    auto fine = backend.refineCandidate(source, target, coarse, config_);
    if (fine.valid && fine.observability_valid &&
        std::isfinite(fine.fitness) &&
        fine.fitness <= config_.maximum_fitness) {
      accepted.push_back(std::move(fine));
    }
  }
  std::sort(accepted.begin(), accepted.end(), [](const auto& lhs,
                                                  const auto& rhs) {
    return lhs.fitness < rhs.fitness;
  });
  if (accepted.empty()) {
    result.reason = "no_fine_candidate";
    return result;
  }
  result.best = accepted.front();
  if (accepted.size() > 1U) {
    result.second = accepted[1U];
    result.score_margin = result.second.fitness - result.best.fitness;
    if (result.score_margin < config_.minimum_fitness_margin) {
      result.ambiguous = true;
      result.reason = "ambiguous_top_candidates";
      return result;
    }
  } else {
    result.score_margin = std::numeric_limits<double>::infinity();
  }
  result.valid = true;
  result.reason = "candidate_requires_sequential_verification";
  return result;
}

}  // namespace ndt_slam
