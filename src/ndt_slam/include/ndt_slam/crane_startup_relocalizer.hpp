#pragma once

#include "ndt_slam/crane_place_descriptor.hpp"
#include "ndt_slam/ndt_relocalizer.hpp"
#include "ndt_slam/recovery_checkpoint.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace ndt_slam {

struct CraneStartupRelocalizerConfig {
  double local_x_radius_m = 1.5;
  double local_y_radius_m = 1.5;
  double local_xy_step_m = 0.5;
  double yaw_tolerance_deg = 1.0;
  double yaw_step_deg = 1.0;
  double fixed_z_m = std::numeric_limits<double>::quiet_NaN();
  double fixed_rail_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  std::size_t top_k_fine = 5U;
  double maximum_fitness = 2.0;
  double minimum_fitness_margin = 0.10;
  RelocalizationConfig coarse_ndt;
  RelocalizationConfig fine_ndt;
};

struct CraneRegistrationCandidate {
  bool valid = false;
  Sophus::SE3d pose;
  double fitness = std::numeric_limits<double>::infinity();
  bool observability_valid = false;
  std::string seed_source;
  std::uint64_t place_id = 0U;
  std::string reason;
};

class IGlobalRegistrationBackend {
 public:
  virtual ~IGlobalRegistrationBackend() = default;
  virtual CraneRegistrationCandidate coarseCandidate(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
      const RelocalizationSeed& seed,
      const CraneStartupRelocalizerConfig& config) const = 0;
  virtual CraneRegistrationCandidate refineCandidate(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
      const CraneRegistrationCandidate& coarse,
      const CraneStartupRelocalizerConfig& config) const = 0;
};

class CraneNdtRegistrationBackend final : public IGlobalRegistrationBackend {
 public:
  CraneRegistrationCandidate coarseCandidate(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
      const RelocalizationSeed& seed,
      const CraneStartupRelocalizerConfig& config) const override;
  CraneRegistrationCandidate refineCandidate(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
      const CraneRegistrationCandidate& coarse,
      const CraneStartupRelocalizerConfig& config) const override;
};

struct CraneRecoveryResult {
  bool valid = false;
  bool ambiguous = false;
  CraneRegistrationCandidate best;
  CraneRegistrationCandidate second;
  double score_margin = -std::numeric_limits<double>::infinity();
  std::size_t candidates_tested = 0U;
  std::string reason;
};

class CraneStartupRelocalizer {
 public:
  explicit CraneStartupRelocalizer(
      const CraneStartupRelocalizerConfig& config = {});

  void configure(const CraneStartupRelocalizerConfig& config);
  std::vector<RelocalizationSeed> checkpointSeeds(
      const RecoveryCheckpointData& checkpoint) const;
  std::vector<RelocalizationSeed> placeSeeds(
      const std::vector<CranePlaceCandidate>& candidates) const;
  CraneRecoveryResult recover(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
      const std::vector<RelocalizationSeed>& seeds,
      const IGlobalRegistrationBackend& backend) const;

 private:
  CraneStartupRelocalizerConfig config_;
};

}  // namespace ndt_slam
