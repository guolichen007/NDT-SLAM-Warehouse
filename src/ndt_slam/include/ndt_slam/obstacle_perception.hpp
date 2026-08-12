#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ndt_slam/cargo_rigid_geometry.hpp"

namespace ndt_slam {

struct ObstaclePerceptionConfig {
  double future_stamp_tolerance_sec = 0.05;
  double maximum_cloud_age_sec = 0.50;
  std::size_t minimum_roi_finite_points = 20U;
  float minimum_roi_coverage_ratio = 0.05F;
  float cluster_tolerance_m = 0.25F;
  std::size_t minimum_cluster_points = 5U;
  float top_percentile = 0.95F;
  float bottom_percentile = 0.05F;
  float vertical_bin_size_m = 0.10F;
  float uncertainty_floor_m = 0.05F;
  float uncertainty_max_m = 0.30F;
};

struct ObstaclePerceptionInput {
  // Canonical physical-frame identity. Formal/Pending are transport phases,
  // not perception inputs, and therefore cannot alter clustering.
  double source_stamp_sec = 0.0;
  std::uint64_t source_sequence = 0U;
  std::string frame_id = "base_link";
  CargoObbFootprint query_footprint;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_base;
  bool observation_valid = false;
  double cloud_age_sec = std::numeric_limits<double>::infinity();
  std::size_t roi_finite_points = 0U;
  float roi_coverage_ratio = 0.0F;
};

struct ObstaclePerceptionCluster {
  std::size_t cluster_index = 0U;
  std::size_t point_count = 0U;
  float footprint_distance_m = std::numeric_limits<float>::infinity();
  float top_z95_m = std::numeric_limits<float>::quiet_NaN();
  float bottom_z05_m = std::numeric_limits<float>::quiet_NaN();
  float minimum_z_m = std::numeric_limits<float>::quiet_NaN();
  float maximum_z_m = std::numeric_limits<float>::quiet_NaN();
  float vertical_span_m = std::numeric_limits<float>::quiet_NaN();
  float vertical_continuity_ratio = 0.0F;
  float tail_spread_m = std::numeric_limits<float>::quiet_NaN();
  float obstacle_uncertainty_m = std::numeric_limits<float>::quiet_NaN();
  pcl::PointXYZ centroid_base;
  pcl::PointXYZ nearest_point_base;
  std::vector<int> point_indices;
};

struct ObstaclePerceptionResult {
  double source_stamp_sec = 0.0;
  std::uint64_t source_sequence = 0U;
  std::string frame_id = "base_link";
  bool executed = false;
  bool valid = false;
  bool query_horizontal_valid = false;
  std::size_t finite_input_points = 0U;
  std::size_t cluster_count = 0U;
  std::vector<ObstaclePerceptionCluster> clusters;
  std::string reason = "not_executed";
};

bool validateObstaclePerceptionConfig(
    const ObstaclePerceptionConfig& config,
    std::string* reason = nullptr) noexcept;

ObstaclePerceptionResult perceiveObstacles(
    const ObstaclePerceptionConfig& config,
    const ObstaclePerceptionInput& input);

}  // namespace ndt_slam
