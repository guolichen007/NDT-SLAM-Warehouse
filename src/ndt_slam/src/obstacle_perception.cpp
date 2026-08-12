#include "ndt_slam/obstacle_perception.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <set>

#include <Eigen/Core>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace ndt_slam {
namespace {

bool finitePoint(const pcl::PointXYZ& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
      std::isfinite(point.z);
}

bool validHorizontalFootprint(const CargoObbFootprint& footprint) {
  return footprint.valid && footprint.center_base.allFinite() &&
      std::isfinite(footprint.length_m) && footprint.length_m > 0.0F &&
      std::isfinite(footprint.width_m) && footprint.width_m > 0.0F &&
      std::isfinite(footprint.yaw_base_rad);
}

float nearestRankPercentile(std::vector<float>* values, float percentile) {
  const std::size_t size = values->size();
  std::size_t rank = static_cast<std::size_t>(std::ceil(
      static_cast<double>(percentile) * static_cast<double>(size)));
  rank = std::max<std::size_t>(1U, std::min(rank, size));
  const std::size_t index = rank - 1U;
  std::nth_element(values->begin(), values->begin() + index, values->end());
  return (*values)[index];
}

}  // namespace

bool validateObstaclePerceptionConfig(
    const ObstaclePerceptionConfig& config,
    std::string* reason) noexcept {
  const auto reject = [reason](const char* field) {
    if (reason != nullptr) *reason = field;
    return false;
  };
  if (!std::isfinite(config.future_stamp_tolerance_sec) ||
      config.future_stamp_tolerance_sec < 0.0) {
    return reject("future_stamp_tolerance_sec");
  }
  if (!std::isfinite(config.maximum_cloud_age_sec) ||
      config.maximum_cloud_age_sec < 0.0) {
    return reject("maximum_cloud_age_sec");
  }
  if (config.minimum_roi_finite_points == 0U) {
    return reject("minimum_roi_finite_points");
  }
  if (!std::isfinite(config.minimum_roi_coverage_ratio) ||
      config.minimum_roi_coverage_ratio < 0.0F ||
      config.minimum_roi_coverage_ratio > 1.0F) {
    return reject("minimum_roi_coverage_ratio");
  }
  if (!std::isfinite(config.cluster_tolerance_m) ||
      config.cluster_tolerance_m <= 0.0F) {
    return reject("cluster_tolerance_m");
  }
  if (config.minimum_cluster_points == 0U ||
      config.minimum_cluster_points >
          static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    return reject("minimum_cluster_points");
  }
  if (!std::isfinite(config.top_percentile) ||
      config.top_percentile < 0.0F || config.top_percentile > 1.0F) {
    return reject("top_percentile");
  }
  if (!std::isfinite(config.bottom_percentile) ||
      config.bottom_percentile < 0.0F ||
      config.bottom_percentile >= config.top_percentile) {
    return reject("bottom_percentile");
  }
  if (!std::isfinite(config.vertical_bin_size_m) ||
      config.vertical_bin_size_m <= 0.0F) {
    return reject("vertical_bin_size_m");
  }
  if (!std::isfinite(config.uncertainty_floor_m) ||
      config.uncertainty_floor_m < 0.0F) {
    return reject("uncertainty_floor_m");
  }
  if (!std::isfinite(config.uncertainty_max_m) ||
      config.uncertainty_max_m < config.uncertainty_floor_m) {
    return reject("uncertainty_max_m");
  }
  if (reason != nullptr) reason->clear();
  return true;
}

ObstaclePerceptionResult perceiveObstacles(
    const ObstaclePerceptionConfig& config,
    const ObstaclePerceptionInput& input) {
  ObstaclePerceptionResult result;
  result.executed = true;
  std::string invalid_field;
  if (!validateObstaclePerceptionConfig(config, &invalid_field)) {
    result.reason = "invalid_config:" + invalid_field;
    return result;
  }
  result.query_horizontal_valid =
      validHorizontalFootprint(input.query_footprint);
  if (!result.query_horizontal_valid || !input.cloud_base) {
    result.reason = "invalid_horizontal_query";
    return result;
  }
  if (!input.observation_valid || !std::isfinite(input.cloud_age_sec) ||
      input.cloud_age_sec < -config.future_stamp_tolerance_sec ||
      input.cloud_age_sec > config.maximum_cloud_age_sec ||
      input.roi_finite_points < config.minimum_roi_finite_points ||
      !std::isfinite(input.roi_coverage_ratio) ||
      input.roi_coverage_ratio < config.minimum_roi_coverage_ratio) {
    result.reason = "obstacle_observation_insufficient";
    return result;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr finite_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> finite_source_indices;
  finite_cloud->reserve(input.cloud_base->size());
  finite_source_indices.reserve(input.cloud_base->size());
  for (std::size_t source_index = 0U;
       source_index < input.cloud_base->points.size(); ++source_index) {
    const pcl::PointXYZ& point = input.cloud_base->points[source_index];
    if (!finitePoint(point)) continue;
    finite_cloud->push_back(point);
    finite_source_indices.push_back(static_cast<int>(source_index));
  }
  result.finite_input_points = finite_cloud->size();
  if (finite_cloud->empty()) {
    result.valid = true;
    result.reason = "clear_no_external_obstacle";
    return result;
  }
  if (finite_cloud->size() < config.minimum_cluster_points) {
    result.reason = "sparse_obstacle_returns";
    return result;
  }

  std::vector<pcl::PointIndices> cluster_indices;
  try {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(finite_cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(config.cluster_tolerance_m);
    extraction.setMinClusterSize(
        static_cast<int>(config.minimum_cluster_points));
    extraction.setMaxClusterSize(std::numeric_limits<int>::max());
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(finite_cloud);
    extraction.extract(cluster_indices);
  } catch (const std::exception&) {
    result.reason = "clustering_failed";
    return result;
  }

  result.clusters.reserve(cluster_indices.size());
  for (std::size_t cluster_index = 0U;
       cluster_index < cluster_indices.size(); ++cluster_index) {
    const pcl::PointIndices& indices = cluster_indices[cluster_index];
    if (indices.indices.empty()) continue;
    ObstaclePerceptionCluster cluster;
    cluster.cluster_index = cluster_index;
    cluster.point_count = indices.indices.size();
    cluster.point_indices.reserve(indices.indices.size());
    for (int finite_index : indices.indices) {
      if (finite_index < 0 ||
          static_cast<std::size_t>(finite_index) >=
              finite_source_indices.size()) {
        result.clusters.clear();
        result.cluster_count = 0U;
        result.reason = "cluster_index_out_of_range";
        return result;
      }
      cluster.point_indices.push_back(
          finite_source_indices[static_cast<std::size_t>(finite_index)]);
    }
    std::vector<float> z_values;
    z_values.reserve(indices.indices.size());
    Eigen::Vector3f centroid_sum = Eigen::Vector3f::Zero();
    for (int point_index : indices.indices) {
      const pcl::PointXYZ& point =
          finite_cloud->points[static_cast<std::size_t>(point_index)];
      z_values.push_back(point.z);
      centroid_sum += point.getVector3fMap();
      const float distance = pointToCargoObbDistance2D(
          Eigen::Vector2f(point.x, point.y), input.query_footprint);
      if (distance < cluster.footprint_distance_m) {
        cluster.footprint_distance_m = distance;
        cluster.nearest_point_base = point;
      }
    }
    const Eigen::Vector3f centroid = centroid_sum /
        static_cast<float>(indices.indices.size());
    cluster.centroid_base.x = centroid.x();
    cluster.centroid_base.y = centroid.y();
    cluster.centroid_base.z = centroid.z();
    cluster.maximum_z_m = *std::max_element(z_values.begin(), z_values.end());
    cluster.minimum_z_m = *std::min_element(z_values.begin(), z_values.end());
    cluster.top_z95_m = nearestRankPercentile(
        &z_values, config.top_percentile);
    cluster.bottom_z05_m = nearestRankPercentile(
        &z_values, config.bottom_percentile);
    cluster.vertical_span_m = std::max(
        0.0F, cluster.top_z95_m - cluster.bottom_z05_m);
    std::set<int> occupied_vertical_bins;
    for (float z : z_values) {
      if (z < cluster.bottom_z05_m || z > cluster.top_z95_m) continue;
      occupied_vertical_bins.insert(static_cast<int>(std::floor(
          (z - cluster.bottom_z05_m) / config.vertical_bin_size_m)));
    }
    const int expected_bins = std::max(
        1, static_cast<int>(std::floor(
               cluster.vertical_span_m / config.vertical_bin_size_m)) + 1);
    cluster.vertical_continuity_ratio = std::clamp(
        static_cast<float>(occupied_vertical_bins.size()) /
            static_cast<float>(expected_bins),
        0.0F, 1.0F);
    cluster.tail_spread_m = std::max(
        0.0F, cluster.maximum_z_m - cluster.top_z95_m);
    cluster.obstacle_uncertainty_m = std::clamp(
        config.uncertainty_floor_m + cluster.tail_spread_m,
        config.uncertainty_floor_m, config.uncertainty_max_m);
    if (!std::isfinite(cluster.footprint_distance_m) ||
        !std::isfinite(cluster.top_z95_m) ||
        !std::isfinite(cluster.bottom_z05_m) ||
        !std::isfinite(cluster.obstacle_uncertainty_m)) {
      result.clusters.clear();
      result.cluster_count = 0U;
      result.reason = "non_finite_cluster_result";
      return result;
    }
    result.clusters.push_back(std::move(cluster));
  }
  result.cluster_count = result.clusters.size();
  if (result.clusters.empty()) {
    result.reason = "obstacle_clusters_insufficient";
    return result;
  }
  result.valid = true;
  result.reason = "obstacle_clusters_ready";
  return result;
}

}  // namespace ndt_slam
