#include "ndt_slam/se2_observability_proxy.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ndt_slam {
namespace {

struct Neighbor {
  double squared_distance = 0.0;
  std::size_t index = 0U;
};

}  // namespace

Se2ObservabilityProxy estimateSe2ObservabilityProxy(
    const std::vector<Eigen::Vector2d>& input_points,
    const Se2ObservabilityProxyConfig& input_config) {
  Se2ObservabilityProxy result;
  Se2ObservabilityProxyConfig config = input_config;
  config.maximum_samples = std::max<std::size_t>(1U, config.maximum_samples);
  config.neighbor_count = std::max<std::size_t>(3U, config.neighbor_count);
  config.neighbor_radius_m = std::max(0.05, config.neighbor_radius_m);
  config.minimum_normal_anisotropy = std::clamp(
      config.minimum_normal_anisotropy, 0.0, 1.0);

  std::vector<Eigen::Vector2d> points;
  points.reserve(input_points.size());
  for (const auto& point : input_points) {
    if (point.allFinite()) points.push_back(point);
  }
  if (points.size() < config.minimum_points) {
    result.reason = "structure_points_insufficient";
    return result;
  }

  Eigen::Vector2d global_centroid = Eigen::Vector2d::Zero();
  for (const auto& point : points) global_centroid += point;
  global_centroid /= static_cast<double>(points.size());

  const std::size_t sample_count = std::min(
      points.size(), config.maximum_samples);
  const double stride = static_cast<double>(points.size()) /
      static_cast<double>(sample_count);
  const double radius_squared =
      config.neighbor_radius_m * config.neighbor_radius_m;
  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  Eigen::Matrix2d normal_information = Eigen::Matrix2d::Zero();
  std::vector<Neighbor> neighbors;
  neighbors.reserve(points.size());

  for (std::size_t sample = 0U; sample < sample_count; ++sample) {
    const std::size_t center_index = std::min(
        points.size() - 1U,
        static_cast<std::size_t>(std::floor(sample * stride)));
    neighbors.clear();
    for (std::size_t index = 0U; index < points.size(); ++index) {
      if (index == center_index) continue;
      const double distance =
          (points[index] - points[center_index]).squaredNorm();
      if (distance <= radius_squared) neighbors.push_back({distance, index});
    }
    if (neighbors.size() < 3U) continue;
    const std::size_t keep = std::min(config.neighbor_count, neighbors.size());
    std::nth_element(
        neighbors.begin(), neighbors.begin() + keep - 1U, neighbors.end(),
        [](const Neighbor& lhs, const Neighbor& rhs) {
          return lhs.squared_distance < rhs.squared_distance;
        });
    Eigen::Vector2d centroid = points[center_index];
    for (std::size_t index = 0U; index < keep; ++index) {
      centroid += points[neighbors[index].index];
    }
    centroid /= static_cast<double>(keep + 1U);
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (std::size_t index = 0U; index < keep; ++index) {
      const Eigen::Vector2d centered =
          points[neighbors[index].index] - centroid;
      covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(keep);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> normal_solver(covariance);
    if (normal_solver.info() != Eigen::Success) continue;
    const double small = std::max(0.0, normal_solver.eigenvalues()(0));
    const double large = std::max(0.0, normal_solver.eigenvalues()(1));
    if (large <= 1.0e-12) continue;
    const double anisotropy = (large - small) / (large + small + 1.0e-12);
    if (anisotropy < config.minimum_normal_anisotropy) continue;
    Eigen::Vector2d normal = normal_solver.eigenvectors().col(0).normalized();
    const Eigen::Vector2d radial = points[center_index] - global_centroid;
    const Eigen::Vector2d rotated_radial(-radial.y(), radial.x());
    Eigen::Vector3d jacobian;
    jacobian << normal.x(), normal.y(), normal.dot(rotated_radial);
    information += jacobian * jacobian.transpose();
    normal_information += normal * normal.transpose();
    ++result.local_normals;
  }

  if (result.local_normals < 3U) {
    result.reason = "local_normals_insufficient";
    return result;
  }
  information /= static_cast<double>(result.local_normals);
  normal_information /= static_cast<double>(result.local_normals);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(information);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> normal_solver(
      normal_information);
  if (solver.info() != Eigen::Success ||
      normal_solver.info() != Eigen::Success ||
      !solver.eigenvalues().allFinite()) {
    result.reason = "information_eigendecomposition_failed";
    return result;
  }
  result.minimum_eigenvalue = std::max(0.0, solver.eigenvalues()(0));
  result.maximum_eigenvalue = std::max(0.0, solver.eigenvalues()(2));
  result.eigenvalue_ratio = result.maximum_eigenvalue > 1.0e-12
      ? result.minimum_eigenvalue / result.maximum_eigenvalue
      : 0.0;
  result.yaw_information = information(2, 2);
  const double normal_max = std::max(0.0, normal_solver.eigenvalues()(1));
  result.direction_coverage = normal_max > 1.0e-12
      ? std::max(0.0, normal_solver.eigenvalues()(0)) / normal_max
      : 0.0;
  result.valid = true;
  result.yaw_observability_strong =
      result.eigenvalue_ratio >= config.yaw_eigenvalue_ratio_threshold &&
      result.direction_coverage >= config.minimum_direction_coverage;
  result.reason = result.yaw_observability_strong
      ? "strong_structural_yaw_proxy"
      : "weak_structural_yaw_proxy_normal_operation";
  return result;
}

}  // namespace ndt_slam
