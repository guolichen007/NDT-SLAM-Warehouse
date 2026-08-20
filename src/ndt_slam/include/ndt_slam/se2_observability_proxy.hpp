#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace ndt_slam {

struct Se2ObservabilityProxyConfig {
  std::size_t minimum_points = 30U;
  std::size_t maximum_samples = 500U;
  std::size_t neighbor_count = 10U;
  double neighbor_radius_m = 0.90;
  double minimum_normal_anisotropy = 0.20;
  double yaw_eigenvalue_ratio_threshold = 0.02;
  double minimum_direction_coverage = 0.15;
};

struct Se2ObservabilityProxy {
  bool valid = false;
  bool yaw_observability_strong = false;
  std::size_t local_normals = 0U;
  double minimum_eigenvalue = 0.0;
  double maximum_eigenvalue = 0.0;
  double eigenvalue_ratio = 0.0;
  double yaw_information = 0.0;
  double direction_coverage = 0.0;
  std::string reason = "not_evaluated";
};

// Builds a structural point-to-line information proxy using Jacobians
// [nx, ny, n dot R90(point-centroid)] for [tx, ty, yaw]. This is explicitly
// not the internal NDT Hessian and must be used as diagnostic evidence only.
Se2ObservabilityProxy estimateSe2ObservabilityProxy(
    const std::vector<Eigen::Vector2d>& registration_frame_points,
    const Se2ObservabilityProxyConfig& config = {});

}  // namespace ndt_slam
