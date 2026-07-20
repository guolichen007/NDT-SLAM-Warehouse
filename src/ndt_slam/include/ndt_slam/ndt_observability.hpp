#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace ndt_slam {

struct NdtObservabilityConfig {
    bool enabled = true;
    double moderate_ratio = 0.20;
    double severe_ratio = 0.08;
    double moderate_weak_inflation = 5.0;
    double severe_weak_inflation = 20.0;

    std::size_t min_structure_points = 80U;
    std::size_t min_occupied_cells = 20U;
    std::size_t min_local_normals = 30U;
    std::size_t max_normal_samples = 800U;
    int local_neighbor_count = 10;
    double occupancy_cell_m = 0.50;
    double min_xy_span_m = 2.0;
    double normal_search_radius_m = 0.90;
    double min_normal_anisotropy = 0.20;
};

struct NdtObservability {
    bool valid = false;
    bool degenerate = true;
    bool severely_degenerate = true;

    double strong_eigenvalue = 0.0;
    double weak_eigenvalue = 0.0;
    double eigenvalue_ratio = 0.0;

    Eigen::Vector2d strong_direction = Eigen::Vector2d::UnitX();
    Eigen::Vector2d weak_direction = Eigen::Vector2d::UnitY();

    std::size_t structure_points = 0U;
    std::size_t occupied_cells = 0U;
    std::size_t local_normals = 0U;
    double span_x_m = 0.0;
    double span_y_m = 0.0;
    std::string reason = "not_evaluated";
};

// Estimates translational observability from human-safe static structure.
// This is a local-normal information proxy, not the internal NDT Hessian.
NdtObservability estimateNdtObservabilityFromStructure(
    const std::vector<Eigen::Vector2d>& static_structure_points,
    const NdtObservabilityConfig& config);

// Rotates source/base-frame axes into the EKF measurement frame. The
// eigenvalues and degeneracy classification are frame invariant.
NdtObservability rotateNdtObservability(
    const NdtObservability& observability,
    double target_from_source_yaw_rad);

Eigen::Matrix2d buildObservabilityAwareMeasurementCovariance(
    double base_variance,
    const NdtObservability& observability,
    const NdtObservabilityConfig& config);

}  // namespace ndt_slam
