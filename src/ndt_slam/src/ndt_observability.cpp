#include "ndt_slam/ndt_observability.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace ndt_slam {
namespace {

struct Neighbor {
    double squared_distance = 0.0;
    std::size_t index = 0U;
};

bool finitePoint(const Eigen::Vector2d& point) {
    return point.allFinite();
}

}  // namespace

NdtObservability estimateNdtObservabilityFromStructure(
    const std::vector<Eigen::Vector2d>& input_points,
    const NdtObservabilityConfig& input_config) {
    NdtObservability result;
    NdtObservabilityConfig config = input_config;
    if (!config.enabled) {
        result.reason = "disabled";
        return result;
    }

    config.moderate_ratio = std::clamp(config.moderate_ratio, 0.0, 1.0);
    config.severe_ratio = std::clamp(
        config.severe_ratio, 0.0, config.moderate_ratio);
    config.local_neighbor_count = std::max(3, config.local_neighbor_count);
    config.max_normal_samples = std::max<std::size_t>(
        1U, config.max_normal_samples);
    config.occupancy_cell_m = std::max(0.05, config.occupancy_cell_m);
    config.normal_search_radius_m =
        std::max(0.05, config.normal_search_radius_m);
    config.min_normal_anisotropy =
        std::clamp(config.min_normal_anisotropy, 0.0, 1.0);

    std::vector<Eigen::Vector2d> points;
    points.reserve(input_points.size());
    std::set<std::pair<long long, long long>> unique_xy;
    for (const auto& point : input_points) {
        if (!finitePoint(point)) continue;
        ++result.structure_points;
        const auto key = std::make_pair(
            static_cast<long long>(std::llround(point.x() / 0.02)),
            static_cast<long long>(std::llround(point.y() / 0.02)));
        if (unique_xy.insert(key).second) points.push_back(point);
    }
    if (result.structure_points < config.min_structure_points) {
        result.reason = "structure_points_insufficient";
        return result;
    }
    if (points.size() < 4U) {
        result.reason = "unique_xy_points_insufficient";
        return result;
    }

    Eigen::Vector2d minimum = points.front();
    Eigen::Vector2d maximum = points.front();
    std::set<std::pair<int, int>> occupied;
    for (const auto& point : points) {
        minimum = minimum.cwiseMin(point);
        maximum = maximum.cwiseMax(point);
        occupied.emplace(
            static_cast<int>(std::floor(
                point.x() / config.occupancy_cell_m)),
            static_cast<int>(std::floor(
                point.y() / config.occupancy_cell_m)));
    }
    result.span_x_m = maximum.x() - minimum.x();
    result.span_y_m = maximum.y() - minimum.y();
    result.occupied_cells = occupied.size();
    if (result.occupied_cells < config.min_occupied_cells) {
        result.reason = "occupied_cells_insufficient";
        return result;
    }
    if (std::max(result.span_x_m, result.span_y_m) <
        config.min_xy_span_m) {
        result.reason = "xy_span_insufficient";
        return result;
    }

    const std::size_t sample_count = std::min(
        points.size(), config.max_normal_samples);
    const double sample_stride =
        static_cast<double>(points.size()) /
        static_cast<double>(sample_count);
    const double radius_squared =
        config.normal_search_radius_m * config.normal_search_radius_m;
    Eigen::Matrix2d information = Eigen::Matrix2d::Zero();
    std::vector<Neighbor> neighbors;
    neighbors.reserve(points.size());

    for (std::size_t sample = 0U; sample < sample_count; ++sample) {
        const std::size_t center_index = std::min(
            points.size() - 1U,
            static_cast<std::size_t>(std::floor(sample * sample_stride)));
        neighbors.clear();
        for (std::size_t index = 0U; index < points.size(); ++index) {
            if (index == center_index) continue;
            const double squared_distance =
                (points[index] - points[center_index]).squaredNorm();
            if (squared_distance <= radius_squared) {
                neighbors.push_back({squared_distance, index});
            }
        }
        if (neighbors.size() < 3U) continue;
        const std::size_t keep = std::min<std::size_t>(
            neighbors.size(),
            static_cast<std::size_t>(config.local_neighbor_count));
        std::nth_element(
            neighbors.begin(), neighbors.begin() + keep - 1U,
            neighbors.end(),
            [](const Neighbor& lhs, const Neighbor& rhs) {
                return lhs.squared_distance < rhs.squared_distance;
            });

        Eigen::Vector2d centroid = points[center_index];
        for (std::size_t index = 0U; index < keep; ++index) {
            centroid += points[neighbors[index].index];
        }
        centroid /= static_cast<double>(keep + 1U);

        Eigen::Matrix2d covariance =
            (points[center_index] - centroid) *
            (points[center_index] - centroid).transpose();
        for (std::size_t index = 0U; index < keep; ++index) {
            const Eigen::Vector2d centered =
                points[neighbors[index].index] - centroid;
            covariance += centered * centered.transpose();
        }
        covariance /= static_cast<double>(keep + 1U);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
        if (solver.info() != Eigen::Success ||
            !solver.eigenvalues().allFinite() ||
            !solver.eigenvectors().allFinite()) {
            continue;
        }
        const double small = std::max(0.0, solver.eigenvalues()(0));
        const double large = std::max(0.0, solver.eigenvalues()(1));
        if (large <= 1.0e-12) continue;
        const double anisotropy =
            std::clamp((large - small) / (large + small + 1.0e-12),
                       0.0, 1.0);
        if (anisotropy < config.min_normal_anisotropy) continue;
        const Eigen::Vector2d normal = solver.eigenvectors().col(0);
        information += anisotropy * normal * normal.transpose();
        ++result.local_normals;
    }

    if (result.local_normals < config.min_local_normals ||
        !information.allFinite()) {
        result.reason = "local_normals_insufficient";
        return result;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(information);
    if (solver.info() != Eigen::Success ||
        !solver.eigenvalues().allFinite() ||
        !solver.eigenvectors().allFinite()) {
        result.reason = "information_eigendecomposition_failed";
        return result;
    }

    result.weak_eigenvalue = std::max(0.0, solver.eigenvalues()(0));
    result.strong_eigenvalue = std::max(0.0, solver.eigenvalues()(1));
    if (result.strong_eigenvalue <= 1.0e-12) {
        result.reason = "information_rank_zero";
        return result;
    }
    result.weak_direction = solver.eigenvectors().col(0).normalized();
    result.strong_direction = solver.eigenvectors().col(1).normalized();
    result.eigenvalue_ratio =
        result.weak_eigenvalue / result.strong_eigenvalue;
    result.valid = std::isfinite(result.eigenvalue_ratio);
    result.degenerate = result.valid &&
        result.eigenvalue_ratio < config.moderate_ratio;
    result.severely_degenerate = result.valid &&
        result.eigenvalue_ratio < config.severe_ratio;
    result.reason = !result.valid
        ? "nonfinite_information_ratio"
        : result.severely_degenerate
            ? "severe_structure_normal_proxy_degeneracy"
            : result.degenerate
                ? "moderate_structure_normal_proxy_degeneracy"
                : "structure_normal_proxy_observable";
    return result;
}

NdtObservability rotateNdtObservability(
    const NdtObservability& observability,
    double target_from_source_yaw_rad) {
    NdtObservability rotated = observability;
    if (!observability.valid ||
        !observability.strong_direction.allFinite() ||
        !observability.weak_direction.allFinite() ||
        !std::isfinite(target_from_source_yaw_rad)) {
        return rotated;
    }
    const double cosine = std::cos(target_from_source_yaw_rad);
    const double sine = std::sin(target_from_source_yaw_rad);
    Eigen::Matrix2d target_from_source;
    target_from_source << cosine, -sine, sine, cosine;
    rotated.strong_direction =
        (target_from_source * observability.strong_direction).normalized();
    rotated.weak_direction =
        (target_from_source * observability.weak_direction).normalized();
    return rotated;
}

Eigen::Matrix2d buildObservabilityAwareMeasurementCovariance(
    double base_variance,
    const NdtObservability& observability,
    const NdtObservabilityConfig& config) {
    const double base = std::max(1.0e-9, base_variance);
    if (!config.enabled || !observability.valid ||
        !observability.strong_direction.allFinite() ||
        !observability.weak_direction.allFinite()) {
        return base * Eigen::Matrix2d::Identity();
    }

    double weak_inflation = 1.0;
    if (observability.severely_degenerate) {
        weak_inflation = std::max(1.0, config.severe_weak_inflation);
    } else if (observability.degenerate) {
        weak_inflation = std::max(1.0, config.moderate_weak_inflation);
    }

    Eigen::Matrix2d directions;
    directions.col(0) = observability.strong_direction.normalized();
    directions.col(1) = observability.weak_direction.normalized();
    Eigen::Matrix2d directional_variance = Eigen::Matrix2d::Zero();
    directional_variance(0, 0) = base;
    directional_variance(1, 1) = base * weak_inflation;
    return directions * directional_variance * directions.transpose();
}

}  // namespace ndt_slam
