#include "ndt_slam/cargo_oriented_footprint.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

float percentile(std::vector<float> values, float fraction) {
    if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const float clamped = std::clamp(fraction, 0.0F, 1.0F);
    const float position =
        clamped * static_cast<float>(values.size() - 1U);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = std::min(lower + 1U, values.size() - 1U);
    const float alpha = position - static_cast<float>(lower);
    return values[lower] * (1.0F - alpha) + values[upper] * alpha;
}

bool validConfig(const CargoOrientedFootprintConfig& config) {
    return config.minimum_points >= 3U &&
        std::isfinite(config.percentile_low) &&
        std::isfinite(config.percentile_high) &&
        config.percentile_low >= 0.0F &&
        config.percentile_high <= 1.0F &&
        config.percentile_low < config.percentile_high &&
        std::isfinite(config.margin_m) && config.margin_m >= 0.0F &&
        std::isfinite(config.minimum_axis_ratio) &&
        config.minimum_axis_ratio >= 1.0F &&
        std::isfinite(config.minimum_long_side_m) &&
        std::isfinite(config.minimum_short_side_m) &&
        std::isfinite(config.maximum_long_side_m) &&
        std::isfinite(config.maximum_short_side_m) &&
        config.minimum_long_side_m > 0.0F &&
        config.minimum_short_side_m > 0.0F &&
        config.maximum_long_side_m >= config.minimum_long_side_m &&
        config.maximum_short_side_m >= config.minimum_short_side_m;
}

}  // namespace

float normalizeCargoAxialYaw(float yaw_rad) {
    if (!std::isfinite(yaw_rad)) return 0.0F;
    constexpr float kPi = 3.14159265358979323846F;
    constexpr float kHalfPi = 0.5F * kPi;
    while (yaw_rad >= kHalfPi) yaw_rad -= kPi;
    while (yaw_rad < -kHalfPi) yaw_rad += kPi;
    return yaw_rad;
}

bool meanCargoAxialYaw(const std::vector<float>& yaw_samples,
                       float* mean_yaw_rad) {
    if (!mean_yaw_rad) return false;
    double cosine_sum = 0.0;
    double sine_sum = 0.0;
    std::size_t finite_count = 0U;
    for (const float yaw : yaw_samples) {
        if (!std::isfinite(yaw)) continue;
        cosine_sum += std::cos(2.0 * static_cast<double>(yaw));
        sine_sum += std::sin(2.0 * static_cast<double>(yaw));
        ++finite_count;
    }
    if (finite_count == 0U ||
        std::hypot(cosine_sum, sine_sum) <= 1.0e-9) {
        return false;
    }
    *mean_yaw_rad = normalizeCargoAxialYaw(static_cast<float>(
        0.5 * std::atan2(sine_sum, cosine_sum)));
    return true;
}

CargoOrientedFootprint estimateCargoOrientedFootprint(
    const std::vector<Eigen::Vector2f>& points_base,
    const Eigen::Vector2f& anchor_base,
    const CargoOrientedFootprintConfig& config) {
    CargoOrientedFootprint result;
    if (!validConfig(config)) {
        result.reason = "invalid_config";
        return result;
    }
    if (!anchor_base.allFinite()) {
        result.reason = "nonfinite_anchor";
        return result;
    }

    std::vector<Eigen::Vector2f> finite_points;
    finite_points.reserve(points_base.size());
    std::vector<float> finite_x;
    std::vector<float> finite_y;
    finite_x.reserve(points_base.size());
    finite_y.reserve(points_base.size());
    for (const auto& point : points_base) {
        if (!point.allFinite()) continue;
        finite_points.push_back(point);
        finite_x.push_back(point.x());
        finite_y.push_back(point.y());
    }
    result.finite_points = finite_points.size();
    if (finite_points.size() < config.minimum_points) {
        result.reason = "too_few_finite_points";
        return result;
    }
    const Eigen::Vector2f median_center(
        percentile(finite_x, 0.50F), percentile(finite_y, 0.50F));
    std::vector<float> squared_distances;
    squared_distances.reserve(finite_points.size());
    for (const auto& point : finite_points) {
        squared_distances.push_back((point - median_center).squaredNorm());
    }
    const float covariance_distance_limit =
        percentile(squared_distances, 0.95F) + 1.0e-6F;
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    std::size_t covariance_points = 0U;
    for (const auto& point : finite_points) {
        if ((point - median_center).squaredNorm() >
            covariance_distance_limit) continue;
        centroid += point;
        ++covariance_points;
    }
    if (covariance_points < config.minimum_points) {
        result.reason = "too_few_robust_covariance_points";
        return result;
    }
    centroid /= static_cast<float>(covariance_points);

    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    for (const auto& point : finite_points) {
        if ((point - median_center).squaredNorm() >
            covariance_distance_limit) continue;
        const Eigen::Vector2f delta = point - centroid;
        covariance.noalias() += delta * delta.transpose();
    }
    covariance /= static_cast<float>(covariance_points);
    if (!covariance.allFinite()) {
        result.reason = "nonfinite_covariance";
        return result;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
    if (solver.info() != Eigen::Success ||
        !solver.eigenvalues().allFinite() ||
        !solver.eigenvectors().allFinite()) {
        result.reason = "eigendecomposition_failed";
        return result;
    }
    const float weak = std::max(0.0F, solver.eigenvalues()(0));
    const float strong = std::max(0.0F, solver.eigenvalues()(1));
    result.axis_ratio = strong / std::max(weak, 1.0e-8F);
    if (!std::isfinite(result.axis_ratio) ||
        result.axis_ratio < config.minimum_axis_ratio) {
        result.reason = "orientation_ambiguous";
        return result;
    }

    Eigen::Vector2f long_axis = solver.eigenvectors().col(1).normalized();
    Eigen::Vector2f short_axis(-long_axis.y(), long_axis.x());
    std::vector<float> along_long;
    std::vector<float> along_short;
    along_long.reserve(finite_points.size());
    along_short.reserve(finite_points.size());
    for (const auto& point : finite_points) {
        const Eigen::Vector2f anchored = point - anchor_base;
        along_long.push_back(anchored.dot(long_axis));
        along_short.push_back(anchored.dot(short_axis));
    }

    const float long_low = percentile(along_long, config.percentile_low);
    const float long_high = percentile(along_long, config.percentile_high);
    const float short_low = percentile(along_short, config.percentile_low);
    const float short_high = percentile(along_short, config.percentile_high);
    if (!std::isfinite(long_low) || !std::isfinite(long_high) ||
        !std::isfinite(short_low) || !std::isfinite(short_high)) {
        result.reason = "nonfinite_projection";
        return result;
    }

    float long_size = 2.0F *
        (std::max(std::abs(long_low), std::abs(long_high)) + config.margin_m);
    float short_size = 2.0F *
        (std::max(std::abs(short_low), std::abs(short_high)) + config.margin_m);
    float yaw = std::atan2(long_axis.y(), long_axis.x());
    if (short_size > long_size) {
        std::swap(long_size, short_size);
        yaw += 0.5F * 3.14159265358979323846F;
    }
    long_size = std::clamp(
        long_size, config.minimum_long_side_m, config.maximum_long_side_m);
    short_size = std::clamp(
        short_size, config.minimum_short_side_m, config.maximum_short_side_m);
    if (long_size < short_size) {
        result.reason = "invalid_clamped_axes";
        return result;
    }

    result.valid = true;
    result.size_long_short = Eigen::Vector2f(long_size, short_size);
    result.yaw_base_rad = normalizeCargoAxialYaw(yaw);
    result.reason = "robust_pca_obb";
    return result;
}

}  // namespace ndt_slam
