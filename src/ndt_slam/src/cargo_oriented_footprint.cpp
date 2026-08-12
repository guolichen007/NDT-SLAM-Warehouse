#include "ndt_slam/cargo_oriented_footprint.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_set>

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
        std::isfinite(config.minimum_geometric_aspect_ratio) &&
        config.minimum_geometric_aspect_ratio >= 1.0F &&
        std::isfinite(config.minimum_eigenvalue_ratio) &&
        config.minimum_eigenvalue_ratio >= 1.0F &&
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

float normalizeAxialYaw(float yaw_rad) {
    return normalizeCargoAxialYaw(yaw_rad);
}

float baseYawToMap(float yaw_base, float base_pose_yaw_map) {
    return normalizeCargoAxialYaw(yaw_base + base_pose_yaw_map);
}

float mapYawToBase(float yaw_map, float base_pose_yaw_map) {
    return normalizeCargoAxialYaw(yaw_map - base_pose_yaw_map);
}

bool meanCargoAxialYaw(const std::vector<float>& yaw_samples,
                       float* mean_yaw_rad) {
    if (!mean_yaw_rad) return false;
    const CargoAxialYawSummary summary =
        summarizeCargoAxialYaw(yaw_samples);
    if (!summary.valid) return false;
    *mean_yaw_rad = summary.mean_yaw_rad;
    return true;
}

CargoAxialYawSummary summarizeCargoAxialYaw(
    const std::vector<float>& yaw_samples) {
    CargoAxialYawSummary summary;
    double cosine_sum = 0.0;
    double sine_sum = 0.0;
    for (const float yaw : yaw_samples) {
        if (!std::isfinite(yaw)) continue;
        cosine_sum += std::cos(2.0 * static_cast<double>(yaw));
        sine_sum += std::sin(2.0 * static_cast<double>(yaw));
        ++summary.sample_count;
    }
    if (summary.sample_count == 0U ||
        std::hypot(cosine_sum, sine_sum) <= 1.0e-9) {
        return summary;
    }
    summary.mean_yaw_rad = normalizeCargoAxialYaw(static_cast<float>(
        0.5 * std::atan2(sine_sum, cosine_sum)));
    summary.concentration = static_cast<float>(
        std::hypot(cosine_sum, sine_sum) /
        static_cast<double>(summary.sample_count));
    for (const float yaw : yaw_samples) {
        if (!std::isfinite(yaw)) continue;
        summary.maximum_deviation_rad = std::max(
            summary.maximum_deviation_rad,
            std::abs(normalizeCargoAxialYaw(
                yaw - summary.mean_yaw_rad)));
    }
    summary.valid = true;
    return summary;
}

CargoOrientedFootprint estimateCargoOrientedFootprint(
    const std::vector<Eigen::Vector2f>& points_base,
    const CargoOrientedFootprintConfig& config) {
    CargoOrientedFootprint result;
    if (!validConfig(config)) {
        result.reason = "invalid_config";
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
    result.eigenvalue_ratio = strong / std::max(weak, 1.0e-8F);
    if (!std::isfinite(result.eigenvalue_ratio) ||
        result.eigenvalue_ratio < config.minimum_eigenvalue_ratio) {
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
        const Eigen::Vector2f centered = point - centroid;
        along_long.push_back(centered.dot(long_axis));
        along_short.push_back(centered.dot(short_axis));
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

    float long_size = long_high - long_low + 2.0F * config.margin_m;
    float short_size = short_high - short_low + 2.0F * config.margin_m;
    float long_mid = 0.5F * (long_low + long_high);
    float short_mid = 0.5F * (short_low + short_high);
    float yaw = std::atan2(long_axis.y(), long_axis.x());
    if (short_size > long_size) {
        std::swap(long_size, short_size);
        std::swap(long_mid, short_mid);
        std::swap(long_axis, short_axis);
        yaw += 0.5F * 3.14159265358979323846F;
    }
    result.geometric_aspect_ratio =
        long_size / std::max(short_size, 1.0e-6F);
    if (!std::isfinite(result.geometric_aspect_ratio) ||
        result.geometric_aspect_ratio + 1.0e-6F <
            config.minimum_geometric_aspect_ratio) {
        result.reason = "geometric_aspect_ambiguous";
        return result;
    }
    result.center_base = centroid + long_mid * long_axis +
        short_mid * short_axis;
    result.raw_size_long_short =
        Eigen::Vector2f(long_size, short_size);
    result.long_side_clamped =
        long_size > config.maximum_long_side_m + 1.0e-4F ||
        long_size < config.minimum_long_side_m - 1.0e-4F;
    result.short_side_clamped =
        short_size > config.maximum_short_side_m + 1.0e-4F ||
        short_size < config.minimum_short_side_m - 1.0e-4F;
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
    const float eigen_confidence = std::clamp(
        (result.eigenvalue_ratio - 1.0F) /
            std::max(1.0e-6F,
                     config.minimum_eigenvalue_ratio - 1.0F),
        0.0F, 1.0F);
    const float geometry_confidence = std::clamp(
        (result.geometric_aspect_ratio - 1.0F) /
            std::max(1.0e-6F,
                     config.minimum_geometric_aspect_ratio - 1.0F),
        0.0F, 1.0F);
    result.orientation_confidence =
        std::min(eigen_confidence, geometry_confidence);
    result.reason = "robust_pca_obb";
    return result;
}

CargoAnchorGridFootprint refineCargoAnchorGridFootprint(
    const std::vector<Eigen::Vector2f>& points_base,
    const Eigen::Vector2f& anchor_base,
    float yaw_base_rad,
    const Eigen::Vector2f& robust_size_long_short,
    const CargoAnchorGridFootprintConfig& config) {
    CargoAnchorGridFootprint result;
    if (!anchor_base.allFinite() || !robust_size_long_short.allFinite() ||
        !std::isfinite(yaw_base_rad) ||
        !std::isfinite(config.cell_size_m) || config.cell_size_m <= 0.0F ||
        config.minimum_component_cells == 0U) {
        result.reason = "invalid_grid_input";
        return result;
    }

    const auto key = [](int x, int y) {
        const std::uint64_t packed =
            (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
            static_cast<std::uint32_t>(y);
        return static_cast<std::int64_t>(packed);
    };
    const auto keyX = [](std::int64_t value) {
        return static_cast<std::int32_t>(
            static_cast<std::uint64_t>(value) >> 32);
    };
    const auto keyY = [](std::int64_t value) {
        return static_cast<int>(static_cast<std::uint32_t>(value));
    };

    std::unordered_set<std::int64_t> occupied;
    for (const auto& point : points_base) {
        if (!point.allFinite()) continue;
        occupied.insert(key(
            static_cast<int>(std::floor(point.x() / config.cell_size_m)),
            static_cast<int>(std::floor(point.y() / config.cell_size_m))));
    }
    result.occupied_cells = occupied.size();
    if (occupied.size() < config.minimum_component_cells) {
        result.reason = "sparse_grid";
        return result;
    }

    // Drop isolated single cells before component selection.
    std::unordered_set<std::int64_t> supported;
    for (const std::int64_t cell : occupied) {
        const int x = keyX(cell);
        const int y = keyY(cell);
        bool has_neighbor = false;
        for (int dx = -1; dx <= 1 && !has_neighbor; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if ((dx == 0 && dy == 0) ||
                    occupied.count(key(x + dx, y + dy)) == 0U) continue;
                has_neighbor = true;
                break;
            }
        }
        if (has_neighbor) supported.insert(cell);
    }
    if (supported.size() < config.minimum_component_cells) {
        result.reason = "isolated_grid";
        return result;
    }

    const int anchor_x = static_cast<int>(
        std::floor(anchor_base.x() / config.cell_size_m));
    const int anchor_y = static_cast<int>(
        std::floor(anchor_base.y() / config.cell_size_m));
    std::int64_t seed = *supported.begin();
    int best_distance = std::numeric_limits<int>::max();
    for (const std::int64_t cell : supported) {
        const int distance = std::abs(keyX(cell) - anchor_x) +
            std::abs(keyY(cell) - anchor_y);
        if (distance < best_distance) {
            best_distance = distance;
            seed = cell;
        }
    }

    std::unordered_set<std::int64_t> component;
    std::queue<std::int64_t> pending;
    component.insert(seed);
    pending.push(seed);
    while (!pending.empty()) {
        const std::int64_t cell = pending.front();
        pending.pop();
        const int x = keyX(cell);
        const int y = keyY(cell);
        // A two-cell reach bridges one missing VLP-16 grid cell.
        for (int dx = -2; dx <= 2; ++dx) {
            for (int dy = -2; dy <= 2; ++dy) {
                if (dx == 0 && dy == 0) continue;
                if (std::max(std::abs(dx), std::abs(dy)) > 2) continue;
                const std::int64_t neighbor = key(x + dx, y + dy);
                if (supported.count(neighbor) == 0U ||
                    component.count(neighbor) != 0U) continue;
                component.insert(neighbor);
                pending.push(neighbor);
            }
        }
    }
    result.component_cells = component.size();
    if (component.size() < config.minimum_component_cells) {
        result.reason = "anchor_component_too_small";
        return result;
    }

    const Eigen::Vector2f long_axis(
        std::cos(yaw_base_rad), std::sin(yaw_base_rad));
    const Eigen::Vector2f short_axis(-long_axis.y(), long_axis.x());
    float half_long = 0.0F;
    float half_short = 0.0F;
    for (const std::int64_t cell : component) {
        const Eigen::Vector2f center(
            (static_cast<float>(keyX(cell)) + 0.5F) * config.cell_size_m,
            (static_cast<float>(keyY(cell)) + 0.5F) * config.cell_size_m);
        const Eigen::Vector2f delta = center - anchor_base;
        half_long = std::max(half_long, std::abs(delta.dot(long_axis)));
        half_short = std::max(half_short, std::abs(delta.dot(short_axis)));
    }
    const float cell_radius = 0.707107F * config.cell_size_m;
    float length = 2.0F * (half_long + cell_radius + config.margin_m);
    float width = 2.0F * (half_short + cell_radius + config.margin_m);
    const float long_cap = std::min(
        config.maximum_long_side_m,
        robust_size_long_short.x() * config.maximum_growth_ratio);
    const float short_cap = std::min(
        config.maximum_short_side_m,
        robust_size_long_short.y() * config.maximum_growth_ratio);
    result.clamped = length > long_cap || width > short_cap;
    length = std::min(length, long_cap);
    width = std::min(width, short_cap);
    if (!std::isfinite(length) || !std::isfinite(width) ||
        length <= 0.0F || width <= 0.0F) {
        result.reason = "invalid_grid_extent";
        return result;
    }
    result.valid = true;
    result.size_long_short = Eigen::Vector2f(length, width);
    result.reason = result.clamped
        ? "anchor_grid_extent_clamped"
        : "anchor_grid_extent";
    return result;
}

CargoLiveObbFilter::CargoLiveObbFilter(const CargoLiveObbConfig& config) {
    setConfig(config);
}

void CargoLiveObbFilter::setConfig(const CargoLiveObbConfig& config) {
    config_ = config;
    config_.window_frames = std::max<std::size_t>(10U, config_.window_frames);
    config_.establishment_window_frames = std::clamp<std::size_t>(
        config_.establishment_window_frames, 3U, config_.window_frames);
    config_.minimum_valid_frames = std::clamp<std::size_t>(
        config_.minimum_valid_frames, 1U, config_.window_frames);
    config_.expansion_window_frames = std::clamp<std::size_t>(
        config_.expansion_window_frames, 1U, config_.window_frames);
    config_.expansion_valid_frames = std::clamp<std::size_t>(
        config_.expansion_valid_frames, 1U,
        config_.expansion_window_frames);
    config_.shrink_window_frames = std::clamp<std::size_t>(
        config_.shrink_window_frames, 1U, config_.window_frames);
    config_.shrink_valid_frames = std::clamp<std::size_t>(
        config_.shrink_valid_frames, 1U, config_.shrink_window_frames);
    config_.update_alpha = std::clamp(config_.update_alpha, 0.01F, 1.0F);
    config_.maximum_size_step_m = std::max(
        0.01F, config_.maximum_size_step_m);
    config_.minimum_yaw_concentration = std::clamp(
        config_.minimum_yaw_concentration, 0.0F, 1.0F);
    config_.maximum_yaw_spread_deg = std::max(
        0.0F, config_.maximum_yaw_spread_deg);
    config_.observation_hold_sec = std::max(
        0.0, config_.observation_hold_sec);
    reset();
}

void CargoLiveObbFilter::reset() {
    samples_.clear();
    result_ = CargoLiveObb{};
    last_valid_stamp_sec_ = 0.0;
}

CargoLiveObb CargoLiveObbFilter::update(
    bool observation_valid,
    const Eigen::Vector2f& size_long_short,
    float yaw_base_rad,
    float center_z,
    float height_m,
    double stamp_sec) {
    if (!samples_.empty() &&
        (!std::isfinite(stamp_sec) ||
         stamp_sec + 1.0e-6 < samples_.back().stamp_sec)) {
        reset();
    }
    Sample sample;
    sample.valid = observation_valid && size_long_short.allFinite() &&
        std::isfinite(yaw_base_rad) && std::isfinite(center_z) &&
        std::isfinite(height_m) && size_long_short.minCoeff() > 0.0F &&
        height_m > 0.0F && std::isfinite(stamp_sec);
    sample.size = size_long_short;
    sample.yaw = normalizeCargoAxialYaw(yaw_base_rad);
    sample.center_z = center_z;
    sample.height = height_m;
    sample.stamp_sec = stamp_sec;
    samples_.push_back(sample);
    while (samples_.size() > config_.window_frames) samples_.pop_front();

    std::vector<float> lengths;
    std::vector<float> widths;
    std::vector<float> yaws;
    std::vector<float> centers_z;
    std::vector<float> heights;
    for (const Sample& item : samples_) {
        if (!item.valid) continue;
        lengths.push_back(item.size.x());
        widths.push_back(item.size.y());
        yaws.push_back(item.yaw);
        centers_z.push_back(item.center_z);
        heights.push_back(item.height);
    }
    const auto countRecentValid = [this](std::size_t window) {
        const std::size_t begin = samples_.size() > window
            ? samples_.size() - window : 0U;
        std::size_t count = 0U;
        for (std::size_t index = begin; index < samples_.size(); ++index) {
            if (samples_[index].valid) ++count;
        }
        return count;
    };
    const bool establishment_ready =
        countRecentValid(config_.establishment_window_frames) >=
            config_.minimum_valid_frames;
    if (establishment_ready) {
        const CargoAxialYawSummary yaw_summary =
            summarizeCargoAxialYaw(yaws);
        const float maximum_spread =
            config_.maximum_yaw_spread_deg * 3.14159265358979323846F /
            180.0F;
        if (yaw_summary.valid &&
            yaw_summary.concentration >=
                config_.minimum_yaw_concentration &&
            yaw_summary.maximum_deviation_rad <= maximum_spread) {
            const Eigen::Vector2f median_size(
                percentile(lengths, 0.50F), percentile(widths, 0.50F));
            std::vector<float> length_deviations;
            std::vector<float> width_deviations;
            length_deviations.reserve(lengths.size());
            width_deviations.reserve(widths.size());
            for (const float length : lengths) {
                length_deviations.push_back(
                    std::abs(length - median_size.x()));
            }
            for (const float width : widths) {
                width_deviations.push_back(
                    std::abs(width - median_size.y()));
            }
            const Eigen::Vector2f size_mad(
                percentile(length_deviations, 0.50F),
                percentile(width_deviations, 0.50F));

            // Use MAD to keep an occasional merged wall or isolated return
            // from moving the live box. The 0.10 m floor preserves sparse
            // VLP-16 observations when the MAD itself is nearly zero.
            const Eigen::Vector2f inlier_limit(
                std::max(0.10F, 3.0F * size_mad.x()),
                std::max(0.10F, 3.0F * size_mad.y()));
            std::vector<float> robust_lengths;
            std::vector<float> robust_widths;
            robust_lengths.reserve(lengths.size());
            robust_widths.reserve(widths.size());
            for (const float length : lengths) {
                if (std::abs(length - median_size.x()) <= inlier_limit.x()) {
                    robust_lengths.push_back(length);
                }
            }
            for (const float width : widths) {
                if (std::abs(width - median_size.y()) <= inlier_limit.y()) {
                    robust_widths.push_back(width);
                }
            }
            const Eigen::Vector2f target(
                percentile(robust_lengths, 0.50F),
                percentile(robust_widths, 0.50F));
            const float target_z = percentile(centers_z, 0.50F);
            const float target_height = percentile(heights, 0.50F);
            const bool expansion = result_.valid &&
                (target.x() > result_.size_long_short.x() ||
                 target.y() > result_.size_long_short.y());
            const bool shrink = result_.valid && !expansion &&
                (target.x() < result_.size_long_short.x() ||
                 target.y() < result_.size_long_short.y());
            const bool update_ready = !result_.valid ||
                (expansion &&
                 countRecentValid(config_.expansion_window_frames) >=
                     config_.expansion_valid_frames) ||
                (shrink &&
                 countRecentValid(config_.shrink_window_frames) >=
                     config_.shrink_valid_frames) ||
                (!expansion && !shrink);
            if (!update_ready) {
                result_.held = result_.valid;
                return result_;
            }
            if (!result_.valid) {
                result_.size_long_short = target;
                result_.center_z = target_z;
                result_.height_m = target_height;
                result_.yaw_base_rad = yaw_summary.mean_yaw_rad;
            } else {
                Eigen::Vector2f delta =
                    config_.update_alpha *
                    (target - result_.size_long_short);
                delta.x() = std::clamp(
                    delta.x(), -config_.maximum_size_step_m,
                    config_.maximum_size_step_m);
                delta.y() = std::clamp(
                    delta.y(), -config_.maximum_size_step_m,
                    config_.maximum_size_step_m);
                result_.size_long_short += delta;
                result_.center_z += config_.update_alpha *
                    (target_z - result_.center_z);
                result_.height_m += config_.update_alpha *
                    (target_height - result_.height_m);
                const float yaw_delta = normalizeCargoAxialYaw(
                    yaw_summary.mean_yaw_rad - result_.yaw_base_rad);
                result_.yaw_base_rad = normalizeCargoAxialYaw(
                    result_.yaw_base_rad +
                    config_.update_alpha * yaw_delta);
            }
            result_.valid = true;
            result_.held = false;
            result_.valid_frames = lengths.size();
            result_.yaw_concentration = yaw_summary.concentration;
            result_.size_mad = size_mad;
            last_valid_stamp_sec_ = stamp_sec;
            return result_;
        }
    }

    if (result_.valid && std::isfinite(stamp_sec) &&
        stamp_sec - last_valid_stamp_sec_ <=
            config_.observation_hold_sec) {
        result_.held = true;
        return result_;
    }
    result_ = CargoLiveObb{};
    return result_;
}

}  // namespace ndt_slam
