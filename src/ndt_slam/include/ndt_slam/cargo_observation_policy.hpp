#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ndt_slam {

struct ExternalGroundConfig {
    float ring_width_m = 2.0F;
    float cell_size_m = 0.50F;
    int minimum_cells = 4;
    int minimum_points_per_cell = 3;
    int minimum_quadrants = 3;
    bool allow_opposite_sides = true;
    float maximum_range_m = 0.15F;
    bool expected_height_enabled = false;
    float expected_height_m = 0.0F;
    float maximum_expected_height_delta_m = 0.30F;
};

struct ExternalGroundEstimate {
    bool valid = false;
    float z_m = std::numeric_limits<float>::quiet_NaN();
    std::size_t cells = 0U;
    std::size_t points = 0U;
    int quadrants = 0;
    bool has_opposite_sides = false;
    float range_m = std::numeric_limits<float>::infinity();
};

inline float minimumFiniteZ(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    float minimum = std::numeric_limits<float>::infinity();
    for (const auto& point : cloud.points) {
        if (std::isfinite(point.z)) minimum = std::min(minimum, point.z);
    }
    return minimum;
}

inline ExternalGroundEstimate estimateExternalGround(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    float center_x, float center_y,
    float payload_half_x, float payload_half_y,
    const ExternalGroundConfig& config) {
    ExternalGroundEstimate estimate;
    const float cell = std::max(0.20F, config.cell_size_m);
    const float outer_x = payload_half_x +
        std::max(0.50F, config.ring_width_m);
    const float outer_y = payload_half_y +
        std::max(0.50F, config.ring_width_m);
    struct CellSamples {
        std::vector<float> z_values;
        float sum_x = 0.0F;
        float sum_y = 0.0F;
    };
    std::map<std::pair<int, int>, CellSamples> cell_samples;
    for (const auto& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) continue;
        const float dx = std::abs(point.x - center_x);
        const float dy = std::abs(point.y - center_y);
        if ((dx <= payload_half_x && dy <= payload_half_y) ||
            dx > outer_x || dy > outer_y) continue;
        const std::pair<int, int> key{
            static_cast<int>(std::floor((point.x - center_x) / cell)),
            static_cast<int>(std::floor((point.y - center_y) / cell))};
        auto& samples = cell_samples[key];
        samples.z_values.push_back(point.z);
        samples.sum_x += point.x - center_x;
        samples.sum_y += point.y - center_y;
    }
    std::vector<float> minima;
    minima.reserve(cell_samples.size());
    int quadrant_mask = 0;
    int side_mask = 0;
    const std::size_t minimum_points = static_cast<std::size_t>(
        std::max(1, config.minimum_points_per_cell));
    for (auto& entry : cell_samples) {
        auto& samples = entry.second;
        if (samples.z_values.size() < minimum_points) continue;
        std::sort(samples.z_values.begin(), samples.z_values.end());
        // The second-lowest sample rejects one isolated negative outlier while
        // retaining the low surface needed to ignore overhanging objects.
        const std::size_t robust_low_index =
            samples.z_values.size() > 1U ? 1U : 0U;
        minima.push_back(samples.z_values[robust_low_index]);
        estimate.points += samples.z_values.size();

        const float mean_dx = samples.sum_x /
            static_cast<float>(samples.z_values.size());
        const float mean_dy = samples.sum_y /
            static_cast<float>(samples.z_values.size());
        const int quadrant = mean_dx >= 0.0F
            ? (mean_dy >= 0.0F ? 0 : 3)
            : (mean_dy >= 0.0F ? 1 : 2);
        quadrant_mask |= (1 << quadrant);
        if (std::abs(mean_dx) >= std::abs(mean_dy)) {
            side_mask |= mean_dx >= 0.0F ? 0x1 : 0x2;
        } else {
            side_mask |= mean_dy >= 0.0F ? 0x4 : 0x8;
        }
    }
    estimate.cells = minima.size();
    if (minima.size() < static_cast<std::size_t>(
            std::max(1, config.minimum_cells))) return estimate;
    for (int bit = 0; bit < 4; ++bit) {
        if ((quadrant_mask & (1 << bit)) != 0) ++estimate.quadrants;
    }
    estimate.has_opposite_sides =
        ((side_mask & 0x3) == 0x3) || ((side_mask & 0xC) == 0xC);
    const bool spatially_distributed =
        estimate.quadrants >= std::max(1, config.minimum_quadrants) ||
        (config.allow_opposite_sides && estimate.has_opposite_sides);
    if (!spatially_distributed) return estimate;
    std::sort(minima.begin(), minima.end());
    estimate.range_m = minima.back() - minima.front();
    if (!std::isfinite(estimate.range_m) ||
        estimate.range_m > config.maximum_range_m) return estimate;
    const std::size_t middle = minima.size() / 2U;
    estimate.z_m = minima.size() % 2U == 0U
        ? 0.5F * (minima[middle - 1U] + minima[middle])
        : minima[middle];
    if (config.expected_height_enabled &&
        std::abs(estimate.z_m - config.expected_height_m) >
            std::max(0.0F, config.maximum_expected_height_delta_m)) {
        estimate.z_m = std::numeric_limits<float>::quiet_NaN();
        return estimate;
    }
    estimate.valid = std::isfinite(estimate.z_m);
    return estimate;
}

enum class CargoLockProfile { NONE, LARGE_BODY, COMPACT_BODY };

inline CargoLockProfile classifyCargoLockProfile(
    std::size_t points, float visible_height_m, float xy_area_m2,
    bool hook_loaded,
    int large_points, float large_height_m, float large_area_m2,
    bool compact_enabled, int compact_points,
    float compact_height_m, float compact_area_m2) {
    if (points >= static_cast<std::size_t>(std::max(1, large_points)) &&
        visible_height_m >= large_height_m && xy_area_m2 >= large_area_m2) {
        return CargoLockProfile::LARGE_BODY;
    }
    if (compact_enabled && hook_loaded &&
        points >= static_cast<std::size_t>(std::max(1, compact_points)) &&
        visible_height_m >= compact_height_m &&
        xy_area_m2 >= compact_area_m2) {
        return CargoLockProfile::COMPACT_BODY;
    }
    return CargoLockProfile::NONE;
}

}  // namespace ndt_slam
