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
    float maximum_range_m = 0.15F;
};

struct ExternalGroundEstimate {
    bool valid = false;
    float z_m = std::numeric_limits<float>::quiet_NaN();
    std::size_t cells = 0U;
    float range_m = std::numeric_limits<float>::infinity();
};

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
    std::map<std::pair<int, int>, float> cell_minima;
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
        const auto found = cell_minima.find(key);
        if (found == cell_minima.end() || point.z < found->second) {
            cell_minima[key] = point.z;
        }
    }
    std::vector<float> minima;
    minima.reserve(cell_minima.size());
    for (const auto& entry : cell_minima) minima.push_back(entry.second);
    estimate.cells = minima.size();
    if (minima.size() < static_cast<std::size_t>(
            std::max(1, config.minimum_cells))) return estimate;
    std::sort(minima.begin(), minima.end());
    estimate.range_m = minima.back() - minima.front();
    if (!std::isfinite(estimate.range_m) ||
        estimate.range_m > config.maximum_range_m) return estimate;
    estimate.z_m = minima[minima.size() / 2U];
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
