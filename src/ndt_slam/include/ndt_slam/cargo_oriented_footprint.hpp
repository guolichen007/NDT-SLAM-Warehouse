#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace ndt_slam {

struct CargoOrientedFootprintConfig {
    std::size_t minimum_points = 20U;
    float percentile_low = 0.08F;
    float percentile_high = 0.92F;
    float margin_m = 0.05F;
    float minimum_geometric_aspect_ratio = 1.20F;
    float minimum_eigenvalue_ratio = 1.44F;
    float minimum_long_side_m = 0.60F;
    float minimum_short_side_m = 0.40F;
    float maximum_long_side_m = 2.50F;
    float maximum_short_side_m = 1.60F;
};

struct CargoOrientedFootprint {
    bool valid = false;
    Eigen::Vector2f center_base = Eigen::Vector2f::Zero();
    Eigen::Vector2f size_long_short = Eigen::Vector2f::Zero();
    float yaw_base_rad = 0.0F;
    float eigenvalue_ratio = 1.0F;
    float geometric_aspect_ratio = 1.0F;
    float orientation_confidence = 0.0F;
    std::size_t finite_points = 0U;
    std::string reason = "not_estimated";
};

// Estimates a robust hook-centred OBB. yaw_base_rad is an axial angle: yaw and
// yaw + pi describe the same suspended-cargo footprint.
CargoOrientedFootprint estimateCargoOrientedFootprint(
    const std::vector<Eigen::Vector2f>& points_base,
    const CargoOrientedFootprintConfig& config = {});

struct CargoAxialYawSummary {
    bool valid = false;
    float mean_yaw_rad = 0.0F;
    float concentration = 0.0F;
    float maximum_deviation_rad = 0.0F;
    std::size_t sample_count = 0U;
};

CargoAxialYawSummary summarizeCargoAxialYaw(
    const std::vector<float>& yaw_samples);

// Circular mean for axial observations. It uses doubled angles so samples near
// +90 and -90 degrees do not incorrectly average to zero.
bool meanCargoAxialYaw(const std::vector<float>& yaw_samples,
                       float* mean_yaw_rad);

float normalizeCargoAxialYaw(float yaw_rad);

}  // namespace ndt_slam
