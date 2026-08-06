#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

namespace ndt_slam {

struct CargoOrientedFootprintConfig {
    std::size_t minimum_points = 20U;
    float percentile_low = 0.03F;
    float percentile_high = 0.97F;
    float margin_m = 0.10F;
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
    Eigen::Vector2f raw_size_long_short = Eigen::Vector2f::Zero();
    bool long_side_clamped = false;
    bool short_side_clamped = false;
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

struct CargoAnchorGridFootprintConfig {
    float cell_size_m = 0.05F;
    float margin_m = 0.10F;
    float maximum_growth_ratio = 1.20F;
    float maximum_long_side_m = 3.50F;
    float maximum_short_side_m = 2.00F;
    std::size_t minimum_component_cells = 3U;
};

struct CargoAnchorGridFootprint {
    bool valid = false;
    bool clamped = false;
    Eigen::Vector2f size_long_short = Eigen::Vector2f::Zero();
    std::size_t occupied_cells = 0U;
    std::size_t component_cells = 0U;
    std::string reason = "not_refined";
};

// Refines an anchor-centred footprint from a connected 5 cm occupancy grid.
// Isolated returns and disconnected wall cells cannot enlarge the result.
CargoAnchorGridFootprint refineCargoAnchorGridFootprint(
    const std::vector<Eigen::Vector2f>& points_base,
    const Eigen::Vector2f& anchor_base,
    float yaw_base_rad,
    const Eigen::Vector2f& robust_size_long_short,
    const CargoAnchorGridFootprintConfig& config = {});

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

float normalizeAxialYaw(float yaw_rad);
float baseYawToMap(float yaw_base, float base_pose_yaw_map);
float mapYawToBase(float yaw_map, float base_pose_yaw_map);

struct CargoLiveObbConfig {
    std::size_t window_frames = 10U;
    std::size_t establishment_window_frames = 5U;
    std::size_t minimum_valid_frames = 3U;
    std::size_t expansion_window_frames = 8U;
    std::size_t expansion_valid_frames = 5U;
    std::size_t shrink_window_frames = 10U;
    std::size_t shrink_valid_frames = 7U;
    float update_alpha = 0.30F;
    float maximum_size_step_m = 0.10F;
    float minimum_yaw_concentration = 0.70F;
    float maximum_yaw_spread_deg = 12.0F;
    double observation_hold_sec = 2.0;
};

struct CargoLiveObb {
    bool valid = false;
    bool held = false;
    Eigen::Vector2f size_long_short = Eigen::Vector2f::Zero();
    float yaw_base_rad = 0.0F;
    float center_z = 0.0F;
    float height_m = 0.0F;
    std::size_t valid_frames = 0U;
    float yaw_concentration = 0.0F;
    Eigen::Vector2f size_mad = Eigen::Vector2f::Zero();
};

class CargoLiveObbFilter {
public:
    explicit CargoLiveObbFilter(const CargoLiveObbConfig& config = {});
    void setConfig(const CargoLiveObbConfig& config);
    void reset();
    CargoLiveObb update(bool observation_valid,
                        const Eigen::Vector2f& size_long_short,
                        float yaw_base_rad,
                        float center_z,
                        float height_m,
                        double stamp_sec);
    const CargoLiveObb& result() const { return result_; }

private:
    struct Sample {
        bool valid = false;
        Eigen::Vector2f size = Eigen::Vector2f::Zero();
        float yaw = 0.0F;
        float center_z = 0.0F;
        float height = 0.0F;
        double stamp_sec = 0.0;
    };
    CargoLiveObbConfig config_;
    std::deque<Sample> samples_;
    CargoLiveObb result_;
    double last_valid_stamp_sec_ = 0.0;
};

}  // namespace ndt_slam
