#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ndt_slam {

/** Axis-aligned cargo bounds expressed in the base frame. */
struct CargoBaseFootprint {
    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;

    // The vertical bounds are used only to reject returns from the cargo itself.
    float min_z = 0.0f;
    float max_z = 0.0f;
};

/** Latest fused cargo-bottom estimate supplied to the safety evaluator. */
struct CargoHeightState {
    bool valid = false;
    bool stale = false;
    float bottom_z = std::numeric_limits<float>::quiet_NaN();
    float bottom_uncertainty_m = std::numeric_limits<float>::quiet_NaN();
    double stamp_sec = std::numeric_limits<double>::quiet_NaN();
};

struct CargoSafetyConfig {
    float level1_distance_m = 3.0f;
    float level2_distance_m = 5.0f;
    float minimum_vertical_clearance_m = 0.80f;

    float cargo_bottom_extra_margin_m = 0.05f;
    double maximum_height_age_sec = 0.50;
    double future_stamp_tolerance_sec = 0.05;

    float obstacle_top_percentile = 0.95f;
    float obstacle_uncertainty_floor_m = 0.05f;
    float obstacle_uncertainty_max_m = 0.30f;
    float obstacle_cluster_tolerance_m = 0.25f;
    std::size_t obstacle_min_cluster_points = 5;
    std::size_t obstacle_max_cluster_points = 10000;

    bool exclude_self_cargo = true;
    float self_cargo_margin_x_m = 0.45f;
    float self_cargo_margin_y_m = 0.45f;
    float self_cargo_margin_z_m = 0.35f;
};

struct CargoSafetyInput {
    CargoHeightState height;
    CargoBaseFootprint footprint_base;
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr obstacle_cloud_base;
    double evaluation_time_sec = std::numeric_limits<double>::quiet_NaN();
};

/** Evidence calculated entirely from one Euclidean obstacle cluster. */
struct CargoSafetyClusterEvidence {
    bool valid = false;
    std::size_t cluster_index = 0;
    std::size_t point_count = 0;
    std::uint16_t raw_code = 14;

    float footprint_distance_m = std::numeric_limits<float>::infinity();
    float obstacle_top_z95_m = std::numeric_limits<float>::quiet_NaN();
    float obstacle_max_z_m = std::numeric_limits<float>::quiet_NaN();
    float obstacle_tail_spread_m = std::numeric_limits<float>::quiet_NaN();
    float obstacle_uncertainty_m = std::numeric_limits<float>::quiet_NaN();
    float conservative_clearance_m = std::numeric_limits<float>::quiet_NaN();

    pcl::PointXYZ nearest_point_base;
    std::string uncertainty_reason;
};

struct CargoSafetyResult {
    // Raw PLC contract: 14=safe, 17=hazard within level 1, 18=level 2/fail-safe.
    std::uint16_t raw_code = 18;
    bool input_valid = false;
    bool height_stale = false;
    bool has_cluster_evidence = false;
    double height_age_sec = std::numeric_limits<double>::quiet_NaN();

    std::size_t finite_input_points = 0;
    std::size_t self_cargo_points_removed = 0;
    std::size_t evaluated_cluster_count = 0;

    CargoSafetyClusterEvidence most_dangerous_cluster;
    std::string reason = "invalid_input";
};

/**
 * Stateless raw safety evaluator.
 *
 * Debouncing, clear-hold timing and publication policy deliberately belong to
 * the caller.  Every evaluate() call depends only on its argument and config.
 */
class CargoSafetyEvaluator {
public:
    static constexpr std::uint16_t kSafeCode = 14;
    static constexpr std::uint16_t kLevel1Code = 17;
    static constexpr std::uint16_t kLevel2OrFailSafeCode = 18;

    explicit CargoSafetyEvaluator(const CargoSafetyConfig& config = CargoSafetyConfig());

    void setConfig(const CargoSafetyConfig& config);
    const CargoSafetyConfig& config() const;

    CargoSafetyResult evaluate(const CargoSafetyInput& input) const;

private:
    CargoSafetyConfig config_;
};

}  // namespace ndt_slam
