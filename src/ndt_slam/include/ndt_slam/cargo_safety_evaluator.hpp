#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ndt_slam/hook_load_evidence_policy.hpp"
#include "ndt_slam/cargo_rigid_geometry.hpp"

namespace ndt_slam {

using CargoBaseFootprint = CargoObbFootprint;

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
    double maximum_obstacle_cloud_age_sec = 0.50;
    std::size_t minimum_roi_finite_points = 20;
    float minimum_roi_coverage_ratio = 0.05F;

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
    bool obstacle_observation_valid = false;
    double obstacle_cloud_age_sec = std::numeric_limits<double>::infinity();
    std::size_t obstacle_roi_finite_points = 0;
    float obstacle_roi_coverage_ratio = 0.0f;
};

/** Evidence calculated entirely from one Euclidean obstacle cluster. */
struct CargoSafetyClusterEvidence {
    bool valid = false;
    std::size_t cluster_index = 0;
    std::size_t point_count = 0;
    std::uint16_t warning_code = 14;

    float footprint_distance_m = std::numeric_limits<float>::infinity();
    float obstacle_top_z95_m = std::numeric_limits<float>::quiet_NaN();
    float obstacle_max_z_m = std::numeric_limits<float>::quiet_NaN();
    float obstacle_tail_spread_m = std::numeric_limits<float>::quiet_NaN();
    float obstacle_uncertainty_m = std::numeric_limits<float>::quiet_NaN();
    float conservative_clearance_m = std::numeric_limits<float>::quiet_NaN();

    pcl::PointXYZ centroid_base;
    pcl::PointXYZ nearest_point_base;
    std::string uncertainty_reason;
};

enum class CargoSafetyFault : std::uint8_t {
    NONE = 0,
    CARGO_HEIGHT_INVALID,
    OBSTACLE_EVIDENCE_INVALID,
    INTERNAL_ERROR
};

struct CargoSafetyProtocol {
    static constexpr std::int32_t kClear = 14;
    static constexpr std::int32_t kLevel1Warning = 17;
    static constexpr std::int32_t kLevel2Warning = 18;
    static constexpr std::int32_t kSystemNotReady = 30;
    static constexpr std::int32_t kLocalizationInvalid = 31;
    static constexpr std::int32_t kGravityInvalid = 32;
    static constexpr std::int32_t kCargoInvalid = 33;
    static constexpr std::int32_t kObstacleInvalid = 34;
    static constexpr std::int32_t kInternalError = 35;

    static constexpr std::uint32_t kFaultStatusStale = 1U;
    static constexpr std::uint32_t kFaultLocalization = 2U;
    static constexpr std::uint32_t kFaultGravity = 4U;
    static constexpr std::uint32_t kFaultCargo = 8U;
    static constexpr std::uint32_t kFaultObstacle = 16U;
    static constexpr std::uint32_t kFaultInternal = 32U;
};

struct CargoSafetyDecisionInput {
    bool system_ready = false;
    bool localization_valid = false;
    HookLoadSignalRole hook_signal_role = HookLoadSignalRole::REQUIRED;
    bool gravity_valid = false;
    bool gravity_conflict = false;
    bool safe_empty = false;
    bool hook_loaded = false;
    bool cargo_fault = false;
    bool obstacle_fault = false;
    bool internal_fault = false;
    bool warning_valid = false;
    std::uint16_t warning_code = 0;
    std::string evidence_reason;
};

struct CargoSafetyDecision {
    bool valid = false;
    bool warning_valid = false;
    std::int32_t requested_code = CargoSafetyProtocol::kSystemNotReady;
    std::int32_t warning_code = 0;
    std::int32_t fault_code = CargoSafetyProtocol::kSystemNotReady;
    std::uint32_t fault_mask = CargoSafetyProtocol::kFaultStatusStale;
    std::string reason = "system_not_ready";
};

CargoSafetyDecision composeCargoSafetyDecision(
    const CargoSafetyDecisionInput& input);

struct CargoSafetyResult {
    bool input_valid = false;
    bool warning_valid = false;
    std::uint16_t warning_code = 0;
    CargoSafetyFault fault = CargoSafetyFault::NONE;
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
    static constexpr std::uint16_t kLevel2Code = 18;

    explicit CargoSafetyEvaluator(const CargoSafetyConfig& config = CargoSafetyConfig());

    void setConfig(const CargoSafetyConfig& config);
    const CargoSafetyConfig& config() const;

    CargoSafetyResult evaluate(const CargoSafetyInput& input) const;

private:
    CargoSafetyConfig config_;
};

}  // namespace ndt_slam
