#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ndt_slam/hook_load_evidence_policy.hpp"
#include "ndt_slam/cargo_config_validation.hpp"
#include "ndt_slam/cargo_rigid_geometry.hpp"
#include "ndt_slam/obstacle_perception.hpp"

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
    double future_stamp_tolerance_sec = 0.05;
    double maximum_obstacle_cloud_age_sec = 0.50;
    std::size_t minimum_roi_finite_points = 20;
    float minimum_roi_coverage_ratio = 0.05F;

    float obstacle_top_percentile = 0.95f;
    float obstacle_bottom_percentile = 0.05f;
    float obstacle_vertical_bin_size_m = 0.10f;
    float obstacle_min_vertical_continuity_ratio = 0.45f;
    float overhead_separation_margin_m = 0.10f;
    float obstacle_uncertainty_floor_m = 0.05f;
    float obstacle_uncertainty_max_m = 0.30f;
    float obstacle_cluster_tolerance_m = 0.25f;
    std::size_t obstacle_min_cluster_points = 5;
    std::size_t obstacle_max_cluster_points = 10000;

};

struct CargoSafetyInput {
    double source_stamp_sec = 0.0;
    std::uint64_t source_sequence = 0U;
    std::string frame_id = "base_link";
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
    float obstacle_bottom_z05_m = std::numeric_limits<float>::quiet_NaN();
    float obstacle_min_z_m = std::numeric_limits<float>::quiet_NaN();
    float obstacle_max_z_m = std::numeric_limits<float>::quiet_NaN();
    float obstacle_vertical_span_m = std::numeric_limits<float>::quiet_NaN();
    float vertical_continuity_ratio = 0.0F;
    bool entirely_above_cargo = false;
    float obstacle_tail_spread_m = std::numeric_limits<float>::quiet_NaN();
    float obstacle_uncertainty_m = std::numeric_limits<float>::quiet_NaN();
    float conservative_clearance_m = std::numeric_limits<float>::quiet_NaN();

    pcl::PointXYZ centroid_base;
    pcl::PointXYZ nearest_point_base;
    std::vector<int> point_indices;
    bool source_validated = true;
    float inside_xy_obb_ratio = 0.0F;
    float identity_match_ratio = 0.0F;
    float surface_band_ratio = 0.0F;
    float moves_with_cargo_score = 0.0F;
    std::string source_reason = "ordinary_external_cluster";
    std::string uncertainty_reason;
};

enum class CargoSafetyFault : std::uint8_t {
    NONE = 0,
    CARGO_HEIGHT_INVALID,
    OBSTACLE_EVIDENCE_INVALID,
    INTERNAL_ERROR
};

enum class CargoSafetyEvidenceState : std::uint8_t {
    UNKNOWN = 0,
    CLEAR,
    HAZARD_CONFIRMED,
    HAZARD_CANDIDATE,
    TRACK_CONFIRMATION_PENDING,
    SPARSE_PENDING,
    SOURCE_UNRESOLVED,
    HARD_FAULT,
    REVIEW_REQUIRED
};

struct CargoSafetyProtocol {
    static constexpr std::int32_t kClear = 14;
    static constexpr std::int32_t kLevel1Warning = 17;
    static constexpr std::int32_t kLevel2Warning = 18;
    static constexpr std::int32_t kAnomalyReview = 29;
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

enum class CargoSafetyDecisionPhase : std::uint8_t {
    STARTUP_NOT_READY = 0,
    LOCALIZATION_INVALID,
    GRAVITY_REQUIRED_INVALID,
    CARGO_EXPECTED_NOT_AUTHORITATIVE,
    CARGO_FORMAL_OBSTACLE_NOT_READY,
    SAFE_EMPTY,
    VALID_WARNING_OR_CLEAR,
    INTERNAL_CONTRACT_ERROR,
};

struct CargoSafetyDecisionInput {
    bool system_ready = false;
    bool localization_valid = false;
    HookLoadSignalRole hook_signal_role = HookLoadSignalRole::REQUIRED;
    bool gravity_valid = false;
    bool gravity_empty = false;
    bool gravity_loaded = false;
    bool gravity_conflict = false;
    bool safe_empty = false;
    bool hook_loaded = false;
    bool cargo_fault = false;
    bool obstacle_fault = false;
    bool internal_fault = false;
    bool warning_valid = false;
    bool pending_positive_warning = false;
    bool formal_cargo_valid = false;
    bool formal_clear_authorized = false;
    bool obstacle_evidence_ready = false;
    std::uint16_t warning_code = 0;
    std::string evidence_reason;
};

struct CargoSafetyPhaseSelection {
    CargoSafetyDecisionPhase phase =
        CargoSafetyDecisionPhase::STARTUP_NOT_READY;
    std::string reason;
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
CargoSafetyPhaseSelection deriveCargoSafetyDecisionPhase(
    const CargoSafetyDecisionInput& input);
bool cargoSafetyDecisionSelfConsistent(
    const CargoSafetyDecision& decision);

struct CargoSafetyResult {
    bool input_valid = false;
    bool warning_valid = false;
    std::uint16_t warning_code = 0;
    CargoSafetyFault fault = CargoSafetyFault::NONE;
    CargoSafetyEvidenceState evidence_state =
        CargoSafetyEvidenceState::HARD_FAULT;
    bool height_stale = false;
    bool has_cluster_evidence = false;
    double height_age_sec = std::numeric_limits<double>::quiet_NaN();

    std::size_t finite_input_points = 0;
    // Always zero: runtime identity/motion classification is the sole
    // authority allowed to remove cargo returns before this evaluator.
    std::size_t self_cargo_points_removed = 0;
    std::size_t evaluated_cluster_count = 0;

    CargoSafetyClusterEvidence most_dangerous_cluster;
    std::vector<CargoSafetyClusterEvidence> cluster_evidence;
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
    static constexpr std::uint16_t kReviewCode = 29;

    explicit CargoSafetyEvaluator(const CargoSafetyConfig& config = CargoSafetyConfig());

    CargoConfigValidationResult setConfig(const CargoSafetyConfig& config);
    const CargoSafetyConfig& config() const;
    const CargoConfigValidationResult& configValidation() const noexcept {
        return config_validation_;
    }

    CargoSafetyResult evaluate(const CargoSafetyInput& input) const;

    // Reuses a canonical physical-perception result. This prevents Formal and
    // Pending authority projections from clustering the same physical frame
    // more than once.
    CargoSafetyResult evaluate(
        const CargoSafetyInput& input,
        const ObstaclePerceptionResult& perception) const;

    // Physical external-cluster perception deliberately does not require a
    // Cargo bottom. Callers may keep obstacle identity/far-history alive while
    // evaluate() remains fail-closed for 14/17/18.
    ObstaclePerceptionResult perceive(const CargoSafetyInput& input) const;

private:
    CargoSafetyConfig config_;
    CargoConfigValidationResult config_validation_;
};

CargoConfigValidationResult validateCargoSafetyConfig(
    const CargoSafetyConfig& config);

}  // namespace ndt_slam
