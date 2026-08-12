#include <ndt_slam/cargo_safety_evaluator.hpp>
#include <ndt_slam/hazard_evaluator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

bool isFinite(float value) {
    return std::isfinite(value);
}

bool isFinite(double value) {
    return std::isfinite(value);
}

bool isValidConfig(const CargoSafetyConfig& config) {
    return validateCargoSafetyConfig(config).valid;
}

bool isValidFootprint(const CargoBaseFootprint& footprint) {
    return footprint.valid && footprint.center_base.allFinite() &&
           isFinite(footprint.length_m) && footprint.length_m > 0.0F &&
           isFinite(footprint.width_m) && footprint.width_m > 0.0F &&
           isFinite(footprint.yaw_base_rad) &&
           isFinite(footprint.min_z) && isFinite(footprint.max_z) &&
           footprint.min_z < footprint.max_z;
}

int warningPriority(std::uint16_t warning_code) {
    if (warning_code == CargoSafetyEvaluator::kLevel1Code) {
        return 3;
    }
    if (warning_code == CargoSafetyEvaluator::kLevel2Code) {
        return 2;
    }
    return 1;
}

bool isMoreDangerous(const CargoSafetyClusterEvidence& candidate,
                     const CargoSafetyClusterEvidence& current) {
    const int candidate_priority = warningPriority(candidate.warning_code);
    const int current_priority = warningPriority(current.warning_code);
    if (candidate_priority != current_priority) {
        return candidate_priority > current_priority;
    }
    if (candidate.conservative_clearance_m != current.conservative_clearance_m) {
        return candidate.conservative_clearance_m < current.conservative_clearance_m;
    }
    if (candidate.footprint_distance_m != current.footprint_distance_m) {
        return candidate.footprint_distance_m < current.footprint_distance_m;
    }
    return candidate.cluster_index < current.cluster_index;
}

}  // namespace

CargoConfigValidationResult validateCargoSafetyConfig(
    const CargoSafetyConfig& config) {
    CargoConfigValidationResult result;
    const auto finite_nonnegative = [&result](
        const char* field, double value) {
        if (!std::isfinite(value)) result.reject(field, "non_finite");
        else if (value < 0.0) result.reject(field, "must_be_nonnegative");
    };
    finite_nonnegative("level1_distance_m", config.level1_distance_m);
    finite_nonnegative("level2_distance_m", config.level2_distance_m);
    if (std::isfinite(config.level1_distance_m) &&
        std::isfinite(config.level2_distance_m) &&
        config.level2_distance_m < config.level1_distance_m) {
        result.reject("level2_distance_m", "must_be_at_least_level1");
    }
    if (!std::isfinite(config.minimum_vertical_clearance_m))
        result.reject("minimum_vertical_clearance_m", "non_finite");
    finite_nonnegative("cargo_bottom_extra_margin_m",
                       config.cargo_bottom_extra_margin_m);
    finite_nonnegative("future_stamp_tolerance_sec",
                       config.future_stamp_tolerance_sec);
    finite_nonnegative("maximum_obstacle_cloud_age_sec",
                       config.maximum_obstacle_cloud_age_sec);
    if (config.minimum_roi_finite_points == 0U)
        result.reject("minimum_roi_finite_points", "must_be_positive");
    const auto ratio = [&result](const char* field, double value) {
        if (!std::isfinite(value)) result.reject(field, "non_finite");
        else if (value < 0.0 || value > 1.0)
            result.reject(field, "outside_0_1");
    };
    ratio("minimum_roi_coverage_ratio", config.minimum_roi_coverage_ratio);
    ratio("obstacle_top_percentile", config.obstacle_top_percentile);
    ratio("obstacle_bottom_percentile", config.obstacle_bottom_percentile);
    if (std::isfinite(config.obstacle_top_percentile) &&
        std::isfinite(config.obstacle_bottom_percentile) &&
        config.obstacle_bottom_percentile >= config.obstacle_top_percentile) {
        result.reject("obstacle_bottom_percentile", "must_be_below_top");
    }
    if (!std::isfinite(config.obstacle_vertical_bin_size_m) ||
        config.obstacle_vertical_bin_size_m <= 0.0F)
        result.reject("obstacle_vertical_bin_size_m", "must_be_positive");
    ratio("obstacle_min_vertical_continuity_ratio",
          config.obstacle_min_vertical_continuity_ratio);
    finite_nonnegative("overhead_separation_margin_m",
                       config.overhead_separation_margin_m);
    finite_nonnegative("obstacle_uncertainty_floor_m",
                       config.obstacle_uncertainty_floor_m);
    finite_nonnegative("obstacle_uncertainty_max_m",
                       config.obstacle_uncertainty_max_m);
    if (std::isfinite(config.obstacle_uncertainty_floor_m) &&
        std::isfinite(config.obstacle_uncertainty_max_m) &&
        config.obstacle_uncertainty_max_m <
            config.obstacle_uncertainty_floor_m)
        result.reject("obstacle_uncertainty_max_m", "below_floor");
    if (!std::isfinite(config.obstacle_cluster_tolerance_m) ||
        config.obstacle_cluster_tolerance_m <= 0.0F)
        result.reject("obstacle_cluster_tolerance_m", "must_be_positive");
    if (config.obstacle_min_cluster_points == 0U)
        result.reject("obstacle_min_cluster_points", "must_be_positive");
    if (config.obstacle_max_cluster_points <
        config.obstacle_min_cluster_points)
        result.reject("obstacle_max_cluster_points", "below_minimum");
    if (config.obstacle_max_cluster_points >
        static_cast<std::size_t>(std::numeric_limits<int>::max()))
        result.reject("obstacle_max_cluster_points", "exceeds_int_range");
    return result;
}

CargoSafetyEvaluator::CargoSafetyEvaluator(const CargoSafetyConfig& config) {
    setConfig(config);
}

CargoConfigValidationResult CargoSafetyEvaluator::setConfig(
    const CargoSafetyConfig& config) {
    config_ = config;
    config_validation_ = validateCargoSafetyConfig(config_);
    return config_validation_;
}

const CargoSafetyConfig& CargoSafetyEvaluator::config() const {
    return config_;
}

CargoSafetyResult CargoSafetyEvaluator::evaluate(const CargoSafetyInput& input) const {
    return evaluate(input, perceive(input));
}

CargoSafetyResult CargoSafetyEvaluator::evaluate(
    const CargoSafetyInput& input,
    const ObstaclePerceptionResult& perception) const {
    CargoSafetyResult result;

    if (!isValidConfig(config_)) {
        result.fault = CargoSafetyFault::INTERNAL_ERROR;
        result.reason = "invalid_config";
        return result;
    }
    if (!isFinite(input.evaluation_time_sec) || !input.obstacle_cloud_base) {
        result.fault = CargoSafetyFault::INTERNAL_ERROR;
        result.reason = "invalid_input";
        return result;
    }
    if (!input.height.valid) {
        result.fault = CargoSafetyFault::CARGO_HEIGHT_INVALID;
        result.reason = "height_invalid";
        return result;
    }
    if (!isFinite(input.height.bottom_z) ||
        !isFinite(input.height.bottom_uncertainty_m) ||
        input.height.bottom_uncertainty_m < 0.0f ||
        !isFinite(input.height.stamp_sec)) {
        result.fault = CargoSafetyFault::CARGO_HEIGHT_INVALID;
        result.reason = "invalid_height_values";
        return result;
    }

    result.height_age_sec = input.evaluation_time_sec - input.height.stamp_sec;
    if (result.height_age_sec < -config_.future_stamp_tolerance_sec) {
        result.fault = CargoSafetyFault::CARGO_HEIGHT_INVALID;
        result.reason = "height_timestamp_in_future";
        return result;
    }
    if (input.height.stale) {
        result.height_stale = true;
        result.fault = CargoSafetyFault::CARGO_HEIGHT_INVALID;
        result.reason = "height_stale";
        return result;
    }

    if (!isValidFootprint(input.footprint_base)) {
        result.fault = CargoSafetyFault::INTERNAL_ERROR;
        result.reason = "invalid_input";
        return result;
    }

    result.finite_input_points = perception.finite_input_points;
    if (!perception.valid) {
        result.fault = perception.reason.rfind("invalid_config:", 0U) == 0U ||
                perception.reason == "clustering_failed" ||
                perception.reason == "non_finite_cluster_result" ||
                perception.reason == "invalid_horizontal_query"
            ? CargoSafetyFault::INTERNAL_ERROR
            : CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID;
        result.evidence_state = CargoSafetyEvidenceState::SPARSE_PENDING;
        result.reason = perception.reason;
        return result;
    }
    result.input_valid = true;
    if (perception.clusters.empty()) {
        result.warning_valid = true;
        result.warning_code = kSafeCode;
        result.fault = CargoSafetyFault::NONE;
        result.evidence_state = CargoSafetyEvidenceState::CLEAR;
        result.reason = "clear_no_external_obstacle";
        return result;
    }
    const float conservative_bottom =
        input.height.bottom_z - input.height.bottom_uncertainty_m -
        config_.cargo_bottom_extra_margin_m;
    const HazardEvaluator hazard_evaluator;

    for (const ObstaclePerceptionCluster& perceived : perception.clusters) {
        CargoSafetyClusterEvidence evidence;
        evidence.valid = true;
        evidence.cluster_index = perceived.cluster_index;
        evidence.point_count = perceived.point_count;
        evidence.point_indices = perceived.point_indices;
        evidence.footprint_distance_m = perceived.footprint_distance_m;
        evidence.nearest_point_base = perceived.nearest_point_base;
        evidence.centroid_base = perceived.centroid_base;
        evidence.obstacle_max_z_m = perceived.maximum_z_m;
        evidence.obstacle_min_z_m = perceived.minimum_z_m;
        evidence.obstacle_top_z95_m = perceived.top_z95_m;
        evidence.obstacle_bottom_z05_m = perceived.bottom_z05_m;
        evidence.obstacle_vertical_span_m = perceived.vertical_span_m;
        evidence.vertical_continuity_ratio =
            perceived.vertical_continuity_ratio;
        evidence.obstacle_tail_spread_m = perceived.tail_spread_m;
        evidence.obstacle_uncertainty_m = perceived.obstacle_uncertainty_m;
        evidence.uncertainty_reason =
            "clamp(floor + max(0,z_max-z95), floor, max)";
        HazardEvaluationInput hazard_input;
        hazard_input.source_stamp_sec = input.source_stamp_sec;
        hazard_input.safe_bottom_z_m = conservative_bottom;
        hazard_input.cargo_max_z_m = input.footprint_base.max_z;
        hazard_input.overhead_separation_margin_m =
            config_.overhead_separation_margin_m;
        hazard_input.minimum_vertical_continuity_ratio =
            config_.obstacle_min_vertical_continuity_ratio;
        hazard_input.vertical_geometry_valid = true;
        const HazardEvaluationResult hazard =
            hazard_evaluator.evaluate(hazard_input, perceived);
        evidence.entirely_above_cargo = hazard.entirely_above_cargo;
        evidence.conservative_clearance_m =
            hazard.assessment.conservative_clearance_m;

        if (!hazard.assessment.valid ||
            !isFinite(evidence.footprint_distance_m) ||
            !isFinite(evidence.obstacle_top_z95_m) ||
            !isFinite(evidence.obstacle_uncertainty_m) ||
            !isFinite(evidence.conservative_clearance_m)) {
            result.input_valid = false;
            result.fault = CargoSafetyFault::INTERNAL_ERROR;
            result.reason = "non_finite_cluster_result";
            return result;
        }

        const bool low_clearance = hazard.low_clearance &&
            evidence.conservative_clearance_m <
                config_.minimum_vertical_clearance_m;
        if (low_clearance &&
            evidence.footprint_distance_m <= config_.level1_distance_m) {
            evidence.warning_code = kLevel1Code;
        } else if (low_clearance &&
                   evidence.footprint_distance_m > config_.level1_distance_m &&
                   evidence.footprint_distance_m <= config_.level2_distance_m) {
            evidence.warning_code = kLevel2Code;
        } else {
            evidence.warning_code = kSafeCode;
        }

        ++result.evaluated_cluster_count;
        result.cluster_evidence.push_back(evidence);
        if (!result.has_cluster_evidence ||
            isMoreDangerous(evidence, result.most_dangerous_cluster)) {
            result.most_dangerous_cluster = evidence;
            result.has_cluster_evidence = true;
        }
    }

    if (!result.has_cluster_evidence) {
        result.input_valid = false;
        result.warning_valid = false;
        result.warning_code = 0;
        result.fault = CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID;
        result.evidence_state = CargoSafetyEvidenceState::SPARSE_PENDING;
        result.reason = "obstacle_clusters_insufficient";
        return result;
    }

    result.warning_valid = true;
    result.warning_code = result.most_dangerous_cluster.warning_code;
    result.fault = CargoSafetyFault::NONE;
    if (result.warning_code == kLevel1Code) {
        result.evidence_state = CargoSafetyEvidenceState::HAZARD_CANDIDATE;
        result.reason = "level1_low_clearance";
    } else if (result.warning_code == kLevel2Code) {
        result.evidence_state = CargoSafetyEvidenceState::HAZARD_CANDIDATE;
        result.reason = "level2_low_clearance";
    } else {
        result.evidence_state = CargoSafetyEvidenceState::CLEAR;
        result.reason = "clear";
    }
    return result;
}

ObstaclePerceptionResult CargoSafetyEvaluator::perceive(
    const CargoSafetyInput& input) const {
    ObstaclePerceptionConfig config;
    config.future_stamp_tolerance_sec = config_.future_stamp_tolerance_sec;
    config.maximum_cloud_age_sec = config_.maximum_obstacle_cloud_age_sec;
    config.minimum_roi_finite_points = config_.minimum_roi_finite_points;
    config.minimum_roi_coverage_ratio = config_.minimum_roi_coverage_ratio;
    config.cluster_tolerance_m = config_.obstacle_cluster_tolerance_m;
    config.minimum_cluster_points = config_.obstacle_min_cluster_points;
    config.top_percentile = config_.obstacle_top_percentile;
    config.bottom_percentile = config_.obstacle_bottom_percentile;
    config.vertical_bin_size_m = config_.obstacle_vertical_bin_size_m;
    config.uncertainty_floor_m = config_.obstacle_uncertainty_floor_m;
    config.uncertainty_max_m = config_.obstacle_uncertainty_max_m;
    ObstaclePerceptionInput perception_input;
    perception_input.source_stamp_sec = input.source_stamp_sec;
    perception_input.source_sequence = input.source_sequence;
    perception_input.frame_id = input.frame_id;
    perception_input.query_footprint = input.footprint_base;
    perception_input.cloud_base = input.obstacle_cloud_base;
    perception_input.observation_valid = input.obstacle_observation_valid;
    perception_input.cloud_age_sec = input.obstacle_cloud_age_sec;
    perception_input.roi_finite_points = input.obstacle_roi_finite_points;
    perception_input.roi_coverage_ratio = input.obstacle_roi_coverage_ratio;
    return perceiveObstacles(config, perception_input);
}

}  // namespace ndt_slam
