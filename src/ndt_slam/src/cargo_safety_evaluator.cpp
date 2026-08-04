#include <ndt_slam/cargo_safety_evaluator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <vector>

#include <Eigen/Core>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace ndt_slam {
namespace {

bool isFinite(float value) {
    return std::isfinite(value);
}

bool isFinite(double value) {
    return std::isfinite(value);
}

bool isValidConfig(const CargoSafetyConfig& config) {
    return isFinite(config.level1_distance_m) && config.level1_distance_m >= 0.0f &&
           isFinite(config.level2_distance_m) &&
           config.level2_distance_m >= config.level1_distance_m &&
           isFinite(config.minimum_vertical_clearance_m) &&
           isFinite(config.cargo_bottom_extra_margin_m) &&
           config.cargo_bottom_extra_margin_m >= 0.0f &&
           isFinite(config.future_stamp_tolerance_sec) &&
           config.future_stamp_tolerance_sec >= 0.0 &&
           isFinite(config.maximum_obstacle_cloud_age_sec) &&
           config.maximum_obstacle_cloud_age_sec >= 0.0 &&
           config.minimum_roi_finite_points > 0U &&
           isFinite(config.minimum_roi_coverage_ratio) &&
           config.minimum_roi_coverage_ratio >= 0.0F &&
           config.minimum_roi_coverage_ratio <= 1.0F &&
           isFinite(config.obstacle_top_percentile) &&
           config.obstacle_top_percentile >= 0.0f &&
           config.obstacle_top_percentile <= 1.0f &&
           isFinite(config.obstacle_bottom_percentile) &&
           config.obstacle_bottom_percentile >= 0.0F &&
           config.obstacle_bottom_percentile < config.obstacle_top_percentile &&
           isFinite(config.obstacle_vertical_bin_size_m) &&
           config.obstacle_vertical_bin_size_m > 0.0F &&
           isFinite(config.obstacle_min_vertical_continuity_ratio) &&
           config.obstacle_min_vertical_continuity_ratio >= 0.0F &&
           config.obstacle_min_vertical_continuity_ratio <= 1.0F &&
           isFinite(config.overhead_separation_margin_m) &&
           config.overhead_separation_margin_m >= 0.0F &&
           isFinite(config.obstacle_uncertainty_floor_m) &&
           config.obstacle_uncertainty_floor_m >= 0.0f &&
           isFinite(config.obstacle_uncertainty_max_m) &&
           config.obstacle_uncertainty_max_m >= config.obstacle_uncertainty_floor_m &&
           isFinite(config.obstacle_cluster_tolerance_m) &&
           config.obstacle_cluster_tolerance_m > 0.0f &&
           config.obstacle_min_cluster_points > 0 &&
           config.obstacle_max_cluster_points >= config.obstacle_min_cluster_points &&
           config.obstacle_max_cluster_points <=
               static_cast<std::size_t>(std::numeric_limits<int>::max());
}

bool isValidFootprint(const CargoBaseFootprint& footprint) {
    return footprint.valid && footprint.center_base.allFinite() &&
           isFinite(footprint.length_m) && footprint.length_m > 0.0F &&
           isFinite(footprint.width_m) && footprint.width_m > 0.0F &&
           isFinite(footprint.yaw_base_rad) &&
           isFinite(footprint.min_z) && isFinite(footprint.max_z) &&
           footprint.min_z < footprint.max_z;
}

bool isFinitePoint(const pcl::PointXYZ& point) {
    return isFinite(point.x) && isFinite(point.y) && isFinite(point.z);
}

float pointToFootprintDistance(const pcl::PointXYZ& point,
                               const CargoBaseFootprint& footprint) {
    return pointToCargoObbDistance2D(
        Eigen::Vector2f(point.x, point.y), footprint);
}

float nearestRankPercentile(std::vector<float>* values, float percentile) {
    const std::size_t size = values->size();
    std::size_t rank = static_cast<std::size_t>(
        std::ceil(static_cast<double>(percentile) * static_cast<double>(size)));
    rank = std::max<std::size_t>(1, std::min(rank, size));
    const std::size_t index = rank - 1;
    std::nth_element(values->begin(), values->begin() + index, values->end());
    return (*values)[index];
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

CargoSafetyEvaluator::CargoSafetyEvaluator(const CargoSafetyConfig& config)
    : config_(config) {}

void CargoSafetyEvaluator::setConfig(const CargoSafetyConfig& config) {
    config_ = config;
}

const CargoSafetyConfig& CargoSafetyEvaluator::config() const {
    return config_;
}

CargoSafetyPhaseSelection deriveCargoSafetyDecisionPhase(
    const CargoSafetyDecisionInput& input) {
    CargoSafetyPhaseSelection selection;
    const bool gravity_required_unavailable =
        input.hook_signal_role == HookLoadSignalRole::REQUIRED &&
        (!input.gravity_valid ||
         (!input.gravity_empty && !input.gravity_loaded));
    const bool warning_code_known =
        input.warning_code == 0U ||
        input.warning_code == CargoSafetyProtocol::kClear ||
        input.warning_code == CargoSafetyProtocol::kLevel1Warning ||
        input.warning_code == CargoSafetyProtocol::kLevel2Warning;
    const bool positive_warning = input.warning_valid &&
        (input.warning_code == CargoSafetyProtocol::kLevel1Warning ||
         input.warning_code == CargoSafetyProtocol::kLevel2Warning) &&
        input.obstacle_evidence_ready &&
        (input.pending_positive_warning || input.formal_cargo_valid) &&
        !input.cargo_fault && !input.obstacle_fault;

    if (input.internal_fault) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "explicit_internal_fault";
    } else if (input.warning_valid && !warning_code_known) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "invalid_warning_protocol_code";
    } else if (input.hook_signal_role != HookLoadSignalRole::DISABLED &&
               input.gravity_empty && input.gravity_loaded) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "gravity_state_not_exclusive";
    } else if (!input.system_ready) {
        selection.phase = CargoSafetyDecisionPhase::STARTUP_NOT_READY;
        selection.reason = "system_not_ready";
    } else if (!input.localization_valid) {
        selection.phase = CargoSafetyDecisionPhase::LOCALIZATION_INVALID;
        selection.reason = "localization_unreliable";
    } else if (gravity_required_unavailable) {
        selection.phase =
            CargoSafetyDecisionPhase::GRAVITY_REQUIRED_INVALID;
        selection.reason = "required_gravity_unavailable";
    } else if (positive_warning) {
        selection.phase = CargoSafetyDecisionPhase::VALID_WARNING_OR_CLEAR;
        selection.reason = input.gravity_conflict
            ? "gravity_lidar_conflict_hazard_retained"
            : "positive_hazard";
    } else if (input.gravity_conflict &&
               input.hook_signal_role != HookLoadSignalRole::DISABLED) {
        selection.phase = CargoSafetyDecisionPhase::
            CARGO_EXPECTED_NOT_AUTHORITATIVE;
        selection.reason = "gravity_lidar_conflict";
    } else if (input.safe_empty && input.hook_loaded) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "safe_empty_conflicts_with_hook_loaded";
    } else if (input.safe_empty && input.formal_cargo_valid) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "safe_empty_conflicts_with_formal_cargo";
    } else if (input.safe_empty && input.cargo_fault) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "safe_empty_conflicts_with_cargo_fault";
    } else if (input.safe_empty && input.obstacle_fault) {
        selection.phase = CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR;
        selection.reason = "safe_empty_conflicts_with_obstacle_fault";
    } else if (input.formal_cargo_valid) {
        const bool formal_clear = input.warning_valid &&
            input.warning_code == CargoSafetyProtocol::kClear &&
            input.formal_clear_authorized &&
            input.obstacle_evidence_ready && !input.cargo_fault &&
            !input.obstacle_fault && !input.gravity_conflict;
        if (formal_clear) {
            selection.phase =
                CargoSafetyDecisionPhase::VALID_WARNING_OR_CLEAR;
            selection.reason = "formal_live_static_clear";
        } else if (input.cargo_fault) {
            selection.phase = CargoSafetyDecisionPhase::
                CARGO_EXPECTED_NOT_AUTHORITATIVE;
            selection.reason = "formal_cargo_invalid";
        } else {
            selection.phase = CargoSafetyDecisionPhase::
                CARGO_FORMAL_OBSTACLE_NOT_READY;
            selection.reason = "formal_cargo_clear_authority_incomplete";
        }
    } else if (input.safe_empty) {
        const bool required_empty_valid =
            input.hook_signal_role != HookLoadSignalRole::REQUIRED ||
            (input.gravity_valid && input.gravity_empty &&
             !input.gravity_loaded);
        const bool auxiliary_loaded_conflict =
            input.hook_signal_role == HookLoadSignalRole::AUXILIARY &&
            input.gravity_valid && input.gravity_loaded;
        if (required_empty_valid && !auxiliary_loaded_conflict &&
            !input.cargo_fault && !input.obstacle_fault) {
            selection.phase = CargoSafetyDecisionPhase::SAFE_EMPTY;
            selection.reason = "strict_lidar_empty_confirmed";
        } else {
            selection.phase = CargoSafetyDecisionPhase::
                CARGO_EXPECTED_NOT_AUTHORITATIVE;
            selection.reason = "safe_empty_authority_incomplete";
        }
    } else {
        selection.phase = CargoSafetyDecisionPhase::
            CARGO_EXPECTED_NOT_AUTHORITATIVE;
        selection.reason = input.pending_positive_warning
            ? "pending_hazard_not_authoritative"
            : "cargo_expected_not_authoritative";
    }
    return selection;
}

bool cargoSafetyDecisionSelfConsistent(
    const CargoSafetyDecision& decision) {
    const bool warning =
        decision.requested_code == CargoSafetyProtocol::kClear ||
        decision.requested_code == CargoSafetyProtocol::kLevel1Warning ||
        decision.requested_code == CargoSafetyProtocol::kLevel2Warning;
    if (warning) {
        return decision.valid && decision.warning_valid &&
            decision.warning_code == decision.requested_code &&
            decision.fault_code == 0 && decision.fault_mask == 0U;
    }
    return !decision.valid && !decision.warning_valid &&
        decision.warning_code == 0 &&
        decision.requested_code == decision.fault_code &&
        decision.fault_code >= CargoSafetyProtocol::kSystemNotReady &&
        decision.fault_code <= CargoSafetyProtocol::kInternalError &&
        decision.fault_mask != 0U;
}

CargoSafetyDecision composeCargoSafetyDecision(
    const CargoSafetyDecisionInput& input) {
    CargoSafetyDecision decision;
    decision.fault_mask = 0U;
    const CargoSafetyPhaseSelection selection =
        deriveCargoSafetyDecisionPhase(input);
    const bool use_selection_reason =
        selection.phase == CargoSafetyDecisionPhase::
            INTERNAL_CONTRACT_ERROR || input.gravity_conflict;
    const std::string reason = use_selection_reason ||
        input.evidence_reason.empty()
        ? selection.reason : input.evidence_reason;

    switch (selection.phase) {
        case CargoSafetyDecisionPhase::STARTUP_NOT_READY:
            decision.fault_code = CargoSafetyProtocol::kSystemNotReady;
            decision.fault_mask = CargoSafetyProtocol::kFaultStatusStale;
            break;
        case CargoSafetyDecisionPhase::LOCALIZATION_INVALID:
            decision.fault_code = CargoSafetyProtocol::kLocalizationInvalid;
            decision.fault_mask = CargoSafetyProtocol::kFaultLocalization;
            break;
        case CargoSafetyDecisionPhase::GRAVITY_REQUIRED_INVALID:
            decision.fault_code = CargoSafetyProtocol::kGravityInvalid;
            decision.fault_mask = CargoSafetyProtocol::kFaultGravity;
            break;
        case CargoSafetyDecisionPhase::CARGO_EXPECTED_NOT_AUTHORITATIVE:
            decision.fault_code = CargoSafetyProtocol::kCargoInvalid;
            decision.fault_mask = CargoSafetyProtocol::kFaultCargo;
            break;
        case CargoSafetyDecisionPhase::CARGO_FORMAL_OBSTACLE_NOT_READY:
            decision.fault_code = CargoSafetyProtocol::kObstacleInvalid;
            decision.fault_mask = CargoSafetyProtocol::kFaultObstacle;
            break;
        case CargoSafetyDecisionPhase::SAFE_EMPTY:
            decision.fault_code = 0;
            decision.warning_code = CargoSafetyProtocol::kClear;
            break;
        case CargoSafetyDecisionPhase::VALID_WARNING_OR_CLEAR:
            decision.fault_code = 0;
            decision.warning_code = input.warning_code;
            break;
        case CargoSafetyDecisionPhase::INTERNAL_CONTRACT_ERROR:
        default:
            decision.fault_code = CargoSafetyProtocol::kInternalError;
            decision.fault_mask = CargoSafetyProtocol::kFaultInternal;
            break;
    }

    decision.warning_valid = decision.fault_code == 0;
    decision.valid = decision.warning_valid;
    decision.requested_code = decision.fault_code != 0
        ? decision.fault_code : decision.warning_code;
    decision.reason = reason.empty() ? "safety_decision" : reason;
    if (!cargoSafetyDecisionSelfConsistent(decision)) {
        decision = CargoSafetyDecision{};
        decision.requested_code = CargoSafetyProtocol::kInternalError;
        decision.fault_code = CargoSafetyProtocol::kInternalError;
        decision.fault_mask = CargoSafetyProtocol::kFaultInternal;
        decision.reason = "decision_self_consistency_failure";
    }
    return decision;
}

CargoSafetyResult CargoSafetyEvaluator::evaluate(const CargoSafetyInput& input) const {
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

    if (!input.obstacle_observation_valid ||
        !isFinite(input.obstacle_cloud_age_sec) ||
        input.obstacle_cloud_age_sec < -config_.future_stamp_tolerance_sec ||
        input.obstacle_cloud_age_sec > config_.maximum_obstacle_cloud_age_sec ||
        input.obstacle_roi_finite_points < config_.minimum_roi_finite_points ||
        !isFinite(input.obstacle_roi_coverage_ratio) ||
        input.obstacle_roi_coverage_ratio < config_.minimum_roi_coverage_ratio) {
        result.fault = CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID;
        result.evidence_state = CargoSafetyEvidenceState::SPARSE_PENDING;
        result.reason = "obstacle_observation_insufficient";
        return result;
    }
    if (!isValidFootprint(input.footprint_base)) {
        result.fault = CargoSafetyFault::INTERNAL_ERROR;
        result.reason = "invalid_input";
        return result;
    }

    result.input_valid = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidates(
        new pcl::PointCloud<pcl::PointXYZ>);
    obstacle_candidates->reserve(input.obstacle_cloud_base->size());
    for (const pcl::PointXYZ& point : input.obstacle_cloud_base->points) {
        if (!isFinitePoint(point)) {
            continue;
        }
        ++result.finite_input_points;
        obstacle_candidates->push_back(point);
    }

    if (obstacle_candidates->empty()) {
        result.warning_valid = true;
        result.warning_code = kSafeCode;
        result.fault = CargoSafetyFault::NONE;
        result.evidence_state = CargoSafetyEvidenceState::CLEAR;
        result.reason = "clear_no_external_obstacle";
        return result;
    }
    if (obstacle_candidates->size() < config_.obstacle_min_cluster_points) {
        result.input_valid = false;
        result.fault = CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID;
        result.evidence_state = CargoSafetyEvidenceState::SPARSE_PENDING;
        result.reason = "sparse_obstacle_returns";
        return result;
    }

    std::vector<pcl::PointIndices> cluster_indices;
    try {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(obstacle_candidates);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
        extraction.setClusterTolerance(config_.obstacle_cluster_tolerance_m);
        extraction.setMinClusterSize(
            static_cast<int>(config_.obstacle_min_cluster_points));
        // The ROI is already bounded. Never discard a wall or a large cargo
        // stack merely because it exceeds a tuning-oriented cluster limit.
        extraction.setMaxClusterSize(std::numeric_limits<int>::max());
        extraction.setSearchMethod(tree);
        extraction.setInputCloud(obstacle_candidates);
        extraction.extract(cluster_indices);
    } catch (const std::exception&) {
        result.input_valid = false;
        result.fault = CargoSafetyFault::INTERNAL_ERROR;
        result.reason = "clustering_failed";
        return result;
    }

    const float conservative_bottom =
        input.height.bottom_z - input.height.bottom_uncertainty_m -
        config_.cargo_bottom_extra_margin_m;

    for (std::size_t cluster_index = 0; cluster_index < cluster_indices.size();
         ++cluster_index) {
        const pcl::PointIndices& indices = cluster_indices[cluster_index];
        if (indices.indices.empty()) {
            continue;
        }

        CargoSafetyClusterEvidence evidence;
        evidence.valid = true;
        evidence.cluster_index = cluster_index;
        evidence.point_count = indices.indices.size();
        evidence.point_indices = indices.indices;

        std::vector<float> z_values;
        z_values.reserve(indices.indices.size());
        Eigen::Vector3f centroid_sum = Eigen::Vector3f::Zero();
        for (int point_index : indices.indices) {
            const pcl::PointXYZ& point = obstacle_candidates->points[
                static_cast<std::size_t>(point_index)];
            z_values.push_back(point.z);
            centroid_sum += point.getVector3fMap();

            const float distance = pointToFootprintDistance(point, input.footprint_base);
            if (distance < evidence.footprint_distance_m) {
                evidence.footprint_distance_m = distance;
                evidence.nearest_point_base = point;
            }
        }
        const Eigen::Vector3f centroid = centroid_sum /
            static_cast<float>(indices.indices.size());
        evidence.centroid_base.x = centroid.x();
        evidence.centroid_base.y = centroid.y();
        evidence.centroid_base.z = centroid.z();

        evidence.obstacle_max_z_m =
            *std::max_element(z_values.begin(), z_values.end());
        evidence.obstacle_min_z_m =
            *std::min_element(z_values.begin(), z_values.end());
        evidence.obstacle_top_z95_m =
            nearestRankPercentile(&z_values, config_.obstacle_top_percentile);
        evidence.obstacle_bottom_z05_m =
            nearestRankPercentile(&z_values, config_.obstacle_bottom_percentile);
        evidence.obstacle_vertical_span_m = std::max(
            0.0F, evidence.obstacle_top_z95_m -
                evidence.obstacle_bottom_z05_m);
        std::set<int> occupied_vertical_bins;
        for (const float z : z_values) {
            if (z < evidence.obstacle_bottom_z05_m ||
                z > evidence.obstacle_top_z95_m) {
                continue;
            }
            occupied_vertical_bins.insert(static_cast<int>(std::floor(
                (z - evidence.obstacle_bottom_z05_m) /
                config_.obstacle_vertical_bin_size_m)));
        }
        const int expected_vertical_bins = std::max(
            1, static_cast<int>(std::floor(
                evidence.obstacle_vertical_span_m /
                config_.obstacle_vertical_bin_size_m)) + 1);
        evidence.vertical_continuity_ratio = std::clamp(
            static_cast<float>(occupied_vertical_bins.size()) /
                static_cast<float>(expected_vertical_bins),
            0.0F, 1.0F);
        evidence.entirely_above_cargo =
            evidence.obstacle_bottom_z05_m >
            input.footprint_base.max_z +
                config_.overhead_separation_margin_m;
        evidence.obstacle_tail_spread_m =
            std::max(0.0f, evidence.obstacle_max_z_m - evidence.obstacle_top_z95_m);
        evidence.obstacle_uncertainty_m = std::clamp(
            config_.obstacle_uncertainty_floor_m + evidence.obstacle_tail_spread_m,
            config_.obstacle_uncertainty_floor_m,
            config_.obstacle_uncertainty_max_m);
        evidence.uncertainty_reason =
            "clamp(floor + max(0,z_max-z95), floor, max)";
        evidence.conservative_clearance_m =
            conservative_bottom -
            (evidence.obstacle_top_z95_m + evidence.obstacle_uncertainty_m);

        if (!isFinite(evidence.footprint_distance_m) ||
            !isFinite(evidence.obstacle_top_z95_m) ||
            !isFinite(evidence.obstacle_uncertainty_m) ||
            !isFinite(evidence.conservative_clearance_m)) {
            result.input_valid = false;
            result.fault = CargoSafetyFault::INTERNAL_ERROR;
            result.reason = "non_finite_cluster_result";
            return result;
        }

        const bool vertically_continuous =
            evidence.vertical_continuity_ratio >=
                config_.obstacle_min_vertical_continuity_ratio;
        const bool low_clearance = !evidence.entirely_above_cargo &&
            vertically_continuous &&
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

}  // namespace ndt_slam
