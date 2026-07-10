#include <ndt_slam/cargo_safety_evaluator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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
           isFinite(config.maximum_height_age_sec) && config.maximum_height_age_sec >= 0.0 &&
           isFinite(config.future_stamp_tolerance_sec) &&
           config.future_stamp_tolerance_sec >= 0.0 &&
           isFinite(config.obstacle_top_percentile) &&
           config.obstacle_top_percentile >= 0.0f &&
           config.obstacle_top_percentile <= 1.0f &&
           isFinite(config.obstacle_uncertainty_floor_m) &&
           config.obstacle_uncertainty_floor_m >= 0.0f &&
           isFinite(config.obstacle_uncertainty_max_m) &&
           config.obstacle_uncertainty_max_m >= config.obstacle_uncertainty_floor_m &&
           isFinite(config.obstacle_cluster_tolerance_m) &&
           config.obstacle_cluster_tolerance_m > 0.0f &&
           config.obstacle_min_cluster_points > 0 &&
           config.obstacle_max_cluster_points >= config.obstacle_min_cluster_points &&
           config.obstacle_max_cluster_points <=
               static_cast<std::size_t>(std::numeric_limits<int>::max()) &&
           isFinite(config.self_cargo_margin_x_m) && config.self_cargo_margin_x_m >= 0.0f &&
           isFinite(config.self_cargo_margin_y_m) && config.self_cargo_margin_y_m >= 0.0f &&
           isFinite(config.self_cargo_margin_z_m) && config.self_cargo_margin_z_m >= 0.0f;
}

bool isValidFootprint(const CargoBaseFootprint& footprint) {
    return isFinite(footprint.min_x) && isFinite(footprint.max_x) &&
           isFinite(footprint.min_y) && isFinite(footprint.max_y) &&
           isFinite(footprint.min_z) && isFinite(footprint.max_z) &&
           footprint.min_x <= footprint.max_x &&
           footprint.min_y <= footprint.max_y &&
           footprint.min_z <= footprint.max_z;
}

bool isFinitePoint(const pcl::PointXYZ& point) {
    return isFinite(point.x) && isFinite(point.y) && isFinite(point.z);
}

bool isInsideExpandedCargo(const pcl::PointXYZ& point,
                           const CargoBaseFootprint& footprint,
                           const CargoSafetyConfig& config) {
    return point.x >= footprint.min_x - config.self_cargo_margin_x_m &&
           point.x <= footprint.max_x + config.self_cargo_margin_x_m &&
           point.y >= footprint.min_y - config.self_cargo_margin_y_m &&
           point.y <= footprint.max_y + config.self_cargo_margin_y_m &&
           point.z >= footprint.min_z - config.self_cargo_margin_z_m &&
           point.z <= footprint.max_z + config.self_cargo_margin_z_m;
}

float pointToFootprintDistance(const pcl::PointXYZ& point,
                               const CargoBaseFootprint& footprint) {
    const float dx = std::max(
        std::max(footprint.min_x - point.x, 0.0f), point.x - footprint.max_x);
    const float dy = std::max(
        std::max(footprint.min_y - point.y, 0.0f), point.y - footprint.max_y);
    return std::hypot(dx, dy);
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

int alarmPriority(std::uint16_t raw_code) {
    if (raw_code == CargoSafetyEvaluator::kLevel1Code) {
        return 3;
    }
    if (raw_code == CargoSafetyEvaluator::kLevel2OrFailSafeCode) {
        return 2;
    }
    return 1;
}

bool isMoreDangerous(const CargoSafetyClusterEvidence& candidate,
                     const CargoSafetyClusterEvidence& current) {
    const int candidate_priority = alarmPriority(candidate.raw_code);
    const int current_priority = alarmPriority(current.raw_code);
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

CargoSafetyResult CargoSafetyEvaluator::evaluate(const CargoSafetyInput& input) const {
    CargoSafetyResult result;

    if (!isValidConfig(config_)) {
        result.reason = "invalid_config";
        return result;
    }
    if (!isFinite(input.evaluation_time_sec) || !isValidFootprint(input.footprint_base) ||
        !input.obstacle_cloud_base) {
        result.reason = "invalid_input";
        return result;
    }
    if (!input.height.valid) {
        result.reason = "height_invalid";
        return result;
    }
    if (!isFinite(input.height.bottom_z) ||
        !isFinite(input.height.bottom_uncertainty_m) ||
        input.height.bottom_uncertainty_m < 0.0f ||
        !isFinite(input.height.stamp_sec)) {
        result.reason = "invalid_height_values";
        return result;
    }

    result.height_age_sec = input.evaluation_time_sec - input.height.stamp_sec;
    if (result.height_age_sec < -config_.future_stamp_tolerance_sec) {
        result.reason = "height_timestamp_in_future";
        return result;
    }
    if (input.height.stale || result.height_age_sec > config_.maximum_height_age_sec) {
        result.height_stale = true;
        result.reason = "height_stale";
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
        if (config_.exclude_self_cargo &&
            isInsideExpandedCargo(point, input.footprint_base, config_)) {
            ++result.self_cargo_points_removed;
            continue;
        }
        obstacle_candidates->push_back(point);
    }

    if (obstacle_candidates->size() < config_.obstacle_min_cluster_points) {
        result.raw_code = kSafeCode;
        result.reason = "no_obstacle_clusters";
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
        extraction.setMaxClusterSize(
            static_cast<int>(config_.obstacle_max_cluster_points));
        extraction.setSearchMethod(tree);
        extraction.setInputCloud(obstacle_candidates);
        extraction.extract(cluster_indices);
    } catch (const std::exception&) {
        result.raw_code = kLevel2OrFailSafeCode;
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

        std::vector<float> z_values;
        z_values.reserve(indices.indices.size());
        for (int point_index : indices.indices) {
            const pcl::PointXYZ& point = obstacle_candidates->points[
                static_cast<std::size_t>(point_index)];
            z_values.push_back(point.z);

            const float distance = pointToFootprintDistance(point, input.footprint_base);
            if (distance < evidence.footprint_distance_m) {
                evidence.footprint_distance_m = distance;
                evidence.nearest_point_base = point;
            }
        }

        evidence.obstacle_max_z_m =
            *std::max_element(z_values.begin(), z_values.end());
        evidence.obstacle_top_z95_m =
            nearestRankPercentile(&z_values, config_.obstacle_top_percentile);
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

        const bool low_clearance = evidence.conservative_clearance_m <
                                   config_.minimum_vertical_clearance_m;
        if (low_clearance &&
            evidence.footprint_distance_m <= config_.level1_distance_m) {
            evidence.raw_code = kLevel1Code;
        } else if (low_clearance &&
                   evidence.footprint_distance_m <= config_.level2_distance_m) {
            evidence.raw_code = kLevel2OrFailSafeCode;
        } else {
            evidence.raw_code = kSafeCode;
        }

        ++result.evaluated_cluster_count;
        if (!result.has_cluster_evidence ||
            isMoreDangerous(evidence, result.most_dangerous_cluster)) {
            result.most_dangerous_cluster = evidence;
            result.has_cluster_evidence = true;
        }
    }

    if (!result.has_cluster_evidence) {
        result.raw_code = kSafeCode;
        result.reason = "no_obstacle_clusters";
        return result;
    }

    result.raw_code = result.most_dangerous_cluster.raw_code;
    if (result.raw_code == kLevel1Code) {
        result.reason = "level1_low_clearance";
    } else if (result.raw_code == kLevel2OrFailSafeCode) {
        result.reason = "level2_low_clearance";
    } else {
        result.reason = "clear";
    }
    return result;
}

}  // namespace ndt_slam
