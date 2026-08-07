#include "ndt_slam/clean_map_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ndt_slam {
namespace {

CleanMapCell cellFor(const Eigen::Vector3f& point, float cell_size) {
    return {
        static_cast<int>(std::floor(point.x() / cell_size)),
        static_cast<int>(std::floor(point.y() / cell_size))};
}

bool pointDeniedByRange(
    const Eigen::Vector3f& point,
    const std::vector<CleanMapDenyRange>& ranges) {
    for (const auto& range : ranges) {
        if (point.z() >= range.z_min && point.z() <= range.z_max) {
            return true;
        }
    }
    return false;
}

float minimumHeightForDistance(float distance_m) {
    if (distance_m < 10.0F) return 0.35F;
    if (distance_m < 20.0F) return 0.25F;
    return 0.15F;
}

int minimumObservationsForDistance(float distance_m) {
    return distance_m < 10.0F ? 2 : 1;
}

float median(std::vector<float> values) {
    const std::size_t middle = values.size() / 2U;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const float upper = values[middle];
    if ((values.size() & 1U) != 0U) return upper;
    std::nth_element(
        values.begin(), values.begin() + middle - 1U, values.end());
    return 0.5F * (values[middle - 1U] + upper);
}

std::vector<std::size_t> rejectIsolatedVerticalOutliers(
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<std::size_t>& indices,
    const CleanMapBuildInput& input,
    int* rejected_points) {
    if (!input.vertical_outlier_filter_enabled ||
        indices.size() < input.vertical_outlier_minimum_points) {
        return indices;
    }
    std::vector<float> z_values;
    z_values.reserve(indices.size());
    for (const std::size_t index : indices) {
        z_values.push_back(points[index].z());
    }
    const float center = median(z_values);
    std::vector<float> deviations;
    deviations.reserve(z_values.size());
    for (const float z : z_values) {
        deviations.push_back(std::abs(z - center));
    }
    const float mad = median(std::move(deviations));
    const double band = std::max(
        static_cast<double>(input.vertical_outlier_minimum_band_m),
        input.vertical_outlier_mad_multiplier * static_cast<double>(mad));
    std::vector<std::size_t> accepted;
    accepted.reserve(indices.size());
    int local_rejected = 0;
    for (const std::size_t index : indices) {
        if (std::abs(static_cast<double>(points[index].z() - center)) <= band) {
            accepted.push_back(index);
        } else {
            ++local_rejected;
        }
    }
    // A robust filter may remove isolated returns, never an entire structure.
    if (accepted.size() < 3U) return indices;
    if (rejected_points) *rejected_points += local_rejected;
    return accepted;
}

}  // namespace

CleanMapBuildAction evaluateCleanMapBuildAction(
    bool build_valid,
    bool newer_request_pending,
    std::uint64_t source_objects_version,
    std::uint64_t current_objects_version) {
    if (!build_valid) return CleanMapBuildAction::DISCARD_INVALID;
    // A newer request must trigger another build, but it must not starve the
    // last completed result. Apply the completed snapshot when its objects
    // generation is still current, then let the pending request converge to
    // the latest deny/protect evidence.
    (void)newer_request_pending;
    if (source_objects_version != current_objects_version) {
        return CleanMapBuildAction::PUBLISH_SNAPSHOT_ONLY;
    }
    return CleanMapBuildAction::APPLY;
}

CleanMapBuildResult buildCleanMapFromSnapshot(
    const CleanMapBuildInput& input) {
    CleanMapBuildResult result;
    if (!std::isfinite(input.cell_size_m) || input.cell_size_m <= 0.0F) {
        result.reason = "invalid_cell_size";
        return result;
    }
    if (input.object_points.empty()) {
        result.valid = true;
        result.reason = "objects_empty";
        return result;
    }
    if (input.object_points.size() > 1000U &&
        input.observation_counts.empty()) {
        result.reason = "observation_history_empty";
        return result;
    }

    float global_min_z = std::numeric_limits<float>::infinity();
    std::map<CleanMapCell, std::vector<std::size_t>> object_indices;
    std::map<CleanMapCell, std::vector<std::size_t>> previous_clean_indices;
    std::map<CleanMapCell, std::vector<std::size_t>> payload_indices;
    std::map<CleanMapCell, double> distance_sum;
    std::map<CleanMapCell, int> finite_count;
    for (std::size_t index = 0U; index < input.object_points.size(); ++index) {
        const Eigen::Vector3f& point = input.object_points[index];
        if (!point.allFinite()) continue;
        global_min_z = std::min(global_min_z, point.z());
        object_indices[cellFor(point, input.cell_size_m)].push_back(index);
    }
    if (!std::isfinite(global_min_z)) {
        result.reason = "objects_nonfinite";
        return result;
    }
    for (const auto& item : object_indices) {
        for (const std::size_t index : item.second) {
            const Eigen::Vector3f& point = input.object_points[index];
            distance_sum[item.first] += std::hypot(point.x(), point.y());
            ++finite_count[item.first];
        }
    }
    for (std::size_t index = 0U;
         index < input.previous_clean_points.size(); ++index) {
        const Eigen::Vector3f& point = input.previous_clean_points[index];
        if (!point.allFinite()) continue;
        previous_clean_indices[cellFor(point, input.cell_size_m)].push_back(
            index);
    }
    for (std::size_t index = 0U;
         index < input.payload_candidate_points.size(); ++index) {
        const Eigen::Vector3f& point = input.payload_candidate_points[index];
        if (!point.allFinite()) continue;
        payload_indices[cellFor(point, input.cell_size_m)].push_back(index);
    }

    for (const auto& item : object_indices) {
        ++result.total_cells;
        const CleanMapCell& cell = item.first;
        const std::vector<std::size_t> filtered_object_indices =
            rejectIsolatedVerticalOutliers(
                input.object_points, item.second, input,
                &result.vertical_outlier_points);
        const auto protect = input.protect_cells.find(cell);
        if (protect != input.protect_cells.end()) {
            for (const std::size_t index : filtered_object_indices) {
                result.clean_points.push_back(input.object_points[index]);
            }
            const auto payload = payload_indices.find(cell);
            if (payload != payload_indices.end()) {
                for (const std::size_t index : payload->second) {
                    result.clean_points.push_back(
                        input.payload_candidate_points[index]);
                }
                result.protected_points +=
                    static_cast<int>(payload->second.size());
            }
            ++result.protected_cells;
            result.protected_points +=
                static_cast<int>(filtered_object_indices.size());
            ++result.passed_cells;
            continue;
        }
        if (input.deny_cells.find(cell) != input.deny_cells.end()) {
            ++result.denied_cells;
            result.denied_points += static_cast<int>(item.second.size());
            continue;
        }
        if (input.use_human_deny &&
            input.human_deny_cells.find(cell) !=
                input.human_deny_cells.end()) {
            ++result.denied_cells;
            ++result.human_denied_cells;
            result.denied_points += static_cast<int>(item.second.size());
            continue;
        }

        const int count = finite_count[cell];
        const float distance = count > 0
            ? static_cast<float>(distance_sum[cell] / count) : 0.0F;
        const auto observation = input.observation_counts.find(cell);
        const int observation_count = observation ==
            input.observation_counts.end() ? 0 : observation->second;
        const auto ranges = input.deny_ranges.find(cell);
        if (observation_count < minimumObservationsForDistance(distance)) {
            const auto previous = previous_clean_indices.find(cell);
            if (previous == previous_clean_indices.end()) continue;

            const std::vector<std::size_t> filtered_previous_indices =
                rejectIsolatedVerticalOutliers(
                    input.previous_clean_points, previous->second, input,
                    &result.vertical_outlier_points);

            const int retained_before = result.retained_points;
            for (const std::size_t index : filtered_previous_indices) {
                const Eigen::Vector3f& point =
                    input.previous_clean_points[index];
                if (input.use_3d_deny &&
                    ranges != input.deny_ranges.end() &&
                    pointDeniedByRange(point, ranges->second)) {
                    ++result.denied_points;
                    continue;
                }
                result.clean_points.push_back(point);
                ++result.retained_points;
            }
            if (result.retained_points > retained_before) {
                ++result.retained_cells;
                ++result.passed_cells;
            }
            continue;
        }
        float robust_maximum_height = 0.0F;
        for (const std::size_t index : filtered_object_indices) {
            robust_maximum_height = std::max(
                robust_maximum_height,
                input.object_points[index].z() - global_min_z);
        }
        if (robust_maximum_height < minimumHeightForDistance(distance) ||
            filtered_object_indices.size() < 3U) {
            continue;
        }

        for (const std::size_t index : filtered_object_indices) {
            const Eigen::Vector3f& point = input.object_points[index];
            if (input.use_3d_deny && ranges != input.deny_ranges.end() &&
                pointDeniedByRange(point, ranges->second)) {
                ++result.denied_points;
                continue;
            }
            result.clean_points.push_back(point);
        }
        ++result.passed_cells;
    }

    result.valid = true;
    result.reason = "complete";
    return result;
}

}  // namespace ndt_slam
