#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ndt_slam {

// Keeps channel classification separate from human/dynamic classification.
// The human filter processes the complete object cloud exactly once.  This
// helper then restores the channel partition for NDT weighting without
// resurrecting points that the human filter rejected.
struct RegistrationObjectPartition {
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_objects{
        new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr uncertain_candidates{
        new pcl::PointCloud<pcl::PointXYZ>};
    std::size_t candidate_input_points = 0U;
    std::size_t candidate_survivor_points = 0U;
    std::size_t candidate_human_filtered_points = 0U;
};

using RegistrationPointKey = std::tuple<float, float, float>;

inline RegistrationPointKey makeRegistrationPointKey(
    const pcl::PointXYZ& point) {
    return std::make_tuple(point.x, point.y, point.z);
}

inline RegistrationObjectPartition partitionRegistrationObjects(
    const pcl::PointCloud<pcl::PointXYZ>& human_safe_objects,
    const pcl::PointCloud<pcl::PointXYZ>& channel_candidates) {
    RegistrationObjectPartition result;
    result.candidate_input_points = channel_candidates.size();

    // A multiset count preserves duplicate points and guarantees that a
    // candidate can be classified at most as many times as it appeared in the
    // channel-filter output.
    std::map<RegistrationPointKey, std::size_t> candidate_counts;
    for (const auto& point : channel_candidates.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) continue;
        ++candidate_counts[makeRegistrationPointKey(point)];
    }

    for (const auto& point : human_safe_objects.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) continue;
        auto candidate = candidate_counts.find(makeRegistrationPointKey(point));
        if (candidate != candidate_counts.end() && candidate->second > 0U) {
            result.uncertain_candidates->push_back(point);
            --candidate->second;
        } else {
            result.static_objects->push_back(point);
        }
    }

    result.candidate_survivor_points =
        result.uncertain_candidates->size();
    result.candidate_human_filtered_points =
        result.candidate_input_points - std::min(
            result.candidate_input_points,
            result.candidate_survivor_points);
    return result;
}

}  // namespace ndt_slam
