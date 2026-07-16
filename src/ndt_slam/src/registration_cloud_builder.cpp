#include "ndt_slam/registration_cloud_builder.hpp"

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

namespace ndt_slam {
namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;

bool finitePoint(const pcl::PointXYZ& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z);
}

CloudPtr finiteCopy(const Cloud::ConstPtr& input) {
    CloudPtr output(new Cloud);
    if (!input) return output;
    output->reserve(input->size());
    for (const auto& point : input->points) {
        if (finitePoint(point)) output->push_back(point);
    }
    return output;
}

CloudPtr voxelized(const Cloud::ConstPtr& input, double leaf_size) {
    CloudPtr output(new Cloud);
    if (!input || input->empty()) return output;
    if (!std::isfinite(leaf_size) || leaf_size <= 1.0e-6) {
        *output = *input;
        return output;
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(input);
    const float leaf = static_cast<float>(leaf_size);
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.filter(*output);
    return output;
}

void limitUniform(CloudPtr& cloud, std::size_t maximum) {
    if (!cloud || maximum == 0U) {
        if (cloud) cloud->clear();
        return;
    }
    if (!cloud || cloud->size() <= maximum) return;
    CloudPtr limited(new Cloud);
    limited->reserve(maximum);
    const double stride =
        static_cast<double>(cloud->size()) / static_cast<double>(maximum);
    for (std::size_t index = 0U; index < maximum; ++index) {
        const std::size_t source = std::min(
            cloud->size() - 1U,
            static_cast<std::size_t>(std::floor(index * stride)));
        limited->push_back((*cloud)[source]);
    }
    cloud.swap(limited);
}

std::size_t countXyCells(const Cloud& cloud, double cell_size) {
    const double cell = std::max(0.05, cell_size);
    std::set<std::pair<int, int>> cells;
    for (const auto& point : cloud.points) {
        if (!finitePoint(point)) continue;
        cells.emplace(
            static_cast<int>(std::floor(point.x / cell)),
            static_cast<int>(std::floor(point.y / cell)));
    }
    return cells.size();
}

CloudPtr verticalEdgeRecovery(const Cloud::ConstPtr& input,
                              double xy_cell_size) {
    struct Range {
        bool initialized = false;
        pcl::PointXYZ minimum;
        pcl::PointXYZ maximum;
    };
    std::map<std::pair<int, int>, Range> ranges;
    const double cell = std::max(0.05, xy_cell_size);
    if (input) {
        for (const auto& point : input->points) {
            if (!finitePoint(point)) continue;
            const auto key = std::make_pair(
                static_cast<int>(std::floor(point.x / cell)),
                static_cast<int>(std::floor(point.y / cell)));
            auto& range = ranges[key];
            if (!range.initialized) {
                range.initialized = true;
                range.minimum = point;
                range.maximum = point;
            } else {
                if (point.z < range.minimum.z) range.minimum = point;
                if (point.z > range.maximum.z) range.maximum = point;
            }
        }
    }

    CloudPtr edges(new Cloud);
    for (const auto& entry : ranges) {
        const Range& range = entry.second;
        if (!range.initialized ||
            range.maximum.z - range.minimum.z < 0.40F) {
            continue;
        }
        edges->push_back(range.minimum);
        edges->push_back(range.maximum);
    }
    return edges;
}

CloudPtr gridSampleGround(const Cloud::ConstPtr& input,
                          double cell_size,
                          double edge_height_change) {
    struct Range {
        bool initialized = false;
        pcl::PointXYZ minimum;
        pcl::PointXYZ maximum;
    };
    std::map<std::pair<int, int>, Range> ranges;
    const double cell = std::max(0.05, cell_size);
    if (input) {
        for (const auto& point : input->points) {
            if (!finitePoint(point)) continue;
            const auto key = std::make_pair(
                static_cast<int>(std::floor(point.x / cell)),
                static_cast<int>(std::floor(point.y / cell)));
            auto& range = ranges[key];
            if (!range.initialized) {
                range.initialized = true;
                range.minimum = point;
                range.maximum = point;
            } else {
                if (point.z < range.minimum.z) range.minimum = point;
                if (point.z > range.maximum.z) range.maximum = point;
            }
        }
    }

    CloudPtr sampled(new Cloud);
    sampled->reserve(ranges.size() * 2U);
    for (const auto& entry : ranges) {
        const Range& range = entry.second;
        if (!range.initialized) continue;
        sampled->push_back(range.minimum);
        if (range.maximum.z - range.minimum.z >= edge_height_change) {
            sampled->push_back(range.maximum);
        }
    }
    return sampled;
}

void appendRepeated(const Cloud& source, int repeat, Cloud& destination) {
    for (int copy = 0; copy < std::max(0, repeat); ++copy) {
        destination += source;
    }
}

}  // namespace

RegistrationCloudBuildResult buildStructurePreservingRegistrationCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& human_safe_static_objects,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& uncertain_candidates,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& ground,
    const RegistrationCloudBuildConfig& input_config) {
    RegistrationCloudBuildResult result;
    RegistrationCloudBuildConfig config = input_config;
    config.static_object_voxel_size =
        std::max(0.02, config.static_object_voxel_size);
    config.uncertain_candidate_voxel_size =
        std::max(0.02, config.uncertain_candidate_voxel_size);
    config.ground_grid_cell_m = std::max(0.05, config.ground_grid_cell_m);
    config.ground_max_fraction =
        std::clamp(config.ground_max_fraction, 0.0, 0.49);
    config.static_object_repeat = std::max(1, config.static_object_repeat);
    config.uncertain_candidate_repeat =
        std::max(0, config.uncertain_candidate_repeat);
    config.target_registration_points = std::max(
        config.min_registration_points, config.target_registration_points);
    config.max_ndt_points = std::max(
        config.min_registration_points, config.max_ndt_points);
    const std::size_t target_points = std::min(
        config.target_registration_points, config.max_ndt_points);

    const CloudPtr finite_static = finiteCopy(human_safe_static_objects);
    CloudPtr selected_static = voxelized(
        finite_static, config.static_object_voxel_size);
    const std::size_t normal_static_count = selected_static->size();

    bool used_recovery = false;
    if (selected_static->size() < config.min_static_object_points) {
        CloudPtr recovery_input(new Cloud(*finite_static));
        const CloudPtr vertical_edges = verticalEdgeRecovery(
            finite_static, config.static_object_voxel_size);
        *recovery_input += *vertical_edges;
        selected_static = voxelized(
            recovery_input, config.static_object_voxel_size * 0.5);
        used_recovery = true;
    }

    const std::size_t max_static_unique = std::max<std::size_t>(
        1U, target_points /
            static_cast<std::size_t>(config.static_object_repeat));
    limitUniform(selected_static, max_static_unique);
    result.static_object_points = selected_static->size();
    result.structure_xy_cells = countXyCells(
        *selected_static, config.static_object_voxel_size * 2.0);
    result.structure_quality_valid =
        result.static_object_points >= config.min_static_object_points &&
        result.structure_xy_cells >= config.min_structure_xy_cells;
    *result.structure_cloud = *selected_static;
    *result.static_component = *selected_static;

    if (!result.structure_quality_valid) {
        result.mode = "INSUFFICIENT_STRUCTURE";
        if (result.static_object_points < config.min_static_object_points) {
            result.reason = "static_object_points_below_minimum";
        } else {
            result.reason = "static_structure_xy_coverage_insufficient";
        }
        result.total_points = 0U;
        return result;
    }

    CloudPtr selected_uncertain = voxelized(
        finiteCopy(uncertain_candidates),
        config.uncertain_candidate_voxel_size);

    appendRepeated(*selected_static, config.static_object_repeat,
                   *result.cloud);
    const std::size_t remaining_after_static =
        target_points > result.cloud->size()
            ? target_points - result.cloud->size()
            : 0U;
    const std::size_t uncertain_repeat = static_cast<std::size_t>(
        std::max(1, config.uncertain_candidate_repeat));
    limitUniform(selected_uncertain,
                 remaining_after_static / uncertain_repeat);
    result.uncertain_candidate_points = selected_uncertain->size();
    *result.uncertain_component = *selected_uncertain;
    appendRepeated(*selected_uncertain, config.uncertain_candidate_repeat,
                   *result.cloud);

    const std::size_t non_ground_points = result.cloud->size();
    CloudPtr selected_ground = gridSampleGround(
        finiteCopy(ground), config.ground_grid_cell_m,
        std::max(0.0, config.ground_edge_height_change_m));
    const std::size_t ratio_cap = config.ground_max_fraction > 0.0
        ? static_cast<std::size_t>(std::floor(
            (config.ground_max_fraction * non_ground_points) /
            (1.0 - config.ground_max_fraction)))
        : 0U;
    const std::size_t target_cap = target_points > non_ground_points
        ? target_points - non_ground_points
        : 0U;
    limitUniform(selected_ground, std::min(ratio_cap, target_cap));
    result.ground_points = selected_ground->size();
    *result.ground_component = *selected_ground;
    *result.cloud += *selected_ground;

    result.total_points = result.cloud->size();
    result.ground_fraction = result.total_points > 0U
        ? static_cast<double>(result.ground_points) /
            static_cast<double>(result.total_points)
        : 0.0;
    result.valid = result.structure_quality_valid &&
        result.total_points >= config.min_registration_points &&
        result.total_points <= config.max_ndt_points &&
        result.ground_fraction <= config.ground_max_fraction + 1.0e-9;

    if (!result.valid) {
        result.mode = "INSUFFICIENT_STRUCTURE";
        result.reason = result.total_points < config.min_registration_points
            ? "safe_registration_points_below_minimum"
            : "ground_fraction_or_size_contract_failed";
    } else if (used_recovery &&
               normal_static_count < config.min_static_object_points) {
        result.mode = "STRUCTURE_RECOVERY";
        result.reason = "static_voxel_recovery";
    } else if (result.ground_points > 0U &&
               non_ground_points < config.min_registration_points) {
        result.mode = "GROUND_AUGMENTED";
        result.reason = "bounded_grid_ground_augmentation";
    } else {
        result.mode = "STRUCTURE_RICH";
        result.reason = "safe_static_structure_sufficient";
    }
    return result;
}

RegistrationCloudBuildResult excludeCargoObbFromRegistrationCloud(
    const RegistrationCloudBuildResult& input,
    const CargoObbFootprint& footprint,
    float margin_xy_m,
    float margin_z_m,
    const RegistrationCloudBuildConfig& input_config,
    std::size_t* removed_weighted_points) {
    RegistrationCloudBuildResult result;
    if (removed_weighted_points) *removed_weighted_points = 0U;
    if (!footprint.valid) {
        result.reason = "invalid_cargo_obb";
        return result;
    }
    const auto filter_component = [&](const Cloud::ConstPtr& source,
                                      CloudPtr destination) {
        if (!source) return;
        destination->reserve(source->size());
        for (const auto& point : source->points) {
            if (!finitePoint(point)) continue;
            if (!containsPointInCargoObbBase(
                    Eigen::Vector3f(point.x, point.y, point.z), footprint,
                    margin_xy_m, margin_z_m)) {
                destination->push_back(point);
            }
        }
    };
    filter_component(input.static_component, result.static_component);
    filter_component(input.uncertain_component, result.uncertain_component);
    filter_component(input.ground_component, result.ground_component);

    RegistrationCloudBuildConfig config = input_config;
    config.static_object_repeat = std::max(1, config.static_object_repeat);
    config.uncertain_candidate_repeat =
        std::max(0, config.uncertain_candidate_repeat);
    config.ground_max_fraction =
        std::clamp(config.ground_max_fraction, 0.0, 0.49);
    *result.structure_cloud = *result.static_component;
    appendRepeated(*result.static_component, config.static_object_repeat,
                   *result.cloud);
    appendRepeated(*result.uncertain_component,
                   config.uncertain_candidate_repeat, *result.cloud);
    *result.cloud += *result.ground_component;

    result.static_object_points = result.static_component->size();
    result.uncertain_candidate_points =
        result.uncertain_component->size();
    result.ground_points = result.ground_component->size();
    result.total_points = result.cloud->size();
    result.structure_xy_cells = countXyCells(
        *result.static_component,
        std::max(0.05, config.static_object_voxel_size * 2.0));
    result.ground_fraction = result.total_points > 0U
        ? static_cast<double>(result.ground_points) /
            static_cast<double>(result.total_points)
        : 0.0;
    result.structure_quality_valid =
        result.static_object_points >= config.min_static_object_points &&
        result.structure_xy_cells >= config.min_structure_xy_cells;
    result.valid = result.structure_quality_valid &&
        result.total_points >= config.min_registration_points &&
        result.total_points <= config.max_ndt_points &&
        result.ground_fraction <= config.ground_max_fraction + 1.0e-9;
    result.mode = result.valid ? input.mode : "INSUFFICIENT_STRUCTURE";
    result.reason = result.valid
        ? "authorized_cargo_obb_removed_metrics_recomputed"
        : "authorized_cargo_removal_left_insufficient_structure";
    if (removed_weighted_points) {
        *removed_weighted_points = input.total_points > result.total_points
            ? input.total_points - result.total_points : 0U;
    }
    return result;
}

}  // namespace ndt_slam
