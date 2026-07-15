#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <string>

namespace ndt_slam {

struct RegistrationCloudBuildConfig {
    double static_object_voxel_size = 0.22;
    double uncertain_candidate_voxel_size = 0.30;
    double ground_grid_cell_m = 0.50;
    double ground_edge_height_change_m = 0.08;

    int static_object_repeat = 2;
    int uncertain_candidate_repeat = 1;

    double ground_max_fraction = 0.35;
    std::size_t min_static_object_points = 600U;
    std::size_t min_structure_xy_cells = 40U;
    std::size_t min_registration_points = 2500U;
    std::size_t target_registration_points = 4000U;
    std::size_t max_ndt_points = 6000U;

    // Accepted only for configuration compatibility. Production code always
    // rejects this unsafe mode.
    bool allow_full_ground_fallback = false;
};

struct RegistrationCloudBuildResult {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{
        new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr structure_cloud{
        new pcl::PointCloud<pcl::PointXYZ>};

    bool valid = false;
    bool structure_quality_valid = false;

    std::size_t static_object_points = 0U;
    std::size_t uncertain_candidate_points = 0U;
    std::size_t ground_points = 0U;
    std::size_t total_points = 0U;
    std::size_t structure_xy_cells = 0U;

    double ground_fraction = 0.0;
    std::string mode = "INSUFFICIENT_STRUCTURE";
    std::string reason = "not_built";
};

RegistrationCloudBuildResult buildStructurePreservingRegistrationCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& human_safe_static_objects,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& uncertain_candidates,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& ground,
    const RegistrationCloudBuildConfig& config);

}  // namespace ndt_slam
