#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ndt_slam {

struct SensorBodySelfMaskConfig {
    bool enabled = true;
    bool commissioned = false;
    std::string frame_id;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f half_extent = Eigen::Vector3f::Zero();
    float yaw_rad = 0.0F;
    double maximum_removed_ratio = 0.35;
};

struct SensorBodySelfMaskResult {
    pcl::PointCloud<pcl::PointXYZ>::Ptr kept{
        new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed{
        new pcl::PointCloud<pcl::PointXYZ>};
    std::size_t input_points = 0U;
    double removed_ratio = 0.0;
    bool commissioned = false;
    bool frame_matches = false;
    bool ratio_within_limit = false;
    bool mapping_ready = false;
    std::string reason = "not_configured";
};

// The mask is expressed in the rigid sensor/trolley body frame. The caller
// must pass a cloud already expressed in exactly that frame; map/odom are
// deliberately not accepted as implicit mask frames.
class SensorBodySelfMask {
public:
    void configure(const SensorBodySelfMaskConfig& config);
    const SensorBodySelfMaskConfig& config() const;

    SensorBodySelfMaskResult filter(
        const pcl::PointCloud<pcl::PointXYZ>& cloud,
        const std::string& cloud_frame) const;

private:
    SensorBodySelfMaskConfig config_;
};

}  // namespace ndt_slam
