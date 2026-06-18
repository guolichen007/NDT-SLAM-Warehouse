#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <string>

#include <ndt_slam/point_cloud_processing.hpp>

namespace ndt_slam {

// 候选门控配置
struct PayloadGateConfig {
    bool enabled = true;

    // 作业通道 ROI（map 坐标系）
    double roi_x_min = -50.0;
    double roi_x_max =  50.0;
    double roi_y_min =  -5.0;
    double roi_y_max =   5.0;
};

// 分流结果
struct GatedObjects {
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_objects;      // ROI 外，直接进正式地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr payload_candidates;  // ROI 内，进跟踪器
};

// 候选分流门控
// 将 objects 点云按是否在作业通道 ROI 内分流
class PayloadCandidateGate {
public:
    PayloadCandidateGate() = default;
    ~PayloadCandidateGate() = default;

    void configure(const PayloadGateConfig& config);
    void configureFromYaml(const std::string& config_file);

    // 主接口：分流 objects 点云
    GatedObjects gate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud);

    // PLC 扩展接口（预留）
    void updateHookState(const PointCloudProcessing::HookState& hook_state);

private:
    PayloadGateConfig config_;
    PointCloudProcessing::HookState hook_state_;
};

} // namespace ndt_slam
