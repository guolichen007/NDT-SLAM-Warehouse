#pragma once

// BasePayloadChannelFilter: base_link 下中间同线通道吊货候选筛选
//
// 职责：当前帧在 base_link 下发现同线吊货候选
// 不做最终动态判断，只做候选筛选
//
// 输入: objects_cloud_base (base_link 下的非地面点), ground_model
// 输出:
//   safe_objects_cloud: 通道外的安全物体点（可进入 NDT 和正式地图）
//   payload_candidate_cloud: 通道内的吊货候选点（不进 NDT，不进正式地图）

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include <map>
#include <string>

#include <ndt_slam/point_cloud_processing.hpp>

namespace ndt_slam {

// 通道过滤配置
struct BasePayloadChannelConfig {
    bool enabled = true;

    // base_link 坐标系下的通道定义
    // longitudinal_axis = x, lateral_axis = y, vertical_axis = z
    double lateral_center = 0.0;       // 通道横向中心（y 方向）
    double lateral_half_width = 2.0;   // 通道横向半宽（y 方向 ±2m）

    double longitudinal_min = -8.0;    // 通道纵向最小值（x 方向）
    double longitudinal_max = 8.0;     // 通道纵向最大值（x 方向）

    // 高度过滤（相对于地面）
    double min_object_hag = 0.6;       // 低于此高度的点不视为吊货候选

    // BEV 聚类参数（用于从通道内点中提取吊货候选簇）
    double cluster_bev_resolution = 0.25;  // BEV 网格大小
    int min_payload_points = 80;           // 最少点数
    double min_payload_area_m2 = 0.5;      // 最小面积
    double max_payload_area_m2 = 80.0;     // 最大面积

    // 候选膨胀参数（删除时扩大范围，覆盖边界）
    double expand_xy = 0.5;   // XY 方向膨胀（米）
    double expand_z = 0.4;    // Z 方向膨胀（米）
    bool include_weak_points_in_bbox = true;  // bbox 内弱候选也删除
};

// 通道过滤结果
struct ChannelFilterResult {
    pcl::PointCloud<pcl::PointXYZ>::Ptr safe_objects;          // 通道外安全物体
    pcl::PointCloud<pcl::PointXYZ>::Ptr payload_candidates;    // 通道内吊货候选
    pcl::PointCloud<pcl::PointXYZ>::Ptr channel_all_points;    // 通道内所有点（调试用）

    int channel_points = 0;
    int candidate_points = 0;
    int safe_points = 0;
    int candidate_clusters = 0;
};

// BaseLink 下中间通道吊货候选筛选器
class BasePayloadChannelFilter {
public:
    BasePayloadChannelFilter() = default;
    ~BasePayloadChannelFilter() = default;

    void configure(const BasePayloadChannelConfig& config);
    void configureFromYaml(const std::string& config_file);

    // 主接口：在 base_link 下筛选吊货候选
    ChannelFilterResult filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud_base,
        const std::map<CellKey, float>& ground_model);

    // 获取配置
    const BasePayloadChannelConfig& getConfig() const { return config_; }

private:
    BasePayloadChannelConfig config_;

    // 判断点是否在通道内
    bool isInChannel(const pcl::PointXYZ& p, float ground_z) const;

    // 从通道内点中提取吊货候选簇（BEV 聚类）
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractPayloadCandidates(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& channel_points,
        const std::map<CellKey, float>& ground_model,
        int& cluster_count);
};

} // namespace ndt_slam
