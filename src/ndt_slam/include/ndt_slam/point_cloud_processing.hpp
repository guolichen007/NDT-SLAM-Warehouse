#pragma once

// NDT-SLAM 点云处理头文件
// 新增：地面分割、滤波、物体层优化

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>

namespace ndt_slam {

// 点云处理配置
struct PointCloudProcessingConfig {
    // 地面分割参数
    double grid_cell_size = 1.5;
    double height_above_ground = 0.3;

    // 自适应过滤参数
    struct AdaptiveFilter {
        bool enabled = true;
        double near_distance = 10.0;
        double mid_distance = 20.0;

        // SOR 参数
        struct SORParams {
            int mean_k = 10;
            double stddev_mul_thresh = 2.0;
        };
        SORParams sor_near;
        SORParams sor_mid;
        SORParams sor_far;

        // BEV min_height 参数
        double bev_min_height_near = 0.35;
        double bev_min_height_mid = 0.25;
        double bev_min_height_far = 0.15;

        // 时间一致性参数
        int min_observations_near = 2;
        int min_observations_mid = 1;
        int min_observations_far = 1;
    };
    AdaptiveFilter adaptive_filter;

    // 体素滤波参数
    double voxel_size_registration = 0.3;
    double voxel_size_display = 0.1;
    double voxel_size_ground = 0.15;
    double voxel_size_objects = 0.06;

    // 范围过滤
    double max_map_size = 200.0;
};

// 网格单元键
struct CellKey {
    int x, y;
    bool operator<(const CellKey& o) const { return x < o.x || (x == o.x && y < o.y); }
};

// BEV 网格键
struct BevKey {
    int x, y;
    bool operator<(const BevKey& o) const { return x < o.x || (x == o.x && y < o.y); }
};

// 点云处理类
class PointCloudProcessing {
public:
    PointCloudProcessing() = default;
    explicit PointCloudProcessing(const PointCloudProcessingConfig& config);
    ~PointCloudProcessing() = default;

    // 配置
    void configure(const PointCloudProcessingConfig& config);
    void configureFromYaml(const std::string& config_file_path);

    // 地面分割：将点云分为地面和非地面
    void separateGroundByGrid(const pcl::PointCloud<pcl::PointXYZ>& input,
                              pcl::PointCloud<pcl::PointXYZ>& ground_out,
                              pcl::PointCloud<pcl::PointXYZ>& objects_out);

    // 提取地面点
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // 提取非地面点（物体）
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractObjects(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // 低矮点过滤
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterLowPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                         float height_threshold);

    // 噪点过滤（SOR）
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                     int mean_k = 20,
                                                     double stddev_mul_thresh = 1.5);

    // 自适应噪点过滤（根据距离动态调整参数）
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterNoiseAdaptive(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // 边缘保持体素下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr edgePreservingVoxelGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                                  double voxel_size);

    // 普通体素下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                         double voxel_size);

    // 范围过滤
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterByRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                       double max_range);

    // 近场过滤（去除起重机抓臂等固定结构）
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterNearField(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                         double near_field_radius,
                                                         double near_field_z_min,
                                                         pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points = nullptr);

    // 获取局部地面高度
    float getLocalGroundHeight(float x, float y) const;

    // ========== 动态货物过滤 ==========
    // 过滤动态货物（点云自检测模式）
    // 检测并剔除在 base_link 坐标系中相对稳定、但在 map 坐标系中移动的大物体
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterPayloadByTracking(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud,
        const Eigen::Matrix4d& current_pose,
        pcl::PointCloud<pcl::PointXYZ>::Ptr payload_out = nullptr);

    // 获取配置
    const PointCloudProcessingConfig& getConfig() const { return config_; }

private:
    PointCloudProcessingConfig config_;

    // 局部地面模型（网格 -> 地面高度）
    std::map<CellKey, float> cell_ground_z_;

    // 更新局部地面模型
    void updateLocalGroundModel(const pcl::PointCloud<pcl::PointXYZ>& cloud);

    // 计算点到传感器的距离
    float computeDistance(const pcl::PointXYZ& point) const;
};

} // namespace ndt_slam
