#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include <map>
#include <deque>
#include <set>

namespace ndt_slam {

// 货物框类型
enum class CargoBoxType {
    CORE,       // 真实货物框，只包裹吊装大件本体
    REMOVE,     // 删除用框，比 core bbox 略大
    FORBIDDEN   // 禁行区/安全区，投影到地面
};

// 拒绝原因
enum class RejectReason {
    NONE = 0,
    LOW_HAG,
    SIZE_JUMP,
    TOO_LARGE,
    LOW_CORE_POINTS,
    INIT_AREA,
    ASPECT_RATIO,
    GROUND_CONTACT,
    NO_DENSE_BAND,
    TOO_FEW_COMPONENTS
};

// 货物框输出
struct CargoBox {
    bool valid = false;
    CargoBoxType type = CargoBoxType::CORE;
    RejectReason reject_reason = RejectReason::NONE;

    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f size = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_min = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max = Eigen::Vector3f::Zero();

    // 悬空主体层信息
    int suspended_points = 0;
    float bottom_hag = 0.0f;  // height above ground
    float z_band_min = 0.0f;
    float z_band_max = 0.0f;
    int removed_low_points = 0;

    // BEV component 信息
    int component_id = -1;
    int component_count = 0;
};

// BEV Cell 数据
struct BevCell {
    int point_count = 0;
    float max_hag = 0.0f;
    float mean_hag = 0.0f;
    float min_z = 0.0f;
    float max_z = 0.0f;
    std::vector<int> point_indices;
};

// BEV Component 数据
struct BevComponent {
    int id = -1;
    std::set<std::pair<int,int>> cells;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    float max_hag = 0.0f;
    int point_count = 0;

    BevComponent() : points(new pcl::PointCloud<pcl::PointXYZ>) {}
};

// CargoBoxEstimator 配置
struct CargoBoxEstimatorConfig {
    bool enabled = true;

    // 聚类参数
    int min_cluster_points = 40;
    int max_cluster_points = 20000;

    // BEV 连通域参数
    float bev_resolution = 0.10f;
    int bev_min_points_per_cell = 2;
    int bev_min_component_cells = 4;
    int bev_connectivity = 8;  // 4 或 8 邻域
    int bev_max_gap_fill_cells = 1;

    // 悬空主体层检测
    float min_hag_for_candidate = 0.25f;   // 最小 HAG 才认为是吊货候选
    float min_payload_bottom_hag = 0.45f;  // 吊货底部最小 HAG
    float z_hist_bin = 0.10f;              // z histogram bin 大小
    int min_z_band_points = 20;            // 最小悬空主体层点数
    float dense_bin_ratio = 0.15f;         // dense bin 阈值比例
    int dense_bin_min_points = 5;          // dense bin 最小点数
    float min_dense_band_height = 0.10f;   // dense band 最小高度
    float max_dense_band_height = 3.50f;   // dense band 最大高度

    // 分位数参数
    float xy_percentile_low = 2.0f;
    float xy_percentile_high = 98.0f;
    float z_percentile_low = 5.0f;
    float z_percentile_high = 98.0f;

    // 轴向约束 OBB
    bool use_crane_axis_obb = true;  // 使用天车轴向约束 OBB

    // 尺寸限制
    float max_width = 6.0f;
    float max_length = 16.0f;
    float max_height = 5.0f;

    // 绝对尺寸约束（初始化检查）
    float max_core_length = 12.0f;
    float max_core_width = 4.0f;
    float max_core_height = 4.0f;
    float min_core_length = 0.30f;
    float min_core_width = 0.30f;
    float min_core_height = 0.10f;

    // 初始化质量约束
    float init_max_area = 30.0f;
    float init_max_aspect_ratio = 8.0f;
    float init_min_bottom_hag = 0.45f;
    int init_min_core_points = 40;

    // 尺寸增长率限制（防止框体突然变大）
    float max_size_growth_ratio = 1.35f;  // 新框比旧框大 1.35 倍则拒绝

    // 最小悬空高度（不能作为吊货的最低 HAG）
    float min_suspended_hag = 0.35f;

    // 最小 core points
    int min_core_points = 30;

    // P0-6: 候选/确认两级阈值
    int min_candidate_core_points = 5;    // 候选阶段最小点数
    int min_confirm_core_points = 25;     // 确认阶段最小点数
    int min_update_core_points = 8;       // 已锁定 track 更新最小点数
    int min_confirm_observed_frames = 3;  // 确认所需观察帧数

    // 显示框扩展
    float core_expand_xy = 0.05f;
    float core_expand_z_down = 0.03f;
    float core_expand_z_up = 0.10f;

    // 删除框扩展
    float remove_expand_xy = 0.25f;
    float remove_expand_z_down = 0.05f;
    float remove_expand_z_up = 0.20f;

    // 禁行区扩展
    float forbidden_expand_xy = 0.50f;
    float forbidden_height = 3.0f;
};

// 地面模型（简化版）
struct SimpleGroundModel {
    float global_z_min = 0.0f;
    float resolution = 1.5f;
    std::map<std::pair<int,int>, float> cell_z;  // (x_idx, y_idx) -> ground_z

    float getGroundZ(float x, float y) const {
        int ix = static_cast<int>(std::floor(x / resolution));
        int iy = static_cast<int>(std::floor(y / resolution));
        auto it = cell_z.find({ix, iy});
        if (it != cell_z.end()) return it->second;
        return global_z_min;
    }
};

class CargoBoxEstimator {
public:
    CargoBoxEstimator() = default;
    ~CargoBoxEstimator() = default;

    void configure(const CargoBoxEstimatorConfig& config);
    void configureFromYaml(const std::string& config_file);

    // 主接口：估计货物框
    // 输入：cluster 点云（base_link 坐标系）、地面模型、上一帧 core_box（可选）
    // 输出：core_box、remove_box、forbidden_box
    bool estimateCargoBox(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
        const SimpleGroundModel& ground_model,
        CargoBox& core_box,
        CargoBox& remove_box,
        CargoBox& forbidden_box,
        const CargoBox* prev_core_box = nullptr);

    // 获取调试点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr getHagFilteredCloud() const { return hag_filtered_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCorePointsCloud() const { return core_points_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getRejectedLowCloud() const { return rejected_low_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getComponentsCloud() const { return components_cloud_; }

    // 获取配置
    const CargoBoxEstimatorConfig& getConfig() const { return config_; }

private:
    CargoBoxEstimatorConfig config_;

    // 调试点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr hag_filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr core_points_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rejected_low_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr components_cloud_;

    // 计算分位数
    float percentile(std::vector<float>& values, float p);

    // BEV 连通域
    std::vector<BevComponent> buildBevComponents(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
        const SimpleGroundModel& ground_model);

    // HAG z-density slicing
    bool findDenseBandByHagHistogram(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
        float ground_z,
        float& band_min_hag,
        float& band_max_hag,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& core_points_out);

    // 计算 core bbox（使用分位数）
    void computeCoreBbox(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
        float ground_z,
        CargoBox& box);

    // 计算轴向约束 OBB
    void computeCraneAxisOBB(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
        Eigen::Vector3f& center,
        Eigen::Vector3f& size);

    // 扩展框
    void expandBox(
        CargoBox& box,
        float xy_expand,
        float z_down_expand,
        float z_up_expand);

    // 检查框是否有效
    RejectReason validateBox(
        const CargoBox& box,
        int core_points_count,
        float bottom_hag);
};

} // namespace ndt_slam
