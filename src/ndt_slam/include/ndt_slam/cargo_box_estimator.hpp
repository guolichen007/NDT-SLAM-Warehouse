#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include <map>
#include <deque>

namespace ndt_slam {

// 货物框类型
enum class CargoBoxType {
    CORE,       // 真实货物框，只包裹吊装大件本体
    REMOVE,     // 删除用框，比 core bbox 略大
    FORBIDDEN   // 禁行区/安全区，投影到地面
};

// 货物框输出
struct CargoBox {
    bool valid = false;
    CargoBoxType type = CargoBoxType::CORE;

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
};

// CargoBoxEstimator 配置
struct CargoBoxEstimatorConfig {
    bool enabled = true;

    // 聚类参数
    int min_cluster_points = 40;
    int max_cluster_points = 20000;

    // 悬空主体层检测
    float min_hag_for_candidate = 0.25f;   // 最小 HAG 才认为是吊货候选
    float min_payload_bottom_hag = 0.45f;  // 吊货底部最小 HAG
    float z_hist_bin = 0.10f;              // z histogram bin 大小
    int min_z_band_points = 20;            // 最小悬空主体层点数

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

    // 尺寸增长率限制（防止框体突然变大）
    float max_size_growth_ratio = 1.35f;  // 新框比旧框大 1.35 倍则拒绝

    // 最小悬空高度（不能作为吊货的最低 HAG）
    float min_suspended_hag = 0.35f;

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
    // 输入：cluster 点云（base_link 坐标系）、地面模型
    // 输出：core_box、remove_box、forbidden_box
    bool estimateCargoBox(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
        const SimpleGroundModel& ground_model,
        CargoBox& core_box,
        CargoBox& remove_box,
        CargoBox& forbidden_box);

    // 获取配置
    const CargoBoxEstimatorConfig& getConfig() const { return config_; }

private:
    CargoBoxEstimatorConfig config_;

    // 上一帧的 size（用于尺寸增长率检查）
    Eigen::Vector3f last_core_size_ = Eigen::Vector3f::Zero();
    bool has_last_size_ = false;

    // 计算分位数
    float percentile(std::vector<float>& values, float p);

    // z histogram 悬空主体层检测
    bool findSuspendedDenseBand(
        const std::vector<float>& z_values,
        float ground_z,
        float& band_min,
        float& band_max);

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
};

} // namespace ndt_slam
