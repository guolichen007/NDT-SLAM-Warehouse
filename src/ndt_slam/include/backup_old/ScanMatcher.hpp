#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <memory>

// NDT_OMP
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>

namespace lidar_slam2 {

enum class MatcherType {
    NDT_OMP,
    GICP_OMP
};

class ScanMatcher {
public:
    struct Config {
        MatcherType type = MatcherType::NDT_OMP;
        double resolution = 1.0;          // NDT 体素分辨率 (m)
        double step_size = 0.1;           // NDT 步长
        double max_iterations = 100;      // 最大迭代次数
        double transformation_epsilon = 0.01;
        int num_threads = 4;              // 并行线程数
        double max_correspondence = 3.0;  // GICP 最大对应距离
    };

    explicit ScanMatcher(const Config& config);
    ~ScanMatcher() = default;

    // 设置目标点云（局部地图）
    void setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target);

    // scan-to-map 配准
    Eigen::Matrix4d align(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
        const Eigen::Matrix4d& initial_guess);

    // 获取配准分数（越低越好）
    double getFitnessScore() const;

    // 获取是否收敛
    bool hasConverged() const;

    // 获取最终变换矩阵
    Eigen::Matrix4d getFinalTransformation() const;

private:
    Config config_;

    // NDT_OMP
    std::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ndt_;

    // GICP_OMP (备选)
    std::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp_;

    bool target_set_ = false;
    Eigen::Matrix4d final_transformation_ = Eigen::Matrix4d::Identity();
    double fitness_score_ = 0.0;
    bool has_converged_ = false;
};

} // namespace lidar_slam2
