#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace lidar_slam2 {

class ScanContext {
public:
    ScanContext() : num_rings_(20), num_sectors_(60), max_range_(80.0) {}

    void setNumRings(int num_rings) { num_rings_ = num_rings; }
    void setNumSectors(int num_sectors) { num_sectors_ = num_sectors; }
    void setMaxRange(double max_range) { max_range_ = max_range; }

    int getNumRings() const { return num_rings_; }
    int getNumSectors() const { return num_sectors_; }
    double getMaxRange() const { return max_range_; }
    
    // 批量设置参数
    void configure(int num_rings, int num_sectors, double max_range) {
        num_rings_ = num_rings;
        num_sectors_ = num_sectors;
        max_range_ = max_range;
    }
    
    // 生成ScanContext描述子
    Eigen::MatrixXd generate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Vector3d& origin);
    
    // 计算两个ScanContext之间的相似度
    double calculateSimilarity(const Eigen::MatrixXd& sc1, const Eigen::MatrixXd& sc2);
    
    // 找到最相似的候选关键帧
    int findBestMatch(const Eigen::MatrixXd& current_sc, const std::vector<Eigen::MatrixXd>& sc_list);
    
private:
    int num_rings_;      // 环的数量
    int num_sectors_;    // 扇区的数量
    double max_range_;   // 最大距离
};

} // namespace lidar_slam2
