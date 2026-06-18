#include "lidar_slam2/ScanContext.hpp"
#include <cmath>

namespace lidar_slam2 {

Eigen::MatrixXd ScanContext::generate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Vector3d& origin) {
    Eigen::MatrixXd sc = Eigen::MatrixXd::Zero(num_rings_, num_sectors_);
    
    if (!cloud || cloud->empty()) {
        return sc;
    }
    
    try {
        for (const auto& point : cloud->points) {
            // 检查点是否有效
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            
            Eigen::Vector3d p(point.x, point.y, point.z);
            Eigen::Vector3d rel_p = p - origin;
            
            double range = rel_p.norm();
            if (range > max_range_) continue;
            
            double angle = std::atan2(rel_p.y(), rel_p.x());
            if (angle < 0) angle += 2 * M_PI;
            
            int ring_idx = static_cast<int>((range / max_range_) * num_rings_);
            int sector_idx = static_cast<int>((angle / (2 * M_PI)) * num_sectors_);
            
            if (ring_idx >= num_rings_) ring_idx = num_rings_ - 1;
            if (sector_idx >= num_sectors_) sector_idx = num_sectors_ - 1;
            
            // 确保索引有效
            if (ring_idx >= 0 && ring_idx < num_rings_ && sector_idx >= 0 && sector_idx < num_sectors_) {
                // 取最大高度
                if (rel_p.z() > sc(ring_idx, sector_idx)) {
                    sc(ring_idx, sector_idx) = rel_p.z();
                }
            }
        }
    } catch (const std::exception& e) {
        // 捕获异常，避免程序崩溃
    }
    
    return sc;
}

double ScanContext::calculateSimilarity(const Eigen::MatrixXd& sc1, const Eigen::MatrixXd& sc2) {
    // 计算两个ScanContext之间的归一化L2距离
    double diff = (sc1 - sc2).norm();
    double max_diff = std::sqrt(sc1.rows() * sc1.cols()) * 5.0; // 假设最大高度差为5米
    double similarity = 1.0 - std::min(diff / max_diff, 1.0);
    
    return similarity;
}

int ScanContext::findBestMatch(const Eigen::MatrixXd& current_sc, const std::vector<Eigen::MatrixXd>& sc_list) {
    if (sc_list.empty()) return -1;
    
    double best_similarity = -1.0;
    int best_idx = -1;
    
    for (int i = 0; i < sc_list.size(); ++i) {
        double similarity = calculateSimilarity(current_sc, sc_list[i]);
        if (similarity > best_similarity) {
            best_similarity = similarity;
            best_idx = i;
        }
    }
    
    return best_idx;
}

} // namespace lidar_slam2
