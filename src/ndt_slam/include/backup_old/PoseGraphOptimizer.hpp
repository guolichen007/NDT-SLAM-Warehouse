#pragma once

#include "lidar_slam2/KeyFrame.hpp"
#include "lidar_slam2/LoopClosureDetector.hpp"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>

namespace lidar_slam2 {

class PoseGraphOptimizer {
public:
    PoseGraphOptimizer();
    
    // 添加关键帧到位姿图
    void addKeyFrame(const KeyFrame& keyframe);
    
    // 添加里程计边
    void addOdometryEdge(int from_id, int to_id, const Sophus::SE3d& relative_pose, const Eigen::Matrix<double, 6, 6>& information);
    
    // 添加回环边
    void addLoopEdge(int from_id, int to_id, const Sophus::SE3d& relative_pose, const Eigen::Matrix<double, 6, 6>& information);
    
    // 执行优化
    bool optimize(int iterations = 10);
    
    // 获取优化后的位姿
    Sophus::SE3d getOptimizedPose(int keyframe_id) const;
    
    // 更新所有关键帧的位姿
    void updateKeyFramePoses(std::vector<KeyFrame>& keyframes);
    
private:
    std::unique_ptr<g2o::SparseOptimizer> optimizer_;
    std::map<int, g2o::VertexSE3*> vertices_;
    
    // 信息矩阵
    Eigen::Matrix<double, 6, 6> odometry_information_;
    Eigen::Matrix<double, 6, 6> loop_information_;
};

} // namespace lidar_slam2
