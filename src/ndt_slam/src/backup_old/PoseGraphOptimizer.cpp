#include "lidar_slam2/PoseGraphOptimizer.hpp"
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace lidar_slam2 {

PoseGraphOptimizer::PoseGraphOptimizer() {
    // 初始化g2o优化器
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>())
    );
    
    optimizer_ = std::make_unique<g2o::SparseOptimizer>();
    optimizer_->setAlgorithm(solver);
    optimizer_->setVerbose(false);
    
    // 初始化信息矩阵
    odometry_information_.setIdentity();
    odometry_information_(0, 0) = 100.0; // x
    odometry_information_(1, 1) = 100.0; // y
    odometry_information_(2, 2) = 100.0; // z
    odometry_information_(3, 3) = 100.0; // roll
    odometry_information_(4, 4) = 100.0; // pitch
    odometry_information_(5, 5) = 100.0; // yaw
    
    loop_information_.setIdentity();
    loop_information_(0, 0) = 100.0; // x
    loop_information_(1, 1) = 100.0; // y
    loop_information_(2, 2) = 100.0; // z
    loop_information_(3, 3) = 100.0; // roll
    loop_information_(4, 4) = 100.0; // pitch
    loop_information_(5, 5) = 100.0; // yaw
}

void PoseGraphOptimizer::addKeyFrame(const KeyFrame& keyframe) {
    // 检查是否已经存在该关键帧
    if (vertices_.find(keyframe.id_) != vertices_.end()) {
        return;
    }
    
    // 创建顶点
    g2o::VertexSE3* vertex = new g2o::VertexSE3();
    vertex->setId(keyframe.id_);
    
    // 设置初始位姿
    Eigen::Isometry3d pose;
    pose = keyframe.pose_.matrix();
    vertex->setEstimate(pose);
    
    // 第一个关键帧固定
    if (keyframe.id_ == 0) {
        vertex->setFixed(true);
    }
    
    // 添加到优化器
    optimizer_->addVertex(vertex);
    vertices_[keyframe.id_] = vertex;
}

void PoseGraphOptimizer::addOdometryEdge(int from_id, int to_id, const Sophus::SE3d& relative_pose, const Eigen::Matrix<double, 6, 6>& information) {
    // 检查顶点是否存在
    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end()) {
        return;
    }
    
    // 创建边
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->setVertex(0, vertices_[from_id]);
    edge->setVertex(1, vertices_[to_id]);
    
    // 设置相对位姿
    Eigen::Isometry3d relative_pose_isometry;
    relative_pose_isometry = relative_pose.matrix();
    edge->setMeasurement(relative_pose_isometry);
    
    // 设置信息矩阵
    edge->setInformation(information);
    
    // 添加到优化器
    optimizer_->addEdge(edge);
}

void PoseGraphOptimizer::addLoopEdge(int from_id, int to_id, const Sophus::SE3d& relative_pose, const Eigen::Matrix<double, 6, 6>& information) {
    // 检查顶点是否存在
    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end()) {
        return;
    }
    
    // 创建边
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->setVertex(0, vertices_[from_id]);
    edge->setVertex(1, vertices_[to_id]);
    
    // 设置相对位姿
    Eigen::Isometry3d relative_pose_isometry;
    relative_pose_isometry = relative_pose.matrix();
    edge->setMeasurement(relative_pose_isometry);
    
    // 设置信息矩阵
    edge->setInformation(information);
    
    // 添加鲁棒核函数
    g2o::RobustKernelHuber* robust_kernel = new g2o::RobustKernelHuber();
    robust_kernel->setDelta(1.0);
    edge->setRobustKernel(robust_kernel);
    
    // 添加到优化器
    optimizer_->addEdge(edge);
}

bool PoseGraphOptimizer::optimize(int iterations) {
    if (optimizer_->vertices().size() < 2) {
        return false;
    }
    
    optimizer_->initializeOptimization();
    int result = optimizer_->optimize(iterations);
    
    return result > 0;
}

Sophus::SE3d PoseGraphOptimizer::getOptimizedPose(int keyframe_id) const {
    auto it = vertices_.find(keyframe_id);
    if (it == vertices_.end()) {
        return Sophus::SE3d();
    }
    
    g2o::VertexSE3* vertex = it->second;
    Eigen::Isometry3d pose = vertex->estimate();
    
    return Sophus::SE3d(pose.matrix());
}

void PoseGraphOptimizer::updateKeyFramePoses(std::vector<KeyFrame>& keyframes) {
    for (auto& keyframe : keyframes) {
        Sophus::SE3d optimized_pose = getOptimizedPose(keyframe.id_);
        if (optimized_pose.so3().matrix().allFinite()) {
            keyframe.pose_ = optimized_pose;
        }
    }
}

} // namespace lidar_slam2
