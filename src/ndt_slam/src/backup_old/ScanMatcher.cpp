#include "lidar_slam2/ScanMatcher.hpp"
#include <ros/ros.h>

namespace lidar_slam2 {

ScanMatcher::ScanMatcher(const Config& config) : config_(config) {
    if (config_.type == MatcherType::NDT_OMP) {
        ndt_ = std::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
        ndt_->setResolution(config_.resolution);
        ndt_->setStepSize(config_.step_size);
        ndt_->setTransformationEpsilon(config_.transformation_epsilon);
        ndt_->setMaximumIterations(config_.max_iterations);
        ndt_->setNumThreads(config_.num_threads);
        ROS_INFO("ScanMatcher initialized: NDT_OMP, resolution=%.2f, threads=%d",
                 config_.resolution, config_.num_threads);
    } else {
        gicp_ = std::make_shared<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
        gicp_->setTransformationEpsilon(config_.transformation_epsilon);
        gicp_->setMaximumIterations(config_.max_iterations);
        gicp_->setMaxCorrespondenceDistance(config_.max_correspondence);
        // GICP_OMP doesn't have setNumThreads, it uses OpenMP
        ROS_INFO("ScanMatcher initialized: GICP_OMP");
    }
}

void ScanMatcher::setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) {
    if (config_.type == MatcherType::NDT_OMP) {
        ndt_->setInputTarget(target);
    } else {
        gicp_->setInputTarget(target);
    }
    target_set_ = true;
}

Eigen::Matrix4d ScanMatcher::align(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const Eigen::Matrix4d& initial_guess) {

    if (!target_set_) {
        ROS_ERROR("ScanMatcher: target not set!");
        return initial_guess;
    }

    pcl::PointCloud<pcl::PointXYZ> aligned;

    if (config_.type == MatcherType::NDT_OMP) {
        ndt_->setInputSource(source);
        ndt_->align(aligned, initial_guess.cast<float>());
        final_transformation_ = ndt_->getFinalTransformation().cast<double>();
        has_converged_ = ndt_->hasConverged();
        fitness_score_ = ndt_->getFitnessScore();
    } else {
        gicp_->setInputSource(source);
        gicp_->align(aligned, initial_guess.cast<float>());
        final_transformation_ = gicp_->getFinalTransformation().cast<double>();
        has_converged_ = gicp_->hasConverged();
        fitness_score_ = gicp_->getFitnessScore();
    }

    return final_transformation_;
}

double ScanMatcher::getFitnessScore() const {
    return fitness_score_;
}

bool ScanMatcher::hasConverged() const {
    return has_converged_;
}

Eigen::Matrix4d ScanMatcher::getFinalTransformation() const {
    return final_transformation_;
}

} // namespace lidar_slam2
