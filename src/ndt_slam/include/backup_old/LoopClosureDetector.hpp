#pragma once

#include "lidar_slam2/KeyFrame.hpp"
#include "lidar_slam2/ScanContext.hpp"
#include <pcl/registration/icp.h>
#include <sophus/se3.hpp>
#include <ros/ros.h>

namespace lidar_slam2 {

struct LoopCandidate {
    int current_keyframe_id;
    int candidate_keyframe_id;
    Sophus::SE3d relative_pose;
    double similarity;
};

class LoopClosureDetector {
public:
    LoopClosureDetector() : scan_context_(), keyframe_manager_() {}

    void configureFromYaml(const std::string& config_file_path);

    void configure(int num_rings, int num_sectors, double max_range,
                   double spatial_search_radius, double similarity_threshold,
                   double translation_threshold, double rotation_threshold);

    void addKeyFrame(const Sophus::SE3d& pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Time& stamp);

    LoopCandidate detectLoop();

    bool checkConsistency(const Sophus::SE3d& loop_pose, const Sophus::SE3d& odometry_pose);

    Sophus::SE3d globalRelocalization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    const std::deque<KeyFrame>& getKeyFrames() const { return keyframe_manager_.getKeyFrames(); }

    KeyFrameManager& getKeyFrameManager() { return keyframe_manager_; }
    const KeyFrameManager& getKeyFrameManager() const { return keyframe_manager_; }

    void updateKeyFramePoses(const std::vector<KeyFrame>& updated_keyframes);

private:
    ScanContext scan_context_;
    KeyFrameManager keyframe_manager_;
    std::vector<Eigen::MatrixXd> scan_context_list_;

    double spatial_search_radius_ = 8.0;
    double similarity_threshold_ = 0.8;

    double translation_threshold_ = 1.0;
    double rotation_threshold_ = 10.0 * M_PI / 180.0;

    Sophus::SE3d refinePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                            const Sophus::SE3d& initial_guess);
};

} // namespace lidar_slam2
