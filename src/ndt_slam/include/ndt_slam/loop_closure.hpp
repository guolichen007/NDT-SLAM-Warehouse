#pragma once

// NDT-SLAM 闭环检测头文件
// 合并自：LoopClosureNode.hpp + LoopClosureDetector.hpp + ScanContext.hpp + PoseGraphOptimizer.hpp

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <sophus/se3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <Eigen/Core>
#include <vector>
#include <string>
#include <deque>
#include <map>
#include <memory>

// g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel_impl.h>

#include "ndt_slam/keyframe_manager.hpp"

namespace ndt_slam {

// Scan Context 描述子类
class ScanContext {
public:
    ScanContext() : num_rings_(20), num_sectors_(60), max_range_(80.0) {}

    void setNumRings(int num_rings) { num_rings_ = num_rings; }
    void setNumSectors(int num_sectors) { num_sectors_ = num_sectors; }
    void setMaxRange(double max_range) { max_range_ = max_range; }

    int getNumRings() const { return num_rings_; }
    int getNumSectors() const { return num_sectors_; }
    double getMaxRange() const { return max_range_; }

    void configure(int num_rings, int num_sectors, double max_range) {
        num_rings_ = num_rings;
        num_sectors_ = num_sectors;
        max_range_ = max_range;
    }

    Eigen::MatrixXd generate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Vector3d& origin);
    double calculateSimilarity(const Eigen::MatrixXd& sc1, const Eigen::MatrixXd& sc2);
    int findBestMatch(const Eigen::MatrixXd& current_sc, const std::vector<Eigen::MatrixXd>& sc_list);

private:
    int num_rings_;
    int num_sectors_;
    double max_range_;
};

// 闭环候选
struct LoopCandidate {
    int current_keyframe_id;
    int candidate_keyframe_id;
    Sophus::SE3d relative_pose;
    double similarity;
};

// 闭环检测器类
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

// 位姿图优化器类
class PoseGraphOptimizer {
public:
    PoseGraphOptimizer();

    void addKeyFrame(const KeyFrame& keyframe);
    void addOdometryEdge(int from_id, int to_id, const Sophus::SE3d& relative_pose, const Eigen::Matrix<double, 6, 6>& information);
    void addLoopEdge(int from_id, int to_id, const Sophus::SE3d& relative_pose, const Eigen::Matrix<double, 6, 6>& information);
    bool optimize(int iterations = 10);
    Sophus::SE3d getOptimizedPose(int keyframe_id) const;
    void updateKeyFramePoses(std::vector<KeyFrame>& keyframes);

private:
    std::unique_ptr<g2o::SparseOptimizer> optimizer_;
    std::map<int, g2o::VertexSE3*> vertices_;

    Eigen::Matrix<double, 6, 6> odometry_information_;
    Eigen::Matrix<double, 6, 6> loop_information_;
};

// 闭环节点类
class LoopClosureNode {
public:
    LoopClosureNode() = delete;
    explicit LoopClosureNode(const ros::NodeHandle& nh = ros::NodeHandle());
    explicit LoopClosureNode(const std::string& config_file_path, const ros::NodeHandle& nh = ros::NodeHandle());
    ~LoopClosureNode();

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr msg);
    void processLoopClosure();
    void initializeParameters();
    void initializeParameters(const std::string& config_file_path);
    void timerCallback(const ros::TimerEvent&);
    bool relocalizeService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    ros::Timer timer_;

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::ServiceServer relocalize_srv_;
    ros::Publisher relocalization_pub_;

    LoopClosureDetector loop_closure_detector_;
    PoseGraphOptimizer pose_graph_optimizer_;

    std::string odom_topic_ = "/odom";
    std::string pointcloud_topic_ = "/points_raw";
    std::string relocalization_topic_ = "/relocalization";

    Sophus::SE3d last_pose_;
    ros::Time last_stamp_;
    bool initialized_ = false;

    int loop_detection_interval_ = 5;
    int keyframe_count_ = 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_;
    bool tracking_lost_ = false;
    Sophus::SE3d relocalized_pose_;
};

} // namespace ndt_slam
