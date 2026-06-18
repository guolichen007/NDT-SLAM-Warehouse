#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <sophus/se3.hpp>

#include "lidar_slam2/LoopClosureDetector.hpp"
#include "lidar_slam2/PoseGraphOptimizer.hpp"

namespace lidar_slam2 {

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

    ros::Timer timer_;

    bool relocalizeService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

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

} // namespace lidar_slam2
