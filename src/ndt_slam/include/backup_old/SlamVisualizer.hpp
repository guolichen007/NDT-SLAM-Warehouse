#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_slam2/Visualizer.hpp"

namespace lidar_slam2 {

class SlamVisualizer {
public:
    SlamVisualizer() = delete;
    explicit SlamVisualizer(const ros::NodeHandle& nh = ros::NodeHandle());
    explicit SlamVisualizer(const std::string& config_file_path, const ros::NodeHandle& nh = ros::NodeHandle());
    ~SlamVisualizer();

private:
    void initializeParameters();
    void initializeParameters(const std::string& config_file_path);

    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr msg);
    void currentCloudCallback(const sensor_msgs::PointCloud2::ConstPtr msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr msg);

    ros::NodeHandle nh_;
    std::shared_ptr<Visualizer> visualizer_;

    ros::Subscriber map_sub_;
    ros::Subscriber current_cloud_sub_;
    ros::Subscriber odom_sub_;

    std::string map_topic_ = "/map";
    std::string current_cloud_topic_ = "/mapping_current_cloud";
    std::string odom_topic_ = "/odom";
};

} // namespace lidar_slam2
