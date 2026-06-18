#include "lidar_slam2/SlamVisualizer.hpp"

#include <yaml-cpp/yaml.h>
#include <pcl_conversions/pcl_conversions.h>

namespace lidar_slam2 {

SlamVisualizer::SlamVisualizer(const ros::NodeHandle& nh)
    : nh_(nh) {

    initializeParameters();

    visualizer_ = std::make_shared<Visualizer>();
    if (!visualizer_->initialize()) {
        ROS_ERROR("Failed to initialize visualizer");
        throw std::runtime_error("Visualizer initialization failed");
    }
    visualizer_->startThread();

    map_sub_ = nh_.subscribe(map_topic_, 10, &SlamVisualizer::mapCallback, this);
    current_cloud_sub_ = nh_.subscribe(current_cloud_topic_, 10, &SlamVisualizer::currentCloudCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &SlamVisualizer::odomCallback, this);

    ROS_INFO("SlamVisualizer initialized");
    ROS_INFO("  Subscribing to map: %s", map_topic_.c_str());
    ROS_INFO("  Subscribing to current cloud: %s", current_cloud_topic_.c_str());
    ROS_INFO("  Subscribing to odom: %s", odom_topic_.c_str());
}

SlamVisualizer::SlamVisualizer(const std::string& config_file_path, const ros::NodeHandle& nh)
    : nh_(nh) {

    initializeParameters(config_file_path);

    visualizer_ = std::make_shared<Visualizer>();
    if (!visualizer_->initialize()) {
        ROS_ERROR("Failed to initialize visualizer");
        throw std::runtime_error("Visualizer initialization failed");
    }
    visualizer_->startThread();

    map_sub_ = nh_.subscribe(map_topic_, 10, &SlamVisualizer::mapCallback, this);
    current_cloud_sub_ = nh_.subscribe(current_cloud_topic_, 10, &SlamVisualizer::currentCloudCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &SlamVisualizer::odomCallback, this);

    ROS_INFO("SlamVisualizer initialized (config: %s)", config_file_path.c_str());
    ROS_INFO("  Subscribing to map: %s", map_topic_.c_str());
    ROS_INFO("  Subscribing to current cloud: %s", current_cloud_topic_.c_str());
    ROS_INFO("  Subscribing to odom: %s", odom_topic_.c_str());
}

SlamVisualizer::~SlamVisualizer() {
    if (visualizer_) {
        visualizer_->stop();
        visualizer_->join();
    }
}

void SlamVisualizer::initializeParameters() {
    std::string config_file_path = "/home/ydkj/lidarslam_ws/src/lidar_slam2/config/slam_params.yaml";
    initializeParameters(config_file_path);
}

void SlamVisualizer::initializeParameters(const std::string& config_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_file_path);

        if (config["map_topic"]) {
            map_topic_ = config["map_topic"].as<std::string>();
        }
        if (config["current_cloud_topic"]) {
            current_cloud_topic_ = config["current_cloud_topic"].as<std::string>();
        }
        if (config["odom_topic"]) {
            odom_topic_ = config["odom_topic"].as<std::string>();
        }

        ROS_INFO("Loaded visualization parameters from %s", config_file_path.c_str());
    } catch (const std::exception& e) {
        ROS_WARN("Failed to load config file: %s, using defaults", e.what());
    }
}

void SlamVisualizer::mapCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    visualizer_->updateMap(cloud);
}

void SlamVisualizer::currentCloudCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    visualizer_->updateCurrentCloud(cloud);
}

void SlamVisualizer::odomCallback(const nav_msgs::Odometry::ConstPtr msg) {
    Eigen::Vector3d position(msg->pose.pose.position.x,
                               msg->pose.pose.position.y,
                               msg->pose.pose.position.z);
    visualizer_->addTrajectoryPoint(position);

    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Sophus::SE3d pose(Sophus::SO3d(q), position);
    visualizer_->updatePose(pose);
}

} // namespace lidar_slam2
