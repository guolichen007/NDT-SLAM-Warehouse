#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "kiss_icp/pipeline/KissICP.hpp"

namespace lidar_slam2 {

class OdometryNode {
public:
    OdometryNode() = delete;
    explicit OdometryNode(const ros::NodeHandle& nh = ros::NodeHandle());
    explicit OdometryNode(const std::string& config_file_path, const ros::NodeHandle& nh = ros::NodeHandle());
    ~OdometryNode();

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    void applyLidar2BaseTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterOutlierPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    void publishOdometry(const ros::Time& stamp, const std::string& cloud_frame_id, const Sophus::SE3d& pose = Sophus::SE3d());

    void publishTF(const ros::Time& stamp);

    bool resetService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    bool setPoseService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    void relocalizationCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void processCloudThread();

    void initializeParameters();
    void initializeParameters(const std::string& config_file_path);

public:
    void updatePose(const Sophus::SE3d& new_pose);
    Sophus::SE3d getPose() const { return current_pose_; }

    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher debug_pub_;
    ros::Publisher mapping_pointcloud_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    ros::ServiceServer reset_srv_;
    ros::ServiceServer set_pose_srv_;
    ros::ServiceClient relocalize_client_;
    ros::Subscriber relocalization_sub_;

    std::string pointcloud_topic_ = "/points_raw";
    std::string mapping_pointcloud_topic_ = "/current_cloud";
    std::string odom_topic_ = "/odom";
    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";
    std::string lidar_odom_frame_ = "odom_lidar";
    std::string relocalization_topic_ = "/relocalization";
    std::string relocalize_service_ = "/loop_closure_node/relocalize";
    bool use_sim_time_ = false;
    bool publish_odom_tf_ = true;
    bool invert_odom_tf_ = true;
    bool publish_debug_clouds_ = false;

    bool use_lidar2base_transform_ = false;
    Eigen::Matrix4d lidar2base_transform_ = Eigen::Matrix4d::Identity();

    double position_covariance_ = 0.1;
    double orientation_covariance_ = 0.1;

    int min_neighbors_ = 3;
    double neighbor_search_radius_ = 0.5;

    double inlier_ratio_threshold_ = 0.5;
    double mean_distance_threshold_ = 0.2;
    double model_deviation_threshold_ = 0.4;

    Sophus::SE3d current_pose_;

    bool initialized_ = false;
    ros::Time last_stamp_;
    std::atomic<bool> tracking_lost_{false};

    std::mutex cloud_mutex_;

    std::queue<sensor_msgs::PointCloud2::ConstPtr> cloud_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::condition_variable tracking_cv_;
    bool shutdown_ = false;

    std::thread process_thread_;

    std::unique_ptr<kiss_icp::pipeline::KissICP> kiss_icp_;
    kiss_icp::pipeline::KISSConfig kiss_icp_config_;
};

} // namespace lidar_slam2
