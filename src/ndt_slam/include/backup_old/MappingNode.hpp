#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include <thread>
#include <queue>
#include <atomic>
#include <condition_variable>
#include "lidar_slam2_msgs/SaveMap.h"
#include "lidar_slam2_msgs/LoadMap.h"

namespace lidar_slam2 {

struct MapTask {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    ros::Time stamp;
};

class MappingNode {
public:
    MappingNode() = delete;
    explicit MappingNode(const ros::NodeHandle& nh = ros::NodeHandle());
    explicit MappingNode(const std::string& config_file_path, const ros::NodeHandle& nh = ros::NodeHandle());
    ~MappingNode();

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void addPointsToMap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                        const Eigen::Vector3d& position,
                        const Eigen::Quaterniond& orientation);

    void publishMap();

    void publishInitialTransform();

    void initializeParameters();
    void initializeParameters(const std::string& config_file_path);

    void processingWorker();

    bool saveMapService(lidar_slam2_msgs::SaveMap::Request& request,
                        lidar_slam2_msgs::SaveMap::Response& response);

    bool loadMapService(lidar_slam2_msgs::LoadMap::Request& request,
                        lidar_slam2_msgs::LoadMap::Response& response);

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber mapping_pointcloud_sub_;
    ros::Publisher map_pub_;
    ros::Publisher current_cloud_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    ros::ServiceServer save_map_srv_;
    ros::ServiceServer load_map_srv_;

    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::Odometry,
        sensor_msgs::PointCloud2> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;

    std::string odom_topic_ = "/odom";
    std::string mapping_pointcloud_topic_ = "/mapping_current_cloud";
    std::string map_topic_ = "/map";
    std::string current_cloud_topic_ = "/mapping_current_cloud";
    std::string map_frame_ = "map";
    std::string odom_frame_ = "odom";

    double voxel_size_ = 0.2;
    double max_map_size_ = 200.0;
    bool use_voxel_filter_ = true;

    std::mutex map_mutex_;
    std::mutex task_queue_mutex_;
    std::condition_variable task_cv_;

    std::queue<MapTask> task_queue_;
    std::vector<std::thread> worker_threads_;
    std::atomic<bool> running_{true};
    int num_worker_threads_ = 0;

    bool has_first_odom_ = false;
    Eigen::Vector3d last_position_;
    Eigen::Quaterniond last_orientation_;

    int frame_count_ = 0;
    int map_update_interval_ = 1;
};

} // namespace lidar_slam2
