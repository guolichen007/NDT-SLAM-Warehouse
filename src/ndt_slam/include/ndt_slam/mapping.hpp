#pragma once

// NDT-SLAM 建图头文件
// 合并自：MappingNode.hpp + KeyFrame.hpp

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
#include <deque>
#include <unordered_map>
#include <string>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "lidar_slam2_msgs/SaveMap.h"
#include "lidar_slam2_msgs/LoadMap.h"

namespace ndt_slam {

// 关键帧质量指标
struct KeyFrameMetrics {
    double fitness_score = 0.0;
    double transformation_probability = 0.0;
    double inlier_ratio = 0.0;
    double ground_thickness = 0.0;
    double obj_ratio = 0.0;
    int ground_points = 0;
    int object_points = 0;
    double registration_time_ms = 0.0;
    bool accepted_for_localization = false;
    bool accepted_for_detail_map = false;
    bool accepted_for_clean_map = false;
};

// 动态 mask（标记哪些区域被动态物体覆盖）
struct DynamicMask {
    // 吊货 swept volume 的 BEV cell 集合
    std::set<std::pair<int,int>> payload_deny_cells;
    // 人体 capsule 的 BEV cell 集合
    std::set<std::pair<int,int>> human_deny_cells;
    // BEV 分辨率
    double bev_resolution = 0.15;
    // 是否已应用
    bool applied = false;
};

// 关键帧类
class KeyFrame {
public:
    KeyFrame() = default;
    KeyFrame(uint64_t id, const ros::Time& stamp, const Sophus::SE3d& pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
        : id_(id), stamp_(stamp), pose_(pose), cloud_(cloud) {}

    uint64_t id_ = 0;
    ros::Time stamp_;
    Sophus::SE3d pose_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;           // 原始完整点云（用于闭环检测）
    Eigen::MatrixXd scan_context_;
    KeyFrameMetrics metrics_;
    Sophus::SE3d pose_refined_;
    bool has_refined_pose_ = false;

    // ========== 动态过滤相关 ==========
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_raw;       // 原始非地面点（调试/重新过滤用）
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_filtered;  // 过滤后的非地面点（正式建图用）
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points;     // 地面点
    DynamicMask dynamic_mask;                              // 动态 mask
    bool dirty_dynamic = true;                             // 是否需要重新过滤（默认需要）
    bool dirty_pose = false;                               // 位姿是否已更新
};

// 关键帧管理器类
class KeyFrameManager {
public:
    KeyFrameManager() : last_keyframe_id_(0), last_keyframe_time_(0) {}

    void configureFromYaml(const YAML::Node& config);
    void configure(double translation_threshold, double rotation_threshold,
                   double time_threshold, int max_keyframes);

    bool isKeyFrame(const Sophus::SE3d& current_pose, const ros::Time& current_time);
    void addKeyFrame(const Sophus::SE3d& pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Time& stamp);

    const std::deque<KeyFrame>& getKeyFrames() const { return keyframes_; }
    const KeyFrame* getLastKeyFrame() const {
        return keyframes_.empty() ? nullptr : &keyframes_.back();
    }

    std::vector<int> getNearbyKeyFrames(const Eigen::Vector3d& position, double radius) const;
    const KeyFrame* getKeyFrameById(uint64_t id) const;

    bool saveKeyFrameDatabase(const std::string& session_dir) const;
    bool loadKeyFrameDatabase(const std::string& session_dir);
    bool saveOptimizedPoses(const std::string& filepath) const;
    bool loadOptimizedPoses(const std::string& filepath);
    void updateKeyFramePose(uint64_t id, const Sophus::SE3d& new_pose);
    size_t getKeyFrameCount() const { return keyframes_.size(); }

private:
    std::deque<KeyFrame> keyframes_;
    uint64_t last_keyframe_id_;
    ros::Time last_keyframe_time_;
    Sophus::SE3d last_keyframe_pose_;

    std::unordered_map<uint64_t, std::vector<uint64_t>> spatial_index_;

    double translation_threshold_ = 0.8;
    double rotation_threshold_ = 8.0 * M_PI / 180.0;
    double time_threshold_ = 1.5;
    int max_keyframes_ = 0;

    uint64_t computeSpatialHash(const Eigen::Vector3d& position, double cell_size) const;
};

// 建图任务
struct MapTask {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    ros::Time stamp;
};

// 建图节点类
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

} // namespace ndt_slam
