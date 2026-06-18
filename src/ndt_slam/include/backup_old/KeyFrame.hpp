#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <string>
#include <chrono>
#include <deque>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

namespace lidar_slam2 {

// 关键帧质量指标
struct KeyFrameMetrics {
    double fitness_score = 0.0;           // NDT 配准适应度
    double transformation_probability = 0.0;  // NDT 变换概率
    double inlier_ratio = 0.0;            // 内点比率
    double ground_thickness = 0.0;        // 地面厚度
    double obj_ratio = 0.0;               // 非地面点比例
    int ground_points = 0;                // 地面点数
    int object_points = 0;                // 非地面点数
    double registration_time_ms = 0.0;    // 配准耗时
    bool accepted_for_localization = false;  // 是否可用于定位
    bool accepted_for_detail_map = false;    // 是否可用于细节地图
    bool accepted_for_clean_map = false;     // 是否可用于干净地图
};

class KeyFrame {
public:
    KeyFrame() = default;
    KeyFrame(uint64_t id, const ros::Time& stamp, const Sophus::SE3d& pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
        : id_(id), stamp_(stamp), pose_(pose), cloud_(cloud) {}

    uint64_t id_ = 0;
    ros::Time stamp_;
    Sophus::SE3d pose_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    Eigen::MatrixXd scan_context_;
    KeyFrameMetrics metrics_;  // 质量指标
    Sophus::SE3d pose_refined_;  // 精配准后的位姿
    bool has_refined_pose_ = false;  // 是否有精配准位姿
};

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

    // 保存关键帧数据库到目录
    bool saveKeyFrameDatabase(const std::string& session_dir) const;

    // 从目录加载关键帧数据库
    bool loadKeyFrameDatabase(const std::string& session_dir);

    // 保存优化位姿到文件
    bool saveOptimizedPoses(const std::string& filepath) const;

    // 从文件加载优化位姿
    bool loadOptimizedPoses(const std::string& filepath);

    // 更新关键帧位姿（用于精配准）
    void updateKeyFramePose(uint64_t id, const Sophus::SE3d& new_pose);

    // 获取关键帧数量
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

} // namespace lidar_slam2
