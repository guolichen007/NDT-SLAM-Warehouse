#pragma once

// 关键帧管理器头文件
// 从 mapping.hpp 拆分

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <mutex>
#include <deque>
#include <unordered_map>
#include <string>
#include <set>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <yaml-cpp/yaml.h>

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
    std::deque<KeyFrame>& getKeyFramesNonConst() { return keyframes_; }
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

    // ScanContext 数据库
    bool saveScanContextDatabase(const std::string& session_dir, int num_rings, int num_sectors, double max_range) const;
    bool loadScanContextDatabase(const std::string& session_dir, int expected_rings, int expected_sectors);

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

} // namespace ndt_slam
