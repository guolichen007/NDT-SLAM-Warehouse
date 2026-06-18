#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <deque>
#include <vector>
#include <map>
#include <string>

#include <ndt_slam/point_cloud_processing.hpp>

namespace ndt_slam {

// 轨迹状态
enum class TrackState {
    NEW,               // 刚出现，状态未知
    PENDING_STATIC,    // 在 ROI 内但暂时没动，可能是堆货
    DYNAMIC_PAYLOAD,   // 已确认是移动吊货，触发过滤
    EXPIRED            // 轨迹消失，等待清理
};

// 聚类信息
struct ClusterInfo {
    Eigen::Vector3f centroid;
    Eigen::Vector3f bbox_min;
    Eigen::Vector3f bbox_max;
    float hag_min;  // height above ground
    float hag_max;
    int point_count;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

// 轨迹点
struct TrackPoint {
    double stamp;
    Eigen::Vector3f position;
};

// 物体轨迹
struct ObjectTrack {
    int track_id;
    TrackState state;

    // 当前帧信息 - map 坐标系
    Eigen::Vector3f centroid_map;
    Eigen::Vector3f bbox_min_map;
    Eigen::Vector3f bbox_max_map;

    // 当前帧信息 - base_link 坐标系
    Eigen::Vector3f centroid_base;
    Eigen::Vector3f bbox_min_base;
    Eigen::Vector3f bbox_max_base;

    float hag_min, hag_max;

    // 轨迹历史 - map 坐标系
    std::deque<TrackPoint> trajectory_map;
    // 轨迹历史 - base_link 坐标系
    std::deque<TrackPoint> trajectory_base;

    // map 坐标系运动统计
    float map_displacement;
    float velocity;
    float direction_consistency;

    // base_link 坐标系稳定性统计
    float base_center_std;    // base_link 下中心位置标准差（越小越稳定）
    float base_lateral_std;   // base_link 下横向稳定性

    // 帧计数
    int observed_frames;
    int missed_frames;
    double first_seen_time;
    double last_seen_time;

    // 关联的点云（用于后续反删）
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_history;
};

// 跟踪配置
struct PayloadTrackerConfig {
    bool enabled = true;

    // BEV 聚类参数
    double bev_resolution = 0.25;
    int min_cluster_points = 30;
    double min_cluster_area = 0.20;
    double cluster_tolerance = 0.5;

    // 跟踪匹配参数
    double max_match_distance = 0.8;
    double max_match_bbox_ratio = 0.5;

    // 运动判断参数（map 坐标系）
    double window_sec = 3.0;
    int motion_confirm_frames = 3;
    double displacement_thresh = 0.25;
    double velocity_thresh = 0.05;
    double direction_consistency_thresh = 0.65;

    // base_link 稳定性参数
    double base_stability_std_thresh = 0.35;  // base_link 下中心标准差阈值（低于此值认为稳定）

    // 轨迹管理
    int max_missed_frames = 5;
};

// 跟踪结果
struct TrackResult {
    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_payload;  // 确认动态，过滤
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_confirmed; // 确认静态，入图
    pcl::PointCloud<pcl::PointXYZ>::Ptr pending;          // 未确定，暂不入图
    int active_tracks;
    int dynamic_tracks;
    int pending_tracks;
};

// 匹配结果
struct MatchResult {
    std::vector<std::pair<int, int>> matches;  // (track_idx, cluster_idx)
    std::vector<int> unmatched_tracks;
    std::vector<int> unmatched_clusters;
};

// 轨迹跟踪管理器
class PayloadTrackManager {
public:
    PayloadTrackManager() = default;
    ~PayloadTrackManager() = default;

    void configure(const PayloadTrackerConfig& config);
    void configureFromYaml(const std::string& config_file);

    // 主接口：更新跟踪（base_link 坐标系输入，map 坐标系跟踪）
    // candidates_base: base_link 下的吊货候选点云
    // T_map_base: 从 base_link 到 map 的变换矩阵
    TrackResult update(const pcl::PointCloud<pcl::PointXYZ>::Ptr& candidates_base,
                       const Eigen::Matrix4d& T_map_base,
                       double stamp,
                       const std::map<CellKey, float>& ground_model);

    // 获取所有活跃轨迹
    const std::vector<ObjectTrack>& getTracks() const { return tracks_; }

    // 获取确认为动态的轨迹
    std::vector<ObjectTrack> getDynamicTracks() const;

    // 获取配置
    const PayloadTrackerConfig& getConfig() const { return config_; }

    // 重置
    void reset();

private:
    PayloadTrackerConfig config_;
    std::vector<ObjectTrack> tracks_;
    int next_track_id_ = 0;

    // 从点云中提取 cluster（在指定坐标系下）
    std::vector<ClusterInfo> extractClusters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::map<CellKey, float>& ground_model);

    // 匹配当前帧 cluster 到已有 track（使用 map 坐标系）
    MatchResult matchClusters(const std::vector<ClusterInfo>& clusters_map);

    // 更新运动统计（双坐标系）
    void updateMotionStats(ObjectTrack& track);

    // 检查状态转换（双坐标系判断）
    void checkStateTransition(ObjectTrack& track);

    // 计算 base_link 下稳定性
    float computeBaseStability(const std::deque<TrackPoint>& trajectory_base);
};

} // namespace ndt_slam
