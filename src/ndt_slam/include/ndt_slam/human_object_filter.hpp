#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include <deque>
#include <map>
#include <mutex>

namespace ndt_slam {

// 人体候选检测配置
struct HumanObjectFilterConfig {
    bool enabled = true;

    // HAG 高度筛选
    double min_hag = 0.35;
    double max_hag = 2.30;

    // cluster 尺寸筛选
    double min_cluster_height = 0.45;
    double max_cluster_height = 2.40;
    int min_points = 10;
    int max_points = 2500;
    double min_area_m2 = 0.02;
    double max_area_m2 = 1.80;
    double max_width_m = 1.60;
    double max_length_m = 2.00;

    // BEV 聚类参数
    double bev_resolution = 0.15;
    double merge_gap_m = 0.25;
};

// 人体跟踪配置
struct HumanTrackingConfig {
    bool enabled = true;
    double window_sec = 2.0;
    int confirm_frames = 2;
    double map_displacement_thresh_m = 0.20;
    double velocity_thresh_mps = 0.10;
    double max_match_distance_m = 0.80;
    int max_missed_frames = 5;
};

// 人体历史反删配置
struct HumanEraserConfig {
    bool enabled = true;
    double history_sec = 10.0;
    double capsule_radius_m = 0.65;
    bool use_track_height_range = true;
    double z_margin_m = 0.30;
    double hag_margin_m = 0.30;
    double pre_guard_sec = 2.0;
    double post_guard_sec = 3.0;
    bool erase_objects_only = true;
    bool erase_ground = false;
    bool async_update = true;
};

// 人体跟踪状态
enum class HumanTrackState {
    NEW,
    PENDING,
    DYNAMIC_CONFIRMED,
    STATIC_RECOVERED,
    EXPIRED
};

// 人体跟踪记录
struct HumanTrack {
    int id;
    HumanTrackState state;

    // 当前帧信息
    Eigen::Vector3d centroid_base;
    Eigen::Vector3d centroid_map;
    Eigen::Vector3d bbox_min;
    Eigen::Vector3d bbox_max;
    int point_count;
    double height;
    double area;

    // 历史信息
    std::deque<Eigen::Vector3d> centroid_map_history;
    std::deque<double> timestamp_history;
    std::deque<Eigen::Vector3d> bbox_min_history;
    std::deque<Eigen::Vector3d> bbox_max_history;

    // 跟踪统计
    int observed_frames;
    int missed_frames;
    double first_seen_time;
    double last_seen_time;
    double velocity;
    double map_displacement;

    // 高度范围（用于 capsule）
    double track_z_min;
    double track_z_max;
    double track_hag_min;
    double track_hag_max;
};

// 轨迹胶囊（用于历史反删）
struct TrajectoryCapsule {
    int track_id;
    std::deque<Eigen::Vector3d> centerline;
    double radius;
    double z_min;
    double z_max;
    double hag_min;
    double hag_max;
    double pre_guard_sec;
    double post_guard_sec;
    double start_time;
    double end_time;
};

class HumanObjectDynamicFilter {
public:
    HumanObjectDynamicFilter() = default;

    void initialize(const HumanObjectFilterConfig& filter_config,
                    const HumanTrackingConfig& tracking_config,
                    const HumanEraserConfig& eraser_config);

    // 主处理函数：输入 objects_cloud（base_link），输出 safe_objects
    void processFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud_base,
                      const Eigen::Matrix4d& T_map_base,
                      double timestamp,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& safe_objects_out,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& human_candidate_out,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& human_dynamic_out,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& human_pending_out);

    // 历史反删：从地图中删除动态人体的历史残留
    void eraseHumanHistory(pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_map,
                           double current_time);

    // 清除过期的跟踪记录
    void cleanupExpiredTracks(double current_time);

    // 获取当前活跃的人体跟踪数
    int getActiveTrackCount() const;

    // 获取确认的动态人体数
    int getDynamicHumanCount() const;

private:
    // 候选检测
    void detectHumanCandidates(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& candidates_out,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& safe_out);

    // BEV 聚类
    void clusterBEV(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

    // 判断 cluster 是否符合人体特征
    bool isHumanLikeCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
                            Eigen::Vector3d& centroid,
                            Eigen::Vector3d& bbox_min,
                            Eigen::Vector3d& bbox_max);

    // 更新跟踪
    void updateTracks(const std::vector<HumanTrack>& current_detections,
                      double timestamp);

    // 匹配检测到的 cluster 到已有跟踪
    int matchToExistingTrack(const HumanTrack& detection);

    // 判断跟踪是否为动态人体
    bool isDynamicHuman(const HumanTrack& track) const;

    // 生成轨迹胶囊
    void generateTrajectoryCapsule(const HumanTrack& track);

    // 判断点是否在胶囊内
    bool isPointInCapsule(const pcl::PointXYZ& point,
                          const TrajectoryCapsule& capsule,
                          double current_time) const;

    // 判断点是否在多边形柱体内
    bool isPointInPolygonPrism(const pcl::PointXYZ& point,
                               const std::vector<Eigen::Vector2d>& polygon,
                               double z_min, double z_max) const;

    // 计算 BEV 网格键
    std::pair<int, int> bevKey(double x, double y) const;

    HumanObjectFilterConfig filter_config_;
    HumanTrackingConfig tracking_config_;
    HumanEraserConfig eraser_config_;

    std::map<int, HumanTrack> active_tracks_;
    std::vector<TrajectoryCapsule> trajectory_capsules_;
    int next_track_id_ = 0;

    mutable std::mutex mutex_;
};

} // namespace ndt_slam
