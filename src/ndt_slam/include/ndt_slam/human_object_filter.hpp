#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include <deque>
#include <map>
#include <mutex>
#include <set>

#include "ndt_slam/avoidance_map_mutation.hpp"

namespace ndt_slam {

// 人体候选检测配置
struct HumanObjectFilterConfig {
    bool enabled = true;

    // HAG 高度筛选
    double min_hag = 0.15;   // P1: 降低到 0.15m
    double max_hag = 2.30;

    // cluster 尺寸筛选
    double min_cluster_height = 0.40;  // P1: 降低到 0.40m
    double max_cluster_height = 2.20;
    int min_points = 5;       // P1: 降低到 5，捕获弱小簇
    int max_points = 500;     // P1: 降低到 500
    double min_area_m2 = 0.02;
    double max_area_m2 = 1.20;  // P1: 降低到 1.20
    double max_width_m = 1.20;   // P1: 降低到 1.20
    double max_length_m = 1.20;  // P1: 降低到 1.20

    // P1: 区分 strong/weak 的阈值
    int min_points_strong = 30;  // >= 30 points: strong human/dynamic，直接删除
    int min_points_weak = 5;     // 5-30 points: weak transient，禁止进入地图

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

    // 跟踪统计
    int observed_frames;
    int missed_frames;
    double first_seen_time;
    double last_seen_time;
    double velocity;
    double map_displacement;

};

struct HumanClusterObservation {
    Eigen::Vector3d centroid_base = Eigen::Vector3d::Zero();
    Eigen::Vector3d bbox_min_base = Eigen::Vector3d::Zero();
    Eigen::Vector3d bbox_max_base = Eigen::Vector3d::Zero();
    int point_count = 0;
    bool strong = false;
};

// Immutable, base-frame result produced before registration.  It deliberately
// contains no track state and can therefore be consumed only once by the
// final-pose owner without advancing temporal state during classification.
struct HumanFrameClassification {
    bool valid = false;
    double source_stamp_sec = 0.0;
    std::uint64_t source_cloud_instance_id = 0U;
    SourceFrameIdentity source_frame_identity;
    HumanCurrentFramePointOwnership owned_points;
    std::vector<HumanClusterObservation> clusters;
};

struct HumanMapFilterSnapshot {
    bool valid = false;
    double source_stamp_sec = 0.0;
    std::uint64_t source_cloud_instance_id = 0U;
    SourceFrameIdentity source_frame_identity;
    HumanCurrentFramePointOwnership owned_points;
    StaticLearningBlockCells static_learning_blocks;
    int active_track_count = 0;
    int dynamic_track_count = 0;
};

struct HumanMapAuthorityDiagnostics {
    std::uint64_t classification_count = 0U;
    std::uint64_t map_track_update_count = 0U;
    std::uint64_t duplicate_update_reject_count = 0U;
    std::uint64_t out_of_order_update_reject_count = 0U;
    std::uint64_t registration_owned_point_count = 0U;
    std::uint64_t map_owned_point_count = 0U;
    std::size_t static_learning_block_cell_count = 0U;
    std::size_t static_learning_block_high_water = 0U;
    std::size_t track_high_water = 0U;
};

class HumanObjectDynamicFilter {
public:
    HumanObjectDynamicFilter() = default;

    void initialize(const HumanObjectFilterConfig& filter_config,
                    const HumanTrackingConfig& tracking_config,
                    const HumanEraserConfig& eraser_config);

    void reset();

    // Stateless classification for registration filtering.  This function
    // never updates tracks, deny history or trajectory capsules.
    HumanFrameClassification classifyFrame(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud_base,
        const SourceFrameIdentity& source_frame_identity,
        float ownership_voxel_size_m,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& safe_objects_out,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& human_candidate_out) const;

    // The only product owner of Human temporal/map state.  It is called once
    // after the final FrameAuthorityContext pose is known.
    HumanMapFilterSnapshot updateMapTracks(
        const HumanFrameClassification& classification,
        const Eigen::Matrix4d& T_map_base,
        float static_learning_cell_size_m);

    // 清除过期的跟踪记录
    void cleanupExpiredTracks(double current_time);

    // 获取当前活跃的人体跟踪数
    int getActiveTrackCount() const;

    // 获取确认的动态人体数
    int getDynamicHumanCount() const;

    HumanMapAuthorityDiagnostics diagnostics() const;

private:
    // BEV 聚类
    void clusterBEV(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) const;

    // 判断 cluster 是否符合人体特征
    bool isHumanLikeCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
                            Eigen::Vector3d& centroid,
                            Eigen::Vector3d& bbox_min,
                            Eigen::Vector3d& bbox_max) const;

    // 更新跟踪
    void updateTracks(const std::vector<HumanTrack>& current_detections,
                      double timestamp);

    // 匹配检测到的 cluster 到已有跟踪
    int matchToExistingTrack(const HumanTrack& detection);

    // 判断跟踪是否为动态人体
    bool isDynamicHuman(const HumanTrack& track) const;

    // 计算 BEV 网格键
    std::pair<int, int> bevKey(double x, double y) const;

    std::set<std::pair<int, int>> buildStaticLearningBlockCells(
        const std::vector<HumanTrack>& detections,
        float cell_size_m) const;

    HumanObjectFilterConfig filter_config_;
    HumanTrackingConfig tracking_config_;
    std::map<int, HumanTrack> active_tracks_;
    int next_track_id_ = 0;

    bool has_last_map_update_stamp_ = false;
    double last_map_update_stamp_sec_ = 0.0;
    std::uint64_t last_map_update_cloud_instance_id_ = 0U;
    mutable HumanMapAuthorityDiagnostics diagnostics_;

    mutable std::mutex mutex_;
};

} // namespace ndt_slam
