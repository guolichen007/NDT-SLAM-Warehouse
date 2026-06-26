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
#include <ndt_slam/cargo_box_estimator.hpp>

namespace ndt_slam {

// 轨迹状态
enum class TrackState {
    NEW,               // 刚出现，状态未知
    PENDING_STATIC,    // 在 ROI 内但暂时没动，可能是堆货
    DYNAMIC_PAYLOAD,   // 已确认是移动吊货，触发过滤
    SUSPENDED_STATIC,  // 吊着但悬停（HAG 高，velocity 低）
    SUSPENDED_MOVING,  // 吊着并移动（HAG 高，velocity 高）
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

    // ========== P0.5 新增：base_link 坐标系下的 EMA 平滑 ==========
    Eigen::Vector3f centroid_base_ema = Eigen::Vector3f::Zero();
    Eigen::Vector3f size_ema = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_min_base_ema = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max_base_ema = Eigen::Vector3f::Zero();

    // map 坐标系下的显示用 bbox（由 base EMA 转换而来）
    Eigen::Vector3f centroid_map_display = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_min_map_display = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max_map_display = Eigen::Vector3f::Zero();

    // 上一帧的 T_map_base，用于 odom 预测
    Eigen::Matrix4d last_T_map_base = Eigen::Matrix4d::Identity();
    double last_bbox_time = 0.0;

    // track 锁定相关
    int lock_confirm_count = 0;
    bool locked = false;

    // P0-2: per-track size jump 检查（替代 CargoBoxEstimator 的全局状态）
    Eigen::Vector3f last_core_size = Eigen::Vector3f::Zero();
    bool has_last_size = false;

    // P0-7: 上一帧的 core_box 信息（用于 track 一致性评分）
    CargoBox last_core_box;
    bool has_last_core_box = false;
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

    // P0.5 新增：base_link 优先匹配参数
    double max_match_distance_base = 0.80;   // base_link 下最大匹配距离
    double max_match_residual_map = 1.20;    // map 下 odom 预测残差阈值
    double max_match_bbox_ratio_base = 0.70; // base_link 下 bbox 尺寸差异比例

    // 运动判断参数（map 坐标系）
    double window_sec = 3.0;
    int motion_confirm_frames = 3;
    double displacement_thresh = 0.25;
    double velocity_thresh = 0.05;
    double direction_consistency_thresh = 0.65;

    // base_link 稳定性参数
    double base_stability_std_thresh = 0.35;  // base_link 下中心标准差阈值（低于此值认为稳定）

    // P0.5 新增：EMA 平滑参数
    double centroid_base_alpha = 0.60;   // base_link 下中心点 EMA 系数
    double size_base_alpha = 0.25;       // base_link 下尺寸 EMA 系数
    int lock_switch_confirm_frames = 3;  // 切换 track 前需要确认的帧数

    // 轨迹管理
    int max_missed_frames = 5;

    // 悬浮识别参数
    bool suspended_detection_enabled = true;
    double min_floating_gap = 0.30;         // 最小悬浮高度（米）
    double strong_floating_gap = 0.60;      // 强悬浮高度（米）
    double max_support_ratio = 0.20;        // 最大支撑比例
    int min_suspended_observed_frames = 3;  // 最小观察帧数
    bool keep_suspended_when_stopped = true; // 悬停时保持 SUSPENDED_STATIC
    double suspended_timeout_sec = 5.0;     // 悬浮超时（秒）
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

// 轻量只读结构，用于 /payload_track_info 发布
struct PayloadTrackInfo {
    int track_id = -1;
    int state = 0;  // 0=NONE, 1=PENDING, 2=DYNAMIC

    Eigen::Vector3f centroid_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity_map = Eigen::Vector3f::Zero();

    Eigen::Vector3f bbox_min_map = Eigen::Vector3f::Zero();
    Eigen::Vector3f bbox_max_map = Eigen::Vector3f::Zero();

    int point_count = 0;
    float track_duration = 0.0f;
    float direction_consistency = 0.0f;
    float map_displacement = 0.0f;
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

    // P0-2: 获取可修改的轨迹列表（用于 per-track size jump 检查）
    std::vector<ObjectTrack>& getMutableTracks() { return tracks_; }

    // 获取确认为动态的轨迹
    std::vector<ObjectTrack> getDynamicTracks() const;

    // 获取当前最可信的动态吊货 track
    bool getBestDynamicPayloadTrack(PayloadTrackInfo& out) const;

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

    // 匹配当前帧 cluster 到已有 track（P0.5: base_link 优先匹配）
    MatchResult matchClusters(const std::vector<ClusterInfo>& clusters_map,
                              const std::vector<ClusterInfo>& clusters_base,
                              const Eigen::Matrix4d& T_map_base);

    // 更新运动统计（双坐标系）
    void updateMotionStats(ObjectTrack& track);

    // 检查状态转换（双坐标系判断）
    void checkStateTransition(ObjectTrack& track);

    // 检查货物尺寸是否有效
    bool isCargoSizeValid(const ObjectTrack& track) const;

    // 计算 base_link 下稳定性
    float computeBaseStability(const std::deque<TrackPoint>& trajectory_base);

    // P0.5 新增：状态判断辅助函数
    bool isDynamicLike(TrackState s) const {
        return s == TrackState::DYNAMIC_PAYLOAD ||
               s == TrackState::SUSPENDED_MOVING ||
               s == TrackState::SUSPENDED_STATIC;
    }

    bool isHardRemove(TrackState s) const {
        return s == TrackState::DYNAMIC_PAYLOAD ||
               s == TrackState::SUSPENDED_MOVING ||
               s == TrackState::SUSPENDED_STATIC ||
               s == TrackState::PENDING_STATIC;
    }

    // P0.5 新增：更新 base_link 下的 EMA 平滑
    void updateBaseEma(ObjectTrack& track, const ClusterInfo& cluster_base);

    // P0.5 新增：用 T_map_base 把 base bbox 转成 map bbox
    void transformBaseBboxToMap(ObjectTrack& track, const Eigen::Matrix4d& T_map_base);
};

} // namespace ndt_slam
