#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>

// P0.5: 货物框估计器
#include <ndt_slam/cargo_box_estimator.hpp>
#include <set>

// NDT_OMP
#include <pclomp/ndt_omp.h>

#include "ndt_slam/loop_closure.hpp"
#include "ndt_slam/base_payload_channel_filter.hpp"
#include "ndt_slam/payload_tracker.hpp"
#include "ndt_slam/human_object_filter.hpp"
#include "ndt_slam/dynamic_event_manager.hpp"
#include "lidar_slam2_msgs/SaveMap.h"
#include "lidar_slam2_msgs/LoadMap.h"

// KISS-ICP config struct (保留用于兼容)
namespace kiss_icp { namespace pipeline {
struct KISSConfig {
    double max_range = 100.0;
    double min_range = 0.0;
    double voxel_size = 1.0;
    int max_points_per_voxel = 20;
    bool deskew = false;
    int max_num_iterations = 500;
    double convergence_criterion = 0.0001;
    double initial_threshold = 2.0;
    double min_motion_th = 0.1;
};
}}

namespace ndt_slam {

struct MappingTask {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    ros::Time stamp;
};

class NdtSlamNode {
public:
    NdtSlamNode() = delete;
    explicit NdtSlamNode(const ros::NodeHandle& nh = ros::NodeHandle());
    explicit NdtSlamNode(const std::string& config_file_path, const ros::NodeHandle& nh = ros::NodeHandle());
    ~NdtSlamNode();

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void processCloudThread();

    void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void applyLidar2BaseTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterOutlierPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // 特征提取
    void extractFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& registration_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& mapping_cloud);

    bool use_feature_extraction_ = true;
    double feature_voxel_size_ = 0.2;
    double height_diff_threshold_ = 0.10;
    int feature_weight_ = 8;

    void publishOdometry(const ros::Time& stamp, const std::string& cloud_frame_id, const Sophus::SE3d& pose = Sophus::SE3d());

    void publishInitialTransform();
    void publishTF(const ros::Time& stamp);

    void addFrameToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       const Sophus::SE3d& pose,
                       const ros::Time& stamp);

    void rebuildGlobalMap();
    void rebuildGlobalMapFiltered();  // 使用 filtered keyframes + dynamic mask 重建地图
    void rebuildDisplayMap();     // 重建细体素显示地图
    void publishDisplayMap();     // 发布显示地图
    void rebuildGroundAndObjectsMap();  // 重建地面/非地面分层地图
    void rebuildCleanMap();             // 异步重建 clean map（带时间一致性）
    void publishGroundMap();
    void publishObjectsMap();
    void publishObjectsCleanMap();

    // 网格局部地面分割：将点云分为 ground 和 objects
    // 使用 XY 网格，每个格子独立计算局部地面高度
    void separateGroundByGrid(const pcl::PointCloud<pcl::PointXYZ>& input,
                              pcl::PointCloud<pcl::PointXYZ>& ground_out,
                              pcl::PointCloud<pcl::PointXYZ>& objects_out);

    void addKeyFrameToLoopClosure(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                  const Sophus::SE3d& pose,
                                  const ros::Time& stamp);

    void publishMap();
    void publishCurrentCloud();

    void processingWorker();
    void processLoopClosure();

    bool resetService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool setPoseService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool relocalizeService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool saveMapService(lidar_slam2_msgs::SaveMap::Request& request,
                        lidar_slam2_msgs::SaveMap::Response& response);
    bool loadMapService(lidar_slam2_msgs::LoadMap::Request& request,
                        lidar_slam2_msgs::LoadMap::Response& response);
    bool rebuildMapService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    void initializeParameters();
    void initializeParameters(const std::string& config_file_path);

    void timerCallback(const ros::TimerEvent&);

    void performRelocalization();
    void updatePoseFromLoopClosure(const Sophus::SE3d& new_pose);

    // 动态点过滤（统计离群点去除）
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterDynamicPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    ros::NodeHandle nh_;

    ros::Subscriber pointcloud_sub_;

    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher map_pub_;
    ros::Publisher display_map_pub_;      // 显示用细地图（全量）
    ros::Publisher ground_map_pub_;       // 地面点地图
    ros::Publisher objects_map_pub_;      // 非地面/货物地图（raw）
    ros::Publisher objects_clean_map_pub_; // 非地面/货物地图（clean，BEV过滤后）
    ros::Publisher current_cloud_pub_;
    ros::Publisher path_pub_;

    // v12: cargo core box marker 由 ndt_slam_node 直接发布
    ros::Publisher cargo_core_bbox_marker_pub_;
    bool publish_cargo_core_box_marker_ = true;

    struct CargoDisplayBoxState {
        bool valid = false;
        int track_id = -1;
        Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
        Eigen::Vector3f size = Eigen::Vector3f::Zero();
        int reject_count = 0;
        int stable_candidate_count = 0;
        Eigen::Vector3f candidate_center_base = Eigen::Vector3f::Zero();
        Eigen::Vector3f candidate_size = Eigen::Vector3f::Zero();
        double last_update_time = 0.0;
    };

    CargoDisplayBoxState cargo_display_box_;
    double cargo_display_center_alpha_moving_ = 0.25;
    double cargo_display_center_alpha_static_ = 0.12;
    double cargo_display_size_alpha_ = 0.08;
    double cargo_display_max_center_jump_ = 0.80;
    double cargo_display_max_size_ratio_ = 1.35;
    int cargo_display_reinit_after_rejects_ = 3;
    int cargo_display_min_core_points_ = 30;
    int cargo_display_min_observed_frames_ = 5;
    double cargo_display_marker_lifetime_ = 0.30;

    struct CargoDisplayCandidate {
        bool valid = false;
        int track_id = -1;
        int state = 0;
        int observed_frames = 0;
        int core_points = 0;
        Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
        Eigen::Vector3f size = Eigen::Vector3f::Zero();
        double overlap = 0.0;
        double score = 0.0;
    };

    void publishCargoCoreBoxMarker(const CargoDisplayBoxState& box);
    void publishDeleteCargoCoreBoxMarker();
    void updateAndPublishCargoCoreDisplayBox(const CargoDisplayCandidate& c, bool crane_stopped);
    void appendBoxLineList(visualization_msgs::Marker& m, const Eigen::Vector3f& center, const Eigen::Vector3f& size);

    // 轨迹历史
    nav_msgs::Path path_msg_;
    int path_max_size_ = 5000;  // 最大轨迹点数

    ros::ServiceServer reset_srv_;
    ros::ServiceServer set_pose_srv_;
    ros::ServiceServer relocalize_srv_;
    ros::ServiceServer save_map_srv_;
    ros::ServiceServer load_map_srv_;
    ros::ServiceServer rebuild_map_srv_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

    std::string pointcloud_topic_ = "/points_raw";
    std::string odom_topic_ = "/odom";
    std::string pose_topic_ = "/current_pose";
    std::string map_topic_ = "/map";
    std::string current_cloud_topic_ = "/mapping_current_cloud";
    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";
    std::string lidar_odom_frame_ = "odom_lidar";
    std::string map_frame_ = "map";

    bool use_sim_time_ = false;
    bool publish_odom_tf_ = true;
    bool invert_odom_tf_ = true;

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
    Sophus::SE3d refined_pose_;           // ICP 精炼位姿（用于地图插入）
    std::atomic<bool> has_refined_pose_{false};  // 是否有可用的精炼位姿
    std::atomic<bool> refined_pose_high_quality_{false};  // 精炼位姿是否满足高质量入图条件
    bool initialized_ = false;

    // ========== Crane Motion Constraint（天车运动约束）==========
    bool crane_constraint_enabled_ = false;
    bool lock_z_ = true;
    bool constrain_z_ = false;       // 是否限制 z 漂移范围
    std::string fixed_z_source_ = "config";  // config 或 first_frame
    double fixed_z_value_ = 0.0;     // 配置文件中的固定 z 值
    bool lock_roll_ = true;
    bool lock_pitch_ = true;
    bool lock_yaw_ = false;
    bool constrain_yaw_ = false;
    double fixed_z_ = 0.0;
    double fixed_roll_ = 0.0;
    double fixed_pitch_ = 0.0;
    double fixed_yaw_ = 0.0;
    double max_abs_z_drift_ = 0.10;
    double max_roll_deg_ = 0.3;
    double max_pitch_deg_ = 0.3;
    double max_yaw_deg_ = 1.0;
    bool first_pose_initialized_ = false;

    // 约束函数
    Sophus::SE3d applyCraneMotionConstraint(const Sophus::SE3d& raw_pose, const std::string& stage);
    void so3ToRpy(const Sophus::SO3d& r, double& roll, double& pitch, double& yaw);
    Sophus::SO3d rpyToSO3(double roll, double pitch, double yaw);
    ros::Time last_stamp_;
    std::atomic<bool> tracking_lost_{false};

    std::mutex cloud_mutex_;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> cloud_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::condition_variable tracking_cv_;
    bool shutdown_ = false;
    std::thread process_thread_;

    // NDT_OMP 配准器
    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_;

    // NDT_OMP 配置参数
    double ndt_resolution_ = 1.0;
    double ndt_step_size_ = 0.1;
    double ndt_transformation_epsilon_ = 0.01;
    int ndt_max_iterations_ = 100;

    // KISS-ICP config (保留用于参数读取兼容)
    kiss_icp::pipeline::KISSConfig kiss_icp_config_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;     // 配准用粗地图（体素较大）
    pcl::PointCloud<pcl::PointXYZ>::Ptr display_map_;    // 显示用细地图（体素较小，保留货物轮廓）
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_map_;     // 地面点地图（粗体素）
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_map_;    // 非地面/货物/设备地图（细体素，保留轮廓）
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_clean_map_; // clean objects（BEV过滤后，更干净）

    // rebuild 用的中间数据（用于 save_map 输出调试/检测 PCD）
    pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_objects_filtered_;    // 过滤后的 objects
    pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_payload_candidate_;   // 吊货候选
    pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_payload_dynamic_;     // 动态吊货
    pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_candidate_;     // 人体候选
    pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_dynamic_;       // 动态人体
    pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_pending_;       // 待确认人体
    pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_ground_raw_;          // 原始地面点
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_transformed_;
    std::mutex map_mutex_;
    std::mutex task_queue_mutex_;
    std::condition_variable task_cv_;
    std::queue<MappingTask> task_queue_;
    std::vector<std::thread> worker_threads_;
    std::atomic<bool> running_{true};
    int num_worker_threads_ = 0;
    double voxel_size_ = 0.2;            // 配准地图体素大小
    double display_voxel_size_ = 0.1;    // 显示地图体素大小（全量）
    double ground_voxel_size_ = 0.15;    // 地面点体素大小（较粗）
    double objects_voxel_size_ = 0.06;   // 非地面/货物体素大小（很细，保留轮廓）
    double max_map_size_ = 200.0;

    // 网格局部地面模型参数
    double grid_cell_size_ = 2.0;        // XY 网格大小（米）
    double height_above_ground_ = 0.35;  // 高于此值才算 objects（米）

    // 传感器近场过滤参数（去除起重机抓臂、吊具等固定结构）
    double near_field_radius_ = 4.0;     // 水平距离小于此值的点（米）
    double near_field_z_min_ = 3.0;      // Z 高于此值才过滤（米，base_link 坐标系）
    bool use_voxel_filter_ = true;
    int frame_count_ = 0;
    int map_update_interval_ = 1;

    // 近场过滤可视化
    ros::Publisher near_field_removed_pub_;

    // clean map 时间一致性：每个 BEV cell 被多少个关键帧观测到
    struct BevKey { int x, y; bool operator<(const BevKey& o) const { return x<o.x||(x==o.x&&y<o.y); } };
    std::map<BevKey, int> bev_observation_count_;
    int clean_min_observations_ = 2;  // 至少被 2 个关键帧观测到

    // clean map 异步构建
    std::thread clean_rebuild_thread_;
    std::atomic<bool> clean_rebuild_running_{false};

    bool has_first_odom_ = false;
    Eigen::Vector3d last_position_;
    Eigen::Quaterniond last_orientation_;

    LoopClosureDetector loop_closure_detector_;
    PoseGraphOptimizer pose_graph_optimizer_;
    int loop_detection_interval_ = 10;
    int keyframe_count_ = 0;

    // P0: DuplicateFrameGuard 内容指纹
    struct FrameSignature {
        size_t cloud_size = 0;
        double stamp = 0.0;
        Eigen::Vector3d pose_xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3f first_pt = Eigen::Vector3f::Zero();
        Eigen::Vector3f mid_pt = Eigen::Vector3f::Zero();
        Eigen::Vector3f last_pt = Eigen::Vector3f::Zero();
        Eigen::Vector3f centroid_sample = Eigen::Vector3f::Zero();
        uint64_t hash = 0;
    };

    FrameSignature last_frame_signature_;
    uint64_t frame_seq_ = 0;
    double last_processed_stamp_ = -1.0;
    uint64_t skipped_duplicate_frames_ = 0;
    size_t last_clean_points_ = 0;

    FrameSignature computeFrameSignature(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const ros::Time& stamp,
        const Sophus::SE3d& pose);
    bool isDuplicateFrameBySignature(const FrameSignature& cur) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_;
    Sophus::SE3d relocalized_pose_;
    ros::Timer timer_;

    // Loop closure deduplication
    std::set<std::pair<int, int>> processed_loops_;

    // 异步地图重建
    std::thread rebuild_thread_;
    std::atomic<bool> rebuild_pending_{false};
    std::atomic<bool> rebuild_running_{false};
    void asyncRebuildGlobalMap();

    // 动态点过滤参数
    bool use_dynamic_filter_ = true;         // 是否启用动态点过滤
    int sor_mean_k_ = 20;                    // SOR 邻域点数
    double sor_stddev_mul_thresh_ = 1.5;     // SOR 标准差倍数阈值

    // BasePayloadChannelFilter: base_link 下中间通道吊货候选筛选
    BasePayloadChannelFilter channel_filter_;
    BasePayloadChannelConfig channel_filter_config_;

    // PayloadTrackManager: 双坐标系吊货跟踪
    PayloadTrackManager payload_tracker_;
    PayloadTrackerConfig payload_tracker_config_;

    // P0.5: CargoBoxEstimator 货物框估计器
    CargoBoxEstimator cargo_box_estimator_;
    CargoBoxEstimatorConfig cargo_box_estimator_config_;

    // 通道过滤 debug 话题
    ros::Publisher payload_channel_pub_;      // /payload_channel_cloud
    ros::Publisher payload_candidate_pub_;    // /payload_candidate_cloud
    ros::Publisher safe_objects_pub_;         // /safe_objects_cloud
    ros::Publisher payload_dynamic_pub_;      // /payload_dynamic_cloud
    ros::Publisher payload_pending_pub_;      // /payload_pending_cloud
    ros::Publisher cargo_dynamic_removed_pub_; // /cargo_dynamic_removed_cloud

    // CargoBoxEstimator V2 调试 topic
    ros::Publisher cargo_core_points_pub_;     // /cargo_core_points_cloud
    ros::Publisher cargo_hag_filtered_pub_;    // /cargo_hag_filtered_cloud
    ros::Publisher cargo_components_pub_;      // /cargo_bev_components_cloud
    ros::Publisher cargo_rejected_low_pub_;    // /cargo_rejected_low_points_cloud

    // 人体过滤模块
    HumanObjectDynamicFilter human_filter_;
    HumanObjectFilterConfig human_filter_config_;
    HumanTrackingConfig human_tracking_config_;
    HumanEraserConfig human_eraser_config_;

    // 人体过滤 debug 话题
    ros::Publisher human_candidate_pub_;      // /human_candidate_cloud
    ros::Publisher human_dynamic_pub_;        // /human_dynamic_cloud
    ros::Publisher human_pending_pub_;        // /human_pending_cloud
    ros::Publisher human_trajectory_pub_;     // /human_trajectory_capsule
    ros::Publisher human_removed_pub_;        // /human_removed_history_cloud

    // 动态事件管理器（统一管理吊货和人体的动态事件）
    DynamicEventManager dynamic_event_manager_;
    DynamicEventConfig dynamic_event_config_;

    // P1: Cargo deny history（持久化）
    struct DenyCellEntry {
        double first_seen_time;
        double last_seen_time;
        int hit_count;
    };
    std::map<std::pair<int,int>, DenyCellEntry> cargo_deny_history_;
    double cargo_deny_ttl_ = 120.0;  // cargo deny cells 保留 120 秒

    // 添加 cargo deny cells
    void addCargoDenyCells(const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
                           double current_time);

    // 检查 cell 是否被 cargo deny
    bool isCargoDenied(double x, double y, double current_time) const;

    // 清理过期的 cargo deny cells
    void cleanupExpiredCargoDenyCells(double current_time);

    // ========== P0-3: 3D Dynamic Deny Volume（替代 2D BEV deny）==========
    struct DynamicDenyVolume3D {
        int ix;
        int iy;
        float z_min;
        float z_max;
        double stamp;
        int source;      // 0=cargo, 1=human
        int track_id;
    };

    // 3D deny volume 存储（key = (ix, iy)）
    std::map<std::pair<int,int>, std::vector<DynamicDenyVolume3D>> dynamic_deny_volume_map_;
    double dynamic_deny_resolution_ = 0.15;  // 与 BEV 分辨率一致
    double dynamic_deny_ttl_ = 8.0;          // 3D deny volume 保留时间

    // 添加 3D deny volume
    void addCargoDenyVolume3D(const CargoBox& remove_box, double current_time, int track_id);

    // 清理过期的 3D deny volumes
    void cleanupExpiredCargoDenyVolumes3D(double current_time);

    // 检查点是否被 3D deny volume 拒绝
    bool isPointDeniedBy3DHistory(float x, float y, float z) const;

    // ========== v6: DynamicHistoryEraser 增量反删 ==========
    struct SweptVolumeMap {
        Eigen::Vector3f min_map;
        Eigen::Vector3f max_map;
        double stamp;
        int track_id;
        bool from_fallback;
    };

    // 当前帧新增的 swept volume（用于立即反删）
    std::vector<SweptVolumeMap> new_cargo_volumes_this_frame_;

    // 历史 swept volume（用于 CleanMap rebuild）
    std::vector<SweptVolumeMap> cargo_swept_history_;
    double cargo_swept_ttl_ = 15.0;  // 历史 volume 保留时间

    // 从 cloud 中删除被 swept volume 覆盖的点
    size_t eraseDynamicPointsFromCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<SweptVolumeMap>& volumes);

    // 清理过期的 swept volume
    void cleanupExpiredSweptVolumes(double current_time);

    // ========== P0-1: 新的关键帧提交流程 ==========
    // 重写的关键帧提交函数，确保正确的处理顺序：
    // 1. ground/objects 分割
    // 2. CargoBoxV2 + PayloadTracker
    // 3. 吊货点删除
    // 4. HumanFilter
    // 5. MapCommit（最后）
    void commitKeyFrameWithDynamicFiltering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const Sophus::SE3d& pose,
        const ros::Time& stamp);

    // 从 objects 中删除吊货 remove_box 内的点（3D 检查）
    void removePointsInsideCargoRemoveBoxes3D(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_base,
        const std::vector<Box3D>& remove_boxes_map,
        const Eigen::Matrix4d& T_map_base,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_base,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& removed_base);

    // P2: 在 base_link 坐标系下删除吊货点（不用变换到 map）
    void removePointsInsideCargoRemoveBoxesBase(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_base,
        const std::vector<CargoBox>& remove_boxes_base,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_base,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& removed_base);

    // v8: 统一 active remove box 生成
    struct ActiveRemoveDecision {
        bool active = false;
        CargoBox box;
        std::string source;
        int overlap = 0;
        std::string reason;
    };

    ActiveRemoveDecision buildActiveRemoveBoxForTrack(
        const ObjectTrack& track,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_base,
        double stamp);

    int countPointsInsideBoxBase(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const CargoBox& box);

    CargoBox expandCoreToRemoveBox(const CargoBox& core_box);

    // 吊货跟踪信息发布（用于避障节点）
    ros::Publisher payload_track_info_pub_;
    void publishPayloadTrackInfo(const ros::Time& stamp);

    // ========== 长期建图功能 ==========
    // MotionGate：静止检测和门控
    bool longterm_mapping_enabled_ = false;
    bool motion_gate_enabled_ = false;
    double motion_gate_min_translation_m_ = 0.30;
    double motion_gate_min_rotation_deg_ = 3.0;
    double motion_gate_min_time_sec_ = 2.0;
    Sophus::SE3d last_keyframe_pose_for_gate_;
    ros::Time last_keyframe_time_for_gate_;
    bool is_stationary_ = false;
    int stationary_frame_count_ = 0;
    int moved_frame_count_ = 0;

    // P1: MotionGate stationary anchor（防止静止漂移误触发）
    bool stationary_anchor_valid_ = false;
    Sophus::SE3d stationary_anchor_pose_;
    double stationary_start_time_ = 0.0;
    int moving_confirm_frames_ = 0;
    double motion_gate_stationary_drift_ignore_radius_ = 0.60;
    int motion_gate_moving_confirm_frames_ = 2;
    double motion_gate_moving_min_velocity_ = 0.08;
    double last_frame_stamp_for_gate_ = 0.0;
    Eigen::Vector3d last_frame_pos_for_gate_ = Eigen::Vector3d::Zero();

    // v8: PoseFreeze - 静止时冻结发布姿态
    Sophus::SE3d published_pose_;
    bool motion_gate_stationary_ = false;
    int moving_confirm_count_ = 0;
    bool stationary_freeze_tf_odom_ = true;
    bool stationary_freeze_xy_ = true;
    bool stationary_freeze_yaw_ = true;
    double stationary_pose_freeze_release_m_ = 1.50;  // v12: 增大释放阈值
    int stationary_move_confirm_frames_ = 5;  // v12: 增加确认帧数

    // v12: PoseFreeze 状态机
    enum class PoseFreezeState {
        MOVING = 0,
        STATIONARY_FROZEN = 1,
        RELEASE_BLEND = 2
    };
    PoseFreezeState pose_freeze_state_ = PoseFreezeState::MOVING;
    Sophus::SE3d release_start_pose_;
    Sophus::SE3d release_target_pose_;
    int pose_release_frame_ = 0;
    int pose_release_total_frames_ = 10;

    Sophus::SE3d selectPublishedPose(const Sophus::SE3d& constrained_pose, const ros::Time& stamp);
    bool shouldReleasePoseFreeze(double drift, int confirm, double frame_velocity, double fitness, std::string& reason);
    Sophus::SE3d computeReleaseBlendPose(const Sophus::SE3d& current_target_pose);

    // 关键帧 active window
    int max_active_keyframes_ = 80;
    int keyframes_since_last_release_ = 0;
    int keyframe_release_interval_ = 10;

    // MotionGate 判定函数
    bool shouldCommitKeyframe(const Sophus::SE3d& current_pose, const ros::Time& current_time);
    void releaseOldKeyframeClouds();

    // 磁盘 tile 写入
    bool persistent_map_enabled_ = false;
    std::string persistent_map_root_dir_;
    double tile_size_m_ = 20.0;
    int flush_interval_sec_ = 60;
    int max_dirty_tiles_ = 20;

    // 多层 tile 支持
    struct TileLayers {
        pcl::PointCloud<pcl::PointXYZ>::Ptr registration;
        pcl::PointCloud<pcl::PointXYZ>::Ptr display;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground;
        pcl::PointCloud<pcl::PointXYZ>::Ptr objects;
    };
    std::map<std::string, TileLayers> dirty_tiles_;
    ros::Time last_flush_time_;

    // tile 体素大小配置
    double tile_voxel_registration_ = 0.30;
    double tile_voxel_display_ = 0.10;
    double tile_voxel_ground_ = 0.15;
    double tile_voxel_objects_ = 0.08;

    // runtime status
    int total_frames_ = 0;
    int total_keyframes_ = 0;
    int active_keyframes_ = 0;
    int dirty_tile_count_ = 0;
    int flushed_tile_count_ = 0;
    double delta_translation_ = 0.0;
    double delta_yaw_ = 0.0;
    double average_process_time_ms_ = 0.0;
    double average_ndt_time_ms_ = 0.0;
    bool memory_guard_triggered_ = false;
    bool disk_guard_triggered_ = false;
    ros::Time last_flush_time_local_;
    ros::Time last_active_map_rebuild_time_;

    void writeRuntimeStatus();
    void flushDirtyTiles();

    // ========== 统一提交检查 ==========
    bool commit_enabled_ = true;              // observe_only 模式时为 false
    bool mapping_paused_by_memory_guard_ = false;
    bool ndt_health_bad_ = false;
    bool canCommit();                         // 统一检查是否可以提交

    // ========== 内存保护（分级） ==========
    enum class MemoryGuardLevel { OK, SOFT, HARD, EMERGENCY };
    bool memory_guard_enabled_ = false;
    int soft_threshold_mb_ = 6000;            // 6GB: 释放缓存 + flush
    int hard_threshold_mb_ = 7000;            // 7GB: 暂停地图 commit
    int emergency_threshold_mb_ = 8000;       // 8GB: 降采样 active map
    int memory_check_interval_sec_ = 30;
    ros::Time last_memory_check_time_;
    MemoryGuardLevel memory_guard_level_ = MemoryGuardLevel::OK;

    void checkMemoryGuard();
    void forceDownsampleAllMaps();
    void releaseMemoryCache();
    long getProcessMemoryMB();
    void rebuildActiveMapFromRecentKeyframes();

    // ========== 磁盘保护 ==========
    bool disk_guard_enabled_ = false;
    double min_free_disk_gb_ = 30.0;
    bool pause_mapping_when_disk_low_ = true;

    bool checkDiskGuard();
    double getDiskFreeGB();

    // ========== 点云看门狗 ==========
    ros::Time last_pointcloud_time_;
    double pointcloud_stale_timeout_sec_ = 10.0;
    bool pointcloud_stale_ = false;

    // ========== NDT 健康监控 ==========
    double last_ndt_fitness_ = 0.0;
    int consecutive_high_fitness_ = 0;
    double fitness_warning_threshold_ = 2.0;
    int fitness_warning_count_ = 50;

    // ========== Active Map 重建（非阻塞） ==========
    int rebuild_every_keyframes_ = 10;
    std::atomic<bool> active_map_rebuild_running_{false};

    // 边缘保留点云融合
    struct VoxelData {
        std::vector<Eigen::Vector3d> points;
        Eigen::Vector3d centroid;
        Eigen::Matrix3d covariance;
        int observation_count;
        bool is_edge;
    };

    // 使用协方差分析保留边缘
    pcl::PointCloud<pcl::PointXYZ>::Ptr edgePreservingMerge(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& existing_map,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& new_cloud,
        double voxel_size,
        int min_observations = 2);

    // TF 时间戳去重
    ros::Time last_tf_stamp_;

    // ========== 工程化建图功能 ==========

    // 更新关键帧质量指标
    void updateKeyFrameMetrics();

    // 保存多层地图到目录
    void saveMultiLayerMaps(const std::string& session_dir);

    // 从关键帧重建地图（不叠加旧 PCD）
    void rebuildMapFromKeyframes(const std::string& session_dir);

    // 生成地图质量报告
    void generateMapQualityReport(const std::string& session_dir);

    // 离线精配准模式
    void offlineRefinePoses(const std::string& session_dir, const std::string& localization_map_path);

    // 导出导航地图
    void exportNavigationMap(const std::string& session_dir, double resolution = 0.1);

    // 地图质量统计
    struct MapQualityStats {
        int total_keyframes = 0;
        int accepted_keyframes = 0;
        int rejected_keyframes = 0;
        double avg_fitness = 0.0;
        double avg_inlier_ratio = 0.0;
        double map_thickness_avg = 0.0;
        double map_thickness_max = 0.0;
        int localization_points = 0;
        int detail_points = 0;
        int ground_points = 0;
        int objects_raw_points = 0;
        int objects_clean_points = 0;
        double trajectory_length = 0.0;
        int loop_closures = 0;
    };
};

} // namespace ndt_slam
