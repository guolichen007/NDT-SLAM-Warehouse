#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
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

// v8-stable-r3: CraneMotionEKF
#include <ndt_slam/crane_motion_ekf.hpp>

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

// P0-5: CargoBoxSource 枚举
enum CargoBoxSource : int {
    BOX_SOURCE_NONE = 0,
    BOX_SOURCE_V2_CORE = 1,
    BOX_SOURCE_LAST_GOOD = 2,
    BOX_SOURCE_OLD_BBOX = 3,
    BOX_SOURCE_CENTER_ONLY = 4
};

// P0-5: payload_track_info 索引常量
constexpr int IDX_VALID = 0;
constexpr int IDX_TRACK_ID = 1;
constexpr int IDX_STATE = 2;
constexpr int IDX_CENTROID_X = 3;
constexpr int IDX_CENTROID_Y = 4;
constexpr int IDX_CENTROID_Z = 5;
constexpr int IDX_VEL_X = 6;
constexpr int IDX_VEL_Y = 7;
constexpr int IDX_VEL_Z = 8;
constexpr int IDX_BBOX_MIN_X = 9;
constexpr int IDX_BBOX_MIN_Y = 10;
constexpr int IDX_BBOX_MIN_Z = 11;
constexpr int IDX_BBOX_MAX_X = 12;
constexpr int IDX_BBOX_MAX_Y = 13;
constexpr int IDX_BBOX_MAX_Z = 14;
constexpr int IDX_POINT_COUNT = 15;
constexpr int IDX_SCORE = 16;
constexpr int IDX_BOTTOM_HAG = 17;
constexpr int IDX_SUPPORT_RATIO = 18;
constexpr int IDX_BOX_SOURCE = 19;
constexpr int PAYLOAD_TRACK_INFO_SIZE = 20;

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
    void publishRuntimePath(const Sophus::SE3d& pose, const ros::Time& stamp);

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
    ros::Publisher runtime_path_pub_;

    // 轨迹历史
    nav_msgs::Path path_msg_;
    int path_max_size_ = 5000;  // 最大轨迹点数

    // 运行时轨迹（不受 MotionGate 影响）
    nav_msgs::Path runtime_path_msg_;
    Eigen::Matrix4d last_path_pose_ = Eigen::Matrix4d::Identity();
    bool has_last_path_pose_ = false;

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

    // ========== 调试配置 ==========
    struct DebugConfig {
        bool publish_runtime_path = false;
    } debug_cfg_;

    // ========== v8-stable-r3: CraneMotionEKF ==========
    CraneMotionEKF crane_motion_ekf_;
    CraneMotionEKFConfig crane_motion_ekf_cfg_;
    bool crane_motion_ekf_enabled_ = true;

    // ========== v8-stable-r3: SoftYawFilter ==========
    bool soft_yaw_enabled_ = true;
    bool filtered_yaw_initialized_ = false;
    double filtered_yaw_rad_ = 0.0;

    double yaw_filter_alpha_stationary_ = 0.04;
    double yaw_filter_alpha_moving_ = 0.18;
    double yaw_filter_alpha_speed_extra_ = 0.05;

    double yaw_max_step_stationary_rad_ = 0.08 * M_PI / 180.0;
    double yaw_max_step_moving_rad_ = 0.35 * M_PI / 180.0;
    double yaw_warn_raw_filtered_diff_rad_ = 3.0 * M_PI / 180.0;

    double updateSoftYaw(double raw_yaw, double speed_xy, bool is_stationary);
    Sophus::SE3d applyCraneOutputConstraint(const Sophus::SE3d& pose_in,
                                            bool is_stationary,
                                            double speed_xy);

    // ========== v8-stable-r3: Registration Input ==========
    double ndt_input_voxel_size_ = 0.30;
    int object_weight_repeat_ = 2;
    double ground_sample_ratio_ = 0.20;
    int max_ndt_points_ = 8000;
    int min_objects_for_weighting_ = 500;

    pcl::PointCloud<pcl::PointXYZ>::Ptr sampleCloudByRatio(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double ratio);
    void voxelDownsampleInPlace(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double leaf);
    void limitCloudUniformInPlace(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int max_points);
    pcl::PointCloud<pcl::PointXYZ>::Ptr buildRegistrationCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& human_safe_objects,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud);

    // ========== v8-stable-r3: Adaptive NDT ==========
    bool adaptive_ndt_enabled_ = true;
    double adaptive_target_total_ms_ = 80.0;
    double adaptive_emergency_total_ms_ = 120.0;
    double last_total_ms_ = 0.0;
    int consecutive_good_perf_frames_ = 0;

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

    // P0: processCloudThread 重复帧检测
    ros::Time last_processed_frame_stamp_;
    size_t last_processed_frame_size_ = 0;
    uint64_t last_processed_frame_hash_ = 0;
    int duplicate_frame_skip_count_ = 0;

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
    void publishPayloadTrackInfoInvalid(const std::string& reason);
    void buildLockedOdomFixedCargoBox(const ros::Time& stamp);

    // Commit C: payload precise box info

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
    // v8-stable-r3-hotfix-minimal: PoseFreeze 已禁用
    bool stationary_freeze_tf_odom_ = false;
    bool stationary_freeze_xy_ = false;
    bool stationary_freeze_yaw_ = false;
    double stationary_pose_freeze_release_m_ = 0.80;

    // P0-3: MapCommit evidence only. Does NOT affect runtime odom/TF/path.
    Sophus::SE3d last_raw_ndt_pose_;
    Sophus::SE3d last_commit_raw_pose_;
    Sophus::SE3d last_commit_refined_pose_;
    Sophus::SE3d last_commit_runtime_pose_;

    bool has_last_raw_ndt_pose_ = false;
    bool has_commit_gate_reference_ = false;
    int stationary_move_confirm_frames_ = 3;

    Sophus::SE3d selectPublishedPose(const Sophus::SE3d& constrained_pose, const ros::Time& stamp);

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

    // ========== Commit B: cargo target 一致性 ==========
    int selected_payload_track_id_ = -1;
    bool has_selected_payload_track_ = false;
    ros::Time selected_payload_stamp_;

    // ========== Commit C: payload precise box info ==========
    ros::Publisher payload_precise_box_info_pub_;
    static constexpr int PRECISE_BOX_INFO_SIZE = 40;
    static constexpr int IDX_CORNER_MAP_START = 16;

    // ========== HookFixedCargoDetector ==========
    struct HookCargoDetection {
        bool valid = false;
        int local_id = 0;
        Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
        Eigen::Vector3f size_visible = Eigen::Vector3f::Zero();
        float z05 = 0.0f;
        float z50 = 0.0f;
        float z95 = 0.0f;
        float visible_height = 0.0f;
        float xy_area = 0.0f;  // XY 面积
        pcl::PointCloud<pcl::PointXYZ>::Ptr core_points_base;
        float score = 0.0f;
        std::string reject_reason;
    };

    struct HookCargoBottomEstimate {
        bool valid = false;
        float bottom_z_base = 0.0f;
        float top_z_base = 0.0f;
        float height = 0.0f;
        float uncertainty = 0.0f;
        float confidence = 0.0f;
        std::string source;
    };

    // ========== OdomAnchorBox 配置 ==========
    struct OdomAnchorBoxConfig {
        bool enabled = true;

        // 绿色框中心锚点（默认 base_link/odom 原点）
        float anchor_x = 0.0f;
        float anchor_y = 0.0f;

        // 检测和 marker 降频
        float detect_rate_hz = 5.0f;
        float marker_rate_hz = 5.0f;

        // debug 点云发布（默认关闭）
        bool publish_debug_points = false;
        bool publish_selected_core_points = false;
        bool publish_raw_candidate_points = false;
        bool publish_default_box_marker = false;

        // 日志控制
        bool verbose_debug = false;
        float summary_log_period = 2.0f;

        // 旧 cargo 链路开关（默认关闭）
        bool use_global_payload_tracker = false;
        bool use_cargobox_v2 = false;
        bool use_dynamic_history_eraser = false;
        bool enable_hook_cargo_removal = false;

        // 检测窗口围绕 anchor 裁剪
        float search_half_x = 1.20f;
        float search_half_y = 1.20f;
        float search_z_min = 0.05f;
        float search_z_max = 3.20f;

        // 默认小参考框（仅 debug）
        float default_size_x = 0.50f;
        float default_size_y = 0.35f;
        float default_size_z = 0.25f;

        // 检测到货物后的尺寸范围
        float min_size_x = 0.60f;
        float min_size_y = 0.40f;
        float min_size_z = 0.20f;
        float max_size_x = 2.50f;
        float max_size_y = 1.60f;
        float max_size_z = 2.00f;

        float size_margin_x = 0.10f;
        float size_margin_y = 0.10f;
        float size_margin_z = 0.05f;

        int lock_confirm_frames = 2;
        int strong_min_points = 50;
        int weak_min_points = 10;

        int size_update_confirm_frames = 5;
        float size_change_min_ratio = 0.20f;
        float size_change_max_ratio = 0.60f;
        float size_update_alpha = 0.15f;

        float bottom_alpha_points = 0.25f;
        float bottom_alpha_hold = 0.05f;
        float bottom_max_uncertainty = 0.35f;

        float lost_hold_sec = 5.0f;
        float lost_clear_sec = 15.0f;

        // Tight Box 子配置
        struct TightBoxConfig {
            bool enabled = true;

            // 软对称模式
            std::string anchor_symmetry_mode = "soft";  // strict / soft / off
            float max_center_offset_m = 0.35f;

            // HAG 预过滤
            bool hag_filter_enabled = true;
            float hag_min_m = 0.15f;
            float hag_max_m = 2.50f;

            // 分位数参数
            float percentile_low = 0.08f;
            float percentile_high = 0.92f;

            // 框扩展 margin
            float margin_xy_m = 0.05f;
            float margin_z_m = 0.03f;

            // LOCKED 后尺寸自适应
            std::string size_update_mode = "adaptive";  // locked / adaptive
            float size_update_alpha = 0.30f;
            float max_size_change_per_frame_m = 0.10f;

            // 子簇重聚类
            bool sub_cluster_enabled = true;
            float sub_cluster_tolerance_m = 0.10f;
            int sub_cluster_min_points = 20;
        } tight_box;

        // Cargo Warning 子配置
        struct CargoWarningConfig {
            bool enabled = true;
            bool publish_alarm_msg = false;  // 不向外发布正式报警，只发布 debug
            bool publish_debug_marker = true;

            float level1_distance_m = 3.0f;
            float level2_distance_m = 5.0f;
            float min_vertical_clearance_m = 0.80f;

            bool cargo_bottom_use_uncertainty = true;
            float cargo_bottom_extra_margin_m = 0.05f;

            float obstacle_top_percentile = 0.95f;
            int obstacle_min_points = 5;
            float obstacle_cluster_tolerance_m = 0.25f;

            bool exclude_ground = true;
            float ground_hag_min_m = 0.20f;

            bool exclude_self_cargo = true;
            float self_cargo_margin_xy_m = 0.45f;
            float self_cargo_margin_z_m = 0.35f;

            int debounce_frames = 2;
            float clear_hold_sec = 0.5f;

            int level1_alarm_code = 17;
            int level2_alarm_code = 18;
            int clear_alarm_code = 0;
        } cargo_warning;
    };

    OdomAnchorBoxConfig odom_anchor_config_;

    // 降频控制
    ros::Time last_anchor_detect_stamp_;
    ros::Time last_anchor_marker_stamp_;
    ros::Time last_anchor_summary_stamp_;

    bool shouldRunOdomAnchorDetect(const ros::Time& stamp) {
        if (last_anchor_detect_stamp_.isZero()) return true;
        return (stamp - last_anchor_detect_stamp_).toSec() >=
               1.0 / odom_anchor_config_.detect_rate_hz;
    }

    bool shouldPublishOdomAnchorMarker(const ros::Time& stamp) {
        if (last_anchor_marker_stamp_.isZero()) return true;
        return (stamp - last_anchor_marker_stamp_).toSec() >=
               1.0 / odom_anchor_config_.marker_rate_hz;
    }

    // 获取 anchor XY
    Eigen::Vector2f getCargoAnchorXY() const {
        return Eigen::Vector2f(odom_anchor_config_.anchor_x, odom_anchor_config_.anchor_y);
    }

    struct HookFixedCargoConfig {
        bool enabled = true;
        float roi_center_x = 0.0f;
        float roi_center_y = -2.2f;
        float roi_half_x = 1.5f;
        float roi_half_y = 1.5f;
        float roi_z_min = 0.25f;
        float roi_z_max = 3.0f;
        float voxel_leaf = 0.05f;
        float cluster_tolerance = 0.20f;
        int min_cluster_points = 15;
        int max_cluster_points = 8000;
        float reject_rope_radius = 0.08f;
        float reject_rope_min_z = 2.0f;
        float reject_structure_z = 3.2f;
        std::string xy_mode = "roi_center";
        float xy_alpha = 0.15f;
        float z_alpha = 0.30f;
        float size_alpha = 0.20f;
        float min_long_side = 0.30f;
        float min_short_side = 0.20f;
        float min_visible_height = 0.08f;
        float max_long_side = 4.0f;
        float max_short_side = 3.0f;
        float max_height = 3.0f;
        int bottom_min_points = 15;
        float visible_side_min_height = 0.30f;
        float bottom_band_height = 0.10f;
        int bottom_band_min_points = 5;
        int bottom_band_min_xy_cells = 2;
        float bottom_xy_cell_size = 0.10f;
        float points_uncertainty = 0.12f;
        float height_memory_uncertainty = 0.18f;
        float invalid_uncertainty = 0.30f;
        float stable_height_alpha = 0.25f;
        bool allow_visible_box_without_bottom = true;
    };

    HookFixedCargoConfig hook_fixed_config_;
    HookCargoDetection hook_fixed_cargo_;
    HookCargoBottomEstimate hook_fixed_bottom_;
    float stable_height_ = 0.0f;
    bool has_stable_height_ = false;

    ros::Publisher cargo_selected_core_points_pub_;

    HookCargoBottomEstimate estimateCargoBottom(const HookCargoDetection& detection);
    void publishSelectedCorePoints(const HookCargoDetection& detection, const ros::Time& stamp);
    void publishSelectedCorePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Time& stamp);
    Eigen::Vector3f smoothVector(const Eigen::Vector3f& current, const Eigen::Vector3f& new_val, float alpha);
    float smoothFloat(float current, float new_val, float alpha);

    // ========== HookCargoLock 状态机 ==========
    enum class HookCargoLockState {
        EMPTY = 0,
        CANDIDATE = 1,
        LOCKED = 2,
        LOST_HOLD = 3
    };

    struct HookCargoLockConfig {
        bool enabled = true;
        int lock_confirm_frames = 3;
        int size_init_window = 5;
        float lost_hold_sec = 3.0f;
        float lost_clear_sec = 8.0f;
        int strong_min_points = 30;
        int weak_min_points = 5;
        float candidate_hold_sec = 1.0f;
        int candidate_max_weak_frames = 10;
        float size_change_min_ratio = 0.20f;
        float size_change_max_ratio = 0.60f;
        int size_update_confirm_frames = 5;
        float size_update_alpha = 0.15f;
        float bottom_alpha_points = 0.30f;
        float bottom_alpha_memory = 0.15f;
        float bottom_hold_uncertainty_growth = 0.02f;
        float bottom_max_uncertainty = 0.35f;

        // locked association gate 相关
        float locked_update_max_center_dist = 0.65f;
        float locked_update_min_overlap_ratio = 0.30f;
        float locked_update_max_z_jump = 0.45f;
        float locked_update_max_top_jump = 0.60f;
        int locked_update_min_points = 20;

        // 锁定时 strong 条件（比更新时更严格）
        int lock_strong_min_points = 80;
        float lock_min_visible_height = 0.50f;
        float lock_min_xy_area = 0.40f;

        // locked search margin
        float locked_search_margin_x = 0.30f;
        float locked_search_margin_y = 0.30f;

        // 吊物点云去除开关
        bool enable_hook_cargo_removal = true;
    };

    // ========== 统一货物状态结构 ==========
    struct CargoBoxObservation {
        bool valid = false;

        Eigen::Vector3f center_base = Eigen::Vector3f::Zero();   // tight box center in base_link
        Eigen::Vector3f size = Eigen::Vector3f::Zero();          // tight box size
        float z_min = 0.0f;
        float z_max = 0.0f;

        int selected_points = 0;
        float confidence = 0.0f;
        std::string source = "tight_box";
    };

    struct CargoState {
        enum State { EMPTY, CANDIDATE, LOCKED, LOST };

        State state = EMPTY;
        bool valid_geometry = false;
        bool valid_height = false;

        Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
        Eigen::Vector3f size = Eigen::Vector3f::Zero();

        float bottom_z = 0.0f;
        float bottom_safe_z = 0.0f;
        float top_z = 0.0f;
        float bottom_unc = 0.0f;

        int locked_frames = 0;
        ros::Time stamp;
        std::string source;
    };

    CargoState cargo_state_;
    CargoBoxObservation last_tight_box_obs_;

    // 底部高度稳定保护
    int low_bottom_reject_count_ = 0;
    int high_bottom_reject_count_ = 0;

    // 统一配置 getter
    bool isHookCargoRemovalEnabled() const {
        return hook_lock_config_.enable_hook_cargo_removal;
    }

    struct HookCargoLock {
        HookCargoLockState state = HookCargoLockState::EMPTY;
        int confirm_count = 0;
        int weak_count = 0;
        int lost_count = 0;
        int size_update_count = 0;

        ros::Time last_seen_stamp;
        ros::Time last_good_height_stamp;
        ros::Time locked_stamp;

        Eigen::Vector3f locked_size = Eigen::Vector3f::Zero();
        Eigen::Vector3f locked_center_base = Eigen::Vector3f::Zero();  // CargoState 同步

        float stable_bottom_z = 0.0f;
        float stable_top_z = 0.0f;
        float stable_height = 0.0f;
        float bottom_uncertainty = 0.30f;

        bool has_locked_size = false;
        bool has_good_height = false;

        std::deque<Eigen::Vector3f> init_size_buffer;
        std::deque<Eigen::Vector3f> size_candidate_buffer;

        // 重复帧检测
        ros::Time last_hook_processed_stamp;
        uint64_t last_hook_processed_hash = 0;

        // last accepted detection（用于 LOCKED 后显示）
        pcl::PointCloud<pcl::PointXYZ>::Ptr last_accepted_core_points;
        Eigen::Vector3f last_accepted_center = Eigen::Vector3f::Zero();
        bool has_last_accepted = false;
    };

    HookCargoLockConfig hook_lock_config_;
    HookCargoLock hook_lock_;

    // ========== Cargo Warning 数据结构 ==========
    struct CargoWarningData {
        bool valid = false;
        uint8_t level = 0;  // 0=NONE, 1=LEVEL_1, 2=LEVEL_2
        uint16_t alarm_code = 0;

        // 距离和净空
        float distance_to_footprint_m = 0.0f;
        float clearance_m = 0.0f;

        // 货物高度
        float cargo_bottom_z = 0.0f;
        float cargo_bottom_safe_z = 0.0f;
        float cargo_top_z = 0.0f;
        float cargo_bottom_uncertainty = 0.0f;

        // 障碍物信息
        float obstacle_top_z = 0.0f;
        uint32_t obstacle_point_count = 0;
        Eigen::Vector3f obstacle_nearest_point = Eigen::Vector3f::Zero();

        // 货物框
        Eigen::Vector3f cargo_center = Eigen::Vector3f::Zero();
        Eigen::Vector3f cargo_size = Eigen::Vector3f::Zero();

        // 元信息
        std::string source;
        std::string reason;
    };

    // Cargo Warning 状态
    int cargo_warning_debounce_count_ = 0;
    CargoWarningData last_cargo_warning_;
    ros::Time last_cargo_warning_stamp_;

    // Cargo Warning publisher
    ros::Publisher cargo_warning_pub_;
    ros::Publisher cargo_warning_text_pub_;
    ros::Publisher cargo_tight_box_marker_pub_;
    ros::Publisher cargo_warning_zone_marker_pub_;
    ros::Publisher cargo_warning_obstacle_marker_pub_;

    // OdomAnchorBox 新函数
    HookCargoDetection detectCargoAroundOdomAnchor(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base,
        const ros::Time& stamp);
    void publishPayloadTrackInfoFromOdomAnchorBox(const ros::Time& stamp);

    void updateHookCargoLock(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom, const ros::Time& stamp);
    bool isStrongDetection(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom);
    bool isWeakDetection(const HookCargoDetection& det);
    bool isLockStrongDetection(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom);
    bool isBodyStrongCandidate(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom);
    bool isDetectionConsistentWithLockedBox(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom, std::string* reject_reason);
    Eigen::Vector3f computeFixedCenterSize(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom);
    Eigen::Vector3f medianSize(const std::deque<Eigen::Vector3f>& buffer);
    void updateLockedHeight(const HookCargoBottomEstimate& bottom, const ros::Time& stamp, bool initialize);
    void maybeUpdateLockedSize(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom);
    void growUncertainty();
    void clearHookLock();
    uint64_t computeCloudHash(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // CargoState 更新函数
    void updateCargoState(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom, const ros::Time& stamp);

    // Cargo Warning 函数
    CargoWarningData computeCargoWarning(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base,
        const Eigen::Vector3f& cargo_center,
        const Eigen::Vector3f& cargo_size,
        float cargo_bottom_z,
        float cargo_bottom_uncertainty,
        const ros::Time& stamp);
    void publishCargoWarning(const CargoWarningData& warning, const ros::Time& stamp);
    void publishCargoWarningMarkers(
        const Eigen::Vector3f& cargo_center,
        const Eigen::Vector3f& cargo_size,
        const CargoWarningData& warning,
        const ros::Time& stamp);
};

} // namespace ndt_slam
