#include "ndt_slam/ndt_slam.hpp"
#include "ndt_slam/point_cloud_processing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sophus/se3.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <boost/filesystem.hpp>
#include <malloc.h>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>

// KD-Tree 用于特征提取
#include <pcl/kdtree/kdtree_flann.h>

// NDT_OMP
#include <pclomp/ndt_omp.h>
// ICP（用于关键帧精配准）
#include <pcl/registration/icp.h>

#include "lidar_slam2/Utils.hpp"

namespace {
Sophus::SE3d LookupTransform(const std::string &target_frame,
                             const std::string &source_frame,
                             tf2_ros::Buffer &tf2_buffer) {
    std::string err_msg;
    if (tf2_buffer.canTransform(target_frame, source_frame, ros::Time(0), &err_msg)) {
        try {
            auto tf = tf2_buffer.lookupTransform(target_frame, source_frame, ros::Time(0));
            Sophus::SE3d transform;
            transform.translation() = Eigen::Vector3d(
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z
            );
            Eigen::Quaterniond q(
                tf.transform.rotation.w,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z
            );
            transform.so3() = Sophus::SO3d(q);
            return transform;
        } catch (tf2::TransformException &ex) {
            ROS_DEBUG("%s", ex.what());
        }
    }
    ROS_DEBUG("Failed to find tf. Reason=%s", err_msg.c_str());
    return Sophus::SE3d();
}
}  // namespace

namespace ndt_slam {
using lidar_slam2::utils::PointCloud2ToEigen;
using lidar_slam2::utils::GetTimestamps;

static_assert(static_cast<std::uint8_t>(CargoBottomSource::INVALID) ==
              lidar_slam2_msgs::CargoBottomEstimate::SOURCE_INVALID,
              "Cargo source schema drift: INVALID");
static_assert(static_cast<std::uint8_t>(CargoBottomSource::POINTS) ==
              lidar_slam2_msgs::CargoBottomEstimate::SOURCE_POINTS,
              "Cargo source schema drift: POINTS");
static_assert(static_cast<std::uint8_t>(CargoBottomSource::MAP_DIFF) ==
              lidar_slam2_msgs::CargoBottomEstimate::SOURCE_MAP_DIFF,
              "Cargo source schema drift: MAP_DIFF");
static_assert(static_cast<std::uint8_t>(CargoBottomSource::MAP_STATIC) ==
              lidar_slam2_msgs::CargoBottomEstimate::SOURCE_MAP_STATIC,
              "Cargo source schema drift: MAP_STATIC");
static_assert(static_cast<std::uint8_t>(CargoBottomSource::RECENT_STABLE) ==
              lidar_slam2_msgs::CargoBottomEstimate::SOURCE_RECENT_STABLE,
              "Cargo source schema drift: RECENT_STABLE");
static_assert(static_cast<std::uint8_t>(CargoBottomSource::ORIGIN_HEIGHT) ==
              lidar_slam2_msgs::CargoBottomEstimate::SOURCE_ORIGIN_HEIGHT,
              "Cargo source schema drift: ORIGIN_HEIGHT");

NdtSlamNode::NdtSlamNode(const ros::NodeHandle& nh)
    : nh_(nh) {
    initializeParameters();

    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 10, &NdtSlamNode::pointCloudCallback, this);
    if (hook_load_signal_enabled_) {
        hook_load_state_sub_ = nh_.subscribe(
            hook_load_state_topic_, 1,
            &NdtSlamNode::hookLoadStateCallback, this);
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 1, true);
    display_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map", 1, true);
    ground_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_ground", 1, true);
    objects_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_objects", 1, true);
    objects_clean_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_objects_clean", 1, true);
    near_field_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/near_field_removed", 10);
    payload_channel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_channel_cloud", 10);
    payload_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_candidate_cloud", 10);
    safe_objects_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/safe_objects_cloud", 10);
    payload_dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_dynamic_cloud", 10);
    payload_pending_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_pending_cloud", 10);
    cargo_dynamic_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cargo_dynamic_removed_cloud", 10);
    payload_track_info_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/payload_track_info", 10);
    payload_precise_box_info_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/payload_precise_box_info", 10);
    cargo_selected_core_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cargo_selected_core_points", 10);

    // Cargo Warning publishers
    cargo_warning_pub_ = nh_.advertise<std_msgs::String>("/cargo_warning", 10);
    cargo_warning_text_pub_ = nh_.advertise<std_msgs::String>("/cargo_warning_text", 10);
    cargo_tight_box_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/cargo_tight_box_marker", 10);
    cargo_warning_zone_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_warning_zone_marker", 10);
    cargo_warning_obstacle_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/cargo_warning_obstacle_marker", 10);
    cargo_bottom_estimate_pub_ = nh_.advertise<lidar_slam2_msgs::CargoBottomEstimate>(
        "/cargo_avoidance/bottom_estimate", 1);
    cargo_safety_status_pub_ = nh_.advertise<lidar_slam2_msgs::CargoSafetyStatus>(
        "/cargo_avoidance/safety_status", 1);
    cargo_fused_box_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/cargo_avoidance/fused_box_marker", 2);

    human_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_candidate_cloud", 10);
    human_dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_dynamic_cloud", 10);
    human_pending_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_pending_cloud", 10);
    human_trajectory_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_trajectory_capsule", 10);
    human_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_removed_history_cloud", 10);
    current_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(current_cloud_topic_, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 10);
    runtime_path_pub_ = nh_.advertise<nav_msgs::Path>("/ndt_slam/runtime_path", 1, true);
    relocalization_status_pub_ = nh_.advertise<std_msgs::String>(
        "/ndt_slam/relocalization_status", 1, true);

    // 初始化轨迹
    path_msg_.header.frame_id = "map";
    runtime_path_msg_.header.frame_id = "map";

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    publishInitialTransform();

    reset_srv_ = nh_.advertiseService("reset", &NdtSlamNode::resetService, this);
    set_pose_srv_ = nh_.advertiseService("set_pose", &NdtSlamNode::setPoseService, this);
    relocalize_srv_ = nh_.advertiseService("relocalize", &NdtSlamNode::relocalizeService, this);
    save_map_srv_ = nh_.advertiseService("save_map", &NdtSlamNode::saveMapService, this);
    load_map_srv_ = nh_.advertiseService("load_map", &NdtSlamNode::loadMapService, this);
    rebuild_map_srv_ = nh_.advertiseService("rebuild_map", &NdtSlamNode::rebuildMapService, this);

    current_pose_ = Sophus::SE3d();
    global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    display_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    local_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_cloud_transformed_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    last_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // V3: Localization Target 初始化
    localization_target_front_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    localization_target_back_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    localization_target_snapshot_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cached_target_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // 初始化 NDT_OMP（使用配置参数）
    ndt_.reset(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_->setResolution(ndt_resolution_);
    ndt_->setStepSize(ndt_step_size_);
    ndt_->setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_->setMaximumIterations(ndt_max_iterations_);

    // V3: 设置 NDT_OMP 多线程和邻域搜索方法
    ndt_->setNumThreads(ndt_num_threads_);
    if (ndt_neighbor_search_method_ == "DIRECT7") {
        ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ROS_INFO("NDT_OMP: using DIRECT7 neighbor search");
    } else if (ndt_neighbor_search_method_ == "DIRECT1") {
        ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT1);
        ROS_INFO("NDT_OMP: using DIRECT1 neighbor search");
    } else {
        ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
        ROS_INFO("NDT_OMP: using KDTREE neighbor search");
    }

    ROS_INFO("NDT_OMP initialized: resolution=%.2f, step_size=%.2f, max_iter=%d, threads=%d, search=%s",
             ndt_resolution_, ndt_step_size_, ndt_max_iterations_, ndt_num_threads_,
             ndt_neighbor_search_method_.c_str());

    relocalizer_.configure(relocalization_cfg_);
    relocalizer_.start();
    shutdown_ = false;
    running_ = true;
    process_thread_ = std::thread(&NdtSlamNode::processCloudThread, this);

    timer_ = nh_.createTimer(ros::Duration(5.0), &NdtSlamNode::timerCallback, this);

    ROS_INFO("NdtSlamNode initialized with NDT_OMP");
    ROS_INFO("Services: reset, set_pose, relocalize, save_map, load_map, rebuild_map");
}

NdtSlamNode::NdtSlamNode(const std::string& config_file_path, const ros::NodeHandle& nh)
    : nh_(nh) {
    initializeParameters(config_file_path);

    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 10, &NdtSlamNode::pointCloudCallback, this);
    if (hook_load_signal_enabled_) {
        hook_load_state_sub_ = nh_.subscribe(
            hook_load_state_topic_, 1,
            &NdtSlamNode::hookLoadStateCallback, this);
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 1, true);
    display_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map", 1, true);
    ground_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_ground", 1, true);
    objects_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_objects", 1, true);
    objects_clean_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_objects_clean", 1, true);
    near_field_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/near_field_removed", 10);
    payload_channel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_channel_cloud", 10);
    payload_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_candidate_cloud", 10);
    safe_objects_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/safe_objects_cloud", 10);
    payload_dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_dynamic_cloud", 10);
    payload_pending_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_pending_cloud", 10);
    cargo_dynamic_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cargo_dynamic_removed_cloud", 10);
    payload_track_info_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/payload_track_info", 10);
    payload_precise_box_info_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/payload_precise_box_info", 10);
    cargo_selected_core_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cargo_selected_core_points", 10);

    // Cargo Warning publishers
    cargo_warning_pub_ = nh_.advertise<std_msgs::String>("/cargo_warning", 10);
    cargo_warning_text_pub_ = nh_.advertise<std_msgs::String>("/cargo_warning_text", 10);
    cargo_tight_box_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/cargo_tight_box_marker", 10);
    cargo_warning_zone_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_warning_zone_marker", 10);
    cargo_warning_obstacle_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/cargo_warning_obstacle_marker", 10);
    cargo_bottom_estimate_pub_ = nh_.advertise<lidar_slam2_msgs::CargoBottomEstimate>(
        "/cargo_avoidance/bottom_estimate", 1);
    cargo_safety_status_pub_ = nh_.advertise<lidar_slam2_msgs::CargoSafetyStatus>(
        "/cargo_avoidance/safety_status", 1);
    cargo_fused_box_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/cargo_avoidance/fused_box_marker", 2);

    human_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_candidate_cloud", 10);
    human_dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_dynamic_cloud", 10);
    human_pending_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_pending_cloud", 10);
    human_trajectory_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_trajectory_capsule", 10);
    human_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_removed_history_cloud", 10);
    current_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(current_cloud_topic_, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 10);
    runtime_path_pub_ = nh_.advertise<nav_msgs::Path>("/ndt_slam/runtime_path", 1, true);
    relocalization_status_pub_ = nh_.advertise<std_msgs::String>(
        "/ndt_slam/relocalization_status", 1, true);

    // 初始化轨迹
    path_msg_.header.frame_id = "map";
    runtime_path_msg_.header.frame_id = "map";

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    publishInitialTransform();

    reset_srv_ = nh_.advertiseService("reset", &NdtSlamNode::resetService, this);
    set_pose_srv_ = nh_.advertiseService("set_pose", &NdtSlamNode::setPoseService, this);
    relocalize_srv_ = nh_.advertiseService("relocalize", &NdtSlamNode::relocalizeService, this);
    save_map_srv_ = nh_.advertiseService("save_map", &NdtSlamNode::saveMapService, this);
    load_map_srv_ = nh_.advertiseService("load_map", &NdtSlamNode::loadMapService, this);
    rebuild_map_srv_ = nh_.advertiseService("rebuild_map", &NdtSlamNode::rebuildMapService, this);

    current_pose_ = Sophus::SE3d();
    global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    display_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ground_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    objects_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    objects_clean_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rebuild_objects_filtered_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rebuild_payload_candidate_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rebuild_payload_dynamic_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rebuild_human_candidate_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rebuild_human_dynamic_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rebuild_human_pending_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rebuild_ground_raw_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    local_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_cloud_transformed_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    last_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // V3: Localization Target 初始化
    localization_target_front_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    localization_target_back_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    localization_target_snapshot_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cached_target_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // 初始化 NDT_OMP（使用从配置文件加载的参数，而非硬编码）
    ndt_.reset(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_->setResolution(ndt_resolution_);
    ndt_->setStepSize(ndt_step_size_);
    ndt_->setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_->setMaximumIterations(ndt_max_iterations_);

    // V3: 设置 NDT_OMP 多线程和邻域搜索方法
    ndt_->setNumThreads(ndt_num_threads_);
    if (ndt_neighbor_search_method_ == "DIRECT7") {
        ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    } else if (ndt_neighbor_search_method_ == "DIRECT1") {
        ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    } else {
        ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }

    ROS_INFO("NDT_OMP initialized: resolution=%.2f, step_size=%.2f, max_iter=%d, threads=%d, search=%s",
             ndt_resolution_, ndt_step_size_, ndt_max_iterations_, ndt_num_threads_,
             ndt_neighbor_search_method_.c_str());

    // 初始化 PayloadTrackManager
    payload_tracker_.configureFromYaml(config_file_path);
    payload_tracker_config_ = payload_tracker_.getConfig();
    ROS_INFO("[PayloadTracker] initialized: enabled=%d, base_stability_std_thresh=%.2f",
             payload_tracker_config_.enabled ? 1 : 0, payload_tracker_config_.base_stability_std_thresh);

    // P0.5: 初始化 CargoBoxEstimator
    cargo_box_estimator_.configureFromYaml(config_file_path);
    cargo_box_estimator_config_ = cargo_box_estimator_.getConfig();
    ROS_INFO("[CargoBoxEstimator] initialized: enabled=%d, use_crane_axis_obb=%d",
             cargo_box_estimator_config_.enabled ? 1 : 0, cargo_box_estimator_config_.use_crane_axis_obb);

    // Runtime diagnostics: configure and log startup
    if (runtime_diag_config_.enabled) {
        runtime_diag_.configure(runtime_diag_config_, diag_output_dir_);
        logStartupConfig();
        logBuildId();
    }

    // Start callbacks only after all publishers, maps, algorithms and
    // diagnostics are fully configured.
    relocalizer_.configure(relocalization_cfg_);
    relocalizer_.start();
    shutdown_ = false;
    running_ = true;
    process_thread_ = std::thread(&NdtSlamNode::processCloudThread, this);

    timer_ = nh_.createTimer(ros::Duration(5.0), &NdtSlamNode::timerCallback, this);

    ROS_INFO("NdtSlamNode initialized with NDT_OMP");
    ROS_INFO("Config file: %s", config_file_path.c_str());
    ROS_INFO("Services: reset, set_pose, relocalize, save_map, load_map, rebuild_map");
}

NdtSlamNode::~NdtSlamNode() {
    pointcloud_sub_.shutdown();
    hook_load_state_sub_.shutdown();
    timer_.stop();
    shutdown_ = true;
    queue_cv_.notify_all();
    tracking_cv_.notify_all();

    if (process_thread_.joinable()) {
        process_thread_.join();
    }
    relocalizer_.stop();
    if (icp_thread_.joinable()) {
        icp_thread_.join();
    }
    if (rebuild_thread_.joinable()) {
        rebuild_thread_.join();
    }
    if (clean_rebuild_thread_.joinable()) {
        clean_rebuild_thread_.join();
    }
    running_ = false;

    ROS_WARN("[Shutdown] Final flush dirty tiles...");
    if (persistent_map_enabled_ && !dirty_tiles_.empty()) {
        flushDirtyTiles();
    }
    writeRuntimeStatus();
    if (diag_pending_ndt_record_valid_) {
        runtime_diag_.writeNdtFrame(diag_pending_ndt_record_);
        diag_pending_ndt_record_valid_ = false;
    }
    runtime_diag_.flushCsv();
    ROS_WARN("[Shutdown] Complete");
}

void NdtSlamNode::timerCallback(const ros::TimerEvent&) {
    static int timer_count = 0;
    timer_count++;

    const auto& keyframes = loop_closure_detector_.getKeyFrames();
    ROS_DEBUG("[Timer] keyframes=%zu, cloud=%zu, init=%d",
             keyframes.size(), current_cloud_->size(), initialized_ ? 1 : 0);

    // 不在这里发布 TF，避免与 publishOdometry 冲突导致 TF_REPEATED_DATA
    // TF 由 publishOdometry 在每帧处理后统一发布
}

void NdtSlamNode::initializeParameters(const std::string& config_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_file_path);

        // 调试配置
        if (config["debug"]) {
            auto dbg = config["debug"];
            debug_cfg_.publish_runtime_path = dbg["publish_runtime_path"].as<bool>(false);
            ROS_INFO("[DebugConfig] publish_runtime_path=%d", debug_cfg_.publish_runtime_path ? 1 : 0);
        }

        // 日志配置
        if (config["logging"]) {
            auto log = config["logging"];
            debug_cfg_.summary_interval_sec = log["summary_interval_sec"].as<double>(2.0);
            debug_cfg_.warn_throttle_sec = log["warn_throttle_sec"].as<double>(2.0);
            debug_cfg_.debug_config = log["debug_config"].as<bool>(false);
            debug_cfg_.debug_frame_start = log["debug_frame_start"].as<bool>(false);
            debug_cfg_.debug_ndt_health = log["debug_ndt_health"].as<bool>(false);
            debug_cfg_.debug_ekf = log["debug_ekf"].as<bool>(false);
            debug_cfg_.debug_motion_gate = log["debug_motion_gate"].as<bool>(false);
            debug_cfg_.debug_pose_flow = log["debug_pose_flow"].as<bool>(false);
            debug_cfg_.debug_map_commit = log["debug_map_commit"].as<bool>(false);
            debug_cfg_.debug_perf = log["debug_perf"].as<bool>(true);
            debug_cfg_.debug_cargo = log["debug_cargo"].as<bool>(false);
            debug_cfg_.debug_cargo_bottom = log["debug_cargo_bottom"].as<bool>(false);
            debug_cfg_.debug_cargo_warning = log["debug_cargo_warning"].as<bool>(false);
            debug_cfg_.debug_tight_box = log["debug_tight_box"].as<bool>(false);
            debug_cfg_.debug_dynamic_filter = log["debug_dynamic_filter"].as<bool>(false);
            debug_cfg_.debug_odom_anchor = log["debug_odom_anchor"].as<bool>(false);
            debug_cfg_.debug_registration_removal = log["debug_registration_removal"].as<bool>(false);
            debug_cfg_.debug_hook_removal = log["debug_hook_removal"].as<bool>(false);
        }

        if (config["pointcloud_topic"]) {
            pointcloud_topic_ = config["pointcloud_topic"].as<std::string>();
        }
        if (config["odom_topic"]) {
            odom_topic_ = config["odom_topic"].as<std::string>();
        }
        if (config["pose_topic"]) {
            pose_topic_ = config["pose_topic"].as<std::string>();
        }
        if (config["map_topic"]) {
            map_topic_ = config["map_topic"].as<std::string>();
        }
        if (config["current_cloud_topic"]) {
            current_cloud_topic_ = config["current_cloud_topic"].as<std::string>();
        }
        if (config["processing_queue"]) {
            const int configured_capacity =
                config["processing_queue"]["capacity"].as<int>(1);
            const int bounded_capacity = std::clamp(configured_capacity, 1, 4);
            localization_queue_capacity_ =
                static_cast<std::size_t>(bounded_capacity);
            if (configured_capacity != bounded_capacity) {
                ROS_WARN("[ProcessingQueue] capacity=%d out of range; clamped to %d",
                         configured_capacity, bounded_capacity);
            }
        }

        if (config["max_range"]) {
            kiss_icp_config_.max_range = config["max_range"].as<double>();
        } else if (config["data"] && config["data"]["max_range"]) {
            kiss_icp_config_.max_range = config["data"]["max_range"].as<double>();
        }
        if (config["min_range"]) {
            kiss_icp_config_.min_range = config["min_range"].as<double>();
        } else if (config["data"] && config["data"]["min_range"]) {
            kiss_icp_config_.min_range = config["data"]["min_range"].as<double>();
        }
        if (config["voxel_size"]) {
            kiss_icp_config_.voxel_size = config["voxel_size"].as<double>();
        } else if (config["mapping"] && config["mapping"]["voxel_size"]) {
            kiss_icp_config_.voxel_size = config["mapping"]["voxel_size"].as<double>();
        }

        if (config["deskew"]) {
            kiss_icp_config_.deskew = config["deskew"].as<bool>();
        } else if (config["data"] && config["data"]["deskew"]) {
            kiss_icp_config_.deskew = config["data"]["deskew"].as<bool>();
        }

        if (config["max_num_iterations"]) {
            kiss_icp_config_.max_num_iterations = config["max_num_iterations"].as<int>();
        } else if (config["registration"] && config["registration"]["max_num_iterations"]) {
            kiss_icp_config_.max_num_iterations = config["registration"]["max_num_iterations"].as<int>();
        }
        if (config["convergence_criterion"]) {
            kiss_icp_config_.convergence_criterion = config["convergence_criterion"].as<double>();
        } else if (config["registration"] && config["registration"]["convergence_criterion"]) {
            kiss_icp_config_.convergence_criterion = config["registration"]["convergence_criterion"].as<double>();
        }
        if (config["initial_threshold"]) {
            kiss_icp_config_.initial_threshold = config["initial_threshold"].as<double>();
        } else if (config["adaptive_threshold"] && config["adaptive_threshold"]["initial_threshold"]) {
            kiss_icp_config_.initial_threshold = config["adaptive_threshold"]["initial_threshold"].as<double>();
        }
        if (config["min_motion_th"]) {
            kiss_icp_config_.min_motion_th = config["min_motion_th"].as<double>();
        } else if (config["adaptive_threshold"] && config["adaptive_threshold"]["min_motion_th"]) {
            kiss_icp_config_.min_motion_th = config["adaptive_threshold"]["min_motion_th"].as<double>();
        }
        if (config["max_points_per_voxel"]) {
            kiss_icp_config_.max_points_per_voxel = config["max_points_per_voxel"].as<int>();
        } else if (config["mapping"] && config["mapping"]["max_points_per_voxel"]) {
            kiss_icp_config_.max_points_per_voxel = config["mapping"]["max_points_per_voxel"].as<int>();
        }

        if (config["odom_frame"]) {
            odom_frame_ = config["odom_frame"].as<std::string>();
        }
        if (config["base_frame"]) {
            base_frame_ = config["base_frame"].as<std::string>();
        }
        if (config["lidar_odom_frame"]) {
            lidar_odom_frame_ = config["lidar_odom_frame"].as<std::string>();
        }
        if (config["map_frame"]) {
            map_frame_ = config["map_frame"].as<std::string>();
        }

        if (config["map_voxel_size"]) {
            voxel_size_ = config["map_voxel_size"].as<double>();
        }
        if (config["display_voxel_size"]) {
            display_voxel_size_ = config["display_voxel_size"].as<double>();
        }
        if (config["ground_voxel_size"]) {
            ground_voxel_size_ = config["ground_voxel_size"].as<double>();
        }
        if (config["objects_voxel_size"]) {
            objects_voxel_size_ = config["objects_voxel_size"].as<double>();
        }
        if (config["grid_cell_size"]) {
            grid_cell_size_ = config["grid_cell_size"].as<double>();
        }
        if (config["height_above_ground"]) {
            height_above_ground_ = config["height_above_ground"].as<double>();
        }
        if (config["near_field_radius"]) {
            near_field_radius_ = config["near_field_radius"].as<double>();
        }
        if (config["near_field_z_min"]) {
            near_field_z_min_ = config["near_field_z_min"].as<double>();
        }
        if (config["max_map_size"]) {
            max_map_size_ = config["max_map_size"].as<double>();
        }
        if (config["use_voxel_filter"]) {
            use_voxel_filter_ = config["use_voxel_filter"].as<bool>();
        }
        if (config["map_update_interval"]) {
            map_update_interval_ = config["map_update_interval"].as<int>();
        }

        if (config["loop_detection_interval"]) {
            loop_detection_interval_ = config["loop_detection_interval"].as<int>();
        }

        // 特征提取参数
        if (config["feature_extraction"]) {
            auto fe = config["feature_extraction"];
            if (fe["enabled"]) use_feature_extraction_ = fe["enabled"].as<bool>();
            if (fe["voxel_size"]) feature_voxel_size_ = fe["voxel_size"].as<double>();
            if (fe["height_diff_threshold"]) height_diff_threshold_ = fe["height_diff_threshold"].as<double>();
            if (fe["feature_weight"]) feature_weight_ = fe["feature_weight"].as<int>();
        }

        // NDT_OMP 配置参数
        if (config["ndt_omp"]) {
            auto ndt = config["ndt_omp"];
            if (ndt["resolution"]) ndt_resolution_ = ndt["resolution"].as<double>();
            if (ndt["step_size"]) ndt_step_size_ = ndt["step_size"].as<double>();
            if (ndt["transformation_epsilon"]) ndt_transformation_epsilon_ = ndt["transformation_epsilon"].as<double>();
            if (ndt["max_iterations"]) ndt_max_iterations_ = ndt["max_iterations"].as<int>();
            if (ndt["num_threads"]) ndt_num_threads_ = ndt["num_threads"].as<int>();
            if (ndt["neighbor_search_method"]) ndt_neighbor_search_method_ = ndt["neighbor_search_method"].as<std::string>();
        }

        // 动态点过滤参数
        if (config["dynamic_filter"]) {
            auto df = config["dynamic_filter"];
            if (df["enabled"]) use_dynamic_filter_ = df["enabled"].as<bool>();
            if (df["mean_k"]) sor_mean_k_ = df["mean_k"].as<int>();
            if (df["stddev_mul_thresh"]) sor_stddev_mul_thresh_ = df["stddev_mul_thresh"].as<double>();
        }

        // BasePayloadChannelFilter 配置
        channel_filter_.configureFromYaml(config_file_path);
        channel_filter_config_ = channel_filter_.getConfig();

        // HumanObjectDynamicFilter 配置
        if (config["human_object_filter"]) {
            auto hof = config["human_object_filter"];
            if (hof["enabled"]) human_filter_config_.enabled = hof["enabled"].as<bool>();
            if (hof["min_hag"]) human_filter_config_.min_hag = hof["min_hag"].as<double>();
            if (hof["max_hag"]) human_filter_config_.max_hag = hof["max_hag"].as<double>();
            if (hof["min_cluster_height"]) human_filter_config_.min_cluster_height = hof["min_cluster_height"].as<double>();
            if (hof["max_cluster_height"]) human_filter_config_.max_cluster_height = hof["max_cluster_height"].as<double>();
            if (hof["min_points"]) human_filter_config_.min_points = hof["min_points"].as<int>();
            if (hof["max_points"]) human_filter_config_.max_points = hof["max_points"].as<int>();
            if (hof["min_area_m2"]) human_filter_config_.min_area_m2 = hof["min_area_m2"].as<double>();
            if (hof["max_area_m2"]) human_filter_config_.max_area_m2 = hof["max_area_m2"].as<double>();
            if (hof["max_width_m"]) human_filter_config_.max_width_m = hof["max_width_m"].as<double>();
            if (hof["max_length_m"]) human_filter_config_.max_length_m = hof["max_length_m"].as<double>();
            if (hof["bev_resolution"]) human_filter_config_.bev_resolution = hof["bev_resolution"].as<double>();
            if (hof["merge_gap_m"]) human_filter_config_.merge_gap_m = hof["merge_gap_m"].as<double>();
            // P1: 区分 strong/weak 的阈值
            if (hof["min_points_strong"]) human_filter_config_.min_points_strong = hof["min_points_strong"].as<int>();
            if (hof["min_points_weak"]) human_filter_config_.min_points_weak = hof["min_points_weak"].as<int>();
        }

        if (config["human_object_tracking"]) {
            auto hot = config["human_object_tracking"];
            if (hot["enabled"]) human_tracking_config_.enabled = hot["enabled"].as<bool>();
            if (hot["window_sec"]) human_tracking_config_.window_sec = hot["window_sec"].as<double>();
            if (hot["confirm_frames"]) human_tracking_config_.confirm_frames = hot["confirm_frames"].as<int>();
            if (hot["map_displacement_thresh_m"]) human_tracking_config_.map_displacement_thresh_m = hot["map_displacement_thresh_m"].as<double>();
            if (hot["velocity_thresh_mps"]) human_tracking_config_.velocity_thresh_mps = hot["velocity_thresh_mps"].as<double>();
            if (hot["max_match_distance_m"]) human_tracking_config_.max_match_distance_m = hot["max_match_distance_m"].as<double>();
            if (hot["max_missed_frames"]) human_tracking_config_.max_missed_frames = hot["max_missed_frames"].as<int>();
        }

        if (config["human_object_eraser"]) {
            auto hoe = config["human_object_eraser"];
            if (hoe["enabled"]) human_eraser_config_.enabled = hoe["enabled"].as<bool>();
            if (hoe["history_sec"]) human_eraser_config_.history_sec = hoe["history_sec"].as<double>();
            if (hoe["capsule_radius_m"]) human_eraser_config_.capsule_radius_m = hoe["capsule_radius_m"].as<double>();
            if (hoe["use_track_height_range"]) human_eraser_config_.use_track_height_range = hoe["use_track_height_range"].as<bool>();
            if (hoe["z_margin_m"]) human_eraser_config_.z_margin_m = hoe["z_margin_m"].as<double>();
            if (hoe["hag_margin_m"]) human_eraser_config_.hag_margin_m = hoe["hag_margin_m"].as<double>();
            if (hoe["pre_guard_sec"]) human_eraser_config_.pre_guard_sec = hoe["pre_guard_sec"].as<double>();
            if (hoe["post_guard_sec"]) human_eraser_config_.post_guard_sec = hoe["post_guard_sec"].as<double>();
            if (hoe["erase_objects_only"]) human_eraser_config_.erase_objects_only = hoe["erase_objects_only"].as<bool>();
            if (hoe["erase_ground"]) human_eraser_config_.erase_ground = hoe["erase_ground"].as<bool>();
            if (hoe["async_update"]) human_eraser_config_.async_update = hoe["async_update"].as<bool>();
        }

        // 初始化人体过滤模块
        human_filter_.initialize(human_filter_config_, human_tracking_config_, human_eraser_config_);

        // DynamicEventManager 配置
        if (config["dynamic_event_manager"]) {
            auto dem = config["dynamic_event_manager"];
            if (dem["enabled"]) dynamic_event_config_.enabled = dem["enabled"].as<bool>();
            if (dem["payload_min_candidate_frames"]) dynamic_event_config_.payload_min_candidate_frames = dem["payload_min_candidate_frames"].as<int>();
            if (dem["payload_pre_guard_sec"]) dynamic_event_config_.payload_pre_guard_sec = dem["payload_pre_guard_sec"].as<double>();
            if (dem["moving_post_guard_sec"]) dynamic_event_config_.moving_post_guard_sec = dem["moving_post_guard_sec"].as<double>();
            if (dem["unknown_post_guard_sec"]) dynamic_event_config_.unknown_post_guard_sec = dem["unknown_post_guard_sec"].as<double>();
            if (dem["merge_same_track"]) dynamic_event_config_.merge_same_track = dem["merge_same_track"].as<bool>();
            if (dem["merge_time_gap_sec"]) dynamic_event_config_.merge_time_gap_sec = dem["merge_time_gap_sec"].as<double>();
            if (dem["merge_iou_thresh"]) dynamic_event_config_.merge_iou_thresh = dem["merge_iou_thresh"].as<double>();
            if (dem["max_active_sessions"]) dynamic_event_config_.max_active_sessions = dem["max_active_sessions"].as<int>();
            if (dem["placement_detection_enabled"]) dynamic_event_config_.placement_detection_enabled = dem["placement_detection_enabled"].as<bool>();
            if (dem["stable_window_sec"]) dynamic_event_config_.stable_window_sec = dem["stable_window_sec"].as<double>();
            if (dem["stable_frames_thresh"]) dynamic_event_config_.stable_frames_thresh = dem["stable_frames_thresh"].as<int>();
            if (dem["stable_map_disp_thresh_m"]) dynamic_event_config_.stable_map_disp_thresh_m = dem["stable_map_disp_thresh_m"].as<double>();
            if (dem["stable_velocity_thresh_mps"]) dynamic_event_config_.stable_velocity_thresh_mps = dem["stable_velocity_thresh_mps"].as<double>();
            if (dem["placed_bbox_expand_xy"]) dynamic_event_config_.placed_bbox_expand_xy = dem["placed_bbox_expand_xy"].as<double>();
            if (dem["placed_bbox_expand_z"]) dynamic_event_config_.placed_bbox_expand_z = dem["placed_bbox_expand_z"].as<double>();
            if (dem["human_pre_guard_sec"]) dynamic_event_config_.human_pre_guard_sec = dem["human_pre_guard_sec"].as<double>();
            if (dem["human_post_guard_sec"]) dynamic_event_config_.human_post_guard_sec = dem["human_post_guard_sec"].as<double>();
            if (dem["human_capsule_radius"]) dynamic_event_config_.human_capsule_radius = dem["human_capsule_radius"].as<double>();
            if (dem["human_use_track_height"]) dynamic_event_config_.human_use_track_height = dem["human_use_track_height"].as<bool>();
            if (dem["human_z_margin"]) dynamic_event_config_.human_z_margin = dem["human_z_margin"].as<double>();
            if (dem["clean_deny_enabled"]) dynamic_event_config_.clean_deny_enabled = dem["clean_deny_enabled"].as<bool>();
            if (dem["max_dynamic_ratio"]) dynamic_event_config_.max_dynamic_ratio = dem["max_dynamic_ratio"].as<double>();
            if (dem["placed_to_objects_clean"]) dynamic_event_config_.placed_to_objects_clean = dem["placed_to_objects_clean"].as<bool>();
            if (dem["placed_to_display_map"]) dynamic_event_config_.placed_to_display_map = dem["placed_to_display_map"].as<bool>();
            if (dem["placed_to_registration_map"]) dynamic_event_config_.placed_to_registration_map = dem["placed_to_registration_map"].as<bool>();
        }
        dynamic_event_manager_.configure(dynamic_event_config_);

        ROS_INFO("=== DynamicEventManager Config ===");
        ROS_INFO("  enabled: %s", dynamic_event_config_.enabled ? "true" : "false");
        ROS_INFO("  payload_pre_guard: %.1fs, moving_post_guard: %.1fs, unknown_post_guard: %.1fs",
                 dynamic_event_config_.payload_pre_guard_sec,
                 dynamic_event_config_.moving_post_guard_sec,
                 dynamic_event_config_.unknown_post_guard_sec);
        ROS_INFO("  placement: enabled=%s, stable_frames=%d, disp_thresh=%.2f, vel_thresh=%.2f",
                 dynamic_event_config_.placement_detection_enabled ? "true" : "false",
                 dynamic_event_config_.stable_frames_thresh,
                 dynamic_event_config_.stable_map_disp_thresh_m,
                 dynamic_event_config_.stable_velocity_thresh_mps);
        ROS_INFO("  human_pre_guard: %.1fs, post_guard: %.1fs",
                 dynamic_event_config_.human_pre_guard_sec, dynamic_event_config_.human_post_guard_sec);

        loop_closure_detector_.configureFromYaml(config_file_path);

        ROS_INFO("=== NdtSlamNode Parameters ===");
        ROS_INFO("=== HumanObjectFilter Config ===");
        ROS_INFO("  enabled: %s", human_filter_config_.enabled ? "true" : "false");
        ROS_INFO("  min_hag: %.2f m", human_filter_config_.min_hag);
        ROS_INFO("  max_hag: %.2f m", human_filter_config_.max_hag);
        ROS_INFO("  min_cluster_height: %.2f m", human_filter_config_.min_cluster_height);
        ROS_INFO("  max_cluster_height: %.2f m", human_filter_config_.max_cluster_height);
        ROS_INFO("  min_points: %d", human_filter_config_.min_points);
        ROS_INFO("  max_points: %d", human_filter_config_.max_points);
        ROS_INFO("  min_area_m2: %.2f", human_filter_config_.min_area_m2);
        ROS_INFO("  max_area_m2: %.2f", human_filter_config_.max_area_m2);
        ROS_INFO("  max_width_m: %.2f", human_filter_config_.max_width_m);
        ROS_INFO("  max_length_m: %.2f", human_filter_config_.max_length_m);
        ROS_INFO("  bev_resolution: %.2f m", human_filter_config_.bev_resolution);
        ROS_INFO("  merge_gap_m: %.2f m", human_filter_config_.merge_gap_m);
        ROS_INFO("PointCloud topic: %s", pointcloud_topic_.c_str());
        ROS_INFO("Odometry topic: %s", odom_topic_.c_str());
        ROS_INFO("Map topic: %s", map_topic_.c_str());
        ROS_INFO("Base frame: %s", base_frame_.c_str());
        ROS_INFO("Odom frame: %s", odom_frame_.c_str());
        ROS_INFO("Map frame: %s", map_frame_.c_str());
        ROS_INFO("Publish TF: %d", publish_odom_tf_);
        ROS_INFO("Voxel size: %.3f m (registration), %.3f m (display)", voxel_size_, display_voxel_size_);
        ROS_INFO("Max map size: %.1f m", max_map_size_);
        ROS_INFO("Map update interval: %d frames", map_update_interval_);
        ROS_INFO("Use voxel filter: %d", use_voxel_filter_);
        ROS_INFO("Loop detection interval: %d keyframes", loop_detection_interval_);
        ROS_INFO("=== Feature Extraction Config ===");
        ROS_INFO("  enabled: %s", use_feature_extraction_ ? "true" : "false");
        ROS_INFO("  voxel_size: %.3f m", feature_voxel_size_);
        ROS_INFO("  height_diff_threshold: %.3f m", height_diff_threshold_);
        ROS_INFO("  feature_weight: %d", feature_weight_);
        ROS_INFO("=== NDT_OMP Config ===");
        ROS_INFO("  resolution: %.2f m", ndt_resolution_);
        ROS_INFO("  step_size: %.2f", ndt_step_size_);
        ROS_INFO("  transformation_epsilon: %.4f", ndt_transformation_epsilon_);
        ROS_INFO("  max_iterations: %d", ndt_max_iterations_);
        ROS_INFO("=== Dynamic Filter Config ===");
        ROS_INFO("  enabled: %s", use_dynamic_filter_ ? "true" : "false");
        ROS_INFO("  mean_k: %d", sor_mean_k_);
        ROS_INFO("  stddev_mul_thresh: %.2f", sor_stddev_mul_thresh_);
        ROS_INFO("=== Ground Model Config ===");
        ROS_INFO("  grid_cell_size: %.1f m", grid_cell_size_);
        ROS_INFO("  height_above_ground: %.2f m", height_above_ground_);
        ROS_INFO("=== Near-Field Filter Config ===");
        ROS_INFO("  near_field_radius: %.1f m", near_field_radius_);
        ROS_INFO("  near_field_z_min: %.1f m", near_field_z_min_);

        // ========== 长期建图参数 ==========
        if (config["longterm_mapping"]) {
            auto ltm = config["longterm_mapping"];
            longterm_mapping_enabled_ = ltm["enabled"].as<bool>(false);
        }

        if (config["motion_gate"]) {
            auto mg = config["motion_gate"];
            motion_gate_enabled_ = mg["enabled"].as<bool>(false);
            motion_gate_min_translation_m_ = mg["min_translation_m"].as<double>(0.30);
            motion_gate_min_rotation_deg_ = mg["min_rotation_deg"].as<double>(3.0);
            motion_gate_min_time_sec_ = mg["min_time_between_keyframes_sec"].as<double>(2.0);
        }

        if (config["online_cache"]) {
            auto oc = config["online_cache"];
            max_active_keyframes_ = oc["max_active_keyframes"].as<int>(80);
            keyframe_release_interval_ = oc["release_check_interval"].as<int>(10);
        }

        if (config["persistent_map"]) {
            auto pm = config["persistent_map"];
            persistent_map_enabled_ = pm["enabled"].as<bool>(false);
            persistent_map_root_dir_ = pm["root_dir"].as<std::string>("/home/ydkj/NDT-slam-ws/maps/live/current");
            tile_size_m_ = pm["tile_size_m"].as<double>(20.0);
            flush_interval_sec_ = pm["flush_interval_sec"].as<int>(60);
            max_dirty_tiles_ = pm["max_dirty_tiles_in_memory"].as<int>(20);
            tile_voxel_registration_ = pm["tile_voxel_registration"].as<double>(0.30);
            tile_voxel_display_ = pm["tile_voxel_display"].as<double>(0.10);
            tile_voxel_ground_ = pm["tile_voxel_ground"].as<double>(0.15);
            tile_voxel_objects_ = pm["tile_voxel_objects"].as<double>(0.08);
        }

        ROS_INFO("=== Long-Term Mapping Config ===");
        ROS_INFO("  longterm_mapping: %s", longterm_mapping_enabled_ ? "true" : "false");
        ROS_INFO("  motion_gate: %s", motion_gate_enabled_ ? "true" : "false");
        ROS_INFO("  motion_gate_min_translation: %.2f m", motion_gate_min_translation_m_);
        ROS_INFO("  motion_gate_min_rotation: %.1f deg", motion_gate_min_rotation_deg_);
        ROS_INFO("  motion_gate_min_time: %.1f sec", motion_gate_min_time_sec_);
        ROS_INFO("  max_active_keyframes: %d", max_active_keyframes_);
        ROS_INFO("  persistent_map: %s", persistent_map_enabled_ ? "true" : "false");
        ROS_INFO("  persistent_map_dir: %s", persistent_map_root_dir_.c_str());

        if (config["memory_guard"]) {
            auto mg = config["memory_guard"];
            memory_guard_enabled_ = mg["enabled"].as<bool>(false);
            soft_threshold_mb_ = mg["soft_threshold_mb"].as<int>(6000);
            hard_threshold_mb_ = mg["hard_threshold_mb"].as<int>(7000);
            emergency_threshold_mb_ = mg["emergency_threshold_mb"].as<int>(8000);
            memory_check_interval_sec_ = mg["check_interval_sec"].as<int>(30);
        }

        // Crane Motion Constraint 配置
        if (config["crane_motion_constraint"]) {
            auto cmc = config["crane_motion_constraint"];
            crane_constraint_enabled_ = cmc["enabled"].as<bool>(false);
            lock_z_ = cmc["lock_z"].as<bool>(true);
            fixed_z_source_ = cmc["fixed_z_source"].as<std::string>("config");
            fixed_z_value_ = cmc["fixed_z"].as<double>(0.0);
            constrain_z_ = cmc["constrain_z"].as<bool>(false);
            lock_roll_ = cmc["lock_roll"].as<bool>(true);
            lock_pitch_ = cmc["lock_pitch"].as<bool>(true);
            lock_yaw_ = cmc["lock_yaw"].as<bool>(false);
            constrain_yaw_ = cmc["constrain_yaw"].as<bool>(false);
            max_abs_z_drift_ = cmc["max_abs_z_drift"].as<double>(0.05);
            max_roll_deg_ = cmc["max_roll_deg"].as<double>(0.3);
            max_pitch_deg_ = cmc["max_pitch_deg"].as<double>(0.3);
            max_yaw_deg_ = cmc["max_yaw_deg"].as<double>(10.0);
        }

        // 如果 fixed_z_source=config，直接设置 fixed_z
        if (fixed_z_source_ == "config") {
            fixed_z_ = fixed_z_value_;
            first_pose_initialized_ = true;  // 不需要从第一帧初始化
            ROS_INFO("[CraneConstraint] fixed_z_source=config, fixed_z=%.3f", fixed_z_);
        }

        ROS_INFO("=== Crane Motion Constraint ===");
        ROS_INFO("  enabled: %s", crane_constraint_enabled_ ? "true" : "false");
        ROS_INFO("  lock_z: %s, fixed_z_source: %s, fixed_z: %.3f",
                 lock_z_ ? "true" : "false", fixed_z_source_.c_str(), fixed_z_);
        ROS_INFO("  lock_roll: %s, lock_pitch: %s", lock_roll_ ? "true" : "false", lock_pitch_ ? "true" : "false");
        ROS_INFO("  lock_yaw: %s, constrain_yaw: %s", lock_yaw_ ? "true" : "false", constrain_yaw_ ? "true" : "false");

        ROS_INFO("=== Memory Guard Config ===");
        ROS_INFO("  enabled: %s", memory_guard_enabled_ ? "true" : "false");
        ROS_INFO("  soft_threshold: %d MB", soft_threshold_mb_);
        ROS_INFO("  hard_threshold: %d MB", hard_threshold_mb_);
        ROS_INFO("  emergency_threshold: %d MB", emergency_threshold_mb_);
        ROS_INFO("  check_interval: %d sec", memory_check_interval_sec_);

        // commit_enabled 配置（observe_only 模式）
        if (config["longterm_mapping"]) {
            auto ltm = config["longterm_mapping"];
            commit_enabled_ = ltm["commit_enabled"].as<bool>(true);
        }
        ROS_INFO("  commit_enabled: %s", commit_enabled_ ? "true" : "false");

        if (config["disk_guard"]) {
            auto dg = config["disk_guard"];
            disk_guard_enabled_ = dg["enabled"].as<bool>(false);
            min_free_disk_gb_ = dg["min_free_disk_gb"].as<double>(30.0);
            pause_mapping_when_disk_low_ = dg["pause_mapping_when_low"].as<bool>(true);
        }

        ROS_INFO("=== Disk Guard Config ===");
        ROS_INFO("  enabled: %s", disk_guard_enabled_ ? "true" : "false");
        ROS_INFO("  min_free_disk: %.1f GB", min_free_disk_gb_);

        if (config["pointcloud_watchdog"]) {
            auto pw = config["pointcloud_watchdog"];
            pointcloud_stale_timeout_sec_ = pw["stale_timeout_sec"].as<double>(10.0);
        }

        ROS_INFO("=== Pointcloud Watchdog Config ===");
        ROS_INFO("  stale_timeout: %.1f sec", pointcloud_stale_timeout_sec_);

        if (config["ndt_health"]) {
            auto nh = config["ndt_health"];
            fitness_warning_threshold_ = nh["fitness_warning_threshold"].as<double>(2.0);
            fitness_warning_count_ = nh["fitness_warning_count"].as<int>(50);
        }

        if (config["active_map"]) {
            auto am = config["active_map"];
            rebuild_every_keyframes_ = am["rebuild_every_keyframes"].as<int>(10);
        }

        ROS_INFO("=== NDT Health Config ===");
        ROS_INFO("  fitness_warning_threshold: %.2f", fitness_warning_threshold_);
        ROS_INFO("  rebuild_every_keyframes: %d", rebuild_every_keyframes_);
        ROS_INFO("===========================");

        // v8-stable-r3: CraneMotionEKF 参数
        if (config["crane_motion_ekf"]) {
            const auto n = config["crane_motion_ekf"];
            crane_motion_ekf_enabled_ = n["enabled"].as<bool>(true);

            crane_motion_ekf_cfg_.q_pos = n["q_pos"].as<double>(0.05);
            crane_motion_ekf_cfg_.q_vel = n["q_vel"].as<double>(0.30);

            crane_motion_ekf_cfg_.r_ndt_base = n["r_ndt_base"].as<double>(0.02);
            crane_motion_ekf_cfg_.r_ndt_max = n["r_ndt_max"].as<double>(2.0);
            crane_motion_ekf_cfg_.fitness_to_r_scale = n["fitness_to_r_scale"].as<double>(5.0);

            crane_motion_ekf_cfg_.innovation_gate_m = n["innovation_gate_m"].as<double>(0.35);
            crane_motion_ekf_cfg_.innovation_reject_m = n["innovation_reject_m"].as<double>(1.00);

            // 高 fitness 拒绝
            crane_motion_ekf_cfg_.reject_high_fitness = n["reject_high_fitness"].as<bool>(true);
            crane_motion_ekf_cfg_.ndt_fitness_reject_threshold = n["ndt_fitness_reject_threshold"].as<double>(0.30);
            crane_motion_ekf_cfg_.ndt_fitness_recover_threshold = n["ndt_fitness_recover_threshold"].as<double>(0.12);

            crane_motion_ekf_cfg_.max_speed_x = n["max_speed_x"].as<double>(2.0);
            crane_motion_ekf_cfg_.max_speed_y = n["max_speed_y"].as<double>(2.0);
            crane_motion_ekf_cfg_.max_accel_x = n["max_accel_x"].as<double>(1.0);
            crane_motion_ekf_cfg_.max_accel_y = n["max_accel_y"].as<double>(1.0);
            map_commit_requires_ndt_accept_ =
                n["commit_requires_ndt_accept"].as<bool>(true);
            map_commit_max_fitness_ =
                n["map_commit_max_fitness"].as<double>(2.0);
            if (!map_commit_requires_ndt_accept_) {
                ROS_WARN("[MapCommit] commit_requires_ndt_accept=false is unsafe and will be ignored");
            }

            // V3: 慢帧保护配置
            if (n["ndt_runtime_guard"]) {
                const auto g = n["ndt_runtime_guard"];
                crane_motion_ekf_cfg_.slow_frame_guard_enabled = g["enabled"].as<bool>(true);
                crane_motion_ekf_cfg_.slow_frame_warn_ms = g["warn_ms"].as<double>(100.0);
                crane_motion_ekf_cfg_.slow_frame_emergency_ms = g["emergency_ms"].as<double>(120.0);
                crane_motion_ekf_cfg_.slow_frame_extra_r = g["slow_extra_r"].as<double>(0.30);
            }

            // V3: 物理步长保护配置
            if (n["crane_motion_limit"]) {
                const auto m = n["crane_motion_limit"];
                crane_motion_ekf_cfg_.max_speed_mps = m["max_speed_mps"].as<double>(0.50);
                crane_motion_ekf_cfg_.max_step_safety_factor = m["max_step_safety_factor"].as<double>(1.5);
                crane_motion_ekf_cfg_.max_step_min_m = m["max_step_min_m"].as<double>(0.08);
                crane_motion_ekf_cfg_.max_step_max_m = m["max_step_max_m"].as<double>(0.25);
            }

            if (n["diagonal_mode"]) {
                const auto d = n["diagonal_mode"];
                crane_motion_ekf_cfg_.axis_independent_gate =
                    d["axis_independent_gate"].as<bool>(true);
                crane_motion_ekf_cfg_.diagonal_enabled = d["enabled"].as<bool>(false);
                crane_motion_ekf_cfg_.diagonal_min_vx = d["min_vx"].as<double>(0.05);
                crane_motion_ekf_cfg_.diagonal_min_vy = d["min_vy"].as<double>(0.05);
                crane_motion_ekf_cfg_.diagonal_min_speed = d["min_speed"].as<double>(0.10);
                crane_motion_ekf_cfg_.lateral_gate_m = d["lateral_gate_m"].as<double>(0.20);
                crane_motion_ekf_cfg_.tangential_gate_m = d["tangential_gate_m"].as<double>(0.40);
                crane_motion_ekf_cfg_.lateral_damping = d["lateral_damping"].as<double>(0.70);
                crane_motion_ekf_cfg_.tangential_damping = d["tangential_damping"].as<double>(0.40);
                crane_motion_ekf_cfg_.nis_reject_threshold =
                    d["nis_reject_threshold"].as<double>(13.82);
            }

            if (n["recovery"]) {
                const auto r = n["recovery"];
                crane_motion_ekf_cfg_.max_frames_since_good_ndt = r["max_frames_since_good_ndt"].as<int>(30);
                crane_motion_ekf_cfg_.max_high_fitness_frames = r["max_high_fitness_frames"].as<int>(10);
                crane_motion_ekf_cfg_.max_reject_innovation_frames = r["max_reject_innovation_frames"].as<int>(5);
                crane_motion_ekf_cfg_.high_fitness_threshold = r["high_fitness_threshold"].as<double>(0.15);
            }

            crane_motion_ekf_.setConfig(crane_motion_ekf_cfg_);

            ROS_INFO("[CraneMotionEKF] enabled=%d q_pos=%.3f q_vel=%.3f gate=%.2f reject=%.2f reject_high_fitness=%d fitness_reject=%.3f fitness_recover=%.3f",
                     crane_motion_ekf_enabled_ ? 1 : 0,
                     crane_motion_ekf_cfg_.q_pos,
                     crane_motion_ekf_cfg_.q_vel,
                     crane_motion_ekf_cfg_.innovation_gate_m,
                     crane_motion_ekf_cfg_.innovation_reject_m,
                     crane_motion_ekf_cfg_.reject_high_fitness ? 1 : 0,
                     crane_motion_ekf_cfg_.ndt_fitness_reject_threshold,
                     crane_motion_ekf_cfg_.ndt_fitness_recover_threshold);
            ROS_INFO("[CraneMotionEKF:RuntimeGuard] enabled=%d warn_ms=%.1f emergency_ms=%.1f slow_extra_r=%.3f",
                     crane_motion_ekf_cfg_.slow_frame_guard_enabled ? 1 : 0,
                     crane_motion_ekf_cfg_.slow_frame_warn_ms,
                     crane_motion_ekf_cfg_.slow_frame_emergency_ms,
                     crane_motion_ekf_cfg_.slow_frame_extra_r);
            ROS_INFO("[CraneMotionEKF:MotionLimit] max_speed=%.3f safety=%.3f step_min=%.3f step_max=%.3f axis_speed=(%.3f,%.3f) axis_accel=(%.3f,%.3f) axis_gate=%d map_commit_fitness=%.3f",
                     crane_motion_ekf_cfg_.max_speed_mps,
                     crane_motion_ekf_cfg_.max_step_safety_factor,
                     crane_motion_ekf_cfg_.max_step_min_m,
                     crane_motion_ekf_cfg_.max_step_max_m,
                     crane_motion_ekf_cfg_.max_speed_x,
                     crane_motion_ekf_cfg_.max_speed_y,
                     crane_motion_ekf_cfg_.max_accel_x,
                     crane_motion_ekf_cfg_.max_accel_y,
                     crane_motion_ekf_cfg_.axis_independent_gate ? 1 : 0,
                     map_commit_max_fitness_);
        }

        // v8-stable-r3: SoftYawFilter 参数
        // ICP is disabled in the production profile. Parse every field so
        // `enabled: false` is an executable gate, not documentation.
        if (config["icp_refine"]) {
            const auto n = config["icp_refine"];
            icp_refine_cfg_.enabled = n["enabled"].as<bool>(false);
            icp_refine_cfg_.run_after_ndt = n["run_after_ndt"].as<bool>(true);
            icp_refine_cfg_.use_objects_only = n["use_objects_only"].as<bool>(true);
            icp_refine_cfg_.max_iterations =
                std::max(1, n["max_iterations"].as<int>(8));
            icp_refine_cfg_.max_correspondence_distance =
                std::max(0.01, n["max_correspondence_distance"].as<double>(0.20));
            icp_refine_cfg_.transformation_epsilon =
                std::max(1e-8, n["transformation_epsilon"].as<double>(0.002));
            icp_refine_cfg_.min_object_points =
                std::max(10, n["min_object_points"].as<int>(800));
            icp_refine_cfg_.max_icp_ms =
                std::max(1.0, n["max_icp_ms"].as<double>(15.0));
            icp_refine_cfg_.max_fitness =
                std::max(0.0, n["max_fitness"].as<double>(0.50));
        }
        ROS_INFO("[ICPConfig] enabled=%d run_after_ndt=%d objects_only=%d "
                 "min_points=%d iterations=%d max_corr=%.3f epsilon=%.6f "
                 "max_ms=%.1f max_fitness=%.3f",
                 icp_refine_cfg_.enabled ? 1 : 0,
                 icp_refine_cfg_.run_after_ndt ? 1 : 0,
                 icp_refine_cfg_.use_objects_only ? 1 : 0,
                 icp_refine_cfg_.min_object_points,
                 icp_refine_cfg_.max_iterations,
                 icp_refine_cfg_.max_correspondence_distance,
                 icp_refine_cfg_.transformation_epsilon,
                 icp_refine_cfg_.max_icp_ms,
                 icp_refine_cfg_.max_fitness);

        if (config["soft_yaw_filter"]) {
            const auto n = config["soft_yaw_filter"];
            soft_yaw_enabled_ = n["enabled"].as<bool>(true);

            yaw_filter_alpha_stationary_ = n["alpha_stationary"].as<double>(0.04);
            yaw_filter_alpha_moving_ = n["alpha_moving"].as<double>(0.18);
            yaw_filter_alpha_speed_extra_ = n["alpha_speed_extra"].as<double>(0.05);

            yaw_max_step_stationary_rad_ = n["max_step_deg_stationary"].as<double>(0.08) * M_PI / 180.0;
            yaw_max_step_moving_rad_ = n["max_step_deg_moving"].as<double>(0.35) * M_PI / 180.0;

            yaw_warn_raw_filtered_diff_rad_ = n["warn_raw_filtered_diff_deg"].as<double>(3.0) * M_PI / 180.0;

            ROS_INFO("[SoftYaw] enabled=%d alpha_static=%.3f alpha_moving=%.3f",
                     soft_yaw_enabled_ ? 1 : 0,
                     yaw_filter_alpha_stationary_,
                     yaw_filter_alpha_moving_);
        }

        // v8-stable-r3: Registration Input 参数
        if (config["registration_input"]) {
            const auto n = config["registration_input"];
            ndt_input_voxel_size_ = n["ndt_input_voxel_size"].as<double>(0.30);
            object_weight_repeat_ = n["object_weight_repeat"].as<int>(2);
            ground_sample_ratio_ = n["ground_sample_ratio"].as<double>(0.20);
            max_ndt_points_ = n["max_ndt_points"].as<int>(8000);
            min_objects_for_weighting_ = n["min_objects_for_weighting"].as<int>(500);
            min_registration_points_ = n["min_registration_points"].as<int>(2500);

            ROS_INFO("[RegistrationInput] voxel=%.2f repeat=%d ground_ratio=%.2f max_points=%d min_points=%d",
                     ndt_input_voxel_size_,
                     object_weight_repeat_,
                     ground_sample_ratio_,
                     max_ndt_points_,
                     min_registration_points_);
        }

        // V3: Localization Target 参数
        if (config["localization_target"]) {
            const auto lt = config["localization_target"];
            // Backward compatibility: old "enabled" maps to build_enabled only
            bool old_enabled = lt["enabled"].as<bool>(true);
            localization_target_build_enabled_ = lt["build_enabled"].as<bool>(old_enabled);
            localization_target_use_for_ndt_ = lt["use_for_ndt"].as<bool>(false);
            // legacy flag: true only if both build and use are true
            localization_target_enabled_ = localization_target_build_enabled_;
            use_objects_only_initial_ = lt["use_objects_only_initial"].as<bool>(true);
            include_ground_edge_ = lt["include_ground_edge"].as<bool>(false);
            localization_target_min_points_ = lt["min_points"].as<int>(3000);
            localization_target_max_points_ = lt["max_points"].as<int>(60000);
            localization_target_voxel_size_ = lt["voxel_size"].as<double>(0.30);
            crop_enabled_ = lt["crop_enabled"].as<bool>(true);
            crop_radius_x_ = lt["radius_x"].as<double>(15.0);
            crop_radius_y_ = lt["radius_y"].as<double>(7.0);
            crop_update_distance_m_ = lt["update_distance_m"].as<double>(0.50);
            crop_update_yaw_deg_ = lt["update_yaw_deg"].as<double>(2.0);
            crop_update_min_interval_frames_ = lt["update_min_interval_frames"].as<int>(3);

            ROS_INFO("[LocTarget] build_enabled=%d use_for_ndt=%d objects_only=%d ground_edge=%d min=%d max=%d voxel=%.2f crop=%d radius=(%.1f,%.1f) update_dist=%.2f update_yaw=%.1f min_interval=%d",
                     localization_target_build_enabled_ ? 1 : 0,
                     localization_target_use_for_ndt_ ? 1 : 0,
                     use_objects_only_initial_ ? 1 : 0,
                     include_ground_edge_ ? 1 : 0,
                     localization_target_min_points_,
                     localization_target_max_points_,
                     localization_target_voxel_size_,
                     crop_enabled_ ? 1 : 0,
                     crop_radius_x_, crop_radius_y_,
                     crop_update_distance_m_, crop_update_yaw_deg_,
                     crop_update_min_interval_frames_);
        }

        // v8-stable-r3: Adaptive NDT 参数
        if (config["adaptive_ndt"]) {
            const auto n = config["adaptive_ndt"];
            adaptive_ndt_enabled_ = n["enabled"].as<bool>(true);
            adaptive_target_total_ms_ = n["target_total_ms"].as<double>(80.0);
            adaptive_emergency_total_ms_ = n["emergency_total_ms"].as<double>(120.0);

            ROS_INFO("[AdaptiveNDT] enabled=%d target_ms=%.1f emergency_ms=%.1f",
                     adaptive_ndt_enabled_ ? 1 : 0,
                     adaptive_target_total_ms_,
                     adaptive_emergency_total_ms_);
        }

        // HookFixedCargoDetector 配置
        if (config["hook_fixed_cargo_detector"]) {
            const auto hfc = config["hook_fixed_cargo_detector"];
            hook_fixed_config_.enabled = hfc["enabled"].as<bool>(true);
            hook_fixed_config_.roi_center_x = hfc["roi_center_x"].as<float>(0.0f);
            hook_fixed_config_.roi_center_y = hfc["roi_center_y"].as<float>(-2.2f);
            hook_fixed_config_.roi_half_x = hfc["roi_half_x"].as<float>(1.5f);
            hook_fixed_config_.roi_half_y = hfc["roi_half_y"].as<float>(1.5f);
            hook_fixed_config_.roi_z_min = hfc["roi_z_min"].as<float>(0.25f);
            hook_fixed_config_.roi_z_max = hfc["roi_z_max"].as<float>(3.0f);
            hook_fixed_config_.voxel_leaf = hfc["voxel_leaf"].as<float>(0.05f);
            hook_fixed_config_.cluster_tolerance = hfc["cluster_tolerance"].as<float>(0.20f);
            hook_fixed_config_.min_cluster_points = hfc["min_cluster_points"].as<int>(15);
            hook_fixed_config_.max_cluster_points = hfc["max_cluster_points"].as<int>(8000);
            hook_fixed_config_.reject_rope_radius = hfc["reject_rope_radius"].as<float>(0.08f);
            hook_fixed_config_.reject_rope_min_z = hfc["reject_rope_min_z"].as<float>(2.0f);
            hook_fixed_config_.reject_structure_z = hfc["reject_structure_z"].as<float>(3.2f);
            hook_fixed_config_.xy_mode = hfc["xy_mode"].as<std::string>("roi_center");
            hook_fixed_config_.min_long_side = hfc["min_long_side"].as<float>(0.30f);
            hook_fixed_config_.min_short_side = hfc["min_short_side"].as<float>(0.20f);
            hook_fixed_config_.min_visible_height = hfc["min_visible_height"].as<float>(0.08f);
            hook_fixed_config_.max_long_side = hfc["max_long_side"].as<float>(4.0f);
            hook_fixed_config_.max_short_side = hfc["max_short_side"].as<float>(3.0f);
            hook_fixed_config_.max_height = hfc["max_height"].as<float>(3.0f);
            hook_fixed_config_.allow_visible_box_without_bottom = hfc["allow_visible_box_without_bottom"].as<bool>(true);

            ROS_INFO("[HookFixedCargoDetector] enabled=%d roi_center=(%.2f,%.2f) roi_half=(%.2f,%.2f) z=[%.2f,%.2f]",
                     hook_fixed_config_.enabled ? 1 : 0,
                     hook_fixed_config_.roi_center_x, hook_fixed_config_.roi_center_y,
                     hook_fixed_config_.roi_half_x, hook_fixed_config_.roi_half_y,
                     hook_fixed_config_.roi_z_min, hook_fixed_config_.roi_z_max);
        }

        // HookCargoLock 配置
        // The voltage classifier runs in a small independent node. The SLAM
        // node consumes only its typed, debounced state and applies lifecycle
        // and fail-safe policy here.
        if (config["hook_load_signal"]) {
            const auto hls = config["hook_load_signal"];
            hook_load_signal_enabled_ = hls["enabled"].as<bool>(true);
            hook_load_signal_required_ = hls["required"].as<bool>(true);
            hook_load_state_topic_ =
                hls["state_topic"].as<std::string>("/hook/load_state");
            hook_load_state_stale_timeout_sec_ = std::max(
                0.10, hls["consumer_stale_timeout_sec"].as<double>(0.80));
            const int history_samples = std::clamp(
                hls["origin_history_samples"].as<int>(10), 3, 200);
            empty_hook_height_history_max_samples_ =
                static_cast<std::size_t>(history_samples);
            ROS_INFO(
                "[HookLoadSignal] enabled=%d required=%d state_topic=%s "
                "consumer_stale=%.2fs origin_samples=%zu",
                hook_load_signal_enabled_ ? 1 : 0,
                hook_load_signal_required_ ? 1 : 0,
                hook_load_state_topic_.c_str(),
                hook_load_state_stale_timeout_sec_,
                empty_hook_height_history_max_samples_);
        }

        if (config["hook_cargo_lock"]) {
            const auto hcl = config["hook_cargo_lock"];
            hook_lock_config_.enabled = hcl["enabled"].as<bool>(true);
            hook_lock_config_.lock_confirm_frames = hcl["lock_confirm_frames"].as<int>(3);
            hook_lock_config_.size_init_window = hcl["size_init_window"].as<int>(5);
            hook_lock_config_.lost_hold_sec = hcl["lost_hold_sec"].as<float>(3.0f);
            hook_lock_config_.lost_clear_sec = hcl["lost_clear_sec"].as<float>(8.0f);
            hook_lock_config_.strong_min_points = hcl["strong_min_points"].as<int>(30);
            hook_lock_config_.weak_min_points = hcl["weak_min_points"].as<int>(5);
            hook_lock_config_.size_change_min_ratio = hcl["size_change_min_ratio"].as<float>(0.20f);
            hook_lock_config_.size_change_max_ratio = hcl["size_change_max_ratio"].as<float>(0.60f);
            hook_lock_config_.size_update_confirm_frames = hcl["size_update_confirm_frames"].as<int>(5);
            hook_lock_config_.size_update_alpha = hcl["size_update_alpha"].as<float>(0.15f);
            hook_lock_config_.bottom_alpha_points = hcl["bottom_alpha_points"].as<float>(0.30f);
            hook_lock_config_.bottom_alpha_memory = hcl["bottom_alpha_memory"].as<float>(0.15f);
            hook_lock_config_.bottom_hold_uncertainty_growth = hcl["bottom_hold_uncertainty_growth"].as<float>(0.02f);
            hook_lock_config_.bottom_max_uncertainty = hcl["bottom_max_uncertainty"].as<float>(0.35f);
            hook_lock_config_.candidate_hold_sec = hcl["candidate_hold_sec"].as<float>(1.0f);
            hook_lock_config_.candidate_max_weak_frames = hcl["candidate_max_weak_frames"].as<int>(10);

            // locked association gate 配置
            hook_lock_config_.locked_update_max_center_dist = hcl["locked_update_max_center_dist"].as<float>(0.65f);
            hook_lock_config_.locked_update_min_overlap_ratio = hcl["locked_update_min_overlap_ratio"].as<float>(0.30f);
            hook_lock_config_.locked_update_max_z_jump = hcl["locked_update_max_z_jump"].as<float>(0.45f);
            hook_lock_config_.locked_update_max_top_jump = hcl["locked_update_max_top_jump"].as<float>(0.60f);
            hook_lock_config_.locked_update_min_points = hcl["locked_update_min_points"].as<int>(20);

            // 锁定时 strong 条件
            hook_lock_config_.lock_strong_min_points = hcl["lock_strong_min_points"].as<int>(80);
            hook_lock_config_.lock_min_visible_height = hcl["lock_min_visible_height"].as<float>(0.50f);
            hook_lock_config_.lock_min_xy_area = hcl["lock_min_xy_area"].as<float>(0.40f);

            // locked search margin
            hook_lock_config_.locked_search_margin_x = hcl["locked_search_margin_x"].as<float>(0.30f);
            hook_lock_config_.locked_search_margin_y = hcl["locked_search_margin_y"].as<float>(0.30f);

            // 吊物点云去除
            hook_lock_config_.enable_hook_cargo_removal = hcl["enable_hook_cargo_removal"].as<bool>(false);

            ROS_INFO("[HookCargoLock] enabled=%d lock_confirm=%d lost_hold=%.1f lost_clear=%.1f strong=%d weak=%d removal=%d",
                     hook_lock_config_.enabled ? 1 : 0,
                     hook_lock_config_.lock_confirm_frames,
                     hook_lock_config_.lost_hold_sec,
                     hook_lock_config_.lost_clear_sec,
                     hook_lock_config_.strong_min_points,
                     hook_lock_config_.weak_min_points,
                     hook_lock_config_.enable_hook_cargo_removal ? 1 : 0);
            ROS_INFO("[HookCargoLock] lock_strong=%d min_visible_h=%.2f min_xy_area=%.2f max_center_dist=%.2f",
                     hook_lock_config_.lock_strong_min_points,
                     hook_lock_config_.lock_min_visible_height,
                     hook_lock_config_.lock_min_xy_area,
                     hook_lock_config_.locked_update_max_center_dist);
        }

        // ========== OdomAnchorBox 配置 ==========
        if (config["odom_anchored_cargo_box"]) {
            auto oac = config["odom_anchored_cargo_box"];
            odom_anchor_config_.enabled = oac["enabled"].as<bool>(true);
            odom_anchor_config_.anchor_x = oac["anchor_x"].as<float>(0.0f);
            odom_anchor_config_.anchor_y = oac["anchor_y"].as<float>(0.0f);

            // 检测和 marker 降频
            odom_anchor_config_.detect_rate_hz = oac["detect_rate_hz"].as<float>(5.0f);
            odom_anchor_config_.marker_rate_hz = oac["marker_rate_hz"].as<float>(5.0f);

            // debug 点云发布
            odom_anchor_config_.publish_debug_points = oac["publish_debug_points"].as<bool>(false);
            odom_anchor_config_.publish_selected_core_points = oac["publish_selected_core_points"].as<bool>(false);
            odom_anchor_config_.publish_raw_candidate_points = oac["publish_raw_candidate_points"].as<bool>(false);
            odom_anchor_config_.publish_default_box_marker = oac["publish_default_box_marker"].as<bool>(false);

            // 日志控制
            odom_anchor_config_.verbose_debug = oac["verbose_debug"].as<bool>(false);
            odom_anchor_config_.summary_log_period = oac["summary_log_period"].as<float>(2.0f);

            // 旧 cargo 链路开关
            odom_anchor_config_.use_global_payload_tracker = oac["use_global_payload_tracker"].as<bool>(false);
            odom_anchor_config_.use_cargobox_v2 = oac["use_cargobox_v2"].as<bool>(false);
            odom_anchor_config_.use_dynamic_history_eraser = oac["use_dynamic_history_eraser"].as<bool>(false);

            // 检测窗口
            odom_anchor_config_.search_half_x = oac["search_half_x"].as<float>(1.20f);
            odom_anchor_config_.search_half_y = oac["search_half_y"].as<float>(1.20f);
            odom_anchor_config_.search_z_min = oac["search_z_min"].as<float>(0.05f);
            odom_anchor_config_.search_z_max = oac["search_z_max"].as<float>(3.20f);

            // 尺寸配置
            odom_anchor_config_.default_size_x = oac["default_size_x"].as<float>(0.50f);
            odom_anchor_config_.default_size_y = oac["default_size_y"].as<float>(0.35f);
            odom_anchor_config_.default_size_z = oac["default_size_z"].as<float>(0.25f);
            odom_anchor_config_.min_size_x = oac["min_size_x"].as<float>(0.60f);
            odom_anchor_config_.min_size_y = oac["min_size_y"].as<float>(0.40f);
            odom_anchor_config_.min_size_z = oac["min_size_z"].as<float>(0.20f);
            odom_anchor_config_.max_size_x = oac["max_size_x"].as<float>(2.50f);
            odom_anchor_config_.max_size_y = oac["max_size_y"].as<float>(1.60f);
            odom_anchor_config_.max_size_z = oac["max_size_z"].as<float>(2.00f);
            odom_anchor_config_.size_margin_x = oac["size_margin_x"].as<float>(0.10f);
            odom_anchor_config_.size_margin_y = oac["size_margin_y"].as<float>(0.10f);
            odom_anchor_config_.size_margin_z = oac["size_margin_z"].as<float>(0.05f);
            odom_anchor_config_.lock_confirm_frames = oac["lock_confirm_frames"].as<int>(2);
            odom_anchor_config_.strong_min_points = oac["strong_min_points"].as<int>(50);
            odom_anchor_config_.weak_min_points = oac["weak_min_points"].as<int>(10);
            odom_anchor_config_.size_update_confirm_frames = oac["size_update_confirm_frames"].as<int>(5);
            odom_anchor_config_.size_change_min_ratio = oac["size_change_min_ratio"].as<float>(0.20f);
            odom_anchor_config_.size_change_max_ratio = oac["size_change_max_ratio"].as<float>(0.60f);
            odom_anchor_config_.size_update_alpha = oac["size_update_alpha"].as<float>(0.15f);
            odom_anchor_config_.bottom_alpha_points = oac["bottom_alpha_points"].as<float>(0.25f);
            odom_anchor_config_.bottom_alpha_hold = oac["bottom_alpha_hold"].as<float>(0.05f);
            odom_anchor_config_.bottom_max_uncertainty = oac["bottom_max_uncertainty"].as<float>(0.35f);
            odom_anchor_config_.lost_hold_sec = oac["lost_hold_sec"].as<float>(5.0f);
            odom_anchor_config_.lost_clear_sec = oac["lost_clear_sec"].as<float>(15.0f);

            // Tight Box 配置
            if (oac["tight_box"]) {
                auto tb = oac["tight_box"];
                odom_anchor_config_.tight_box.enabled = tb["enabled"].as<bool>(true);
                odom_anchor_config_.tight_box.anchor_symmetry_mode = tb["anchor_symmetry_mode"].as<std::string>("soft");
                odom_anchor_config_.tight_box.max_center_offset_m = tb["max_center_offset_m"].as<float>(0.35f);
                odom_anchor_config_.tight_box.hag_filter_enabled = tb["hag_filter_enabled"].as<bool>(true);
                odom_anchor_config_.tight_box.hag_min_m = tb["hag_min_m"].as<float>(0.15f);
                odom_anchor_config_.tight_box.hag_max_m = tb["hag_max_m"].as<float>(2.50f);
                odom_anchor_config_.tight_box.percentile_low = tb["percentile_low"].as<float>(0.08f);
                odom_anchor_config_.tight_box.percentile_high = tb["percentile_high"].as<float>(0.92f);
                odom_anchor_config_.tight_box.margin_xy_m = tb["margin_xy_m"].as<float>(0.05f);
                odom_anchor_config_.tight_box.margin_z_m = tb["margin_z_m"].as<float>(0.03f);
                odom_anchor_config_.tight_box.size_update_mode = tb["size_update_mode"].as<std::string>("adaptive");
                odom_anchor_config_.tight_box.size_update_alpha = tb["size_update_alpha"].as<float>(0.30f);
                odom_anchor_config_.tight_box.max_size_change_per_frame_m = tb["max_size_change_per_frame_m"].as<float>(0.10f);
                odom_anchor_config_.tight_box.sub_cluster_enabled = tb["sub_cluster_enabled"].as<bool>(true);
                odom_anchor_config_.tight_box.sub_cluster_tolerance_m = tb["sub_cluster_tolerance_m"].as<float>(0.10f);
                odom_anchor_config_.tight_box.sub_cluster_min_points = tb["sub_cluster_min_points"].as<int>(20);

                ROS_INFO("[TightBoxConfig] enabled=%d symmetry=%s hag_filter=%d percentile=[%.2f,%.2f] sub_cluster=%d",
                         odom_anchor_config_.tight_box.enabled ? 1 : 0,
                         odom_anchor_config_.tight_box.anchor_symmetry_mode.c_str(),
                         odom_anchor_config_.tight_box.hag_filter_enabled ? 1 : 0,
                         odom_anchor_config_.tight_box.percentile_low,
                         odom_anchor_config_.tight_box.percentile_high,
                         odom_anchor_config_.tight_box.sub_cluster_enabled ? 1 : 0);
            }

            // Cargo Warning 配置
            if (oac["cargo_warning"]) {
                auto cw = oac["cargo_warning"];
                odom_anchor_config_.cargo_warning.enabled = cw["enabled"].as<bool>(true);
                odom_anchor_config_.cargo_warning.publish_alarm_msg = cw["publish_alarm_msg"].as<bool>(false);
                odom_anchor_config_.cargo_warning.publish_debug_marker = cw["publish_debug_marker"].as<bool>(true);
                odom_anchor_config_.cargo_warning.level1_distance_m = cw["level1_distance_m"].as<float>(3.0f);
                odom_anchor_config_.cargo_warning.level2_distance_m = cw["level2_distance_m"].as<float>(5.0f);
                odom_anchor_config_.cargo_warning.min_vertical_clearance_m = cw["min_vertical_clearance_m"].as<float>(0.80f);
                odom_anchor_config_.cargo_warning.cargo_bottom_use_uncertainty = cw["cargo_bottom_use_uncertainty"].as<bool>(true);
                odom_anchor_config_.cargo_warning.cargo_bottom_extra_margin_m = cw["cargo_bottom_extra_margin_m"].as<float>(0.05f);
                odom_anchor_config_.cargo_warning.obstacle_top_percentile = cw["obstacle_top_percentile"].as<float>(0.95f);
                odom_anchor_config_.cargo_warning.obstacle_min_points = cw["obstacle_min_points"].as<int>(5);
                odom_anchor_config_.cargo_warning.obstacle_cluster_tolerance_m = cw["obstacle_cluster_tolerance_m"].as<float>(0.25f);
                odom_anchor_config_.cargo_warning.maximum_obstacle_cloud_age_sec =
                    std::max(0.05f, cw["maximum_obstacle_cloud_age_sec"].as<float>(0.50f));
                odom_anchor_config_.cargo_warning.minimum_roi_finite_points =
                    std::max(1, cw["minimum_roi_finite_points"].as<int>(20));
                odom_anchor_config_.cargo_warning.minimum_roi_coverage_ratio =
                    std::clamp(cw["minimum_roi_coverage_ratio"].as<float>(0.01f),
                               0.0f, 1.0f);
                odom_anchor_config_.cargo_warning.exclude_ground = cw["exclude_ground"].as<bool>(true);
                odom_anchor_config_.cargo_warning.ground_hag_min_m = cw["ground_hag_min_m"].as<float>(0.20f);
                odom_anchor_config_.cargo_warning.exclude_self_cargo = cw["exclude_self_cargo"].as<bool>(true);
                odom_anchor_config_.cargo_warning.self_cargo_margin_xy_m = cw["self_cargo_margin_xy_m"].as<float>(0.45f);
                odom_anchor_config_.cargo_warning.self_cargo_margin_z_m = cw["self_cargo_margin_z_m"].as<float>(0.35f);
                odom_anchor_config_.cargo_warning.debounce_frames = cw["debounce_frames"].as<int>(2);
                odom_anchor_config_.cargo_warning.clear_hold_sec = cw["clear_hold_sec"].as<float>(0.5f);
                odom_anchor_config_.cargo_warning.level1_alarm_code = cw["level1_alarm_code"].as<int>(17);
                odom_anchor_config_.cargo_warning.level2_alarm_code = cw["level2_alarm_code"].as<int>(18);
                odom_anchor_config_.cargo_warning.clear_alarm_code = cw["clear_alarm_code"].as<int>(0);

                ROS_INFO("[CargoWarningConfig] enabled=%d publish_alarm=%d debug_marker=%d level1_dist=%.1f level2_dist=%.1f clearance=%.2f",
                         odom_anchor_config_.cargo_warning.enabled ? 1 : 0,
                         odom_anchor_config_.cargo_warning.publish_alarm_msg ? 1 : 0,
                         odom_anchor_config_.cargo_warning.publish_debug_marker ? 1 : 0,
                         odom_anchor_config_.cargo_warning.level1_distance_m,
                         odom_anchor_config_.cargo_warning.level2_distance_m,
                         odom_anchor_config_.cargo_warning.min_vertical_clearance_m);
            }

            ROS_INFO("[OdomAnchorBoxConfig] enabled=%d anchor=(%.2f,%.2f) detect_rate=%.1f marker_rate=%.1f debug_points=%d global_payload=%d cargobox_v2=%d dynamic_eraser=%d",
                     odom_anchor_config_.enabled ? 1 : 0,
                     odom_anchor_config_.anchor_x, odom_anchor_config_.anchor_y,
                     odom_anchor_config_.detect_rate_hz, odom_anchor_config_.marker_rate_hz,
                     odom_anchor_config_.publish_debug_points ? 1 : 0,
                     odom_anchor_config_.use_global_payload_tracker ? 1 : 0,
                     odom_anchor_config_.use_cargobox_v2 ? 1 : 0,
                     odom_anchor_config_.use_dynamic_history_eraser ? 1 : 0);
        }

        // ConfigFinal 日志
        ROS_INFO("[ConfigFinal] hook_cargo_removal=%d source=config",
                 hook_lock_config_.enable_hook_cargo_removal ? 1 : 0);

        // Runtime Diagnostics 配置
        if (config["debug"] && config["debug"]["runtime_diagnostics"]) {
            auto diag = config["debug"]["runtime_diagnostics"];
            runtime_diag_config_.enabled = diag["enabled"].as<bool>(false);
            runtime_diag_config_.console_period_sec = diag["console_period_sec"].as<double>(1.0);
            runtime_diag_config_.csv_enabled = diag["csv_enabled"].as<bool>(true);
            runtime_diag_config_.csv_flush_period_sec = diag["csv_flush_period_sec"].as<double>(1.0);
            runtime_diag_config_.warn_consecutive_overrun_frames = diag["warn_consecutive_overrun_frames"].as<int>(3);
            runtime_diag_config_.warn_prediction_only_frames = diag["warn_prediction_only_frames"].as<int>(3);
            runtime_diag_config_.warn_target_fallback_frames = diag["warn_target_fallback_frames"].as<int>(3);
            runtime_diag_config_.warn_cargo_bottom_jump_m = diag["warn_cargo_bottom_jump_m"].as<double>(0.20);
            runtime_diag_config_.warn_cargo_height_jump_m = diag["warn_cargo_height_jump_m"].as<double>(0.20);

            // 默认输出目录
            diag_output_dir_ = "/home/ydkj/ndt_slam_runtime_data";

            ROS_INFO("[RuntimeDiagnostics] enabled=%d console_period=%.1f csv=%d csv_flush=%.1f",
                     runtime_diag_config_.enabled ? 1 : 0,
                     runtime_diag_config_.console_period_sec,
                     runtime_diag_config_.csv_enabled ? 1 : 0,
                     runtime_diag_config_.csv_flush_period_sec);
        }

        if (config["relocalization"]) {
            const auto r = config["relocalization"];
            relocalization_enabled_ = r["enabled"].as<bool>(true);
            relocalization_trigger_frames_ =
                std::max(2, r["trigger_frames"].as<int>(5));
            relocalization_global_trigger_frames_ = std::max(
                relocalization_trigger_frames_,
                r["global_trigger_frames"].as<int>(15));
            relocalization_confirm_frames_ = std::clamp(
                r["confirm_frames"].as<int>(2), 2, 5);
            relocalization_request_interval_frames_ = std::max(
                1, r["request_interval_frames"].as<int>(3));
            relocalization_result_max_age_frames_ = std::max(
                2, r["result_max_age_frames"].as<int>(8));
            relocalization_result_max_age_sec_ = std::max(
                0.10, r["result_max_age_sec"].as<double>(0.50));
            relocalization_cooldown_frames_ = std::max(
                1, r["cooldown_frames"].as<int>(12));
            relocalization_global_hint_count_ = std::clamp(
                r["global_hint_count"].as<int>(4), 1, 12);
            relocalization_global_min_similarity_ =
                r["global_min_similarity"].as<double>(0.55);
            relocalization_local_xy_window_m_ =
                r["local_xy_window_m"].as<double>(1.5);
            relocalization_local_xy_step_m_ = std::max(
                0.25, r["local_xy_step_m"].as<double>(1.5));
            relocalization_local_yaw_window_deg_ =
                r["local_yaw_window_deg"].as<double>(12.0);
            relocalization_local_yaw_step_deg_ = std::max(
                1.0, r["local_yaw_step_deg"].as<double>(12.0));
            relocalization_confirm_translation_m_ =
                r["confirm_translation_m"].as<double>(0.35);
            relocalization_confirm_yaw_deg_ =
                r["confirm_yaw_deg"].as<double>(5.0);

            relocalization_cfg_.enabled = relocalization_enabled_;
            relocalization_cfg_.min_source_points =
                r["min_source_points"].as<int>(800);
            relocalization_cfg_.min_target_points =
                r["min_target_points"].as<int>(1200);
            relocalization_cfg_.max_candidates =
                r["max_candidates"].as<int>(12);
            relocalization_cfg_.max_iterations =
                r["max_iterations"].as<int>(35);
            relocalization_cfg_.num_threads =
                r["num_threads"].as<int>(2);
            relocalization_cfg_.resolution =
                r["resolution"].as<double>(ndt_resolution_);
            relocalization_cfg_.step_size =
                r["step_size"].as<double>(0.20);
            relocalization_cfg_.transformation_epsilon =
                r["transformation_epsilon"].as<double>(0.01);
            relocalization_cfg_.target_crop_radius_m =
                r["target_crop_radius_m"].as<double>(18.0);
            relocalization_cfg_.source_voxel_m =
                r["source_voxel_m"].as<double>(0.30);
            relocalization_cfg_.target_voxel_m =
                r["target_voxel_m"].as<double>(0.40);
            relocalization_cfg_.max_fitness =
                r["max_fitness"].as<double>(2.0);
            relocalization_cfg_.min_probability =
                r["min_probability"].as<double>(0.0);
            relocalization_cfg_.max_local_seed_correction_m =
                r["max_local_seed_correction_m"].as<double>(3.0);
            relocalization_cfg_.max_local_seed_yaw_correction_deg =
                r["max_local_seed_yaw_correction_deg"].as<double>(20.0);
            relocalization_cfg_.max_roll_pitch_deg =
                r["max_roll_pitch_deg"].as<double>(3.0);
            relocalization_cfg_.max_z_correction_m =
                r["max_z_correction_m"].as<double>(0.50);
        }

        CargoBottomFusionConfig bottom_fusion_config;
        bottom_fusion_config.points_min_points = static_cast<std::size_t>(
            std::max(20, hook_fixed_config_.bottom_min_points));
        bottom_fusion_config.points_min_bottom_band_points =
            static_cast<std::size_t>(
                std::max(5, hook_fixed_config_.bottom_band_min_points));
        bottom_fusion_config.points_min_bottom_band_xy_cells =
            static_cast<std::size_t>(
                std::max(2, hook_fixed_config_.bottom_band_min_xy_cells));
        bottom_fusion_config.bottom_band_height =
            std::max(0.05F, hook_fixed_config_.bottom_band_height);
        bottom_fusion_config.xy_cell_size =
            std::max(0.05F, hook_fixed_config_.bottom_xy_cell_size);
        cargo_bottom_fusion_.setConfig(bottom_fusion_config);

        CargoSafetyConfig safety_config;
        safety_config.level1_distance_m =
            odom_anchor_config_.cargo_warning.level1_distance_m;
        safety_config.level2_distance_m =
            odom_anchor_config_.cargo_warning.level2_distance_m;
        safety_config.minimum_vertical_clearance_m =
            odom_anchor_config_.cargo_warning.min_vertical_clearance_m;
        safety_config.cargo_bottom_extra_margin_m =
            odom_anchor_config_.cargo_warning.cargo_bottom_extra_margin_m;
        safety_config.maximum_height_age_sec = 0.80;
        safety_config.obstacle_top_percentile =
            odom_anchor_config_.cargo_warning.obstacle_top_percentile;
        safety_config.obstacle_cluster_tolerance_m =
            odom_anchor_config_.cargo_warning.obstacle_cluster_tolerance_m;
        safety_config.obstacle_min_cluster_points =
            static_cast<std::size_t>(std::max(
                1, odom_anchor_config_.cargo_warning.obstacle_min_points));
        safety_config.maximum_obstacle_cloud_age_sec =
            odom_anchor_config_.cargo_warning.maximum_obstacle_cloud_age_sec;
        safety_config.minimum_roi_finite_points =
            static_cast<std::size_t>(
                odom_anchor_config_.cargo_warning.minimum_roi_finite_points);
        safety_config.minimum_roi_coverage_ratio =
            odom_anchor_config_.cargo_warning.minimum_roi_coverage_ratio;
        safety_config.exclude_self_cargo =
            odom_anchor_config_.cargo_warning.exclude_self_cargo;
        cargo_safety_evaluator_.setConfig(safety_config);

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parse error: %s", e.what());
    }
}

void NdtSlamNode::initializeParameters() {
    kiss_icp_config_ = kiss_icp::pipeline::KISSConfig();
}

void NdtSlamNode::hookLoadStateCallback(
    const lidar_slam2_msgs::HookLoadState::ConstPtr& msg) {
    if (!msg) return;

    const bool schema_valid =
        msg->schema_version == lidar_slam2_msgs::HookLoadState::SCHEMA_VERSION;
    const bool state_valid =
        msg->state == lidar_slam2_msgs::HookLoadState::STATE_INHIBIT ||
        msg->state == lidar_slam2_msgs::HookLoadState::STATE_EMPTY ||
        msg->state == lidar_slam2_msgs::HookLoadState::STATE_LOADED;
    const bool voltage_valid = std::isfinite(msg->voltage);
    const bool basic_valid = schema_valid && msg->valid && msg->fresh &&
                             state_valid && voltage_valid;

    std::lock_guard<std::mutex> lock(hook_load_state_mutex_);
    const bool source_stamp_valid = !msg->header.stamp.isZero() &&
        (hook_load_snapshot_.source_stamp.isZero() ||
         msg->header.stamp.toSec() + 1.0e-6 >=
             hook_load_snapshot_.source_stamp.toSec());
    const bool valid = basic_valid && source_stamp_valid;
    const double receipt_wall_sec = ros::WallTime::now().toSec();
    const bool source_stamp_advanced =
        hook_load_snapshot_.source_stamp.isZero() ||
        msg->header.stamp.toSec() >
            hook_load_snapshot_.source_stamp.toSec() + 1.0e-6;
    const bool was_empty = hook_load_snapshot_.valid &&
        hook_load_snapshot_.state ==
            lidar_slam2_msgs::HookLoadState::STATE_EMPTY;
    const bool becomes_loaded = valid &&
        msg->state == lidar_slam2_msgs::HookLoadState::STATE_LOADED;

    if (was_empty && becomes_loaded) {
        pending_origin_height_valid_ = false;
        if (empty_hook_height_history_.size() >= 3U) {
            std::vector<float> heights(
                empty_hook_height_history_.begin(),
                empty_hook_height_history_.end());
            const auto middle = heights.begin() +
                static_cast<std::ptrdiff_t>(heights.size() / 2U);
            std::nth_element(heights.begin(), middle, heights.end());
            pending_origin_height_m_ = *middle;
            pending_origin_height_valid_ =
                std::isfinite(pending_origin_height_m_) &&
                pending_origin_height_m_ >=
                    cargo_bottom_fusion_.config().minimum_prior_height &&
                pending_origin_height_m_ <=
                    cargo_bottom_fusion_.config().maximum_prior_height;
        }
        empty_hook_height_history_.clear();
    } else if (valid &&
               msg->state ==
                   lidar_slam2_msgs::HookLoadState::STATE_EMPTY &&
               (!hook_load_snapshot_.valid ||
                hook_load_snapshot_.state !=
                    lidar_slam2_msgs::HookLoadState::STATE_EMPTY)) {
        empty_hook_height_history_.clear();
        pending_origin_height_valid_ = false;
    }

    hook_load_snapshot_.valid = valid;
    hook_load_snapshot_.state = valid
        ? msg->state
        : lidar_slam2_msgs::HookLoadState::STATE_UNKNOWN;
    hook_load_snapshot_.voltage = msg->voltage;
    hook_load_snapshot_.stable_samples = msg->stable_samples;
    hook_load_snapshot_.source_stamp = msg->header.stamp;
    hook_load_snapshot_.receipt_wall_sec = receipt_wall_sec;
    if (source_stamp_advanced ||
        hook_load_snapshot_.source_progress_wall_sec <= 0.0) {
        hook_load_snapshot_.source_progress_wall_sec = receipt_wall_sec;
    }
    hook_load_snapshot_.reason = valid
        ? msg->reason
        : (!schema_valid ? "schema_mismatch" :
           (!source_stamp_valid ? "source_time_rollback_or_zero" :
            (!msg->fresh ? "signal_stale" :
             (!voltage_valid ? "invalid_voltage" : msg->reason))));
}

NdtSlamNode::HookLoadSnapshot NdtSlamNode::currentHookLoadSnapshot() const {
    if (!hook_load_signal_enabled_) {
        HookLoadSnapshot legacy;
        legacy.valid = true;
        legacy.state = lidar_slam2_msgs::HookLoadState::STATE_LOADED;
        legacy.reason = "hook_signal_disabled_legacy_mode";
        return legacy;
    }

    HookLoadSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lock(hook_load_state_mutex_);
        snapshot = hook_load_snapshot_;
    }
    const double now = ros::WallTime::now().toSec();
    const double receipt_age = now - snapshot.receipt_wall_sec;
    const double progress_age = now - snapshot.source_progress_wall_sec;
    if (!std::isfinite(receipt_age) || !std::isfinite(progress_age) ||
        snapshot.receipt_wall_sec <= 0.0 ||
        snapshot.source_progress_wall_sec <= 0.0 ||
        receipt_age < 0.0 || progress_age < 0.0 ||
        receipt_age > hook_load_state_stale_timeout_sec_ ||
        progress_age > hook_load_state_stale_timeout_sec_) {
        snapshot.valid = false;
        snapshot.state = lidar_slam2_msgs::HookLoadState::STATE_UNKNOWN;
        snapshot.reason = "consumer_signal_stale";
    }
    if (!snapshot.valid && !hook_load_signal_required_) {
        snapshot.valid = true;
        snapshot.state = lidar_slam2_msgs::HookLoadState::STATE_LOADED;
        snapshot.reason = "optional_signal_legacy_fallback";
    }
    return snapshot;
}

bool NdtSlamNode::isHookCargoRemovalEnabled() const {
    if (!hook_lock_config_.enable_hook_cargo_removal) return false;
    const HookLoadSnapshot hook = currentHookLoadSnapshot();
    return hook.valid &&
           hook.state == lidar_slam2_msgs::HookLoadState::STATE_LOADED;
}

bool NdtSlamNode::hookAllowsMapCommit() const {
    if (!hook_load_signal_enabled_) return true;
    const HookLoadSnapshot hook = currentHookLoadSnapshot();
    if (hook.reason == "optional_signal_legacy_fallback") return true;
    if (!hook.valid) return false;
    if (hook.state == lidar_slam2_msgs::HookLoadState::STATE_EMPTY) return true;
    if (hook.state != lidar_slam2_msgs::HookLoadState::STATE_LOADED) return false;
    return cargo_state_.state == CargoState::LOCKED &&
           cargo_state_.valid_geometry && cargo_state_.valid_height;
}

void NdtSlamNode::recordEmptyHookOriginHeight(float height_m) {
    if (!std::isfinite(height_m) ||
        height_m < cargo_bottom_fusion_.config().minimum_prior_height ||
        height_m > cargo_bottom_fusion_.config().maximum_prior_height) {
        return;
    }
    std::lock_guard<std::mutex> lock(hook_load_state_mutex_);
    if (!hook_load_snapshot_.valid ||
        hook_load_snapshot_.state !=
            lidar_slam2_msgs::HookLoadState::STATE_EMPTY) {
        return;
    }
    empty_hook_height_history_.push_back(height_m);
    while (empty_hook_height_history_.size() >
           empty_hook_height_history_max_samples_) {
        empty_hook_height_history_.pop_front();
    }
}

void NdtSlamNode::resetCargoForHookState(bool preserve_origin_height) {
    clearHookLock();
    cargo_state_ = CargoState{};
    hook_fixed_cargo_ = HookCargoDetection{};
    hook_fixed_bottom_ = HookCargoBottomEstimate{};
    hook_observation_associated_current_ = false;
    cargo_bottom_fusion_.reset();
    cargo_fusion_track_active_ = false;
    cargo_origin_height_valid_ = false;
    cargo_origin_height_m_ = 0.0F;
    cargo_origin_height_track_id_ = 0U;
    last_cargo_bottom_result_ = CargoBottomResult{};
    last_cargo_safety_result_ = CargoSafetyResult{};
    has_stable_height_ = false;
    stable_height_ = 0.0F;
    if (!preserve_origin_height) {
        std::lock_guard<std::mutex> lock(hook_load_state_mutex_);
        pending_origin_height_valid_ = false;
        pending_origin_height_m_ = 0.0F;
    }
}

void NdtSlamNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_callback_count_.fetch_add(1, std::memory_order_relaxed);
    if (runtime_diag_.isEnabled()) {
        runtime_diag_.recordCallback(msg->header.stamp.toSec());
    }
    last_pointcloud_time_ = ros::Time::now();
    pointcloud_stale_ = false;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        // 只保留最新帧，丢弃旧帧，避免处理积压
        while (cloud_queue_.size() >= localization_queue_capacity_) {
            cloud_queue_.pop_front();
            queue_overwrite_drop_count_.fetch_add(
                1U, std::memory_order_relaxed);
        }
        cloud_queue_.push_back(
            CloudQueueEntry{msg, std::chrono::steady_clock::now()});
    }
    queue_cv_.notify_one();
}

void NdtSlamNode::processCloudThread() {
    ROS_INFO("Processing thread started");

    // 统计变量
    int total_frames = 0;
    int success_frames = 0;
    ros::Time last_log_time = ros::Time::now();
    Sophus::SE3d diag_previous_raw_ndt_pose;
    Sophus::SE3d diag_previous_published_pose;
    ros::Time diag_previous_published_stamp;
    bool diag_have_previous_raw_ndt_pose = false;
    bool diag_have_previous_published_pose = false;

    using DiagClock = std::chrono::steady_clock;
    const auto elapsedMs = [](const DiagClock::time_point& begin) {
        return std::chrono::duration<double, std::milli>(
            DiagClock::now() - begin).count();
    };

    while (ros::ok() && !shutdown_) {
        sensor_msgs::PointCloud2::ConstPtr msg;
        double diag_queue_age_ms = 0.0;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() { return !cloud_queue_.empty() || shutdown_; });
            if (shutdown_) break;
            CloudQueueEntry entry = std::move(cloud_queue_.front());
            cloud_queue_.pop_front();
            msg = std::move(entry.message);
            diag_queue_age_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - entry.enqueued_at).count();
        }

        if (!msg) continue;
        cloud_dequeue_count_.fetch_add(1, std::memory_order_relaxed);

        total_frames++;
        auto start_time = std::chrono::steady_clock::now();
        last_ndt_converged_ = false;
        RuntimeStageTimes diag_stage;
        const int diag_raw_points =
            static_cast<int>(msg->width * msg->height);
        Sophus::SE3d diag_initial_guess_pose = current_pose_;
        Sophus::SE3d diag_raw_ndt_pose = current_pose_;
        Sophus::SE3d diag_ekf_pose = current_pose_;
        Sophus::SE3d diag_output_pose = current_pose_;
        bool diag_have_initial_guess = false;
        bool diag_have_raw_ndt_pose = false;
        bool diag_have_output_pose = false;
        const int diag_set_input_target_count_before = setInputTarget_count_;
        bool diag_map_commit_allowed = false;
        bool diag_motion_gate_blocked = false;
        std::string diag_map_commit_reason = "registration_not_published";
        double diag_transformation_probability = 0.0;
        double diag_raw_ndt_step_from_previous = 0.0;
        double diag_output_dx = 0.0;
        double diag_output_dy = 0.0;
        double diag_output_step = 0.0;
        double diag_output_yaw_step_deg = 0.0;
        double diag_output_speed_mps = 0.0;

        // 保存消息时间戳，供 publishCurrentCloud 使用
        last_stamp_ = msg->header.stamp;

        // ========== 阶段 1：解析点云 ==========
        const auto ros_to_pcl_start = DiagClock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);
        diag_stage.ros_to_pcl_ms = elapsedMs(ros_to_pcl_start);

        if (input_cloud->empty()) {
            empty_cloud_skip_count_.fetch_add(1, std::memory_order_relaxed);
            ROS_WARN("Empty pointCloud, skipping");
            continue;
        }

        // ========== P0：重复帧检测（在所有处理之前拦截）==========
        // 使用 stamp + cloud_size + hash 作为帧签名
        uint64_t cloud_hash = 0;
        for (size_t i = 0; i < std::min(input_cloud->size(), size_t(100)); ++i) {
            const auto& p = input_cloud->points[i];
            cloud_hash ^= std::hash<float>{}(p.x) + 0x9e3779b9 + (cloud_hash << 6) + (cloud_hash >> 2);
            cloud_hash ^= std::hash<float>{}(p.y) + 0x9e3779b9 + (cloud_hash << 6) + (cloud_hash >> 2);
            cloud_hash ^= std::hash<float>{}(p.z) + 0x9e3779b9 + (cloud_hash << 6) + (cloud_hash >> 2);
        }

        // 检查是否与上一帧相同
        if (msg->header.stamp == last_processed_frame_stamp_ &&
            input_cloud->size() == last_processed_frame_size_ &&
            cloud_hash == last_processed_frame_hash_) {
            duplicate_frame_skip_count_++;
            duplicate_cloud_skip_count_.fetch_add(1, std::memory_order_relaxed);
            ROS_WARN_THROTTLE(1.0,
                "[FrameSkipAll] reason=duplicate_frame stamp=%.3f cloud_size=%zu hash=%lu skipped=%d",
                msg->header.stamp.toSec(),
                input_cloud->size(),
                cloud_hash,
                duplicate_frame_skip_count_);
            continue;  // 跳过本帧，不执行任何后续处理
        }

        // Preserve the previous processed sensor stamp before updating the
        // duplicate-frame signature.  All prediction/crop/gating in this
        // frame must use sensor time, not NDT wall-clock runtime.
        double sensor_dt = 0.10;
        if (!last_processed_frame_stamp_.isZero()) {
            sensor_dt = (msg->header.stamp - last_processed_frame_stamp_).toSec();
            if (!std::isfinite(sensor_dt) || sensor_dt <= 0.0 || sensor_dt > 1.0) {
                invalid_sensor_dt_count_.fetch_add(1, std::memory_order_relaxed);
                ROS_WARN_THROTTLE(2.0,
                    "[SensorTime] invalid_dt=%.6f current=%.6f previous=%.6f fallback=0.100",
                    sensor_dt,
                    msg->header.stamp.toSec(),
                    last_processed_frame_stamp_.toSec());
                sensor_dt = 0.10;
            }
        }
        last_sensor_dt_ = sensor_dt;

        // 更新帧签名
        last_processed_frame_stamp_ = msg->header.stamp;
        last_processed_frame_size_ = input_cloud->size();
        last_processed_frame_hash_ = cloud_hash;
        const uint64_t processing_frame_index = ++runtime_frame_index_;

        // ========== 阶段 1.5：近场过滤（去除起重机抓臂、吊具等固定结构）==========
        const auto near_filter_start = DiagClock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr near_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr near_removed(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& p : input_cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            double xy_dist = std::sqrt(p.x * p.x + p.y * p.y);
            if (xy_dist < near_field_radius_ && p.z > near_field_z_min_) {
                near_removed->push_back(p);  // 保存被删除的点用于可视化
            } else {
                near_filtered->push_back(p);
            }
        }
        input_cloud = near_filtered;

        // 每 50 帧发布一次被删除的点云（用于 RViz 可视化抓臂/吊具）
        static int near_field_log_count = 0;
        near_field_log_count++;
        if (near_field_log_count % 50 == 1 && !near_removed->empty()) {
            sensor_msgs::PointCloud2 removed_msg;
            pcl::toROSMsg(*near_removed, removed_msg);
            removed_msg.header.stamp = msg->header.stamp;
            removed_msg.header.frame_id = "base_link";
            near_field_removed_pub_.publish(removed_msg);

            float removed_ratio = 100.0f * near_removed->size() / (near_removed->size() + near_filtered->size());
            ROS_DEBUG("[NearFieldFilter] input=%lu removed=%lu kept=%lu ratio=%.1f%%",
                     near_removed->size() + near_filtered->size(),
                     near_removed->size(), near_filtered->size(), removed_ratio);
        }

        // ========== 阶段 2：预处理（范围过滤 + 降采样）==========
        diag_stage.near_filter_ms = elapsedMs(near_filter_start);
        const auto hook_prepare_start = DiagClock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr hook_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& p : input_cloud->points) {
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                double range = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
                if (range > 0.5 && range < 30.0) {
                    filtered_cloud->push_back(p);
                    hook_input_cloud->push_back(p);
                }
            }
        }

        // 为 HookFixedCargoDetector 创建 0.05m 细节点云（用于吊货检测）
        if (hook_input_cloud->size() > 0) {
            pcl::VoxelGrid<pcl::PointXYZ> hook_vf;
            hook_vf.setInputCloud(hook_input_cloud);
            hook_vf.setLeafSize(0.05f, 0.05f, 0.05f);
            pcl::PointCloud<pcl::PointXYZ>::Ptr hook_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            hook_vf.filter(*hook_downsampled);
            hook_input_cloud = hook_downsampled;
            ROS_DEBUG_THROTTLE(1.0, "[HookInput] source=fine_base_before_ndt_voxel frame=base_link raw_before_voxel=%zu hook_input=%zu leaf=0.05",
                              filtered_cloud->size(), hook_input_cloud->size());
        }

        // ========== HookFixedCargoDetector（每帧都执行，在 NDT 和 MapCommit 之前）==========
        diag_stage.hook_prepare_ms = elapsedMs(hook_prepare_start);
        const auto cargo_detect_start = DiagClock::now();
        ROS_DEBUG_THROTTLE(2.0,
            "[HookFrame] frame=%d before_detect hook_input=%zu longterm=%d enabled=%d",
            total_frames,
            hook_input_cloud ? hook_input_cloud->size() : 0,
            longterm_mapping_enabled_ ? 1 : 0,
            hook_fixed_config_.enabled ? 1 : 0);

        const HookLoadSnapshot hook_load = currentHookLoadSnapshot();
        if (hook_load.state != last_processed_hook_load_state_) {
            const bool entering_loaded = hook_load.valid &&
                hook_load.state ==
                    lidar_slam2_msgs::HookLoadState::STATE_LOADED;
            resetCargoForHookState(entering_loaded);
            last_processed_hook_load_state_ = hook_load.state;
            ROS_INFO("[HookCargoLifecycle] state=%u valid=%d reason=%s",
                     static_cast<unsigned int>(hook_load.state),
                     hook_load.valid ? 1 : 0,
                     hook_load.reason.c_str());
        }
        const bool hook_allows_tracking = hook_load.valid &&
            hook_load.state ==
                lidar_slam2_msgs::HookLoadState::STATE_LOADED;
        const bool hook_is_empty = hook_load.valid &&
            hook_load.state ==
                lidar_slam2_msgs::HookLoadState::STATE_EMPTY;

        // Stamp guard：防止重复处理同一帧 hook cloud
        bool skip_hook_this_frame = false;
        if (hook_input_cloud && !hook_input_cloud->empty()) {
            uint64_t hook_hash = computeCloudHash(hook_input_cloud);
            if (msg->header.stamp == hook_lock_.last_hook_processed_stamp &&
                hook_hash == hook_lock_.last_hook_processed_hash) {
                ROS_WARN_THROTTLE(1.0,
                    "[HookFrameSkip] reason=duplicate_hook_cloud stamp=%.3f points=%zu hash=%lu",
                    msg->header.stamp.toSec(),
                    hook_input_cloud->size(),
                    hook_hash);
                skip_hook_this_frame = true;
            } else {
                hook_lock_.last_hook_processed_stamp = msg->header.stamp;
                hook_lock_.last_hook_processed_hash = hook_hash;
            }
        }

        // OdomAnchorBox 检测（降频执行）
        if (!skip_hook_this_frame && odom_anchor_config_.enabled && hook_input_cloud && !hook_input_cloud->empty()) {
            if (shouldRunOdomAnchorDetect(msg->header.stamp)) {
                hook_fixed_cargo_ = detectCargoAroundOdomAnchor(hook_input_cloud, msg->header.stamp);
                hook_fixed_bottom_ = estimateCargoBottom(hook_fixed_cargo_);
                if (hook_allows_tracking) {
                    updateHookCargoLock(
                        hook_fixed_cargo_, hook_fixed_bottom_,
                        msg->header.stamp);
                } else {
                    hook_observation_associated_current_ = false;
                    hook_observation_association_stamp_ = msg->header.stamp;
                    if (hook_is_empty && hook_fixed_bottom_.valid &&
                        hook_fixed_bottom_.source == "points_visible_side") {
                        recordEmptyHookOriginHeight(hook_fixed_bottom_.height);
                    }
                }

                // 发布 selected_core_points（默认关闭）
                if (odom_anchor_config_.publish_selected_core_points && hook_fixed_cargo_.valid) {
                    publishSelectedCorePoints(hook_fixed_cargo_, msg->header.stamp);
                }

                last_anchor_detect_stamp_ = msg->header.stamp;

                // OdomAnchorSummary 日志：debug_odom_anchor 开启时输出
                if (debug_cfg_.debug_odom_anchor && (msg->header.stamp - last_anchor_summary_stamp_).toSec() >= odom_anchor_config_.summary_log_period) {
                    ROS_INFO("[OdomAnchorSummary] locked=%d center=(%.2f,%.2f) size=(%.2f,%.2f,%.2f) z=[%.2f,%.2f] points=%zu debug_points=%d",
                             (cargo_state_.state == CargoState::LOCKED || cargo_state_.state == CargoState::LOST) ? 1 : 0,
                             cargo_state_.center_base.x(), cargo_state_.center_base.y(),
                             cargo_state_.size.x(), cargo_state_.size.y(), cargo_state_.size.z(),
                             cargo_state_.bottom_z, cargo_state_.top_z,
                             hook_fixed_cargo_.core_points_base ? hook_fixed_cargo_.core_points_base->size() : 0,
                             odom_anchor_config_.publish_debug_points ? 1 : 0);
                    last_anchor_summary_stamp_ = msg->header.stamp;
                }
            }

            // marker 发布（降频执行）
            if (false && shouldPublishOdomAnchorMarker(msg->header.stamp)) {
                if (cargo_state_.state == CargoState::LOCKED ||
                    cargo_state_.state == CargoState::LOST) {
                    publishPayloadTrackInfoFromOdomAnchorBox(msg->header.stamp);

                    // Cargo Warning 计算和发布（使用 CargoState）
                    if (odom_anchor_config_.cargo_warning.enabled &&
                        cargo_state_.valid_geometry && cargo_state_.valid_height) {
                        const auto cargo_warning_start = DiagClock::now();
                        CargoWarningData warning = computeCargoWarning(
                            hook_input_cloud,
                            cargo_state_.center_base,
                            cargo_state_.size,
                            cargo_state_.bottom_z,
                            cargo_state_.bottom_unc,
                            msg->header.stamp);

                        publishCargoWarning(warning, msg->header.stamp);
                        publishCargoWarningMarkers(
                            cargo_state_.center_base,
                            cargo_state_.size,
                            warning,
                            msg->header.stamp);
                        diag_stage.cargo_warning_ms +=
                            elapsedMs(cargo_warning_start);
                    }
                } else {
                    publishPayloadTrackInfoInvalid("not_locked");
                }
                last_anchor_marker_stamp_ = msg->header.stamp;
            }
        } else if (!skip_hook_this_frame) {
            hook_fixed_cargo_.valid = false;
            hook_fixed_bottom_.valid = false;
            if (hook_allows_tracking) {
                updateHookCargoLock(
                    hook_fixed_cargo_, hook_fixed_bottom_, msg->header.stamp);
            } else {
                hook_observation_associated_current_ = false;
                hook_observation_association_stamp_ = msg->header.stamp;
            }
        }

        // 体素降采样（0.2m，比 merger 的 0.15m 略粗，实现有效降采样）
        diag_stage.cargo_detect_ms = elapsedMs(cargo_detect_start);
        const auto slam_voxel_start = DiagClock::now();
        size_t pre_voxel_size = filtered_cloud->size();
        if (filtered_cloud->size() > 5000) {
            pcl::VoxelGrid<pcl::PointXYZ> vf;
            vf.setInputCloud(filtered_cloud);
            vf.setLeafSize(0.2, 0.2, 0.2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            vf.filter(*downsampled);
            filtered_cloud = downsampled;
            ROS_DEBUG("SLAM voxel: %lu -> %lu", pre_voxel_size, filtered_cloud->size());
        }
        diag_stage.slam_voxel_ms = elapsedMs(slam_voxel_start);

        if (filtered_cloud->size() < 100) {
            too_few_points_skip_count_.fetch_add(
                1U, std::memory_order_relaxed);
            ROS_WARN("Too few points after filter: %lu", filtered_cloud->size());
            continue;
        }
        if (runtime_diag_.isEnabled()) {
            // "processed" means a unique frame with enough valid points to
            // enter registration. Successful odometry remains a separate
            // odom_publish_count_; invalid/too-small inputs are skip counters.
            runtime_diag_.recordProcessed(msg->header.stamp.toSec());
        }

        // ========== 阶段 3：特征提取（网格局部地面分割 + 非地面点加权）==========
        const auto ground_split_start = DiagClock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr feature_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 使用 XY 网格局部地面模型分割（处理倾斜地面和局部高度变化）
        separateGroundByGrid(*filtered_cloud, *ground_cloud, *feature_cloud);
        diag_stage.ground_split_ms = elapsedMs(ground_split_start);

        // ========== 阶段 3.5：BasePayloadChannelFilter（base_link 下吊货候选筛选）==========
        const auto channel_filter_start = DiagClock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr safe_objects(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr payload_candidates(new pcl::PointCloud<pcl::PointXYZ>);

        if (channel_filter_config_.enabled) {
            // 获取局部地面模型
            auto ground_model = channel_filter_.getConfig().enabled ?
                std::map<CellKey, float>() : std::map<CellKey, float>();
            // 从 separateGroundByGrid 获取地面模型（需要重构，暂时用空模型）
            // channel filter 内部会自行处理 HAG 计算
            ChannelFilterResult ch_result = channel_filter_.filter(feature_cloud, ground_model);

            safe_objects = ch_result.safe_objects;
            payload_candidates = ch_result.payload_candidates;

            // 发布 debug 话题（每 10 帧一次）
            static int ch_debug_count = 0;
            ch_debug_count++;
            if (ch_debug_count % 10 == 1) {
                ROS_DEBUG("[PayloadChannel] channel_points=%d, candidate_clusters=%d, "
                         "candidate_points=%d, safe_points=%d",
                         ch_result.channel_points, ch_result.candidate_clusters,
                         ch_result.candidate_points, ch_result.safe_points);

                // 发布通道内所有点
                if (!ch_result.channel_all_points->empty()) {
                    sensor_msgs::PointCloud2 ch_msg;
                    pcl::toROSMsg(*ch_result.channel_all_points, ch_msg);
                    ch_msg.header.stamp = msg->header.stamp;
                    ch_msg.header.frame_id = "base_link";
                    payload_channel_pub_.publish(ch_msg);
                }

                // 发布候选吊货点
                if (!payload_candidates->empty()) {
                    sensor_msgs::PointCloud2 cand_msg;
                    pcl::toROSMsg(*payload_candidates, cand_msg);
                    cand_msg.header.stamp = msg->header.stamp;
                    cand_msg.header.frame_id = "base_link";
                    payload_candidate_pub_.publish(cand_msg);
                }

                // 发布安全物体点
                if (!safe_objects->empty()) {
                    sensor_msgs::PointCloud2 safe_msg;
                    pcl::toROSMsg(*safe_objects, safe_msg);
                    safe_msg.header.stamp = msg->header.stamp;
                    safe_msg.header.frame_id = "base_link";
                    safe_objects_pub_.publish(safe_msg);
                }
            }
        } else {
            // 通道过滤未启用，所有 objects 都是 safe 的
            safe_objects = feature_cloud;
        }

        // ========== 阶段 3.6：HumanObjectDynamicFilter（人体动态过滤）==========
        diag_stage.channel_filter_ms = elapsedMs(channel_filter_start);
        const auto human_filter_start = DiagClock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr human_safe_objects(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr human_candidates(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr human_dynamic(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr human_pending(new pcl::PointCloud<pcl::PointXYZ>);

        if (human_filter_config_.enabled) {
            // 获取 T_map_base（当前位姿）
            Eigen::Matrix4d T_map_base = current_pose_.matrix();

            // 获取时间戳
            double timestamp = msg->header.stamp.toSec();

            // 处理人体过滤
            human_filter_.processFrame(safe_objects, T_map_base, timestamp,
                                       human_safe_objects, human_candidates,
                                       human_dynamic, human_pending);

            // 发布调试话题（每 10 帧一次）
            static int hf_debug_count = 0;
            hf_debug_count++;
            if (hf_debug_count % 10 == 1) {
                ROS_DEBUG("[HumanFilter] input=%lu, safe=%lu, candidate=%lu, dynamic=%lu, pending=%lu",
                         safe_objects->size(), human_safe_objects->size(),
                         human_candidates->size(), human_dynamic->size(), human_pending->size());

                if (!human_candidates->empty()) {
                    sensor_msgs::PointCloud2 cand_msg;
                    pcl::toROSMsg(*human_candidates, cand_msg);
                    cand_msg.header.stamp = msg->header.stamp;
                    cand_msg.header.frame_id = "base_link";
                    human_candidate_pub_.publish(cand_msg);
                }

                if (!human_dynamic->empty()) {
                    sensor_msgs::PointCloud2 dyn_msg;
                    pcl::toROSMsg(*human_dynamic, dyn_msg);
                    dyn_msg.header.stamp = msg->header.stamp;
                    dyn_msg.header.frame_id = "map";
                    human_dynamic_pub_.publish(dyn_msg);
                }

                if (!human_pending->empty()) {
                    sensor_msgs::PointCloud2 pend_msg;
                    pcl::toROSMsg(*human_pending, pend_msg);
                    pend_msg.header.stamp = msg->header.stamp;
                    pend_msg.header.frame_id = "map";
                    human_pending_pub_.publish(pend_msg);
                }
            }
        } else {
            // 人体过滤未启用，所有 safe_objects 都是安全的
            human_safe_objects = safe_objects;
        }

        // v8-stable-r3: 构建配准用点云（NDT 输入减负）
        // 使用 buildRegistrationCloud 替代原来的 objects x4 + ground full
        diag_stage.human_filter_ms = elapsedMs(human_filter_start);
        const auto registration_build_start = DiagClock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr registration_cloud =
            buildRegistrationCloud(human_safe_objects, ground_cloud);

        // ========== Hook locked box 剔除（NDT 输入）==========
        // 使用 CargoState 统一状态
        if (isHookCargoRemovalEnabled() &&
            cargo_state_.state == CargoState::LOCKED &&
            cargo_state_.valid_geometry && cargo_state_.valid_height) {
            // 使用 CargoState 的中心和尺寸
            Eigen::Vector3f center = cargo_state_.center_base;
            Eigen::Vector3f size = cargo_state_.size;
            float zmin = cargo_state_.bottom_z - 0.10f;
            float zmax = cargo_state_.top_z + 0.10f;

            // 构建 Hook locked box
            Eigen::Vector3f hook_bbox_min(center.x() - size.x() * 0.5f, center.y() - size.y() * 0.5f, zmin);
            Eigen::Vector3f hook_bbox_max(center.x() + size.x() * 0.5f, center.y() + size.y() * 0.5f, zmax);

            // 从 registration_cloud 中剔除 Hook locked box 内的点
            pcl::PointCloud<pcl::PointXYZ>::Ptr registration_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            size_t hook_removed_count = 0;

            for (const auto& p : registration_cloud->points) {
                if (p.x >= hook_bbox_min.x() && p.x <= hook_bbox_max.x() &&
                    p.y >= hook_bbox_min.y() && p.y <= hook_bbox_max.y() &&
                    p.z >= hook_bbox_min.z() && p.z <= hook_bbox_max.z()) {
                    hook_removed_count++;
                } else {
                    registration_cloud_filtered->push_back(p);
                }
            }

            if (hook_removed_count > 0) {
                registration_cloud = registration_cloud_filtered;
                if (debug_cfg_.debug_registration_removal) {
                    ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec, "[RegistrationCargoRemoval] enabled=1 before=%zu removed=%zu after=%zu center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) z=[%.2f,%.2f]",
                                     registration_cloud->size() + hook_removed_count, hook_removed_count, registration_cloud->size(),
                                     center.x(), center.y(), center.z(),
                                     size.x(), size.y(), size.z(),
                                     cargo_state_.bottom_z, cargo_state_.top_z);
                }
            }
        } else if (!isHookCargoRemovalEnabled() && debug_cfg_.debug_hook_removal) {
            ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec,
                "[HookCargoRemoval] enabled=0 reason=config_disabled");
        }

        // ========== 地面法向量诊断 ==========
        // 初始化时和每 100 帧输出一次，用于检测外参 roll/pitch 误差
        diag_stage.registration_build_ms = elapsedMs(registration_build_start);
        static int ground_diag_count = 0;
        ground_diag_count++;
        if ((ground_diag_count <= 3 || ground_diag_count % 100 == 0) && ground_cloud->size() > 100) {
            // 计算地面点质心
            Eigen::Vector3d centroid(0, 0, 0);
            for (const auto& p : ground_cloud->points) {
                centroid += Eigen::Vector3d(p.x, p.y, p.z);
            }
            centroid /= ground_cloud->size();

            // 构建协方差矩阵
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            for (const auto& p : ground_cloud->points) {
                Eigen::Vector3d d(p.x - centroid.x(), p.y - centroid.y(), p.z - centroid.z());
                cov += d * d.transpose();
            }
            cov /= ground_cloud->size();

            // SVD 分解，最小奇异值对应的向量即为法向量
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU);
            Eigen::Vector3d normal = svd.matrixU().col(2);  // 最小奇异值方向

            // 确保法向量朝上
            if (normal.z() < 0) normal = -normal;

            // 计算相对于 Z 轴 (0,0,1) 的 roll/pitch 误差
            double roll_error = std::atan2(normal.y(), normal.z()) * 180.0 / M_PI;
            double pitch_error = std::atan2(-normal.x(), normal.z()) * 180.0 / M_PI;

            // 每格子局部厚度统计（比全局 z 范围更有意义）
            struct CellKey { int x, y; bool operator<(const CellKey& o) const { return x<o.x||(x==o.x&&y<o.y); } };
            std::map<CellKey, std::vector<float>> cell_z;
            for (const auto& p : ground_cloud->points) {
                CellKey k{(int)std::floor(p.x/grid_cell_size_), (int)std::floor(p.y/grid_cell_size_)};
                cell_z[k].push_back(p.z);
            }
            float local_thickness_sum = 0, local_thickness_max = 0;
            int valid_cells = 0;
            for (auto& [k, zv] : cell_z) {
                if (zv.size() >= 3) {
                    auto [min_it, max_it] = std::minmax_element(zv.begin(), zv.end());
                    float lt = *max_it - *min_it;
                    local_thickness_sum += lt;
                    local_thickness_max = std::max(local_thickness_max, lt);
                    valid_cells++;
                }
            }
            float local_thickness_avg = valid_cells > 0 ? local_thickness_sum / valid_cells : 0;
            float obj_ratio = (float)feature_cloud->size() / (ground_cloud->size() + feature_cloud->size()) * 100.0f;

            // Objects 高度分布统计（相对于局部地面）
            int obj_low = 0, obj_mid = 0, obj_high = 0;
            for (const auto& p : feature_cloud->points) {
                // 用最近格子的局部地面高度
                CellKey ck{(int)std::floor(p.x/grid_cell_size_), (int)std::floor(p.y/grid_cell_size_)};
                auto it = cell_z.find(ck);
                float local_gz = 0;
                if (it != cell_z.end() && !it->second.empty()) {
                    auto min_it = std::min_element(it->second.begin(), it->second.end());
                    local_gz = *min_it;
                }
                float h = p.z - local_gz;
                if (h < 0.8f) obj_low++;
                else if (h < 1.5f) obj_mid++;
                else obj_high++;
            }
            int obj_total = obj_low + obj_mid + obj_high;
            float pct_low = obj_total > 0 ? 100.0f * obj_low / obj_total : 0;
            float pct_mid = obj_total > 0 ? 100.0f * obj_mid / obj_total : 0;
            float pct_high = obj_total > 0 ? 100.0f * obj_high / obj_total : 0;

            ROS_DEBUG("[GroundDiag] roll=%.2f° pitch=%.2f° | "
                     "local_thickness: avg=%.3fm max=%.3fm | "
                     "obj_ratio=%.1f%% | obj_height: low=%.0f%% mid=%.0f%% high=%.0f%% | "
                     "grid=%.1fm hag=%.2fm | ground=%lu obj=%lu cells=%d",
                     roll_error, pitch_error,
                     local_thickness_avg, local_thickness_max,
                     obj_ratio,
                     pct_low, pct_mid, pct_high,
                     grid_cell_size_, height_above_ground_,
                     ground_cloud->size(), feature_cloud->size(), valid_cells);

            if (std::abs(roll_error) > 2.0 || std::abs(pitch_error) > 2.0) {
                ROS_WARN("[GroundDiag] Large tilt detected! roll=%.2f° pitch=%.2f° — "
                         "check LiDAR extrinsic or base_link orientation",
                         roll_error, pitch_error);
            }
            if (local_thickness_avg > 0.5f) {
                ROS_WARN("[GroundDiag] Local thickness avg=%.3fm > 0.5m — "
                         "ground model may need refinement", local_thickness_avg);
            }
        }

        // ========== 阶段 4：初始化 ==========
        bool should_init = false;
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!initialized_) {
                should_init = true;
                initialized_ = true;
            }
        }

        if (should_init) {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            current_pose_ = Sophus::SE3d();
            ROS_INFO("SLAM initialized: total=%lu, feature=%lu, ground=%lu, reg=%lu",
                     filtered_cloud->size(), feature_cloud->size(), ground_cloud->size(), registration_cloud->size());
        }

        // ========== 阶段 5：NDT_OMP 配准 ==========
        // Apply only a twice-confirmed asynchronous recovery result before the
        // current NDT prediction so this frame immediately refines it.
        consumeRelocalizationResult(processing_frame_index, msg->header.stamp);

        Sophus::SE3d new_pose = current_pose_;
        bool registration_success = false;
        bool ndt_attempted_this_frame = false;
        static Sophus::SE3d last_local_map_pose = Sophus::SE3d();
        static int frames_since_last_update = 0;

        try {
            if (local_map_->empty() || local_map_->size() < 500) {
                // 累积阶段
                *local_map_ += *registration_cloud;
                ++local_map_version_;
                registration_success = true;

                if (local_map_->size() >= 500 && local_map_->size() < 600) {
                    ROS_INFO("Local map ready: %lu points", local_map_->size());
                    last_local_map_pose = current_pose_;
                }
            } else {
                // 配准阶段（带时间预算）

                // Bind one immutable target for this alignment.  The
                // configured minimum applies after every crop/downsample
                // stage; an undersized target falls back immediately.
                const auto target_bind_start = DiagClock::now();
                pcl::PointCloud<pcl::PointXYZ>::ConstPtr selected_target = local_map_;
                std::string selected_source = "bootstrap_local_map";
                std::string selected_reason = "target_not_ready";
                uint64_t selected_version = local_map_version_;

                if (localization_target_use_for_ndt_ && localization_target_enabled_ && localization_target_ready_) {
                    if (crop_enabled_) {
                        Sophus::SE3d predicted_pose = current_pose_;
                        if (crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized()) {
                            predicted_pose = crane_motion_ekf_.predictPoseReadOnly(
                                current_pose_, last_sensor_dt_);
                        }

                        if (updateCroppedCachedTarget(predicted_pose) &&
                            cached_target_valid_ &&
                            static_cast<int>(cached_target_->size()) >=
                                localization_target_min_points_) {
                            selected_target = cached_target_;
                            selected_source = "cropped_localization_target";
                            selected_reason = "ready";
                            selected_version = cached_target_version_;
                        } else {
                            selected_source = "fallback_local_map";
                            selected_reason = last_target_reason_;
                        }
                    } else {
                        std::lock_guard<std::mutex> lock(localization_target_mutex_);
                        if (localization_target_snapshot_ &&
                            static_cast<int>(localization_target_snapshot_->size()) >=
                                localization_target_min_points_) {
                            selected_target = localization_target_snapshot_;
                            selected_source = "localization_target_snapshot";
                            selected_reason = "ready";
                            selected_version = localization_target_snapshot_version_;
                        } else {
                            selected_source = "fallback_local_map";
                            selected_reason = "snapshot_below_min_points";
                        }
                    }
                } else if (!localization_target_enabled_) {
                    selected_source = "local_map";
                    selected_reason = "localization_target_disabled";
                } else if (!localization_target_use_for_ndt_) {
                    selected_source = "local_map";
                    selected_reason = "SHADOW_ONLY";
                }

                bindNdtInputTarget(selected_target, selected_source,
                                   selected_version, selected_reason);
                ndt_->setInputSource(registration_cloud);
                diag_stage.target_bind_ms = elapsedMs(target_bind_start);

                pcl::PointCloud<pcl::PointXYZ> aligned;

                // V3: 使用 EKF 预测作为 initial guess（如果 EKF 已初始化）
                Eigen::Matrix4f initial_guess;
                if (crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized()) {
                    Sophus::SE3d predicted = crane_motion_ekf_.predictPoseReadOnly(
                        current_pose_, last_sensor_dt_);
                    initial_guess = predicted.matrix().cast<float>();
                } else {
                    initial_guess = current_pose_.matrix().cast<float>();
                }
                diag_initial_guess_pose =
                    Sophus::SE3d(initial_guess.cast<double>());
                diag_have_initial_guess = true;

                last_source_points_ = static_cast<int>(registration_cloud->size());

                // 计算 initial_guess 到 current_pose_ 的距离（用于诊断）
                Eigen::Vector3f initial_pos = initial_guess.block<3,1>(0,3);
                Eigen::Vector3f current_pos = current_pose_.matrix().cast<float>().block<3,1>(0,3);
                last_init_dist_ = (initial_pos - current_pos).norm();

                auto ndt_start = std::chrono::steady_clock::now();
                ndt_attempt_count_.fetch_add(1, std::memory_order_relaxed);
                ndt_attempted_this_frame = true;
                ndt_->align(aligned, initial_guess);
                auto ndt_end = std::chrono::steady_clock::now();
                double ndt_time_ms = std::chrono::duration<double, std::milli>(ndt_end - ndt_start).count();
                diag_stage.ndt_ms = ndt_time_ms;
                last_ndt_time_ms_ = ndt_time_ms;
                last_ndt_converged_ = ndt_->hasConverged();
                if (last_ndt_converged_) {
                    ndt_converged_count_.fetch_add(1, std::memory_order_relaxed);
                } else {
                    ndt_nonconverged_count_.fetch_add(1, std::memory_order_relaxed);
                }

                // 计算 raw_step（NDT结果到initial_guess的距离）
                if (last_ndt_converged_) {
                    Eigen::Matrix4f ndt_result = ndt_->getFinalTransformation();
                    Eigen::Vector3f ndt_pos = ndt_result.block<3,1>(0,3);
                    last_raw_step_ =
                        (ndt_pos - initial_pos).head<2>().norm();
                }

                // Use the single configured runtime threshold.  The previous
                // implementation had conflicting hard-coded 80/100/300 ms
                // definitions, which made diagnostics impossible to compare.
                static int ndt_warn_count = 0;
                if (crane_motion_ekf_cfg_.slow_frame_guard_enabled &&
                    ndt_time_ms > crane_motion_ekf_cfg_.slow_frame_warn_ms) {
                    ndt_warn_count++;
                    // 静止时不输出，运动时每 100 次输出一次
                    if (ndt_warn_count <= 3 || ndt_warn_count % 100 == 0) {
                        ROS_WARN("[NDT-guard] time=%.1fms (count=%d)", ndt_time_ms, ndt_warn_count);
                    }
                }

                if (last_ndt_converged_) {
                    Eigen::Matrix4f result = ndt_->getFinalTransformation();
                    double fitness_score = ndt_->getFitnessScore();
                    double trans_prob = ndt_->getTransformationProbability();
                    diag_transformation_probability = trans_prob;
                    last_ndt_iterations_ = ndt_->getFinalNumIteration();
                    ROS_DEBUG("NDT: converged, fitness=%.4f, prob=%.6f", fitness_score, trans_prob);
                    if (fitness_score > 5.0) {
                        ROS_WARN("NDT: high fitness score=%.4f, matching quality may be poor", fitness_score);
                    }

                    // NDT 健康监控
                    last_ndt_fitness_ = fitness_score;
                    if (fitness_score > fitness_warning_threshold_) {
                        consecutive_high_fitness_++;
                        if (consecutive_high_fitness_ >= fitness_warning_count_) {
                            ndt_health_bad_ = true;
                            ROS_WARN_THROTTLE(30, "[NDT-Health] BAD: fitness=%.4f for %d frames",
                                              fitness_score, consecutive_high_fitness_);
                        }
                    } else {
                        consecutive_high_fitness_ = 0;
                        ndt_health_bad_ = false;
                    }
                    if (!result.isZero() && !result.hasNaN()) {
                        // 正交化旋转矩阵
                        Eigen::Matrix3d R = result.block<3,3>(0,0).cast<double>();
                        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
                        Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();
                        if (R_ortho.determinant() < 0) {
                            R_ortho.col(0) *= -1;
                        }

                        Eigen::Matrix4d result_ortho = Eigen::Matrix4d::Identity();
                        result_ortho.block<3,3>(0,0) = R_ortho;
                        result_ortho.block<3,1>(0,3) = result.block<3,1>(0,3).cast<double>();

                        new_pose = Sophus::SE3d(result_ortho);
                        diag_raw_ndt_pose = new_pose;
                        diag_have_raw_ndt_pose = true;
                        if (diag_have_previous_raw_ndt_pose) {
                            diag_raw_ndt_step_from_previous =
                                (new_pose.translation().head<2>() -
                                 diag_previous_raw_ndt_pose.translation().head<2>()).norm();
                        }
                        diag_previous_raw_ndt_pose = new_pose;
                        diag_have_previous_raw_ndt_pose = true;

                        // P0-3: 保存 raw pose for MapCommit evidence
                        // Note: This is ONLY used for MapCommit evidence, NOT for odom/TF/runtime_path
                        last_raw_ndt_pose_ = new_pose;
                        has_last_raw_ndt_pose_ = true;

                        // v8-stable-r3: CraneMotionEKF update
                        Sophus::SE3d ekf_pose = new_pose;
                        bool ndt_accepted = true;

                        // V3: 慢帧保护 - 检查是否需要使用 EKF prediction
                        bool use_prediction = false;
                        std::string reject_reason_slow = "NONE";
                        if (crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized()) {
                            // 物理步长保护
                            if (crane_motion_ekf_.isNonPhysicalStep(
                                    last_raw_step_, ndt_time_ms, last_sensor_dt_)) {
                                use_prediction = true;
                                reject_reason_slow = "NONPHYSICAL_NDT_CORRECTION";
                                ROS_WARN_THROTTLE(2.0, "[NDT-guard] nonphysical correction: raw_step=%.3f ndt_ms=%.1f dt=%.3f",
                                                 last_raw_step_, ndt_time_ms, last_sensor_dt_);
                            }
                        }

                        const auto ekf_start = DiagClock::now();
                        if (crane_motion_ekf_enabled_) {
                            if (!crane_motion_ekf_.initialized()) {
                                crane_motion_ekf_.initialize(new_pose, msg->header.stamp);
                                ekf_pose = new_pose;
                            } else if (use_prediction) {
                                // A published prediction must advance EKF x/P/stamp.
                                ekf_pose = crane_motion_ekf_.predictWithoutMeasurement(
                                    new_pose, msg->header.stamp,
                                    reject_reason_slow);
                            } else {
                                ekf_pose = crane_motion_ekf_.updateWithNDT(
                                    new_pose,
                                    fitness_score,
                                    new_pose,
                                    msg->header.stamp,
                                    ndt_time_ms);
                            }

                            ndt_accepted = crane_motion_ekf_.status().ndt_accepted;
                            if (ndt_accepted) {
                                ekf_accept_count_.fetch_add(1, std::memory_order_relaxed);
                            } else {
                                ekf_reject_count_.fetch_add(1, std::memory_order_relaxed);
                            }

                            // EKF 日志：debug_ekf 开启时输出
                            if (debug_cfg_.debug_ekf) {
                                const auto& ekf_status = crane_motion_ekf_.status();
                                ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec,
                                    "[EKF] pred=(%.2f,%.2f) ndt=(%.2f,%.2f) out=(%.2f,%.2f) "
                                    "vel=(%.2f,%.2f) innov=%.3f nis=%.2f step=%.3f/%.3f "
                                    "lat=%.3f tan=%.3f R=%.4f P=%.4f mode=%s accept=%d predict=%d limited=%d reject=%s",
                                    ekf_status.predicted_pos.x(), ekf_status.predicted_pos.y(),
                                    ekf_status.ndt_pos.x(), ekf_status.ndt_pos.y(),
                                    ekf_status.output_pos.x(), ekf_status.output_pos.y(),
                                    ekf_status.velocity.x(), ekf_status.velocity.y(),
                                    ekf_status.innovation_norm,
                                    ekf_status.nis,
                                    ekf_status.output_step,
                                    ekf_status.max_allowed_step,
                                    ekf_status.lateral_error,
                                    ekf_status.tangential_error,
                                    ekf_status.measurement_r,
                                    ekf_status.p_trace,
                                    ekf_status.diagonal_mode ? "DIAG" : "NORMAL",
                                    ekf_status.ndt_accepted ? 1 : 0,
                                    ekf_status.prediction_only ? 1 : 0,
                                    ekf_status.step_limited ? 1 : 0,
                                    ekf_status.reject_reason.c_str());
                            }
                        }

                        // v8-stable-r3: 使用 EKF 输出作为 new_pose
                        diag_stage.ekf_ms += elapsedMs(ekf_start);
                        new_pose = ekf_pose;
                        diag_ekf_pose = ekf_pose;

                        // NDT 健康日志（每秒一次）
                        if (debug_cfg_.debug_ndt_health) {
                            Eigen::Vector3d raw_pos = new_pose.translation();
                            double raw_roll, raw_pitch, raw_yaw;
                            so3ToRpy(new_pose.so3(), raw_roll, raw_pitch, raw_yaw);
                            ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec,
                                              "[NDTHealth] fitness=%.3f, converged=%d, "
                                              "raw_pose=(%.2f, %.2f, %.2f), raw_rpy=(%.1f, %.1f, %.1f)deg",
                                              fitness_score, last_ndt_converged_ ? 1 : 0,
                                              raw_pos.x(), raw_pos.y(), raw_pos.z(),
                                              raw_roll * 180.0 / M_PI, raw_pitch * 180.0 / M_PI,
                                              raw_yaw * 180.0 / M_PI);
                        }

                        // 注意：约束不应用到 new_pose，因为 new_pose 用于：
                        // - local_map 更新
                        // - 下一帧 NDT initial guess
                        // 约束只在发布和 keyframe 存储时应用

                        registration_success = true;

                        // 关键帧策略：需要足够的运动才更新局部地图
                        frames_since_last_update++;
                        Sophus::SE3d delta = last_local_map_pose.inverse() * new_pose;
                        double move_dist = delta.translation().norm();
                        double move_rot = delta.so3().log().norm();

                        // Prediction-only/rejected poses must not feed either
                        // the persistent map or the runtime registration map.
                        if (!ndt_accepted || !relocalization_pose_reliable_) {
                            ROS_DEBUG("[LocalMap] skipped rejected/prediction-only frame");
                        } else if (move_dist > 0.5 || move_rot > 0.08 || frames_since_last_update > 15) {
                            // 用配准点云更新局部地图
                            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
                            pcl::transformPointCloud(
                                *registration_cloud, *transformed,
                                new_pose.matrix().cast<float>());
                            *local_map_ += *transformed;

                            // 清理远处的点（15m 半径，更紧凑的局部地图）
                            Eigen::Vector3d current_pos = new_pose.translation();
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
                            for (const auto& p : local_map_->points) {
                                double dx = p.x - current_pos.x();
                                double dy = p.y - current_pos.y();
                                double dz = p.z - current_pos.z();
                                if (dx*dx + dy*dy + dz*dz < 225.0) {  // 15m 半径
                                    cropped->push_back(p);
                                }
                            }

                            // 体素滤波：增大 leaf size 到 0.3m，与 NDT resolution 匹配
                            if (cropped->size() > 8000) {
                                pcl::VoxelGrid<pcl::PointXYZ> vf;
                                vf.setInputCloud(cropped);
                                vf.setLeafSize(0.3, 0.3, 0.3);
                                pcl::PointCloud<pcl::PointXYZ> filtered_map;
                                vf.filter(filtered_map);
                                *local_map_ = filtered_map;
                            } else {
                                *local_map_ = *cropped;
                            }
                            ++local_map_version_;

                            last_local_map_pose = new_pose;
                            frames_since_last_update = 0;
                        }
                    } else if (crane_motion_ekf_enabled_ &&
                               crane_motion_ekf_.initialized()) {
                        const auto ekf_start = DiagClock::now();
                        ROS_WARN_THROTTLE(
                            1.0,
                            "NDT returned an invalid transformation; advancing EKF prediction");
                        new_pose = crane_motion_ekf_.predictWithoutMeasurement(
                            current_pose_, msg->header.stamp,
                            "NDT_INVALID_TRANSFORM");
                        registration_success = true;
                        ekf_reject_count_.fetch_add(1, std::memory_order_relaxed);
                        diag_stage.ekf_ms += elapsedMs(ekf_start);
                        diag_ekf_pose = new_pose;
                    }
                } else {
                    static int no_converge_count = 0;
                    no_converge_count++;
                    if (no_converge_count <= 5 || no_converge_count % 50 == 0) {
                        ROS_WARN("NDT: not converged (count=%d), advancing EKF prediction", no_converge_count);
                    }
                    last_ndt_fitness_ = std::numeric_limits<double>::infinity();
                    last_raw_step_ = 0.0;
                    if (crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized()) {
                        const auto ekf_start = DiagClock::now();
                        new_pose = crane_motion_ekf_.predictWithoutMeasurement(
                            current_pose_, msg->header.stamp,
                            "NDT_NOT_CONVERGED");
                        registration_success = true;
                        ekf_reject_count_.fetch_add(1, std::memory_order_relaxed);
                        diag_stage.ekf_ms += elapsedMs(ekf_start);
                        diag_ekf_pose = new_pose;
                    }
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("NDT_OMP exception: %s", e.what());
        }

        // ========== 阶段 6：更新位姿 ==========
        // v8-stable-r3-hotfix-minimal: 统一 final_pose 发布链路
        Sophus::SE3d constrained_pose = new_pose;
        if (registration_success && crane_constraint_enabled_) {
            const auto& ekf_status = crane_motion_ekf_.status();
            double speed_xy = ekf_status.velocity.norm();
            // Soft-yaw low-motion behavior is derived from the EKF velocity,
            // never from MotionGate's map-commit state.
            const bool runtime_low_motion =
                speed_xy < motion_gate_moving_min_velocity_;
            constrained_pose = applyCraneOutputConstraint(
                new_pose, runtime_low_motion, speed_xy);
        }

        if (registration_success) {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            current_pose_ = constrained_pose;  // 发布约束后的 pose
        }

        if (ndt_attempted_this_frame) {
            const bool frame_ndt_healthy =
                last_ndt_converged_ && std::isfinite(last_ndt_fitness_) &&
                (!crane_motion_ekf_enabled_ ||
                 crane_motion_ekf_.status().ndt_accepted);
            updateRelocalization(processing_frame_index, msg->header.stamp,
                                 registration_cloud, frame_ndt_healthy);
        }

        // ========== 阶段 7：发布结果（用完整点云建图）==========
        if (registration_success) {
            // Keep the complete frame context on the sensor timestamp.  The
            // callback already rejects duplicate sensor frames.
            ros::Time publish_time = msg->header.stamp.isZero()
                ? ros::Time::now()
                : msg->header.stamp;
            const bool runtime_ndt_accepted =
                !crane_motion_ekf_enabled_ ||
                crane_motion_ekf_.status().ndt_accepted;

            // v8-stable-r3-hotfix-minimal: selectPublishedPose 已透传
            const auto publish_odom_start = DiagClock::now();
            Sophus::SE3d final_pose = selectPublishedPose(constrained_pose, publish_time);
            publishOdometry(publish_time, msg->header.frame_id, final_pose);
            odom_publish_count_.fetch_add(1, std::memory_order_relaxed);

            // runtime_path 只是调试显示，不参与算法
            if (debug_cfg_.publish_runtime_path) {
                publishRuntimePath(final_pose, publish_time);
            }
            diag_stage.publish_odom_ms = elapsedMs(publish_odom_start);
            diag_output_pose = final_pose;
            diag_have_output_pose = true;

            if (diag_have_previous_published_pose) {
                const Eigen::Vector3d delta =
                    final_pose.translation() -
                    diag_previous_published_pose.translation();
                diag_output_dx = delta.x();
                diag_output_dy = delta.y();
                diag_output_step = delta.head<2>().norm();

                double prev_roll = 0.0, prev_pitch = 0.0, prev_yaw = 0.0;
                double out_roll = 0.0, out_pitch = 0.0, out_yaw = 0.0;
                so3ToRpy(diag_previous_published_pose.so3(),
                         prev_roll, prev_pitch, prev_yaw);
                so3ToRpy(final_pose.so3(), out_roll, out_pitch, out_yaw);
                const double yaw_delta =
                    std::atan2(std::sin(out_yaw - prev_yaw),
                               std::cos(out_yaw - prev_yaw));
                diag_output_yaw_step_deg = yaw_delta * 180.0 / M_PI;

                const double output_dt =
                    (publish_time - diag_previous_published_stamp).toSec();
                if (std::isfinite(output_dt) && output_dt > 1e-6) {
                    diag_output_speed_mps = diag_output_step / output_dt;
                }
            }
            diag_previous_published_pose = final_pose;
            diag_previous_published_stamp = publish_time;
            diag_have_previous_published_pose = true;

            // Formal cargo chain: same-stamp tracked points + final odometry
            // pose -> fused bottom -> conservative per-cluster safety status.
            // The heartbeat node is the only producer of the PLC alarm topic.
            if (relocalization_pose_reliable_) {
                updateAndPublishCargoSafetyPipeline(
                    feature_cloud, filtered_cloud, final_pose, publish_time);
                relocalization_invalid_safety_published_ = false;
            } else if (!relocalization_invalid_safety_published_) {
                publishRelocalizationSafetyInvalid(
                    publish_time, "localization_degraded");
                relocalization_invalid_safety_published_ = true;
            }

            // ICP never changes odom/TF. When explicitly enabled, a bounded
            // asynchronous refinement may assist only the current map cloud.
            // Optional ICP refinement is strictly gated before any cloud copy,
            // job creation or thread activity. A result is frame/map scoped and
            // is invalidated after one consume attempt.
            const auto icp_prepare_start = DiagClock::now();
            Sophus::SE3d map_pose = constrained_pose;
            if (icp_refine_cfg_.enabled &&
                icp_refine_cfg_.run_after_ndt &&
                runtime_ndt_accepted) {
                Sophus::SE3d frame_refined_pose;
                if (consumeIcpRefineResult(
                        processing_frame_index, publish_time,
                        local_map_version_, frame_refined_pose)) {
                    map_pose = applyCraneMotionConstraint(
                        frame_refined_pose, "icp_frame_scoped");
                }

                const pcl::PointCloud<pcl::PointXYZ>::Ptr& icp_input =
                    icp_refine_cfg_.use_objects_only
                        ? human_safe_objects
                        : registration_cloud;

                if (!icp_running_.load(std::memory_order_acquire) &&
                    icp_input &&
                    static_cast<int>(icp_input->size()) >=
                        icp_refine_cfg_.min_object_points &&
                    local_map_ && local_map_->size() > 500) {
                    if (icp_thread_.joinable()) {
                        icp_thread_.join();
                    }

                    IcpRefineJob job;
                    job.frame_index = processing_frame_index;
                    job.stamp = publish_time;
                    job.local_map_version = local_map_version_;
                    job.ndt_pose = new_pose;
                    job.source.reset(
                        new pcl::PointCloud<pcl::PointXYZ>(*icp_input));
                    job.target.reset(
                        new pcl::PointCloud<pcl::PointXYZ>(*local_map_));
                    icp_cloud_copy_count_.fetch_add(
                        2, std::memory_order_relaxed);
                    startIcpRefineJob(std::move(job));
                }
            }
            diag_stage.icp_prepare_ms = elapsedMs(icp_prepare_start);

            ROS_INFO_THROTTLE(
                1.0,
                "[ICP_FLOW] enabled=%d running=%d copies=%llu jobs=%llu "
                "threads=%llu used=%llu stale_drop=%llu",
                icp_refine_cfg_.enabled ? 1 : 0,
                icp_running_.load(std::memory_order_relaxed) ? 1 : 0,
                static_cast<unsigned long long>(
                    icp_cloud_copy_count_.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    icp_job_count_.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    icp_thread_count_.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    icp_result_use_count_.load(std::memory_order_relaxed)),
                static_cast<unsigned long long>(
                    icp_stale_drop_count_.load(std::memory_order_relaxed)));

            const auto current_cloud_start = DiagClock::now();
            addFrameToMap(filtered_cloud, map_pose, publish_time);
            diag_stage.current_cloud_ms = elapsedMs(current_cloud_start);

            // v8-stable-r3: MapCommit 只允许在 ndt_accepted=true 且 fitness 合格时执行
            // EKF prediction-only 帧可以发布 TF/odom，但不能写地图
            const bool ndt_accepted_for_commit = runtime_ndt_accepted;

            // Prediction-only poses are valid runtime outputs but can never
            // become map evidence.  Keep this invariant even if a legacy
            // configuration attempts to relax the old flag.
            const bool commit_accept_ok = ndt_accepted_for_commit;
            const bool commit_fitness_ok = std::isfinite(last_ndt_fitness_) &&
                last_ndt_fitness_ <= map_commit_max_fitness_;
            bool motion_gate_allows_commit = true;
            if (commit_accept_ok && commit_fitness_ok &&
                motion_gate_enabled_) {
                motion_gate_allows_commit =
                    evaluateMotionGateForMapCommit(final_pose, publish_time);
            }
            const bool relocalization_commit_ok =
                relocalization_pose_reliable_ &&
                processing_frame_index >= relocalization_cooldown_until_frame_ &&
                relocalization_state_ != RelocalizationState::SEARCHING_LOCAL &&
                relocalization_state_ != RelocalizationState::SEARCHING_GLOBAL &&
                relocalization_state_ != RelocalizationState::CONFIRMING;
            const bool hook_commit_ok = hookAllowsMapCommit();
            const bool allow_map_commit =
                commit_accept_ok && commit_fitness_ok &&
                motion_gate_allows_commit && relocalization_commit_ok &&
                hook_commit_ok;
            diag_map_commit_allowed = allow_map_commit;
            diag_motion_gate_blocked =
                commit_accept_ok && commit_fitness_ok &&
                !motion_gate_allows_commit;
            if (allow_map_commit) {
                diag_map_commit_reason = "accepted";
            } else if (!commit_accept_ok) {
                diag_map_commit_reason = "ndt_rejected_or_prediction_only";
            } else if (!commit_fitness_ok) {
                diag_map_commit_reason = "fitness_rejected";
            } else if (!relocalization_commit_ok) {
                diag_map_commit_reason = "relocalization_guard";
            } else if (!hook_commit_ok) {
                diag_map_commit_reason = "hook_load_guard";
            } else {
                diag_map_commit_reason = "motion_gate_blocked";
            }

            if (allow_map_commit) {
                const auto map_commit_start = DiagClock::now();
                commitKeyFrameWithDynamicFiltering(filtered_cloud, final_pose, publish_time);
                diag_stage.map_commit_ms = elapsedMs(map_commit_start);
                diag_stage.clean_map_ms = last_commit_clean_map_ms_;
                diag_stage.display_map_ms = last_commit_display_map_ms_;

                // V3: 更新 Localization Target（只在 accepted keyframe 时更新）
                if (localization_target_enabled_) {
                    const auto shadow_target_start = DiagClock::now();
                    updateLocalizationTarget(objects_clean_map_, final_pose);
                    swapLocalizationTargetBuffers();
                    diag_stage.shadow_target_ms =
                        elapsedMs(shadow_target_start);
                }
            } else {
                ROS_DEBUG("[MapCommit] skipped: ndt_accepted=%d require_accept=%d fitness=%.3f max_fitness=%.3f motion_gate=%d",
                         ndt_accepted_for_commit ? 1 : 0,
                         map_commit_requires_ndt_accept_ ? 1 : 0,
                         last_ndt_fitness_, map_commit_max_fitness_,
                         motion_gate_allows_commit ? 1 : 0);
            }
            success_frames++;
        }

        // ========== 阶段 8：统计日志 + 长期建图维护 ==========
        auto end_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(end_time - start_time).count();

        // 更新统计
        total_frames_ = total_frames;
        average_process_time_ms_ = elapsed * 1000.0;
        last_total_ms_ = average_process_time_ms_;

        // [Perf] 日志：debug_perf 开启时逐帧输出，否则不输出
        if (debug_cfg_.debug_perf) {
            // 计算 innovation（EKF状态到NDT结果的距离）
            double innovation = 0.0;
            if (crane_motion_ekf_enabled_) {
                const auto& ekf_status = crane_motion_ekf_.status();
                innovation = ekf_status.innovation_norm;
            }

            ROS_INFO("[Perf] total=%.1fms ndt=%.1fms sensor_dt=%.3fs "
                     "fitness=%.3f iter=%d converged=%d "
                     "init_dist=%.3f raw_step=%.3f innov=%.3f "
                     "src_pts=%d tgt_pts=%d tgt_ver=%llu setInput=%d "
                     "tgt_source=%s tgt_reason=%s "
                     "frame=%d",
                     average_process_time_ms_,
                     last_ndt_time_ms_,
                     last_sensor_dt_,
                     last_ndt_fitness_,
                     last_ndt_iterations_,
                     last_ndt_converged_ ? 1 : 0,
                     last_init_dist_,
                     last_raw_step_,
                     innovation,
                     last_source_points_,
                     last_target_points_,
                     static_cast<unsigned long long>(target_version_),
                     setInputTarget_count_,
                     last_actual_target_source_.c_str(),
                     last_target_reason_.c_str(),
                     total_frames);
        }

        // CSV 输出：每帧记录到 /tmp/588_ndt_profile.csv
        const auto csv_log_start = DiagClock::now();
        {
            static auto last_legacy_csv_flush = DiagClock::now();
            if (!csv_initialized_) {
                csv_file_.open("/tmp/588_ndt_profile.csv", std::ios::out | std::ios::trunc);
                if (csv_file_.is_open()) {
                    csv_file_ << "frame,stamp,sensor_dt,"
                              << "source_points,target_points,target_version,setInput_count,"
                              << "ndt_ms,total_ms,"
                              << "fitness,converged,iterations,"
                              << "init_to_result_dist,raw_step,innovation,nis,"
                              << "output_step,max_allowed_step,prediction_only,step_limited,"
                              << "target_source_type,target_reason,"
                              << "ekf_accept,reject_reason\n";
                    csv_initialized_ = true;
                }
            }

            if (csv_file_.is_open()) {
                // 计算 EKF accept 和 reject_reason
                bool ekf_accept = true;
                std::string reject_reason = "NONE";
                if (crane_motion_ekf_enabled_) {
                    const auto& ekf_status = crane_motion_ekf_.status();
                    ekf_accept = ekf_status.ndt_accepted;
                    reject_reason = ekf_status.reject_reason;
                }

                // 计算 innovation
                double innovation = 0.0;
                double nis = 0.0;
                double output_step = 0.0;
                double max_allowed_step = 0.0;
                bool prediction_only = false;
                bool step_limited = false;
                if (crane_motion_ekf_enabled_) {
                    const auto& status = crane_motion_ekf_.status();
                    innovation = status.innovation_norm;
                    nis = status.nis;
                    output_step = status.output_step;
                    max_allowed_step = status.max_allowed_step;
                    prediction_only = status.prediction_only;
                    step_limited = status.step_limited;
                }

                csv_file_ << total_frames << ","
                          << last_stamp_.toSec() << ","
                          << last_sensor_dt_ << ","
                          << last_source_points_ << ","
                          << last_target_points_ << ","
                          << target_version_ << ","
                          << setInputTarget_count_ << ","
                          << last_ndt_time_ms_ << ","
                          << average_process_time_ms_ << ","
                          << last_ndt_fitness_ << ","
                          << (last_ndt_converged_ ? 1 : 0) << ","
                          << last_ndt_iterations_ << ","
                          << last_init_dist_ << ","
                          << last_raw_step_ << ","
                          << innovation << ","
                          << nis << ","
                          << output_step << ","
                          << max_allowed_step << ","
                          << (prediction_only ? 1 : 0) << ","
                          << (step_limited ? 1 : 0) << ","
                          << last_actual_target_source_ << ","
                          << last_target_reason_ << ","
                          << (ekf_accept ? 1 : 0) << ","
                          << reject_reason << '\n';
                if (std::chrono::duration<double>(
                        DiagClock::now() - last_legacy_csv_flush).count() >= 1.0) {
                    csv_file_.flush();
                    last_legacy_csv_flush = DiagClock::now();
                }
            }
        }
        diag_stage.csv_log_ms = elapsedMs(csv_log_start);
        // Preliminary total after legacy CSV. The final value is refreshed
        // after synchronous diagnostics and stored for next-cycle CSV output.
        average_process_time_ms_ = elapsedMs(start_time);
        last_total_ms_ = average_process_time_ms_;

        if (runtime_diag_.isEnabled() && diag_pending_ndt_record_valid_) {
            runtime_diag_.writeNdtFrame(diag_pending_ndt_record_);
            diag_pending_ndt_record_valid_ = false;
        }

        // ========== Runtime Diagnostics 输出 ==========
        if (runtime_diag_.isEnabled()) {
            // 从 EKF 获取状态（安全访问）
            double diag_innovation = 0.0;
            double diag_max_allowed_step = 0.0;
            bool diag_prediction_only = false;
            std::string diag_reject_reason = "NONE";
            try {
                if (crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized()) {
                    const auto& ekf_status = crane_motion_ekf_.status();
                    diag_innovation = ekf_status.innovation_norm;
                    diag_max_allowed_step = ekf_status.max_allowed_step;
                    diag_prediction_only = ekf_status.prediction_only;
                    diag_reject_reason = ekf_status.reject_reason;
                }
            } catch (...) {
                // 如果EKF状态访问失败，使用默认值
                diag_prediction_only = true;
                diag_reject_reason = "EKF_STATUS_UNAVAILABLE";
            }

            const PipelineRateSnapshot pipeline_rate =
                runtime_diag_.pipelineRateSnapshot(
                    queue_overwrite_drop_count_.load(std::memory_order_relaxed));
            size_t diag_queue_size = 0;
            double diag_oldest_queue_age_ms = 0.0;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                diag_queue_size = cloud_queue_.size();
                if (!cloud_queue_.empty()) {
                    diag_oldest_queue_age_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() -
                            cloud_queue_.front().enqueued_at).count();
                }
            }
            runtime_diag_.logPipelineRate(
                pipeline_rate, diag_queue_size, diag_oldest_queue_age_ms);

            diag_frame_index_++;
            diag_last_cloud_stamp_ = last_stamp_.toSec();
            diag_processed_frame_count_++;
            if (last_ndt_converged_) diag_converged_count_++;

            // 写入 NDT 帧 CSV
            NdtFrameRecord ndt_rec;
            ndt_rec.frame_index = diag_frame_index_;
            ndt_rec.cloud_stamp = diag_last_cloud_stamp_;
            ndt_rec.sensor_dt_ms = last_sensor_dt_ * 1000.0;
            ndt_rec.wall_interarrival_ms =
                pipeline_rate.processed_wall_dt_last_ms;
            ndt_rec.callback_sensor_dt_ms =
                pipeline_rate.callback_sensor_dt_last_ms;
            ndt_rec.callback_wall_dt_ms =
                pipeline_rate.callback_wall_dt_last_ms;
            ndt_rec.processed_sensor_dt_ms =
                pipeline_rate.processed_sensor_dt_last_ms;
            ndt_rec.processed_wall_dt_ms =
                pipeline_rate.processed_wall_dt_last_ms;
            ndt_rec.queue_age_ms = diag_queue_age_ms;
            ndt_rec.queue_degraded =
                diag_queue_age_ms > pipeline_rate.frame_budget_ms;
            ndt_rec.raw_points = diag_raw_points;
            ndt_rec.merged_points = diag_raw_points;
            ndt_rec.filtered_points =
                static_cast<int>(filtered_cloud->size());
            ndt_rec.registration_points = last_source_points_;
            ndt_rec.target_points = last_target_points_;
            ndt_rec.target_source = last_actual_target_source_;
            ndt_rec.target_version = target_version_;
            ndt_rec.target_fallback =
                last_actual_target_source_.find("fallback") != std::string::npos;
            ndt_rec.target_reused = last_target_points_ > 0 &&
                setInputTarget_count_ ==
                    diag_set_input_target_count_before;
            ndt_rec.preprocess_ms =
                diag_stage.ros_to_pcl_ms + diag_stage.near_filter_ms +
                diag_stage.hook_prepare_ms + diag_stage.cargo_detect_ms +
                diag_stage.slam_voxel_ms + diag_stage.ground_split_ms +
                diag_stage.channel_filter_ms + diag_stage.human_filter_ms +
                diag_stage.registration_build_ms;
            ndt_rec.target_prepare_ms = diag_stage.target_bind_ms;
            ndt_rec.set_input_target_ms = diag_stage.target_bind_ms;
            ndt_rec.ndt_align_ms = last_ndt_time_ms_;
            ndt_rec.ekf_ms = diag_stage.ekf_ms;
            ndt_rec.map_commit_ms = diag_stage.map_commit_ms;
            ndt_rec.total_ms = average_process_time_ms_;
            ndt_rec.stage = diag_stage;
            ndt_rec.ndt_converged = last_ndt_converged_;
            ndt_rec.ndt_iterations = last_ndt_iterations_;
            ndt_rec.fitness = last_ndt_fitness_;
            ndt_rec.transformation_probability =
                diag_transformation_probability;
            ndt_rec.raw_step_m = last_raw_step_;
            ndt_rec.output_step_m = diag_output_step;
            ndt_rec.allowed_step_m = diag_max_allowed_step;
            ndt_rec.innovation_m = diag_innovation;
            ndt_rec.prediction_only = diag_prediction_only;
            ndt_rec.prediction_reason = diag_reject_reason;
            ndt_rec.map_commit_allowed = diag_map_commit_allowed;
            ndt_rec.map_commit_reason = diag_map_commit_reason;
            auto fillPose = [this](const Sophus::SE3d& pose,
                               double* x, double* y, double* z,
                               double* yaw_deg) {
                *x = pose.translation().x();
                *y = pose.translation().y();
                if (z != nullptr) *z = pose.translation().z();
                double roll = 0.0, pitch = 0.0, yaw = 0.0;
                so3ToRpy(pose.so3(), roll, pitch, yaw);
                *yaw_deg = yaw * 180.0 / M_PI;
            };
            double ignored_z = 0.0;
            if (diag_have_initial_guess) {
                fillPose(diag_initial_guess_pose,
                         &ndt_rec.initial_guess_x,
                         &ndt_rec.initial_guess_y, &ignored_z,
                         &ndt_rec.initial_guess_yaw_deg);
            }
            if (diag_have_raw_ndt_pose) {
                fillPose(diag_raw_ndt_pose,
                         &ndt_rec.raw_x, &ndt_rec.raw_y, &ndt_rec.raw_z,
                         &ndt_rec.raw_yaw_deg);
            }
            double ignored_ekf_yaw = 0.0;
            fillPose(diag_ekf_pose,
                     &ndt_rec.ekf_x, &ndt_rec.ekf_y, &ignored_z,
                     &ignored_ekf_yaw);
            if (diag_have_output_pose) {
                fillPose(diag_output_pose,
                         &ndt_rec.output_x, &ndt_rec.output_y,
                         &ndt_rec.output_z, &ndt_rec.output_yaw_deg);
            }
            ndt_rec.raw_ndt_step_from_previous_m =
                diag_raw_ndt_step_from_previous;
            ndt_rec.ndt_correction_from_initial_guess_m = last_raw_step_;
            ndt_rec.output_dx = diag_output_dx;
            ndt_rec.output_dy = diag_output_dy;
            ndt_rec.output_yaw_step_deg = diag_output_yaw_step_deg;
            ndt_rec.output_speed_mps = diag_output_speed_mps;
            ndt_rec.motion_gate_stationary = motion_gate_stationary_;
            ndt_rec.motion_gate_velocity_modified = false;
            ndt_rec.motion_gate_map_commit_blocked =
                diag_motion_gate_blocked;
            ndt_rec.motion_gate_check_count =
                motion_gate_invariant_check_count_.load(
                    std::memory_order_relaxed);
            ndt_rec.motion_gate_block_count =
                motion_gate_map_commit_block_count_.load(
                    std::memory_order_relaxed);
            ndt_rec.motion_gate_violation_count =
                motion_gate_invariant_violation_count_.load(
                    std::memory_order_relaxed);
            ndt_rec.icp_config_enabled = icp_refine_cfg_.enabled;
            ndt_rec.icp_job_count =
                icp_job_count_.load(std::memory_order_relaxed);
            ndt_rec.icp_stale_drop_count =
                icp_stale_drop_count_.load(std::memory_order_relaxed);
            ndt_rec.icp_map_use_count =
                icp_result_use_count_.load(std::memory_order_relaxed);
            // NDT 风险检测
            if (!last_ndt_converged_) {
                runtime_diag_.logNdtRiskNotConverged(
                    diag_frame_index_, diag_last_cloud_stamp_, last_ndt_fitness_,
                    last_ndt_iterations_, last_actual_target_source_, last_target_points_,
                    last_source_points_, last_ndt_time_ms_, average_process_time_ms_);
            }

            // Fitness spike 检测
            double fitness_median = runtime_diag_.fitnessStats().median();
            double fitness_mad = runtime_diag_.fitnessStats().mad();
            if (last_ndt_fitness_ > fitness_median + 5 * fitness_mad ||
                last_ndt_fitness_ > fitness_median * 2.0) {
                runtime_diag_.logNdtRiskFitnessSpike(
                    diag_frame_index_, diag_last_cloud_stamp_, last_ndt_fitness_,
                    fitness_median, fitness_mad, 2.0, last_ndt_converged_,
                    last_raw_step_, diag_innovation);
            }

            // Frame overrun 检测
            // The budget comes from callback sensor cadence. Processed sensor
            // dt is already distorted after drops and must never define PASS.
            // Prediction-only 检测
            if (diag_prediction_only) {
                runtime_diag_.incrementPredictionOnly();
                diag_consecutive_prediction_only_++;
                if (diag_consecutive_prediction_only_ == 1) {
                    runtime_diag_.logEkfRiskPredictionOnly(
                        diag_frame_index_, diag_last_cloud_stamp_, diag_reject_reason,
                        last_ndt_fitness_, last_ndt_converged_, diag_innovation,
                        diag_consecutive_prediction_only_);
                }
                if (diag_consecutive_prediction_only_ > 3) {
                    runtime_diag_.logEkfRiskPredictionStreak(
                        diag_consecutive_prediction_only_, 0, diag_last_valid_ndt_stamp_);
                }
            } else {
                runtime_diag_.resetConsecutivePredictionOnly();
                diag_consecutive_prediction_only_ = 0;
                diag_last_valid_ndt_stamp_ = diag_last_cloud_stamp_;
            }

            // Raw step exceeded 检测
            if (diag_max_allowed_step > 0.0 &&
                diag_raw_ndt_step_from_previous >
                    diag_max_allowed_step + 0.001) {
                runtime_diag_.incrementRawStepExceeded();
                runtime_diag_.logOdomRiskRawStepExceeded(
                    diag_frame_index_, diag_last_cloud_stamp_,
                    pipeline_rate.callback_sensor_dt_last_ms,
                    diag_output_dx, diag_output_dy, 0.0,
                    diag_raw_ndt_step_from_previous,
                    diag_max_allowed_step, last_ndt_fitness_,
                    last_ndt_converged_);
            }

            // Output step violation 检测
            if (diag_max_allowed_step > 0.0 &&
                diag_output_step > diag_max_allowed_step + 0.0001) {
                runtime_diag_.incrementOutputStepViolation();
                runtime_diag_.logOdomRiskOutputStepViolation(
                    diag_frame_index_, diag_last_cloud_stamp_,
                    diag_output_dx, diag_output_dy, 0.0,
                    diag_output_step, diag_max_allowed_step);
            }

            // 周期性健康日志
            static ros::Time last_diag_health_time;
            if ((ros::Time::now() - last_diag_health_time).toSec() >= 1.0) {
                logNdtHealthPeriodic();
                last_diag_health_time = ros::Time::now();
            }

            // Cargo 健康日志
            static ros::Time last_cargo_health_time;
            if ((ros::Time::now() - last_cargo_health_time).toSec() >= 1.0) {
                logCargoHealthPeriodic();
                last_cargo_health_time = ros::Time::now();
            }

            // Create the pending record now; total_ms is finalized at the true
            // loop tail after periodic maintenance and risk output.
            average_process_time_ms_ = elapsedMs(start_time);
            last_total_ms_ = average_process_time_ms_;
            ndt_rec.total_ms = average_process_time_ms_;
            diag_pending_ndt_record_ = std::move(ndt_rec);
            diag_pending_ndt_record_valid_ = true;
        }

        ROS_INFO_THROTTLE(2.0,
            "[PipelineCounters] callback=%llu queue_drop=%llu dequeued=%llu empty=%llu too_few=%llu duplicate=%llu invalid_dt=%llu ndt_attempt=%llu converged=%llu nonconverged=%llu ekf_accept=%llu ekf_reject=%llu odom=%llu",
            static_cast<unsigned long long>(cloud_callback_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(queue_overwrite_drop_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(cloud_dequeue_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(empty_cloud_skip_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(too_few_points_skip_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(duplicate_cloud_skip_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(invalid_sensor_dt_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(ndt_attempt_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(ndt_converged_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(ndt_nonconverged_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(ekf_accept_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(ekf_reject_count_.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(odom_publish_count_.load(std::memory_order_relaxed)));

        // Status 只在运动时报告，每 30 秒一次
        if ((ros::Time::now() - last_log_time).toSec() > 30.0) {
            Eigen::Vector3d pos = current_pose_.translation();
            ROS_INFO("[Status] frames=%d/%d, pose=(%.2f, %.2f, %.2f), "
                     "keyframes=%d, tiles_flushed=%d, "
                     "local_map=%zu, active_map=%zu, process=%.2fs",
                     success_frames, total_frames, pos.x(), pos.y(), pos.z(),
                     keyframe_count_, flushed_tile_count_,
                     local_map_->size(), global_map_->size(), elapsed);
            last_log_time = ros::Time::now();

            // ========== 长期建图维护 ==========
            if (longterm_mapping_enabled_) {
                // 更新关键帧统计
                total_keyframes_ = loop_closure_detector_.getKeyFrames().size();
                active_keyframes_ = std::min(total_keyframes_, max_active_keyframes_);

                // 定期释放旧关键帧
                static int release_check_count = 0;
                release_check_count++;
                if (release_check_count >= keyframe_release_interval_) {
                    releaseOldKeyframeClouds();
                    release_check_count = 0;
                }

                // 定期 flush dirty tiles
                if (persistent_map_enabled_) {
                    double time_since_flush = (ros::Time::now() - last_flush_time_).toSec();
                    if (time_since_flush >= flush_interval_sec_ || dirty_tile_count_ >= max_dirty_tiles_) {
                        flushDirtyTiles();
                    }

                    // 定期写入 runtime status（每 5 秒）
                    static ros::Time last_status_write_time;
                    double time_since_status = (ros::Time::now() - last_status_write_time).toSec();
                    if (time_since_status >= 5.0) {
                        writeRuntimeStatus();
                        last_status_write_time = ros::Time::now();
                    }
                }

                // 内存保护检查
                if (memory_guard_enabled_) {
                    double time_since_check = (ros::Time::now() - last_memory_check_time_).toSec();
                    if (time_since_check >= memory_check_interval_sec_) {
                        checkMemoryGuard();
                        last_memory_check_time_ = ros::Time::now();
                    }
                }

                // 定期重建 active map（每 10 个关键帧）
                if (longterm_mapping_enabled_ && keyframe_count_ > 0 && keyframe_count_ % rebuild_every_keyframes_ == 0) {
                    rebuildActiveMapFromRecentKeyframes();
                }
            }

        }

        if (runtime_diag_.isEnabled() && diag_pending_ndt_record_valid_) {
            average_process_time_ms_ = elapsedMs(start_time);
            last_total_ms_ = average_process_time_ms_;
            diag_pending_ndt_record_.total_ms = average_process_time_ms_;

            const PipelineRateSnapshot final_rate =
                runtime_diag_.pipelineRateSnapshot(
                    queue_overwrite_drop_count_.load(
                        std::memory_order_relaxed));
            const double frame_budget_ms = final_rate.frame_budget_ms;
            if (frame_budget_ms > 0.0 &&
                average_process_time_ms_ > frame_budget_ms) {
                std::size_t final_queue_size = 0U;
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    final_queue_size = cloud_queue_.size();
                }
                runtime_diag_.incrementFrameOverrun();
                runtime_diag_.logPipelineRiskFrameOverrun(
                    diag_pending_ndt_record_.frame_index,
                    diag_pending_ndt_record_.cloud_stamp, 1.0,
                    frame_budget_ms, average_process_time_ms_,
                    diag_pending_ndt_record_.preprocess_ms,
                    diag_pending_ndt_record_.target_prepare_ms,
                    diag_pending_ndt_record_.ndt_align_ms,
                    diag_pending_ndt_record_.ekf_ms,
                    diag_pending_ndt_record_.map_commit_ms,
                    runtime_diag_.consecutiveOverruns());
                if (runtime_diag_.consecutiveOverruns() >= 3) {
                    runtime_diag_.logPipelineRiskSustainedOverrun(
                        runtime_diag_.consecutiveOverruns(),
                        static_cast<double>(final_queue_size),
                        final_rate.processed_hz, final_rate.callback_hz);
                }
            } else {
                runtime_diag_.resetConsecutiveOverruns();
            }
            average_process_time_ms_ = elapsedMs(start_time);
            last_total_ms_ = average_process_time_ms_;
            diag_pending_ndt_record_.total_ms = average_process_time_ms_;
        }
    }

    ROS_INFO("Processing thread stopped");
}

void NdtSlamNode::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty()) return;
    cloud = filterOutlierPoints(cloud);

    if (neighbor_search_radius_ > 0 && min_neighbors_ > 0) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
        outlier_filter.setInputCloud(cloud);
        outlier_filter.setRadiusSearch(neighbor_search_radius_);
        outlier_filter.setMinNeighborsInRadius(min_neighbors_);
        pcl::PointCloud<pcl::PointXYZ> cleaned;
        outlier_filter.filter(cleaned);
        *cloud = cleaned;
    }
}

void NdtSlamNode::extractFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& registration_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& mapping_cloud) {
    if (cloud->empty()) {
        registration_cloud = cloud;
        mapping_cloud = cloud;
        return;
    }

    mapping_cloud = cloud;

    if (!use_feature_extraction_) {
        registration_cloud = cloud;
        return;
    }

    // 体素化降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(feature_voxel_size_, feature_voxel_size_, feature_voxel_size_);
    voxel_filter.filter(*downsampled);

    // 提取特征点
    pcl::PointCloud<pcl::PointXYZ>::Ptr feature_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);

    float mean_z = 0;
    for (const auto& p : downsampled->points) mean_z += p.z;
    mean_z /= downsampled->size();

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(downsampled);

    for (const auto& point : downsampled->points) {
        std::vector<int> indices;
        std::vector<float> distances;
        kdtree.radiusSearch(point, feature_voxel_size_ * 1.5, indices, distances);

        if (indices.size() < 3) {
            feature_points->push_back(point);
            continue;
        }

        float min_z_local = 1e9, max_z_local = -1e9;
        for (int idx : indices) {
            min_z_local = std::min(min_z_local, downsampled->points[idx].z);
            max_z_local = std::max(max_z_local, downsampled->points[idx].z);
        }
        float height_diff = max_z_local - min_z_local;

        if (height_diff > height_diff_threshold_) {
            feature_points->push_back(point);
        } else {
            ground_points->push_back(point);
        }
    }

    // 构建配准用点云（特征点加权）
    registration_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < feature_weight_; i++) {
        *registration_cloud += *feature_points;
    }
    *registration_cloud += *ground_points;

    static int extract_count = 0;
    extract_count++;
    if (extract_count % 100 == 0) {
        ROS_INFO("Feature: total=%lu, feature=%lu, ground=%lu, reg=%lu",
                 cloud->size(), feature_points->size(), ground_points->size(), registration_cloud->size());
    }
}

void NdtSlamNode::applyLidar2BaseTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty()) return;
    Eigen::Matrix4f transform = lidar2base_transform_.cast<float>();
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, transform);
    *cloud = transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NdtSlamNode::filterOutlierPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-kiss_icp_config_.max_range, kiss_icp_config_.max_range);
    pass_x.filter(*filtered_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(filtered_cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-kiss_icp_config_.max_range, kiss_icp_config_.max_range);
    pass_y.filter(*filtered_cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(filtered_cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-kiss_icp_config_.max_range, kiss_icp_config_.max_range);
    pass_z.filter(*filtered_cloud);

    return filtered_cloud;
}

// CRITICAL RUNTIME CHAIN - DO NOT MODIFY
// a7be4bf runtime pose chain must stay unchanged:
// NDT/refined/EKF -> publishOdometry -> TF -> publishRuntimePath.
// MotionGate controls MapCommit only.
// raw_ndt_pose is allowed only as MapCommit evidence.
// Do NOT route raw_pose/tracking_pose to odom/TF/runtime_path/current_cloud.
void NdtSlamNode::publishOdometry(const ros::Time& stamp, const std::string& cloud_frame_id, const Sophus::SE3d& pose_override) {
    const Sophus::SE3d final_pose = pose_override.matrix().isIdentity() ?
        [this]() { std::lock_guard<std::mutex> lock(cloud_mutex_); return current_pose_; }() :
        pose_override;

    // 时间戳去重：避免 TF_REPEATED_DATA
    if (stamp == last_tf_stamp_) {
        return;  // 同一时间戳的 TF 已发布，跳过
    }
    last_tf_stamp_ = stamp;

    // 发布 TF: odom -> base_link
    if (publish_odom_tf_) {
        geometry_msgs::TransformStamped odom_to_base;
        odom_to_base.header.stamp = stamp;
        odom_to_base.header.frame_id = odom_frame_;
        odom_to_base.child_frame_id = base_frame_;
        odom_to_base.transform = tf2::sophusToTransform(final_pose);
        tf_broadcaster_->sendTransform(odom_to_base);

        // 发布 TF: map -> odom（固定单位变换）
        geometry_msgs::TransformStamped map_to_odom;
        map_to_odom.header.stamp = stamp;
        map_to_odom.header.frame_id = map_frame_;
        map_to_odom.child_frame_id = odom_frame_;
        map_to_odom.transform.translation.x = 0.0;
        map_to_odom.transform.translation.y = 0.0;
        map_to_odom.transform.translation.z = 0.0;
        map_to_odom.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(map_to_odom);
    }

    // 发布 odom topic
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(final_pose);
    odom_msg.pose.covariance.fill(0.0);
    odom_msg.pose.covariance[0] = position_covariance_;
    odom_msg.pose.covariance[7] = position_covariance_;
    odom_msg.pose.covariance[14] = position_covariance_;
    odom_msg.pose.covariance[21] = orientation_covariance_;
    odom_msg.pose.covariance[28] = orientation_covariance_;
    odom_msg.pose.covariance[35] = orientation_covariance_;
    odom_pub_.publish(odom_msg);

    // 发布 pose topic
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = tf2::sophusToPose(final_pose);
    pose_pub_.publish(pose_msg);

    // 发布 path topic（轨迹）
    geometry_msgs::PoseStamped path_pose;
    path_pose.header.stamp = stamp;
    path_pose.header.frame_id = "map";
    path_pose.pose = tf2::sophusToPose(final_pose);
    path_msg_.poses.push_back(path_pose);

    // 限制轨迹长度
    if (path_msg_.poses.size() > path_max_size_) {
        path_msg_.poses.erase(path_msg_.poses.begin());
    }

    path_msg_.header.stamp = stamp;
    path_pub_.publish(path_msg_);
}

// CRITICAL RUNTIME CHAIN - DO NOT MODIFY
// a7be4bf runtime pose chain must stay unchanged:
// NDT/refined/EKF -> publishOdometry -> TF -> publishRuntimePath.
// MotionGate controls MapCommit only.
// raw_ndt_pose is allowed only as MapCommit evidence.
// Do NOT route raw_pose/tracking_pose to odom/TF/runtime_path/current_cloud.
void NdtSlamNode::publishRuntimePath(const Sophus::SE3d& pose, const ros::Time& stamp) {
    Eigen::Vector3d t = pose.translation();

    if (has_last_path_pose_) {
        double dist = (t - last_path_pose_.block<3,1>(0,3)).head<2>().norm();
        if (dist < 0.03) return;  // 3cm 以下不添加
    }

    geometry_msgs::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = "map";
    ps.pose.position.x = t.x();
    ps.pose.position.y = t.y();
    ps.pose.position.z = t.z();

    Eigen::Quaterniond q = pose.so3().unit_quaternion();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();

    runtime_path_msg_.header.stamp = stamp;
    runtime_path_msg_.poses.push_back(ps);

    // 限制最大长度
    if (runtime_path_msg_.poses.size() > 5000) {
        runtime_path_msg_.poses.erase(runtime_path_msg_.poses.begin());
    }

    runtime_path_pub_.publish(runtime_path_msg_);

    last_path_pose_.block<3,1>(0,3) = t;
    has_last_path_pose_ = true;
}

void NdtSlamNode::publishInitialTransform() {
    // 发布 odom -> base_link
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(transform);

    // 发布 map -> odom
    geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.header.stamp = ros::Time::now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = odom_frame_;
    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;
    map_to_odom.transform.rotation.x = 0.0;
    map_to_odom.transform.rotation.y = 0.0;
    map_to_odom.transform.rotation.z = 0.0;
    map_to_odom.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(map_to_odom);

    ROS_INFO("Published initial TF: map -> %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
}

void NdtSlamNode::publishTF(const ros::Time& stamp) {
    // 时间戳去重：避免 TF_REPEATED_DATA 警告
    if (stamp == last_tf_stamp_) {
        return;
    }
    last_tf_stamp_ = stamp;

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;

    Eigen::Vector3d pos = current_pose_.translation();
    Eigen::Quaterniond ori = current_pose_.so3().unit_quaternion();

    transform.transform.translation.x = pos.x();
    transform.transform.translation.y = pos.y();
    transform.transform.translation.z = pos.z();
    transform.transform.rotation.x = ori.x();
    transform.transform.rotation.y = ori.y();
    transform.transform.rotation.z = ori.z();
    transform.transform.rotation.w = ori.w();

    tf_broadcaster_->sendTransform(transform);
}

void NdtSlamNode::startIcpRefineJob(IcpRefineJob job) {
    // This guard is intentionally inside the helper as well as at the call
    // site. A disabled configuration can never create an ICP worker.
    if (!icp_refine_cfg_.enabled || !icp_refine_cfg_.run_after_ndt) {
        ROS_ERROR_THROTTLE(
            1.0, "[ICPInvariant] rejected job while ICP is disabled");
        return;
    }

    bool expected = false;
    if (!icp_running_.compare_exchange_strong(
            expected, true, std::memory_order_acq_rel)) {
        return;
    }

    icp_job_count_.fetch_add(1, std::memory_order_relaxed);
    icp_thread_count_.fetch_add(1, std::memory_order_relaxed);
    icp_thread_ = std::thread([this, job]() mutable {
        const auto start = std::chrono::steady_clock::now();
        try {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(job.source);
            icp.setInputTarget(job.target);
            icp.setMaximumIterations(icp_refine_cfg_.max_iterations);
            icp.setTransformationEpsilon(
                icp_refine_cfg_.transformation_epsilon);
            icp.setMaxCorrespondenceDistance(
                icp_refine_cfg_.max_correspondence_distance);

            pcl::PointCloud<pcl::PointXYZ> aligned;
            icp.align(aligned, job.ndt_pose.matrix().cast<float>());

            const double elapsed_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - start).count();

            if (!icp.hasConverged()) {
                ROS_DEBUG("[ICPResult] frame=%llu rejected=not_converged",
                          static_cast<unsigned long long>(job.frame_index));
            } else if (elapsed_ms > icp_refine_cfg_.max_icp_ms) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[ICPResult] frame=%llu rejected=deadline elapsed_ms=%.1f max_ms=%.1f",
                    static_cast<unsigned long long>(job.frame_index),
                    elapsed_ms, icp_refine_cfg_.max_icp_ms);
            } else {
                const double fitness = icp.getFitnessScore();
                if (std::isfinite(fitness) &&
                    fitness <= icp_refine_cfg_.max_fitness) {
                    const Eigen::Matrix4f raw =
                        icp.getFinalTransformation();
                    Eigen::Matrix3d rotation =
                        raw.block<3, 3>(0, 0).cast<double>();
                    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
                        rotation,
                        Eigen::ComputeFullU | Eigen::ComputeFullV);
                    Eigen::Matrix3d orthogonal =
                        svd.matrixU() * svd.matrixV().transpose();
                    if (orthogonal.determinant() < 0.0) {
                        orthogonal.col(0) *= -1.0;
                    }

                    Eigen::Matrix4d refined_matrix =
                        Eigen::Matrix4d::Identity();
                    refined_matrix.block<3, 3>(0, 0) = orthogonal;
                    refined_matrix.block<3, 1>(0, 3) =
                        raw.block<3, 1>(0, 3).cast<double>();
                    const Sophus::SE3d refined(refined_matrix);

                    const Sophus::SE3d correction =
                        job.ndt_pose.inverse() * refined;
                    const double position_delta =
                        correction.translation().norm();
                    const double rotation_delta_deg =
                        correction.so3().log().norm() * 180.0 / M_PI;

                    // Keep ICP as a small refinement only; it must not become
                    // an unbounded second localization source.
                    const bool plausible =
                        position_delta <= 0.15 &&
                        rotation_delta_deg <= 1.0;
                    if (plausible) {
                        IcpRefineResult result;
                        result.valid = true;
                        result.frame_index = job.frame_index;
                        result.stamp = job.stamp;
                        result.local_map_version =
                            job.local_map_version;
                        result.pose = refined;
                        result.fitness = fitness;
                        result.elapsed_ms = elapsed_ms;
                        {
                            std::lock_guard<std::mutex> lock(
                                icp_result_mutex_);
                            icp_result_ = result;
                        }
                        ROS_DEBUG(
                            "[ICPResult] frame=%llu stamp=%.6f map_version=%llu "
                            "fitness=%.4f elapsed_ms=%.1f",
                            static_cast<unsigned long long>(
                                job.frame_index),
                            job.stamp.toSec(),
                            static_cast<unsigned long long>(
                                job.local_map_version),
                            fitness, elapsed_ms);
                    } else {
                        ROS_DEBUG(
                            "[ICPResult] frame=%llu rejected=large_correction "
                            "position=%.3f rotation_deg=%.3f",
                            static_cast<unsigned long long>(
                                job.frame_index),
                            position_delta, rotation_delta_deg);
                    }
                }
            }
        } catch (const std::exception& e) {
            ROS_WARN_THROTTLE(
                1.0, "[ICPResult] frame=%llu exception=%s",
                static_cast<unsigned long long>(job.frame_index),
                e.what());
        }
        icp_running_.store(false, std::memory_order_release);
    });
}

bool NdtSlamNode::consumeIcpRefineResult(
    uint64_t current_frame,
    const ros::Time& current_stamp,
    uint64_t current_map_version,
    Sophus::SE3d& refined_pose) {
    std::lock_guard<std::mutex> lock(icp_result_mutex_);
    if (!icp_result_.valid) {
        return false;
    }

    const bool future_frame =
        icp_result_.frame_index > current_frame;
    const uint64_t age_frames = future_frame
        ? std::numeric_limits<uint64_t>::max()
        : current_frame - icp_result_.frame_index;
    const double age_sensor_sec =
        (current_stamp - icp_result_.stamp).toSec();

    const char* stale_reason = nullptr;
    if (future_frame || age_sensor_sec < -1e-6) {
        stale_reason = "future_identity";
    } else if (age_frames > 1) {
        stale_reason = "age_frames";
    } else if (age_sensor_sec > 1.0) {
        stale_reason = "age_sensor_sec";
    } else if (icp_result_.local_map_version != current_map_version) {
        stale_reason = "map_version";
    }

    if (stale_reason != nullptr) {
        icp_stale_drop_count_.fetch_add(1, std::memory_order_relaxed);
        ROS_WARN(
            "[ICP_STALE_DROP] job_frame=%llu current_frame=%llu age_frames=%llu "
            "job_stamp=%.6f current_stamp=%.6f age_sensor_sec=%.3f "
            "job_map_version=%llu current_map_version=%llu reason=%s",
            static_cast<unsigned long long>(icp_result_.frame_index),
            static_cast<unsigned long long>(current_frame),
            static_cast<unsigned long long>(age_frames),
            icp_result_.stamp.toSec(), current_stamp.toSec(),
            age_sensor_sec,
            static_cast<unsigned long long>(
                icp_result_.local_map_version),
            static_cast<unsigned long long>(current_map_version),
            stale_reason);
        icp_result_.valid = false;
        return false;
    }

    refined_pose = icp_result_.pose;
    icp_result_.valid = false;  // exactly-once consumption
    icp_result_use_count_.fetch_add(1, std::memory_order_relaxed);
    ROS_DEBUG(
        "[ICP_USE] job_frame=%llu current_frame=%llu age_frames=%llu "
        "map_version=%llu fitness=%.4f",
        static_cast<unsigned long long>(icp_result_.frame_index),
        static_cast<unsigned long long>(current_frame),
        static_cast<unsigned long long>(age_frames),
        static_cast<unsigned long long>(current_map_version),
        icp_result_.fitness);
    return true;
}

void NdtSlamNode::addFrameToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             const Sophus::SE3d& pose,
                             const ros::Time& stamp) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = pose.so3().matrix();
    transform.block<3, 1>(0, 3) = pose.translation();

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, transform.cast<float>());

    std::lock_guard<std::mutex> lock(map_mutex_);
    current_cloud_ = transformed_cloud.makeShared();

    frame_count_++;
    if (frame_count_ % map_update_interval_ == 0) {
        ROS_DEBUG("Publishing current cloud");
        publishCurrentCloud();
    }
}

void NdtSlamNode::asyncRebuildGlobalMap() {
    // 如果已有重建在运行，标记需要再次重建并返回
    if (rebuild_running_.load()) {
        rebuild_pending_.store(true);
        ROS_INFO("Rebuild already running, queued for next rebuild");
        return;
    }

    // 等待之前的重建线程结束
    if (rebuild_thread_.joinable()) {
        rebuild_thread_.join();
    }

    rebuild_running_.store(true);
    rebuild_pending_.store(false);

    rebuild_thread_ = std::thread([this]() {
        auto start = std::chrono::steady_clock::now();

        // 使用 filtered rebuild（从 filtered keyframes + optimized poses 重建）
        if (dynamic_event_config_.enabled) {
            rebuildGlobalMapFiltered();
        } else {
            rebuildGlobalMap();
            rebuildDisplayMap();
            rebuildGroundAndObjectsMap();
        }

        auto end = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(end - start).count();
        ROS_INFO("[AsyncRebuild] all maps rebuilt in %.2fs", elapsed);
        rebuild_running_.store(false);

        // 如果在重建期间又触发了新的重建，递归执行
        if (rebuild_pending_.load()) {
            rebuild_pending_.store(false);
            asyncRebuildGlobalMap();
        }
    });
}

void NdtSlamNode::rebuildGlobalMap() {
    std::lock_guard<std::mutex> lock(map_mutex_);

    global_map_->clear();

    const auto& keyframes = loop_closure_detector_.getKeyFrames();

    // 重建时对每个 keyframe 做通道过滤，确保 registration map 不含吊货
    for (const auto& kf : keyframes) {
        if (kf.cloud_->empty()) continue;

        Eigen::Matrix4d transform = kf.pose_.matrix();

        if (channel_filter_config_.enabled) {
            // 在 base_link 下做通道过滤
            pcl::PointCloud<pcl::PointXYZ> base_ground, base_objects;
            separateGroundByGrid(*kf.cloud_, base_ground, base_objects);

            std::map<CellKey, float> empty_ground_model;
            ChannelFilterResult ch_result = channel_filter_.filter(base_objects.makeShared(), empty_ground_model);

            // ========== HumanObjectDynamicFilter（人体动态过滤）==========
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_safe(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_candidates(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_dynamic(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_pending(new pcl::PointCloud<pcl::PointXYZ>);

            if (human_filter_config_.enabled) {
                // 使用 0 作为时间戳（rebuild 不需要精确时间）
                human_filter_.processFrame(ch_result.safe_objects, transform, 0.0,
                                           rebuild_human_safe, rebuild_human_candidates,
                                           rebuild_human_dynamic, rebuild_human_pending);
            } else {
                rebuild_human_safe = ch_result.safe_objects;
            }

            // 只把 human_safe_objects + ground 变换到 map 并加入 global_map
            pcl::PointCloud<pcl::PointXYZ> safe_transformed;
            pcl::transformPointCloud(*rebuild_human_safe, safe_transformed, transform.cast<float>());

            pcl::PointCloud<pcl::PointXYZ> ground_transformed;
            pcl::transformPointCloud(base_ground, ground_transformed, transform.cast<float>());

            for (const auto& point : safe_transformed.points) {
                if (std::abs(point.x) <= max_map_size_ && std::abs(point.y) <= max_map_size_ &&
                    std::abs(point.z) <= max_map_size_ && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    global_map_->push_back(point);
                }
            }
            for (const auto& point : ground_transformed.points) {
                if (std::abs(point.x) <= max_map_size_ && std::abs(point.y) <= max_map_size_ &&
                    std::abs(point.z) <= max_map_size_ && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    global_map_->push_back(point);
                }
            }
        } else {
            // 旧逻辑：全部点进 map
            pcl::PointCloud<pcl::PointXYZ> transformed;
            pcl::transformPointCloud(*kf.cloud_, transformed, transform.cast<float>());

            for (const auto& point : transformed.points) {
                if (std::abs(point.x) > max_map_size_ ||
                    std::abs(point.y) > max_map_size_ ||
                    std::abs(point.z) > max_map_size_) {
                    continue;
                }
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                    continue;
                }
                global_map_->push_back(point);
            }
        }
    }

    if (use_voxel_filter_ && global_map_->size() > 100) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(global_map_);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> filtered;
        voxel_filter.filter(filtered);
        *global_map_ = filtered;
    }

    publishMap();
    ROS_INFO("Global map rebuilt from %zu keyframes, size: %zu",
             keyframes.size(), global_map_->size());
}

void NdtSlamNode::rebuildGlobalMapFiltered() {
    std::lock_guard<std::mutex> lock(map_mutex_);

    auto start_time = std::chrono::steady_clock::now();

    // 清空所有地图
    global_map_->clear();
    display_map_->clear();
    ground_map_->clear();
    objects_map_->clear();
    objects_clean_map_->clear();
    rebuild_objects_filtered_->clear();
    rebuild_payload_candidate_->clear();
    rebuild_payload_dynamic_->clear();
    rebuild_human_candidate_->clear();
    rebuild_human_dynamic_->clear();
    rebuild_human_pending_->clear();
    rebuild_ground_raw_->clear();

    auto& keyframes = const_cast<std::deque<KeyFrame>&>(loop_closure_detector_.getKeyFrames());

    int skipped_dynamic_points = 0;
    int inserted_points = 0;
    int reapplied_count = 0;

    auto addInRange = [&](const pcl::PointCloud<pcl::PointXYZ>& src,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr dst) {
        for (const auto& p : src.points) {
            if (std::abs(p.x) <= max_map_size_ &&
                std::abs(p.y) <= max_map_size_ &&
                std::abs(p.z) <= max_map_size_ &&
                std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                dst->push_back(p);
            }
        }
    };

    for (auto& kf : keyframes) {
        if (!kf.cloud_ || kf.cloud_->empty()) continue;

        Sophus::SE3d pose = kf.has_refined_pose_ ? kf.pose_refined_ : kf.pose_;
        Eigen::Matrix4d transform = pose.matrix();

        // 如果 keyframe 没有 filtered_objects，需要从原始点云重新过滤
        if (!kf.objects_filtered || kf.objects_filtered->empty() || kf.dirty_dynamic) {
            if (channel_filter_config_.enabled) {
                // 从原始点云重新过滤
                pcl::PointCloud<pcl::PointXYZ> base_ground, base_objects;
                separateGroundByGrid(*kf.cloud_, base_ground, base_objects);

                std::map<CellKey, float> empty_ground_model;
                ChannelFilterResult ch_result = channel_filter_.filter(
                    base_objects.makeShared(), empty_ground_model);

                // 人体过滤
                pcl::PointCloud<pcl::PointXYZ>::Ptr human_safe(new pcl::PointCloud<pcl::PointXYZ>);
                if (human_filter_config_.enabled) {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr human_cand, human_dyn, human_pend;
                    human_filter_.processFrame(ch_result.safe_objects, transform, kf.stamp_.toSec(),
                                               human_safe, human_cand, human_dyn, human_pend);
                } else {
                    human_safe = ch_result.safe_objects;
                }

                kf.objects_raw = base_objects.makeShared();
                kf.objects_filtered = human_safe;
                kf.ground_points = base_ground.makeShared();
                kf.dirty_dynamic = false;
                reapplied_count++;
            }
        }

        // 使用 filtered_objects 插入正式地图
        if (kf.objects_filtered && !kf.objects_filtered->empty()) {
            pcl::PointCloud<pcl::PointXYZ> filtered_transformed;
            pcl::transformPointCloud(*kf.objects_filtered, filtered_transformed, transform.cast<float>());
            addInRange(filtered_transformed, global_map_);
            addInRange(filtered_transformed, objects_map_);
            addInRange(filtered_transformed, rebuild_objects_filtered_);
            inserted_points += filtered_transformed.size();
        }

        // 地面点
        if (kf.ground_points && !kf.ground_points->empty()) {
            pcl::PointCloud<pcl::PointXYZ> ground_transformed;
            pcl::transformPointCloud(*kf.ground_points, ground_transformed, transform.cast<float>());
            addInRange(ground_transformed, global_map_);
            addInRange(ground_transformed, ground_map_);
            addInRange(ground_transformed, rebuild_ground_raw_);
        }

        // 应用 dynamic mask（如果有已确认的事件）
        if (dynamic_event_config_.enabled) {
            double kf_time = kf.stamp_.toSec();
            for (const auto& session : dynamic_event_manager_.getPayloadSessions()) {
                if (!session.confirmed) continue;
                if (kf_time < session.first_candidate_time || kf_time > session.end_time + 5.0) continue;

                // 检查点是否在停放保护区域
                if (session.state == PayloadSessionState::PLACED_STATIC && session.placed_protected) {
                    // 停放货物不删除
                    continue;
                }
            }
        }
    }

    // 体素滤波
    auto voxelFilter = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, double size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        if (input->size() > 100) {
            pcl::VoxelGrid<pcl::PointXYZ> vf;
            vf.setInputCloud(input);
            vf.setLeafSize(size, size, size);
            vf.filter(*output);
        } else {
            *output = *input;
        }
        return output;
    };

    *global_map_ = *voxelFilter(global_map_, voxel_size_);
    *display_map_ = *voxelFilter(display_map_, display_voxel_size_);
    *ground_map_ = *voxelFilter(ground_map_, ground_voxel_size_);
    *objects_map_ = *voxelFilter(objects_map_, objects_voxel_size_);

    // 重建 clean map（带 dynamic deny gate）
    rebuildCleanMap();

    publishMap();

    auto end_time = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(end_time - start_time).count();

    ROS_INFO("[FilteredRebuild] keyframes=%zu reapplied=%d inserted=%d skipped_dynamic=%d time=%.2fs",
             keyframes.size(), reapplied_count, inserted_points, skipped_dynamic_points, elapsed);
    ROS_INFO("[FilteredRebuild] registration=%zu objects=%zu ground=%zu clean=%zu",
             global_map_->size(), objects_map_->size(), ground_map_->size(), objects_clean_map_->size());
}

void NdtSlamNode::rebuildDisplayMap() {
    // 显示地图使用更细的体素，保留货物轮廓
    display_map_->clear();

    const auto& keyframes = loop_closure_detector_.getKeyFrames();

    for (const auto& kf : keyframes) {
        if (kf.cloud_->empty()) continue;

        Eigen::Matrix4d transform = kf.pose_.matrix();

        if (channel_filter_config_.enabled) {
            // 在 base_link 下做通道过滤
            pcl::PointCloud<pcl::PointXYZ> base_ground, base_objects;
            separateGroundByGrid(*kf.cloud_, base_ground, base_objects);

            std::map<CellKey, float> empty_ground_model;
            ChannelFilterResult ch_result = channel_filter_.filter(base_objects.makeShared(), empty_ground_model);

            // 变换 safe_objects + ground 到 map
            pcl::PointCloud<pcl::PointXYZ> safe_transformed;
            pcl::transformPointCloud(*ch_result.safe_objects, safe_transformed, transform.cast<float>());

            pcl::PointCloud<pcl::PointXYZ> ground_transformed;
            pcl::transformPointCloud(base_ground, ground_transformed, transform.cast<float>());

            for (const auto& point : safe_transformed.points) {
                if (std::abs(point.x) <= max_map_size_ && std::abs(point.y) <= max_map_size_ &&
                    std::abs(point.z) <= max_map_size_ && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    display_map_->push_back(point);
                }
            }
            for (const auto& point : ground_transformed.points) {
                if (std::abs(point.x) <= max_map_size_ && std::abs(point.y) <= max_map_size_ &&
                    std::abs(point.z) <= max_map_size_ && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                    display_map_->push_back(point);
                }
            }

            // ========== Placed Cargo: 将停放货物的 payload_candidate 点添加到 display_map ==========
            if (dynamic_event_config_.enabled && dynamic_event_config_.placed_to_display_map) {
                // 变换 payload_candidates 到 map
                pcl::PointCloud<pcl::PointXYZ> payload_cand_transformed;
                if (ch_result.payload_candidates && !ch_result.payload_candidates->empty()) {
                    pcl::transformPointCloud(*ch_result.payload_candidates, payload_cand_transformed, transform.cast<float>());
                }

                // 检查是否有 PLACED_STATIC session 覆盖当前 keyframe 时间
                double kf_time = kf.stamp_.toSec();
                for (const auto& session : dynamic_event_manager_.getPayloadSessions()) {
                    if (session.state != PayloadSessionState::PLACED_STATIC) continue;
                    if (!session.placed_protected) continue;

                    // 检查 keyframe 时间是否在 session 时间范围内
                    if (kf_time < session.first_candidate_time || kf_time > session.placed_time + 10.0) continue;

                    // 将 payload_candidate 点添加到 display_map（如果在 placed_bbox 内）
                    for (const auto& point : payload_cand_transformed.points) {
                        if (std::abs(point.x) <= max_map_size_ && std::abs(point.y) <= max_map_size_ &&
                            std::abs(point.z) <= max_map_size_ && std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                            if (session.placed_bbox.contains(point)) {
                                display_map_->push_back(point);
                            }
                        }
                    }
                }
            }
        } else {
            // 旧逻辑
            pcl::PointCloud<pcl::PointXYZ> transformed;
            pcl::transformPointCloud(*kf.cloud_, transformed, transform.cast<float>());

            for (const auto& point : transformed.points) {
                if (std::abs(point.x) > max_map_size_ ||
                    std::abs(point.y) > max_map_size_ ||
                    std::abs(point.z) > max_map_size_) {
                    continue;
                }
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                    continue;
                }
                display_map_->push_back(point);
            }
        }
    }

    if (use_voxel_filter_ && display_map_->size() > 100) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(display_map_);
        voxel_filter.setLeafSize(display_voxel_size_, display_voxel_size_, display_voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> filtered;
        voxel_filter.filter(filtered);
        *display_map_ = filtered;
    }

    publishDisplayMap();
    ROS_INFO("[DisplayMap] rebuilt from %zu keyframes, size: %zu (voxel=%.3fm)",
             keyframes.size(), display_map_->size(), display_voxel_size_);
}

void NdtSlamNode::rebuildGroundAndObjectsMap() {
    ground_map_->clear();
    objects_map_->clear();

    const auto& keyframes = loop_closure_detector_.getKeyFrames();

    for (const auto& kf : keyframes) {
        if (kf.cloud_->empty()) continue;

        Eigen::Matrix4d transform = kf.pose_.matrix();

        // 范围裁剪并加入各层地图
        auto addInRange = [&](const pcl::PointCloud<pcl::PointXYZ>& src,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr dst) {
            for (const auto& p : src.points) {
                if (std::abs(p.x) <= max_map_size_ &&
                    std::abs(p.y) <= max_map_size_ &&
                    std::abs(p.z) <= max_map_size_ &&
                    std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                    dst->push_back(p);
                }
            }
        };

        if (channel_filter_config_.enabled) {
            // 在 base_link 下做通道过滤
            pcl::PointCloud<pcl::PointXYZ> base_ground, base_objects;
            separateGroundByGrid(*kf.cloud_, base_ground, base_objects);

            std::map<CellKey, float> empty_ground_model;
            ChannelFilterResult ch_result = channel_filter_.filter(base_objects.makeShared(), empty_ground_model);

            // ========== HumanObjectDynamicFilter（人体动态过滤）==========
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_safe(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_candidates(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_dynamic(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_pending(new pcl::PointCloud<pcl::PointXYZ>);

            if (human_filter_config_.enabled) {
                // 使用 0 作为时间戳（rebuild 不需要精确时间）
                human_filter_.processFrame(ch_result.safe_objects, transform, 0.0,
                                           rebuild_human_safe, rebuild_human_candidates,
                                           rebuild_human_dynamic, rebuild_human_pending);
            } else {
                rebuild_human_safe = ch_result.safe_objects;
            }

            // 变换到 map 坐标系
            pcl::PointCloud<pcl::PointXYZ> ground_transformed;
            pcl::transformPointCloud(base_ground, ground_transformed, transform.cast<float>());

            pcl::PointCloud<pcl::PointXYZ> safe_transformed;
            pcl::transformPointCloud(*rebuild_human_safe, safe_transformed, transform.cast<float>());

            addInRange(ground_transformed, ground_map_);
            addInRange(safe_transformed, objects_map_);
        } else {
            // 旧逻辑
            pcl::PointCloud<pcl::PointXYZ> transformed;
            pcl::transformPointCloud(*kf.cloud_, transformed, transform.cast<float>());

            pcl::PointCloud<pcl::PointXYZ> kf_ground, kf_objects;
            separateGroundByGrid(transformed, kf_ground, kf_objects);

            addInRange(kf_ground, ground_map_);
            addInRange(kf_objects, objects_map_);
        }
    }

    // 地面地图体素滤波
    if (use_voxel_filter_ && ground_map_->size() > 100) {
        pcl::VoxelGrid<pcl::PointXYZ> vf;
        vf.setInputCloud(ground_map_);
        vf.setLeafSize(ground_voxel_size_, ground_voxel_size_, ground_voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> f;
        vf.filter(f);
        *ground_map_ = f;
    }

    // 非地面/货物地图体素滤波（很细）
    if (use_voxel_filter_ && objects_map_->size() > 100) {
        pcl::VoxelGrid<pcl::PointXYZ> vf;
        vf.setInputCloud(objects_map_);
        vf.setLeafSize(objects_voxel_size_, objects_voxel_size_, objects_voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> f;
        vf.filter(f);
        *objects_map_ = f;
    }

    // 生成 clean objects map（rebuild 时使用简化版本）
    if (objects_map_->size() > 50) {
        const double clean_bev_cell = 0.15;
        const float clean_min_height = 0.35f;  // 与 height_above_ground=0.3m 配合
        const int clean_min_points = 3;

        struct BevKey { int x, y; bool operator<(const BevKey& o) const { return x<o.x||(x==o.x&&y<o.y); } };
        std::map<BevKey, float> bev_max_z;
        std::map<BevKey, int> bev_count;
        std::map<BevKey, std::vector<int>> bev_indices;

        // 计算全局 z 最小值作为地面参考
        float global_z_min = 1e9;
        for (const auto& p : objects_map_->points) {
            if (p.z < global_z_min) global_z_min = p.z;
        }

        for (int i = 0; i < (int)objects_map_->size(); ++i) {
            const auto& p = objects_map_->points[i];
            BevKey bk{(int)std::floor(p.x / clean_bev_cell), (int)std::floor(p.y / clean_bev_cell)};
            bev_indices[bk].push_back(i);
            bev_count[bk]++;
            float h = p.z - global_z_min;
            if (bev_max_z.find(bk) == bev_max_z.end() || h > bev_max_z[bk]) {
                bev_max_z[bk] = h;
            }
        }

        objects_clean_map_->clear();
        for (auto& [bk, indices] : bev_indices) {
            if (bev_max_z[bk] >= clean_min_height && bev_count[bk] >= clean_min_points) {
                for (int idx : indices) {
                    objects_clean_map_->push_back(objects_map_->points[idx]);
                }
            }
        }
    }

    publishGroundMap();
    publishObjectsMap();
    publishObjectsCleanMap();
    ROS_INFO("[GroundMap] rebuilt: ground=%zu, objects=%zu, clean=%zu",
             ground_map_->size(), objects_map_->size(), objects_clean_map_->size());
}

void NdtSlamNode::publishDisplayMap() {
    if (display_map_->empty()) return;

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*display_map_, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = map_frame_;
    display_map_pub_.publish(map_msg);
}

void NdtSlamNode::publishGroundMap() {
    if (ground_map_->empty()) return;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*ground_map_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    ground_map_pub_.publish(msg);
}

void NdtSlamNode::publishObjectsMap() {
    if (objects_map_->empty()) return;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*objects_map_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    objects_map_pub_.publish(msg);
}

void NdtSlamNode::publishObjectsCleanMap() {
    if (objects_clean_map_->empty()) return;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*objects_clean_map_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = map_frame_;
    objects_clean_map_pub_.publish(msg);
}

void NdtSlamNode::rebuildCleanMap() {
    // 使用持久化的 BEV 观测计数做时间一致性过滤
    // 只有被 >= clean_min_observations_ 个关键帧观测到的 cell 才进入 clean map
    // 自适应：远处点云稀疏，放宽高度和时间一致性要求
    if (objects_map_->empty()) {
        ROS_DEBUG("[CleanMap] objects_map_ is empty, skipping");
        return;
    }

    // P1: 保护检查 - 如果 objects_map 有点但 bev_observation_count_ 为空，跳过
    if (objects_map_->size() > 1000 && bev_observation_count_.empty()) {
        ROS_ERROR("[CleanMap] FAIL: objects_map has %zu points but bev_observation_count_ is empty, skip clean rebuild",
                  objects_map_->size());
        return;
    }

    ROS_INFO("[CleanMap] rebuilding: objects_map=%zu, bev_obs_count=%zu",
             objects_map_->size(), bev_observation_count_.size());

    // ========== Dynamic Deny Gate + Static Protect ==========
    std::set<std::pair<int,int>> deny_cells;
    std::set<std::pair<int,int>> protect_cells;
    int cargo_deny_count = 0;
    int human_deny_count = 0;

    if (dynamic_event_config_.enabled && dynamic_event_config_.clean_deny_enabled) {
        double current_time = ros::Time::now().toSec();

        // 吊货 deny cells（从 DynamicEventManager 获取）
        auto cargo_deny = dynamic_event_manager_.getDynamicDenyCells(0.15, current_time);
        cargo_deny_count = cargo_deny.size();
        deny_cells.insert(cargo_deny.begin(), cargo_deny.end());

        // P2: 人员 deny cells（从 HumanFilter 获取）
        // 在下面的过滤逻辑中检查

        protect_cells = dynamic_event_manager_.getStaticProtectCells(0.15, current_time);

        if (!deny_cells.empty()) {
            ROS_INFO("[CleanMapDynamicGate] cargo_deny_cells=%d, total_deny_cells=%zu",
                     cargo_deny_count, deny_cells.size());
        }
        if (!protect_cells.empty()) {
            ROS_INFO("[CleanMapStaticProtect] protect_cells=%zu", protect_cells.size());
        }
    }

    const double clean_bev_cell = 0.15;
    const int clean_min_points = 3;

    // ========== 构建 payload_candidate 的 BEV 索引（用于 placed cargo 保护）==========
    std::map<BevKey, std::vector<int>> payload_cand_bev_indices;
    if (!protect_cells.empty() && rebuild_payload_candidate_ && !rebuild_payload_candidate_->empty()) {
        for (int i = 0; i < (int)rebuild_payload_candidate_->size(); ++i) {
            const auto& p = rebuild_payload_candidate_->points[i];
            BevKey bk{(int)std::floor(p.x / clean_bev_cell), (int)std::floor(p.y / clean_bev_cell)};
            payload_cand_bev_indices[bk].push_back(i);
        }
        ROS_INFO("[CleanMapPlacedCargo] payload_candidate BEV cells=%zu", payload_cand_bev_indices.size());
    }

    // 使用 objects_map_ 的全局 z 最小值作为地面参考
    float obj_z_min = 1e9;
    for (const auto& p : objects_map_->points) {
        if (p.z < obj_z_min) obj_z_min = p.z;
    }

    // 按 BEV cell 分组
    std::map<BevKey, float> bev_max_h;
    std::map<BevKey, float> bev_dist;    // 每个 cell 的平均距离
    std::map<BevKey, int> bev_count;
    std::map<BevKey, std::vector<int>> bev_indices;

    for (int i = 0; i < (int)objects_map_->size(); ++i) {
        const auto& p = objects_map_->points[i];
        BevKey bk{(int)std::floor(p.x / clean_bev_cell), (int)std::floor(p.y / clean_bev_cell)};
        bev_indices[bk].push_back(i);
        bev_count[bk]++;
        float h = p.z - obj_z_min;
        if (bev_max_h.find(bk) == bev_max_h.end() || h > bev_max_h[bk]) {
            bev_max_h[bk] = h;
        }

        // 记录距离（用于自适应阈值）
        float dist = std::sqrt(p.x * p.x + p.y * p.y);
        if (bev_dist.find(bk) == bev_dist.end()) {
            bev_dist[bk] = dist;
        } else {
            bev_dist[bk] = (bev_dist[bk] + dist) / 2.0f;  // 平均距离
        }
    }

    // 自适应阈值函数
    auto getAdaptiveMinHeight = [](float dist) -> float {
        if (dist < 10.0f) return 0.35f;   // 近处：0.35m
        if (dist < 20.0f) return 0.25f;   // 中距离：0.25m
        return 0.15f;                      // 远处：0.15m
    };

    auto getAdaptiveMinObs = [](float dist) -> int {
        if (dist < 10.0f) return 2;        // 近处：2次观测
        if (dist < 20.0f) return 1;        // 中距离：1次观测
        return 1;                           // 远处：1次观测（首次进入即可保留）
    };

    // 构建 clean map：自适应高度 + 点数 + 自适应时间一致性
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_clean(new pcl::PointCloud<pcl::PointXYZ>);
    int total_cells = 0, passed_cells = 0;
    int near_passed = 0, mid_passed = 0, far_passed = 0;
    int near_failed = 0, mid_failed = 0, far_failed = 0;

    int deny_rejected_cells = 0;
    int deny_rejected_points = 0;
    int protect_kept_cells = 0;
    int protect_kept_points = 0;

    // P0-4: 3D deny volume 替代 2D BEV deny
    // 只有 enable_cargo_history_clean=true 时才使用旧的 2D deny
    // 新的 3D deny 在后面逐点检查
    if (dynamic_event_config_.enabled && dynamic_event_config_.clean_deny_enabled &&
        dynamic_event_config_.enable_cargo_history_clean) {
        double current_time = ros::Time::now().toSec();
        for (const auto& [bk, indices] : bev_indices) {
            float cell_center_x = bk.x * clean_bev_cell + clean_bev_cell / 2;
            float cell_center_y = bk.y * clean_bev_cell + clean_bev_cell / 2;
            if (isCargoDenied(cell_center_x, cell_center_y, current_time)) {
                deny_cells.insert({bk.x, bk.y});
            }
        }
        if (!cargo_deny_history_.empty()) {
            ROS_INFO("[CleanMapCargoHistory] cargo_deny_history_cells=%zu", cargo_deny_history_.size());
        }
    }

    for (auto& [bk, indices] : bev_indices) {
        total_cells++;

        std::pair<int,int> bk_pair = {bk.x, bk.y};

        // ========== Static Protect 优先级最高 ==========
        if (!protect_cells.empty() && protect_cells.find(bk_pair) != protect_cells.end()) {
            // 停放保护区域：直接保留，不走 dynamic deny
            for (int idx : indices) {
                new_clean->push_back(objects_map_->points[idx]);
            }

            // 将 placed cargo 的 payload_candidate 点也添加到 clean map
            auto payload_it = payload_cand_bev_indices.find(bk);
            if (payload_it != payload_cand_bev_indices.end()) {
                for (int idx : payload_it->second) {
                    new_clean->push_back(rebuild_payload_candidate_->points[idx]);
                }
                protect_kept_points += payload_it->second.size();
                ROS_DEBUG("[CleanMapPlacedCargo] added %zu payload_candidate points to protect cell (%d,%d)",
                          payload_it->second.size(), bk.x, bk.y);
            }

            protect_kept_cells++;
            protect_kept_points += indices.size();
            passed_cells++;
            continue;
        }

        // ========== Dynamic Deny Gate ==========
        if (!deny_cells.empty() && deny_cells.find(bk_pair) != deny_cells.end()) {
            deny_rejected_cells++;
            deny_rejected_points += indices.size();
            continue;  // 跳过被动态事件覆盖的 cell
        }

        // P2: Human Deny Gate（从 HumanFilter 的 deny history 检查）
        // P0-4: 只有 enable_human_history_clean=true 时才使用
        if (human_filter_config_.enabled && dynamic_event_config_.enable_human_history_clean) {
            // 检查该 cell 的中心点是否被 deny
            float cell_center_x = bk.x * clean_bev_cell + clean_bev_cell / 2;
            float cell_center_y = bk.y * clean_bev_cell + clean_bev_cell / 2;
            double current_time = ros::Time::now().toSec();

            if (human_filter_.isCellDenied(cell_center_x, cell_center_y, current_time)) {
                human_deny_count++;
                deny_rejected_cells++;
                deny_rejected_points += indices.size();
                continue;  // 跳过被人员动态覆盖的 cell
            }
        }

        int obs_count = 0;
        auto it = bev_observation_count_.find(bk);
        if (it != bev_observation_count_.end()) obs_count = it->second;

        float dist = bev_dist[bk];
        float min_height = getAdaptiveMinHeight(dist);
        int min_obs = getAdaptiveMinObs(dist);

        // 三重过滤：自适应高度，点数 >= 3，自适应观测次数
        if (bev_max_h[bk] >= min_height &&
            bev_count[bk] >= clean_min_points &&
            obs_count >= min_obs) {
            for (int idx : indices) {
                const auto& p = objects_map_->points[idx];

                // P0-3: 3D deny volume 检查（替代 2D BEV 全高度删除）
                // 只删除在 deny volume z 范围内的点，保护下方静态货物
                if (dynamic_event_config_.enabled && !dynamic_deny_volume_map_.empty()) {
                    if (isPointDeniedBy3DHistory(p.x, p.y, p.z)) {
                        deny_rejected_points++;
                        continue;
                    }
                }

                new_clean->push_back(p);
            }
            passed_cells++;
            if (dist < 10.0f) near_passed++;
            else if (dist < 20.0f) mid_passed++;
            else far_passed++;
        } else {
            if (dist < 10.0f) near_failed++;
            else if (dist < 20.0f) mid_failed++;
            else far_failed++;
        }
    }

    if (protect_kept_cells > 0) {
        ROS_INFO("[CleanMapStaticProtect] protect_cells=%d, protected_points=%d",
                 protect_kept_cells, protect_kept_points);
    }
    if (deny_rejected_cells > 0) {
        ROS_INFO("[CleanMapDynamicGate] rejected_cells=%d, rejected_points=%d, cargo_deny=%d, human_deny=%d, cargo_history=%zu",
                 deny_rejected_cells, deny_rejected_points, cargo_deny_count, human_deny_count, cargo_deny_history_.size());
    }

    // 更新 clean map（线程安全）
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        *objects_clean_map_ = *new_clean;
    }

    publishObjectsCleanMap();

    // [CleanMapDynamicGate3D] 日志
    ROS_INFO("[CleanMapDynamicGate3D] cargo_volumes=%zu human_volumes=%zu cargo_3d_denied_points=%d human_3d_denied_points=%d protected_static_points=%d clean_points=%zu",
             dynamic_deny_volume_map_.size(),
             (size_t)0,  // human volumes 暂时为 0
             deny_rejected_points,  // cargo 3D denied
             human_deny_count,      // human denied
             protect_kept_points,   // protected static
             new_clean->size());

    ROS_INFO("[CleanMaskStatus] using_mask=true, deny_cells=%zu, protect_cells=%zu, deny_rejected=%d, protect_kept=%d",
             deny_cells.size(), protect_cells.size(), deny_rejected_cells, protect_kept_cells);
    ROS_INFO("[CleanMap] rebuilt: %d/%d cells passed (near=%d/%d mid=%d/%d far=%d/%d), points=%zu",
             passed_cells, total_cells,
             near_passed, near_passed + near_failed,
             mid_passed, mid_passed + mid_failed,
             far_passed, far_passed + far_failed,
             new_clean->size());
}

void NdtSlamNode::separateGroundByGrid(const pcl::PointCloud<pcl::PointXYZ>& input,
                                     pcl::PointCloud<pcl::PointXYZ>& ground_out,
                                     pcl::PointCloud<pcl::PointXYZ>& objects_out) {
    if (input.empty()) return;

    // 第一步：按 XY 网格分组，每个格子收集 z 值
    struct CellKey {
        int x, y;
        bool operator<(const CellKey& o) const { return x < o.x || (x == o.x && y < o.y); }
    };
    std::map<CellKey, std::vector<float>> cell_z_values;
    std::map<CellKey, std::vector<int>> cell_indices;

    for (int i = 0; i < (int)input.size(); ++i) {
        const auto& p = input.points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        CellKey key{(int)std::floor(p.x / grid_cell_size_), (int)std::floor(p.y / grid_cell_size_)};
        cell_z_values[key].push_back(p.z);
        cell_indices[key].push_back(i);
    }

    // 第二步：每个格子计算局部地面高度（第 20 百分位）
    std::map<CellKey, float> cell_ground_z;
    for (auto& [key, z_vals] : cell_z_values) {
        if (z_vals.size() < 3) {
            // 点太少，用最小值作为地面
            cell_ground_z[key] = *std::min_element(z_vals.begin(), z_vals.end());
        } else {
            std::sort(z_vals.begin(), z_vals.end());
            cell_ground_z[key] = z_vals[z_vals.size() / 5];  // 第20百分位
        }
    }

    // 第三步：根据局部地面高度分类
    for (auto& [key, indices] : cell_indices) {
        float local_ground_z = cell_ground_z[key];
        for (int idx : indices) {
            const auto& p = input.points[idx];
            float height_above_ground = p.z - local_ground_z;
            if (height_above_ground < height_above_ground_) {
                ground_out.push_back(p);
            } else {
                objects_out.push_back(p);
            }
        }
    }

    // 第四步：自适应 SOR 过滤——根据距离动态调整参数
    // 远处点云稀疏，放宽过滤条件；近处点云密集，保持严格过滤
    if (objects_out.size() > 50) {
        // 按距离分组：近处（<10m）、中距离（10-20m）、远处（>20m）
        pcl::PointCloud<pcl::PointXYZ> near_cloud, mid_cloud, far_cloud;
        std::vector<int> near_idx, mid_idx, far_idx;

        for (int i = 0; i < (int)objects_out.size(); ++i) {
            const auto& p = objects_out.points[i];
            float dist = std::sqrt(p.x * p.x + p.y * p.y);
            if (dist < 10.0f) {
                near_cloud.push_back(p);
                near_idx.push_back(i);
            } else if (dist < 20.0f) {
                mid_cloud.push_back(p);
                mid_idx.push_back(i);
            } else {
                far_cloud.push_back(p);
                far_idx.push_back(i);
            }
        }

        pcl::PointCloud<pcl::PointXYZ> filtered_objects;
        std::vector<int> filtered_indices;

        // 近处：严格过滤（mean_k=10, threshold=2.0）
        if (near_cloud.size() > 30) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = near_cloud.makeShared();
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_ptr);
            sor.setMeanK(10);
            sor.setStddevMulThresh(2.0);
            pcl::PointCloud<pcl::PointXYZ> result;
            sor.filter(result);
            if (result.size() > near_cloud.size() / 2) {
                for (const auto& p : result.points) filtered_objects.push_back(p);
            } else {
                for (const auto& p : near_cloud.points) filtered_objects.push_back(p);
            }
        } else {
            for (const auto& p : near_cloud.points) filtered_objects.push_back(p);
        }

        // 中距离：适中过滤（mean_k=8, threshold=2.5）
        if (mid_cloud.size() > 20) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = mid_cloud.makeShared();
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_ptr);
            sor.setMeanK(8);
            sor.setStddevMulThresh(2.5);
            pcl::PointCloud<pcl::PointXYZ> result;
            sor.filter(result);
            if (result.size() > mid_cloud.size() / 2) {
                for (const auto& p : result.points) filtered_objects.push_back(p);
            } else {
                for (const auto& p : mid_cloud.points) filtered_objects.push_back(p);
            }
        } else {
            for (const auto& p : mid_cloud.points) filtered_objects.push_back(p);
        }

        // 远处：宽松过滤（mean_k=5, threshold=3.0）
        if (far_cloud.size() > 10) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr = far_cloud.makeShared();
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_ptr);
            sor.setMeanK(5);
            sor.setStddevMulThresh(3.0);
            pcl::PointCloud<pcl::PointXYZ> result;
            sor.filter(result);
            if (result.size() > far_cloud.size() / 2) {
                for (const auto& p : result.points) filtered_objects.push_back(p);
            } else {
                for (const auto& p : far_cloud.points) filtered_objects.push_back(p);
            }
        } else {
            for (const auto& p : far_cloud.points) filtered_objects.push_back(p);
        }

        if (filtered_objects.size() > objects_out.size() / 2) {
            ROS_DEBUG("[Adaptive SOR] near=%lu mid=%lu far=%lu -> filtered=%lu (orig=%lu)",
                      near_cloud.size(), mid_cloud.size(), far_cloud.size(),
                      filtered_objects.size(), objects_out.size());
            objects_out = filtered_objects;
        }
    }

    // 第五步：自适应 BEV 网格清理——根据距离动态调整高度阈值
    // 远处点云稀疏，降低 min_height 要求；近处保持较高要求去除噪声
    if (objects_out.size() > 50) {
        const double bev_cell_size = 0.15;

        struct BevKey { int x, y; bool operator<(const BevKey& o) const { return x<o.x||(x==o.x&&y<o.y); } };
        std::map<BevKey, float> bev_max_h;   // 每个 cell 的最大高度
        std::map<BevKey, float> bev_dist;    // 每个 cell 的平均距离
        std::map<BevKey, std::vector<int>> bev_indices;

        // 需要每个 objects 点的局部地面高度，先重建 cell_ground_z 查询
        // 复用上面已有的 cell_ground_z（按 grid_cell_size_ 网格）
        for (int i = 0; i < (int)objects_out.size(); ++i) {
            const auto& p = objects_out.points[i];
            BevKey bk{(int)std::floor(p.x / bev_cell_size), (int)std::floor(p.y / bev_cell_size)};
            bev_indices[bk].push_back(i);

            // 查找该点所在大格子的局部地面高度
            CellKey ck{(int)std::floor(p.x / grid_cell_size_), (int)std::floor(p.y / grid_cell_size_)};
            auto it = cell_ground_z.find(ck);
            float local_gz = (it != cell_ground_z.end()) ? it->second : 0;
            float h = p.z - local_gz;
            if (bev_max_h.find(bk) == bev_max_h.end() || h > bev_max_h[bk]) {
                bev_max_h[bk] = h;
            }

            // 记录距离（用于自适应阈值）
            float dist = std::sqrt(p.x * p.x + p.y * p.y);
            if (bev_dist.find(bk) == bev_dist.end()) {
                bev_dist[bk] = dist;
            } else {
                bev_dist[bk] = (bev_dist[bk] + dist) / 2.0f;  // 平均距离
            }
        }

        // 自适应阈值：近处 0.35m，中距离 0.25m，远处 0.15m
        auto getAdaptiveMinHeight = [](float dist) -> float {
            if (dist < 10.0f) return 0.35f;
            if (dist < 20.0f) return 0.25f;
            return 0.15f;
        };

        // 只保留 max_height >= adaptive_min_height 的 cell
        pcl::PointCloud<pcl::PointXYZ> cleaned;
        int removed_cells = 0, kept_cells = 0;
        int near_removed = 0, mid_removed = 0, far_removed = 0;
        for (auto& [bk, indices] : bev_indices) {
            float dist = bev_dist[bk];
            float min_height = getAdaptiveMinHeight(dist);
            if (bev_max_h[bk] >= min_height) {
                for (int idx : indices) {
                    cleaned.push_back(objects_out.points[idx]);
                }
                kept_cells++;
            } else {
                removed_cells++;
                if (dist < 10.0f) near_removed++;
                else if (dist < 20.0f) mid_removed++;
                else far_removed++;
            }
        }

        // 如果清理后点数不过少（>40%），使用清理结果
        if (cleaned.size() > objects_out.size() * 0.4) {
            ROS_DEBUG("[Adaptive BEV] removed %d cells (near=%d mid=%d far=%d), kept %d cells, points: %lu -> %lu",
                      removed_cells, near_removed, mid_removed, far_removed,
                      kept_cells, objects_out.size(), cleaned.size());
            objects_out = cleaned;
        }
    }
}

void NdtSlamNode::addKeyFrameToLoopClosure(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                        const Sophus::SE3d& pose,
                                        const ros::Time& stamp) {
    // ========== 统一门控：observe_only / 内存保护 / 磁盘保护 / NDT 健康 ==========
    if (longterm_mapping_enabled_ && !canCommit()) {
        ROS_WARN_THROTTLE(5.0, "[CommitGate] BLOCKED: commit_enabled=%s mem_pause=%s disk_guard=%s ndt_bad=%s",
                          commit_enabled_ ? "true" : "false",
                          mapping_paused_by_memory_guard_ ? "true" : "false",
                          disk_guard_triggered_ ? "true" : "false",
                          ndt_health_bad_ ? "true" : "false");
        return;
    }

    // pose 已经在调用前被约束过了，直接使用

    // ========== MotionGate：静止时不添加关键帧 ==========
    if (longterm_mapping_enabled_ && motion_gate_enabled_) {
        if (!evaluateMotionGateForMapCommit(pose, stamp)) {
            ROS_DEBUG("[MotionGate] Stationary, skipping keyframe commit");
            return;
        }
        ROS_DEBUG("[MotionGate] Moved enough, committing keyframe (trans=%.2fm, rot=%.1fdeg)",
                 delta_translation_, delta_yaw_);
    }

    *last_cloud_ = *cloud;

    // ========== P0-5: 此函数已废弃 ==========
    // 已迁移到 commitKeyFrameWithDynamicFiltering()
    // 此函数保留用于参考，不再调用 addKeyFrame

    // 旧代码已禁用 - 使用 commitKeyFrameWithDynamicFiltering 替代
    ROS_WARN_THROTTLE(5.0, "[DEPRECATED] addKeyFrameToLoopClosure called - should use commitKeyFrameWithDynamicFiltering");
    return;
}
#if 0
    // 以下旧代码保留用于参考，不再编译
    if (false) {
        keyframe_count_++;
        Eigen::Vector3d pos = pose.translation();
        ROS_INFO("[MapCommit] keyframe #%d added | pos=(%.1f, %.1f, %.1f) | tiles=%d",
                 keyframe_count_, pos.x(), pos.y(), pos.z(), flushed_tile_count_);

        std::lock_guard<std::mutex> lock(map_mutex_);
        Eigen::Matrix4d transform = pose.matrix();
        pcl::PointCloud<pcl::PointXYZ> transformed;
        pcl::transformPointCloud(*cloud, transformed, transform.cast<float>());

        // 动态点过滤：每3个关键帧执行一次SOR，减少计算开销
        static int sor_counter = 0;
        sor_counter++;
        if (sor_counter % 3 == 1) {
            auto filtered_for_map = filterDynamicPoints(transformed.makeShared());
            transformed = *filtered_for_map;
        }

        // 使用网格局部地面模型分割当前关键帧
        // 注意：这里在 map 坐标系下分割，但 channel filter 需要 base_link 坐标系
        // 由于 cloud 本身是 base_link 下的原始点云，我们先在 base_link 下做 channel filter
        // 再变换到 map 坐标系

        // ========== BasePayloadChannelFilter（base_link 下吊货候选筛选）==========
        pcl::PointCloud<pcl::PointXYZ>::Ptr kf_safe_objects(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr kf_payload_candidates(new pcl::PointCloud<pcl::PointXYZ>);

        if (channel_filter_config_.enabled) {
            // 在 base_link 下做通道过滤（cloud 是 base_link 下的原始点云）
            // 先分割地面/非地面
            pcl::PointCloud<pcl::PointXYZ> base_ground, base_objects;
            separateGroundByGrid(*cloud, base_ground, base_objects);

            // 用空 ground_model（channel filter 内部自行处理 HAG）
            std::map<CellKey, float> empty_ground_model;
            ChannelFilterResult ch_result = channel_filter_.filter(base_objects.makeShared(), empty_ground_model);

            kf_safe_objects = ch_result.safe_objects;
            kf_payload_candidates = ch_result.payload_candidates;

            // ========== PayloadTrackManager 双坐标系跟踪 ==========
            // 将候选传入跟踪器，在 base_link 下跟踪，map 下判断动态
            TrackResult track_result;
            if (payload_tracker_config_.enabled && !kf_payload_candidates->empty()) {
                track_result = payload_tracker_.update(
                    kf_payload_candidates, transform, stamp.toSec(), empty_ground_model);

                // P0.5: 使用 CargoBoxEstimator 计算货物框
                if (cargo_box_estimator_config_.enabled) {
                    // 构建简单地面模型
                    SimpleGroundModel ground_model;
                    ground_model.global_z_min = 0.0f;  // 使用默认值
                    ground_model.resolution = 1.5f;

                    // 对每个活跃的吊货 track 计算货物框
                    // P0-2: 使用可修改的 tracks 列表，支持 per-track size jump 检查
                    auto& tracks = payload_tracker_.getMutableTracks();
                    for (auto& t : tracks) {
                        if (t.state == TrackState::EXPIRED) continue;
                        if (t.cloud_history.empty()) continue;

                        // 使用最新的点云
                        auto cluster = t.cloud_history.back();

                        // P0-7: 传递上一帧的 core_box 信息（用于 track 一致性评分）
                        const CargoBox* prev_core_box = t.has_last_core_box ? &t.last_core_box : nullptr;

                        CargoBox core_box, remove_box, forbidden_box;
                        if (cargo_box_estimator_.estimateCargoBox(cluster, ground_model,
                                                                   core_box, remove_box, forbidden_box,
                                                                   prev_core_box)) {
                            // P0-2: per-track size jump 检查
                            bool size_jump_rejected = false;
                            if (t.has_last_size) {
                                float growth_x = core_box.size.x() / std::max(t.last_core_size.x(), 0.1f);
                                float growth_y = core_box.size.y() / std::max(t.last_core_size.y(), 0.1f);
                                float growth_z = core_box.size.z() / std::max(t.last_core_size.z(), 0.1f);
                                float max_growth = std::max({growth_x, growth_y, growth_z});

                                if (max_growth > cargo_box_estimator_config_.max_size_growth_ratio) {
                                    ROS_WARN("[CargoBoxV2] track=%d rejected by size jump: growth=%.2f > %.2f",
                                             t.track_id, max_growth, cargo_box_estimator_config_.max_size_growth_ratio);
                                    size_jump_rejected = true;
                                }
                            }

                            if (!size_jump_rejected) {
                                // P0-2: 更新 per-track size 历史
                                t.last_core_size = core_box.size;
                                t.has_last_size = true;

                                // P0-7: 更新上一帧的 core_box 信息
                                t.last_core_box = core_box;
                                t.has_last_core_box = true;

                                ROS_DEBUG("[CargoBoxV2] track=%d core_pts=%d bottom_hag=%.2f size=(%.2f,%.2f,%.2f) "
                                         "components=%d selected=%d",
                                         t.track_id, core_box.suspended_points, core_box.bottom_hag,
                                         core_box.size.x(), core_box.size.y(), core_box.size.z(),
                                         core_box.component_count, core_box.component_id);

                                // 发布调试点云
                                static int cargo_debug_count = 0;
                                cargo_debug_count++;
                                if (cargo_debug_count % 20 == 1) {
                                    // 发布 core points
                                    auto core_pts = cargo_box_estimator_.getCorePointsCloud();
                                    if (core_pts && !core_pts->empty()) {
                                        sensor_msgs::PointCloud2 msg;
                                        pcl::toROSMsg(*core_pts, msg);
                                        msg.header.stamp = stamp;
                                        msg.header.frame_id = "base_link";
                                        cargo_core_points_pub_.publish(msg);
                                    }

                                    // 发布 HAG filtered cloud
                                    auto hag_cloud = cargo_box_estimator_.getHagFilteredCloud();
                                    if (hag_cloud && !hag_cloud->empty()) {
                                        sensor_msgs::PointCloud2 msg;
                                        pcl::toROSMsg(*hag_cloud, msg);
                                        msg.header.stamp = stamp;
                                        msg.header.frame_id = "base_link";
                                        cargo_hag_filtered_pub_.publish(msg);
                                    }

                                    // 发布 components cloud
                                    auto comp_cloud = cargo_box_estimator_.getComponentsCloud();
                                    if (comp_cloud && !comp_cloud->empty()) {
                                        sensor_msgs::PointCloud2 msg;
                                        pcl::toROSMsg(*comp_cloud, msg);
                                        msg.header.stamp = stamp;
                                        msg.header.frame_id = "base_link";
                                        cargo_components_pub_.publish(msg);
                                    }
                                }
                            }
                        } else {
                            ROS_DEBUG("[CargoBoxV2] track=%d rejected: reason=%d",
                                      t.track_id, (int)core_box.reject_reason);
                        }
                    }
                }

                // P1: 将动态吊货的 remove_box 写入 cargo deny history
                for (const auto& t : payload_tracker_.getTracks()) {
                    if (t.state == TrackState::DYNAMIC_PAYLOAD ||
                        t.state == TrackState::SUSPENDED_MOVING) {
                        // 使用 track 的 bbox 作为 deny 区域
                        Eigen::Vector3d bbox_min = t.bbox_min_map.cast<double>();
                        Eigen::Vector3d bbox_max = t.bbox_max_map.cast<double>();

                        // 扩展一点（与 remove_expand_xy 一致）
                        bbox_min.x() -= 0.25;
                        bbox_min.y() -= 0.25;
                        bbox_min.z() -= 0.05;
                        bbox_max.x() += 0.25;
                        bbox_max.y() += 0.25;
                        bbox_max.z() += 0.20;

                        addCargoDenyCells(bbox_min, bbox_max, stamp.toSec());
                    }
                }

                // 清理过期的 cargo deny cells
                cleanupExpiredCargoDenyCells(stamp.toSec());

                // 跟踪器确认的动态点不进地图
                // 跟踪器确认的 pending 点也不进地图（等待确认）
                // 只有 safe_objects 进地图

                // ========== DynamicEventManager：吊货动态事件 ==========
                if (dynamic_event_config_.enabled) {
                    for (const auto& t : payload_tracker_.getTracks()) {
                        if (t.state == TrackState::DYNAMIC_PAYLOAD ||
                            t.state == TrackState::PENDING_STATIC) {
                            Box3D bbox;
                            bbox.min_pt = t.bbox_min_map.cast<double>();
                            bbox.max_pt = t.bbox_max_map.cast<double>();
                            Eigen::Vector3d centroid_d = t.centroid_map.cast<double>();

                            // ========== PlacedCargoSuppressor ==========
                            if (dynamic_event_manager_.shouldSuppressNewSession(centroid_d, bbox)) {
                                static int suppress_count = 0;
                                suppress_count++;
                                if (suppress_count % 10 == 1) {
                                    ROS_INFO("[PlacedCargoSuppressor] suppress track=%d, reason=inside_placed_bbox, "
                                             "centroid=(%.2f,%.2f,%.2f), bbox=(%.1f,%.1f,%.1f)-(%.1f,%.1f,%.1f), "
                                             "placed_sessions=%zu",
                                             t.track_id,
                                             centroid_d.x(), centroid_d.y(), centroid_d.z(),
                                             bbox.min_pt.x(), bbox.min_pt.y(), bbox.min_pt.z(),
                                             bbox.max_pt.x(), bbox.max_pt.y(), bbox.max_pt.z(),
                                             dynamic_event_manager_.getPlacedSessions().size());
                                }
                                continue;  // 跳过，不创建 session
                            }

                            int event_id = dynamic_event_manager_.findOrCreatePayloadSession(
                                t.track_id, stamp.toSec(), centroid_d, bbox, t.velocity);

                            // 调试日志（每次都打印）
                            ROS_INFO("[PayloadSessionDebug] track=%d state=%d event_id=%d vel=%.3f disp=%.3f",
                                     t.track_id, (int)t.state, event_id, t.velocity, t.map_displacement);

                            if (event_id >= 0 && t.state == TrackState::DYNAMIC_PAYLOAD) {
                                dynamic_event_manager_.updatePayloadSession(
                                    event_id, stamp.toSec(), centroid_d, bbox,
                                    t.velocity, t.map_displacement);
                                dynamic_event_manager_.confirmPayloadSession(event_id, stamp.toSec());
                            }
                        }
                    }
                }

                // 发布动态和 pending 点云（调试用）
                static int track_debug_count = 0;
                track_debug_count++;
                if (track_debug_count % 5 == 1) {
                    ROS_DEBUG("[PayloadTrack] tracks=%d, dynamic=%d, pending=%d",
                             track_result.active_tracks, track_result.dynamic_tracks, track_result.pending_tracks);

                    // 输出每个活跃轨迹的状态
                    for (const auto& t : payload_tracker_.getTracks()) {
                        if (t.state != TrackState::EXPIRED) {
                            ROS_DEBUG("[PayloadTrack]   id=%d state=%d base_std=%.2f map_disp=%.2f vel=%.2f",
                                     t.track_id, (int)t.state, t.base_center_std,
                                     t.map_displacement, t.velocity);
                        }
                    }

                    if (!track_result.dynamic_payload->empty()) {
                        sensor_msgs::PointCloud2 dyn_msg;
                        pcl::toROSMsg(*track_result.dynamic_payload, dyn_msg);
                        dyn_msg.header.stamp = stamp;
                        dyn_msg.header.frame_id = "base_link";
                        payload_dynamic_pub_.publish(dyn_msg);
                    }

                    if (!track_result.pending->empty()) {
                        sensor_msgs::PointCloud2 pend_msg;
                        pcl::toROSMsg(*track_result.pending, pend_msg);
                        pend_msg.header.stamp = stamp;
                        pend_msg.header.frame_id = "base_link";
                        payload_pending_pub_.publish(pend_msg);
                    }

                    // 发布吊货候选点云（/suspended_payload_candidate_cloud）
                    if (!kf_payload_candidates->empty()) {
                        sensor_msgs::PointCloud2 cand_msg;
                        pcl::toROSMsg(*kf_payload_candidates, cand_msg);
                        cand_msg.header.stamp = stamp;
                        cand_msg.header.frame_id = "base_link";
                        payload_candidate_pub_.publish(cand_msg);
                    }

                    // 发布吊货点云（/suspended_payload_cloud）= dynamic_payload + suspended_moving
                    pcl::PointCloud<pcl::PointXYZ>::Ptr suspended_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    *suspended_cloud += *track_result.dynamic_payload;
                    // 注意：suspended_moving 点已经在 dynamic_payload 中（P0-1 修改）
                    if (!suspended_cloud->empty()) {
                        sensor_msgs::PointCloud2 suspended_msg;
                        pcl::toROSMsg(*suspended_cloud, suspended_msg);
                        suspended_msg.header.stamp = stamp;
                        suspended_msg.header.frame_id = "base_link";
                        payload_dynamic_pub_.publish(suspended_msg);  // 复用 payload_dynamic_pub_
                    }

                    // 发布被拒绝进入地图的吊货点（/cargo_dynamic_removed_cloud）
                    // 这些点是吊货跟踪器确认的动态点，不应该进入 permanent map
                    if (!track_result.dynamic_payload->empty()) {
                        sensor_msgs::PointCloud2 removed_msg;
                        pcl::toROSMsg(*track_result.dynamic_payload, removed_msg);
                        removed_msg.header.stamp = stamp;
                        removed_msg.header.frame_id = "base_link";
                        cargo_dynamic_removed_pub_.publish(removed_msg);
                        ROS_DEBUG("[CargoRemoved] dynamic_payload points=%zu", track_result.dynamic_payload->size());
                    }
                }
            }

            // ========== HumanObjectDynamicFilter（人体动态过滤）==========
            pcl::PointCloud<pcl::PointXYZ>::Ptr kf_human_safe_objects(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr kf_human_candidates(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr kf_human_dynamic(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr kf_human_pending(new pcl::PointCloud<pcl::PointXYZ>);

            if (human_filter_config_.enabled) {
                // 在 base_link 下做人人体过滤（kf_safe_objects 是 base_link 下的）
                double timestamp = stamp.toSec();
                human_filter_.processFrame(kf_safe_objects, transform, timestamp,
                                           kf_human_safe_objects, kf_human_candidates,
                                           kf_human_dynamic, kf_human_pending);

                // ========== DynamicEventManager：人体动态事件 ==========
                if (dynamic_event_config_.enabled && !kf_human_dynamic->empty()) {
                    // 获取动态人体的跟踪信息
                    auto active_tracks = human_filter_.getActiveTrackCount();
                    auto dynamic_count = human_filter_.getDynamicHumanCount();
                    if (dynamic_count > 0) {
                        // 创建人体事件（简化版：使用当前帧的 centroid）
                        Eigen::Vector4f centroid_4f;
                        pcl::compute3DCentroid(*kf_human_dynamic, centroid_4f);
                        Eigen::Vector3d centroid = centroid_4f.head<3>().cast<double>();

                        std::deque<Eigen::Vector3d> history;
                        history.push_back(centroid);

                        // 手动计算 z 范围
                        float z_min = 1e9, z_max = -1e9;
                        for (const auto& pt : kf_human_dynamic->points) {
                            if (pt.z < z_min) z_min = pt.z;
                            if (pt.z > z_max) z_max = pt.z;
                        }

                        int event_id = dynamic_event_manager_.createHumanEvent(
                            timestamp, timestamp, history, z_min, z_max);
                        ROS_INFO("[DynamicEvent] HumanEvent created: id=%d, points=%zu",
                                 event_id, kf_human_dynamic->size());
                    }
                }

                // 发布人体过滤调试话题（每 5 个关键帧一次）
                static int kf_hf_debug_count = 0;
                kf_hf_debug_count++;
                if (kf_hf_debug_count % 5 == 1) {
                    ROS_DEBUG("[HumanFilter-KF] input=%lu, safe=%lu, candidate=%lu, dynamic=%lu, pending=%lu",
                             kf_safe_objects->size(), kf_human_safe_objects->size(),
                             kf_human_candidates->size(), kf_human_dynamic->size(), kf_human_pending->size());

                    if (!kf_human_candidates->empty()) {
                        sensor_msgs::PointCloud2 cand_msg;
                        pcl::toROSMsg(*kf_human_candidates, cand_msg);
                        cand_msg.header.stamp = stamp;
                        cand_msg.header.frame_id = "base_link";
                        human_candidate_pub_.publish(cand_msg);
                    }

                    if (!kf_human_dynamic->empty()) {
                        sensor_msgs::PointCloud2 dyn_msg;
                        pcl::toROSMsg(*kf_human_dynamic, dyn_msg);
                        dyn_msg.header.stamp = stamp;
                        dyn_msg.header.frame_id = "map";
                        human_dynamic_pub_.publish(dyn_msg);
                    }

                    if (!kf_human_pending->empty()) {
                        sensor_msgs::PointCloud2 pend_msg;
                        pcl::toROSMsg(*kf_human_pending, pend_msg);
                        pend_msg.header.stamp = stamp;
                        pend_msg.header.frame_id = "map";
                        human_pending_pub_.publish(pend_msg);
                    }
                }
            } else {
                kf_human_safe_objects = kf_safe_objects;
            }

            // ========== 保存 raw/filtered/ground 到关键帧 ==========
            auto& kf_deque = const_cast<std::deque<KeyFrame>&>(loop_closure_detector_.getKeyFrames());
            if (!kf_deque.empty()) {
                auto& kf = kf_deque.back();
                kf.objects_raw = base_objects.makeShared();
                kf.objects_filtered = kf_human_safe_objects;
                kf.ground_points = base_ground.makeShared();
                kf.dirty_dynamic = false;  // 刚过滤完，不需要重新过滤
                ROS_DEBUG("[KeyFrame] id=%lu raw=%zu filtered=%zu ground=%zu",
                         kf.id_, kf.objects_raw->size(), kf.objects_filtered->size(),
                         kf.ground_points->size());
            }

            // 变换 safe_objects 到 map 坐标系
            pcl::PointCloud<pcl::PointXYZ> safe_transformed;
            pcl::transformPointCloud(*kf_human_safe_objects, safe_transformed, transform.cast<float>());

            // 变换 payload_candidates 到 map 坐标系（用于调试发布）
            pcl::PointCloud<pcl::PointXYZ> candidates_transformed;
            if (!kf_payload_candidates->empty()) {
                pcl::transformPointCloud(*kf_payload_candidates, candidates_transformed, transform.cast<float>());
            }

            // 地面也变换到 map
            pcl::PointCloud<pcl::PointXYZ> ground_transformed;
            pcl::transformPointCloud(base_ground, ground_transformed, transform.cast<float>());

            // 范围裁剪并加入各层地图
            // 关键：global_map_ 和 display_map_ 只接收 safe_objects + ground
            auto addInRange = [&](const pcl::PointCloud<pcl::PointXYZ>& src,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr dst) {
                for (const auto& p : src.points) {
                    if (std::abs(p.x) <= max_map_size_ &&
                        std::abs(p.y) <= max_map_size_ &&
                        std::abs(p.z) <= max_map_size_ &&
                        std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                        dst->push_back(p);
                    }
                }
            };

            addInRange(safe_transformed, global_map_);       // registration map: safe_objects only
            addInRange(safe_transformed, display_map_);      // display map: safe_objects
            addInRange(ground_transformed, display_map_);    // display map: also include ground
            addInRange(ground_transformed, ground_map_);     // ground map
            addInRange(safe_transformed, objects_map_);      // objects map: safe_objects only

            // ========== 长期建图：写入 tiles ==========
            if (longterm_mapping_enabled_ && persistent_map_enabled_ && canCommit()) {
                // 计算 tile 索引
                Eigen::Vector3d kf_pos = pose.translation();
                int tile_x = std::floor(kf_pos.x() / tile_size_m_);
                int tile_y = std::floor(kf_pos.y() / tile_size_m_);
                std::string tile_key = "x" + std::to_string(tile_x) + "_y" + std::to_string(tile_y);

                // 初始化 tile layers（如果不存在）
                if (dirty_tiles_.find(tile_key) == dirty_tiles_.end()) {
                    dirty_tiles_[tile_key].registration.reset(new pcl::PointCloud<pcl::PointXYZ>);
                    dirty_tiles_[tile_key].display.reset(new pcl::PointCloud<pcl::PointXYZ>);
                    dirty_tiles_[tile_key].ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
                    dirty_tiles_[tile_key].objects.reset(new pcl::PointCloud<pcl::PointXYZ>);
                }

                // 添加到各层
                *dirty_tiles_[tile_key].registration += safe_transformed;
                *dirty_tiles_[tile_key].display += safe_transformed;
                *dirty_tiles_[tile_key].display += ground_transformed;
                *dirty_tiles_[tile_key].ground += ground_transformed;
                *dirty_tiles_[tile_key].objects += safe_transformed;

                dirty_tile_count_ = dirty_tiles_.size();

                ROS_DEBUG("[PersistentMap] Added points to tile %s layers, dirty_tiles=%d",
                          tile_key.c_str(), dirty_tile_count_);
            }

            // 发布 debug 话题
            static int kf_ch_debug_count = 0;
            kf_ch_debug_count++;
            if (kf_ch_debug_count % 5 == 1) {
                ROS_DEBUG("[PayloadChannel-KF] channel=%d, candidate=%d, safe=%d, clusters=%d",
                         ch_result.channel_points, ch_result.candidate_points,
                         ch_result.safe_points, ch_result.candidate_clusters);

                if (!candidates_transformed.empty()) {
                    sensor_msgs::PointCloud2 cand_msg;
                    pcl::toROSMsg(candidates_transformed, cand_msg);
                    cand_msg.header.stamp = stamp;
                    cand_msg.header.frame_id = map_frame_;
                    payload_candidate_pub_.publish(cand_msg);
                }
            }
        } else {
            // 通道过滤未启用，使用旧逻辑
            pcl::PointCloud<pcl::PointXYZ> kf_ground, kf_objects;
            separateGroundByGrid(transformed, kf_ground, kf_objects);

            // 旧的 payload 过滤（保留兼容）
            static PointCloudProcessing payload_filter;
            static ros::Publisher payload_pub = nh_.advertise<sensor_msgs::PointCloud2>("/payload_dynamic_cloud", 10);
            pcl::PointCloud<pcl::PointXYZ>::Ptr payload_removed(new pcl::PointCloud<pcl::PointXYZ>);
            auto kf_objects_filtered = payload_filter.filterPayloadByTracking(
                kf_objects.makeShared(), transform, payload_removed);

            if (payload_removed->size() > 0) {
                sensor_msgs::PointCloud2 payload_msg;
                pcl::toROSMsg(*payload_removed, payload_msg);
                payload_msg.header.stamp = stamp;
                payload_msg.header.frame_id = map_frame_;
                payload_pub.publish(payload_msg);
            }

            kf_objects = *kf_objects_filtered;

            auto addInRange = [&](const pcl::PointCloud<pcl::PointXYZ>& src,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr dst) {
                for (const auto& p : src.points) {
                    if (std::abs(p.x) <= max_map_size_ &&
                        std::abs(p.y) <= max_map_size_ &&
                        std::abs(p.z) <= max_map_size_ &&
                        std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                        dst->push_back(p);
                    }
                }
            };
            addInRange(transformed, global_map_);
            addInRange(transformed, display_map_);
            addInRange(kf_ground, ground_map_);
            addInRange(kf_objects, objects_map_);
        }

        // 粗地图体素滤波（配准用，较大体素）
        if (use_voxel_filter_ && global_map_->size() > 10000) {
            pcl::VoxelGrid<pcl::PointXYZ> vf;
            vf.setInputCloud(global_map_);
            vf.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            pcl::PointCloud<pcl::PointXYZ> f;
            vf.filter(f);
            *global_map_ = f;
        }

        // 全量显示地图体素滤波
        if (use_voxel_filter_ && display_map_->size() > 10000) {
            pcl::VoxelGrid<pcl::PointXYZ> vf;
            vf.setInputCloud(display_map_);
            vf.setLeafSize(display_voxel_size_, display_voxel_size_, display_voxel_size_);
            pcl::PointCloud<pcl::PointXYZ> f;
            vf.filter(f);
            *display_map_ = f;
        }

        // 地面地图体素滤波（较粗，控制点数）
        if (use_voxel_filter_ && ground_map_->size() > 5000) {
            pcl::VoxelGrid<pcl::PointXYZ> vf;
            vf.setInputCloud(ground_map_);
            vf.setLeafSize(ground_voxel_size_, ground_voxel_size_, ground_voxel_size_);
            pcl::PointCloud<pcl::PointXYZ> f;
            vf.filter(f);
            *ground_map_ = f;
        }

        // 非地面/货物地图体素滤波（很细，保留轮廓）
        if (use_voxel_filter_ && objects_map_->size() > 5000) {
            pcl::VoxelGrid<pcl::PointXYZ> vf;
            vf.setInputCloud(objects_map_);
            vf.setLeafSize(objects_voxel_size_, objects_voxel_size_, objects_voxel_size_);
            pcl::PointCloud<pcl::PointXYZ> f;
            vf.filter(f);
            *objects_map_ = f;
        }

        // 更新 BEV 观测计数（时间一致性）
        // 放宽条件：只要 objects_map_ 有足够数据就更新观测计数
        // 不再要求 refined_pose_high_quality_（该条件太严格，几乎从不满足）
        if (objects_map_->size() > 50) {
            const double clean_bev_cell = 0.15;
            std::set<BevKey> seen_cells;
            for (const auto& p : objects_map_->points) {
                BevKey bk{(int)std::floor(p.x / clean_bev_cell), (int)std::floor(p.y / clean_bev_cell)};
                seen_cells.insert(bk);
            }
            for (const auto& bk : seen_cells) {
                bev_observation_count_[bk]++;
            }
        }

        publishMap();

        // 显示地图每3个关键帧更新一次，解耦实时处理和可视化
        static int display_publish_counter = 0;
        display_publish_counter++;
        if (display_publish_counter >= 3) {
            display_publish_counter = 0;
            publishDisplayMap();
            publishGroundMap();
            publishObjectsMap();

            // clean map 异步构建（不阻塞主处理线程）
            if (!clean_rebuild_running_.load()) {
                if (clean_rebuild_thread_.joinable()) clean_rebuild_thread_.join();
                clean_rebuild_running_.store(true);
                clean_rebuild_thread_ = std::thread([this]() {
                    rebuildCleanMap();
                    clean_rebuild_running_.store(false);
                });
            }
        }
    }

    if (keyframe_count_ % loop_detection_interval_ == 0) {
        ROS_DEBUG("Performing loop closure detection...");
        processLoopClosure();
    }
}
#endif  // 旧代码结束

void NdtSlamNode::publishMap() {
    if (global_map_->empty()) {
        ROS_DEBUG("Global map is empty, skipping publish");
        return;
    }

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*global_map_, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = map_frame_;
    map_pub_.publish(map_msg);
}

void NdtSlamNode::publishCurrentCloud() {
    if (current_cloud_->empty()) return;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*current_cloud_, cloud_msg);
    cloud_msg.header.stamp = last_stamp_;
    cloud_msg.header.frame_id = map_frame_;
    current_cloud_pub_.publish(cloud_msg);
}

void NdtSlamNode::processLoopClosure() {
    LoopCandidate candidate = loop_closure_detector_.detectLoop();

    if (candidate.current_keyframe_id != -1 && candidate.candidate_keyframe_id != -1) {
        std::pair<int, int> loop_pair = {candidate.candidate_keyframe_id, candidate.current_keyframe_id};
        if (processed_loops_.find(loop_pair) != processed_loops_.end()) {
            ROS_DEBUG("Loop already processed: %d <-> %d", loop_pair.first, loop_pair.second);
            return;
        }

        ROS_INFO("Loop found: current_keyframe=%d <-> candidate_keyframe=%d",
                 candidate.current_keyframe_id, candidate.candidate_keyframe_id);

        processed_loops_.insert(loop_pair);

        const auto& keyframes = loop_closure_detector_.getKeyFrames();
        for (const auto& keyframe : keyframes) {
            pose_graph_optimizer_.addKeyFrame(keyframe);
        }

        for (size_t i = 0; i < keyframes.size() - 1; ++i) {
            const KeyFrame& kf1 = keyframes[i];
            const KeyFrame& kf2 = keyframes[i + 1];
            Sophus::SE3d relative_pose = kf1.pose_.inverse() * kf2.pose_;
            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
            pose_graph_optimizer_.addOdometryEdge(kf1.id_, kf2.id_, relative_pose, information);
        }

        Eigen::Matrix<double, 6, 6> loop_information = Eigen::Matrix<double, 6, 6>::Identity();
        pose_graph_optimizer_.addLoopEdge(candidate.candidate_keyframe_id, candidate.current_keyframe_id,
                                          candidate.relative_pose, loop_information);

        if (pose_graph_optimizer_.optimize(10)) {
            ROS_INFO("Pose graph optimized successfully");
            std::vector<KeyFrame> updated_keyframes(keyframes.begin(), keyframes.end());
            pose_graph_optimizer_.updateKeyFramePoses(updated_keyframes);

            if (!updated_keyframes.empty()) {
                const auto& last_keyframe = updated_keyframes.back();
                ROS_INFO("Updating pose from loop closure: (%.3f, %.3f, %.3f)",
                         last_keyframe.pose_.translation().x(),
                         last_keyframe.pose_.translation().y(),
                         last_keyframe.pose_.translation().z());
                updatePoseFromLoopClosure(last_keyframe.pose_);
            }

            loop_closure_detector_.updateKeyFramePoses(updated_keyframes);
            asyncRebuildGlobalMap();
        } else {
            ROS_WARN("Pose graph optimization failed");
        }
    }
}

void NdtSlamNode::updatePoseFromLoopClosure(const Sophus::SE3d& new_pose) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    if (use_lidar2base_transform_) {
        Eigen::Matrix4d lidar2base = lidar2base_transform_;

        Eigen::Matrix3d R = lidar2base.block<3, 3>(0, 0);
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();

        if (R_ortho.determinant() < 0) {
            R_ortho.col(0) *= -1;
        }

        Sophus::SE3d lidar2base_se3;
        lidar2base_se3.so3() = Sophus::SO3d(R_ortho);
        lidar2base_se3.translation() = lidar2base.block<3, 1>(0, 3);

        Sophus::SE3d base2lidar = lidar2base_se3.inverse();
        current_pose_ = new_pose * base2lidar;
    } else {
        current_pose_ = new_pose;
    }

    relocalized_pose_ = current_pose_;
    tracking_lost_ = false;
    tracking_cv_.notify_all();
}

bool NdtSlamNode::resetService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    ROS_INFO("Resetting SLAM system...");

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        current_pose_ = Sophus::SE3d();
        initialized_ = false;
        tracking_lost_ = false;
    }

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        global_map_->clear();
        display_map_->clear();
        ground_map_->clear();
        objects_map_->clear();
        objects_clean_map_->clear();
        current_cloud_->clear();
        local_map_->clear();
        local_map_version_ = 0;
    }

    // V3: 清理 Localization Target
    {
        std::lock_guard<std::mutex> lock(localization_target_mutex_);
        localization_target_front_->clear();
        localization_target_back_->clear();
        localization_target_snapshot_->clear();
        localization_target_version_ = 0;
        localization_target_snapshot_version_ = 0;
        localization_target_ready_ = false;
        localization_target_state_ = LocalizationTargetState::BOOTSTRAP_LOCAL_MAP;
        cached_target_valid_ = false;
        cached_target_version_ = 0;
        cached_target_points_ = 0;
        crop_frames_since_update_ = 0;
        last_bound_ndt_target_.reset();
        last_bound_ndt_target_version_ = 0;
        last_bound_ndt_target_source_ = "none";
        last_actual_target_source_ = "bootstrap_local_map";
        last_target_reason_ = "reset";
        target_version_ = 0;
        target_rebuild_count_ = 0;
        setInputTarget_count_ = 0;
    }

    // 重置 NDT_OMP（使用配置参数，而非硬编码）
    ndt_.reset(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_->setResolution(ndt_resolution_);
    ndt_->setStepSize(ndt_step_size_);
    ndt_->setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_->setMaximumIterations(ndt_max_iterations_);

    // V3: 设置 NDT_OMP 多线程和邻域搜索方法
    ndt_->setNumThreads(ndt_num_threads_);
    if (ndt_neighbor_search_method_ == "DIRECT7") {
        ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    } else if (ndt_neighbor_search_method_ == "DIRECT1") {
        ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    } else {
        ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }

    frame_count_ = 0;
    keyframe_count_ = 0;
    processed_loops_.clear();
    resetCargoForHookState(false);
    cargo_fusion_track_id_ = 0U;
    relocalization_force_global_.store(false, std::memory_order_release);
    relocalization_state_ = RelocalizationState::IDLE;
    relocalization_pose_reliable_ = true;
    relocalization_invalid_safety_published_ = false;
    relocalization_bad_frames_ = 0;
    relocalization_good_frames_ = 0;
    relocalization_confirmation_count_ = 0;
    relocalization_cooldown_until_frame_ = 0;

    ROS_INFO("SLAM system reset complete");
    return true;
}

bool NdtSlamNode::setPoseService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    ROS_INFO("Set pose service called");
    return true;
}

bool NdtSlamNode::relocalizeService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    if (!relocalization_enabled_) {
        ROS_WARN("[Relocalization] manual request ignored: disabled");
        return true;
    }
    // Non-blocking and fail-safe: never reset pose to the map origin. The
    // next usable cloud launches a forced global search.
    relocalization_force_global_.store(true, std::memory_order_release);
    ROS_WARN("[Relocalization] manual global search queued");
    return true;
}

bool NdtSlamNode::saveMapService(lidar_slam2_msgs::SaveMap::Request& request,
                              lidar_slam2_msgs::SaveMap::Response& response) {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (global_map_->empty()) {
        response.success = false;
        response.message = "Map is empty";
        response.num_points = 0;
        return true;
    }

    std::string file_path = request.file_path;
    if (file_path.empty()) {
        file_path = "map_" + std::to_string(ros::Time::now().toSec()) + ".pcd";
    }
    if (file_path.find(".pcd") == std::string::npos) {
        file_path += ".pcd";
    }

    try {
        pcl::PCDWriter writer;
        int result = writer.writeBinary(file_path, *global_map_);
        response.success = (result == 0);
        response.message = response.success ? "Map saved" : "Failed to save";
        response.num_points = global_map_->size();
        response.saved_file_path = file_path;

        if (response.success) {
            ROS_INFO("[SaveMap] using_filtered_keyframes=true, placed_cargo_masks=%zu, dynamic_events=%zu",
                     dynamic_event_manager_.getPlacedSessions().size(),
                     dynamic_event_manager_.getActiveCount());
            ROS_INFO("Map saved: %s, points: %lu", file_path.c_str(), global_map_->size());

            // 同时保存关键帧数据库
            std::string session_dir = file_path.substr(0, file_path.find_last_of("/\\"));
            if (session_dir.empty()) session_dir = ".";
            session_dir += "/session_" + std::to_string(ros::Time::now().toSec());

            // 更新关键帧质量指标
            updateKeyFrameMetrics();

            // 保存关键帧数据库
            loop_closure_detector_.getKeyFrameManager().saveKeyFrameDatabase(session_dir);

            // 保存多层地图
            saveMultiLayerMaps(session_dir);

            ROS_INFO("Session saved to: %s", session_dir.c_str());
        }
    } catch (const std::exception& e) {
        response.success = false;
        response.message = std::string("Exception: ") + e.what();
        response.num_points = 0;
    }

    return true;
}

bool NdtSlamNode::loadMapService(lidar_slam2_msgs::LoadMap::Request& request,
                              lidar_slam2_msgs::LoadMap::Response& response) {
    std::string file_path = request.file_path;
    if (file_path.empty()) {
        response.success = false;
        response.message = "File path is empty";
        response.num_points = 0;
        return true;
    }

    try {
        pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *loaded_cloud) == -1) {
            response.success = false;
            response.message = "Failed to load PCD file";
            response.num_points = 0;
            return true;
        }

        std::lock_guard<std::mutex> lock(map_mutex_);
        global_map_ = loaded_cloud;

        response.success = true;
        response.message = "Map loaded";
        response.num_points = global_map_->size();

        publishMap();

        ROS_INFO("Map loaded: %s, points: %lu", file_path.c_str(), global_map_->size());
    } catch (const std::exception& e) {
        response.success = false;
        response.message = std::string("Exception: ") + e.what();
        response.num_points = 0;
    }

    return true;
}

bool NdtSlamNode::rebuildMapService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    ROS_INFO("Rebuilding map from keyframes with edge-preserving fusion...");

    // 使用默认 session 目录
    std::string session_dir = "/home/ydkj/NDT-slam-ws/output/rebuild_" + std::to_string(ros::Time::now().toSec());

    // 先加载关键帧数据库
    auto& keyframe_manager = loop_closure_detector_.getKeyFrameManager();
    if (keyframe_manager.getKeyFrames().empty()) {
        // 尝试加载最新的 session
        std::string latest_session = "/home/ydkj/NDT-slam-ws/output/session_1778217371.097046";
        ROS_INFO("No keyframes loaded, trying to load from: %s", latest_session.c_str());

        if (!keyframe_manager.loadKeyFrameDatabase(latest_session)) {
            ROS_ERROR("Failed to load keyframe database");
            return false;
        }
        loop_closure_detector_.rebuildScanContexts();
        ROS_INFO("Loaded %zu keyframes", keyframe_manager.getKeyFrames().size());
    }

    // 调用 rebuildMapFromKeyframes 函数
    rebuildMapFromKeyframes(session_dir);

    ROS_INFO("Map rebuilt successfully. Output: %s", session_dir.c_str());

    return true;
}

void NdtSlamNode::updateKeyFrameMetrics() {
    auto& keyframes = loop_closure_detector_.getKeyFrameManager();
    const auto& kf_list = keyframes.getKeyFrames();

    for (auto& kf : kf_list) {
        // 计算质量指标
        KeyFrameMetrics metrics;

        // 地面/非地面分割统计
        pcl::PointCloud<pcl::PointXYZ> ground, objects;
        Eigen::Matrix4d transform = kf.pose_.matrix();
        pcl::PointCloud<pcl::PointXYZ> transformed;
        pcl::transformPointCloud(*kf.cloud_, transformed, transform.cast<float>());

        separateGroundByGrid(transformed, ground, objects);

        metrics.ground_points = ground.size();
        metrics.object_points = objects.size();
        metrics.obj_ratio = (ground.size() + objects.size() > 0) ?
            (double)objects.size() / (ground.size() + objects.size()) : 0.0;

        // 地面厚度统计
        if (ground.size() > 10) {
            Eigen::Vector3d centroid(0, 0, 0);
            for (const auto& p : ground.points) {
                centroid += Eigen::Vector3d(p.x, p.y, p.z);
            }
            centroid /= ground.size();

            double thickness_sum = 0;
            for (const auto& p : ground.points) {
                thickness_sum += std::abs(p.z - centroid.z());
            }
            metrics.ground_thickness = thickness_sum / ground.size();
        }

        // 配准质量（从 NDT 获取）
        metrics.fitness_score = 0.0;  // 需要从配准过程中获取
        metrics.transformation_probability = 0.0;
        metrics.inlier_ratio = 0.0;

        // 判断是否可用于各层地图
        metrics.accepted_for_localization = (metrics.ground_thickness < 0.3 && metrics.obj_ratio > 0.1);
        metrics.accepted_for_detail_map = (metrics.object_points > 50);
        metrics.accepted_for_clean_map = (metrics.object_points > 30 && metrics.obj_ratio > 0.05);

        // 更新关键帧指标
        const_cast<KeyFrame&>(kf).metrics_ = metrics;
    }

    ROS_INFO("Updated metrics for %zu keyframes", kf_list.size());
}

void NdtSlamNode::saveMultiLayerMaps(const std::string& session_dir) {
    try {
        std::filesystem::path session_path(session_dir);

        // 创建目录
        std::filesystem::create_directories(session_path);

        // 保存各层地图
        auto saveMap = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& map, const std::string& filename) {
            if (map && !map->empty()) {
                std::string filepath = session_path / filename;
                pcl::io::savePCDFileBinary(filepath, *map);
                ROS_INFO("Saved %s: %zu points", filename.c_str(), map->size());
            }
        };

        // ========== 正式地图层 ==========
        saveMap(global_map_, "map_registration.pcd");
        saveMap(display_map_, "map_display.pcd");
        saveMap(ground_map_, "map_ground.pcd");
        saveMap(objects_map_, "map_objects_raw.pcd");
        saveMap(objects_clean_map_, "map_objects_clean.pcd");

        // ========== 调试/检测用 PCD ==========
        saveMap(rebuild_objects_filtered_, "map_objects_filtered.pcd");
        saveMap(rebuild_payload_candidate_, "map_payload_candidate.pcd");
        saveMap(rebuild_payload_dynamic_, "map_payload_dynamic.pcd");
        saveMap(rebuild_human_candidate_, "map_human_candidate.pcd");
        saveMap(rebuild_human_dynamic_, "map_human_dynamic.pcd");
        saveMap(rebuild_human_pending_, "map_human_pending.pcd");
        saveMap(rebuild_ground_raw_, "map_ground_raw.pcd");

        // 全量显示地图（ground + filtered_objects）
        pcl::PointCloud<pcl::PointXYZ>::Ptr display_full(new pcl::PointCloud<pcl::PointXYZ>);
        if (rebuild_ground_raw_ && !rebuild_ground_raw_->empty()) {
            *display_full += *rebuild_ground_raw_;
        }
        if (rebuild_objects_filtered_ && !rebuild_objects_filtered_->empty()) {
            *display_full += *rebuild_objects_filtered_;
        }
        saveMap(display_full, "map_display_full.pcd");

        int total_saved = 0;
        if (global_map_ && !global_map_->empty()) total_saved++;
        if (display_map_ && !display_map_->empty()) total_saved++;
        if (ground_map_ && !ground_map_->empty()) total_saved++;
        if (objects_map_ && !objects_map_->empty()) total_saved++;
        if (objects_clean_map_ && !objects_clean_map_->empty()) total_saved++;
        if (rebuild_objects_filtered_ && !rebuild_objects_filtered_->empty()) total_saved++;
        if (rebuild_payload_candidate_ && !rebuild_payload_candidate_->empty()) total_saved++;
        if (rebuild_payload_dynamic_ && !rebuild_payload_dynamic_->empty()) total_saved++;
        if (rebuild_human_candidate_ && !rebuild_human_candidate_->empty()) total_saved++;
        if (rebuild_human_dynamic_ && !rebuild_human_dynamic_->empty()) total_saved++;
        if (rebuild_human_pending_ && !rebuild_human_pending_->empty()) total_saved++;
        if (rebuild_ground_raw_ && !rebuild_ground_raw_->empty()) total_saved++;
        if (display_full && !display_full->empty()) total_saved++;

        ROS_INFO("Saved %d multi-layer maps to %s", total_saved, session_dir.c_str());

        // ========== Mask 确认日志 ==========
        if (dynamic_event_config_.enabled) {
            auto placed_sessions = dynamic_event_manager_.getPlacedSessions();
            auto active_sessions = dynamic_event_manager_.getActivePayloadSessions();
            ROS_INFO("[SaveMapMaskConfirm] dynamic_events enabled: placed=%zu, active=%zu",
                     placed_sessions.size(), active_sessions.size());

            if (!placed_sessions.empty()) {
                ROS_INFO("[SaveMapMaskConfirm] placed cargo protected in objects_clean and display_map:");
                for (const auto* session : placed_sessions) {
                    ROS_INFO("[SaveMapMaskConfirm]   session=%d, bbox=(%.1f,%.1f,%.1f)-(%.1f,%.1f,%.1f), placed_time=%.2f",
                             session->id,
                             session->placed_bbox.min_pt.x(), session->placed_bbox.min_pt.y(), session->placed_bbox.min_pt.z(),
                             session->placed_bbox.max_pt.x(), session->placed_bbox.max_pt.y(), session->placed_bbox.max_pt.z(),
                             session->placed_time);
                }
            }

            // 检查 objects_clean_map 是否包含 placed cargo 点
            if (objects_clean_map_ && !objects_clean_map_->empty()) {
                int placed_points = 0;
                for (const auto& p : objects_clean_map_->points) {
                    for (const auto* session : placed_sessions) {
                        if (session->placed_bbox.contains(p)) {
                            placed_points++;
                            break;
                        }
                    }
                }
                ROS_INFO("[SaveMapMaskConfirm] objects_clean_map contains %d placed cargo points out of %zu total",
                         placed_points, objects_clean_map_->size());
            }

            // 检查 display_map 是否包含 placed cargo 点
            if (display_map_ && !display_map_->empty()) {
                int placed_points = 0;
                for (const auto& p : display_map_->points) {
                    for (const auto* session : placed_sessions) {
                        if (session->placed_bbox.contains(p)) {
                            placed_points++;
                            break;
                        }
                    }
                }
                ROS_INFO("[SaveMapMaskConfirm] display_map contains %d placed cargo points out of %zu total",
                         placed_points, display_map_->size());
            }
        } else {
            ROS_INFO("[SaveMapMaskConfirm] dynamic_events disabled, no mask applied");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception saving multi-layer maps: %s", e.what());
    }
}

void NdtSlamNode::rebuildMapFromKeyframes(const std::string& session_dir) {
    ROS_INFO("Rebuilding maps from keyframes...");

    auto& keyframe_manager = loop_closure_detector_.getKeyFrameManager();
    const auto& keyframes = keyframe_manager.getKeyFrames();

    if (keyframes.empty()) {
        ROS_WARN("No keyframes available for rebuilding");
        return;
    }

    // 清空现有地图
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        global_map_->clear();
        display_map_->clear();
        ground_map_->clear();
        objects_map_->clear();
        objects_clean_map_->clear();
        rebuild_objects_filtered_->clear();
        rebuild_payload_candidate_->clear();
        rebuild_payload_dynamic_->clear();
        rebuild_human_candidate_->clear();
        rebuild_human_dynamic_->clear();
        rebuild_human_pending_->clear();
        rebuild_ground_raw_->clear();
    }

    // 逐步构建地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_temp(new pcl::PointCloud<pcl::PointXYZ>);

    int processed_count = 0;
    for (const auto& kf : keyframes) {
        if (!kf.cloud_ || kf.cloud_->empty()) continue;

        Sophus::SE3d pose = kf.has_refined_pose_ ? kf.pose_refined_ : kf.pose_;
        Eigen::Matrix4d transform = pose.matrix();

        auto addInRange = [&](const pcl::PointCloud<pcl::PointXYZ>& src,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr dst) {
            for (const auto& p : src.points) {
                if (std::abs(p.x) <= max_map_size_ &&
                    std::abs(p.y) <= max_map_size_ &&
                    std::abs(p.z) <= max_map_size_ &&
                    std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                    dst->push_back(p);
                }
            }
        };

        if (channel_filter_config_.enabled) {
            // 在 base_link 下做通道过滤
            pcl::PointCloud<pcl::PointXYZ> base_ground, base_objects;
            separateGroundByGrid(*kf.cloud_, base_ground, base_objects);

            std::map<CellKey, float> empty_ground_model;
            ChannelFilterResult ch_result = channel_filter_.filter(base_objects.makeShared(), empty_ground_model);

            // ========== HumanObjectDynamicFilter（人体动态过滤）==========
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_safe(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_candidates(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_dynamic(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr rebuild_human_pending(new pcl::PointCloud<pcl::PointXYZ>);

            if (human_filter_config_.enabled) {
                human_filter_.processFrame(ch_result.safe_objects, transform, 0.0,
                                           rebuild_human_safe, rebuild_human_candidates,
                                           rebuild_human_dynamic, rebuild_human_pending);
            } else {
                rebuild_human_safe = ch_result.safe_objects;
            }

            // 变换到 map
            pcl::PointCloud<pcl::PointXYZ> safe_transformed;
            pcl::transformPointCloud(*rebuild_human_safe, safe_transformed, transform.cast<float>());

            pcl::PointCloud<pcl::PointXYZ> ground_transformed;
            pcl::transformPointCloud(base_ground, ground_transformed, transform.cast<float>());

            // 吊货候选变换到 map
            pcl::PointCloud<pcl::PointXYZ> payload_cand_transformed;
            if (ch_result.payload_candidates && !ch_result.payload_candidates->empty()) {
                pcl::transformPointCloud(*ch_result.payload_candidates, payload_cand_transformed, transform.cast<float>());
            }

            // 人体候选/dynamic/pending 变换到 map
            pcl::PointCloud<pcl::PointXYZ> human_cand_transformed, human_dyn_transformed, human_pend_transformed;
            if (!rebuild_human_candidates->empty()) {
                pcl::transformPointCloud(*rebuild_human_candidates, human_cand_transformed, transform.cast<float>());
            }
            if (!rebuild_human_dynamic->empty()) {
                pcl::transformPointCloud(*rebuild_human_dynamic, human_dyn_transformed, transform.cast<float>());
            }
            if (!rebuild_human_pending->empty()) {
                pcl::transformPointCloud(*rebuild_human_pending, human_pend_transformed, transform.cast<float>());
            }

            // 正式地图只用 filtered
            addInRange(safe_transformed, all_cloud);
            addInRange(ground_transformed, all_cloud);
            addInRange(ground_transformed, ground_temp);
            addInRange(safe_transformed, objects_temp);

            // 收集调试数据
            addInRange(safe_transformed, rebuild_objects_filtered_);
            addInRange(ground_transformed, rebuild_ground_raw_);
            addInRange(payload_cand_transformed, rebuild_payload_candidate_);
            addInRange(human_cand_transformed, rebuild_human_candidate_);
            addInRange(human_dyn_transformed, rebuild_human_dynamic_);
            addInRange(human_pend_transformed, rebuild_human_pending_);
        } else {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*kf.cloud_, *transformed, transform.cast<float>());

            addInRange(*transformed, all_cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
            separateGroundByGrid(*transformed, *ground, *objects);

            *ground_temp += *ground;
            *objects_temp += *objects;
        }

        processed_count++;
        if (processed_count % 20 == 0) {
            ROS_INFO("Processed %d/%zu keyframes", processed_count, keyframes.size());
        }
    }

    ROS_INFO("Point collection done: all=%zu, ground=%zu, objects=%zu",
             all_cloud->size(), ground_temp->size(), objects_temp->size());
    ROS_INFO("Debug clouds: filtered=%zu, payload_cand=%zu, human_cand=%zu, human_dynamic=%zu, human_pending=%zu",
             rebuild_objects_filtered_->size(), rebuild_payload_candidate_->size(),
             rebuild_human_candidate_->size(), rebuild_human_dynamic_->size(),
             rebuild_human_pending_->size());

    // 体素滤波并保存
    auto voxelFilter = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, double size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        if (input->size() > 100) {
            pcl::VoxelGrid<pcl::PointXYZ> vf;
            vf.setInputCloud(input);
            vf.setLeafSize(size, size, size);
            vf.filter(*output);
        } else {
            *output = *input;
        }
        return output;
    };

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        ROS_INFO("Filtering registration map...");
        *global_map_ = *voxelFilter(all_cloud, voxel_size_);
        ROS_INFO("Filtering display map...");
        *display_map_ = *voxelFilter(all_cloud, display_voxel_size_);
        ROS_INFO("Filtering ground map...");
        *ground_map_ = *voxelFilter(ground_temp, ground_voxel_size_);
        ROS_INFO("Filtering objects map...");
        *objects_map_ = *voxelFilter(objects_temp, objects_voxel_size_);
        ROS_INFO("All maps filtered");
    }

    // 保存重建的地图
    saveMultiLayerMaps(session_dir);

    ROS_INFO("Rebuilt maps from %zu keyframes", keyframes.size());
    ROS_INFO("  global_map: %zu points", global_map_->size());
    ROS_INFO("  display_map: %zu points", display_map_->size());
    ROS_INFO("  ground_map: %zu points", ground_map_->size());
    ROS_INFO("  objects_map: %zu points", objects_map_->size());
}

void NdtSlamNode::performRelocalization() {
    if (!relocalization_enabled_) return;
    relocalization_force_global_.store(true, std::memory_order_release);

    // 简单的重定位：重置位姿
}

std::vector<RelocalizationSeed> NdtSlamNode::buildLocalRelocalizationSeeds(
    const Sophus::SE3d& center) const {
    std::vector<RelocalizationSeed> seeds;
    const Eigen::Matrix3d rotation = center.so3().matrix();
    const double base_yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    const auto make_seed = [&](double dx, double dy, double dyaw_deg) {
        Eigen::Vector3d translation = center.translation();
        translation.x() += dx;
        translation.y() += dy;
        const double yaw = base_yaw + dyaw_deg * M_PI / 180.0;
        RelocalizationSeed seed;
        seed.pose = Sophus::SE3d(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
            translation);
        seed.source = "local_grid";
        seeds.push_back(std::move(seed));
    };

    make_seed(0.0, 0.0, 0.0);
    for (double dx = -relocalization_local_xy_window_m_;
         dx <= relocalization_local_xy_window_m_ + 1.0e-6;
         dx += relocalization_local_xy_step_m_) {
        for (double dy = -relocalization_local_xy_window_m_;
             dy <= relocalization_local_xy_window_m_ + 1.0e-6;
             dy += relocalization_local_xy_step_m_) {
            for (double dyaw = -relocalization_local_yaw_window_deg_;
                 dyaw <= relocalization_local_yaw_window_deg_ + 1.0e-6;
                 dyaw += relocalization_local_yaw_step_deg_) {
                if (std::abs(dx) < 1.0e-6 && std::abs(dy) < 1.0e-6 &&
                    std::abs(dyaw) < 1.0e-6) continue;
                make_seed(dx, dy, dyaw);
            }
        }
    }
    std::stable_sort(seeds.begin() + 1, seeds.end(),
        [&center](const RelocalizationSeed& lhs,
                  const RelocalizationSeed& rhs) {
            return (lhs.pose.translation().head<2>() -
                    center.translation().head<2>()).squaredNorm() <
                   (rhs.pose.translation().head<2>() -
                    center.translation().head<2>()).squaredNorm();
        });
    return seeds;
}

std::vector<RelocalizationSeed> NdtSlamNode::buildGlobalRelocalizationSeeds(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::vector<RelocalizationSeed> seeds;
    const auto hints = loop_closure_detector_.findRelocalizationHints(
        cloud, static_cast<std::size_t>(relocalization_global_hint_count_),
        relocalization_global_min_similarity_);
    for (const auto& hint : hints) {
        const Eigen::Matrix3d rotation = hint.pose.so3().matrix();
        const double yaw = std::atan2(rotation(1, 0), rotation(0, 0)) +
            hint.yaw_offset_rad;
        for (const double delta_deg : {0.0, -12.0, 12.0}) {
            RelocalizationSeed seed;
            seed.pose = Sophus::SE3d(
                Eigen::AngleAxisd(yaw + delta_deg * M_PI / 180.0,
                                  Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                hint.pose.translation());
            seed.source = "scan_context";
            seed.keyframe_id = hint.keyframe_id;
            seed.descriptor_similarity = hint.similarity;
            seeds.push_back(std::move(seed));
        }
    }
    return seeds;
}

void NdtSlamNode::consumeRelocalizationResult(
    std::uint64_t frame_index, const ros::Time& stamp) {
    if (!relocalization_enabled_) return;
    RelocalizationResult result;
    if (!relocalizer_.takeResult(result)) return;
    if (relocalization_state_ == RelocalizationState::IDLE &&
        relocalization_pose_reliable_ &&
        !relocalization_force_global_.load(std::memory_order_acquire)) return;
    if (result.frame_index <= relocalization_last_result_frame_) return;
    relocalization_last_result_frame_ = result.frame_index;

    const double result_age_sec = stamp.toSec() - result.stamp_sec;
    if (frame_index > result.frame_index +
            static_cast<std::uint64_t>(relocalization_result_max_age_frames_) ||
        !std::isfinite(result_age_sec) || result_age_sec < -0.05 ||
        result_age_sec > relocalization_result_max_age_sec_) {
        relocalization_confirmation_count_ = 0;
        publishRelocalizationStatus("DEGRADED", "stale_result_discarded");
        return;
    }
    if (!result.valid) {
        relocalization_confirmation_count_ = 0;
        relocalization_state_ = RelocalizationState::DEGRADED;
        publishRelocalizationStatus("DEGRADED", result.reason);
        return;
    }

    const auto yaw_of = [](const Sophus::SE3d& pose) {
        const Eigen::Matrix3d r = pose.so3().matrix();
        return std::atan2(r(1, 0), r(0, 0));
    };
    const Sophus::SE3d correction =
        result.pose * result.reference_pose.inverse();
    bool consistent = false;
    if (relocalization_confirmation_count_ > 0) {
        const double translation =
            (correction.translation().head<2>() -
             relocalization_confirmation_pose_.translation().head<2>()).norm();
        const double yaw_delta_deg = std::abs(std::atan2(
            std::sin(yaw_of(correction) - yaw_of(relocalization_confirmation_pose_)),
            std::cos(yaw_of(correction) - yaw_of(relocalization_confirmation_pose_)))) *
            180.0 / M_PI;
        consistent = translation <= relocalization_confirm_translation_m_ &&
                     yaw_delta_deg <= relocalization_confirm_yaw_deg_;
    }

    relocalization_confirmation_pose_ = correction;
    relocalization_confirmation_count_ = consistent
        ? relocalization_confirmation_count_ + 1 : 1;
    relocalization_state_ = RelocalizationState::CONFIRMING;
    publishRelocalizationStatus(
        "CONFIRMING", "count=" +
        std::to_string(relocalization_confirmation_count_) +
        " fitness=" + std::to_string(result.fitness));

    if (relocalization_confirmation_count_ >= relocalization_confirm_frames_) {
        // The worker result belongs to an earlier sensor frame. Preserve the
        // short-term motion observed since that job so a fast diagonal move
        // is not pulled backwards to a stale absolute pose.
        const Sophus::SE3d motion_since_job =
            result.reference_pose.inverse() * current_pose_;
        const Sophus::SE3d recovered_at_current_stamp =
            result.pose * motion_since_job;
        applyRelocalizedPose(recovered_at_current_stamp, stamp, result);
        relocalization_confirmation_count_ = 0;
        relocalization_bad_frames_ = 0;
        relocalization_good_frames_ = 0;
        relocalization_force_global_ = false;
        relocalization_pose_reliable_ = true;
        relocalization_state_ = RelocalizationState::COOLDOWN;
        relocalization_cooldown_until_frame_ =
            frame_index + static_cast<std::uint64_t>(
                relocalization_cooldown_frames_);
    }
}

void NdtSlamNode::updateRelocalization(
    std::uint64_t frame_index, const ros::Time& stamp,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& registration_cloud,
    bool ndt_healthy) {
    if (!relocalization_enabled_) return;

    if (ndt_healthy && relocalization_pose_reliable_ &&
        relocalization_state_ == RelocalizationState::COOLDOWN) {
        relocalization_bad_frames_ = 0;
        relocalization_good_frames_ = 0;
        if (frame_index >= relocalization_cooldown_until_frame_) {
            relocalization_state_ = RelocalizationState::IDLE;
            publishRelocalizationStatus("IDLE", "cooldown_complete");
        }
        return;
    }

    if (ndt_healthy && !relocalization_force_global_) {
        ++relocalization_good_frames_;
        if (relocalization_state_ == RelocalizationState::IDLE) {
            relocalization_bad_frames_ = 0;
            return;
        }
        // Natural NDT recovery is accepted only after three consecutive
        // healthy frames; a single intermittent acceptance is not enough.
        if (relocalization_good_frames_ >= 3) {
            relocalization_bad_frames_ = 0;
            relocalization_confirmation_count_ = 0;
            resetCargoAfterPoseDiscontinuity();
            relocalization_pose_reliable_ = true;
            relocalization_state_ = RelocalizationState::COOLDOWN;
            relocalization_cooldown_until_frame_ =
                frame_index + static_cast<std::uint64_t>(
                    relocalization_cooldown_frames_);
            relocalization_good_frames_ = 0;
            publishRelocalizationStatus("COOLDOWN", "ndt_self_recovered");
            return;
        }
    } else {
        relocalization_good_frames_ = 0;
        ++relocalization_bad_frames_;
    }

    if (!relocalization_force_global_ &&
        relocalization_bad_frames_ < relocalization_trigger_frames_) return;

    if (relocalization_pose_reliable_) {
        relocalization_pose_reliable_ = false;
        resetCargoAfterPoseDiscontinuity();
        publishRelocalizationSafetyInvalid(stamp, "localization_degraded");
        relocalization_invalid_safety_published_ = true;
    }
    relocalization_state_ = RelocalizationState::DEGRADED;

    if (relocalizer_.busy() ||
        frame_index < relocalization_last_submit_frame_ +
            static_cast<std::uint64_t>(
                relocalization_request_interval_frames_)) return;

    const bool global = relocalization_force_global_ ||
        relocalization_bad_frames_ >= relocalization_global_trigger_frames_;
    RelocalizationJob job;
    job.frame_index = frame_index;
    job.stamp_sec = stamp.toSec();
    job.mode = global ? RelocalizationMode::GLOBAL : RelocalizationMode::LOCAL;
    job.reference_pose = current_pose_;
    job.source.reset(new pcl::PointCloud<pcl::PointXYZ>(*registration_cloud));

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        const auto& selected_map = global && global_map_ && !global_map_->empty()
            ? global_map_ : local_map_;
        if (!selected_map || selected_map->empty()) {
            publishRelocalizationStatus("DEGRADED", "map_unavailable");
            return;
        }
        job.map.reset(new pcl::PointCloud<pcl::PointXYZ>(*selected_map));
    }

    job.seeds = global
        ? buildGlobalRelocalizationSeeds(job.source)
        : buildLocalRelocalizationSeeds(current_pose_);

    // A loaded PCD may not have a keyframe database. Fall back to a bounded
    // coarse map grid instead of pretending global recovery succeeded.
    if (global && job.seeds.empty() && job.map && !job.map->empty()) {
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        for (const auto& p : job.map->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
        }
        const double grid = std::max(6.0,
            relocalization_cfg_.target_crop_radius_m * 0.75);
        const double z = current_pose_.translation().z();
        for (double x = min_x; x <= max_x &&
             static_cast<int>(job.seeds.size()) <
                 relocalization_cfg_.max_candidates; x += grid) {
            for (double y = min_y; y <= max_y &&
                 static_cast<int>(job.seeds.size()) <
                     relocalization_cfg_.max_candidates; y += grid) {
                for (const double yaw : {0.0, M_PI_2, M_PI, -M_PI_2}) {
                    RelocalizationSeed seed;
                    seed.pose = Sophus::SE3d(
                        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                            .toRotationMatrix(),
                        Eigen::Vector3d(x, y, z));
                    seed.source = "coarse_map_grid";
                    job.seeds.push_back(std::move(seed));
                    if (static_cast<int>(job.seeds.size()) >=
                        relocalization_cfg_.max_candidates) break;
                }
            }
        }
    }

    if (job.seeds.empty() || !relocalizer_.submit(std::move(job))) {
        publishRelocalizationStatus("DEGRADED", "no_search_candidates");
        return;
    }
    relocalization_last_submit_frame_ = frame_index;
    relocalization_state_ = global
        ? RelocalizationState::SEARCHING_GLOBAL
        : RelocalizationState::SEARCHING_LOCAL;
    publishRelocalizationStatus(
        global ? "SEARCHING_GLOBAL" : "SEARCHING_LOCAL",
        "bad_frames=" + std::to_string(relocalization_bad_frames_));
}

void NdtSlamNode::applyRelocalizedPose(
    const Sophus::SE3d& pose, const ros::Time& stamp,
    const RelocalizationResult& result) {
    Sophus::SE3d recovered = crane_constraint_enabled_
        ? applyCraneMotionConstraint(pose, "relocalization") : pose;
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        current_pose_ = recovered;
        relocalized_pose_ = recovered;
        tracking_lost_ = false;
    }
    if (crane_motion_ekf_enabled_) {
        crane_motion_ekf_.initialize(recovered, stamp);
    }
    const Eigen::Matrix3d rotation = recovered.so3().matrix();
    filtered_yaw_rad_ = std::atan2(rotation(1, 0), rotation(0, 0));
    filtered_yaw_initialized_ = true;

    // Recenter the runtime map without changing the persistent global map.
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (global_map_ && !global_map_->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr recentered(
                new pcl::PointCloud<pcl::PointXYZ>);
            const Eigen::Vector3d center = recovered.translation();
            const double radius_sq =
                relocalization_cfg_.target_crop_radius_m *
                relocalization_cfg_.target_crop_radius_m;
            for (const auto& p : global_map_->points) {
                const double dx = p.x - center.x();
                const double dy = p.y - center.y();
                if (dx * dx + dy * dy <= radius_sq) recentered->push_back(p);
            }
            if (static_cast<int>(recentered->size()) >=
                relocalization_cfg_.min_target_points) {
                local_map_ = recentered;
                ++local_map_version_;
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock(localization_target_mutex_);
        cached_target_valid_ = false;
        cached_target_version_ = 0;
        cached_target_points_ = 0;
        crop_frames_since_update_ = crop_update_min_interval_frames_;
        last_bound_ndt_target_.reset();
        last_bound_ndt_target_version_ = 0;
        last_bound_ndt_target_source_ = "none";
        last_target_reason_ = "relocalized_rebind";
    }

    path_msg_.poses.clear();
    runtime_path_msg_.poses.clear();
    has_last_path_pose_ = false;
    resetCargoAfterPoseDiscontinuity();
    publishRelocalizationStatus(
        "COOLDOWN", "accepted mode=" +
        std::string(result.mode == RelocalizationMode::GLOBAL
                        ? "global" : "local") +
        " fitness=" + std::to_string(result.fitness) +
        " seed=" + result.seed_source);
    tracking_cv_.notify_all();
}

void NdtSlamNode::resetCargoAfterPoseDiscontinuity() {
    payload_tracker_.reset();
    selected_payload_track_id_ = -1;
    const HookLoadSnapshot hook = currentHookLoadSnapshot();
    const bool preserve_origin = hook.valid &&
        hook.state == lidar_slam2_msgs::HookLoadState::STATE_LOADED;
    resetCargoForHookState(preserve_origin);
    cargo_fusion_track_id_ = 0;
}

void NdtSlamNode::publishRelocalizationStatus(
    const std::string& state, const std::string& detail) {
    std_msgs::String message;
    message.data = "state=" + state + " detail=" + detail +
        " bad_frames=" + std::to_string(relocalization_bad_frames_);
    relocalization_status_pub_.publish(message);
    ROS_WARN_THROTTLE(1.0, "[Relocalization] %s", message.data.c_str());
}

void NdtSlamNode::publishRelocalizationSafetyInvalid(
    const ros::Time& stamp, const std::string& reason) {
    const HookLoadSnapshot hook = currentHookLoadSnapshot();
    lidar_slam2_msgs::CargoSafetyStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = map_frame_;
    status.schema_version = lidar_slam2_msgs::CargoSafetyStatus::SCHEMA_VERSION;
    status.valid = false;
    status.cargo_valid = false;
    status.cargo_source =
        lidar_slam2_msgs::CargoBottomEstimate::SOURCE_INVALID;
    status.requested_alarm_code =
        lidar_slam2_msgs::CargoSafetyStatus::ALARM_OUTER_OR_INVALID;
    status.hook_signal_valid = hook.valid;
    status.hook_load_state = hook.state;
    status.hook_voltage = hook.voltage;
    status.no_cargo_confirmed = false;
    status.obstacle_valid = false;
    status.reason = "relocalization:" + reason;
    cargo_safety_status_pub_.publish(status);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NdtSlamNode::filterDynamicPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!use_dynamic_filter_ || cloud->size() < 100) {
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_stddev_mul_thresh_);
    sor.filter(*filtered);

    // 如果过滤后点数过少（<30%），说明场景本身点云稀疏，跳过过滤
    if (filtered->size() < cloud->size() * 0.3) {
        ROS_DEBUG("Dynamic filter too aggressive (%lu -> %lu), skipping",
                  cloud->size(), filtered->size());
        return cloud;
    }

    static int filter_log_count = 0;
    filter_log_count++;
    if (filter_log_count % 50 == 0) {
        ROS_DEBUG("Dynamic filter: %lu -> %lu points (removed %lu)",
                  cloud->size(), filtered->size(), cloud->size() - filtered->size());
    }

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NdtSlamNode::edgePreservingMerge(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& existing_map,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& new_cloud,
    double voxel_size,
    int min_observations) {

    if (!new_cloud || new_cloud->empty()) {
        return existing_map;
    }

    if (!existing_map || existing_map->empty()) {
        return new_cloud;
    }

    // 使用简化的边缘保留融合：只保留每个 voxel 的第一个点（减少计算量）
    struct VoxelKey {
        int x, y, z;
        bool operator==(const VoxelKey& o) const { return x==o.x && y==o.y && z==o.z; }
    };
    struct VoxelHash {
        size_t operator()(const VoxelKey& k) const {
            return ((k.x * 73856093) ^ (k.y * 19349663) ^ (k.z * 83492791));
        }
    };

    std::unordered_map<VoxelKey, std::vector<Eigen::Vector3d>, VoxelHash> voxels;

    // 将已有地图的点加入 voxel
    for (const auto& p : existing_map->points) {
        VoxelKey key{
            static_cast<int>(std::floor(p.x / voxel_size)),
            static_cast<int>(std::floor(p.y / voxel_size)),
            static_cast<int>(std::floor(p.z / voxel_size))
        };
        voxels[key].push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }

    // 将新点云的点加入 voxel
    for (const auto& p : new_cloud->points) {
        VoxelKey key{
            static_cast<int>(std::floor(p.x / voxel_size)),
            static_cast<int>(std::floor(p.y / voxel_size)),
            static_cast<int>(std::floor(p.z / voxel_size))
        };
        voxels[key].push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }

    // 处理每个 voxel，保留边缘点（简化版本：保留最远的点）
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto& [key, points] : voxels) {
        if (points.size() < min_observations) {
            continue;
        }

        // 计算质心
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& p : points) {
            centroid += p;
        }
        centroid /= points.size();

        // 保留最远的点（边缘点）
        double max_dist = 0;
        Eigen::Vector3d edge_point = points[0];
        for (const auto& p : points) {
            double dist = (p - centroid).norm();
            if (dist > max_dist) {
                max_dist = dist;
                edge_point = p;
            }
        }
        result->points.push_back(pcl::PointXYZ(edge_point.x(), edge_point.y(), edge_point.z()));
    }

    result->width = result->points.size();
    result->height = 1;
    result->is_dense = true;

    ROS_DEBUG("[EdgePreservingMerge] existing=%lu, new=%lu, result=%lu",
              existing_map->size(), new_cloud->size(), result->size());

    return result;
}

// ========== 长期建图功能实现 ==========

// v8-stable-r3-hotfix-minimal: PoseFreeze 已禁用
// TF/odom 始终使用 EKF 输出的 constrained_pose
// 静止时的零速约束由 CraneMotionEKF 内部处理
Sophus::SE3d NdtSlamNode::selectPublishedPose(
    const Sophus::SE3d& constrained_pose,
    const ros::Time& stamp)
{
    (void)stamp;
    published_pose_ = constrained_pose;
    return published_pose_;
}

// CRITICAL RUNTIME CHAIN - DO NOT MODIFY
// a7be4bf runtime pose chain must stay unchanged:
// NDT/refined/EKF -> publishOdometry -> TF -> publishRuntimePath.
// MotionGate controls MapCommit only.
// raw_ndt_pose is allowed only as MapCommit evidence.
// Do NOT route raw_pose/tracking_pose to odom/TF/runtime_path/current_cloud.
bool NdtSlamNode::evaluateMotionGateForMapCommit(
    const Sophus::SE3d& pose,
    const ros::Time& stamp) {
    const Sophus::SE3d runtime_pose_before = current_pose_;
    const bool ekf_initialized =
        crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized();
    Eigen::Vector4d ekf_before = Eigen::Vector4d::Zero();
    if (ekf_initialized) {
        ekf_before = crane_motion_ekf_.state();
    }

    const bool map_commit_allowed = shouldCommitKeyframe(pose, stamp);

    Eigen::Vector4d ekf_after = Eigen::Vector4d::Zero();
    if (ekf_initialized) {
        ekf_after = crane_motion_ekf_.state();
    }
    const double runtime_pose_delta =
        (runtime_pose_before.inverse() * current_pose_).log().norm();
    const bool pose_modified = runtime_pose_delta > 1e-12;
    const bool position_modified =
        (ekf_after.head<2>() - ekf_before.head<2>()).norm() > 1e-12;
    const bool velocity_modified =
        (ekf_after.tail<2>() - ekf_before.tail<2>()).norm() > 1e-12;
    const bool violated =
        pose_modified || position_modified || velocity_modified;

    motion_gate_invariant_check_count_.fetch_add(
        1, std::memory_order_relaxed);
    if (!map_commit_allowed) {
        motion_gate_map_commit_block_count_.fetch_add(
            1, std::memory_order_relaxed);
    }
    if (violated) {
        motion_gate_invariant_violation_count_.fetch_add(
            1, std::memory_order_relaxed);
        ROS_ERROR(
            "[MOTION_GATE_INVARIANT_VIOLATION] pose_modified=%d "
            "position_modified=%d velocity_modified=%d",
            pose_modified ? 1 : 0,
            position_modified ? 1 : 0,
            velocity_modified ? 1 : 0);
    }

    ROS_INFO_THROTTLE(
        1.0,
        "[MOTION_GATE_INVARIANT] stationary=%d pose_modified=%d "
        "position_modified=%d velocity_modified=%d map_commit_allowed=%d "
        "checks=%llu blocked=%llu violations=%llu",
        motion_gate_stationary_ ? 1 : 0,
        pose_modified ? 1 : 0,
        position_modified ? 1 : 0,
        velocity_modified ? 1 : 0,
        map_commit_allowed ? 1 : 0,
        static_cast<unsigned long long>(
            motion_gate_invariant_check_count_.load(
                std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            motion_gate_map_commit_block_count_.load(
                std::memory_order_relaxed)),
        static_cast<unsigned long long>(
            motion_gate_invariant_violation_count_.load(
                std::memory_order_relaxed)));

    return map_commit_allowed;
}

bool NdtSlamNode::shouldCommitKeyframe(const Sophus::SE3d& current_pose, const ros::Time& current_time) {
    if (!motion_gate_enabled_) {
        return true;  // 未启用 MotionGate，始终允许
    }

    // 首帧总是允许
    if (last_keyframe_pose_for_gate_.translation().norm() < 0.001) {
        last_keyframe_pose_for_gate_ = current_pose;
        last_keyframe_time_for_gate_ = current_time;
        last_frame_pos_for_gate_ = current_pose.translation();
        last_frame_stamp_for_gate_ = current_time.toSec();
        return true;
    }

    // 计算位移和旋转
    Sophus::SE3d delta = last_keyframe_pose_for_gate_.inverse() * current_pose;
    double translation = delta.translation().norm();
    double rotation = delta.so3().log().norm() * 180.0 / M_PI;
    double time_elapsed = (current_time - last_keyframe_time_for_gate_).toSec();

    // P1: 计算帧间速度（用于静止检测）
    double frame_dt = current_time.toSec() - last_frame_stamp_for_gate_;
    double frame_dx = (current_pose.translation() - last_frame_pos_for_gate_).norm();
    double frame_vel = frame_dt > 1e-3 ? frame_dx / frame_dt : 0.0;
    last_frame_pos_for_gate_ = current_pose.translation();
    last_frame_stamp_for_gate_ = current_time.toSec();

    delta_translation_ = translation;
    delta_yaw_ = rotation;

    // 检查是否满足条件
    bool moved_enough = (translation >= motion_gate_min_translation_m_ ||
                         rotation >= motion_gate_min_rotation_deg_);
    bool time_elapsed_enough = (time_elapsed >= motion_gate_min_time_sec_);

    // P1: 静止检测（基于帧间速度）
    bool detected_stationary = (frame_vel < motion_gate_moving_min_velocity_ &&
                                rotation < motion_gate_min_rotation_deg_);

    // 进入静止状态
    if (!is_stationary_ && detected_stationary) {
        stationary_frame_count_++;
        if (stationary_frame_count_ > 30) {
            is_stationary_ = true;
            motion_gate_stationary_ = true;  // v8: 设置 PoseFreeze 标志
            stationary_anchor_pose_ = current_pose;
            stationary_anchor_valid_ = true;
            stationary_start_time_ = current_time.toSec();
            moving_confirm_frames_ = 0;
            moving_confirm_count_ = 0;

            // fix/588-runtime-localization-stable: 记录 raw anchor 用于 stationary exit 判断
            if (has_last_raw_ndt_pose_) {
                stationary_raw_anchor_pose_ = last_raw_ndt_pose_;
                stationary_raw_anchor_valid_ = true;
            } else {
                stationary_raw_anchor_pose_ = current_pose;
                stationary_raw_anchor_valid_ = false;
            }

            ROS_INFO("[MotionGate] Crane stopped | keyframes=%d | anchor=(%.2f,%.2f,%.2f) raw_anchor=(%.2f,%.2f,%.2f) | pausing map commit only",
                     keyframe_count_,
                     stationary_anchor_pose_.translation().x(),
                     stationary_anchor_pose_.translation().y(),
                     stationary_anchor_pose_.translation().z(),
                     stationary_raw_anchor_pose_.translation().x(),
                     stationary_raw_anchor_pose_.translation().y(),
                     stationary_raw_anchor_pose_.translation().z());
        }
    } else if (!detected_stationary) {
        stationary_frame_count_ = 0;
    }

    // P1: 静止期间检查漂移（使用 raw NDT drift 避免 frozen pose 反馈环）
    if (is_stationary_ && stationary_anchor_valid_) {
        double elapsed = current_time.toSec() - stationary_start_time_;

        // 计算 filtered_drift（仅用于日志）
        double filtered_drift =
            (current_pose.translation().head<2>() -
             stationary_anchor_pose_.translation().head<2>()).norm();

        // 计算 raw_drift（主退出证据）
        double raw_drift = filtered_drift;
        if (stationary_raw_anchor_valid_ && has_last_raw_ndt_pose_) {
            raw_drift =
                (last_raw_ndt_pose_.translation().head<2>() -
                 stationary_raw_anchor_pose_.translation().head<2>()).norm();
        }

        // P0-3: evidence 检查必须放在任何 SKIP_COMMIT 判断之前
        // 计算 evidence_trans = max(raw_trans, refined_trans, runtime_trans)
        double raw_trans = 0.0;
        double refined_trans = 0.0;
        double runtime_trans = 0.0;

        if (has_commit_gate_reference_) {
            if (has_last_raw_ndt_pose_) {
                raw_trans = (last_raw_ndt_pose_.translation().head<2>() -
                            last_commit_raw_pose_.translation().head<2>()).norm();
            }

            refined_trans = (current_pose.translation().head<2>() -
                            last_commit_refined_pose_.translation().head<2>()).norm();
            runtime_trans = (current_pose.translation().head<2>() -
                            last_commit_runtime_pose_.translation().head<2>()).norm();
        }

        double evidence_trans = std::max(raw_trans, std::max(refined_trans, runtime_trans));

        ROS_INFO_THROTTLE(
            2.0,
            "[MotionGateEvidence] raw=%.3f refined=%.3f runtime=%.3f evidence=%.3f dt=%.2f fitness=%.3f kf=%d",
            raw_trans,
            refined_trans,
            runtime_trans,
            evidence_trans,
            elapsed,
            last_ndt_fitness_,
            keyframe_count_);

        // Outer MapCommit quality gating has already accepted this frame; do
        // not introduce a second, stricter fitness cliff in MotionGate.
        if (keyframe_count_ <= 1 &&
            elapsed > 3.0 &&
            evidence_trans > 0.15) {
            ROS_WARN(
                "[MotionGate] unfreeze_initial_map_commit raw=%.3f refined=%.3f runtime=%.3f evidence=%.3f dt=%.2f fitness=%.3f",
                raw_trans,
                refined_trans,
                runtime_trans,
                evidence_trans,
                elapsed,
                last_ndt_fitness_);
            // 确认移动，恢复提交，必须复位所有 stationary 状态
            is_stationary_ = false;
            motion_gate_stationary_ = false;
            stationary_anchor_valid_ = false;
            stationary_raw_anchor_valid_ = false;
            moving_confirm_frames_ = 0;
            moving_confirm_count_ = 0;
            stationary_frame_count_ = 0;
            return true;
        }

        // 漂移在 ignore_radius 内，认为是 NDT 静止漂移，不提交
        if (raw_drift < motion_gate_stationary_drift_ignore_radius_) {
            ROS_INFO_THROTTLE(2.0,
                "[MotionGate] stationary_skip_map_commit raw_drift=%.3f filtered_drift=%.3f elapsed=%.1f action=SKIP_MAP_COMMIT odom_publish=1 pose_override=0",
                raw_drift, filtered_drift, elapsed);
            return false;
        }

        // 超过 ignore_radius，需要连续确认才能认为真的在移动
        moving_confirm_frames_++;
        if (moving_confirm_frames_ < motion_gate_moving_confirm_frames_) {
            ROS_INFO_THROTTLE(1.0,
                "[MotionGate] possible_move raw_drift=%.3f filtered_drift=%.3f confirm=%d/%d action=WAIT_MAP_COMMIT",
                raw_drift, filtered_drift, moving_confirm_frames_, motion_gate_moving_confirm_frames_);
            return false;
        }

        // 确认移动，恢复提交，必须复位所有 stationary 状态
        is_stationary_ = false;
        motion_gate_stationary_ = false;
        stationary_anchor_valid_ = false;
        stationary_raw_anchor_valid_ = false;
        moving_confirm_frames_ = 0;
        moving_confirm_count_ = 0;
        stationary_frame_count_ = 0;

        ROS_INFO("[MotionGate] exit_stationary raw_drift=%.3f filtered_drift=%.3f confirm=%d action=RESUME_MAP_COMMIT",
                 raw_drift, filtered_drift, motion_gate_moving_confirm_frames_);
    }

    // 正常移动检测（使用 evidence_trans）
    // 计算 evidence_trans = max(raw_trans, refined_trans, runtime_trans)
    double raw_trans_move = 0.0;
    double refined_trans_move = 0.0;
    double runtime_trans_move = 0.0;

    if (has_commit_gate_reference_) {
        if (has_last_raw_ndt_pose_) {
            raw_trans_move = (last_raw_ndt_pose_.translation().head<2>() -
                             last_commit_raw_pose_.translation().head<2>()).norm();
        }

        refined_trans_move = (current_pose.translation().head<2>() -
                             last_commit_refined_pose_.translation().head<2>()).norm();
        runtime_trans_move = (current_pose.translation().head<2>() -
                             last_commit_runtime_pose_.translation().head<2>()).norm();
    }

    double evidence_trans_move = std::max(raw_trans_move, std::max(refined_trans_move, runtime_trans_move));

    // 正常运动提交条件
    if (evidence_trans_move > motion_gate_min_translation_m_ &&
        time_elapsed_enough &&
        !is_stationary_) {
        last_keyframe_pose_for_gate_ = current_pose;
        last_keyframe_time_for_gate_ = current_time;
        moved_frame_count_++;
        return true;
    }

    return false;
}

void NdtSlamNode::releaseOldKeyframeClouds() {
    if (max_active_keyframes_ <= 0) return;

    auto& keyframes = loop_closure_detector_.getKeyFrameManager().getKeyFramesNonConst();
    if (keyframes.size() <= max_active_keyframes_) return;

    // 释放超出窗口的旧关键帧的点云
    int release_count = 0;
    for (size_t i = 0; i < keyframes.size() - max_active_keyframes_; i++) {
        if (keyframes[i].cloud_ && !keyframes[i].cloud_->empty()) {
            keyframes[i].cloud_->clear();
            keyframes[i].cloud_->points.shrink_to_fit();
            release_count++;
        }
    }

    if (release_count > 0) {
        ROS_INFO("[LongTerm] Released %d old keyframe clouds, active window: %zu",
                 release_count, std::min(keyframes.size(), (size_t)max_active_keyframes_));
    }
}

void NdtSlamNode::flushDirtyTiles() {
    if (!persistent_map_enabled_ || dirty_tiles_.empty()) return;

    // 检查磁盘保护
    if (!checkDiskGuard()) {
        ROS_WARN_THROTTLE(60, "[DiskGuard] Skipping flush, disk low");
        return;
    }

    // 创建目录
    std::string reg_dir = persistent_map_root_dir_ + "/tiles_registration";
    std::string disp_dir = persistent_map_root_dir_ + "/tiles_display";
    std::string gnd_dir = persistent_map_root_dir_ + "/tiles_ground";
    std::string obj_dir = persistent_map_root_dir_ + "/tiles_objects";
    boost::filesystem::create_directories(reg_dir);
    boost::filesystem::create_directories(disp_dir);
    boost::filesystem::create_directories(gnd_dir);
    boost::filesystem::create_directories(obj_dir);

    // 体素滤波函数
    auto voxelFilter = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, double voxel_size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        if (input->size() > 100) {
            pcl::VoxelGrid<pcl::PointXYZ> vf;
            vf.setInputCloud(input);
            vf.setLeafSize(voxel_size, voxel_size, voxel_size);
            vf.filter(*output);
        } else {
            *output = *input;
        }
        return output;
    };

    // 增量合并函数：读取已有 tile，合并后再写入
    auto mergeAndWrite = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& new_cloud,
                             const std::string& filepath, double voxel_size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);

        // 如果已有 tile，读取并合并
        if (boost::filesystem::exists(filepath)) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr existing(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *existing) == 0) {
                *merged = *existing;
            }
        }

        // 合并新点云
        *merged += *new_cloud;

        // 体素滤波
        auto filtered = voxelFilter(merged, voxel_size);

        // 写入临时文件然后重命名（防断电损坏）
        std::string tmp_path = filepath + ".tmp";
        pcl::io::savePCDFileBinary(tmp_path, *filtered);
        boost::filesystem::rename(tmp_path, filepath);

        return filtered->size();
    };

    int flushed = 0;
    for (auto& [tile_key, tile_layers] : dirty_tiles_) {
        // 写入 registration layer
        if (tile_layers.registration && !tile_layers.registration->empty()) {
            std::string filepath = reg_dir + "/" + tile_key + ".pcd";
            mergeAndWrite(tile_layers.registration, filepath, tile_voxel_registration_);
        }

        // 写入 display layer
        if (tile_layers.display && !tile_layers.display->empty()) {
            std::string filepath = disp_dir + "/" + tile_key + ".pcd";
            mergeAndWrite(tile_layers.display, filepath, tile_voxel_display_);
        }

        // 写入 ground layer
        if (tile_layers.ground && !tile_layers.ground->empty()) {
            std::string filepath = gnd_dir + "/" + tile_key + ".pcd";
            mergeAndWrite(tile_layers.ground, filepath, tile_voxel_ground_);
        }

        // 写入 objects layer
        if (tile_layers.objects && !tile_layers.objects->empty()) {
            std::string filepath = obj_dir + "/" + tile_key + ".pcd";
            mergeAndWrite(tile_layers.objects, filepath, tile_voxel_objects_);
        }

        flushed++;
    }

    flushed_tile_count_ += flushed;
    dirty_tiles_.clear();
    dirty_tile_count_ = 0;
    last_flush_time_ = ros::Time::now();
    last_flush_time_local_ = ros::Time::now();

    ROS_INFO("[TileFlush] %d tiles flushed to disk | total_flushed=%d",
             flushed, flushed_tile_count_);
}

void NdtSlamNode::writeRuntimeStatus() {
    if (!persistent_map_enabled_) return;

    // 确保目录存在
    boost::filesystem::create_directories(persistent_map_root_dir_);

    // 使用 tmp + rename 防止监控脚本读到半写文件
    std::string status_file = persistent_map_root_dir_ + "/runtime_status.json";
    std::string tmp_file = status_file + ".tmp";
    std::ofstream f(tmp_file);
    if (!f.is_open()) return;

    // 获取磁盘空间
    double disk_free_gb = getDiskFreeGB();

    // 获取内存使用
    long mem_mb = getProcessMemoryMB();

    // 检查点云超时
    double pc_elapsed = (ros::Time::now() - last_pointcloud_time_).toSec();
    pointcloud_stale_ = (pc_elapsed > pointcloud_stale_timeout_sec_);

    // 获取各地图点数
    size_t global_pts = global_map_ ? global_map_->size() : 0;
    size_t display_pts = display_map_ ? display_map_->size() : 0;
    size_t ground_pts = ground_map_ ? ground_map_->size() : 0;
    size_t objects_pts = objects_map_ ? objects_map_->size() : 0;
    size_t local_pts = local_map_ ? local_map_->size() : 0;

    f << std::fixed << std::setprecision(2);
    f << "{\n";
    f << "  \"timestamp\": \"" << ros::Time::now() << "\",\n";
    f << "  \"total_frames\": " << total_frames_ << ",\n";
    f << "  \"total_keyframes\": " << total_keyframes_ << ",\n";
    f << "  \"active_keyframes\": " << active_keyframes_ << ",\n";
    f << "  \"is_stationary\": " << (is_stationary_ ? "true" : "false") << ",\n";
    f << "  \"stationary_frame_count\": " << stationary_frame_count_ << ",\n";
    f << "  \"delta_translation_m\": " << delta_translation_ << ",\n";
    f << "  \"delta_yaw_deg\": " << delta_yaw_ << ",\n";
    f << "  \"global_map_points\": " << global_pts << ",\n";
    f << "  \"display_map_points\": " << display_pts << ",\n";
    f << "  \"ground_map_points\": " << ground_pts << ",\n";
    f << "  \"objects_map_points\": " << objects_pts << ",\n";
    f << "  \"local_map_points\": " << local_pts << ",\n";
    f << "  \"dirty_tile_count\": " << dirty_tile_count_ << ",\n";
    f << "  \"flushed_tile_count\": " << flushed_tile_count_ << ",\n";
    f << "  \"memory_mb\": " << mem_mb << ",\n";
    f << "  \"memory_guard_triggered\": " << (memory_guard_triggered_ ? "true" : "false") << ",\n";
    f << "  \"disk_free_gb\": " << disk_free_gb << ",\n";
    f << "  \"disk_guard_triggered\": " << (disk_guard_triggered_ ? "true" : "false") << ",\n";
    f << "  \"pointcloud_timeout_sec\": " << pc_elapsed << ",\n";
    f << "  \"pointcloud_stale\": " << (pointcloud_stale_ ? "true" : "false") << ",\n";
    f << "  \"last_ndt_fitness\": " << last_ndt_fitness_ << ",\n";
    f << "  \"ndt_fitness_warning\": " << (consecutive_high_fitness_ > fitness_warning_count_ ? "true" : "false") << ",\n";
    f << "  \"consecutive_high_fitness\": " << consecutive_high_fitness_ << ",\n";
    f << "  \"average_process_time_ms\": " << average_process_time_ms_ << ",\n";
    f << "  \"average_ndt_time_ms\": " << average_ndt_time_ms_ << ",\n";
    f << "  \"last_flush_time\": \"" << last_flush_time_local_ << "\",\n";
    f << "  \"last_active_map_rebuild\": \"" << last_active_map_rebuild_time_ << "\",\n";
    f << "  \"last_update\": \"" << ros::Time::now() << "\"\n";
    f << "}\n";
    f.close();

    // 原子重命名
    boost::filesystem::rename(tmp_file, status_file);

    // 内存保护检查
    checkMemoryGuard();
}

// ========== 统一提交检查 ==========

bool NdtSlamNode::canCommit() {
    return commit_enabled_
        && !mapping_paused_by_memory_guard_
        && !disk_guard_triggered_
        && !ndt_health_bad_;
}

// ========== 内存保护实现 ==========

long NdtSlamNode::getProcessMemoryMB() {
    long mem_kb = 0;
    std::ifstream ifs("/proc/self/status");
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.substr(0, 6) == "VmRSS:") {
            std::istringstream iss(line.substr(6));
            iss >> mem_kb;
            break;
        }
    }
    return mem_kb / 1024;
}

void NdtSlamNode::checkMemoryGuard() {
    if (!memory_guard_enabled_) return;

    ros::Time now = ros::Time::now();
    if ((now - last_memory_check_time_).toSec() < memory_check_interval_sec_) return;
    last_memory_check_time_ = now;

    long mem_mb = getProcessMemoryMB();
    MemoryGuardLevel prev_level = memory_guard_level_;

    // 分级判定
    if (mem_mb >= emergency_threshold_mb_) {
        memory_guard_level_ = MemoryGuardLevel::EMERGENCY;
    } else if (mem_mb >= hard_threshold_mb_) {
        memory_guard_level_ = MemoryGuardLevel::HARD;
    } else if (mem_mb >= soft_threshold_mb_) {
        memory_guard_level_ = MemoryGuardLevel::SOFT;
    } else {
        memory_guard_level_ = MemoryGuardLevel::OK;
    }

    // 状态变化时输出日志
    if (memory_guard_level_ != prev_level) {
        switch (memory_guard_level_) {
            case MemoryGuardLevel::OK:
                ROS_INFO("[MemoryGuard] OK: %ldMB, resuming normal operation", mem_mb);
                mapping_paused_by_memory_guard_ = false;
                break;
            case MemoryGuardLevel::SOFT:
                ROS_WARN("[MemoryGuard] SOFT: %ldMB > %dMB, releasing cache + flush tiles",
                         mem_mb, soft_threshold_mb_);
                releaseMemoryCache();
                if (persistent_map_enabled_) flushDirtyTiles();
                mapping_paused_by_memory_guard_ = false;  // SOFT 级别恢复提交
                break;
            case MemoryGuardLevel::HARD:
                ROS_ERROR("[MemoryGuard] HARD: %ldMB > %dMB, pausing map commit",
                         mem_mb, hard_threshold_mb_);
                mapping_paused_by_memory_guard_ = true;
                break;
            case MemoryGuardLevel::EMERGENCY:
                ROS_ERROR("[MemoryGuard] EMERGENCY: %ldMB > %dMB, forcing downsample",
                         mem_mb, emergency_threshold_mb_);
                forceDownsampleAllMaps();
                if (persistent_map_enabled_) flushDirtyTiles();
                break;
        }
    }

    memory_guard_triggered_ = (memory_guard_level_ != MemoryGuardLevel::OK);
}

void NdtSlamNode::releaseMemoryCache() {
    // 1. flush dirty tiles
    if (persistent_map_enabled_ && !dirty_tiles_.empty()) {
        flushDirtyTiles();
    }

    // 2. 释放超出窗口的 keyframe cloud
    releaseOldKeyframeClouds();

    // 3. 清空 path 历史（如果很长）
    if (path_msg_.poses.size() > 1000) {
        size_t half = path_msg_.poses.size() / 2;
        path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + half);
        ROS_INFO("[MemoryGuard] Trimmed path history: %zu -> %zu", half * 2, path_msg_.poses.size());
    }

    // 4. 建议 glibc 归还内存给操作系统
    malloc_trim(0);

    ROS_INFO("[MemoryGuard] SOFT: released caches");
}

void NdtSlamNode::forceDownsampleAllMaps() {
    std::lock_guard<std::mutex> lock(map_mutex_);

    auto forceVoxel = [](pcl::PointCloud<pcl::PointXYZ>::Ptr& map, double voxel, const char* name) {
        if (map && map->size() > 1000) {
            size_t before = map->size();
            pcl::VoxelGrid<pcl::PointXYZ> vf;
            vf.setInputCloud(map);
            vf.setLeafSize(voxel, voxel, voxel);
            pcl::PointCloud<pcl::PointXYZ> f;
            vf.filter(f);
            *map = f;
            ROS_WARN("[MemoryGuard] %s: %zu -> %zu points (voxel=%.2f)",
                     name, before, map->size(), voxel);
        }
    };

    forceVoxel(global_map_, 0.5, "global_map_");
    forceVoxel(display_map_, 0.5, "display_map_");
    forceVoxel(ground_map_, 0.3, "ground_map_");
    forceVoxel(objects_map_, 0.3, "objects_map_");
}

// ========== 磁盘保护实现 ==========

double NdtSlamNode::getDiskFreeGB() {
    try {
        boost::filesystem::space_info si = boost::filesystem::space(persistent_map_root_dir_);
        return static_cast<double>(si.available) / (1024.0 * 1024.0 * 1024.0);
    } catch (...) {
        return -1.0;
    }
}

bool NdtSlamNode::checkDiskGuard() {
    if (!disk_guard_enabled_) return true;

    double free_gb = getDiskFreeGB();
    if (free_gb < 0) return true;  // 获取失败，不阻止

    if (free_gb < min_free_disk_gb_) {
        if (!disk_guard_triggered_) {
            ROS_ERROR("[DiskGuard] CRITICAL: only %.1fGB free (limit %.1fGB), pausing tile writes",
                      free_gb, min_free_disk_gb_);
            disk_guard_triggered_ = true;
        }
        return false;
    } else {
        if (disk_guard_triggered_) {
            ROS_INFO("[DiskGuard] Disk recovered: %.1fGB free, resuming tile writes", free_gb);
            disk_guard_triggered_ = false;
        }
        return true;
    }
}

void NdtSlamNode::rebuildActiveMapFromRecentKeyframes() {
    if (!longterm_mapping_enabled_) return;

    // 防止重入
    if (active_map_rebuild_running_.exchange(true)) {
        ROS_WARN_THROTTLE(10, "[ActiveMap] rebuild already running, skip");
        return;
    }

    auto& keyframes = loop_closure_detector_.getKeyFrameManager().getKeyFramesNonConst();
    if (keyframes.empty()) {
        active_map_rebuild_running_ = false;
        return;
    }

    // 收集最近 80 个有 cloud 的关键帧
    std::vector<const KeyFrame*> recent_kfs;
    for (auto it = keyframes.rbegin(); it != keyframes.rend() && recent_kfs.size() < max_active_keyframes_; ++it) {
        if (it->cloud_ && !it->cloud_->empty()) {
            recent_kfs.push_back(&(*it));
        }
    }

    if (recent_kfs.empty()) {
        active_map_rebuild_running_ = false;
        return;
    }

    ROS_INFO("[ActiveMap] Rebuilding from %zu recent keyframes", recent_kfs.size());

    // 构建新地图（不加锁）
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_global(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_display(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_objects(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto* kf : recent_kfs) {
        // 应用天车运动约束到 keyframe pose
        Sophus::SE3d constrained_kf_pose = applyCraneMotionConstraint(kf->pose_, "active_rebuild");
        Eigen::Matrix4d transform = constrained_kf_pose.matrix();

        // 变换到 map 坐标系
        pcl::PointCloud<pcl::PointXYZ> transformed;
        pcl::transformPointCloud(*kf->cloud_, transformed, transform.cast<float>());

        // ground/object 分离
        pcl::PointCloud<pcl::PointXYZ> kf_ground, kf_objects;
        separateGroundByGrid(transformed, kf_ground, kf_objects);

        // 添加到各层地图
        for (const auto& p : transformed.points) {
            if (std::abs(p.x) <= max_map_size_ && std::abs(p.y) <= max_map_size_ &&
                std::abs(p.z) <= max_map_size_ && std::isfinite(p.x)) {
                new_global->push_back(p);
                new_display->push_back(p);
            }
        }
        for (const auto& p : kf_ground.points) {
            if (std::isfinite(p.x)) new_ground->push_back(p);
        }
        for (const auto& p : kf_objects.points) {
            if (std::isfinite(p.x)) new_objects->push_back(p);
        }
    }

    // 体素滤波
    auto voxelFilter = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, double voxel_size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        if (input->size() > 100) {
            pcl::VoxelGrid<pcl::PointXYZ> vf;
            vf.setInputCloud(input);
            vf.setLeafSize(voxel_size, voxel_size, voxel_size);
            vf.filter(*output);
        } else {
            *output = *input;
        }
        return output;
    };

    new_global = voxelFilter(new_global, voxel_size_);
    new_display = voxelFilter(new_display, display_voxel_size_);
    new_ground = voxelFilter(new_ground, ground_voxel_size_);
    new_objects = voxelFilter(new_objects, objects_voxel_size_);

    // swap 指针（持锁时间最短）
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        global_map_ = new_global;
        display_map_ = new_display;
        ground_map_ = new_ground;
        objects_map_ = new_objects;
    }

    last_active_map_rebuild_time_ = ros::Time::now();

    ROS_INFO("[ActiveMap] Rebuilt: global=%zu, display=%zu, ground=%zu, objects=%zu",
             global_map_->size(), display_map_->size(), ground_map_->size(), objects_map_->size());

    active_map_rebuild_running_ = false;
}

// ========== HookFixedCargoDetector ==========


// ========== OdomAnchorBox 检测函数 ==========

NdtSlamNode::HookCargoDetection NdtSlamNode::detectCargoAroundOdomAnchor(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base,
    const ros::Time& stamp) {

    HookCargoDetection result;
    result.valid = false;
    result.reject_reason = "unknown";
    result.core_points_base.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (!odom_anchor_config_.enabled) {
        result.reject_reason = "disabled";
        return result;
    }

    if (!cloud_base || cloud_base->empty()) {
        result.reject_reason = "empty_cloud";
        return result;
    }

    auto anchor = getCargoAnchorXY();
    float cx = anchor.x();
    float cy = anchor.y();

    // 围绕 anchor 裁剪
    float x_min = cx - odom_anchor_config_.search_half_x;
    float x_max = cx + odom_anchor_config_.search_half_x;
    float y_min = cy - odom_anchor_config_.search_half_y;
    float y_max = cy + odom_anchor_config_.search_half_y;
    float z_min = odom_anchor_config_.search_z_min;
    float z_max = odom_anchor_config_.search_z_max;

    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : cloud_base->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.x >= x_min && p.x <= x_max &&
            p.y >= y_min && p.y <= y_max &&
            p.z >= z_min && p.z <= z_max) {
            crop_cloud->push_back(p);
        }
    }

    if (crop_cloud->empty()) {
        result.reject_reason = "crop_empty";
        ROS_DEBUG_THROTTLE(1.0, "[OdomAnchorDetect] input=%zu crop=0 anchor=(%.2f,%.2f)",
                          cloud_base->size(), cx, cy);
        return result;
    }

    // HAG 预过滤（如果启用）
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (odom_anchor_config_.tight_box.hag_filter_enabled) {
        // 估算地面高度（使用 crop_cloud 的最低点作为地面参考）
        float ground_z = crop_cloud->points[0].z;
        for (const auto& p : crop_cloud->points) {
            if (p.z < ground_z) ground_z = p.z;
        }

        for (const auto& p : crop_cloud->points) {
            float hag = p.z - ground_z;
            if (hag >= odom_anchor_config_.tight_box.hag_min_m &&
                hag <= odom_anchor_config_.tight_box.hag_max_m) {
                filtered_cloud->push_back(p);
            }
        }
    } else {
        filtered_cloud = crop_cloud;
    }

    if (filtered_cloud->empty()) {
        result.reject_reason = "hag_filter_empty";
        ROS_DEBUG_THROTTLE(1.0, "[OdomAnchorDetect] input=%zu crop=%zu hag_filter=0",
                          cloud_base->size(), crop_cloud->size());
        return result;
    }

    // 体素降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setInputCloud(filtered_cloud);
    vf.setLeafSize(0.05f, 0.05f, 0.05f);
    vf.filter(*voxel_cloud);

    if (voxel_cloud->size() < static_cast<size_t>(odom_anchor_config_.weak_min_points)) {
        result.reject_reason = "too_few_points";
        ROS_DEBUG_THROTTLE(1.0, "[OdomAnchorDetect] input=%zu crop=%zu filtered=%zu voxel=%zu too_few",
                          cloud_base->size(), crop_cloud->size(), filtered_cloud->size(), voxel_cloud->size());
        return result;
    }

    // 欧几里得聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(voxel_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.20);
    ec.setMinClusterSize(odom_anchor_config_.weak_min_points);
    ec.setMaxClusterSize(8000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(voxel_cloud);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
        result.reject_reason = "no_clusters";
        ROS_DEBUG_THROTTLE(1.0, "[OdomAnchorDetect] input=%zu crop=%zu filtered=%zu voxel=%zu clusters=0",
                          cloud_base->size(), crop_cloud->size(), filtered_cloud->size(), voxel_cloud->size());
        return result;
    }

    // 选择最大簇
    size_t best_idx = 0;
    size_t best_size = 0;
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        if (cluster_indices[i].indices.size() > best_size) {
            best_size = cluster_indices[i].indices.size();
            best_idx = i;
        }
    }

    const auto& best_cluster = cluster_indices[best_idx];

    // 构建结果点云
    for (int idx : best_cluster.indices) {
        result.core_points_base->push_back(voxel_cloud->points[idx]);
    }

    // 子簇重聚类（如果启用）
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_points(new pcl::PointCloud<pcl::PointXYZ>);
    if (odom_anchor_config_.tight_box.sub_cluster_enabled &&
        result.core_points_base->size() > static_cast<size_t>(odom_anchor_config_.tight_box.sub_cluster_min_points * 2)) {

        pcl::search::KdTree<pcl::PointXYZ>::Ptr sub_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        sub_tree->setInputCloud(result.core_points_base);

        std::vector<pcl::PointIndices> sub_cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> sub_ec;
        sub_ec.setClusterTolerance(odom_anchor_config_.tight_box.sub_cluster_tolerance_m);
        sub_ec.setMinClusterSize(odom_anchor_config_.tight_box.sub_cluster_min_points);
        sub_ec.setMaxClusterSize(8000);
        sub_ec.setSearchMethod(sub_tree);
        sub_ec.setInputCloud(result.core_points_base);
        sub_ec.extract(sub_cluster_indices);

        if (!sub_cluster_indices.empty()) {
            // 选择最靠近 anchor 的子簇
            float best_dist = std::numeric_limits<float>::max();
            size_t best_sub_idx = 0;
            for (size_t i = 0; i < sub_cluster_indices.size(); ++i) {
                Eigen::Vector3f sub_center = Eigen::Vector3f::Zero();
                for (int idx : sub_cluster_indices[i].indices) {
                    sub_center += result.core_points_base->points[idx].getVector3fMap();
                }
                sub_center /= static_cast<float>(sub_cluster_indices[i].indices.size());
                float dist = (sub_center.head<2>() - anchor).norm();
                if (dist < best_dist) {
                    best_dist = dist;
                    best_sub_idx = i;
                }
            }

            for (int idx : sub_cluster_indices[best_sub_idx].indices) {
                final_points->push_back(result.core_points_base->points[idx]);
            }
        } else {
            final_points = result.core_points_base;
        }
    } else {
        final_points = result.core_points_base;
    }

    result.core_points_base = final_points;

    // 计算 bbox（使用配置的分位数）
    std::vector<float> xs, ys, zs;
    for (const auto& p : result.core_points_base->points) {
        xs.push_back(p.x);
        ys.push_back(p.y);
        zs.push_back(p.z);
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::sort(zs.begin(), zs.end());

    int n = xs.size();
    if (n < 3) {
        result.reject_reason = "too_few_points_for_bbox";
        return result;
    }

    float p_low = odom_anchor_config_.tight_box.percentile_low;
    float p_high = odom_anchor_config_.tight_box.percentile_high;
    float x05 = xs[static_cast<int>(n * p_low)];
    float x95 = xs[static_cast<int>(n * p_high)];
    float y05 = ys[static_cast<int>(n * p_low)];
    float y95 = ys[static_cast<int>(n * p_high)];
    float z05 = zs[0];
    float z95 = zs[n - 1];

    // 根据 symmetry_mode 计算尺寸
    float sx, sy;
    float center_x = cx, center_y = cy;

    if (odom_anchor_config_.tight_box.anchor_symmetry_mode == "soft") {
        // Soft symmetry: 允许中心偏移，但限制最大偏移
        float raw_cx = (x05 + x95) * 0.5f;
        float raw_cy = (y05 + y95) * 0.5f;

        float offset_x = raw_cx - cx;
        float offset_y = raw_cy - cy;
        float max_offset = odom_anchor_config_.tight_box.max_center_offset_m;

        // 限制偏移
        offset_x = std::max(-max_offset, std::min(offset_x, max_offset));
        offset_y = std::max(-max_offset, std::min(offset_y, max_offset));

        center_x = cx + offset_x;
        center_y = cy + offset_y;

        // 计算尺寸（基于实际点云范围 + margin）
        float margin_xy = odom_anchor_config_.tight_box.margin_xy_m;
        sx = (x95 - x05) + 2.0f * margin_xy;
        sy = (y95 - y05) + 2.0f * margin_xy;
    } else if (odom_anchor_config_.tight_box.anchor_symmetry_mode == "off") {
        // Off: 直接使用点云范围
        float margin_xy = odom_anchor_config_.tight_box.margin_xy_m;
        center_x = (x05 + x95) * 0.5f;
        center_y = (y05 + y95) * 0.5f;
        sx = (x95 - x05) + 2.0f * margin_xy;
        sy = (y95 - y05) + 2.0f * margin_xy;
    } else {
        // Strict symmetry: 原始逻辑
        float margin_xy = odom_anchor_config_.tight_box.margin_xy_m;
        sx = 2.0f * std::max(std::fabs(x05 - cx), std::fabs(x95 - cx)) + margin_xy;
        sy = 2.0f * std::max(std::fabs(y05 - cy), std::fabs(y95 - cy)) + margin_xy;
    }

    // 限制尺寸范围
    sx = std::max(odom_anchor_config_.min_size_x, std::min(sx, odom_anchor_config_.max_size_x));
    sy = std::max(odom_anchor_config_.min_size_y, std::min(sy, odom_anchor_config_.max_size_y));

    float margin_z = odom_anchor_config_.tight_box.margin_z_m;
    float sz = (z95 - z05) + 2.0f * margin_z;
    sz = std::max(odom_anchor_config_.min_size_z, std::min(sz, odom_anchor_config_.max_size_z));

    result.center_base = Eigen::Vector3f(center_x, center_y, (z05 + z95) * 0.5f);
    result.size_visible = Eigen::Vector3f(sx, sy, sz);
    result.z05 = z05;
    result.z50 = (z05 + z95) * 0.5f;
    result.z95 = z95;
    result.visible_height = z95 - z05;
    result.xy_area = sx * sy;
    result.valid = true;

    if (debug_cfg_.debug_tight_box) {
        ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec,
            "[TightBox] raw=%zu hag=%zu voxel=%zu clusters=%zu sub_cluster=%s selected_points=%zu anchor=(%.2f,%.2f) center=(%.2f,%.2f) size=(%.2f,%.2f,%.2f) z=[%.2f,%.2f] mode=%s",
            cloud_base->size(), filtered_cloud->size(), voxel_cloud->size(),
            cluster_indices.size(),
            odom_anchor_config_.tight_box.sub_cluster_enabled ? "on" : "off",
            result.core_points_base->size(),
            cx, cy, center_x, center_y, sx, sy, sz, z05, z95,
            odom_anchor_config_.tight_box.anchor_symmetry_mode.c_str());
    }

    return result;
}

// ========== HookCargoLock 状态机 ==========

uint64_t NdtSlamNode::computeCloudHash(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) return 0;
    uint64_t hash = cloud->size();
    for (size_t i = 0; i < std::min(cloud->size(), size_t(100)); ++i) {
        const auto& p = cloud->points[i];
        hash ^= std::hash<float>{}(p.x) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<float>{}(p.y) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        hash ^= std::hash<float>{}(p.z) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
}

bool NdtSlamNode::isStrongDetection(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom) {
    if (!det.valid || !det.core_points_base) return false;
    // strong_detection 不依赖 bottom.valid，只检查点数
    // det.valid 已经表示货物在 ROI 内且被检测到
    return det.core_points_base->size() >= static_cast<size_t>(hook_lock_config_.strong_min_points);
}

bool NdtSlamNode::isLockStrongDetection(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom) {
    if (!det.valid || !det.core_points_base) return false;
    // 锁定时使用更严格的条件
    bool points_ok = det.core_points_base->size() >= static_cast<size_t>(hook_lock_config_.lock_strong_min_points);
    bool height_ok = det.visible_height >= hook_lock_config_.lock_min_visible_height;
    bool area_ok = det.xy_area >= hook_lock_config_.lock_min_xy_area;
    return points_ok && height_ok && area_ok;
}

bool NdtSlamNode::isDetectionConsistentWithLockedBox(
    const HookCargoDetection& det,
    const HookCargoBottomEstimate& bottom,
    std::string* reject_reason) {

    if (!det.valid || !det.core_points_base || det.core_points_base->empty()) {
        *reject_reason = "invalid_detection";
        return false;
    }

    // 检查点数
    if (static_cast<int>(det.core_points_base->size()) < hook_lock_config_.locked_update_min_points) {
        *reject_reason = "too_few_points";
        return false;
    }

    // 检查中心距离（相对 anchor）
    auto anchor = getCargoAnchorXY();
    float dx = det.center_base.x() - anchor.x();
    float dy = det.center_base.y() - anchor.y();
    float dist_xy = std::hypot(dx, dy);

    if (dist_xy > hook_lock_config_.locked_update_max_center_dist) {
        *reject_reason = "center_too_far";
        return false;
    }

    if (hook_lock_.has_locked_size) {
        const float detected_sx = det.size_visible.x();
        const float detected_sy = det.size_visible.y();
        const float locked_sx = hook_lock_.locked_size.x();
        const float locked_sy = hook_lock_.locked_size.y();
        if (!std::isfinite(detected_sx) || !std::isfinite(detected_sy) ||
            !std::isfinite(locked_sx) || !std::isfinite(locked_sy) ||
            detected_sx <= 0.0F || detected_sy <= 0.0F ||
            locked_sx <= 0.0F || locked_sy <= 0.0F) {
            *reject_reason = "invalid_association_geometry";
            return false;
        }
        const Eigen::Vector2f detected_min = det.center_base.head<2>() -
            0.5F * Eigen::Vector2f(detected_sx, detected_sy);
        const Eigen::Vector2f detected_max = det.center_base.head<2>() +
            0.5F * Eigen::Vector2f(detected_sx, detected_sy);
        const Eigen::Vector2f locked_center =
            hook_lock_.locked_center_base.head<2>();
        const Eigen::Vector2f locked_min = locked_center -
            0.5F * Eigen::Vector2f(locked_sx, locked_sy);
        const Eigen::Vector2f locked_max = locked_center +
            0.5F * Eigen::Vector2f(locked_sx, locked_sy);
        const Eigen::Vector2f overlap_size =
            detected_max.cwiseMin(locked_max) -
            detected_min.cwiseMax(locked_min);
        const float intersection_area =
            std::max(0.0F, overlap_size.x()) *
            std::max(0.0F, overlap_size.y());
        const float reference_area = std::min(
            detected_sx * detected_sy, locked_sx * locked_sy);
        const float overlap_ratio = reference_area > 1.0e-6F
            ? intersection_area / reference_area
            : 0.0F;
        if (overlap_ratio <
            hook_lock_config_.locked_update_min_overlap_ratio) {
            *reject_reason = "footprint_overlap_too_small";
            return false;
        }
    }

    // 检查高度跳变
    if (bottom.valid && hook_lock_.has_good_height) {
        float bottom_jump = std::fabs(bottom.bottom_z_base - hook_lock_.stable_bottom_z);
        float top_jump = std::fabs(bottom.top_z_base - hook_lock_.stable_top_z);

        if (bottom_jump > hook_lock_config_.locked_update_max_z_jump) {
            *reject_reason = "bottom_z_jump";
            return false;
        }

        if (top_jump > hook_lock_config_.locked_update_max_top_jump) {
            *reject_reason = "top_z_jump";
            return false;
        }
    }

    *reject_reason = "consistent";
    return true;
}

bool NdtSlamNode::isWeakDetection(const HookCargoDetection& det) {
    if (!det.valid || !det.core_points_base) return false;
    return det.core_points_base->size() >= static_cast<size_t>(hook_lock_config_.weak_min_points);
}

Eigen::Vector3f NdtSlamNode::computeFixedCenterSize(
    const HookCargoDetection& det, const HookCargoBottomEstimate& bottom) {
    // 强制使用 anchor
    auto anchor = getCargoAnchorXY();
    float cx = anchor.x();
    float cy = anchor.y();

    if (!det.core_points_base || det.core_points_base->empty()) {
        return Eigen::Vector3f(
            odom_anchor_config_.default_size_x,
            odom_anchor_config_.default_size_y,
            odom_anchor_config_.default_size_z);
    }

    // 使用 P05/P95 代替 min/max
    std::vector<float> xs, ys;
    for (const auto& p : det.core_points_base->points) {
        xs.push_back(p.x);
        ys.push_back(p.y);
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());

    int n = xs.size();
    float x05 = xs[static_cast<int>(n * 0.05)];
    float x95 = xs[static_cast<int>(n * 0.95)];
    float y05 = ys[static_cast<int>(n * 0.05)];
    float y95 = ys[static_cast<int>(n * 0.95)];

    // 以 anchor 对称扩展
    float sx_raw = 2.0f * std::max(std::fabs(x05 - cx), std::fabs(x95 - cx))
                   + odom_anchor_config_.size_margin_x;
    float sy_raw = 2.0f * std::max(std::fabs(y05 - cy), std::fabs(y95 - cy))
                   + odom_anchor_config_.size_margin_y;
    float sz = bottom.valid ? std::max(bottom.height, odom_anchor_config_.min_size_z) : odom_anchor_config_.default_size_z;

    // 收紧尺寸限制
    float sx = std::max(odom_anchor_config_.min_size_x, std::min(sx_raw, odom_anchor_config_.max_size_x));
    float sy = std::max(odom_anchor_config_.min_size_y, std::min(sy_raw, odom_anchor_config_.max_size_y));
    sz = std::max(odom_anchor_config_.min_size_z, std::min(sz, odom_anchor_config_.max_size_z));

    return Eigen::Vector3f(sx, sy, sz);
}

Eigen::Vector3f NdtSlamNode::medianSize(const std::deque<Eigen::Vector3f>& buffer) {
    if (buffer.empty()) return Eigen::Vector3f(0.5f, 0.35f, 0.25f);
    std::vector<float> xs, ys, zs;
    for (const auto& v : buffer) { xs.push_back(v.x()); ys.push_back(v.y()); zs.push_back(v.z()); }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    std::sort(zs.begin(), zs.end());
    int mid = xs.size() / 2;
    return Eigen::Vector3f(xs[mid], ys[mid], zs[mid]);
}

void NdtSlamNode::updateLockedHeight(const HookCargoBottomEstimate& bottom, const ros::Time& stamp, bool initialize) {
    if (!bottom.valid) { growUncertainty(); return; }
    float alpha = (bottom.source == "points_visible_side") ?
                  hook_lock_config_.bottom_alpha_points : hook_lock_config_.bottom_alpha_memory;
    if (initialize || !hook_lock_.has_good_height) {
        hook_lock_.stable_bottom_z = bottom.bottom_z_base;
        hook_lock_.stable_top_z = bottom.top_z_base;
        hook_lock_.stable_height = bottom.height;
    } else {
        hook_lock_.stable_bottom_z = (1 - alpha) * hook_lock_.stable_bottom_z + alpha * bottom.bottom_z_base;
        hook_lock_.stable_top_z = (1 - alpha) * hook_lock_.stable_top_z + alpha * bottom.top_z_base;
        hook_lock_.stable_height = hook_lock_.stable_top_z - hook_lock_.stable_bottom_z;
    }
    hook_lock_.bottom_uncertainty = bottom.uncertainty;
    hook_lock_.has_good_height = true;
    hook_lock_.last_good_height_stamp = stamp;
}

void NdtSlamNode::maybeUpdateLockedSize(const HookCargoDetection& det, const HookCargoBottomEstimate& bottom) {
    if (!hook_lock_.has_locked_size) return;
    Eigen::Vector3f raw_size = computeFixedCenterSize(det, bottom);
    float dx = std::fabs(raw_size.x() - hook_lock_.locked_size.x()) / hook_lock_.locked_size.x();
    float dy = std::fabs(raw_size.y() - hook_lock_.locked_size.y()) / hook_lock_.locked_size.y();
    bool changed = dx > hook_lock_config_.size_change_min_ratio || dy > hook_lock_config_.size_change_min_ratio;
    bool reasonable = dx < hook_lock_config_.size_change_max_ratio && dy < hook_lock_config_.size_change_max_ratio;
    if (changed && reasonable) {
        hook_lock_.size_update_count++;
        hook_lock_.size_candidate_buffer.push_back(raw_size);
        if (hook_lock_.size_update_count >= hook_lock_config_.size_update_confirm_frames) {
            Eigen::Vector3f new_size = medianSize(hook_lock_.size_candidate_buffer);
            hook_lock_.locked_size = hook_lock_.locked_size * (1.0f - hook_lock_config_.size_update_alpha)
                                   + new_size * hook_lock_config_.size_update_alpha;
            hook_lock_.size_update_count = 0;
            hook_lock_.size_candidate_buffer.clear();
            ROS_INFO("[CargoBoxSizeLock] update size=(%.2f,%.2f,%.2f)", hook_lock_.locked_size.x(), hook_lock_.locked_size.y(), hook_lock_.locked_size.z());
        }
    } else {
        hook_lock_.size_update_count = 0;
        hook_lock_.size_candidate_buffer.clear();
    }
}

void NdtSlamNode::growUncertainty() {
    hook_lock_.bottom_uncertainty = std::min(
        hook_lock_.bottom_uncertainty + hook_lock_config_.bottom_hold_uncertainty_growth,
        hook_lock_config_.bottom_max_uncertainty);
}

void NdtSlamNode::clearHookLock() {
    hook_lock_.state = HookCargoLockState::EMPTY;
    hook_lock_.confirm_count = 0;
    hook_lock_.weak_count = 0;
    hook_lock_.lost_count = 0;
    hook_lock_.has_locked_size = false;
    hook_lock_.has_good_height = false;
    hook_lock_.size_update_count = 0;
    hook_lock_.init_size_buffer.clear();
    hook_lock_.size_candidate_buffer.clear();
    hook_lock_.last_accepted_core_points.reset();
    hook_lock_.last_accepted_center.setZero();
    hook_lock_.has_last_accepted = false;
}

void NdtSlamNode::updateHookCargoLock(
    const HookCargoDetection& det,
    const HookCargoBottomEstimate& bottom,
    const ros::Time& stamp) {

    hook_observation_associated_current_ = false;
    hook_observation_association_stamp_ = stamp;
    if (!hook_lock_config_.enabled) return;
    bool observation_associated = false;

    // LOCKED 阶段使用普通 strong（association gate 会过滤）
    bool strong = isStrongDetection(det, bottom);
    bool weak = isWeakDetection(det);

    ROS_DEBUG_THROTTLE(2.0,
        "[UpdateHookCargoLock] state=%d det.valid=%d points=%zu bottom.valid=%d",
        static_cast<int>(hook_lock_.state),
        det.valid ? 1 : 0,
        det.core_points_base ? det.core_points_base->size() : 0,
        bottom.valid ? 1 : 0);

    switch (hook_lock_.state) {
    case HookCargoLockState::EMPTY:
        if (isStrongDetection(det, bottom)) {
            hook_lock_.state = HookCargoLockState::CANDIDATE;
            hook_lock_.confirm_count = 1;
            hook_lock_.init_size_buffer.clear();
            hook_lock_.init_size_buffer.push_back(computeFixedCenterSize(det, bottom));
            hook_lock_.last_seen_stamp = stamp;
            ROS_INFO("[CargoLock] EMPTY->CANDIDATE confirm=1 points=%zu bottom=%.2f top=%.2f",
                     det.core_points_base ? det.core_points_base->size() : 0,
                     bottom.valid ? bottom.bottom_z_base : -1.0f,
                     bottom.valid ? bottom.top_z_base : -1.0f);
        }
        break;

    case HookCargoLockState::CANDIDATE:
        if (isStrongDetection(det, bottom)) {
            hook_lock_.confirm_count++;
            hook_lock_.weak_count = 0;
            hook_lock_.init_size_buffer.push_back(computeFixedCenterSize(det, bottom));
            hook_lock_.last_seen_stamp = stamp;
            ROS_INFO("[CargoLock] CANDIDATE confirm=%d points=%zu visible_h=%.2f xy_area=%.2f bottom=%.2f top=%.2f",
                     hook_lock_.confirm_count,
                     det.core_points_base ? det.core_points_base->size() : 0,
                     det.visible_height,
                     det.xy_area,
                     bottom.valid ? bottom.bottom_z_base : -1.0f,
                     bottom.valid ? bottom.top_z_base : -1.0f);

            if (hook_lock_.confirm_count >= hook_lock_config_.lock_confirm_frames) {
                hook_lock_.state = HookCargoLockState::LOCKED;
                observation_associated = true;
                hook_lock_.locked_size = medianSize(hook_lock_.init_size_buffer);
                hook_lock_.has_locked_size = true;
                hook_lock_.locked_stamp = stamp;

                // 使用 odom anchor 作为中心
                auto anchor = getCargoAnchorXY();
                hook_lock_.locked_center_base = Eigen::Vector3f(
                    anchor.x(), anchor.y(), det.center_base.z());
                hook_lock_.last_accepted_core_points = det.core_points_base;
                hook_lock_.last_accepted_center = det.center_base;
                hook_lock_.has_last_accepted =
                    static_cast<bool>(det.core_points_base) &&
                    !det.core_points_base->empty();
                ROS_WARN("[CargoCenterLock] mode=odom_anchor anchor=(%.2f,%.2f)",
                         anchor.x(), anchor.y());

                updateLockedHeight(bottom, stamp, true);
                ROS_WARN("[CargoLock] CANDIDATE->LOCKED size=(%.2f,%.2f,%.2f) anchor=(%.2f,%.2f)",
                         hook_lock_.locked_size.x(), hook_lock_.locked_size.y(), hook_lock_.locked_size.z(),
                         anchor.x(), anchor.y());
            }
        } else if (weak) {
            // 弱检测保持 CANDIDATE，不清零
            hook_lock_.weak_count++;
            hook_lock_.last_seen_stamp = stamp;
            ROS_INFO("[CargoLock] CANDIDATE weak_hold confirm=%d weak=%d",
                     hook_lock_.confirm_count, hook_lock_.weak_count);
        } else {
            // 无检测，检查超时
            double age = (stamp - hook_lock_.last_seen_stamp).toSec();
            if (age < hook_lock_config_.candidate_hold_sec) {
                ROS_INFO("[CargoLock] CANDIDATE no_detect_hold age=%.2f confirm=%d",
                         age, hook_lock_.confirm_count);
            } else {
                hook_lock_.state = HookCargoLockState::EMPTY;
                hook_lock_.confirm_count = 0;
                ROS_INFO("[CargoLock] CANDIDATE->EMPTY (timeout %.1f)", age);
            }
        }
        break;

    case HookCargoLockState::LOCKED:
        if (strong || weak) {
            // Association gate：检查检测是否与 locked box 一致
            std::string reject_reason;
            bool accepted = isDetectionConsistentWithLockedBox(det, bottom, &reject_reason);

            if (accepted) {
                observation_associated = true;
                hook_lock_.last_seen_stamp = stamp;
                // 更新高度和尺寸
                if (bottom.valid) updateLockedHeight(bottom, stamp, false);
                if (strong) maybeUpdateLockedSize(det, bottom);

                // 更新 last accepted
                if (det.core_points_base && !det.core_points_base->empty()) {
                    hook_lock_.last_accepted_core_points = det.core_points_base;
                    hook_lock_.last_accepted_center = det.center_base;
                    hook_lock_.has_last_accepted = true;
                }

                auto anchor = getCargoAnchorXY();
                ROS_DEBUG_THROTTLE(1.0, "[CargoLockUpdate] accepted=1 reason=%s raw_center=(%.2f,%.2f,%.2f) anchor=(%.2f,%.2f) raw_points=%zu",
                         reject_reason.c_str(),
                         det.center_base.x(), det.center_base.y(), det.center_base.z(),
                         anchor.x(), anchor.y(),
                         det.core_points_base ? det.core_points_base->size() : 0);
            } else {
                // 不更新高度和尺寸，保持 last good
                growUncertainty();

                auto anchor = getCargoAnchorXY();
                ROS_DEBUG_THROTTLE(2.0,
                    "[CargoLockUpdate] accepted=0 reason=%s raw_center=(%.2f,%.2f,%.2f) anchor=(%.2f,%.2f) raw_points=%zu",
                    reject_reason.c_str(),
                    det.center_base.x(), det.center_base.y(), det.center_base.z(),
                    anchor.x(), anchor.y(),
                    det.core_points_base ? det.core_points_base->size() : 0);
                const double lost_age =
                    (stamp - hook_lock_.last_seen_stamp).toSec();
                if (lost_age >= hook_lock_config_.lost_hold_sec) {
                    hook_lock_.state = HookCargoLockState::LOST_HOLD;
                    ROS_WARN_THROTTLE(
                        2.0,
                        "[CargoLock] association rejected for %.2fs; "
                        "entering LOST_HOLD reason=%s",
                        lost_age, reject_reason.c_str());
                }
            }
        } else {
            // 无检测
            growUncertainty();
            double lost_age = (stamp - hook_lock_.last_seen_stamp).toSec();
            if (lost_age >= hook_lock_config_.lost_hold_sec) {
                hook_lock_.state = HookCargoLockState::LOST_HOLD;
                ROS_INFO("[CargoLock] LOCKED->LOST_HOLD lost_age=%.1f", lost_age);
            }
        }
        break;

    case HookCargoLockState::LOST_HOLD:
        if (strong || weak) {
            std::string reject_reason;
            const bool accepted =
                isDetectionConsistentWithLockedBox(det, bottom, &reject_reason);
            if (accepted) {
                observation_associated = true;
                hook_lock_.state = HookCargoLockState::LOCKED;
                hook_lock_.last_seen_stamp = stamp;
                if (bottom.valid) updateLockedHeight(bottom, stamp, false);
                if (strong) maybeUpdateLockedSize(det, bottom);
                ROS_INFO("[CargoLock] LOST_HOLD->LOCKED (associated recovery)");
            } else {
                growUncertainty();
                const double lost_age =
                    (stamp - hook_lock_.last_seen_stamp).toSec();
                ROS_WARN_THROTTLE(
                    2.0,
                    "[CargoLock] LOST_HOLD recovery rejected age=%.2f reason=%s",
                    lost_age, reject_reason.c_str());
                if (lost_age > hook_lock_config_.lost_clear_sec) {
                    clearHookLock();
                }
            }
        } else {
            double lost_age = (stamp - hook_lock_.last_seen_stamp).toSec();
            growUncertainty();
            if (lost_age > hook_lock_config_.lost_clear_sec) {
                clearHookLock();
                ROS_WARN("[CargoLock] LOST_HOLD->CLEAR (timeout %.1f)", lost_age);
            }
        }
        break;
    }

    // ========== 更新 CargoState ==========
    // 将 HookCargoLock 状态同步到统一的 CargoState
    hook_observation_associated_current_ = observation_associated;
    HookCargoDetection associated_detection = det;
    HookCargoBottomEstimate associated_bottom = bottom;
    if (!observation_associated) {
        associated_detection.valid = false;
        associated_detection.core_points_base.reset();
        associated_bottom.valid = false;
    }

    // Only an observation accepted by the lock association gate may update
    // CargoState geometry or height. Rejected clutter ages the prior track out.
    updateCargoState(associated_detection, associated_bottom, stamp);
}

void NdtSlamNode::updateCargoState(
    const HookCargoDetection& det,
    const HookCargoBottomEstimate& bottom,
    const ros::Time& stamp) {

    // 同步状态
    switch (hook_lock_.state) {
    case HookCargoLockState::EMPTY:
        cargo_state_.state = CargoState::EMPTY;
        cargo_state_.valid_geometry = false;
        cargo_state_.valid_height = false;
        break;

    case HookCargoLockState::CANDIDATE:
        cargo_state_.state = CargoState::CANDIDATE;
        // CANDIDATE 阶段不更新几何，等待 LOCKED
        break;

    case HookCargoLockState::LOCKED:
    case HookCargoLockState::LOST_HOLD:
        cargo_state_.state = (hook_lock_.state == HookCargoLockState::LOCKED) ?
                            CargoState::LOCKED : CargoState::LOST;
        cargo_state_.locked_frames++;
        cargo_state_.stamp = stamp;

        // 使用 TightBox 观测更新几何
        if (det.valid && det.core_points_base && !det.core_points_base->empty()) {
            // 保存观测
            last_tight_box_obs_.valid = true;
            last_tight_box_obs_.center_base = det.center_base;
            last_tight_box_obs_.size = det.size_visible;
            last_tight_box_obs_.z_min = det.z05;
            last_tight_box_obs_.z_max = det.z95;
            last_tight_box_obs_.selected_points = det.core_points_base->size();
            last_tight_box_obs_.source = "tight_box";

            // 自适应更新 center（允许一定偏移）
            if (cargo_state_.valid_geometry) {
                float center_alpha = 0.25f;
                float max_center_step = 0.08f;

                Eigen::Vector3f center_diff = det.center_base - cargo_state_.center_base;
                for (int i = 0; i < 2; ++i) {  // 只更新 x, y
                    float step = std::max(-max_center_step, std::min(center_diff(i), max_center_step));
                    cargo_state_.center_base(i) += center_alpha * step;
                }

                // 自适应更新 size
                float size_alpha = 0.30f;
                float max_size_step = 0.10f;

                Eigen::Vector3f size_diff = det.size_visible - cargo_state_.size;
                for (int i = 0; i < 3; ++i) {
                    float step = std::max(-max_size_step, std::min(size_diff(i), max_size_step));
                    cargo_state_.size(i) += size_alpha * step;
                }
            } else {
                // 初始化
                cargo_state_.center_base = det.center_base;
                cargo_state_.size = det.size_visible;
            }

            cargo_state_.valid_geometry = true;
            cargo_state_.source = "tight_box";
        }

        // 更新高度（带稳定保护）
        if (bottom.valid) {
            float new_bottom = bottom.bottom_z_base;
            float new_top = bottom.top_z_base;

            // 底部高度稳定保护
            if (cargo_state_.valid_height) {
                float dz = new_bottom - cargo_state_.bottom_z;

                // 单帧突然下降超过 0.45m，先 hold
                if (dz < -0.45f) {
                    low_bottom_reject_count_++;
                    if (low_bottom_reject_count_ < 3) {
                        // 保持上一帧高度
                        ROS_INFO_THROTTLE(1.0, "[CargoHeightFilter] reject_bottom_drop dz=%.2f hold_count=%d reason=large_drop",
                                         dz, low_bottom_reject_count_);
                        // 不更新 bottom_z, top_z
                    } else {
                        // 连续 3 帧确认，接受新高度
                        cargo_state_.bottom_z = new_bottom;
                        cargo_state_.top_z = new_top;
                        cargo_state_.bottom_unc = bottom.uncertainty;
                        low_bottom_reject_count_ = 0;
                        ROS_INFO("[CargoHeightFilter] accept_bottom_drop dz=%.2f confirmed_frames=3", dz);
                    }
                } else if (dz > 0.60f) {
                    // 单帧突然上升超过 0.60m，先 hold
                    high_bottom_reject_count_++;
                    if (high_bottom_reject_count_ < 3) {
                        ROS_INFO_THROTTLE(1.0, "[CargoHeightFilter] reject_bottom_rise dz=%.2f hold_count=%d reason=large_rise",
                                         dz, high_bottom_reject_count_);
                    } else {
                        cargo_state_.bottom_z = new_bottom;
                        cargo_state_.top_z = new_top;
                        cargo_state_.bottom_unc = bottom.uncertainty;
                        high_bottom_reject_count_ = 0;
                        ROS_INFO("[CargoHeightFilter] accept_bottom_rise dz=%.2f confirmed_frames=3", dz);
                    }
                } else {
                    // 正常更新
                    cargo_state_.bottom_z = new_bottom;
                    cargo_state_.top_z = new_top;
                    cargo_state_.bottom_unc = bottom.uncertainty;
                    low_bottom_reject_count_ = 0;
                    high_bottom_reject_count_ = 0;
                }
            } else {
                // 首次有效高度
                cargo_state_.bottom_z = new_bottom;
                cargo_state_.top_z = new_top;
                cargo_state_.bottom_unc = bottom.uncertainty;
            }

            cargo_state_.valid_height = true;
        } else if (det.valid && !cargo_state_.valid_height) {
            // CargoBottom invalid 但 TightBox z 有效，使用 fallback
            cargo_state_.bottom_z = det.z05;
            cargo_state_.top_z = det.z95;
            cargo_state_.bottom_unc = 0.18f;
            cargo_state_.valid_height = true;
            cargo_state_.source = "tight_box_z_fallback";
        }

        // 计算 bottom_safe_z
        if (cargo_state_.valid_height) {
            cargo_state_.bottom_safe_z = cargo_state_.bottom_z - cargo_state_.bottom_unc - 0.05f;
        }

        // ========== 同步 CargoState 回 hook_lock_ ==========
        // 让所有下游（Marker、Removal、Warning）使用同一份数据
        if (cargo_state_.valid_geometry) {
            hook_lock_.locked_center_base = cargo_state_.center_base;
            hook_lock_.last_accepted_center = cargo_state_.center_base;
            hook_lock_.locked_size = cargo_state_.size;
            hook_lock_.has_locked_size = true;
        }

        if (cargo_state_.valid_height) {
            hook_lock_.stable_bottom_z = cargo_state_.bottom_z;
            hook_lock_.stable_top_z = cargo_state_.top_z;
            hook_lock_.bottom_uncertainty = cargo_state_.bottom_unc;
            hook_lock_.has_good_height = true;
        }

        break;
    }

    // 日志
    if (debug_cfg_.debug_cargo && (cargo_state_.state == CargoState::LOCKED || cargo_state_.state == CargoState::LOST)) {
        ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec, "[CargoState] state=%s center=(%.2f,%.2f) size=(%.2f,%.2f,%.2f) z=[%.2f,%.2f] valid_h=%d source=%s",
                         cargo_state_.state == CargoState::LOCKED ? "LOCKED" : "LOST",
                         cargo_state_.center_base.x(), cargo_state_.center_base.y(),
                         cargo_state_.size.x(), cargo_state_.size.y(), cargo_state_.size.z(),
                         cargo_state_.bottom_z, cargo_state_.top_z,
                         cargo_state_.valid_height ? 1 : 0,
                         cargo_state_.source.c_str());
    }
}

void NdtSlamNode::publishCargoFusionMarker(
    const CargoBottomResult& bottom, const ros::Time& stamp) {
    if (cargo_fused_box_marker_pub_.getNumSubscribers() == 0U) return;

    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = "cargo_fused_box";
    marker.id = 0;
    marker.action = bottom.valid
        ? visualization_msgs::Marker::ADD
        : visualization_msgs::Marker::DELETE;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.r = bottom.source == CargoBottomSource::POINTS ? 0.1F : 1.0F;
    marker.color.g = bottom.source == CargoBottomSource::POINTS ? 1.0F : 0.7F;
    marker.color.b = 0.1F;
    marker.color.a = 0.95F;
    if (bottom.valid) {
        constexpr int kEdges[12][2] = {
            {0, 1}, {1, 3}, {3, 2}, {2, 0},
            {4, 5}, {5, 7}, {7, 6}, {6, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}};
        marker.points.reserve(24U);
        for (const auto& edge : kEdges) {
            for (int endpoint : edge) {
                geometry_msgs::Point point;
                const Eigen::Vector3f& corner =
                    bottom.geometry.corners_map[static_cast<std::size_t>(endpoint)];
                point.x = corner.x();
                point.y = corner.y();
                point.z = corner.z();
                marker.points.push_back(point);
            }
        }
    }
    cargo_fused_box_marker_pub_.publish(marker);
}

void NdtSlamNode::publishHookOnlySafetyStatus(
    const HookLoadSnapshot& hook, const ros::Time& stamp,
    bool visual_conflict, const std::string& reason) {
    const bool safe_empty = hook.valid &&
        hook.state == lidar_slam2_msgs::HookLoadState::STATE_EMPTY &&
        !visual_conflict;

    lidar_slam2_msgs::CargoSafetyStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = map_frame_;
    status.schema_version = lidar_slam2_msgs::CargoSafetyStatus::SCHEMA_VERSION;
    status.valid = safe_empty;
    status.cargo_valid = false;
    status.cargo_source =
        lidar_slam2_msgs::CargoBottomEstimate::SOURCE_INVALID;
    status.requested_alarm_code = safe_empty
        ? lidar_slam2_msgs::CargoSafetyStatus::ALARM_CLEAR
        : lidar_slam2_msgs::CargoSafetyStatus::ALARM_OUTER_OR_INVALID;
    status.hook_signal_valid = hook.valid;
    status.hook_load_state = hook.state;
    status.hook_voltage = hook.voltage;
    status.no_cargo_confirmed = safe_empty;
    status.obstacle_valid = false;
    status.confidence = safe_empty ? 1.0F : 0.0F;
    status.reason = reason.empty() ? hook.reason : reason;
    cargo_safety_status_pub_.publish(status);

    lidar_slam2_msgs::CargoBottomEstimate bottom;
    bottom.header = status.header;
    bottom.schema_version =
        lidar_slam2_msgs::CargoBottomEstimate::SCHEMA_VERSION;
    bottom.valid = false;
    bottom.source = lidar_slam2_msgs::CargoBottomEstimate::SOURCE_INVALID;
    bottom.source_detail = "INVALID";
    bottom.invalid_reason = status.reason;
    bottom.uncertainty_m = cargo_bottom_fusion_.config().invalid_uncertainty;
    cargo_bottom_estimate_pub_.publish(bottom);
    publishCargoFusionMarker(CargoBottomResult{}, stamp);
    publishPayloadTrackInfoInvalid(status.reason);
}

void NdtSlamNode::updateAndPublishCargoSafetyPipeline(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obstacle_cloud_base,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& observation_cloud_base,
    const Sophus::SE3d& pose_map_base,
    const ros::Time& stamp) {
    const HookLoadSnapshot hook = currentHookLoadSnapshot();
    const bool visual_conflict = hook_fixed_cargo_.valid;
    if (!hook.valid ||
        hook.state == lidar_slam2_msgs::HookLoadState::STATE_UNKNOWN ||
        hook.state == lidar_slam2_msgs::HookLoadState::STATE_INHIBIT) {
        publishHookOnlySafetyStatus(
            hook, stamp, visual_conflict,
            std::string("hook_signal_invalid:") + hook.reason);
        return;
    }
    if (hook.state == lidar_slam2_msgs::HookLoadState::STATE_EMPTY) {
        publishHookOnlySafetyStatus(
            hook, stamp, visual_conflict,
            visual_conflict ? "empty_hook_visual_conflict"
                            : "empty_hook_no_cargo_confirmed");
        return;
    }

    // LOST geometry may be retained for short-term association, but it is not
    // current enough to drive bottom height or a non-fail-safe alarm.
    const bool active_track =
        cargo_state_.state == CargoState::LOCKED &&
        cargo_state_.valid_geometry;
    if (active_track && !cargo_fusion_track_active_) {
        ++cargo_fusion_track_id_;
        if (cargo_fusion_track_id_ == 0U) ++cargo_fusion_track_id_;
        cargo_bottom_fusion_.reset();
        cargo_origin_height_valid_ = false;
        cargo_origin_height_m_ = 0.0F;
        cargo_origin_height_track_id_ = cargo_fusion_track_id_;
        {
            std::lock_guard<std::mutex> lock(hook_load_state_mutex_);
            if (pending_origin_height_valid_) {
                cargo_origin_height_valid_ = true;
                cargo_origin_height_m_ = pending_origin_height_m_;
            }
        }
    } else if (!active_track && cargo_fusion_track_active_) {
        cargo_bottom_fusion_.reset();
        cargo_origin_height_valid_ = false;
    }
    cargo_fusion_track_active_ = active_track;

    CargoBottomObservation observation;
    observation.track_valid = active_track;
    observation.track_id = cargo_fusion_track_id_;
    observation.stamp_sec = stamp.toSec();
    observation.transform_stamp_sec = stamp.toSec();
    observation.T_map_base = Eigen::Isometry3f::Identity();
    observation.T_map_base.matrix() = pose_map_base.matrix().cast<float>();

    const bool detection_is_current = hook_fixed_cargo_.valid &&
        hook_observation_associated_current_ &&
        hook_observation_association_stamp_ == stamp &&
        !last_anchor_detect_stamp_.isZero() &&
        std::abs((last_anchor_detect_stamp_ - stamp).toSec()) <= 1.0e-4;
    if (detection_is_current && hook_fixed_cargo_.core_points_base) {
        observation.points_base.reserve(
            hook_fixed_cargo_.core_points_base->size());
        for (const auto& point : hook_fixed_cargo_.core_points_base->points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) &&
                std::isfinite(point.z)) {
                observation.points_base.emplace_back(point.x, point.y, point.z);
            }
        }
        observation.current_top_valid = std::isfinite(hook_fixed_cargo_.z95);
        observation.current_top_z_base = hook_fixed_cargo_.z95;
    }
    if (active_track) {
        observation.footprint_valid = cargo_state_.size.x() > 0.0F &&
                                      cargo_state_.size.y() > 0.0F;
        observation.footprint_center_base = cargo_state_.center_base.head<2>();
        observation.footprint_size_xy = cargo_state_.size.head<2>();
        observation.track_center_valid = cargo_state_.center_base.allFinite();
        observation.track_center_base = cargo_state_.center_base;
        const bool origin_height_matches_track = cargo_origin_height_valid_ &&
            cargo_origin_height_track_id_ == cargo_fusion_track_id_;
        observation.prior_height_valid = origin_height_matches_track;
        observation.prior_height_m = cargo_origin_height_m_;
        observation.origin_height_valid = origin_height_matches_track;
        observation.origin_height_m = cargo_origin_height_m_;
        observation.map_static_height_valid = origin_height_matches_track;
        observation.map_static_height_m = cargo_origin_height_m_;
    }

    last_cargo_bottom_result_ = cargo_bottom_fusion_.update(observation);
    last_cargo_pipeline_stamp_ = stamp;
    if (last_cargo_bottom_result_.valid) {
        cargo_state_.valid_height = true;
        cargo_state_.bottom_z =
            last_cargo_bottom_result_.geometry.bottom_z_base;
        cargo_state_.top_z = last_cargo_bottom_result_.geometry.top_z_base;
        cargo_state_.bottom_unc = last_cargo_bottom_result_.uncertainty;
        cargo_state_.bottom_safe_z = cargo_state_.bottom_z -
            cargo_state_.bottom_unc - 0.05F;
        cargo_state_.source =
            std::string("fusion:") + last_cargo_bottom_result_.source_name;
    } else {
        // Never let the compatibility topics or legacy warning path keep a
        // stale height after the formal fusion has rejected current evidence.
        cargo_state_.valid_height = false;
        cargo_state_.bottom_unc =
            cargo_bottom_fusion_.config().invalid_uncertainty;
        cargo_state_.source = "fusion:INVALID";
    }

    lidar_slam2_msgs::CargoBottomEstimate bottom_msg;
    bottom_msg.header.stamp = stamp;
    bottom_msg.header.frame_id = map_frame_;
    bottom_msg.schema_version =
        lidar_slam2_msgs::CargoBottomEstimate::SCHEMA_VERSION;
    bottom_msg.valid = last_cargo_bottom_result_.valid;
    bottom_msg.track_id = static_cast<std::uint32_t>(
        std::min<std::uint64_t>(cargo_fusion_track_id_,
                               std::numeric_limits<std::uint32_t>::max()));
    bottom_msg.track_state = static_cast<std::uint8_t>(cargo_state_.state);
    bottom_msg.source = static_cast<std::uint8_t>(
        last_cargo_bottom_result_.source);
    bottom_msg.source_detail = last_cargo_bottom_result_.source_name;
    bottom_msg.invalid_reason = last_cargo_bottom_result_.valid
        ? std::string() : last_cargo_bottom_result_.reason;
    if (last_cargo_bottom_result_.geometry_valid) {
        const auto& geometry = last_cargo_bottom_result_.geometry;
        bottom_msg.bottom_z_base = geometry.bottom_z_base;
        bottom_msg.top_z_base = geometry.top_z_base;
        bottom_msg.bottom_z_map = geometry.bottom_z_map;
        bottom_msg.top_z_map = geometry.top_z_map;
        for (std::size_t i = 0; i < geometry.corners_base.size(); ++i) {
            bottom_msg.corners_base[i].x = geometry.corners_base[i].x();
            bottom_msg.corners_base[i].y = geometry.corners_base[i].y();
            bottom_msg.corners_base[i].z = geometry.corners_base[i].z();
            bottom_msg.corners_map[i].x = geometry.corners_map[i].x();
            bottom_msg.corners_map[i].y = geometry.corners_map[i].y();
            bottom_msg.corners_map[i].z = geometry.corners_map[i].z();
        }
    }
    bottom_msg.height_m = last_cargo_bottom_result_.height;
    bottom_msg.uncertainty_m = last_cargo_bottom_result_.uncertainty;
    bottom_msg.confidence = last_cargo_bottom_result_.confidence;
    bottom_msg.z02_base = last_cargo_bottom_result_.selected_stats.z02;
    bottom_msg.z05_base = last_cargo_bottom_result_.selected_stats.z05;
    bottom_msg.z50_base = last_cargo_bottom_result_.selected_stats.z50;
    bottom_msg.z95_base = last_cargo_bottom_result_.selected_stats.z95;
    bottom_msg.sample_count = static_cast<std::uint32_t>(
        std::min<std::size_t>(last_cargo_bottom_result_.selected_stats.finite_points,
                              std::numeric_limits<std::uint32_t>::max()));
    bottom_msg.support_point_count = static_cast<std::uint32_t>(
        std::min<std::size_t>(
            last_cargo_bottom_result_.selected_stats.bottom_band_points,
            std::numeric_limits<std::uint32_t>::max()));
    bottom_msg.support_ratio =
        last_cargo_bottom_result_.selected_stats.bottom_band_point_ratio;
    cargo_bottom_estimate_pub_.publish(bottom_msg);
    publishCargoFusionMarker(last_cargo_bottom_result_, stamp);

    CargoSafetyInput safety_input;
    safety_input.evaluation_time_sec = stamp.toSec();
    safety_input.height.valid = last_cargo_bottom_result_.valid;
    safety_input.height.stale = false;
    safety_input.height.stamp_sec = last_cargo_bottom_result_.stamp_sec;
    safety_input.height.bottom_z =
        last_cargo_bottom_result_.geometry.bottom_z_base;
    safety_input.height.bottom_uncertainty_m =
        last_cargo_bottom_result_.uncertainty;
    if (last_cargo_bottom_result_.geometry_valid) {
        safety_input.footprint_base.min_x =
            last_cargo_bottom_result_.geometry.corners_base[0].x();
        safety_input.footprint_base.max_x =
            last_cargo_bottom_result_.geometry.corners_base[3].x();
        safety_input.footprint_base.min_y =
            last_cargo_bottom_result_.geometry.corners_base[0].y();
        safety_input.footprint_base.max_y =
            last_cargo_bottom_result_.geometry.corners_base[3].y();
        safety_input.footprint_base.min_z =
            last_cargo_bottom_result_.geometry.bottom_z_base;
        safety_input.footprint_base.max_z =
            last_cargo_bottom_result_.geometry.top_z_base;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_roi(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::set<std::pair<int, int>> observed_cells;
    std::size_t roi_finite_points = 0U;
    float coverage_ratio = 0.0F;
    if (last_cargo_bottom_result_.geometry_valid) {
        const float radius = cargo_safety_evaluator_.config().level2_distance_m;
        const float min_x = safety_input.footprint_base.min_x - radius;
        const float max_x = safety_input.footprint_base.max_x + radius;
        const float min_y = safety_input.footprint_base.min_y - radius;
        const float max_y = safety_input.footprint_base.max_y + radius;
        constexpr float kCoverageCell = 0.50F;
        if (obstacle_cloud_base) {
            obstacle_roi->reserve(obstacle_cloud_base->size());
            for (const auto& point : obstacle_cloud_base->points) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    !std::isfinite(point.z) || point.x < min_x ||
                    point.x > max_x || point.y < min_y || point.y > max_y) {
                    continue;
                }
                obstacle_roi->push_back(point);
            }
        }
        if (observation_cloud_base) {
            for (const auto& point : observation_cloud_base->points) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    !std::isfinite(point.z) || point.x < min_x ||
                    point.x > max_x || point.y < min_y || point.y > max_y) {
                    continue;
                }
                ++roi_finite_points;
                observed_cells.emplace(
                    static_cast<int>(
                        std::floor((point.x - min_x) / kCoverageCell)),
                    static_cast<int>(
                        std::floor((point.y - min_y) / kCoverageCell)));
            }
        }
        const std::size_t cells_x = static_cast<std::size_t>(
            std::max(1.0, std::ceil(
                static_cast<double>((max_x - min_x) / kCoverageCell))));
        const std::size_t cells_y = static_cast<std::size_t>(
            std::max(1.0, std::ceil(
                static_cast<double>((max_y - min_y) / kCoverageCell))));
        const std::size_t expected_cells = cells_x * cells_y;
        coverage_ratio = expected_cells > 0U
            ? static_cast<float>(observed_cells.size()) /
                  static_cast<float>(expected_cells)
            : 0.0F;
    }
    safety_input.obstacle_cloud_base = obstacle_roi;
    safety_input.obstacle_observation_valid =
        static_cast<bool>(obstacle_cloud_base) &&
        static_cast<bool>(observation_cloud_base) &&
        last_cargo_bottom_result_.geometry_valid;
    safety_input.obstacle_cloud_age_sec = 0.0;
    safety_input.obstacle_roi_finite_points = roi_finite_points;
    safety_input.obstacle_roi_coverage_ratio = coverage_ratio;
    last_cargo_safety_result_ = cargo_safety_evaluator_.evaluate(safety_input);

    lidar_slam2_msgs::CargoSafetyStatus safety_msg;
    safety_msg.header.stamp = stamp;
    safety_msg.header.frame_id = map_frame_;
    safety_msg.schema_version =
        lidar_slam2_msgs::CargoSafetyStatus::SCHEMA_VERSION;
    safety_msg.valid = last_cargo_bottom_result_.valid &&
                       last_cargo_safety_result_.input_valid;
    safety_msg.cargo_valid = last_cargo_bottom_result_.valid;
    safety_msg.cargo_track_id = bottom_msg.track_id;
    safety_msg.cargo_source = bottom_msg.source;
    safety_msg.hook_signal_valid = hook.valid;
    safety_msg.hook_load_state = hook.state;
    safety_msg.hook_voltage = hook.voltage;
    safety_msg.no_cargo_confirmed = false;
    safety_msg.requested_alarm_code =
        static_cast<std::int32_t>(last_cargo_safety_result_.raw_code);
    safety_msg.cargo_bottom_z_map = bottom_msg.bottom_z_map;
    safety_msg.cargo_bottom_uncertainty_m = bottom_msg.uncertainty_m;
    safety_msg.obstacle_valid =
        last_cargo_safety_result_.has_cluster_evidence;
    safety_msg.obstacle_count = static_cast<std::uint32_t>(
        std::min<std::size_t>(
            last_cargo_safety_result_.evaluated_cluster_count,
            std::numeric_limits<std::uint32_t>::max()));
    if (last_cargo_safety_result_.has_cluster_evidence) {
        const auto& evidence =
            last_cargo_safety_result_.most_dangerous_cluster;
        safety_msg.nearest_obstacle_distance_m = evidence.footprint_distance_m;
        Eigen::Vector3d obstacle_base(
            evidence.nearest_point_base.x, evidence.nearest_point_base.y,
            evidence.obstacle_top_z95_m);
        const Eigen::Vector3d obstacle_map = pose_map_base * obstacle_base;
        safety_msg.obstacle_top_z_map = static_cast<float>(obstacle_map.z());
        safety_msg.obstacle_uncertainty_m = evidence.obstacle_uncertainty_m;
        safety_msg.conservative_vertical_clearance_m =
            evidence.conservative_clearance_m;
    }
    safety_msg.confidence = safety_msg.valid
        ? last_cargo_bottom_result_.confidence : 0.0F;
    safety_msg.reason = last_cargo_bottom_result_.valid
        ? last_cargo_safety_result_.reason
        : std::string("cargo_bottom_invalid:") +
              last_cargo_bottom_result_.reason;
    cargo_safety_status_pub_.publish(safety_msg);
    publishPayloadTrackInfoFromFusion(last_cargo_bottom_result_, stamp);
}

void NdtSlamNode::publishPayloadTrackInfoFromFusion(
    const CargoBottomResult& bottom,
    const ros::Time& stamp) {
    (void)stamp;
    if (!bottom.valid || !bottom.geometry_valid) {
        publishPayloadTrackInfoInvalid(
            bottom.reason.empty() ? "fusion_invalid" : bottom.reason);
        return;
    }

    const CargoBoxGeometry& geometry = bottom.geometry;
    Eigen::Vector3f minimum = geometry.corners_base.front();
    Eigen::Vector3f maximum = geometry.corners_base.front();
    for (const Eigen::Vector3f& corner : geometry.corners_base) {
        minimum = minimum.cwiseMin(corner);
        maximum = maximum.cwiseMax(corner);
    }

    // Preserve the legacy box_source contract: 1 means direct physical point
    // evidence, 2 means a guarded fallback. The typed CargoBottomEstimate
    // carries the complete source enum and must be used by new consumers.
    const float compatibility_source =
        bottom.source == CargoBottomSource::POINTS ? 1.0F : 2.0F;
    std_msgs::Float32MultiArray message;
    message.data = {
        1.0F,
        static_cast<float>(bottom.track_id),
        static_cast<float>(cargo_state_.state),
        geometry.center_base.x(), geometry.center_base.y(),
        geometry.center_base.z(),
        0.0F, 0.0F, 0.0F,
        minimum.x(), minimum.y(), minimum.z(),
        maximum.x(), maximum.y(), maximum.z(),
        static_cast<float>(bottom.selected_stats.finite_points),
        bottom.confidence,
        geometry.bottom_z_base,
        bottom.selected_stats.bottom_band_point_ratio,
        compatibility_source
    };
    payload_track_info_pub_.publish(message);
}

NdtSlamNode::HookCargoBottomEstimate NdtSlamNode::estimateCargoBottom(const HookCargoDetection& detection) {
    HookCargoBottomEstimate result;

    if (!detection.valid || !detection.core_points_base || detection.core_points_base->empty()) {
        result.source = "invalid";
        result.uncertainty = hook_fixed_config_.invalid_uncertainty;
        return result;
    }

    // 计算 z05, z50, z95
    std::vector<float> z_values;
    for (const auto& p : detection.core_points_base->points) {
        z_values.push_back(p.z);
    }
    std::sort(z_values.begin(), z_values.end());

    float z05 = z_values[static_cast<size_t>(z_values.size() * 0.05)];
    float z50 = z_values[static_cast<size_t>(z_values.size() * 0.50)];
    float z95 = z_values[static_cast<size_t>(z_values.size() * 0.95)];
    float visible_height = z95 - z05;

    // 检查 side_visible
    int bottom_band_points = 0;
    for (float z : z_values) {
        if (z >= z05 && z <= z05 + hook_fixed_config_.bottom_band_height) {
            bottom_band_points++;
        }
    }

    bool side_visible = visible_height >= hook_fixed_config_.visible_side_min_height &&
                        bottom_band_points >= hook_fixed_config_.bottom_band_min_points;

    if (side_visible) {
        result.valid = true;
        result.bottom_z_base = z05;
        result.top_z_base = z95;
        result.height = visible_height;
        result.source = "points_visible_side";
        result.uncertainty = hook_fixed_config_.points_uncertainty;
        result.confidence = 0.8f;

        // 更新 stable_height
        if (!has_stable_height_) {
            stable_height_ = visible_height;
            has_stable_height_ = true;
        } else {
            stable_height_ = hook_fixed_config_.stable_height_alpha * visible_height +
                            (1.0f - hook_fixed_config_.stable_height_alpha) * stable_height_;
        }

        if (debug_cfg_.debug_cargo_bottom) {
            ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec, "[CargoBottom] source=points_visible_side z05=%.2f z50=%.2f z95=%.2f visible_h=%.2f band_pts=%d bottom=%.2f top=%.2f h=%.2f unc=%.2f conf=%.2f",
                              z05, z50, z95, visible_height, bottom_band_points,
                              result.bottom_z_base, result.top_z_base, result.height,
                              result.uncertainty, result.confidence);
        }
    } else if (has_stable_height_) {
        result.valid = true;
        result.top_z_base = z95;
        result.bottom_z_base = z95 - stable_height_;
        result.height = stable_height_;
        result.source = "height_memory";
        result.uncertainty = hook_fixed_config_.height_memory_uncertainty;
        result.confidence = 0.6f;

        if (debug_cfg_.debug_cargo_bottom) {
            ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec, "[CargoBottom] source=height_memory z95=%.2f stable_h=%.2f bottom=%.2f top=%.2f unc=%.2f",
                              z95, stable_height_, result.bottom_z_base, result.top_z_base, result.uncertainty);
        }
    } else {
        result.valid = false;
        result.source = "invalid";
        result.uncertainty = hook_fixed_config_.invalid_uncertainty;
        result.confidence = 0.0f;

        if (debug_cfg_.debug_cargo_bottom) {
            ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec, "[CargoBottom] source=invalid reason=no_side_no_height_memory z05=%.2f z95=%.2f visible_h=%.2f",
                              z05, z95, visible_height);
        }
    }

    return result;
}

void NdtSlamNode::publishSelectedCorePoints(const HookCargoDetection& detection, const ros::Time& stamp) {
    sensor_msgs::PointCloud2 msg;
    if (detection.valid && detection.core_points_base && !detection.core_points_base->empty()) {
        pcl::toROSMsg(*detection.core_points_base, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = "base_link";

        ROS_DEBUG_THROTTLE(1.0, "[CargoSelectedCorePoints] source=hook_fixed_roi points=%zu frame=base_link",
                          detection.core_points_base->size());
    } else {
        msg.header.stamp = stamp;
        msg.header.frame_id = "base_link";
    }
    cargo_selected_core_points_pub_.publish(msg);
}

void NdtSlamNode::publishSelectedCorePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Time& stamp) {
    sensor_msgs::PointCloud2 msg;
    if (cloud && !cloud->empty()) {
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = "base_link";

        ROS_DEBUG_THROTTLE(1.0, "[CargoSelectedCorePoints] source=locked_last_good points=%zu frame=base_link",
                          cloud->size());
    } else {
        msg.header.stamp = stamp;
        msg.header.frame_id = "base_link";
    }
    cargo_selected_core_points_pub_.publish(msg);
}

Eigen::Vector3f NdtSlamNode::smoothVector(const Eigen::Vector3f& current, const Eigen::Vector3f& new_val, float alpha) {
    return current * (1.0f - alpha) + new_val * alpha;
}

float NdtSlamNode::smoothFloat(float current, float new_val, float alpha) {
    return current * (1.0f - alpha) + new_val * alpha;
}

// ========== 吊货跟踪信息发布 ==========

void NdtSlamNode::publishPayloadTrackInfo(const ros::Time& stamp) {
    std_msgs::Float32MultiArray msg;

    // 使用 hook_lock_ 状态决定是否发布有效数据
    bool should_publish_valid = (hook_lock_.state == HookCargoLockState::LOCKED ||
                                 hook_lock_.state == HookCargoLockState::LOST_HOLD) &&
                                 hook_lock_.has_locked_size && hook_lock_.has_good_height;

    if (should_publish_valid) {
        // 强制使用 anchor
        auto anchor = getCargoAnchorXY();
        Eigen::Vector3f center(anchor.x(), anchor.y(), 0.0f);
        Eigen::Vector3f size = hook_lock_.locked_size;
        Eigen::Vector3f bbox_min = center - size / 2.0f;
        Eigen::Vector3f bbox_max = center + size / 2.0f;
        bbox_min.z() = hook_lock_.stable_bottom_z;
        bbox_max.z() = hook_lock_.stable_top_z;
        center.z() = 0.5f * (hook_lock_.stable_bottom_z + hook_lock_.stable_top_z);

        float box_source = (hook_lock_.state == HookCargoLockState::LOCKED) ? 1.0f : 2.0f;

        msg.data = {
            1.0f,  // IDX_VALID: 有效
            0.0f,  // IDX_TRACK_ID
            3.0f,  // IDX_STATE = SUSPENDED_MOVING
            center.x(), center.y(), center.z(),
            0.0f, 0.0f, 0.0f,  // velocity
            bbox_min.x(), bbox_min.y(), bbox_min.z(),
            bbox_max.x(), bbox_max.y(), bbox_max.z(),
            static_cast<float>(hook_lock_.has_locked_size ? 30 : 0),  // point_count
            1.0f,  // score
            hook_lock_.stable_bottom_z,  // bottom_hag
            hook_lock_.bottom_uncertainty,  // support_ratio (repurpose as uncertainty)
            box_source
        };

        ROS_INFO_THROTTLE(1.0,
            "[CargoSourceMode] mode=hook_local_roi global_dynamic_track=debug_only source=%s",
            box_source == 1.0f ? "V2_CORE" : "LAST_GOOD");

        ROS_INFO_THROTTLE(1.0,
            "[CargoTargetHardCheck] source=hook_roi selected=0 payload=0 match=1 state=%d",
            static_cast<int>(hook_lock_.state));
    } else {
        // 无效检测
        msg.data = {
            -1.0f,   // IDX_VALID: 无效
            -1.0f,   // IDX_TRACK_ID
            0.0f,    // IDX_STATE = NONE
            0.0f, 0.0f, 0.0f,  // centroid
            0.0f, 0.0f, 0.0f,  // velocity
            0.0f, 0.0f, 0.0f,  // bbox_min
            0.0f, 0.0f, 0.0f,  // bbox_max
            0.0f,              // point_count
            0.0f,              // score
            0.0f,              // bottom_hag
            0.0f,              // support_ratio
            0.0f               // box_source = NONE
        };

        ROS_INFO_THROTTLE(1.0,
            "[CargoTargetHardCheck] source=hook_roi selected=-1 payload=-1 match=0 reason=not_locked state=%d",
            static_cast<int>(hook_lock_.state));
    }

    payload_track_info_pub_.publish(msg);
}

// ========== 从锁定的 Hook Box 发布 payload_track_info ==========

// ========== 从 OdomAnchorBox 发布 payload_track_info ==========
void NdtSlamNode::publishPayloadTrackInfoFromOdomAnchorBox(const ros::Time& stamp) {
    std_msgs::Float32MultiArray msg;

    bool should_publish_valid = (hook_lock_.state == HookCargoLockState::LOCKED ||
                                 hook_lock_.state == HookCargoLockState::LOST_HOLD) &&
                                 hook_lock_.has_locked_size;

    if (should_publish_valid) {
        // 强制使用 anchor
        auto anchor = getCargoAnchorXY();
        float cx = anchor.x();
        float cy = anchor.y();

        Eigen::Vector3f size = hook_lock_.locked_size;
        float zmin = hook_lock_.stable_bottom_z;
        float zmax = hook_lock_.stable_top_z;

        Eigen::Vector3f bbox_min(cx - size.x() * 0.5f, cy - size.y() * 0.5f, zmin);
        Eigen::Vector3f bbox_max(cx + size.x() * 0.5f, cy + size.y() * 0.5f, zmax);
        Eigen::Vector3f center(cx, cy, 0.5f * (zmin + zmax));

        float box_source = (hook_lock_.state == HookCargoLockState::LOCKED) ? 1.0f : 2.0f;

        msg.data = {
            1.0f,  // IDX_VALID: 有效
            0.0f,  // IDX_TRACK_ID
            3.0f,  // IDX_STATE = SUSPENDED_MOVING
            center.x(), center.y(), center.z(),
            0.0f, 0.0f, 0.0f,  // velocity
            bbox_min.x(), bbox_min.y(), bbox_min.z(),
            bbox_max.x(), bbox_max.y(), bbox_max.z(),
            static_cast<float>(hook_lock_.has_locked_size ? 30 : 0),  // point_count
            1.0f,  // score
            hook_lock_.stable_bottom_z,  // bottom_hag
            hook_lock_.bottom_uncertainty,  // support_ratio
            box_source
        };

        if (debug_cfg_.debug_cargo) {
            ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec,
                "[CargoBoxLock] state=%s center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) z=[%.2f,%.2f]",
                hook_lock_.state == HookCargoLockState::LOCKED ? "LOCKED" : "LOST_HOLD",
                center.x(), center.y(), center.z(),
                size.x(), size.y(), size.z(),
                zmin, zmax);
        }

        if (debug_cfg_.debug_cargo) {
            ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec,
                "[CargoTargetHardCheck] source=odom_anchor selected=0 payload=0 match=1 state=%d",
                static_cast<int>(hook_lock_.state));
        }
    } else {
        msg.data = {
            -1.0f,   // IDX_VALID: 无效
            -1.0f,   // IDX_TRACK_ID
            0.0f,    // IDX_STATE = NONE
            0.0f, 0.0f, 0.0f,  // centroid
            0.0f, 0.0f, 0.0f,  // velocity
            0.0f, 0.0f, 0.0f,  // bbox_min
            0.0f, 0.0f, 0.0f,  // bbox_max
            0.0f,              // point_count
            0.0f,              // score
            0.0f,              // bottom_hag
            0.0f,              // support_ratio
            0.0f               // box_source = NONE
        };

        ROS_INFO_THROTTLE(1.0,
            "[CargoTargetHardCheck] source=odom_anchor selected=-1 payload=-1 match=0 reason=not_locked state=%d",
            static_cast<int>(hook_lock_.state));
    }

    payload_track_info_pub_.publish(msg);
}

// ========== 发布无效 payload_track_info ==========
void NdtSlamNode::publishPayloadTrackInfoInvalid(const std::string& reason) {
    std_msgs::Float32MultiArray msg;
    msg.data = {
        -1.0f,   // IDX_VALID: 无效
        -1.0f,   // IDX_TRACK_ID
        0.0f,    // IDX_STATE = NONE
        0.0f, 0.0f, 0.0f,  // centroid
        0.0f, 0.0f, 0.0f,  // velocity
        0.0f, 0.0f, 0.0f,  // bbox_min
        0.0f, 0.0f, 0.0f,  // bbox_max
        0.0f,              // point_count
        0.0f,              // score
        0.0f,              // bottom_hag
        0.0f,              // support_ratio
        0.0f               // box_source = NONE
    };

    ROS_INFO_THROTTLE(1.0,
        "[CargoTargetHardChec