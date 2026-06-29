#include "ndt_slam/ndt_slam.hpp"
#include "ndt_slam/point_cloud_processing.hpp"

#include <chrono>
#include <memory>
#include <sophus/se3.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <malloc.h>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

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

NdtSlamNode::NdtSlamNode(const ros::NodeHandle& nh)
    : nh_(nh) {
    initializeParameters();

    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 10, &NdtSlamNode::pointCloudCallback, this);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 10);
    display_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map", 10);
    ground_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_ground", 10);
    objects_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_objects", 10);
    objects_clean_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_objects_clean", 10);
    near_field_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/near_field_removed", 10);
    payload_channel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_channel_cloud", 10);
    payload_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_candidate_cloud", 10);
    safe_objects_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/safe_objects_cloud", 10);
    payload_dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_dynamic_cloud", 10);
    payload_pending_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_pending_cloud", 10);
    cargo_dynamic_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cargo_dynamic_removed_cloud", 10);
    payload_track_info_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/payload_track_info", 10);
    human_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_candidate_cloud", 10);
    human_dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_dynamic_cloud", 10);
    human_pending_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_pending_cloud", 10);
    human_trajectory_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_trajectory_capsule", 10);
    human_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_removed_history_cloud", 10);
    current_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(current_cloud_topic_, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 10);

    // v12: cargo core box marker 由 ndt_slam_node 直接发布
    nh_.param("publish_cargo_core_box_marker", publish_cargo_core_box_marker_, true);
    if (publish_cargo_core_box_marker_) {
        cargo_core_bbox_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/cargo_core_bbox_marker", 1, true);
        publishDeleteCargoCoreBoxMarker();
        ROS_INFO("[CargoCoreBoxMarker] publisher=ndt_slam_node mode=direct_core_box_only frame=base_link");
    }

    // 初始化轨迹
    path_msg_.header.frame_id = "map";

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

    // 初始化 NDT_OMP（使用配置参数）
    ndt_.reset(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_->setResolution(ndt_resolution_);
    ndt_->setStepSize(ndt_step_size_);
    ndt_->setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_->setMaximumIterations(ndt_max_iterations_);
    ROS_INFO("NDT_OMP initialized: resolution=%.2f, step_size=%.2f, max_iter=%d",
             ndt_resolution_, ndt_step_size_, ndt_max_iterations_);

    shutdown_ = false;
    running_ = true;
    process_thread_ = std::thread(&NdtSlamNode::processCloudThread, this);

    if (num_worker_threads_ > 0) {
        for (int i = 0; i < num_worker_threads_; ++i) {
            worker_threads_.emplace_back(&NdtSlamNode::processingWorker, this);
        }
    }

    timer_ = nh_.createTimer(ros::Duration(5.0), &NdtSlamNode::timerCallback, this);

    ROS_INFO("NdtSlamNode initialized with NDT_OMP");
    ROS_INFO("Services: reset, set_pose, relocalize, save_map, load_map, rebuild_map");
}

NdtSlamNode::NdtSlamNode(const std::string& config_file_path, const ros::NodeHandle& nh)
    : nh_(nh) {
    initializeParameters(config_file_path);

    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 10, &NdtSlamNode::pointCloudCallback, this);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 10);
    display_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map", 10);
    ground_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_ground", 10);
    objects_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_objects", 10);
    objects_clean_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/display_map_objects_clean", 10);
    near_field_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/near_field_removed", 10);
    payload_channel_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_channel_cloud", 10);
    payload_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_candidate_cloud", 10);
    safe_objects_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/safe_objects_cloud", 10);
    payload_dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_dynamic_cloud", 10);
    payload_pending_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/payload_pending_cloud", 10);
    cargo_dynamic_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cargo_dynamic_removed_cloud", 10);
    payload_track_info_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/payload_track_info", 10);
    human_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_candidate_cloud", 10);
    human_dynamic_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_dynamic_cloud", 10);
    human_pending_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_pending_cloud", 10);
    human_trajectory_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_trajectory_capsule", 10);
    human_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/human_removed_history_cloud", 10);
    current_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(current_cloud_topic_, 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 10);

    // 初始化轨迹
    path_msg_.header.frame_id = "map";

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

    // 初始化 NDT_OMP（使用从配置文件加载的参数，而非硬编码）
    ndt_.reset(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_->setResolution(ndt_resolution_);
    ndt_->setStepSize(ndt_step_size_);
    ndt_->setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_->setMaximumIterations(ndt_max_iterations_);
    ROS_INFO("NDT_OMP initialized: resolution=%.2f, step_size=%.2f, max_iter=%d",
             ndt_resolution_, ndt_step_size_, ndt_max_iterations_);

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

    shutdown_ = false;
    running_ = true;
    process_thread_ = std::thread(&NdtSlamNode::processCloudThread, this);

    if (num_worker_threads_ > 0) {
        for (int i = 0; i < num_worker_threads_; ++i) {
            worker_threads_.emplace_back(&NdtSlamNode::processingWorker, this);
        }
    }

    timer_ = nh_.createTimer(ros::Duration(5.0), &NdtSlamNode::timerCallback, this);

    ROS_INFO("NdtSlamNode initialized with NDT_OMP");
    ROS_INFO("Config file: %s", config_file_path.c_str());
    ROS_INFO("Services: reset, set_pose, relocalize, save_map, load_map, rebuild_map");
}

NdtSlamNode::~NdtSlamNode() {
    ROS_WARN("[Shutdown] Final flush dirty tiles...");
    if (persistent_map_enabled_ && !dirty_tiles_.empty()) {
        flushDirtyTiles();
    }
    writeRuntimeStatus();

    shutdown_ = true;
    queue_cv_.notify_all();
    tracking_cv_.notify_all();
    running_ = false;
    task_cv_.notify_all();

    if (process_thread_.joinable()) {
        process_thread_.join();
    }
    for (auto& worker : worker_threads_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    if (rebuild_thread_.joinable()) {
        rebuild_thread_.join();
    }
    if (clean_rebuild_thread_.joinable()) {
        clean_rebuild_thread_.join();
    }
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
        if (config["num_worker_threads"]) {
            num_worker_threads_ = config["num_worker_threads"].as<int>();
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
        ROS_INFO("Num worker threads: %d", num_worker_threads_);
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

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parse error: %s", e.what());
    }
}

void NdtSlamNode::initializeParameters() {
    kiss_icp_config_ = kiss_icp::pipeline::KISSConfig();
}

void NdtSlamNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    last_pointcloud_time_ = ros::Time::now();
    pointcloud_stale_ = false;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        // 只保留最新帧，丢弃旧帧，避免处理积压
        cloud_queue_ = std::queue<sensor_msgs::PointCloud2::ConstPtr>();
        cloud_queue_.push(msg);
    }
    queue_cv_.notify_one();
}

void NdtSlamNode::processCloudThread() {
    ROS_INFO("Processing thread started");

    // 统计变量
    int total_frames = 0;
    int success_frames = 0;
    ros::Time last_log_time = ros::Time::now();

    while (ros::ok() && !shutdown_) {
        sensor_msgs::PointCloud2::ConstPtr msg;

        if (tracking_lost_) {
            ROS_WARN("Tracking lost, waiting for relocalization...");
            std::unique_lock<std::mutex> lock(cloud_mutex_);
            tracking_cv_.wait(lock, [this]() { return !tracking_lost_ || shutdown_; });
            if (shutdown_) break;
            ROS_INFO("Relocalization completed, resuming processing");
        }

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() { return !cloud_queue_.empty() || shutdown_; });
            if (shutdown_) break;
            msg = cloud_queue_.front();
            cloud_queue_.pop();
        }

        if (!msg) continue;

        total_frames++;
        auto start_time = std::chrono::steady_clock::now();

        // 保存消息时间戳，供 publishCurrentCloud 使用
        last_stamp_ = msg->header.stamp;

        // ========== 阶段 1：解析点云 ==========
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->empty()) {
            ROS_WARN("Empty pointCloud, skipping");
            continue;
        }

        // ========== 阶段 1.5：近场过滤（去除起重机抓臂、吊具等固定结构）==========
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& p : input_cloud->points) {
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                double range = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
                if (range > 0.5 && range < 30.0) {
                    filtered_cloud->push_back(p);
                }
            }
        }

        // 体素降采样（0.2m，比 merger 的 0.15m 略粗，实现有效降采样）
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

        if (filtered_cloud->size() < 100) {
            ROS_WARN("Too few points after filter: %lu", filtered_cloud->size());
            continue;
        }

        // ========== 阶段 3：特征提取（网格局部地面分割 + 非地面点加权）==========
        pcl::PointCloud<pcl::PointXYZ>::Ptr feature_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // 使用 XY 网格局部地面模型分割（处理倾斜地面和局部高度变化）
        separateGroundByGrid(*filtered_cloud, *ground_cloud, *feature_cloud);

        // ========== 阶段 3.5：BasePayloadChannelFilter（base_link 下吊货候选筛选）==========
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

        // 构建配准用点云：human_safe_objects x4 + ground x1
        // 人体候选不参与 NDT 配准，防止人体拖影污染定位
        pcl::PointCloud<pcl::PointXYZ>::Ptr registration_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < 4; i++) {
            *registration_cloud += *human_safe_objects;
        }
        *registration_cloud += *ground_cloud;

        // ========== 地面法向量诊断 ==========
        // 初始化时和每 100 帧输出一次，用于检测外参 roll/pitch 误差
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
        Sophus::SE3d new_pose = current_pose_;
        bool registration_success = false;
        static Sophus::SE3d last_local_map_pose = Sophus::SE3d();
        static int frames_since_last_update = 0;

        try {
            if (local_map_->empty() || local_map_->size() < 500) {
                // 累积阶段
                *local_map_ += *registration_cloud;
                registration_success = true;

                if (local_map_->size() >= 500 && local_map_->size() < 600) {
                    ROS_INFO("Local map ready: %lu points", local_map_->size());
                    last_local_map_pose = current_pose_;
                }
            } else {
                // 配准阶段（带时间预算）
                ndt_->setInputTarget(local_map_);
                ndt_->setInputSource(registration_cloud);

                pcl::PointCloud<pcl::PointXYZ> aligned;
                Eigen::Matrix4f initial_guess = current_pose_.matrix().cast<float>();

                auto ndt_start = std::chrono::steady_clock::now();
                ndt_->align(aligned, initial_guess);
                auto ndt_end = std::chrono::steady_clock::now();
                double ndt_time_ms = std::chrono::duration<double, std::milli>(ndt_end - ndt_start).count();

                // NDT 时间预算：超过 100ms 输出警告（起重机场景 NDT 正常需要 70-120ms）
                static const double NDT_TIME_BUDGET_MS = 300.0;
                static int ndt_warn_count = 0;
                if (ndt_time_ms > NDT_TIME_BUDGET_MS) {
                    ndt_warn_count++;
                    // 静止时不输出，运动时每 100 次输出一次
                    if (!is_stationary_ && (ndt_warn_count <= 3 || ndt_warn_count % 100 == 0)) {
                        ROS_WARN("[NDT-guard] time=%.1fms (count=%d)", ndt_time_ms, ndt_warn_count);
                    }
                }

                if (ndt_->hasConverged()) {
                    Eigen::Matrix4f result = ndt_->getFinalTransformation();
                    double fitness_score = ndt_->getFitnessScore();
                    double trans_prob = ndt_->getTransformationProbability();
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

                        // NDT 健康日志（每秒一次）
                        {
                            Eigen::Vector3d raw_pos = new_pose.translation();
                            double raw_roll, raw_pitch, raw_yaw;
                            so3ToRpy(new_pose.so3(), raw_roll, raw_pitch, raw_yaw);
                            // v14: NDTHealth 正常时降为 DEBUG，异常时才 WARN
                            if (fitness_score > 0.15) {
                                ROS_WARN_THROTTLE(2.0,
                                                  "[NDTHealth] fitness=%.3f (HIGH), converged=%d, "
                                                  "raw_pose=(%.2f, %.2f, %.2f), raw_rpy=(%.1f, %.1f, %.1f)deg",
                                                  fitness_score, ndt_->hasConverged() ? 1 : 0,
                                                  raw_pos.x(), raw_pos.y(), raw_pos.z(),
                                                  raw_roll * 180.0 / M_PI, raw_pitch * 180.0 / M_PI,
                                                  raw_yaw * 180.0 / M_PI);
                            } else {
                                ROS_DEBUG("[NDTHealth] fitness=%.3f, converged=%d, "
                                          "raw_pose=(%.2f, %.2f, %.2f), raw_rpy=(%.1f, %.1f, %.1f)deg",
                                          fitness_score, ndt_->hasConverged() ? 1 : 0,
                                          raw_pos.x(), raw_pos.y(), raw_pos.z(),
                                          raw_roll * 180.0 / M_PI, raw_pitch * 180.0 / M_PI,
                                          raw_yaw * 180.0 / M_PI);
                            }
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

                        // 长期模式下，静止时不更新 local_map，保持 NDT target 不变
                        if (longterm_mapping_enabled_ && is_stationary_) {
                            // 静止：不更新 local_map
                            ROS_DEBUG("[LocalMap] Stationary, skipping local_map update");
                        } else if (move_dist > 0.5 || move_rot > 0.08 || frames_since_last_update > 15) {
                            // 用配准点云更新局部地图
                            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
                            pcl::transformPointCloud(*registration_cloud, *transformed, result_ortho.cast<float>());
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

                            last_local_map_pose = new_pose;
                            frames_since_last_update = 0;
                        }
                    }
                } else {
                    static int no_converge_count = 0;
                    no_converge_count++;
                    if (no_converge_count <= 5 || no_converge_count % 50 == 0) {
                        ROS_WARN("NDT: not converged (count=%d), using previous pose", no_converge_count);
                    }
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("NDT_OMP exception: %s", e.what());
        }

        // ========== 阶段 6：更新位姿 ==========
        // 约束后的 pose 用于发布和存储，原始 pose 用于 NDT 内部
        Sophus::SE3d constrained_pose = new_pose;
        if (registration_success && crane_constraint_enabled_) {
            constrained_pose = applyCraneMotionConstraint(new_pose, "ndt_output");
        }

        if (registration_success) {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            current_pose_ = constrained_pose;  // 发布约束后的 pose
        }

        // ========== 阶段 7：发布结果（用完整点云建图）==========
        if (registration_success && !tracking_lost_) {
            // TF 用 ros::Time::now() 避免重复
            ros::Time publish_time = ros::Time::now();

            // v13: 更新 raw motion 状态（必须在 PoseFreeze 之前）
            updateRawMotionState(constrained_pose, publish_time, last_ndt_fitness_);

            // v8: PoseFreeze - 静止时冻结发布姿态
            Sophus::SE3d pub_pose = selectPublishedPose(constrained_pose, publish_time);
            publishOdometry(publish_time, msg->header.frame_id, pub_pose);

            // ICP 精配准移到后台线程，不阻塞主处理
            // NDT 结果直接用于地图插入，ICP 完成后更新位姿
            Sophus::SE3d refined_pose = new_pose;
            if (local_map_->size() > 500 && feature_cloud->size() > 100) {
                // 克隆数据用于后台 ICP
                pcl::PointCloud<pcl::PointXYZ>::Ptr icp_source(new pcl::PointCloud<pcl::PointXYZ>(*feature_cloud));
                pcl::PointCloud<pcl::PointXYZ>::Ptr icp_target(new pcl::PointCloud<pcl::PointXYZ>(*local_map_));
                Sophus::SE3d icp_initial = new_pose;

                // 如果上一轮 ICP 还在运行，跳过本轮
                if (!rebuild_running_.load()) {
                    if (rebuild_thread_.joinable()) rebuild_thread_.join();
                    rebuild_running_.store(true);
                    rebuild_thread_ = std::thread([this, icp_source, icp_target, icp_initial]() {
                        try {
                            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                            icp.setInputSource(icp_source);
                            icp.setInputTarget(icp_target);
                            icp.setMaximumIterations(15);  // 减少迭代次数，加速
                            icp.setTransformationEpsilon(1e-6);
                            icp.setMaxCorrespondenceDistance(0.3);

                            pcl::PointCloud<pcl::PointXYZ> icp_aligned;
                            icp.align(icp_aligned, icp_initial.matrix().cast<float>());

                            if (icp.hasConverged()) {
                                double icp_fitness = icp.getFitnessScore();
                                if (icp_fitness < 0.5) {
                                    Eigen::Matrix4f icp_result = icp.getFinalTransformation();
                                    Eigen::Matrix3d R = icp_result.block<3,3>(0,0).cast<double>();
                                    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
                                    Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();
                                    if (R_ortho.determinant() < 0) R_ortho.col(0) *= -1;
                                    Eigen::Matrix4d result_ortho = Eigen::Matrix4d::Identity();
                                    result_ortho.block<3,3>(0,0) = R_ortho;
                                    result_ortho.block<3,1>(0,3) = icp_result.block<3,1>(0,3).cast<double>();
                                    Sophus::SE3d refined = Sophus::SE3d(result_ortho);

                                    Sophus::SE3d ndt_to_icp = icp_initial.inverse() * refined;
                                    double pos_diff = ndt_to_icp.translation().norm();
                                    double rot_diff = ndt_to_icp.so3().log().norm() * 180.0 / M_PI;

                                    // 转弯时放宽阈值：直线0.08m/0.3°，转弯0.15m/1.0°
                                    double pos_thresh = 0.08;
                                    double rot_thresh = 0.3;
                                    if (rot_diff > 0.15) {
                                        // 检测到转弯，放宽阈值
                                        pos_thresh = 0.15;
                                        rot_thresh = 1.0;
                                    }

                                    if (pos_diff <= pos_thresh && rot_diff <= rot_thresh) {
                                        // ICP 结果合理，更新精炼位姿（用于地图插入，不影响实时 odom）
                                        std::lock_guard<std::mutex> lock(cloud_mutex_);
                                        refined_pose_ = refined;
                                        has_refined_pose_ = true;

                                        // 高质量标志：满足更严格条件时才用于 clean map 入图
                                        refined_pose_high_quality_ = (icp_fitness < 0.015 &&
                                                                      pos_diff < 0.03 &&
                                                                      rot_diff < 0.15);

                                        // 每 50 帧输出一次，减少日志
                                        static int icp_refined_count = 0;
                                        icp_refined_count++;
                                        if (icp_refined_count % 50 == 1) {
                                            ROS_DEBUG("[ICP-async] refined(%d): fitness=%.4f, pos_diff=%.4fm, rot_diff=%.2f°, high_quality=%d",
                                                     icp_refined_count, icp_fitness, pos_diff, rot_diff, refined_pose_high_quality_.load() ? 1 : 0);
                                        }
                                    } else {
                                        // 拒绝时改为 DEBUG，不输出到终端
                                        ROS_DEBUG("[ICP-async] rejected: pos_diff=%.4fm rot_diff=%.2f° too large",
                                                 pos_diff, rot_diff);
                                    }
                                }
                            }
                        } catch (const std::exception& e) {
                            ROS_DEBUG("[ICP-async] exception: %s", e.what());
                        }
                        rebuild_running_.store(false);
                    });
                }

            }

            // 建图使用约束后的 pose（和 current_pose_ 一致）
            // 这样 local_map、keyframe、publish 都使用同一个约束 pose
            Sophus::SE3d map_pose = constrained_pose;
            {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                if (has_refined_pose_.load()) {
                    map_pose = applyCraneMotionConstraint(refined_pose_, "refined");
                }
            }
            addFrameToMap(filtered_cloud, map_pose, publish_time);
            // v14: 传递 constrained_pose 用于 MotionGate 判断，pub_pose 用于 MapCommit
            commitKeyFrameWithDynamicFiltering(filtered_cloud, pub_pose, publish_time, constrained_pose);
            success_frames++;
        }

        // ========== 阶段 8：统计日志 + 长期建图维护 ==========
        auto end_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(end_time - start_time).count();

        // 更新统计
        total_frames_ = total_frames;
        average_process_time_ms_ = elapsed * 1000.0;

        // Status 只在运动时报告，每 30 秒一次
        if (!is_stationary_ && (ros::Time::now() - last_log_time).toSec() > 30.0) {
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
        if (!shouldCommitKeyframe(pose, stamp)) {
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

                // 发布吊货跟踪信息（用于避障节点）
                publishPayloadTrackInfo(stamp);
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

void NdtSlamNode::processingWorker() {
    ROS_INFO("Worker thread started");

    while (ros::ok() && running_) {
        MappingTask task;

        {
            std::unique_lock<std::mutex> lock(task_queue_mutex_);
            task_cv_.wait(lock, [this] {
                return !task_queue_.empty() || !running_;
            });

            if (!running_ && task_queue_.empty()) {
                break;
            }

            if (task_queue_.empty()) continue;

            task = task_queue_.front();
            task_queue_.pop();
        }

        // Process the task
        addFrameToMap(task.cloud, Sophus::SE3d(Eigen::Quaterniond(task.orientation.w(),
                                                                   task.orientation.x(),
                                                                   task.orientation.y(),
                                                                   task.orientation.z()).toRotationMatrix(),
                                               task.position), task.stamp);
    }

    ROS_INFO("Worker thread stopped");
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
    }

    // 重置 NDT_OMP（使用配置参数，而非硬编码）
    ndt_.reset(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_->setResolution(ndt_resolution_);
    ndt_->setStepSize(ndt_step_size_);
    ndt_->setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_->setMaximumIterations(ndt_max_iterations_);

    frame_count_ = 0;
    keyframe_count_ = 0;
    processed_loops_.clear();

    ROS_INFO("SLAM system reset complete");
    return true;
}

bool NdtSlamNode::setPoseService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    ROS_INFO("Set pose service called");
    return true;
}

bool NdtSlamNode::relocalizeService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    ROS_INFO("Relocalize service called");

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        current_pose_ = Sophus::SE3d();
        tracking_lost_ = false;
    }

    tracking_cv_.notify_all();

    ROS_INFO("Relocalization complete");
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

void NdtSlamNode::generateMapQualityReport(const std::string& session_dir) {
    try {
        std::filesystem::path session_path(session_dir);
        std::string report_file = session_path / "quality_report.txt";

        std::ofstream ofs(report_file);
        if (!ofs.is_open()) {
            ROS_ERROR("Failed to open quality report file");
            return;
        }

        auto& keyframe_manager = loop_closure_detector_.getKeyFrameManager();
        const auto& keyframes = keyframe_manager.getKeyFrames();

        MapQualityStats stats;
        stats.total_keyframes = keyframes.size();

        double fitness_sum = 0, inlier_sum = 0;
        int fitness_count = 0, inlier_count = 0;

        for (const auto& kf : keyframes) {
            if (kf.metrics_.accepted_for_localization ||
                kf.metrics_.accepted_for_detail_map ||
                kf.metrics_.accepted_for_clean_map) {
                stats.accepted_keyframes++;
            } else {
                stats.rejected_keyframes++;
            }

            if (kf.metrics_.fitness_score > 0) {
                fitness_sum += kf.metrics_.fitness_score;
                fitness_count++;
            }
            if (kf.metrics_.inlier_ratio > 0) {
                inlier_sum += kf.metrics_.inlier_ratio;
                inlier_count++;
            }
        }

        stats.avg_fitness = fitness_count > 0 ? fitness_sum / fitness_count : 0;
        stats.avg_inlier_ratio = inlier_count > 0 ? inlier_sum / inlier_count : 0;

        // 计算轨迹长度
        double trajectory_length = 0;
        Sophus::SE3d prev_pose;
        bool has_prev = false;
        for (const auto& kf : keyframes) {
            if (has_prev) {
                Sophus::SE3d delta = prev_pose.inverse() * kf.pose_;
                trajectory_length += delta.translation().norm();
            }
            prev_pose = kf.pose_;
            has_prev = true;
        }
        stats.trajectory_length = trajectory_length;

        // 地图点数统计
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            stats.localization_points = global_map_ ? global_map_->size() : 0;
            stats.detail_points = display_map_ ? display_map_->size() : 0;
            stats.ground_points = ground_map_ ? ground_map_->size() : 0;
            stats.objects_raw_points = objects_map_ ? objects_map_->size() : 0;
            stats.objects_clean_points = objects_clean_map_ ? objects_clean_map_->size() : 0;
        }

        // 生成报告
        ofs << "========== Map Quality Report ==========\n\n";
        ofs << "Keyframes:\n";
        ofs << "  Total: " << stats.total_keyframes << "\n";
        ofs << "  Accepted: " << stats.accepted_keyframes << "\n";
        ofs << "  Rejected: " << stats.rejected_keyframes << "\n\n";

        ofs << "Registration Quality:\n";
        ofs << "  Avg Fitness Score: " << std::fixed << std::setprecision(4) << stats.avg_fitness << "\n";
        ofs << "  Avg Inlier Ratio: " << stats.avg_inlier_ratio << "\n\n";

        ofs << "Trajectory:\n";
        ofs << "  Length: " << std::fixed << std::setprecision(2) << stats.trajectory_length << " m\n\n";

        ofs << "Map Points:\n";
        ofs << "  Localization Map: " << stats.localization_points << "\n";
        ofs << "  Detail Map: " << stats.detail_points << "\n";
        ofs << "  Ground Map: " << stats.ground_points << "\n";
        ofs << "  Objects Raw: " << stats.objects_raw_points << "\n";
        ofs << "  Objects Clean: " << stats.objects_clean_points << "\n";

        ofs.close();

        ROS_INFO("Quality report saved to %s", report_file.c_str());
    } catch (const std::exception& e) {
        ROS_ERROR("Exception generating quality report: %s", e.what());
    }
}

void NdtSlamNode::offlineRefinePoses(const std::string& session_dir, const std::string& localization_map_path) {
    ROS_INFO("Starting offline pose refinement...");

    // 加载关键帧数据库
    auto& keyframe_manager = loop_closure_detector_.getKeyFrameManager();
    if (!keyframe_manager.loadKeyFrameDatabase(session_dir)) {
        ROS_ERROR("Failed to load keyframe database");
        return;
    }

    // 加载定位地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr localization_map(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(localization_map_path, *localization_map) < 0) {
        ROS_ERROR("Failed to load localization map");
        return;
    }

    // 创建 NDT 配准器用于精配准
    auto ndt_refine = pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr(
        new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_refine->setResolution(ndt_resolution_);
    ndt_refine->setStepSize(ndt_step_size_);
    ndt_refine->setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_refine->setMaximumIterations(ndt_max_iterations_);
    ndt_refine->setInputTarget(localization_map);

    const auto& keyframes = keyframe_manager.getKeyFrames();
    int refined_count = 0;
    int rejected_count = 0;

    for (const auto& kf : keyframes) {
        if (!kf.cloud_ || kf.cloud_->empty()) continue;

        // 使用上一轮位姿作为初值
        Eigen::Matrix4d pose_matrix = kf.pose_.matrix();
        Eigen::Matrix4f initial_guess = pose_matrix.cast<float>();

        // 精配准
        pcl::PointCloud<pcl::PointXYZ> aligned;
        ndt_refine->setInputSource(kf.cloud_);
        ndt_refine->align(aligned, initial_guess);

        if (ndt_refine->hasConverged()) {
            double fitness = ndt_refine->getFitnessScore();
            double trans_prob = ndt_refine->getTransformationProbability();

            // 质量检查
            if (fitness < 2.0 && trans_prob > 0.3) {
                Eigen::Matrix4f result = ndt_refine->getFinalTransformation();

                // 正交化旋转矩阵
                Eigen::Matrix3d R = result.block<3,3>(0,0).cast<double>();
                Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();
                if (R_ortho.determinant() < 0) R_ortho.col(0) *= -1;

                Eigen::Matrix4d result_ortho = Eigen::Matrix4d::Identity();
                result_ortho.block<3,3>(0,0) = R_ortho;
                result_ortho.block<3,1>(0,3) = result.block<3,1>(0,3).cast<double>();

                Sophus::SE3d refined_pose(result_ortho);
                keyframe_manager.updateKeyFramePose(kf.id_, refined_pose);
                refined_count++;

                ROS_DEBUG("Refined keyframe %d: fitness=%.4f, prob=%.4f", kf.id_, fitness, trans_prob);
            } else {
                rejected_count++;
                ROS_DEBUG("Rejected keyframe %d: fitness=%.4f, prob=%.4f", kf.id_, fitness, trans_prob);
            }
        } else {
            rejected_count++;
            ROS_DEBUG("Failed to converge for keyframe %d", kf.id_);
        }
    }

    // 保存精配准结果
    std::string refined_poses_file = session_dir + "/poses_refined.txt";
    keyframe_manager.saveOptimizedPoses(refined_poses_file);

    ROS_INFO("Offline refinement complete: %d refined, %d rejected out of %zu keyframes",
             refined_count, rejected_count, keyframes.size());
}

void NdtSlamNode::exportNavigationMap(const std::string& session_dir, double resolution) {
    ROS_INFO("Exporting navigation map with resolution %.2f m...", resolution);

    if (!ground_map_ || !objects_map_ || ground_map_->empty()) {
        ROS_WARN("Ground or objects map is empty, cannot export navigation map");
        return;
    }

    // 计算地图边界
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;

    for (const auto& p : ground_map_->points) {
        min_x = std::min(min_x, (double)p.x);
        max_x = std::max(max_x, (double)p.x);
        min_y = std::min(min_y, (double)p.y);
        max_y = std::max(max_y, (double)p.y);
    }
    for (const auto& p : objects_map_->points) {
        min_x = std::min(min_x, (double)p.x);
        max_x = std::max(max_x, (double)p.x);
        min_y = std::min(min_y, (double)p.y);
        max_y = std::max(max_y, (double)p.y);
    }

    // 添加边距
    double margin = 2.0;
    min_x -= margin;
    max_x += margin;
    min_y -= margin;
    max_y += margin;

    // 计算网格大小
    int width = static_cast<int>((max_x - min_x) / resolution) + 1;
    int height = static_cast<int>((max_y - min_y) / resolution) + 1;

    // 创建占用网格 (0=unknown, 128=free, 255=obstacle)
    std::vector<uint8_t> grid(width * height, 0);

    // 标记地面为 free space
    for (const auto& p : ground_map_->points) {
        int ix = static_cast<int>((p.x - min_x) / resolution);
        int iy = static_cast<int>((p.y - min_y) / resolution);
        if (ix >= 0 && ix < width && iy >= 0 && iy < height) {
            grid[iy * width + ix] = 128;  // free
        }
    }

    // 标记物体为 obstacle
    for (const auto& p : objects_map_->points) {
        int ix = static_cast<int>((p.x - min_x) / resolution);
        int iy = static_cast<int>((p.y - min_y) / resolution);
        if (ix >= 0 && ix < width && iy >= 0 && iy < height) {
            grid[iy * width + ix] = 255;  // obstacle
        }
    }

    // 保存为 PGM 文件
    std::string pgm_file = session_dir + "/navigation_map.pgm";
    std::ofstream ofs(pgm_file, std::ios::binary);
    ofs << "P5\n" << width << " " << height << "\n255\n";
    ofs.write(reinterpret_cast<const char*>(grid.data()), grid.size());
    ofs.close();

    // 保存 YAML 配置文件
    std::string yaml_file = session_dir + "/navigation_map.yaml";
    std::ofstream yaml_ofs(yaml_file);
    yaml_ofs << "image: navigation_map.pgm\n";
    yaml_ofs << "resolution: " << resolution << "\n";
    yaml_ofs << "origin: [" << min_x << ", " << min_y << ", 0.0]\n";
    yaml_ofs << "negate: 0\n";
    yaml_ofs << "occupied_thresh: 0.65\n";
    yaml_ofs << "free_thresh: 0.196\n";
    yaml_ofs.close();

    ROS_INFO("Navigation map exported: %s (%dx%d, resolution=%.2f)",
             pgm_file.c_str(), width, height, resolution);
}

void NdtSlamNode::performRelocalization() {
    ROS_WARN("Performing relocalization...");

    // 简单的重定位：重置位姿
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        current_pose_ = Sophus::SE3d();
        tracking_lost_ = false;
    }

    tracking_cv_.notify_all();
    ROS_INFO("Relocalization complete");
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

// v13: 更新 raw motion 状态（必须用 raw/constrained pose，不能用 published pose）
void NdtSlamNode::updateRawMotionState(
    const Sophus::SE3d& constrained_pose,
    const ros::Time& stamp,
    double ndt_fitness)
{
    last_ndt_fitness_ = ndt_fitness;

    if (!has_last_raw_pose_for_motion_) {
        last_raw_pose_for_motion_ = constrained_pose;
        last_raw_pose_stamp_ = stamp;
        has_last_raw_pose_for_motion_ = true;
        raw_frame_velocity_ = 0.0;
        return;
    }

    double dt = (stamp - last_raw_pose_stamp_).toSec();
    if (dt <= 1e-3) {
        return;
    }

    Eigen::Vector2d cur_xy(constrained_pose.translation().x(), constrained_pose.translation().y());
    Eigen::Vector2d last_xy(last_raw_pose_for_motion_.translation().x(), last_raw_pose_for_motion_.translation().y());
    double raw_delta = (cur_xy - last_xy).norm();
    raw_frame_velocity_ = raw_delta / dt;

    last_raw_pose_for_motion_ = constrained_pose;
    last_raw_pose_stamp_ = stamp;
}

// v8: PoseFreeze - 静止时冻结发布姿态
bool NdtSlamNode::shouldReleasePoseFreeze(
    double drift, int confirm, std::string& reason)
{
    // v13: 使用 raw_frame_velocity_ 和 last_ndt_fitness_，不用 frozen published pose
    if (drift < 1.50) {
        reason = "DRIFT_TOO_SMALL";
        return false;
    }
    if (confirm < 5) {
        reason = "CONFIRM_NOT_ENOUGH";
        return false;
    }
    if (raw_frame_velocity_ < 0.08) {
        reason = "VELOCITY_TOO_SMALL";
        return false;
    }
    // v13: 不再要求 fitness < 0.06，否则真实移动时永远释放不了
    reason = "OK";
    return true;
}

Sophus::SE3d NdtSlamNode::computeReleaseBlendPose(const Sophus::SE3d& current_target_pose)
{
    pose_release_frame_++;
    double t = std::min(1.0, static_cast<double>(pose_release_frame_) / static_cast<double>(pose_release_total_frames_));

    Eigen::Vector3d start_t = release_start_pose_.translation();
    Eigen::Vector3d target_t = current_target_pose.translation();
    Eigen::Vector3d smooth_t = start_t + t * (target_t - start_t);

    Eigen::Quaterniond start_q(release_start_pose_.so3().unit_quaternion());
    Eigen::Quaterniond target_q(current_target_pose.so3().unit_quaternion());
    Eigen::Quaterniond smooth_q = start_q.slerp(t, target_q);

    Sophus::SE3d smooth_pose(Sophus::SO3d(smooth_q), smooth_t);

    if (pose_release_frame_ >= pose_release_total_frames_) {
        pose_freeze_state_ = PoseFreezeState::MOVING;
        pose_release_frame_ = 0;
        ROS_INFO("[PoseFreeze] mode=END_RELEASE_BLEND action=RESUME_NORMAL");
    } else {
        ROS_INFO_THROTTLE(0.5,
            "[PoseFreeze] mode=RELEASE_BLEND frame=%d/%d",
            pose_release_frame_, pose_release_total_frames_);
    }

    return smooth_pose;
}

Sophus::SE3d NdtSlamNode::selectPublishedPose(
    const Sophus::SE3d& constrained_pose,
    const ros::Time& stamp)
{
    // v13: RELEASE_BLEND 状态 - 平滑过渡
    if (pose_freeze_state_ == PoseFreezeState::RELEASE_BLEND) {
        published_pose_ = computeReleaseBlendPose(constrained_pose);
        return published_pose_;
    }

    // MOVING 状态 - 正常发布
    if (!motion_gate_stationary_) {
        pose_freeze_state_ = PoseFreezeState::MOVING;
        published_pose_ = constrained_pose;
        return published_pose_;
    }

    // STATIONARY_FROZEN 或 SUSPECTED_MOVING 状态
    const Eigen::Vector3d cur = constrained_pose.translation();
    const Eigen::Vector3d anchor = stationary_anchor_pose_.translation();
    double drift_xy = (cur.head<2>() - anchor.head<2>()).norm();

    // v13: 检查是否应该释放（使用 raw_frame_velocity_ 和 last_ndt_fitness_）
    std::string release_reason;
    bool should_release = shouldReleasePoseFreeze(drift_xy, moving_confirm_count_, release_reason);

    // v14: PoseFreezeCheck 日志降为 DEBUG
    ROS_DEBUG("[PoseFreezeCheck] drift=%.2f confirm=%d/%d raw_vel=%.3f ndt_fitness=%.3f release=%d reason=%s state=%d",
        drift_xy, moving_confirm_count_, stationary_move_confirm_frames_,
        raw_frame_velocity_, last_ndt_fitness_,
        should_release ? 1 : 0, release_reason.c_str(),
        static_cast<int>(pose_freeze_state_));

    if (should_release) {
        // v13: 进入 SUSPECTED_MOVING，继续观察
        if (pose_freeze_state_ == PoseFreezeState::STATIONARY_FROZEN) {
            pose_freeze_state_ = PoseFreezeState::SUSPECTED_MOVING;
            suspected_moving_frames_ = 0;
            ROS_INFO("[PoseFreeze] mode=SUSPECTED_MOVING drift=%.2f raw_vel=%.3f",
                     drift_xy, raw_frame_velocity_);
        }

        suspected_moving_frames_++;

        // 连续确认后开始平滑释放
        if (suspected_moving_frames_ >= 3) {
            pose_freeze_state_ = PoseFreezeState::RELEASE_BLEND;
            release_start_pose_ = published_pose_;
            release_target_pose_ = constrained_pose;
            pose_release_frame_ = 0;

            ROS_INFO("[PoseFreeze] mode=START_RELEASE_BLEND drift=%.2f frames=%d raw_vel=%.3f",
                     drift_xy, pose_release_total_frames_, raw_frame_velocity_);

            published_pose_ = computeReleaseBlendPose(constrained_pose);
            return published_pose_;
        }
    } else {
        // 回退到 STATIONARY_FROZEN
        pose_freeze_state_ = PoseFreezeState::STATIONARY_FROZEN;
        suspected_moving_frames_ = 0;
    }

    // 继续冻结
    Sophus::SE3d frozen = constrained_pose;
    frozen.translation() = anchor;
    if (stationary_freeze_yaw_) {
        frozen.so3() = stationary_anchor_pose_.so3();
    }
    published_pose_ = frozen;

    moving_confirm_count_++;

    // v14: PoseFreeze STATIONARY_ANCHOR 日志降为 DEBUG
    ROS_DEBUG("[PoseFreeze] mode=STATIONARY_ANCHOR raw_xy=(%.3f,%.3f) pub_xy=(%.3f,%.3f) drift=%.3f action=FREEZE_TF_ODOM",
        cur.x(), cur.y(),
        published_pose_.translation().x(),
        published_pose_.translation().y(),
        drift_xy);

    return published_pose_;
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

            ROS_INFO("[MotionGate] Crane stopped | keyframes=%d | anchor=(%.2f,%.2f,%.2f) | pausing map commit",
                     keyframe_count_,
                     current_pose.translation().x(),
                     current_pose.translation().y(),
                     current_pose.translation().z());
        }
    } else if (!detected_stationary) {
        stationary_frame_count_ = 0;
    }

    // v13: 静止期间检查漂移（使用 raw pose，不用 published pose）
    if (is_stationary_ && stationary_anchor_valid_) {
        double drift_from_anchor =
            (current_pose.translation() - stationary_anchor_pose_.translation()).norm();
        double elapsed = current_time.toSec() - stationary_start_time_;

        // v14: MotionGateRaw 日志，显示 raw_drift 必须随 raw_pose 变化
        ROS_DEBUG("[MotionGateRaw] raw_delta=%.3f raw_vel=%.3f raw_drift_from_anchor=%.3f state=%d",
            translation, raw_frame_velocity_, drift_from_anchor,
            static_cast<int>(pose_freeze_state_));

        // v13: 使用 raw_frame_velocity_ 判断是否真的在移动（v14: 降为 DEBUG）
        if (raw_frame_velocity_ < 0.08 && drift_from_anchor < motion_gate_stationary_drift_ignore_radius_) {
            ROS_DEBUG("[MotionGate] stationary_freeze drift=%.3f elapsed=%.1f raw_vel=%.3f action=SKIP_COMMIT",
                drift_from_anchor, elapsed, raw_frame_velocity_);
            return false;
        }

        // 超过 ignore_radius 或 raw_vel 较高，需要连续确认（v14: 降为 DEBUG）
        moving_confirm_frames_++;
        if (moving_confirm_frames_ < motion_gate_moving_confirm_frames_) {
            ROS_DEBUG("[MotionGate] possible_move drift=%.3f raw_vel=%.3f confirm=%d/%d action=WAIT",
                drift_from_anchor, raw_frame_velocity_, moving_confirm_frames_, motion_gate_moving_confirm_frames_);
            return false;
        }

        // v13: 确认移动，恢复提交（需要更多确认帧防止振荡）
        if (moving_confirm_frames_ >= 5) {  // 增加到 5 帧确认
            is_stationary_ = false;
            stationary_anchor_valid_ = false;
            moving_confirm_frames_ = 0;

            ROS_INFO("[MotionGate] Crane moving | drift=%.3f confirmed=%d | resuming map commit",
                     drift_from_anchor, moving_confirm_frames_);
        }
    }

    // 正常移动检测
    if (moved_enough && time_elapsed_enough && !is_stationary_) {
        last_keyframe_pose_for_gate_ = current_pose;
        last_keyframe_time_for_gate_ = current_time;
        moved_frame_count_++;
        return true;
    }

    // v14: 禁止静止时 force commit
    // 之前 v13 的 "静止 30 秒 force commit" 功能已关闭
    // 原因：在 PoseFreeze STATIONARY_ANCHOR 期间，force commit 会把错位引入地图
    // 如果需要静止补帧，只能补 ground/display，不允许补 cargo history 和 dynamic object
    /*
    if (is_stationary_ && time_elapsed >= 30.0) {
        last_keyframe_pose_for_gate_ = current_pose;
        last_keyframe_time_for_gate_ = current_time;
        ROS_INFO_THROTTLE(10.0,
            "[MotionGate] force_commit stationary keyframe, elapsed=%.1f", time_elapsed);
        return true;
    }
    */

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

// ========== 吊货跟踪信息发布 ==========

void NdtSlamNode::publishPayloadTrackInfo(const ros::Time& stamp) {
    PayloadTrackInfo track;
    std_msgs::Float32MultiArray msg;

    // 定义统一索引常量，禁止手写魔法数字
    constexpr int IDX_VALID = 0;
    constexpr int IDX_TRACK_ID = 1;
    constexpr int IDX_STATE = 2;
    constexpr int IDX_CENTER_X = 3;
    constexpr int IDX_CENTER_Y = 4;
    constexpr int IDX_CENTER_Z = 5;
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

    if (payload_tracker_.getBestDynamicPayloadTrack(track)) {
        // 有有效 track
        // 计算 score（综合评分）
        float score = track.track_duration * 0.3f + track.direction_consistency * 0.7f;

        // P3: 使用 core_box 如果可用，否则使用旧 bbox
        Eigen::Vector3f bbox_min, bbox_max;
        float bottom_hag = 0.0f;
        int core_pts = 0;

        if (track.has_core_box) {
            // 使用 CargoBoxV2 的 core_box
            bbox_min = track.core_box_base.bbox_min;
            bbox_max = track.core_box_base.bbox_max;
            bottom_hag = track.core_box_base.bottom_hag;
            core_pts = track.core_box_base.suspended_points;

            // [PayloadTrackInfoCore] 日志（v14: 降为 DEBUG）
            ROS_DEBUG("[PayloadTrackInfoCore] id=%d state=%d source=CORE_BOX "
                     "core_size=(%.2f,%.2f,%.2f) bottom_hag=%.2f core_pts=%d",
                     track.track_id, track.state,
                     track.core_box_base.size.x(),
                     track.core_box_base.size.y(),
                     track.core_box_base.size.z(),
                     bottom_hag, core_pts);
        } else {
            // 回退到旧 bbox
            bbox_min = track.bbox_min_map;
            bbox_max = track.bbox_max_map;
            bottom_hag = track.bbox_min_map.z();

            ROS_WARN("[PayloadTrackInfoCore] id=%d state=%d source=OLD_BBOX (no core_box available)",
                     track.track_id, track.state);
        }

        // 计算 support_ratio（静态支持率）
        float support_ratio = 0.0f;  // 暂时设为 0，后面会从 CargoBoxEstimator 获取

        msg.data = {
            1.0f,  // IDX_VALID: 有效
            static_cast<float>(track.track_id),
            static_cast<float>(track.state),
            track.centroid_map.x(), track.centroid_map.y(), track.centroid_map.z(),
            track.velocity_map.x(), track.velocity_map.y(), track.velocity_map.z(),
            bbox_min.x(), bbox_min.y(), bbox_min.z(),
            bbox_max.x(), bbox_max.y(), bbox_max.z(),
            static_cast<float>(track.point_count),
            score,
            bottom_hag,
            support_ratio
        };

        // 发布端日志
        ROS_DEBUG("[PayloadTrackInfoPub] id=%d state=%d center=(%.2f,%.2f,%.2f) "
                  "bbox_min=(%.2f,%.2f,%.2f) bbox_max=(%.2f,%.2f,%.2f) "
                  "size=(%.2f,%.2f,%.2f) pts=%d score=%.2f hag=%.2f support=%.2f",
                  track.track_id, (int)track.state,
                  track.centroid_map.x(), track.centroid_map.y(), track.centroid_map.z(),
                  bbox_min.x(), bbox_min.y(), bbox_min.z(),
                  bbox_max.x(), bbox_max.y(), bbox_max.z(),
                  bbox_max.x() - bbox_min.x(),
                  bbox_max.y() - bbox_min.y(),
                  bbox_max.z() - bbox_min.z(),
                  track.point_count, score, bottom_hag, support_ratio);
    } else {
        // 没有有效 track
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
            0.0f               // support_ratio
        };
    }

    payload_track_info_pub_.publish(msg);
}

// ========== P1: Cargo Deny History ==========

void NdtSlamNode::addCargoDenyCells(const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
                                     double current_time) {
    double bev_res = 0.15;  // 与 CleanMap 一致
    int x_min = std::floor(bbox_min.x() / bev_res);
    int x_max = std::floor(bbox_max.x() / bev_res);
    int y_min = std::floor(bbox_min.y() / bev_res);
    int y_max = std::floor(bbox_max.y() / bev_res);

    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            auto key = std::make_pair(x, y);
            auto it = cargo_deny_history_.find(key);
            if (it != cargo_deny_history_.end()) {
                it->second.last_seen_time = current_time;
                it->second.hit_count++;
            } else {
                DenyCellEntry entry;
                entry.first_seen_time = current_time;
                entry.last_seen_time = current_time;
                entry.hit_count = 1;
                cargo_deny_history_[key] = entry;
            }
        }
    }
}

bool NdtSlamNode::isCargoDenied(double x, double y, double current_time) const {
    double bev_res = 0.15;
    int bev_x = std::floor(x / bev_res);
    int bev_y = std::floor(y / bev_res);
    auto key = std::make_pair(bev_x, bev_y);

    auto it = cargo_deny_history_.find(key);
    if (it == cargo_deny_history_.end()) {
        return false;
    }

    double age = current_time - it->second.last_seen_time;
    return age < cargo_deny_ttl_;
}

void NdtSlamNode::cleanupExpiredCargoDenyCells(double current_time) {
    std::vector<std::pair<int,int>> expired_keys;

    for (const auto& entry : cargo_deny_history_) {
        double age = current_time - entry.second.last_seen_time;
        if (age >= cargo_deny_ttl_) {
            expired_keys.push_back(entry.first);
        }
    }

    for (const auto& key : expired_keys) {
        cargo_deny_history_.erase(key);
    }
}

// ========== Crane Motion Constraint 实现 ==========

void NdtSlamNode::so3ToRpy(const Sophus::SO3d& r, double& roll, double& pitch, double& yaw) {
    Eigen::Matrix3d R = r.matrix();
    // ZYX 顺序
    pitch = std::asin(-R(2, 0));
    if (std::cos(pitch) > 1e-6) {
        roll = std::atan2(R(2, 1), R(2, 2));
        yaw = std::atan2(R(1, 0), R(0, 0));
    } else {
        roll = std::atan2(-R(0, 1), R(1, 1));
        yaw = 0.0;
    }
}

Sophus::SO3d NdtSlamNode::rpyToSO3(double roll, double pitch, double yaw) {
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return Sophus::SO3d(R);
}

Sophus::SE3d NdtSlamNode::applyCraneMotionConstraint(const Sophus::SE3d& raw_pose, const std::string& stage) {
    if (!crane_constraint_enabled_) {
        return raw_pose;
    }

    Eigen::Vector3d t = raw_pose.translation();
    Sophus::SO3d r = raw_pose.so3();

    double roll, pitch, yaw;
    so3ToRpy(r, roll, pitch, yaw);

    const double raw_z = t.z();
    const double raw_roll = roll * 180.0 / M_PI;
    const double raw_pitch = pitch * 180.0 / M_PI;
    const double raw_yaw = yaw * 180.0 / M_PI;

    // 初始化固定值
    if (!first_pose_initialized_) {
        if (fixed_z_source_ == "first_frame") {
            // 从第一帧初始化
            fixed_z_ = raw_z;
        }
        // fixed_z_source=config 时，fixed_z_ 已经在配置读取时设置
        fixed_roll_ = raw_roll;
        fixed_pitch_ = raw_pitch;
        fixed_yaw_ = raw_yaw;
        first_pose_initialized_ = true;
        ROS_INFO("[CraneConstraint] Initialized: z=%.3f (source=%s), rpy=(%.2f, %.2f, %.2f)deg",
                 fixed_z_, fixed_z_source_.c_str(), fixed_roll_, fixed_pitch_, fixed_yaw_);
    }

    // 约束 z
    if (lock_z_) {
        t.z() = fixed_z_;
    } else if (constrain_z_) {
        // 限幅模式：相对 fixed_z 限幅
        t.z() = std::max(fixed_z_ - max_abs_z_drift_,
                         std::min(fixed_z_ + max_abs_z_drift_, t.z()));
    }
    // else: z 完全使用 NDT 输出，不限制

    // 约束 roll
    if (lock_roll_) {
        roll = fixed_roll_ * M_PI / 180.0;
    } else {
        double max_roll_rad = max_roll_deg_ * M_PI / 180.0;
        roll = std::max(-max_roll_rad, std::min(max_roll_rad, roll));
    }

    // 约束 pitch
    if (lock_pitch_) {
        pitch = fixed_pitch_ * M_PI / 180.0;
    } else {
        double max_pitch_rad = max_pitch_deg_ * M_PI / 180.0;
        pitch = std::max(-max_pitch_rad, std::min(max_pitch_rad, pitch));
    }

    // 约束 yaw
    if (lock_yaw_) {
        yaw = fixed_yaw_ * M_PI / 180.0;
    } else if (constrain_yaw_) {
        // 相对 fixed_yaw 限幅
        double fixed_yaw_rad = fixed_yaw_ * M_PI / 180.0;
        double max_yaw_rad = max_yaw_deg_ * M_PI / 180.0;
        yaw = std::max(fixed_yaw_rad - max_yaw_rad,
                       std::min(fixed_yaw_rad + max_yaw_rad, yaw));
    }
    // else: yaw 完全使用 NDT 输出，不限制

    Sophus::SE3d constrained_pose(rpyToSO3(roll, pitch, yaw), t);

    // v14: CraneConstraint ndt_output 日志降为 DEBUG
    ROS_DEBUG("[CraneConstraint:%s] raw_z=%.3f -> z=%.3f, "
              "raw_rpy=(%.2f, %.2f, %.2f)deg -> rpy=(%.2f, %.2f, %.2f)deg",
              stage.c_str(),
              raw_z, t.z(),
              raw_roll, raw_pitch, raw_yaw,
              roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

    return constrained_pose;
}

// ============================================================================
// P4: 从 ground_base 构建局部地面模型
// ============================================================================

static SimpleGroundModel buildGroundModelFromGroundBase(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_base,
    float resolution = 1.0f)
{
    SimpleGroundModel model;
    model.resolution = resolution;
    model.global_z_min = 0.0f;

    if (!ground_base || ground_base->empty()) {
        return model;
    }

    // 按 cell 收集 z 值
    std::map<std::pair<int,int>, std::vector<float>> cell_zs;

    for (const auto& p : ground_base->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;

        int cx = static_cast<int>(std::floor(p.x / resolution));
        int cy = static_cast<int>(std::floor(p.y / resolution));
        cell_zs[{cx, cy}].push_back(p.z);
    }

    // 对每个 cell 取 20% 分位数作为地面高度
    for (auto& kv : cell_zs) {
        auto& v = kv.second;
        std::sort(v.begin(), v.end());
        size_t idx = std::min<size_t>(v.size() * 0.2, v.size() - 1);
        model.cell_z[kv.first] = v[idx];
    }

    return model;
}

// ============================================================================
// P0: DuplicateFrameGuard 内容指纹
// ============================================================================

NdtSlamNode::FrameSignature NdtSlamNode::computeFrameSignature(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const ros::Time& stamp,
    const Sophus::SE3d& pose)
{
    FrameSignature sig;
    sig.cloud_size = cloud ? cloud->size() : 0;
    sig.stamp = stamp.toSec();
    sig.pose_xyz = pose.translation();

    if (!cloud || cloud->empty()) {
        return sig;
    }

    const size_t n = cloud->size();

    auto toVec = [](const pcl::PointXYZ& p) {
        return Eigen::Vector3f(p.x, p.y, p.z);
    };

    sig.first_pt = toVec(cloud->points.front());
    sig.mid_pt = toVec(cloud->points[n / 2]);
    sig.last_pt = toVec(cloud->points.back());

    // 采样计算 centroid
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    int cnt = 0;
    const size_t step = std::max<size_t>(1, n / 64);

    for (size_t i = 0; i < n; i += step) {
        const auto& p = cloud->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        sum += Eigen::Vector3f(p.x, p.y, p.z);
        cnt++;
    }

    if (cnt > 0) {
        sig.centroid_sample = sum / static_cast<float>(cnt);
    }

    // 计算轻量 hash
    auto quant = [](float v) -> int64_t {
        return static_cast<int64_t>(std::round(v * 1000.0f));  // 1mm quant
    };

    uint64_t h = 1469598103934665603ULL;  // FNV-1a offset basis
    auto mix = [&](int64_t x) {
        h ^= static_cast<uint64_t>(x + 0x9e3779b97f4a7c15ULL);
        h *= 1099511628211ULL;  // FNV-1a prime
    };

    mix(static_cast<int64_t>(sig.cloud_size));
    for (const auto& v : {sig.first_pt, sig.mid_pt, sig.last_pt, sig.centroid_sample}) {
        mix(quant(v.x()));
        mix(quant(v.y()));
        mix(quant(v.z()));
    }

    sig.hash = h;
    return sig;
}

bool NdtSlamNode::isDuplicateFrameBySignature(const FrameSignature& cur) const
{
    if (last_frame_signature_.cloud_size == 0) {
        return false;
    }

    const bool same_stamp = cur.stamp <= last_processed_stamp_ + 1e-6;

    const bool same_cloud =
        cur.cloud_size == last_frame_signature_.cloud_size &&
        cur.hash == last_frame_signature_.hash;

    const bool same_pose =
        (cur.pose_xyz - last_frame_signature_.pose_xyz).norm() < 1e-4;

    // 情况 A：时间戳重复
    if (same_stamp) {
        return true;
    }

    // 情况 B：时间戳变化，但点云内容和 pose 基本相同
    if (same_cloud && same_pose) {
        return true;
    }

    return false;
}

// ============================================================================
// P0-1: 新的关键帧提交流程
// 正确顺序：ground/objects 分割 → CargoBoxV2 → 吊货删除 → HumanFilter → MapCommit
// ============================================================================

void NdtSlamNode::commitKeyFrameWithDynamicFiltering(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Sophus::SE3d& pose,
    const ros::Time& stamp,
    const Sophus::SE3d& constrained_pose_for_motion)
{
    // ------------------------------------------------------------------------
    // 0. 基础准备 + DuplicateFrameGuard（内容指纹）+ MotionGate
    // ------------------------------------------------------------------------
    if (!cloud || cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "[KeyFrameCommit] empty cloud, skip");
        return;
    }

    // P0: DuplicateFrameGuard 使用内容指纹（在 NDT 之前拦截）
    auto sig = computeFrameSignature(cloud, stamp, pose);

    if (isDuplicateFrameBySignature(sig)) {
        skipped_duplicate_frames_++;
        ROS_WARN_THROTTLE(2.0,
            "[DuplicateFrameGuard] skip duplicate frame stamp=%.3f cloud_size=%zu hash=%lu skipped=%lu",
            sig.stamp, sig.cloud_size, sig.hash, skipped_duplicate_frames_);
        return;
    }

    last_frame_signature_ = sig;
    last_processed_stamp_ = stamp.toSec();
    frame_seq_++;

    // [FrameStart] 日志（DEBUG，避免刷屏）
    ROS_DEBUG("[FrameStart] frame=%lu stamp=%.3f raw=%zu pose=(%.2f,%.2f,%.2f)",
        frame_seq_, stamp.toSec(), cloud->size(),
        pose.translation().x(), pose.translation().y(), pose.translation().z());

    // v13: MotionGate 只控制 MapCommit，不阻止整个 pipeline
    // pipeline 始终运行（更新 track、display、marker），但 MapCommit 可能被跳过
    // v14: MotionGate 使用 constrained_pose（raw/constrained pose），不用 published pose
    bool should_map_commit = true;
    if (motion_gate_enabled_) {
        should_map_commit = shouldCommitKeyframe(constrained_pose_for_motion, stamp);
    }

    // v14: PoseFreeze STATIONARY_FROZEN / RELEASE_BLEND 状态下禁止地图写入
    // 这是核心修复：防止 frozen pose 参与 CargoCommit / MapCommitInput / MapCommit / CargoHistoryAdd
    if (pose_freeze_state_ == PoseFreezeState::STATIONARY_FROZEN ||
        pose_freeze_state_ == PoseFreezeState::RELEASE_BLEND) {
        ROS_DEBUG("[PoseFreeze] state=%d, skip CargoCommit/MapCommit/CargoHistoryAdd",
                  static_cast<int>(pose_freeze_state_));
        // 只更新 display 和 marker，不写地图
        // pipeline 继续运行，但跳过所有地图写入操作
        should_map_commit = false;
        // 注意：这里不 return，因为还需要更新 display 和 marker
    }

    const Eigen::Matrix4d T_map_base = pose.matrix();

    // 保存 last_cloud
    *last_cloud_ = *cloud;

    // ------------------------------------------------------------------------
    // 1. base_link 坐标系下做 ground / objects 分割
    // ------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_base(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_base(new pcl::PointCloud<pcl::PointXYZ>);

    {
        pcl::PointCloud<pcl::PointXYZ> tmp_ground, tmp_objects;
        separateGroundByGrid(*cloud, tmp_ground, tmp_objects);
        *ground_base = tmp_ground;
        *objects_base = tmp_objects;
    }

    // [GroundSplit] 日志（DEBUG）
    ROS_DEBUG("[GroundSplit] seq=%d ground=%zu objects=%zu total=%zu",
              keyframe_count_ + 1,
              ground_base->size(),
              objects_base->size(),
              ground_base->size() + objects_base->size());

    // ------------------------------------------------------------------------
    // 2. BasePayloadChannelFilter：提取吊货候选
    // ------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_channel_safe(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr payload_candidates(new pcl::PointCloud<pcl::PointXYZ>);

    ChannelFilterResult channel_result;

    if (channel_filter_config_.enabled) {
        std::map<CellKey, float> empty_ground_model;
        channel_result = channel_filter_.filter(objects_base, empty_ground_model);

        objects_channel_safe = channel_result.safe_objects;
        payload_candidates = channel_result.payload_candidates;
    } else {
        objects_channel_safe = objects_base;
    }

    // [ChannelFilter] 日志（DEBUG）
    ROS_DEBUG("[ChannelFilter] seq=%d enabled=%d safe=%zu payload_candidates=%zu raw_objects=%zu",
              keyframe_count_ + 1,
              channel_filter_config_.enabled ? 1 : 0,
              objects_channel_safe->size(),
              payload_candidates->size(),
              objects_base->size());

    // ------------------------------------------------------------------------
    // 3. CargoBoxV2 + PayloadTracker（必须在 MapCommit 前）
    // ------------------------------------------------------------------------
    TrackResult payload_track_result;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cargo_removed_base(new pcl::PointCloud<pcl::PointXYZ>);

    if (payload_tracker_config_.enabled &&
        channel_filter_config_.enabled &&
        payload_candidates && !payload_candidates->empty())
    {
        // 3.1 先更新 PayloadTracker
        std::map<CellKey, float> empty_ground_model;
        payload_track_result = payload_tracker_.update(
            payload_candidates, T_map_base, stamp.toSec(), empty_ground_model);

        // 3.2 再对每个 track 估计 CargoBoxV2
        if (cargo_box_estimator_config_.enabled) {
            // P4: 从 ground_base 构建局部地面模型
            SimpleGroundModel ground_model = buildGroundModelFromGroundBase(ground_base, 1.0f);

            auto& tracks = payload_tracker_.getMutableTracks();

            for (auto& track : tracks) {
                if (track.state == TrackState::EXPIRED) continue;
                if (track.cloud_history.empty()) continue;

                const auto& cluster_base = track.cloud_history.back();
                if (!cluster_base || cluster_base->empty()) continue;

                const CargoBox* prev_core_box = track.has_last_core_box ? &track.last_core_box : nullptr;

                CargoBox core_box, remove_box, forbidden_box;
                bool box_valid = cargo_box_estimator_.estimateCargoBox(
                    cluster_base, ground_model,
                    core_box, remove_box, forbidden_box,
                    prev_core_box);

                if (!box_valid) {
                    // v14: [CargoBoxReject] 日志 - 拆分具体原因
                    std::string reason_str;
                    switch (core_box.reject_reason) {
                        case RejectReason::TOO_SMALL_X: reason_str = "TOO_SMALL_X"; break;
                        case RejectReason::TOO_SMALL_Y: reason_str = "TOO_SMALL_Y"; break;
                        case RejectReason::TOO_SMALL_Z: reason_str = "TOO_SMALL_Z"; break;
                        case RejectReason::TOO_LARGE_X: reason_str = "TOO_LARGE_X"; break;
                        case RejectReason::TOO_LARGE_Y: reason_str = "TOO_LARGE_Y"; break;
                        case RejectReason::TOO_LARGE_Z: reason_str = "TOO_LARGE_Z"; break;
                        case RejectReason::LOW_Z_BAND_POINTS: reason_str = "LOW_Z_BAND_POINTS"; break;
                        case RejectReason::INVALID_DENSE_BAND: reason_str = "INVALID_DENSE_BAND"; break;
                        case RejectReason::GROUND_TOUCH: reason_str = "GROUND_TOUCH"; break;
                        case RejectReason::LOW_CORE_POINTS: reason_str = "LOW_CORE_POINTS"; break;
                        case RejectReason::INIT_AREA: reason_str = "INIT_AREA"; break;
                        case RejectReason::ASPECT_RATIO: reason_str = "ASPECT_RATIO"; break;
                        case RejectReason::NO_DENSE_BAND: reason_str = "NO_DENSE_BAND"; break;
                        case RejectReason::TOO_FEW_COMPONENTS: reason_str = "TOO_FEW_COMPONENTS"; break;
                        case RejectReason::LOW_HAG: reason_str = "LOW_HAG"; break;
                        case RejectReason::SIZE_JUMP: reason_str = "SIZE_JUMP"; break;
                        default: reason_str = "UNKNOWN"; break;
                    }
                    ROS_DEBUG("[CargoBoxReject] seq=%d track=%d reason=%s action=%s",
                              keyframe_count_ + 1,
                              track.track_id,
                              reason_str.c_str(),
                              "DELETE_OR_PREDICT_ONLY");
                    continue;
                }

                // 3.3 per-track size jump 软处理
                bool size_jump = false;

                if (track.has_last_size && track.observed_frames > 2) {
                    const Eigen::Vector3f& prev_size = track.last_core_size;
                    const Eigen::Vector3f& new_size = core_box.size;

                    const float gx = new_size.x() / std::max(prev_size.x(), 0.10f);
                    const float gy = new_size.y() / std::max(prev_size.y(), 0.10f);
                    const float gz = new_size.z() / std::max(prev_size.z(), 0.10f);
                    const float max_growth = std::max({gx, gy, gz});

                    if (max_growth > cargo_box_estimator_config_.max_size_growth_ratio) {
                        size_jump = true;
                        track.size_jump_count++;

                        // [CargoBoxV2SizeGate] 日志（DEBUG）
                        ROS_DEBUG("[CargoBoxV2SizeGate] seq=%d track=%d growth=%.2f threshold=%.2f count=%d action=%s",
                                  keyframe_count_ + 1,
                                  track.track_id,
                                  max_growth,
                                  cargo_box_estimator_config_.max_size_growth_ratio,
                                  track.size_jump_count,
                                  "CENTER_ONLY_NO_REMOVE");

                        // 软拒绝：更新 center，不更新 size，不用于删除
                        if (track.has_last_core_box) {
                            track.last_core_box.center = core_box.center;
                        }

                        // P4: 严格 reinit 条件（禁止无条件 reinit）
                        bool can_reinit = false;
                        std::string reinit_reason = "conditions_not_met";

                        if (track.size_jump_count >= 3 &&
                            core_box.suspended_points >= cargo_box_estimator_config_.min_confirm_core_points &&
                            (track.state == TrackState::SUSPENDED_MOVING ||
                             track.state == TrackState::DYNAMIC_PAYLOAD)) {
                            if (track.has_last_core_box) {
                                float max_ratio = std::max({
                                    core_box.size.x() / std::max(track.last_core_box.size.x(), 0.1f),
                                    core_box.size.y() / std::max(track.last_core_box.size.y(), 0.1f),
                                    core_box.size.z() / std::max(track.last_core_box.size.z(), 0.1f)});
                                if (max_ratio <= 1.5f) {
                                    can_reinit = true;
                                    reinit_reason = "accepted";
                                } else {
                                    reinit_reason = "size_too_large";
                                    track.consecutive_box_rejects++;
                                }
                            } else {
                                reinit_reason = "no_previous_box";
                            }
                        } else if (track.state == TrackState::SUSPENDED_STATIC) {
                            reinit_reason = "suspended_static_no_reinit";
                        }

                        // [CargoBoxReinitCheck] 日志（DEBUG，减少刷屏）
                        ROS_DEBUG("[CargoBoxReinitCheck] kf=%d track=%d count=%d core_pts=%d state=%d accepted=%d reason=%s",
                                  keyframe_count_ + 1,
                                  track.track_id,
                                  track.size_jump_count,
                                  core_box.suspended_points,
                                  static_cast<int>(track.state),
                                  can_reinit ? 1 : 0,
                                  reinit_reason.c_str());

                        if (can_reinit) {
                            track.last_core_box = core_box;
                            track.last_core_size = core_box.size;
                            track.has_last_core_box = true;
                            track.has_last_size = true;
                            track.size_jump_count = 0;
                            track.consecutive_box_rejects = 0;

                            // 更新 last_good_box
                            track.last_good_core_box = core_box;
                            track.last_good_remove_box = remove_box;
                            track.has_last_good_box = true;
                            track.last_good_box_time = stamp.toSec();

                            // reinit 后允许用于删除（v8: 移到统一后处理）
                            // active_cargo_remove_boxes_base.push_back(remove_box);
                        }

                        // v6: size_too_large 时使用 last_good_box fallback
                        if (!can_reinit && reinit_reason == "size_too_large" &&
                            track.has_last_good_box) {
                            double age = stamp.toSec() - track.last_good_box_time;
                            if (age < 2.0) {  // hold_time = 2.0s
                                // 使用 last_good_remove_box 做当前帧删除（v8: 移到统一后处理）
                                track.using_last_good_box = true;

                                // v14: CargoFallbackActive 降为 DEBUG
                                ROS_DEBUG("[CargoFallbackActive] kf=%d track=%d reason=USE_LAST_GOOD_BOX age=%.2f reject_count=%d",
                                         keyframe_count_ + 1,
                                         track.track_id,
                                         age,
                                         track.consecutive_box_rejects);
                            }
                        }

                        // v6: 僵尸 track 清理
                        if (track.consecutive_box_rejects > 8) {
                            ROS_WARN("[TrackCleanup] expire zombie cargo track=%d reject_count=%d reason=SIZE_TOO_LARGE_ZOMBIE",
                                     track.track_id,
                                     track.consecutive_box_rejects);
                            track.state = TrackState::EXPIRED;
                        }
                    }
                }

                if (!size_jump) {
                    // 正常更新
                    track.last_core_box = core_box;
                    track.last_core_size = core_box.size;
                    track.has_last_core_box = true;
                    track.has_last_size = true;
                    track.size_jump_count = 0;
                    track.consecutive_box_rejects = 0;

                    // v6: 更新 last_good_box
                    track.last_good_core_box = core_box;
                    track.last_good_remove_box = remove_box;
                    track.has_last_good_box = true;
                    track.last_good_box_time = stamp.toSec();
                    track.using_last_good_box = false;

                    // [CargoBoxV2] 日志（DEBUG）
                    ROS_DEBUG("[CargoBoxV2] seq=%d track=%d valid=%d core_pts=%d bottom_hag=%.2f "
                              "size=(%.2f,%.2f,%.2f) remove_size=(%.2f,%.2f,%.2f)",
                              keyframe_count_ + 1,
                              track.track_id,
                              1,
                              core_box.suspended_points,
                              core_box.bottom_hag,
                              core_box.size.x(), core_box.size.y(), core_box.size.z(),
                              remove_box.size.x(), remove_box.size.y(), remove_box.size.z());

                    // P3: 当前帧删除条件（包含 SUSPENDED_STATIC）
                    bool should_use_for_remove =
                        track.has_last_core_box &&
                        (track.state == TrackState::DYNAMIC_PAYLOAD ||
                         track.state == TrackState::SUSPENDED_MOVING ||
                         track.state == TrackState::SUSPENDED_STATIC);

                    // [CargoActiveBox] 日志（DEBUG，减少刷屏）
                    ROS_DEBUG("[CargoActiveBox] kf=%d track=%d state=%d has_core=%d active_remove=%d",
                              keyframe_count_ + 1,
                              track.track_id,
                              static_cast<int>(track.state),
                              track.has_last_core_box ? 1 : 0,
                              should_use_for_remove ? 1 : 0);

                    // v8: 移到统一后处理
                    // if (should_use_for_remove) {
                    //     active_cargo_remove_boxes_base.push_back(remove_box);
                    // }

                    // v6: SWING_FOLLOW - 吊物摆动跟随
                    {
                        float alpha_center = is_stationary_ ? 0.18f : 0.35f;
                        float alpha_size = 0.10f;

                        if (!track.has_swing_anchor) {
                            track.swing_anchor_base = core_box.center;
                            track.has_swing_anchor = true;
                            track.display_center_base = core_box.center;
                            track.display_size = core_box.size;
                        } else {
                            // 检查摆动范围
                            float swing_radius = (core_box.center.head<2>() -
                                                  track.swing_anchor_base.head<2>()).norm();
                            float dz = std::abs(core_box.center.z() - track.swing_anchor_base.z());

                            if (is_stationary_ &&
                                (swing_radius > 0.80f || dz > 0.30f)) {
                                // 摆动过大，可能是 track 跳变，不跟随（v14: 降为 DEBUG）
                                ROS_DEBUG("[BoxFollowReject] track=%d reason=SWING_TOO_LARGE swing=%.2f dz=%.2f",
                                    track.track_id, swing_radius, dz);
                            } else {
                                // 正常跟随摆动
                                track.display_center_base =
                                    alpha_center * core_box.center +
                                    (1.0f - alpha_center) * track.display_center_base;
                                track.display_size =
                                    alpha_size * core_box.size +
                                    (1.0f - alpha_size) * track.display_size;
                            }
                        }

                        // [BoxFollow] 日志（v14: 降为 DEBUG）
                        ROS_DEBUG("[BoxFollow] mode=%s stopped=%d track=%d center_base=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f)",
                            is_stationary_ ? "SWING_FOLLOW" : "MOVING_TRACK",
                            is_stationary_ ? 1 : 0,
                            track.track_id,
                            track.display_center_base.x(),
                            track.display_center_base.y(),
                            track.display_center_base.z(),
                            track.display_size.x(),
                            track.display_size.y(),
                            track.display_size.z());
                    }

                    // 发布调试点云（每 20 帧一次）
                    static int cargo_debug_count = 0;
                    cargo_debug_count++;
                    if (cargo_debug_count % 20 == 1) {
                        auto core_pts = cargo_box_estimator_.getCorePointsCloud();
                        if (core_pts && !core_pts->empty()) {
                            sensor_msgs::PointCloud2 msg;
                            pcl::toROSMsg(*core_pts, msg);
                            msg.header.stamp = stamp;
                            msg.header.frame_id = "base_link";
                            cargo_core_points_pub_.publish(msg);
                        }

                        auto hag_cloud = cargo_box_estimator_.getHagFilteredCloud();
                        if (hag_cloud && !hag_cloud->empty()) {
                            sensor_msgs::PointCloud2 msg;
                            pcl::toROSMsg(*hag_cloud, msg);
                            msg.header.stamp = stamp;
                            msg.header.frame_id = "base_link";
                            cargo_hag_filtered_pub_.publish(msg);
                        }
                    }
                }
            }
        }

        // [PayloadTrack] 日志
        int dynamic_count = 0, suspended_moving_count = 0, suspended_static_count = 0, pending_count = 0;
        for (const auto& t : payload_tracker_.getTracks()) {
            if (t.state == TrackState::DYNAMIC_PAYLOAD) dynamic_count++;
            else if (t.state == TrackState::SUSPENDED_MOVING) suspended_moving_count++;
            else if (t.state == TrackState::SUSPENDED_STATIC) suspended_static_count++;
            else if (t.state == TrackState::PENDING_STATIC) pending_count++;
        }

        // [PayloadTrack] 日志（DEBUG）
        ROS_DEBUG("[PayloadTrack] seq=%d tracks=%d dynamic=%d suspended_moving=%d suspended_static=%d pending=%d",
                  keyframe_count_ + 1,
                  (int)payload_tracker_.getTracks().size(),
                  dynamic_count,
                  suspended_moving_count,
                  suspended_static_count,
                  pending_count);

        // 发布 payload track 调试信息
        static int track_debug_count = 0;
        track_debug_count++;
        if (track_debug_count % 5 == 1) {
            if (!payload_track_result.dynamic_payload->empty()) {
                sensor_msgs::PointCloud2 dyn_msg;
                pcl::toROSMsg(*payload_track_result.dynamic_payload, dyn_msg);
                dyn_msg.header.stamp = stamp;
                dyn_msg.header.frame_id = "base_link";
                payload_dynamic_pub_.publish(dyn_msg);
            }

            if (!payload_candidates->empty()) {
                sensor_msgs::PointCloud2 cand_msg;
                pcl::toROSMsg(*payload_candidates, cand_msg);
                cand_msg.header.stamp = stamp;
                cand_msg.header.frame_id = "base_link";
                payload_candidate_pub_.publish(cand_msg);
            }
        }

        publishPayloadTrackInfo(stamp);

        // P1: 清理过期的 SUSPENDED_STATIC track
        payload_tracker_.cleanupStaleSuspendedStaticTracks(stamp.toSec());
    }

    // ------------------------------------------------------------------------
    // v8: 统一 active remove box 生成（从 CargoBoxV2 循环抽出）
    // ------------------------------------------------------------------------
    std::vector<CargoBox> active_cargo_remove_boxes_base;
    std::vector<CargoDisplayCandidate> display_candidates;  // v12: 显示候选
    int moving_tracks = 0, static_tracks = 0;
    int current_valid_count = 0, last_good_count = 0, core_fallback_count = 0;
    int skipped_no_box = 0, skipped_no_overlap = 0, skipped_state = 0;
    int overlap_total = 0;

    for (const auto& track : payload_tracker_.getTracks()) {
        if (track.state == TrackState::SUSPENDED_MOVING ||
            track.state == TrackState::DYNAMIC_PAYLOAD) {
            moving_tracks++;
        }
        if (track.state == TrackState::SUSPENDED_STATIC) {
            static_tracks++;
        }

        auto decision = buildActiveRemoveBoxForTrack(track, objects_base, stamp.toSec());

        if (decision.active) {
            active_cargo_remove_boxes_base.push_back(decision.box);
            overlap_total += decision.overlap;

            if (decision.source == "CURRENT_VALID") current_valid_count++;
            else if (decision.source == "LAST_GOOD") last_good_count++;
            else if (decision.source == "CORE_FALLBACK") core_fallback_count++;

            // v12: 生成显示候选（只有 CURRENT_VALID 用于显示）
            if (decision.source == "CURRENT_VALID" && track.has_last_core_box) {
                CargoDisplayCandidate c;
                c.valid = true;
                c.track_id = track.track_id;
                c.state = static_cast<int>(track.state);
                c.observed_frames = track.observed_frames;
                c.core_points = track.last_core_box.suspended_points;
                c.center_base = track.last_core_box.center;
                c.size = track.last_core_box.size;
                c.overlap = decision.overlap;
                c.score = 0.5 * std::min(1.0, c.core_points / 100.0) +
                           0.3 * std::min(1.0, c.overlap / 100.0) +
                           0.2 * track.direction_consistency;
                display_candidates.push_back(c);
            }
        } else {
            if (decision.reason == "NO_BOX") skipped_no_box++;
            else if (decision.reason == "NO_OVERLAP") skipped_no_overlap++;
            else if (decision.reason == "STATE_NOT_ACTIVE") skipped_state++;
        }
    }

    // [CargoActiveSummary] 日志（INFO）
    ROS_INFO("[CargoActiveSummary] kf=%d moving=%d static=%d active_boxes=%zu overlap_total=%d "
             "current_valid=%d last_good=%d core_fallback=%d skipped_no_box=%d skipped_no_overlap=%d skipped_state=%d",
             keyframe_count_ + 1,
             moving_tracks, static_tracks,
             active_cargo_remove_boxes_base.size(),
             overlap_total,
             current_valid_count, last_good_count, core_fallback_count,
             skipped_no_box, skipped_no_overlap, skipped_state);

    // v14: [CargoSummary] 日志 - 每 2 秒输出一次
    ROS_INFO_THROTTLE(2.0,
             "[CargoSummary] tracks=%zu moving=%d static=%d active=%zu removed=%zu weak_skip=%d fallback=%d history_added=%zu marker=%s",
             payload_tracker_.getTracks().size(),
             moving_tracks, static_tracks,
             active_cargo_remove_boxes_base.size(),
             cargo_removed_base->size(),
             skipped_no_box + skipped_no_overlap + skipped_state,
             last_good_count + core_fallback_count,
             new_cargo_volumes_this_frame_.size(),
             cargo_display_box_.valid ? "VALID" : "INVALID");

    // v12: 选择最佳显示候选并发布 core box marker
    if (publish_cargo_core_box_marker_) {
        CargoDisplayCandidate best;
        for (const auto& c : display_candidates) {
            if (!c.valid) continue;
            if (c.observed_frames < cargo_display_min_observed_frames_) continue;
            if (c.core_points < cargo_display_min_core_points_) continue;
            if (!best.valid || c.score > best.score) {
                best = c;
            }
        }

        if (best.valid) {
            updateAndPublishCargoCoreDisplayBox(best, is_stationary_);
        } else {
            publishDeleteCargoCoreBoxMarker();
            cargo_display_box_.valid = false;
            // v14: CargoCoreMarkerGate 降为 DEBUG
            ROS_DEBUG("[CargoCoreMarkerGate] publish=0 reason=NO_VALID_DISPLAY_CANDIDATE candidates=%zu",
                display_candidates.size());
        }
    }

    // ------------------------------------------------------------------------
    // 4. CargoCommit：当前帧吊货点删除（必须在 MapCommit 前）
    // v14: PoseFreeze STATIONARY_FROZEN / RELEASE_BLEND 状态下跳过 CargoCommit
    // ------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_after_cargo_base(new pcl::PointCloud<pcl::PointXYZ>);

    if (!should_map_commit) {
        // v14: PoseFreeze 状态下跳过 CargoCommit，直接使用 objects_base
        objects_after_cargo_base = objects_base;
        ROS_DEBUG("[PoseFreeze] state=%d, skip CargoCommit", static_cast<int>(pose_freeze_state_));
    } else {
        removePointsInsideCargoRemoveBoxesBase(
            objects_base,
            active_cargo_remove_boxes_base,
            objects_after_cargo_base,
            cargo_removed_base);
    }

    // [CargoCommit] 日志（v14: active_boxes=0 或 removed=0 时降为 DEBUG）
    if (active_cargo_remove_boxes_base.empty() || cargo_removed_base->empty()) {
        ROS_DEBUG("[CargoCommit] seq=%d source=objects_base before=%zu active_boxes=%zu removed=%zu after=%zu",
                 keyframe_count_ + 1,
                 objects_base->size(),
                 active_cargo_remove_boxes_base.size(),
                 cargo_removed_base->size(),
                 objects_after_cargo_base->size());
    } else {
        ROS_INFO("[CargoCommit] seq=%d source=objects_base before=%zu active_boxes=%zu removed=%zu after=%zu",
                 keyframe_count_ + 1,
                 objects_base->size(),
                 active_cargo_remove_boxes_base.size(),
                 cargo_removed_base->size(),
                 objects_after_cargo_base->size());
    }

    // 发布被删除的吊货点
    if (cargo_removed_base && !cargo_removed_base->empty()) {
        sensor_msgs::PointCloud2 removed_msg;
        pcl::toROSMsg(*cargo_removed_base, removed_msg);
        removed_msg.header.stamp = stamp;
        removed_msg.header.frame_id = "base_link";
        cargo_dynamic_removed_pub_.publish(removed_msg);
    }

    // v6: 构建 swept volumes 用于历史反删
    // v14: PoseFreeze STATIONARY_FROZEN / RELEASE_BLEND 状态下跳过 CargoHistoryAdd
    // v14: CargoHistoryAdd 只在 MapCommit 成功后执行，每个 kf 最多 2 个，只允许强 core box
    new_cargo_volumes_this_frame_.clear();
    if (!should_map_commit) {
        // v14: PoseFreeze 状态下跳过 CargoHistoryAdd
        ROS_DEBUG("[PoseFreeze] state=%d, skip CargoHistoryAdd", static_cast<int>(pose_freeze_state_));
    } else {
        // v14: 限制每个 kf 最多添加 2 个 history
        int history_added_this_kf = 0;
        const int max_history_boxes_per_keyframe = 2;

        for (const auto& box_base : active_cargo_remove_boxes_base) {
            // v14: 限制每个 kf 最多添加 2 个 history
            if (history_added_this_kf >= max_history_boxes_per_keyframe) {
                ROS_DEBUG("[CargoHistoryAdd] kf=%d reached max_history_boxes_per_keyframe=%d, skip remaining",
                         keyframe_count_ + 1, max_history_boxes_per_keyframe);
                break;
            }

            // v14: 只允许强 core box 写入 history
            // 检查是否是 CURRENT_VALID 来源，且 core_pts >= 30，overlap >= 50
            bool is_strong_box = false;
            for (const auto& track : payload_tracker_.getTracks()) {
                if (track.has_last_core_box &&
                    track.last_core_box.suspended_points >= 30 &&
                    track.state != TrackState::SUSPENDED_STATIC &&
                    track.consecutive_box_rejects == 0) {
                    // 检查 overlap
                    int overlap = countPointsInsideBoxBase(objects_base, box_base);
                    if (overlap >= 50) {
                        is_strong_box = true;
                        break;
                    }
                }
            }

            if (!is_strong_box) {
                ROS_DEBUG("[CargoHistoryAdd] kf=%d skip weak box (core_pts<30 or overlap<50 or fallback)",
                         keyframe_count_ + 1);
                continue;
            }

            SweptVolumeMap vol;
            // 将 base_link 坐标系的 box 转换到 map 坐标系
            Eigen::Vector4d min_base(box_base.bbox_min.x(), box_base.bbox_min.y(), box_base.bbox_min.z(), 1.0);
            Eigen::Vector4d max_base(box_base.bbox_max.x(), box_base.bbox_max.y(), box_base.bbox_max.z(), 1.0);
            Eigen::Vector4d min_map = T_map_base * min_base;
            Eigen::Vector4d max_map = T_map_base * max_base;

            // z_down_expand <= 0.03，禁止向下吃掉下方静态货物
            vol.min_map = Eigen::Vector3f(
                std::min(min_map.x(), max_map.x()) - 0.15f,
                std::min(min_map.y(), max_map.y()) - 0.15f,
                std::min(min_map.z(), max_map.z()) - 0.03f);
            vol.max_map = Eigen::Vector3f(
                std::max(min_map.x(), max_map.x()) + 0.15f,
                std::max(min_map.y(), max_map.y()) + 0.15f,
                std::max(min_map.z(), max_map.z()) + 0.10f);
            vol.stamp = stamp.toSec();
            vol.track_id = -1;  // 当前帧不关联特定 track
            vol.from_fallback = false;

            new_cargo_volumes_this_frame_.push_back(vol);
            cargo_swept_history_.push_back(vol);
            history_added_this_kf++;

            // v14: CargoHistoryAdd 单个 volume 明细降为 DEBUG
            ROS_DEBUG("[CargoHistoryAdd] kf=%d volume=(%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f) fallback=%d history_size=%zu",
                     keyframe_count_ + 1,
                     vol.min_map.x(), vol.min_map.y(), vol.min_map.z(),
                     vol.max_map.x(), vol.max_map.y(), vol.max_map.z(),
                     vol.from_fallback ? 1 : 0,
                     cargo_swept_history_.size());
        }

        // v14: CargoHistoryAdd 总结日志
        if (history_added_this_kf > 0) {
            ROS_DEBUG("[CargoHistoryAdd] kf=%d added=%d history_total=%zu",
                     keyframe_count_ + 1, history_added_this_kf, cargo_swept_history_.size());
        }
    }

    // 清理过期的 swept volume
    cleanupExpiredSweptVolumes(stamp.toSec());

    // ------------------------------------------------------------------------
    // 5. HumanFilter（必须在 MapCommit 前）
    // ------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_after_human_base(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr human_candidates_base(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr human_dynamic_base(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr human_pending_base(new pcl::PointCloud<pcl::PointXYZ>);

    size_t rejected_as_human_count = 0;

    if (human_filter_config_.enabled) {
        human_filter_.processFrame(
            objects_after_cargo_base, T_map_base, stamp.toSec(),
            objects_after_human_base, human_candidates_base,
            human_dynamic_base, human_pending_base);

        rejected_as_human_count = objects_after_cargo_base->size() - objects_after_human_base->size();

        // DynamicEventManager：人体动态事件（带人形几何约束）
        if (dynamic_event_config_.enabled && !human_dynamic_base->empty()) {
            auto dynamic_count = human_filter_.getDynamicHumanCount();
            if (dynamic_count > 0) {
                // 计算 bbox
                float z_min = 1e9, z_max = -1e9;
                float x_min = 1e9, x_max = -1e9;
                float y_min = 1e9, y_max = -1e9;
                for (const auto& pt : human_dynamic_base->points) {
                    if (pt.z < z_min) z_min = pt.z;
                    if (pt.z > z_max) z_max = pt.z;
                    if (pt.x < x_min) x_min = pt.x;
                    if (pt.x > x_max) x_max = pt.x;
                    if (pt.y < y_min) y_min = pt.y;
                    if (pt.y > y_max) y_max = pt.y;
                }

                float length = x_max - x_min;
                float width = y_max - y_min;
                float height = z_max - z_min;
                size_t points = human_dynamic_base->size();

                // 人形几何约束检查
                bool valid_human = (length < 1.2f) && (width < 1.2f) &&
                                   (height > 0.5f) && (height < 2.2f) &&
                                   (points < 250);

                if (valid_human) {
                    Eigen::Vector4f centroid_4f;
                    pcl::compute3DCentroid(*human_dynamic_base, centroid_4f);
                    Eigen::Vector3d centroid = centroid_4f.head<3>().cast<double>();

                    std::deque<Eigen::Vector3d> history;
                    history.push_back(centroid);

                    int event_id = dynamic_event_manager_.createHumanEvent(
                        stamp.toSec(), stamp.toSec(), history, z_min, z_max);
                    ROS_DEBUG("[DynamicEvent] HumanEvent created: id=%d, points=%zu",
                             event_id, points);
                } else {
                    ROS_DEBUG("[HumanFilter] rejected human event: points=%zu "
                              "bbox=(%.2f,%.2f,%.2f) - exceeds human geometry limits",
                              points, length, width, height);
                }
            }
        }

        // [HumanFilter] 日志（DEBUG）
        ROS_DEBUG("[HumanFilter] seq=%d input=%zu safe=%zu human_dynamic=%zu human_pending=%zu rejected_as_human=%zu",
                  keyframe_count_ + 1,
                  objects_after_cargo_base->size(),
                  objects_after_human_base->size(),
                  human_dynamic_base->size(),
                  human_pending_base->size(),
                  rejected_as_human_count);

        // 发布人体过滤调试话题
        static int hf_debug_count = 0;
        hf_debug_count++;
        if (hf_debug_count % 5 == 1) {
            if (!human_candidates_base->empty()) {
                sensor_msgs::PointCloud2 cand_msg;
                pcl::toROSMsg(*human_candidates_base, cand_msg);
                cand_msg.header.stamp = stamp;
                cand_msg.header.frame_id = "base_link";
                human_candidate_pub_.publish(cand_msg);
            }

            if (!human_dynamic_base->empty()) {
                sensor_msgs::PointCloud2 dyn_msg;
                pcl::toROSMsg(*human_dynamic_base, dyn_msg);
                dyn_msg.header.stamp = stamp;
                dyn_msg.header.frame_id = "map";
                human_dynamic_pub_.publish(dyn_msg);
            }
        }
    } else {
        objects_after_human_base = objects_after_cargo_base;
    }

    // ------------------------------------------------------------------------
    // 6. 组装最终提交点云（ground + filtered objects）
    // ------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr commit_cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
    *commit_cloud_base += *ground_base;
    *commit_cloud_base += *objects_after_human_base;

    // [MapCommitInput] 日志（DEBUG，避免刷屏）
    ROS_DEBUG("[MapCommitInput] seq=%d raw=%zu ground=%zu raw_objects=%zu commit_objects=%zu commit_total=%zu cargo_removed=%zu human_removed=%zu",
             keyframe_count_ + 1,
             cloud->size(),
             ground_base->size(),
             objects_base->size(),
             objects_after_human_base->size(),
             commit_cloud_base->size(),
             cargo_removed_base->size(),
             human_dynamic_base->size());

    // ------------------------------------------------------------------------
    // 7. 最后才 addKeyFrame（MapCommit）
    // v13: 只有 should_map_commit=true 时才写入地图
    // ------------------------------------------------------------------------
    if (!should_map_commit) {
        ROS_DEBUG("[MotionGate] frame=%lu stationary, skip MapCommit but pipeline runs", frame_seq_);
        // 不写地图，但仍然更新 display 和 marker
        last_map_commit_wall_time_ = ros::Time::now().toSec();
        return;
    }

    const size_t prev_keyframe_count = loop_closure_detector_.getKeyFrames().size();
    loop_closure_detector_.addKeyFrame(pose, commit_cloud_base, stamp);
    const size_t new_keyframe_count = loop_closure_detector_.getKeyFrames().size();

    if (new_keyframe_count <= prev_keyframe_count) {
        return;
    }

    keyframe_count_++;
    last_map_commit_wall_time_ = ros::Time::now().toSec();

    // v14: [MapCommitSummary] 日志 - 每个成功 keyframe 输出一次
    ROS_INFO("[MapCommitSummary] kf=%d raw=%zu ground=%zu obj_raw=%zu obj_commit=%zu cargo_removed=%zu human_removed=%zu display=%zu clean=%zu",
             keyframe_count_,
             cloud->size(),
             ground_base->size(),
             objects_base->size(),
             objects_after_human_base->size(),
             cargo_removed_base->size(),
             human_dynamic_base->size(),
             display_map_->size(),
             last_clean_points_);

    // [MapCommit] 日志（保留原有格式）
    ROS_INFO("[MapCommit] seq=%d keyframe=%d commit_total=%zu commit_objects=%zu cargo_removed=%zu human_removed=%zu",
             keyframe_count_,
             keyframe_count_,
             commit_cloud_base->size(),
             objects_after_human_base->size(),
             cargo_removed_base->size(),
             human_dynamic_base->size());

    // v13: MapStall 警告（超过 10 秒没有 MapCommit）
    double map_stall_age = ros::Time::now().toSec() - last_map_commit_wall_time_;
    if (map_stall_age > 10.0) {
        ROS_WARN_THROTTLE(5.0,
            "[MapStall] no MapCommit for %.1fs, kf=%d, state=%d, raw_vel=%.3f, ndt_fitness=%.3f",
            map_stall_age, keyframe_count_,
            static_cast<int>(pose_freeze_state_),
            raw_frame_velocity_, last_ndt_fitness_);
    }

    // ------------------------------------------------------------------------
    // 8. Map / Tile / Display 更新（只允许使用过滤后的点云）
    // ------------------------------------------------------------------------
    // 保存到 keyframe
    auto& kf_deque = const_cast<std::deque<KeyFrame>&>(loop_closure_detector_.getKeyFrames());
    if (!kf_deque.empty()) {
        auto& kf = kf_deque.back();
        kf.objects_raw = objects_base;
        kf.objects_filtered = objects_after_human_base;
        kf.ground_points = ground_base;
        kf.dirty_dynamic = false;
    }

    // 变换到 map 坐标系
    pcl::PointCloud<pcl::PointXYZ> commit_transformed, objects_transformed, ground_transformed;
    pcl::transformPointCloud(*commit_cloud_base, commit_transformed, T_map_base.cast<float>());
    pcl::transformPointCloud(*objects_after_human_base, objects_transformed, T_map_base.cast<float>());
    pcl::transformPointCloud(*ground_base, ground_transformed, T_map_base.cast<float>());

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

    size_t registration_added = 0, ground_added = 0, objects_added = 0, display_added = 0;

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        size_t before_reg = global_map_->size();
        addInRange(commit_transformed, global_map_);
        registration_added = global_map_->size() - before_reg;

        size_t before_display = display_map_->size();
        addInRange(commit_transformed, display_map_);
        addInRange(ground_transformed, display_map_);
        display_added = display_map_->size() - before_display;

        size_t before_ground = ground_map_->size();
        addInRange(ground_transformed, ground_map_);
        ground_added = ground_map_->size() - before_ground;

        size_t before_objects = objects_map_->size();
        addInRange(objects_transformed, objects_map_);
        objects_added = objects_map_->size() - before_objects;
    }

    // [MapWrite] 日志
    ROS_INFO("[MapWrite] seq=%d registration_added=%zu ground_added=%zu objects_added=%zu display_added=%zu",
             keyframe_count_,
             registration_added,
             ground_added,
             objects_added,
             display_added);

    // v6: DynamicHistoryEraser - 用 swept volume 反删 objects_map/display_map
    if (!new_cargo_volumes_this_frame_.empty()) {
        size_t erased_objects = eraseDynamicPointsFromCloud(objects_map_, new_cargo_volumes_this_frame_);
        size_t erased_display = eraseDynamicPointsFromCloud(display_map_, new_cargo_volumes_this_frame_);

        ROS_INFO("[DynamicHistoryEraser] kf=%d new_volumes=%zu erased_objects=%zu erased_display=%zu objects_left=%zu display_left=%zu",
                 keyframe_count_,
                 new_cargo_volumes_this_frame_.size(),
                 erased_objects,
                 erased_display,
                 objects_map_->size(),
                 display_map_->size());
    }

    // P1: 更新 BEV 观测计数（CleanMap 依赖此数据）
    // 只用过滤后的 objects_commit_map，每个 BEV cell 只加一次
    {
        const double clean_bev_cell = 0.15;
        std::set<BevKey> seen_cells;

        for (const auto& p : objects_transformed.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;

            BevKey bk;
            bk.x = static_cast<int>(std::floor(p.x / clean_bev_cell));
            bk.y = static_cast<int>(std::floor(p.y / clean_bev_cell));
            seen_cells.insert(bk);
        }

        for (const auto& bk : seen_cells) {
            bev_observation_count_[bk]++;
        }

        ROS_DEBUG("[BevObsUpdate] seq=%d object_points=%zu unique_cells=%zu total_obs_cells=%zu",
                  keyframe_count_,
                  objects_transformed.size(),
                  seen_cells.size(),
                  bev_observation_count_.size());
    }

    // 长期建图：写入 tiles
    if (longterm_mapping_enabled_ && persistent_map_enabled_ && canCommit()) {
        Eigen::Vector3d kf_pos = pose.translation();
        int tile_x = std::floor(kf_pos.x() / tile_size_m_);
        int tile_y = std::floor(kf_pos.y() / tile_size_m_);
        std::string tile_key = "x" + std::to_string(tile_x) + "_y" + std::to_string(tile_y);

        if (dirty_tiles_.find(tile_key) == dirty_tiles_.end()) {
            dirty_tiles_[tile_key].registration.reset(new pcl::PointCloud<pcl::PointXYZ>);
            dirty_tiles_[tile_key].display.reset(new pcl::PointCloud<pcl::PointXYZ>);
            dirty_tiles_[tile_key].ground.reset(new pcl::PointCloud<pcl::PointXYZ>);
            dirty_tiles_[tile_key].objects.reset(new pcl::PointCloud<pcl::PointXYZ>);
        }

        *dirty_tiles_[tile_key].registration += commit_transformed;
        *dirty_tiles_[tile_key].display += commit_transformed;
        *dirty_tiles_[tile_key].display += ground_transformed;
        *dirty_tiles_[tile_key].ground += ground_transformed;
        *dirty_tiles_[tile_key].objects += objects_transformed;

        dirty_tile_count_ = dirty_tiles_.size();
    }

    // ------------------------------------------------------------------------
    // 9. Cargo deny history（只有 confirmed moving 且 valid box 时才写入）
    //    使用 3D deny volume，不用 2D BEV cell
    // ------------------------------------------------------------------------
    for (const auto& track : payload_tracker_.getTracks()) {
        if (track.state == TrackState::DYNAMIC_PAYLOAD ||
            track.state == TrackState::SUSPENDED_MOVING) {
            // 只有明确移动的吊货才写 deny history
            if (track.map_displacement < 0.8 || track.velocity < 0.10) {
                continue;
            }

            // 使用 track 的 bbox 作为 deny 区域
            Eigen::Vector3d bbox_min = track.bbox_min_map.cast<double>();
            Eigen::Vector3d bbox_max = track.bbox_max_map.cast<double>();

            // 转换为 CargoBox 格式
            CargoBox deny_box;
            deny_box.bbox_min = bbox_min.cast<float>();
            deny_box.bbox_max = bbox_max.cast<float>();

            addCargoDenyVolume3D(deny_box, stamp.toSec(), track.track_id);
        }
    }

    cleanupExpiredCargoDenyVolumes3D(stamp.toSec());

    // DynamicEventManager：吊货动态事件
    if (dynamic_event_config_.enabled) {
        for (const auto& track : payload_tracker_.getTracks()) {
            if (track.state == TrackState::DYNAMIC_PAYLOAD ||
                track.state == TrackState::PENDING_STATIC) {
                Box3D bbox;
                bbox.min_pt = track.bbox_min_map.cast<double>();
                bbox.max_pt = track.bbox_max_map.cast<double>();
                Eigen::Vector3d centroid_d = track.centroid_map.cast<double>();

                if (dynamic_event_manager_.shouldSuppressNewSession(centroid_d, bbox)) {
                    continue;
                }

                int event_id = dynamic_event_manager_.findOrCreatePayloadSession(
                    track.track_id, stamp.toSec(), centroid_d, bbox, track.velocity);

                if (event_id >= 0 && track.state == TrackState::DYNAMIC_PAYLOAD) {
                    dynamic_event_manager_.updatePayloadSession(
                        event_id, stamp.toSec(), centroid_d, bbox,
                        track.velocity, track.map_displacement);
                    dynamic_event_manager_.confirmPayloadSession(event_id, stamp.toSec());
                }
            }
        }
    }

    // ------------------------------------------------------------------------
    // 10. 触发 CleanMap 重建
    // ------------------------------------------------------------------------
    static int commit_count = 0;
    commit_count++;
    // 每 3 次 commit 重建一次 CleanMap（确保测试时能触发）
    if (commit_count % 3 == 0) {
        rebuildCleanMap();
    }
    // 始终更新 clean_points
    last_clean_points_ = objects_clean_map_ ? objects_clean_map_->size() : 0;

    // 闭环检测
    if (keyframe_count_ % loop_detection_interval_ == 0) {
        ROS_DEBUG("Performing loop closure detection...");
        processLoopClosure();
    }

    // v14: [SystemSummary] 日志 - 每 2 秒输出一次
    ROS_INFO_THROTTLE(2.0,
             "[SystemSummary] frame=%lu kf=%d state=%s raw_pose=(%.2f,%.2f) pub_pose=(%.2f,%.2f) raw_vel=%.2f fitness=%.3f map_points=%zu clean=%zu",
             frame_seq_,
             keyframe_count_,
             pose_freeze_state_ == PoseFreezeState::MOVING ? "MOVING" :
             pose_freeze_state_ == PoseFreezeState::STATIONARY_FROZEN ? "FROZEN" :
             pose_freeze_state_ == PoseFreezeState::SUSPECTED_MOVING ? "SUSPECTED" :
             pose_freeze_state_ == PoseFreezeState::RELEASE_BLEND ? "BLEND" : "UNKNOWN",
             pose.translation().x(),
             pose.translation().y(),
             published_pose_.translation().x(),
             published_pose_.translation().y(),
             raw_frame_velocity_,
             last_ndt_fitness_,
             display_map_->size(),
             last_clean_points_);

    // [PipelineSummary] 单行摘要（INFO，关键验收点）
    {
        int moving = 0, statik = 0, pend = 0;
        for (const auto& t : payload_tracker_.getTracks()) {
            if (t.state == TrackState::SUSPENDED_MOVING || t.state == TrackState::DYNAMIC_PAYLOAD) moving++;
            else if (t.state == TrackState::SUSPENDED_STATIC) statik++;
            else if (t.state == TrackState::PENDING_STATIC) pend++;
        }

        ROS_INFO("[PipelineSummary] "
                 "frame=%lu kf=%d stamp=%.3f "
                 "raw=%zu ground=%zu raw_obj=%zu candidates=%zu "
                 "tracks=%zu moving=%d static=%d pending=%d "
                 "active_boxes=%zu cargo_removed=%zu human_removed=%zu "
                 "commit_obj=%zu clean_points=%zu",
                 frame_seq_,
                 keyframe_count_,
                 stamp.toSec(),
                 cloud->size(),
                 ground_base->size(),
                 objects_base->size(),
                 payload_candidates->size(),
                 payload_tracker_.getTracks().size(),
                 moving, statik, pend,
                 active_cargo_remove_boxes_base.size(),
                 cargo_removed_base->size(),
                 human_dynamic_base->size(),
                 objects_after_human_base->size(),
                 last_clean_points_);
    }
}

// ============================================================================
// 从 objects 中删除吊货 remove_box 内的点（3D 检查）
// ============================================================================

void NdtSlamNode::removePointsInsideCargoRemoveBoxes3D(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_base,
    const std::vector<Box3D>& remove_boxes_map,
    const Eigen::Matrix4d& T_map_base,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_base,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& removed_base)
{
    output_base->clear();
    removed_base->clear();

    if (!input_base || input_base->empty()) {
        return;
    }

    if (remove_boxes_map.empty()) {
        *output_base = *input_base;
        return;
    }

    for (const auto& p_base : input_base->points) {
        // 变换到 map 坐标系
        Eigen::Vector4d pb(p_base.x, p_base.y, p_base.z, 1.0);
        Eigen::Vector4d pm = T_map_base * pb;

        bool inside = false;
        for (const auto& box : remove_boxes_map) {
            if (pm.x() >= box.min_pt.x() && pm.x() <= box.max_pt.x() &&
                pm.y() >= box.min_pt.y() && pm.y() <= box.max_pt.y() &&
                pm.z() >= box.min_pt.z() && pm.z() <= box.max_pt.z()) {
                inside = true;
                break;
            }
        }

        if (inside) {
            removed_base->push_back(p_base);
        } else {
            output_base->push_back(p_base);
        }
    }

    output_base->width = output_base->size();
    output_base->height = 1;
    output_base->is_dense = false;

    removed_base->width = removed_base->size();
    removed_base->height = 1;
    removed_base->is_dense = false;
}

// ============================================================================
// P2: 在 base_link 坐标系下删除吊货点（不用变换到 map）
// ============================================================================

void NdtSlamNode::removePointsInsideCargoRemoveBoxesBase(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_base,
    const std::vector<CargoBox>& remove_boxes_base,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output_base,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& removed_base)
{
    output_base->clear();
    removed_base->clear();

    if (!input_base || input_base->empty()) {
        return;
    }

    if (remove_boxes_base.empty()) {
        *output_base = *input_base;
        return;
    }

    for (const auto& p : input_base->points) {
        bool inside = false;

        for (const auto& box : remove_boxes_base) {
            if (p.x >= box.bbox_min.x() && p.x <= box.bbox_max.x() &&
                p.y >= box.bbox_min.y() && p.y <= box.bbox_max.y() &&
                p.z >= box.bbox_min.z() && p.z <= box.bbox_max.z()) {
                inside = true;
                break;
            }
        }

        if (inside) {
            removed_base->push_back(p);
        } else {
            output_base->push_back(p);
        }
    }

    output_base->width = output_base->size();
    output_base->height = 1;
    output_base->is_dense = false;

    removed_base->width = removed_base->size();
    removed_base->height = 1;
    removed_base->is_dense = false;
}

// ============================================================================
// v8: 统一 active remove box 生成
// ============================================================================

int NdtSlamNode::countPointsInsideBoxBase(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const CargoBox& box)
{
    if (!cloud || cloud->empty()) return 0;

    int count = 0;
    for (const auto& p : cloud->points) {
        if (p.x >= box.bbox_min.x() && p.x <= box.bbox_max.x() &&
            p.y >= box.bbox_min.y() && p.y <= box.bbox_max.y() &&
            p.z >= box.bbox_min.z() && p.z <= box.bbox_max.z()) {
            count++;
        }
    }
    return count;
}

CargoBox NdtSlamNode::expandCoreToRemoveBox(const CargoBox& core_box)
{
    CargoBox remove_box = core_box;
    remove_box.bbox_min.x() -= 0.25f;
    remove_box.bbox_min.y() -= 0.25f;
    remove_box.bbox_min.z() -= 0.05f;
    remove_box.bbox_max.x() += 0.25f;
    remove_box.bbox_max.y() += 0.25f;
    remove_box.bbox_max.z() += 0.15f;
    return remove_box;
}

NdtSlamNode::ActiveRemoveDecision NdtSlamNode::buildActiveRemoveBoxForTrack(
    const ObjectTrack& track,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_base,
    double stamp)
{
    ActiveRemoveDecision d;

    // 检查状态
    const bool state_ok =
        track.state == TrackState::DYNAMIC_PAYLOAD ||
        track.state == TrackState::SUSPENDED_MOVING ||
        track.state == TrackState::SUSPENDED_STATIC;

    if (!state_ok) {
        d.reason = "STATE_NOT_ACTIVE";
        return d;
    }

    // v14: 统一门槛 - core_pts >= 30 才允许 active remove
    // 如果 core_pts < 30：不显示、不 active remove、不 CargoHistoryAdd，只保留为 candidate track
    if (track.has_last_core_box && track.last_core_box.suspended_points < 30) {
        d.reason = "WEAK_CORE";
        return d;
    }

    // 选择候选 box
    CargoBox candidate;
    bool has_candidate = false;

    if (track.has_last_good_box && !track.using_last_good_box) {
        // 当前帧有效测量（不是 fallback）
        candidate = track.last_good_remove_box;
        d.source = "CURRENT_VALID";
        has_candidate = true;
    } else if (track.has_last_good_box &&
               stamp - track.last_good_box_time < 2.0) {
        // last_good fallback
        candidate = track.last_good_remove_box;
        d.source = "LAST_GOOD";
        has_candidate = true;
    } else if (track.has_last_core_box) {
        // core fallback
        candidate = expandCoreToRemoveBox(track.last_core_box);
        d.source = "CORE_FALLBACK";
        has_candidate = true;
    }

    if (!has_candidate) {
        d.reason = "NO_BOX";
        return d;
    }

    // v14: 统一门槛 - overlap >= 50 才允许 active remove
    d.overlap = countPointsInsideBoxBase(objects_base, candidate);
    if (d.overlap < 50) {  // v14: 提高到 50
        d.reason = "NO_OVERLAP";
        return d;
    }

    d.active = true;
    d.box = candidate;
    d.reason = "OK";
    return d;
}

// ============================================================================
// v6: DynamicHistoryEraser 增量反删
// ============================================================================

size_t NdtSlamNode::eraseDynamicPointsFromCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<SweptVolumeMap>& volumes)
{
    if (!cloud || cloud->empty() || volumes.empty()) {
        return 0;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr kept(new pcl::PointCloud<pcl::PointXYZ>);
    kept->reserve(cloud->size());

    size_t removed = 0;

    for (const auto& p : cloud->points) {
        bool inside = false;

        for (const auto& v : volumes) {
            if (p.x >= v.min_map.x() && p.x <= v.max_map.x() &&
                p.y >= v.min_map.y() && p.y <= v.max_map.y() &&
                p.z >= v.min_map.z() && p.z <= v.max_map.z()) {
                inside = true;
                break;
            }
        }

        if (inside) {
            removed++;
        } else {
            kept->push_back(p);
        }
    }

    kept->width = kept->size();
    kept->height = 1;
    kept->is_dense = false;

    cloud.swap(kept);
    return removed;
}

void NdtSlamNode::cleanupExpiredSweptVolumes(double current_time)
{
    cargo_swept_history_.erase(
        std::remove_if(
            cargo_swept_history_.begin(),
            cargo_swept_history_.end(),
            [current_time, this](const SweptVolumeMap& v) {
                return (current_time - v.stamp) >= cargo_swept_ttl_;
            }),
        cargo_swept_history_.end());
}

// ============================================================================
// P0-3: 3D Dynamic Deny Volume（替代 2D BEV deny）
// ============================================================================

void NdtSlamNode::addCargoDenyVolume3D(const CargoBox& remove_box, double current_time, int track_id)
{
    // 计算 BEV cell 范围
    int x_min = std::floor(remove_box.bbox_min.x() / dynamic_deny_resolution_);
    int x_max = std::floor(remove_box.bbox_max.x() / dynamic_deny_resolution_);
    int y_min = std::floor(remove_box.bbox_min.y() / dynamic_deny_resolution_);
    int y_max = std::floor(remove_box.bbox_max.y() / dynamic_deny_resolution_);

    float z_min = remove_box.bbox_min.z() - 0.05;  // z_margin_down
    float z_max = remove_box.bbox_max.z() + 0.15;  // z_margin_up

    for (int ix = x_min; ix <= x_max; ix++) {
        for (int iy = y_min; iy <= y_max; iy++) {
            auto key = std::make_pair(ix, iy);

            DynamicDenyVolume3D volume;
            volume.ix = ix;
            volume.iy = iy;
            volume.z_min = z_min;
            volume.z_max = z_max;
            volume.stamp = current_time;
            volume.source = 0;  // cargo
            volume.track_id = track_id;

            dynamic_deny_volume_map_[key].push_back(volume);
        }
    }
}

void NdtSlamNode::cleanupExpiredCargoDenyVolumes3D(double current_time)
{
    for (auto it = dynamic_deny_volume_map_.begin(); it != dynamic_deny_volume_map_.end(); ) {
        auto& volumes = it->second;
        volumes.erase(
            std::remove_if(volumes.begin(), volumes.end(),
                [current_time, this](const DynamicDenyVolume3D& v) {
                    return (current_time - v.stamp) >= dynamic_deny_ttl_;
                }),
            volumes.end());

        if (volumes.empty()) {
            it = dynamic_deny_volume_map_.erase(it);
        } else {
            ++it;
        }
    }
}

bool NdtSlamNode::isPointDeniedBy3DHistory(float x, float y, float z) const
{
    int ix = static_cast<int>(std::floor(x / dynamic_deny_resolution_));
    int iy = static_cast<int>(std::floor(y / dynamic_deny_resolution_));

    auto it = dynamic_deny_volume_map_.find(std::make_pair(ix, iy));
    if (it == dynamic_deny_volume_map_.end()) {
        return false;
    }

    for (const auto& volume : it->second) {
        if (z >= volume.z_min && z <= volume.z_max) {
            return true;
        }
    }

    return false;
}

// ============================================================================
// v12: Cargo Core Box Marker 由 ndt_slam_node 直接发布
// ============================================================================

void NdtSlamNode::appendBoxLineList(
    visualization_msgs::Marker& m,
    const Eigen::Vector3f& center,
    const Eigen::Vector3f& size)
{
    float cx = center.x(), cy = center.y(), cz = center.z();
    float hx = size.x() / 2.0f, hy = size.y() / 2.0f, hz = size.z() / 2.0f;
    hx = std::max(hx, 0.15f);
    hy = std::max(hy, 0.15f);
    hz = std::max(hz, 0.15f);

    geometry_msgs::Point p[8];
    p[0].x = cx-hx; p[0].y = cy-hy; p[0].z = cz-hz;
    p[1].x = cx+hx; p[1].y = cy-hy; p[1].z = cz-hz;
    p[2].x = cx+hx; p[2].y = cy+hy; p[2].z = cz-hz;
    p[3].x = cx-hx; p[3].y = cy+hy; p[3].z = cz-hz;
    p[4].x = cx-hx; p[4].y = cy-hy; p[4].z = cz+hz;
    p[5].x = cx+hx; p[5].y = cy-hy; p[5].z = cz+hz;
    p[6].x = cx+hx; p[6].y = cy+hy; p[6].z = cz+hz;
    p[7].x = cx-hx; p[7].y = cy+hy; p[7].z = cz+hz;

    // 12 条边
    m.points.push_back(p[0]); m.points.push_back(p[1]);
    m.points.push_back(p[1]); m.points.push_back(p[2]);
    m.points.push_back(p[2]); m.points.push_back(p[3]);
    m.points.push_back(p[3]); m.points.push_back(p[0]);
    m.points.push_back(p[4]); m.points.push_back(p[5]);
    m.points.push_back(p[5]); m.points.push_back(p[6]);
    m.points.push_back(p[6]); m.points.push_back(p[7]);
    m.points.push_back(p[7]); m.points.push_back(p[4]);
    m.points.push_back(p[0]); m.points.push_back(p[4]);
    m.points.push_back(p[1]); m.points.push_back(p[5]);
    m.points.push_back(p[2]); m.points.push_back(p[6]);
    m.points.push_back(p[3]); m.points.push_back(p[7]);
}

void NdtSlamNode::publishCargoCoreBoxMarker(const CargoDisplayBoxState& box)
{
    if (!publish_cargo_core_box_marker_) return;

    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker m;

    m.header.frame_id = "base_link";
    m.header.stamp = ros::Time(0);
    m.ns = "cargo_core_bbox_only_v12";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.orientation.w = 1.0;
    m.scale.x = 0.05;

    // 橙色线框
    m.color.r = 1.0;
    m.color.g = 0.55;
    m.color.b = 0.0;
    m.color.a = 1.0;
    m.lifetime = ros::Duration(cargo_display_marker_lifetime_);

    appendBoxLineList(m, box.center_base, box.size);

    arr.markers.push_back(m);
    cargo_core_bbox_marker_pub_.publish(arr);
}

void NdtSlamNode::publishDeleteCargoCoreBoxMarker()
{
    if (!publish_cargo_core_box_marker_) return;

    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker m;

    m.header.frame_id = "base_link";
    m.header.stamp = ros::Time(0);
    m.ns = "cargo_core_bbox_only_v12";
    m.id = 0;
    m.action = visualization_msgs::Marker::DELETE;

    arr.markers.push_back(m);
    cargo_core_bbox_marker_pub_.publish(arr);
}

void NdtSlamNode::updateAndPublishCargoCoreDisplayBox(
    const CargoDisplayCandidate& c,
    bool crane_stopped)
{
    const double now = ros::Time::now().toSec();

    // 新 track 或首次初始化
    if (!cargo_display_box_.valid ||
        cargo_display_box_.track_id != c.track_id) {

        cargo_display_box_.valid = true;
        cargo_display_box_.track_id = c.track_id;
        cargo_display_box_.center_base = c.center_base;
        cargo_display_box_.size = c.size;
        cargo_display_box_.reject_count = 0;
        cargo_display_box_.stable_candidate_count = 0;
        cargo_display_box_.last_update_time = now;

        publishCargoCoreBoxMarker(cargo_display_box_);

        ROS_INFO("[CargoCoreMarker] action=INIT track=%d center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f)",
                 c.track_id,
                 c.center_base.x(), c.center_base.y(), c.center_base.z(),
                 c.size.x(), c.size.y(), c.size.z());
        return;
    }

    // 检查跳变
    const double center_jump = (c.center_base - cargo_display_box_.center_base).norm();
    const double size_ratio = std::max({
        c.size.x() / std::max(cargo_display_box_.size.x(), 0.1f),
        c.size.y() / std::max(cargo_display_box_.size.y(), 0.1f),
        c.size.z() / std::max(cargo_display_box_.size.z(), 0.1f)});

    const bool jump_bad = center_jump > cargo_display_max_center_jump_;
    const bool size_bad = size_ratio > cargo_display_max_size_ratio_;

    if (jump_bad || size_bad) {
        cargo_display_box_.reject_count++;

        // 检查候选是否稳定
        const double candidate_dist = (c.center_base - cargo_display_box_.candidate_center_base).norm();
        if (candidate_dist < 0.30) {
            cargo_display_box_.stable_candidate_count++;
        } else {
            cargo_display_box_.candidate_center_base = c.center_base;
            cargo_display_box_.candidate_size = c.size;
            cargo_display_box_.stable_candidate_count = 1;
        }

        ROS_WARN_THROTTLE(1.0,
            "[CargoCoreMarkerReject] track=%d center_jump=%.2f size_ratio=%.2f reject=%d stable_candidate=%d",
            c.track_id, center_jump, size_ratio,
            cargo_display_box_.reject_count,
            cargo_display_box_.stable_candidate_count);

        // 连续稳定候选 -> 重初始化
        if (cargo_display_box_.stable_candidate_count >= cargo_display_reinit_after_rejects_) {
            cargo_display_box_.center_base = c.center_base;
            cargo_display_box_.size = c.size;
            cargo_display_box_.reject_count = 0;
            cargo_display_box_.stable_candidate_count = 0;
            cargo_display_box_.last_update_time = now;

            publishCargoCoreBoxMarker(cargo_display_box_);

            ROS_INFO("[CargoCoreMarker] action=REINIT_STABLE_CANDIDATE track=%d center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f)",
                     c.track_id,
                     c.center_base.x(), c.center_base.y(), c.center_base.z(),
                     c.size.x(), c.size.y(), c.size.z());
            return;
        }

        // 连续拒绝 -> 删除旧框
        if (cargo_display_box_.reject_count >= 2) {
            publishDeleteCargoCoreBoxMarker();
            cargo_display_box_.valid = false;

            ROS_WARN("[CargoCoreMarker] action=DELETE_STALE_BOX reason=REPEATED_REJECT track=%d", c.track_id);
            return;
        }

        // 短暂保持旧框
        publishCargoCoreBoxMarker(cargo_display_box_);
        return;
    }

    // 正常平滑更新
    const double alpha_center = crane_stopped ? cargo_display_center_alpha_static_ : cargo_display_center_alpha_moving_;

    cargo_display_box_.center_base = alpha_center * c.center_base + (1.0 - alpha_center) * cargo_display_box_.center_base;
    cargo_display_box_.size = cargo_display_size_alpha_ * c.size + (1.0 - cargo_display_size_alpha_) * cargo_display_box_.size;
    cargo_display_box_.reject_count = 0;
    cargo_display_box_.stable_candidate_count = 0;
    cargo_display_box_.last_update_time = now;

    publishCargoCoreBoxMarker(cargo_display_box_);

    ROS_INFO_THROTTLE(1.0,
        "[CargoCoreMarker] action=UPDATE track=%d center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f)",
        c.track_id,
        cargo_display_box_.center_base.x(),
        cargo_display_box_.center_base.y(),
        cargo_display_box_.center_base.z(),
        cargo_display_box_.size.x(),
        cargo_display_box_.size.y(),
        cargo_display_box_.size.z());
}

} // namespace ndt_slam
