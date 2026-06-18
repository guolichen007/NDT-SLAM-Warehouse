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
}

void NdtSlamNode::timerCallback(const ros::TimerEvent&) {
    static int timer_count = 0;
    timer_count++;

    const auto& keyframes = loop_closure_detector_.getKeyFrames();
    ROS_INFO("[Timer] keyframes=%zu, cloud=%zu, init=%d",
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
            if (dem["payload_post_guard_sec"]) dynamic_event_config_.payload_post_guard_sec = dem["payload_post_guard_sec"].as<double>();
            if (dem["human_pre_guard_sec"]) dynamic_event_config_.human_pre_guard_sec = dem["human_pre_guard_sec"].as<double>();
            if (dem["human_post_guard_sec"]) dynamic_event_config_.human_post_guard_sec = dem["human_post_guard_sec"].as<double>();
            if (dem["human_capsule_radius"]) dynamic_event_config_.human_capsule_radius = dem["human_capsule_radius"].as<double>();
            if (dem["human_use_track_height"]) dynamic_event_config_.human_use_track_height = dem["human_use_track_height"].as<bool>();
            if (dem["human_z_margin"]) dynamic_event_config_.human_z_margin = dem["human_z_margin"].as<double>();
            if (dem["clean_deny_enabled"]) dynamic_event_config_.clean_deny_enabled = dem["clean_deny_enabled"].as<bool>();
            if (dem["max_dynamic_ratio"]) dynamic_event_config_.max_dynamic_ratio = dem["max_dynamic_ratio"].as<double>();
        }
        dynamic_event_manager_.configure(dynamic_event_config_);

        ROS_INFO("=== DynamicEventManager Config ===");
        ROS_INFO("  enabled: %s", dynamic_event_config_.enabled ? "true" : "false");
        ROS_INFO("  payload_pre_guard: %.1fs, post_guard: %.1fs",
                 dynamic_event_config_.payload_pre_guard_sec, dynamic_event_config_.payload_post_guard_sec);
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
        ROS_INFO("===========================");

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parse error: %s", e.what());
    }
}

void NdtSlamNode::initializeParameters() {
    kiss_icp_config_ = kiss_icp::pipeline::KISSConfig();
}

void NdtSlamNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
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
            ROS_INFO("[NearFieldFilter] input=%lu removed=%lu kept=%lu ratio=%.1f%%",
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
                ROS_INFO("[PayloadChannel] channel_points=%d, candidate_clusters=%d, "
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
                ROS_INFO("[HumanFilter] input=%lu, safe=%lu, candidate=%lu, dynamic=%lu, pending=%lu",
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

            ROS_INFO("[GroundDiag] roll=%.2f° pitch=%.2f° | "
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
                static const double NDT_TIME_BUDGET_MS = 100.0;
                static int ndt_warn_count = 0;
                if (ndt_time_ms > NDT_TIME_BUDGET_MS) {
                    ndt_warn_count++;
                    if (ndt_warn_count <= 10 || ndt_warn_count % 50 == 0) {
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
                        registration_success = true;

                        // 关键帧策略：需要足够的运动才更新局部地图
                        frames_since_last_update++;
                        Sophus::SE3d delta = last_local_map_pose.inverse() * new_pose;
                        double move_dist = delta.translation().norm();
                        double move_rot = delta.so3().log().norm();

                        // 增大阈值，减少局部地图更新频率，避免边缘厚化
                        if (move_dist > 0.5 || move_rot > 0.08 || frames_since_last_update > 15) {
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
        if (registration_success) {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            current_pose_ = new_pose;
        }

        // ========== 阶段 7：发布结果（用完整点云建图）==========
        if (registration_success && !tracking_lost_) {
            // TF 用 ros::Time::now() 避免重复
            ros::Time publish_time = ros::Time::now();
            publishOdometry(publish_time, msg->header.frame_id, new_pose);

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
                                            ROS_INFO("[ICP-async] refined(%d): fitness=%.4f, pos_diff=%.4fm, rot_diff=%.2f°, high_quality=%d",
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

            // 建图使用精炼位姿（如果有 ICP 修正），否则用 NDT 位姿
            // 实时 odom/TF 始终用 NDT 位姿，保证低延迟
            Sophus::SE3d map_pose = new_pose;
            {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                if (has_refined_pose_.load()) {
                    map_pose = refined_pose_;
                }
            }
            addFrameToMap(filtered_cloud, map_pose, publish_time);
            addKeyFrameToLoopClosure(filtered_cloud, map_pose, publish_time);
            success_frames++;
        }

        // ========== 阶段 8：统计日志 ==========
        auto end_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(end_time - start_time).count();

        if ((ros::Time::now() - last_log_time).toSec() > 10.0) {
            Eigen::Vector3d pos = current_pose_.translation();
            ROS_INFO("[Status] frames=%d/%d, pose=(%.2f, %.2f, %.2f), "
                     "feature=%lu, ground=%lu, local_map=%lu, global_map=%lu, "
                     "process=%.2fs",
                     success_frames, total_frames, pos.x(), pos.y(), pos.z(),
                     feature_cloud->size(), ground_cloud->size(),
                     local_map_->size(), global_map_->size(), elapsed);
            last_log_time = ros::Time::now();
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
            for (const auto& event : dynamic_event_manager_.getEvents()) {
                if (!event.confirmed) continue;
                if (kf_time < event.start_time || kf_time > event.end_time) continue;

                // 标记受影响的 keyframe
                if (std::find(event.affected_keyframe_ids.begin(),
                              event.affected_keyframe_ids.end(),
                              kf.id_) == event.affected_keyframe_ids.end()) {
                    const_cast<DynamicEvent&>(event).affected_keyframe_ids.push_back(kf.id_);
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

    ROS_INFO("[CleanMap] rebuilding: objects_map=%zu, bev_obs_count=%zu",
             objects_map_->size(), bev_observation_count_.size());

    const double clean_bev_cell = 0.15;
    const int clean_min_points = 3;

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

    for (auto& [bk, indices] : bev_indices) {
        total_cells++;
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
                new_clean->push_back(objects_map_->points[idx]);
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

    // 更新 clean map（线程安全）
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        *objects_clean_map_ = *new_clean;
    }

    publishObjectsCleanMap();
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
    *last_cloud_ = *cloud;

    size_t prev_keyframe_count = loop_closure_detector_.getKeyFrames().size();
    loop_closure_detector_.addKeyFrame(pose, cloud, stamp);
    size_t new_keyframe_count = loop_closure_detector_.getKeyFrames().size();

    if (new_keyframe_count > prev_keyframe_count) {
        keyframe_count_++;
        ROS_DEBUG("Keyframe added: id=%d, total keyframes=%zu",
                  keyframe_count_, new_keyframe_count);

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

                // 跟踪器确认的动态点不进地图
                // 跟踪器确认的 pending 点也不进地图（等待确认）
                // 只有 safe_objects 进地图

                // 发布动态和 pending 点云（调试用）
                static int track_debug_count = 0;
                track_debug_count++;
                if (track_debug_count % 5 == 1) {
                    ROS_INFO("[PayloadTrack] tracks=%d, dynamic=%d, pending=%d",
                             track_result.active_tracks, track_result.dynamic_tracks, track_result.pending_tracks);

                    // 输出每个活跃轨迹的状态
                    for (const auto& t : payload_tracker_.getTracks()) {
                        if (t.state != TrackState::EXPIRED) {
                            ROS_INFO("[PayloadTrack]   id=%d state=%d base_std=%.2f map_disp=%.2f vel=%.2f",
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

                // 发布人体过滤调试话题（每 5 个关键帧一次）
                static int kf_hf_debug_count = 0;
                kf_hf_debug_count++;
                if (kf_hf_debug_count % 5 == 1) {
                    ROS_INFO("[HumanFilter-KF] input=%lu, safe=%lu, candidate=%lu, dynamic=%lu, pending=%lu",
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

            // 发布 debug 话题
            static int kf_ch_debug_count = 0;
            kf_ch_debug_count++;
            if (kf_ch_debug_count % 5 == 1) {
                ROS_INFO("[PayloadChannel-KF] channel=%d, candidate=%d, safe=%d, clusters=%d",
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

} // namespace ndt_slam
