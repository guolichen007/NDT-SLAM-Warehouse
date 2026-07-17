#include "ndt_slam/ndt_slam.hpp"
#include "ndt_slam/point_cloud_processing.hpp"
#include "ndt_slam/build_info.hpp"
#include "ndt_slam/cargo_observation_policy.hpp"
#include "ndt_slam/registration_input_policy.hpp"
#include "ndt_slam/rigid_transform_conversion.hpp"
#include "ndt_slam/pending_origin_binding_policy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <Eigen/Geometry>
#include <memory>
#include <sophus/se3.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
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
            if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) ||
                q.norm() <= 1.0e-12) {
                ROS_WARN("Rejected invalid TF quaternion %s <- %s",
                         target_frame.c_str(), source_frame.c_str());
                return Sophus::SE3d();
            }
            q.normalize();
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

namespace {

struct RotationStageMetrics {
    bool finite = false;
    double orthogonality_error_max =
        std::numeric_limits<double>::infinity();
    double orthogonality_error_frobenius =
        std::numeric_limits<double>::infinity();
    double determinant = std::numeric_limits<double>::quiet_NaN();
    double quaternion_norm = std::numeric_limits<double>::quiet_NaN();
};

RotationStageMetrics inspectRotationStage(
    const Eigen::Matrix3d& rotation, bool complete_input_finite) {
    RotationStageMetrics metrics;
    metrics.finite = complete_input_finite && rotation.allFinite();
    if (!rotation.allFinite()) return metrics;

    const Eigen::Matrix3d error =
        rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
    metrics.orthogonality_error_max = error.cwiseAbs().maxCoeff();
    metrics.orthogonality_error_frobenius = error.norm();
    metrics.determinant = rotation.determinant();
    const Eigen::Quaterniond quaternion(rotation);
    if (quaternion.coeffs().allFinite()) {
        metrics.quaternion_norm = quaternion.norm();
    }
    return metrics;
}

void logSO3GuardStage(const char* stage, std::uint64_t frame,
                      const Eigen::Matrix3d& rotation,
                      bool complete_input_finite, bool valid,
                      const std::string& reason) {
    const RotationStageMetrics metrics =
        inspectRotationStage(rotation, complete_input_finite);
    if (frame > 5U && valid) return;

    if (valid) {
        ROS_DEBUG(
            "[SO3Guard] stage=%s frame=%llu finite=%d orth_max=%.17g "
            "orth_fro=%.17g det=%.17g quaternion_norm=%.17g reason=%s",
            stage, static_cast<unsigned long long>(frame),
            metrics.finite ? 1 : 0, metrics.orthogonality_error_max,
            metrics.orthogonality_error_frobenius, metrics.determinant,
            metrics.quaternion_norm, reason.c_str());
    } else {
        ROS_ERROR_THROTTLE(
            1.0,
            "[SO3Guard] stage=%s frame=%llu finite=%d orth_max=%.17g "
            "orth_fro=%.17g det=%.17g quaternion_norm=%.17g reason=%s",
            stage, static_cast<unsigned long long>(frame),
            metrics.finite ? 1 : 0, metrics.orthogonality_error_max,
            metrics.orthogonality_error_frobenius, metrics.determinant,
            metrics.quaternion_norm, reason.c_str());
    }
}

void logSO3GuardPose(const char* stage, std::uint64_t frame,
                     const Sophus::SE3d& pose) {
    const Eigen::Matrix3d rotation = pose.so3().matrix();
    const bool finite = rotation.allFinite() && pose.translation().allFinite();
    logSO3GuardStage(stage, frame, rotation, finite, finite,
                     finite ? "ok" : "pose_non_finite");
}

}  // namespace

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
    cargo_candidate_components_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/cargo_avoidance/candidate_components", 2);
    cargo_selected_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/cargo_avoidance/selected_candidate_cloud", 2);
    cargo_predicted_obb_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/cargo_avoidance/predicted_obb", 2);
    cargo_self_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/cargo_avoidance/self_removed_cloud", 2);
    cargo_external_obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/cargo_avoidance/external_obstacle_cloud", 2);
    cargo_most_dangerous_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/cargo_avoidance/most_dangerous_cluster", 2);

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
    cargo_raw_safety_status_pub_ =
        nh_.advertise<lidar_slam2_msgs::CargoSafetyStatus>(
            "/cargo_avoidance/raw_safety_status", 1);
    cargo_raw_status_code_pub_ = nh_.advertise<std_msgs::Int32>(
        "/cargo_avoidance/raw_status_code", 1);
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
    sealCurrentMapLayerBundleLocked(ros::Time::now());
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

    ROS_DEBUG("NDT_OMP initialized: resolution=%.2f, step_size=%.2f, max_iter=%d, threads=%d, search=%s",
             ndt_resolution_, ndt_step_size_, ndt_max_iterations_, ndt_num_threads_,
             ndt_neighbor_search_method_.c_str());

    relocalizer_.configure(relocalization_cfg_);
    relocalizer_.start();
    shutdown_ = false;
    running_ = true;
    map_publication_shutdown_ = false;
    map_commit_shutdown_ = false;
    map_publication_thread_ =
        std::thread(&NdtSlamNode::mapPublicationThread, this);
    map_commit_thread_ = std::thread(&NdtSlamNode::mapCommitThread, this);
    process_thread_ = std::thread(&NdtSlamNode::processCloudThread, this);

    timer_ = nh_.createTimer(ros::Duration(5.0), &NdtSlamNode::timerCallback, this);

    ROS_DEBUG("NdtSlamNode initialized with NDT_OMP");
    ROS_DEBUG("Services: reset, set_pose, relocalize, save_map, load_map, rebuild_map");
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
    cargo_candidate_components_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/cargo_avoidance/candidate_components", 2);
    cargo_selected_candidate_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/cargo_avoidance/selected_candidate_cloud", 2);
    cargo_predicted_obb_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/cargo_avoidance/predicted_obb", 2);
    cargo_self_removed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/cargo_avoidance/self_removed_cloud", 2);
    cargo_external_obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/cargo_avoidance/external_obstacle_cloud", 2);
    cargo_most_dangerous_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/cargo_avoidance/most_dangerous_cluster", 2);

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
    cargo_raw_safety_status_pub_ =
        nh_.advertise<lidar_slam2_msgs::CargoSafetyStatus>(
            "/cargo_avoidance/raw_safety_status", 1);
    cargo_raw_status_code_pub_ = nh_.advertise<std_msgs::Int32>(
        "/cargo_avoidance/raw_status_code", 1);
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
    sealCurrentMapLayerBundleLocked(ros::Time::now());
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

    ROS_DEBUG("NDT_OMP initialized: resolution=%.2f, step_size=%.2f, max_iter=%d, threads=%d, search=%s",
             ndt_resolution_, ndt_step_size_, ndt_max_iterations_, ndt_num_threads_,
             ndt_neighbor_search_method_.c_str());

    // 初始化 PayloadTrackManager
    payload_tracker_.configureFromYaml(config_file_path);
    payload_tracker_config_ = payload_tracker_.getConfig();
    ROS_DEBUG("[PayloadTracker] initialized: enabled=%d, base_stability_std_thresh=%.2f",
             payload_tracker_config_.enabled ? 1 : 0, payload_tracker_config_.base_stability_std_thresh);

    // P0.5: 初始化 CargoBoxEstimator
    cargo_box_estimator_.configureFromYaml(config_file_path);
    cargo_box_estimator_config_ = cargo_box_estimator_.getConfig();
    ROS_DEBUG("[CargoBoxEstimator] initialized: enabled=%d, use_crane_axis_obb=%d",
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
    map_publication_shutdown_ = false;
    map_commit_shutdown_ = false;
    map_publication_thread_ =
        std::thread(&NdtSlamNode::mapPublicationThread, this);
    map_commit_thread_ = std::thread(&NdtSlamNode::mapCommitThread, this);
    process_thread_ = std::thread(&NdtSlamNode::processCloudThread, this);

    timer_ = nh_.createTimer(ros::Duration(5.0), &NdtSlamNode::timerCallback, this);

    ROS_DEBUG("NdtSlamNode initialized with NDT_OMP");
    ROS_DEBUG("Config file: %s", config_file_path.c_str());
    ROS_DEBUG("Services: reset, set_pose, relocalize, save_map, load_map, rebuild_map");
}

NdtSlamNode::~NdtSlamNode() {
    pointcloud_sub_.shutdown();
    hook_load_state_sub_.shutdown();
    timer_.stop();
    shutdown_ = true;
    queue_cv_.notify_all();
    tracking_cv_.notify_all();
    {
        std::lock_guard<std::mutex> lock(map_publication_mutex_);
        map_publication_shutdown_ = true;
    }
    {
        std::lock_guard<std::mutex> lock(map_commit_queue_mutex_);
        map_commit_shutdown_ = true;
    }
    map_publication_cv_.notify_all();
    map_commit_cv_.notify_all();

    if (process_thread_.joinable()) {
        process_thread_.join();
    }
    if (map_commit_thread_.joinable()) {
        map_commit_thread_.join();
    }
    if (map_publication_thread_.joinable()) {
        map_publication_thread_.join();
    }
    if (clean_map_rebuild_thread_.joinable()) {
        clean_map_rebuild_thread_.join();
    }
    relocalizer_.stop();
    if (icp_thread_.joinable()) {
        icp_thread_.join();
    }
    if (loop_closure_thread_.joinable()) {
        loop_closure_thread_.join();
    }
    if (rebuild_thread_.joinable()) {
        rebuild_thread_.join();
    }
    if (active_map_rebuild_thread_.joinable()) {
        active_map_rebuild_thread_.join();
    }
    if (tile_flush_thread_.joinable()) {
        tile_flush_thread_.join();
    }
    running_ = false;

    ROS_WARN("[Shutdown] Final flush dirty tiles...");
    bool has_failed_tile_batch = false;
    {
        std::lock_guard<std::mutex> lock(failed_tile_flush_mutex_);
        has_failed_tile_batch = !failed_tile_flush_batch_.empty();
    }
    if (persistent_map_enabled_ &&
        (!dirty_tiles_.empty() || has_failed_tile_batch)) {
        flushDirtyTiles();
    }
    if (tile_flush_thread_.joinable()) {
        tile_flush_thread_.join();
    }
    {
        std::lock_guard<std::mutex> lock(failed_tile_flush_mutex_);
        if (!failed_tile_flush_batch_.empty() || !dirty_tiles_.empty()) {
            ROS_ERROR("[Shutdown] tile data remains after final retry: "
                      "dirty=%zu failed=%zu",
                      dirty_tiles_.size(), failed_tile_flush_batch_.size());
        }
    }
    writeRuntimeStatus();
    if (diag_pending_ndt_record_valid_) {
        runtime_diag_.writeNdtFrame(diag_pending_ndt_record_);
        diag_pending_ndt_record_valid_ = false;
    }
    runtime_diag_.flushCsv();
    ROS_WARN("[Shutdown] Complete");
}

void NdtSlamNode::enqueueMapCommitJob(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Sophus::SE3d& pose,
    const ros::Time& stamp) {
    if (!cloud || cloud->empty()) {
        map_commit_dropped_.fetch_add(1U, std::memory_order_relaxed);
        return;
    }

    MapCommitJob job;
    job.lifecycle_epoch =
        map_rebuild_generation_.load(std::memory_order_acquire);
    job.stamp = stamp;
    job.pose = pose;
    job.cloud = cloud;
    const HookLoadSnapshot hook = currentHookLoadSnapshot();
    job.hook_role = hook_load_signal_role_;
    job.hook_valid = hook.valid;
    job.hook_state = hook.state;
    job.lidar_removal_authorized = shouldRemoveHookCargo();
    job.formal_footprint_valid =
        job.lidar_removal_authorized &&
        current_rigid_cargo_geometry_.valid;
    if (job.formal_footprint_valid) {
        job.formal_footprint =
            toCargoObbFootprint(current_rigid_cargo_geometry_);
    }
    job.allow_persistent_map_commit =
        allow_persistent_map_commit_ && canCommit();
    job.has_raw_ndt_pose = has_last_raw_ndt_pose_;
    if (job.has_raw_ndt_pose) {
        job.raw_ndt_pose = last_raw_ndt_pose_;
    }
    job.refined_pose = pose;
    job.runtime_pose = current_pose_;

    {
        std::lock_guard<std::mutex> lock(map_commit_queue_mutex_);
        if (map_commit_shutdown_) {
            map_commit_dropped_.fetch_add(1U, std::memory_order_relaxed);
            return;
        }
        job.sequence = map_commit_next_sequence_++;
        if (map_commit_queue_.size() >= map_commit_queue_capacity_) {
            map_commit_queue_.back() = std::move(job);
            map_commit_coalesced_.fetch_add(1U, std::memory_order_relaxed);
        } else {
            map_commit_queue_.push_back(std::move(job));
        }
        map_commit_submitted_.fetch_add(1U, std::memory_order_relaxed);
    }
    map_commit_cv_.notify_one();
}

void NdtSlamNode::mapCommitThread() {
    ROS_DEBUG("[MapCommitWorker] started capacity=%zu",
              map_commit_queue_capacity_);
    while (true) {
        MapCommitJob job;
        {
            std::unique_lock<std::mutex> lock(map_commit_queue_mutex_);
            map_commit_cv_.wait(lock, [this]() {
                return map_commit_shutdown_ || !map_commit_queue_.empty();
            });
            if (map_commit_shutdown_ && map_commit_queue_.empty()) {
                break;
            }
            job = std::move(map_commit_queue_.front());
            map_commit_queue_.pop_front();
        }

        if (job.lifecycle_epoch !=
            map_rebuild_generation_.load(std::memory_order_acquire)) {
            map_commit_stale_.fetch_add(1U, std::memory_order_relaxed);
            continue;
        }

        bool committed = false;
        {
            // Reset/load/rebuild takes the same mutex before changing the
            // epoch or any MapCommit-owned tracker/map state.
            std::lock_guard<std::mutex> lifecycle_lock(
                map_commit_lifecycle_mutex_);
            if (job.lifecycle_epoch ==
                map_rebuild_generation_.load(std::memory_order_acquire)) {
                committed = commitKeyFrameWithDynamicFiltering(job);
            }
        }
        if (!committed) {
            continue;
        }

        map_commit_completed_.fetch_add(1U, std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> lock(map_commit_completion_mutex_);
            map_commit_completion_.pending = true;
            map_commit_completion_.lifecycle_epoch = job.lifecycle_epoch;
            map_commit_completion_.pose = job.pose;
            map_commit_completion_.has_raw_ndt_pose = job.has_raw_ndt_pose;
            map_commit_completion_.raw_ndt_pose = job.raw_ndt_pose;
            map_commit_completion_.refined_pose = job.refined_pose;
            map_commit_completion_.runtime_pose = job.runtime_pose;
        }
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            map_maintenance_pending_ = true;
        }
        queue_cv_.notify_one();
    }
    ROS_DEBUG("[MapCommitWorker] stopped submitted=%llu completed=%llu "
              "coalesced=%llu stale=%llu dropped=%llu",
              static_cast<unsigned long long>(
                  map_commit_submitted_.load(std::memory_order_relaxed)),
              static_cast<unsigned long long>(
                  map_commit_completed_.load(std::memory_order_relaxed)),
              static_cast<unsigned long long>(
                  map_commit_coalesced_.load(std::memory_order_relaxed)),
              static_cast<unsigned long long>(
                  map_commit_stale_.load(std::memory_order_relaxed)),
              static_cast<unsigned long long>(
                  map_commit_dropped_.load(std::memory_order_relaxed)));
}

void NdtSlamNode::consumeMapCommitCompletion() {
    MapCommitCompletion completion;
    {
        std::lock_guard<std::mutex> lock(map_commit_completion_mutex_);
        if (!map_commit_completion_.pending) {
            return;
        }
        completion = map_commit_completion_;
        map_commit_completion_.pending = false;
    }
    if (completion.lifecycle_epoch !=
        map_rebuild_generation_.load(std::memory_order_acquire)) {
        return;
    }

    if (localization_target_enabled_) {
        shadow_target_pose_ = completion.pose;
        shadow_target_pending_ = true;
    }
    if (completion.has_raw_ndt_pose) {
        last_commit_raw_pose_ = completion.raw_ndt_pose;
    }
    last_commit_refined_pose_ = completion.refined_pose;
    last_commit_runtime_pose_ = completion.runtime_pose;
    has_commit_gate_reference_ = true;
}

void NdtSlamNode::timerCallback(const ros::TimerEvent&) {
    static int timer_count = 0;
    timer_count++;

    const std::size_t keyframe_count =
        loop_closure_detector_.getKeyFrameCount();
    std::size_t current_cloud_points = 0U;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_cloud_points = current_cloud_ ? current_cloud_->size() : 0U;
    }
    ROS_DEBUG("[Timer] keyframes=%zu, cloud=%zu, init=%d",
             keyframe_count, current_cloud_points, initialized_ ? 1 : 0);

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
            ROS_DEBUG("[DebugConfig] publish_runtime_path=%d", debug_cfg_.publish_runtime_path ? 1 : 0);
        }

        // 日志配置
        if (config["logging"]) {
            auto log = config["logging"];
            debug_cfg_.summary_interval_sec = log["summary_interval_sec"].as<double>(5.0);
            debug_cfg_.warn_throttle_sec = log["warn_throttle_sec"].as<double>(2.0);
            debug_cfg_.debug_config = log["debug_config"].as<bool>(false);
            debug_cfg_.debug_frame_start = log["debug_frame_start"].as<bool>(false);
            debug_cfg_.debug_ndt_health = log["debug_ndt_health"].as<bool>(false);
            debug_cfg_.debug_ekf = log["debug_ekf"].as<bool>(false);
            debug_cfg_.debug_motion_gate = log["debug_motion_gate"].as<bool>(false);
            debug_cfg_.debug_pose_flow = log["debug_pose_flow"].as<bool>(false);
            debug_cfg_.debug_map_commit = log["debug_map_commit"].as<bool>(false);
            debug_cfg_.debug_perf = log["debug_perf"].as<bool>(false);
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

        if (config["loop_closure"]) {
            const auto loop = config["loop_closure"];
            loop_closure_enabled_ = loop["enabled"].as<bool>(false);
            loop_detection_interval_ = std::max(
                1, loop["detection_interval"].as<int>(20));
        } else if (config["loop_detection_interval"]) {
            // Backward-compatible legacy schema. An interval alone never
            // silently enables the production loop-closure path.
            loop_detection_interval_ = std::max(
                1, config["loop_detection_interval"].as<int>());
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

        if (debug_cfg_.debug_config) {
            ROS_DEBUG("=== DynamicEventManager Config ===");
            ROS_DEBUG("  enabled: %s", dynamic_event_config_.enabled ? "true" : "false");
            ROS_DEBUG("  payload_pre_guard: %.1fs, moving_post_guard: %.1fs, unknown_post_guard: %.1fs",
                     dynamic_event_config_.payload_pre_guard_sec,
                     dynamic_event_config_.moving_post_guard_sec,
                     dynamic_event_config_.unknown_post_guard_sec);
            ROS_DEBUG("  placement: enabled=%s, stable_frames=%d, disp_thresh=%.2f, vel_thresh=%.2f",
                     dynamic_event_config_.placement_detection_enabled ? "true" : "false",
                     dynamic_event_config_.stable_frames_thresh,
                     dynamic_event_config_.stable_map_disp_thresh_m,
                     dynamic_event_config_.stable_velocity_thresh_mps);
            ROS_DEBUG("  human_pre_guard: %.1fs, post_guard: %.1fs",
                     dynamic_event_config_.human_pre_guard_sec,
                     dynamic_event_config_.human_post_guard_sec);
        }

        loop_closure_detector_.configureFromYaml(config_file_path);

        if (debug_cfg_.debug_config) {
        ROS_DEBUG("=== NdtSlamNode Parameters ===");
        ROS_DEBUG("=== HumanObjectFilter Config ===");
        ROS_DEBUG("  enabled: %s", human_filter_config_.enabled ? "true" : "false");
        ROS_DEBUG("  min_hag: %.2f m", human_filter_config_.min_hag);
        ROS_DEBUG("  max_hag: %.2f m", human_filter_config_.max_hag);
        ROS_DEBUG("  min_cluster_height: %.2f m", human_filter_config_.min_cluster_height);
        ROS_DEBUG("  max_cluster_height: %.2f m", human_filter_config_.max_cluster_height);
        ROS_DEBUG("  min_points: %d", human_filter_config_.min_points);
        ROS_DEBUG("  max_points: %d", human_filter_config_.max_points);
        ROS_DEBUG("  min_area_m2: %.2f", human_filter_config_.min_area_m2);
        ROS_DEBUG("  max_area_m2: %.2f", human_filter_config_.max_area_m2);
        ROS_DEBUG("  max_width_m: %.2f", human_filter_config_.max_width_m);
        ROS_DEBUG("  max_length_m: %.2f", human_filter_config_.max_length_m);
        ROS_DEBUG("  bev_resolution: %.2f m", human_filter_config_.bev_resolution);
        ROS_DEBUG("  merge_gap_m: %.2f m", human_filter_config_.merge_gap_m);
        ROS_DEBUG("PointCloud topic: %s", pointcloud_topic_.c_str());
        ROS_DEBUG("Odometry topic: %s", odom_topic_.c_str());
        ROS_DEBUG("Map topic: %s", map_topic_.c_str());
        ROS_DEBUG("Base frame: %s", base_frame_.c_str());
        ROS_DEBUG("Odom frame: %s", odom_frame_.c_str());
        ROS_DEBUG("Map frame: %s", map_frame_.c_str());
        ROS_DEBUG("Publish TF: %d", publish_odom_tf_);
        ROS_DEBUG("Voxel size: %.3f m (registration), %.3f m (display)", voxel_size_, display_voxel_size_);
        ROS_DEBUG("Max map size: %.1f m", max_map_size_);
        ROS_DEBUG("Map update interval: %d frames", map_update_interval_);
        ROS_DEBUG("Use voxel filter: %d", use_voxel_filter_);
        ROS_DEBUG("Loop closure: enabled=%d interval=%d keyframes",
                 loop_closure_enabled_ ? 1 : 0, loop_detection_interval_);
        ROS_DEBUG("=== Feature Extraction Config ===");
        ROS_DEBUG("  enabled: %s", use_feature_extraction_ ? "true" : "false");
        ROS_DEBUG("  voxel_size: %.3f m", feature_voxel_size_);
        ROS_DEBUG("  height_diff_threshold: %.3f m", height_diff_threshold_);
        ROS_DEBUG("  feature_weight: %d", feature_weight_);
        ROS_DEBUG("=== NDT_OMP Config ===");
        ROS_DEBUG("  resolution: %.2f m", ndt_resolution_);
        ROS_DEBUG("  step_size: %.2f", ndt_step_size_);
        ROS_DEBUG("  transformation_epsilon: %.4f", ndt_transformation_epsilon_);
        ROS_DEBUG("  max_iterations: %d", ndt_max_iterations_);
        ROS_DEBUG("=== Dynamic Filter Config ===");
        ROS_DEBUG("  enabled: %s", use_dynamic_filter_ ? "true" : "false");
        ROS_DEBUG("  mean_k: %d", sor_mean_k_);
        ROS_DEBUG("  stddev_mul_thresh: %.2f", sor_stddev_mul_thresh_);
        ROS_DEBUG("=== Ground Model Config ===");
        ROS_DEBUG("  grid_cell_size: %.1f m", grid_cell_size_);
        ROS_DEBUG("  height_above_ground: %.2f m", height_above_ground_);
        ROS_DEBUG("=== Near-Field Filter Config ===");
        ROS_DEBUG("  near_field_radius: %.1f m", near_field_radius_);
        ROS_DEBUG("  near_field_z_min: %.1f m", near_field_z_min_);
        }

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

        if (config["stationary_policy"]) {
            const auto policy = config["stationary_policy"];
            stationary_motion_policy_config_.enter_confirm_frames =
                policy["enter_confirm_frames"].as<int>(20);
            stationary_motion_policy_config_.enter_max_raw_increment_m =
                policy["enter_max_raw_increment_m"].as<double>(0.015);
            stationary_motion_policy_config_.enter_max_speed_mps =
                policy["enter_max_speed_mps"].as<double>(0.03);
            stationary_motion_policy_config_.exit_confirm_frames =
                policy["exit_confirm_frames"].as<int>(3);
            stationary_motion_policy_config_.exit_min_increment_m =
                policy["exit_min_increment_m"].as<double>(0.02);
            stationary_motion_policy_config_.exit_cumulative_motion_m =
                policy["exit_cumulative_motion_m"].as<double>(0.15);
            stationary_motion_policy_config_.exit_direction_cosine_min =
                policy["exit_direction_cosine_min"].as<double>(0.80);
            stationary_motion_policy_config_.exit_evidence_window_sec =
                policy["exit_evidence_window_sec"].as<double>(1.50);
            stationary_motion_policy_config_.exit_min_speed_mps =
                policy["exit_min_speed_mps"].as<double>(0.01);
            stationary_motion_policy_config_.exit_force_anchor_drift_m =
                policy["exit_force_anchor_drift_m"].as<double>(0.30);
            stationary_motion_policy_config_.moving_confirm_timeout_sec =
                policy["moving_confirm_timeout_sec"].as<double>(1.50);
            stationary_motion_policy_config_.catch_up_max_step_m =
                policy["catch_up_max_step_m"].as<double>(0.08);
            stationary_motion_policy_config_.catch_up_complete_error_m =
                policy["catch_up_complete_error_m"].as<double>(0.03);
            stationary_motion_policy_config_.catch_up_confirm_frames =
                policy["catch_up_confirm_frames"].as<int>(2);
            crane_motion_ekf_cfg_.stationary_position_hold_variance =
                policy["position_hold_variance"].as<double>(0.0025);
            crane_motion_ekf_cfg_.stationary_velocity_hold_variance =
                policy["velocity_hold_variance"].as<double>(0.001);
        }
        stationary_motion_policy_.setConfig(
            stationary_motion_policy_config_);

        CargoMarkerLifecycleConfig marker_lifecycle_config;
        if (config["cargo_marker"]) {
            marker_lifecycle_config.invalid_hold_sec =
                config["cargo_marker"]["invalid_hold_sec"].as<double>(2.0);
        }
        cargo_marker_lifecycle_.setConfig(marker_lifecycle_config);

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

        if (debug_cfg_.debug_config) {
            ROS_DEBUG("=== Long-Term Mapping Config ===");
            ROS_DEBUG("  longterm_mapping: %s", longterm_mapping_enabled_ ? "true" : "false");
            ROS_DEBUG("  motion_gate: %s", motion_gate_enabled_ ? "true" : "false");
            ROS_DEBUG("  motion_gate_min_translation: %.2f m", motion_gate_min_translation_m_);
            ROS_DEBUG("  motion_gate_min_rotation: %.1f deg", motion_gate_min_rotation_deg_);
            ROS_DEBUG("  motion_gate_min_time: %.1f sec", motion_gate_min_time_sec_);
            ROS_DEBUG("  max_active_keyframes: %d", max_active_keyframes_);
            ROS_DEBUG("  persistent_map: %s", persistent_map_enabled_ ? "true" : "false");
            ROS_DEBUG("  persistent_map_dir: %s", persistent_map_root_dir_.c_str());
        }

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
            if (debug_cfg_.debug_config) {
                ROS_DEBUG("[CraneConstraint] fixed_z_source=config, fixed_z=%.3f", fixed_z_);
            }
        }

        if (debug_cfg_.debug_config) {
        ROS_DEBUG("=== Crane Motion Constraint ===");
        ROS_DEBUG("  enabled: %s", crane_constraint_enabled_ ? "true" : "false");
        ROS_DEBUG("  lock_z: %s, fixed_z_source: %s, fixed_z: %.3f",
                 lock_z_ ? "true" : "false", fixed_z_source_.c_str(), fixed_z_);
        ROS_DEBUG("  lock_roll: %s, lock_pitch: %s", lock_roll_ ? "true" : "false", lock_pitch_ ? "true" : "false");
        ROS_DEBUG("  lock_yaw: %s, constrain_yaw: %s", lock_yaw_ ? "true" : "false", constrain_yaw_ ? "true" : "false");

        ROS_DEBUG("=== Memory Guard Config ===");
        ROS_DEBUG("  enabled: %s", memory_guard_enabled_ ? "true" : "false");
        ROS_DEBUG("  soft_threshold: %d MB", soft_threshold_mb_);
        ROS_DEBUG("  hard_threshold: %d MB", hard_threshold_mb_);
        ROS_DEBUG("  emergency_threshold: %d MB", emergency_threshold_mb_);
        ROS_DEBUG("  check_interval: %d sec", memory_check_interval_sec_);
        }

        // commit_enabled 配置（observe_only 模式）
        if (config["longterm_mapping"]) {
            auto ltm = config["longterm_mapping"];
            commit_enabled_ = ltm["commit_enabled"].as<bool>(true);
        }
        if (debug_cfg_.debug_config) {
            ROS_DEBUG("  commit_enabled: %s", commit_enabled_ ? "true" : "false");
        }

        if (config["disk_guard"]) {
            auto dg = config["disk_guard"];
            disk_guard_enabled_ = dg["enabled"].as<bool>(false);
            min_free_disk_gb_ = dg["min_free_disk_gb"].as<double>(30.0);
            pause_mapping_when_disk_low_ = dg["pause_mapping_when_low"].as<bool>(true);
        }

        if (debug_cfg_.debug_config) {
            ROS_DEBUG("=== Disk Guard Config ===");
            ROS_DEBUG("  enabled: %s", disk_guard_enabled_ ? "true" : "false");
            ROS_DEBUG("  min_free_disk: %.1f GB", min_free_disk_gb_);
        }

        if (config["pointcloud_watchdog"]) {
            auto pw = config["pointcloud_watchdog"];
            pointcloud_stale_timeout_sec_ = pw["stale_timeout_sec"].as<double>(10.0);
        }

        if (debug_cfg_.debug_config) {
            ROS_DEBUG("=== Pointcloud Watchdog Config ===");
            ROS_DEBUG("  stale_timeout: %.1f sec", pointcloud_stale_timeout_sec_);
        }

        if (config["ndt_health"]) {
            auto nh = config["ndt_health"];
            fitness_warning_threshold_ = nh["fitness_warning_threshold"].as<double>(2.0);
            fitness_warning_count_ = nh["fitness_warning_count"].as<int>(50);
        }

        if (config["active_map"]) {
            auto am = config["active_map"];
            rebuild_every_keyframes_ = am["rebuild_every_keyframes"].as<int>(10);
        }

        if (debug_cfg_.debug_config) {
            ROS_DEBUG("=== NDT Health Config ===");
            ROS_DEBUG("  fitness_warning_threshold: %.2f", fitness_warning_threshold_);
            ROS_DEBUG("  rebuild_every_keyframes: %d", rebuild_every_keyframes_);
            ROS_DEBUG("===========================");
        }

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

            ROS_DEBUG("[CraneMotionEKF] enabled=%d q_pos=%.3f q_vel=%.3f gate=%.2f reject=%.2f reject_high_fitness=%d fitness_reject=%.3f fitness_recover=%.3f",
                     crane_motion_ekf_enabled_ ? 1 : 0,
                     crane_motion_ekf_cfg_.q_pos,
                     crane_motion_ekf_cfg_.q_vel,
                     crane_motion_ekf_cfg_.innovation_gate_m,
                     crane_motion_ekf_cfg_.innovation_reject_m,
                     crane_motion_ekf_cfg_.reject_high_fitness ? 1 : 0,
                     crane_motion_ekf_cfg_.ndt_fitness_reject_threshold,
                     crane_motion_ekf_cfg_.ndt_fitness_recover_threshold);
            ROS_DEBUG("[CraneMotionEKF:RuntimeGuard] enabled=%d warn_ms=%.1f emergency_ms=%.1f slow_extra_r=%.3f",
                     crane_motion_ekf_cfg_.slow_frame_guard_enabled ? 1 : 0,
                     crane_motion_ekf_cfg_.slow_frame_warn_ms,
                     crane_motion_ekf_cfg_.slow_frame_emergency_ms,
                     crane_motion_ekf_cfg_.slow_frame_extra_r);
            ROS_DEBUG("[CraneMotionEKF:MotionLimit] max_speed=%.3f safety=%.3f step_min=%.3f step_max=%.3f axis_speed=(%.3f,%.3f) axis_accel=(%.3f,%.3f) axis_gate=%d map_commit_fitness=%.3f",
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
        ROS_DEBUG("[ICPConfig] enabled=%d run_after_ndt=%d objects_only=%d "
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

            ROS_DEBUG("[SoftYaw] enabled=%d alpha_static=%.3f alpha_moving=%.3f",
                     soft_yaw_enabled_ ? 1 : 0,
                     yaw_filter_alpha_stationary_,
                     yaw_filter_alpha_moving_);
        }

        // v8-stable-r3: Registration Input 参数
        if (config["registration_input"]) {
            const auto n = config["registration_input"];
            registration_cloud_config_.static_object_voxel_size =
                n["static_object_voxel_size"].as<double>(0.22);
            registration_cloud_config_.uncertain_candidate_voxel_size =
                n["uncertain_candidate_voxel_size"].as<double>(0.30);
            registration_cloud_config_.ground_grid_cell_m =
                n["ground_grid_cell_m"].as<double>(0.50);
            registration_cloud_config_.ground_edge_height_change_m =
                n["ground_edge_height_change_m"].as<double>(0.08);
            registration_cloud_config_.static_object_repeat =
                n["static_object_repeat"].as<int>(2);
            registration_cloud_config_.uncertain_candidate_repeat =
                n["uncertain_candidate_repeat"].as<int>(1);
            registration_cloud_config_.ground_max_fraction =
                n["ground_max_fraction"].as<double>(0.35);
            registration_cloud_config_.min_static_object_points =
                n["min_static_object_points"].as<std::size_t>(600U);
            registration_cloud_config_.min_structure_xy_cells =
                n["min_structure_xy_cells"].as<std::size_t>(40U);
            registration_cloud_config_.min_registration_points =
                n["min_registration_points"].as<std::size_t>(2500U);
            registration_cloud_config_.target_registration_points =
                n["target_registration_points"].as<std::size_t>(4000U);
            registration_cloud_config_.max_ndt_points =
                n["max_ndt_points"].as<std::size_t>(6000U);
            registration_cloud_config_.allow_full_ground_fallback =
                n["allow_full_ground_fallback"].as<bool>(false);
            if (registration_cloud_config_.allow_full_ground_fallback) {
                ROS_ERROR("[RegistrationInput] allow_full_ground_fallback=true is unsafe and will be ignored");
                registration_cloud_config_.allow_full_ground_fallback = false;
            }

            ROS_DEBUG("[RegistrationInput] static_voxel=%.2f uncertain_voxel=%.2f ground_grid=%.2f ground_max=%.2f static_min=%zu total_min=%zu target=%zu max=%zu",
                     registration_cloud_config_.static_object_voxel_size,
                     registration_cloud_config_.uncertain_candidate_voxel_size,
                     registration_cloud_config_.ground_grid_cell_m,
                     registration_cloud_config_.ground_max_fraction,
                     registration_cloud_config_.min_static_object_points,
                     registration_cloud_config_.min_registration_points,
                     registration_cloud_config_.target_registration_points,
                     registration_cloud_config_.max_ndt_points);
        }

        if (config["ndt_observability"]) {
            const auto n = config["ndt_observability"];
            ndt_observability_config_.enabled =
                n["enabled"].as<bool>(true);
            ndt_observability_config_.moderate_ratio =
                n["moderate_ratio"].as<double>(0.20);
            ndt_observability_config_.severe_ratio =
                n["severe_ratio"].as<double>(0.08);
            ndt_observability_config_.moderate_weak_inflation =
                n["moderate_weak_inflation"].as<double>(5.0);
            ndt_observability_config_.severe_weak_inflation =
                n["severe_weak_inflation"].as<double>(20.0);
            ndt_observability_config_.min_structure_points =
                n["min_structure_points"].as<std::size_t>(600U);
            ndt_observability_config_.min_occupied_cells =
                n["min_occupied_cells"].as<std::size_t>(40U);
            ndt_observability_config_.min_local_normals =
                n["min_local_normals"].as<std::size_t>(80U);
            ndt_observability_config_.max_normal_samples =
                n["max_normal_samples"].as<std::size_t>(800U);
            ndt_observability_config_.local_neighbor_count =
                n["local_neighbor_count"].as<int>(10);
            ndt_observability_config_.occupancy_cell_m =
                n["occupancy_cell_m"].as<double>(0.50);
            ndt_observability_config_.min_xy_span_m =
                n["min_xy_span_m"].as<double>(2.0);
            ndt_observability_config_.normal_search_radius_m =
                n["normal_search_radius_m"].as<double>(0.90);
            ndt_observability_config_.min_normal_anisotropy =
                n["min_normal_anisotropy"].as<double>(0.20);
        }
        crane_motion_ekf_cfg_.observability = ndt_observability_config_;
        crane_motion_ekf_.setConfig(crane_motion_ekf_cfg_);

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

            ROS_DEBUG("[LocTarget] build_enabled=%d use_for_ndt=%d objects_only=%d ground_edge=%d min=%d max=%d voxel=%.2f crop=%d radius=(%.1f,%.1f) update_dist=%.2f update_yaw=%.1f min_interval=%d",
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

            ROS_DEBUG("[AdaptiveNDT] enabled=%d target_ms=%.1f emergency_ms=%.1f",
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
        // and structured safety-status policy here.
        if (config["hook_load_signal"]) {
            const auto hls = config["hook_load_signal"];
            hook_load_signal_enabled_ = hls["enabled"].as<bool>(true);
            const bool legacy_required = hls["required"].as<bool>(true);
            const bool role_present = hls["role"] && hls["role"].IsScalar();
            const HookLoadRoleParseResult role_result =
                parseHookLoadSignalRole({
                    hook_load_signal_enabled_, role_present,
                    role_present ? hls["role"].as<std::string>() : std::string(),
                    legacy_required});
            hook_load_signal_role_ = role_result.role;
            hook_load_signal_role_config_valid_ = role_result.valid;
            hook_load_signal_enabled_ =
                hook_load_signal_role_ != HookLoadSignalRole::DISABLED;
            if (!role_result.valid) {
                ROS_ERROR("[HookLoadSignal] invalid role; fail-safe REQUIRED semantics active");
            }
            hook_load_state_topic_ =
                hls["state_topic"].as<std::string>("/hook/load_state");
            hook_load_state_stale_timeout_sec_ = std::max(
                0.10, hls["consumer_timeout_sec"].as<double>(3.00));
            const int history_samples = std::clamp(
                hls["origin_history_samples"].as<int>(10), 3, 200);
            empty_hook_height_history_max_samples_ =
                static_cast<std::size_t>(history_samples);
            origin_history_max_age_sec_ = std::clamp(
                hls["origin_history_max_age_sec"].as<double>(2.0), 0.2, 10.0);
            origin_future_stamp_tolerance_sec_ = std::clamp(
                hls["origin_future_stamp_tolerance_sec"].as<double>(0.05),
                0.0, 1.0);
            origin_history_max_position_spread_m_ = std::clamp(
                hls["origin_history_max_position_spread_m"].as<float>(0.35F),
                0.05F, 2.0F);
            origin_match_max_distance_m_ = std::clamp(
                hls["origin_match_max_distance_m"].as<float>(0.50F),
                0.05F, 3.0F);
            origin_height_max_mad_m_ = std::clamp(
                hls["origin_height_max_mad_m"].as<float>(0.10F),
                0.01F, 0.50F);
            origin_height_max_range_m_ = std::clamp(
                hls["origin_height_max_range_m"].as<float>(0.25F),
                0.02F, 1.0F);
            origin_size_max_relative_deviation_ = std::clamp(
                hls["origin_size_max_relative_deviation"].as<float>(0.30F),
                0.05F, 1.0F);
            origin_min_confidence_ = std::clamp(
                hls["origin_min_confidence"].as<float>(0.50F),
                0.0F, 1.0F);
            formal_cargo_removal_max_age_sec_ = std::clamp(
                hls["formal_removal_max_age_sec"].as<double>(0.80),
                0.10, 3.0);
            lidar_no_cargo_evidence_.setConfig({
                static_cast<std::uint32_t>(std::clamp(
                    hls["lidar_empty_confirm_frames"].as<int>(3), 2, 20))});
            ROS_INFO(
                "[HookLoadSignal] enabled=%d role=%s role_source=%s state_topic=%s "
                "consumer_stale=%.2fs origin_samples=%zu origin_age=%.2fs "
                "origin_future_tolerance=%.3fs origin_spread=%.2fm "
                "origin_match=%.2fm",
                hook_load_signal_enabled_ ? 1 : 0,
                hookLoadSignalRoleName(hook_load_signal_role_),
                role_result.reason.c_str(),
                hook_load_state_topic_.c_str(),
                hook_load_state_stale_timeout_sec_,
                empty_hook_height_history_max_samples_,
                origin_history_max_age_sec_,
                origin_future_stamp_tolerance_sec_,
                origin_history_max_position_spread_m_,
                origin_match_max_distance_m_);
        }

        if (config["map_maintenance"]) {
            const auto maintenance = config["map_maintenance"];
            map_maintenance_interval_commits_ = std::clamp(
                maintenance["interval_commits"].as<int>(3), 1, 100);
            map_maintenance_max_deferral_frames_ = std::clamp(
                maintenance["max_deferral_frames"].as<int>(5), 1, 1000);
        }

        if (config["hook_cargo_lock"]) {
            const auto hcl = config["hook_cargo_lock"];
            hook_lock_config_.enabled = hcl["enabled"].as<bool>(true);
            hook_lock_config_.lock_confirm_frames = hcl["lock_confirm_frames"].as<int>(3);
            hook_lock_config_.geometry_confirm_frames = std::max(
                3, hcl["geometry_confirm_frames"].as<int>(4));
            hook_lock_config_.size_init_window = hcl["size_init_window"].as<int>(5);
            hook_lock_config_.lost_hold_sec = hcl["lost_hold_sec"].as<float>(3.0f);
            hook_lock_config_.lost_clear_sec = hcl["lost_clear_sec"].as<float>(8.0f);
            hook_lock_config_.strong_min_points = hcl["strong_min_points"].as<int>(30);
            hook_lock_config_.weak_min_points = hcl["weak_min_points"].as<int>(5);
            hook_lock_config_.size_change_min_ratio = hcl["size_change_min_ratio"].as<float>(0.20f);
            hook_lock_config_.size_change_max_ratio = hcl["size_change_max_ratio"].as<float>(0.60f);
            hook_lock_config_.size_update_confirm_frames = hcl["size_update_confirm_frames"].as<int>(5);
            hook_lock_config_.size_update_alpha = hcl["size_update_alpha"].as<float>(0.15f);
            hook_lock_config_.freeze_geometry_after_lock =
                hcl["freeze_geometry_after_lock"].as<bool>(true);
            hook_lock_config_.axis_aligned_yaw_after_lock =
                hcl["axis_aligned_yaw_after_lock"].as<bool>(true);
            hook_lock_config_.freeze_vertical_position_after_lock =
                hcl["freeze_vertical_position_after_lock"].as<bool>(false);
            hook_lock_config_.track_vertical_from_top_surface =
                hcl["track_vertical_from_top_surface"].as<bool>(true);
            hook_lock_config_.top_bottom_center_agreement_m = std::max(
                0.05F, hcl["top_bottom_center_agreement_m"]
                           .as<float>(0.25F));
            hook_lock_config_.bottom_alpha_points = hcl["bottom_alpha_points"].as<float>(0.30f);
            hook_lock_config_.bottom_alpha_memory = hcl["bottom_alpha_memory"].as<float>(0.15f);
            hook_lock_config_.bottom_hold_uncertainty_growth = hcl["bottom_hold_uncertainty_growth"].as<float>(0.02f);
            hook_lock_config_.bottom_max_uncertainty = hcl["bottom_max_uncertainty"].as<float>(0.35f);
            hook_lock_config_.candidate_hold_sec = hcl["candidate_hold_sec"].as<float>(1.0f);
            hook_lock_config_.candidate_max_weak_frames = hcl["candidate_max_weak_frames"].as<int>(10);
            hook_lock_config_.candidate_window_frames = std::max(
                4, hcl["candidate_window_frames"].as<int>(12));
            hook_lock_config_.candidate_required_consistent_frames =
                std::clamp(
                    hcl["candidate_required_consistent_frames"].as<int>(7),
                    3, hook_lock_config_.candidate_window_frames);
            hook_lock_config_.candidate_max_gap_frames = std::max(
                0, hcl["candidate_max_gap_frames"].as<int>(2));
            hook_lock_config_.candidate_progress_timeout_sec = std::max(
                0.5F, hcl["candidate_progress_timeout_sec"].as<float>(3.0F));
            hook_lock_config_.candidate_absolute_timeout_sec = std::max(
                hook_lock_config_.candidate_progress_timeout_sec,
                hcl["candidate_absolute_timeout_sec"].as<float>(8.0F));
            hook_lock_config_.candidate_switch_confirm_frames = std::max(
                2, hcl["candidate_switch_confirm_frames"].as<int>(3));
            hook_lock_config_.candidate_switch_margin = std::clamp(
                hcl["candidate_switch_margin"].as<float>(0.08F),
                0.0F, 1.0F);

            // locked association gate 配置
            hook_lock_config_.locked_update_max_center_dist = hcl["locked_update_max_center_dist"].as<float>(0.65f);
            hook_lock_config_.locked_update_min_overlap_ratio = hcl["locked_update_min_overlap_ratio"].as<float>(0.30f);
            hook_lock_config_.locked_update_max_z_jump = hcl["locked_update_max_z_jump"].as<float>(0.45f);
            hook_lock_config_.locked_update_max_top_jump = hcl["locked_update_max_top_jump"].as<float>(0.60f);
            hook_lock_config_.locked_update_min_points = hcl["locked_update_min_points"].as<int>(20);
            hook_lock_config_.minimum_identity_confidence = std::clamp(
                hcl["minimum_identity_confidence"].as<float>(0.62F),
                0.0F, 1.0F);
            hook_lock_config_.minimum_overall_lock_confidence = std::clamp(
                hcl["minimum_overall_lock_confidence"].as<float>(0.68F),
                0.0F, 1.0F);
            hook_lock_config_.maximum_provisional_shape_cv = std::clamp(
                hcl["maximum_provisional_shape_cv"].as<float>(0.20F),
                0.02F, 0.75F);
            hook_lock_config_.minimum_candidate_score_margin = std::clamp(
                hcl["minimum_candidate_score_margin"].as<float>(0.08F),
                0.0F, 1.0F);
            hook_lock_config_.suspension_confirm_frames = std::max(
                2, hcl["suspension_confirm_frames"].as<int>(3));
            hook_lock_config_.minimum_lift_from_origin_m = std::max(
                0.05F,
                hcl["minimum_lift_from_origin_m"].as<float>(0.25F));
            hook_lock_config_.reacquisition_overlap_extra = std::clamp(
                hcl["reacquisition_overlap_extra"].as<float>(0.10F),
                0.0F, 0.50F);
            hook_lock_config_.residual_uncertainty_decay = std::clamp(
                hcl["residual_uncertainty_decay"].as<float>(0.80F),
                0.0F, 0.99F);
            hook_lock_config_.live_pose_center_alpha = std::clamp(
                hcl["live_pose_center_alpha"].as<float>(0.45F), 0.0F, 1.0F);
            hook_lock_config_.live_pose_max_xy_speed_mps = std::max(
                0.01F,
                hcl["live_pose_max_xy_speed_mps"].as<float>(2.0F));
            hook_lock_config_.live_pose_max_z_speed_mps = std::max(
                0.01F,
                hcl["live_pose_max_z_speed_mps"].as<float>(1.5F));
            hook_lock_config_.live_pose_step_margin_m = std::max(
                0.0F,
                hcl["live_pose_step_margin_m"].as<float>(0.05F));
            hook_lock_config_.live_pose_velocity_alpha = std::clamp(
                hcl["live_pose_velocity_alpha"].as<float>(0.35F),
                0.0F, 1.0F);
            hook_lock_config_.formal_xy_evidence_hold_sec = std::max(
                0.10F,
                hcl["formal_xy_evidence_hold_sec"].as<float>(2.00F));
            hook_lock_config_.formal_vertical_evidence_hold_sec = std::max(
                0.10F,
                hcl["formal_vertical_evidence_hold_sec"].as<float>(2.00F));
            hook_lock_config_.direct_bottom_soft_stale_sec = std::max(
                hook_lock_config_.formal_vertical_evidence_hold_sec,
                hcl["direct_bottom_soft_stale_sec"].as<float>(1.50F));
            hook_lock_config_.velocity_model_uncertainty_mps = std::max(
                0.0F,
                hcl["velocity_model_uncertainty_mps"].as<float>(0.05F));
            hook_lock_config_.association_max_xy_gate_m = std::max(
                0.10F, hcl["association_max_xy_gate_m"].as<float>(0.80F));
            hook_lock_config_.reacquisition_max_xy_gate_m = std::clamp(
                hcl["reacquisition_max_xy_gate_m"].as<float>(0.55F),
                0.10F, hook_lock_config_.association_max_xy_gate_m);
            hook_lock_config_.association_max_z_gate_m = std::max(
                0.10F, hcl["association_max_z_gate_m"].as<float>(0.90F));
            hook_lock_config_.reacquisition_max_z_gate_m = std::clamp(
                hcl["reacquisition_max_z_gate_m"].as<float>(0.65F),
                0.10F, hook_lock_config_.association_max_z_gate_m);
            hook_lock_config_.locked_obb_min_support_ratio = std::clamp(
                hcl["locked_obb_min_support_ratio"].as<float>(0.30F),
                0.05F, 0.95F);
            hook_lock_config_.locked_obb_min_long_axis_coverage = std::clamp(
                hcl["locked_obb_min_long_axis_coverage"].as<float>(0.12F),
                0.02F, 1.0F);
            hook_lock_config_.locked_obb_min_short_axis_coverage = std::clamp(
                hcl["locked_obb_min_short_axis_coverage"].as<float>(0.12F),
                0.02F, 1.0F);
            hook_lock_config_.lost_velocity_decay_tau_sec = std::max(
                0.05F,
                hcl["lost_velocity_decay_tau_sec"].as<float>(0.30F));
            hook_lock_config_.rearm_empty_confirm_sec = std::max(
                0.0F, hcl["rearm_empty_confirm_sec"].as<float>(1.0F));
            hook_lock_config_.self_cargo_base_margin_xy_m = std::max(
                0.0F,
                hcl["self_cargo_base_margin_xy_m"].as<float>(0.15F));
            hook_lock_config_.self_cargo_base_margin_z_m = std::max(
                0.0F,
                hcl["self_cargo_base_margin_z_m"].as<float>(0.12F));
            hook_lock_config_.self_cargo_max_margin_xy_m = std::max(
                hook_lock_config_.self_cargo_base_margin_xy_m,
                hcl["self_cargo_max_margin_xy_m"].as<float>(0.40F));
            hook_lock_config_.self_cargo_max_margin_z_m = std::max(
                hook_lock_config_.self_cargo_base_margin_z_m,
                hcl["self_cargo_max_margin_z_m"].as<float>(0.30F));
            hook_lock_config_.self_cargo_point_match_radius_m = std::clamp(
                hcl["self_cargo_point_match_radius_m"].as<float>(0.15F),
                0.08F, 0.20F);
            hook_lock_config_.self_rigging_radius_m = std::clamp(
                hcl["self_rigging_radius_m"].as<float>(0.10F),
                0.03F, 0.20F);
            hook_lock_config_.lost_position_uncertainty_per_sec = std::max(
                0.0F,
                hcl["lost_position_uncertainty_per_sec"].as<float>(0.05F));
            hook_lock_config_.lost_position_uncertainty_max_m = std::max(
                0.0F,
                hcl["lost_position_uncertainty_max_m"].as<float>(0.50F));

            // 锁定时 strong 条件
            hook_lock_config_.lock_strong_min_points = hcl["lock_strong_min_points"].as<int>(80);
            hook_lock_config_.lock_min_visible_height = hcl["lock_min_visible_height"].as<float>(0.50f);
            hook_lock_config_.lock_min_xy_area = hcl["lock_min_xy_area"].as<float>(0.40f);
            hook_lock_config_.lock_max_center_step_m = std::clamp(
                hcl["lock_max_center_step_m"].as<float>(0.30F),
                0.05F, 1.0F);
            hook_lock_config_.compact_lock_enabled =
                hcl["compact_lock_enabled"].as<bool>(true);
            hook_lock_config_.compact_min_points =
                hcl["compact_min_points"].as<int>(40);
            hook_lock_config_.compact_min_visible_height =
                hcl["compact_min_visible_height"].as<float>(0.18F);
            hook_lock_config_.compact_min_xy_area =
                hcl["compact_min_xy_area"].as<float>(0.12F);
            hook_lock_config_.compact_confirm_frames = std::max(
                3, hcl["compact_confirm_frames"].as<int>(5));
            hook_lock_config_.compact_max_size_relative_step = std::clamp(
                hcl["compact_max_size_relative_step"].as<float>(0.25F),
                0.05F, 0.75F);
            hook_lock_config_.suspended_min_ground_clearance_m = std::clamp(
                hcl["suspended_min_ground_clearance_m"].as<float>(0.30F),
                0.10F, 1.50F);

            // locked search margin
            hook_lock_config_.locked_search_margin_x = hcl["locked_search_margin_x"].as<float>(0.30f);
            hook_lock_config_.locked_search_margin_y = hcl["locked_search_margin_y"].as<float>(0.30f);

            // 吊物点云去除
            hook_lock_config_.enable_hook_cargo_removal = hcl["enable_hook_cargo_removal"].as<bool>(false);

            ROS_DEBUG("[HookCargoLock] enabled=%d lock_confirm=%d lost_hold=%.1f lost_clear=%.1f strong=%d weak=%d removal=%d",
                     hook_lock_config_.enabled ? 1 : 0,
                     hook_lock_config_.lock_confirm_frames,
                     hook_lock_config_.lost_hold_sec,
                     hook_lock_config_.lost_clear_sec,
                     hook_lock_config_.strong_min_points,
                     hook_lock_config_.weak_min_points,
                     hook_lock_config_.enable_hook_cargo_removal ? 1 : 0);
            ROS_DEBUG("[HookCargoLock] lock_strong=%d min_visible_h=%.2f min_xy_area=%.2f max_center_dist=%.2f",
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
            odom_anchor_config_.detect_rate_hz = std::clamp(
                oac["detect_rate_hz"].as<float>(5.0F), 0.1F, 50.0F);
            odom_anchor_config_.marker_rate_hz = std::clamp(
                oac["marker_rate_hz"].as<float>(5.0F), 0.1F, 50.0F);

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
                odom_anchor_config_.tight_box.ground_ring_width_m =
                    tb["ground_ring_width_m"].as<float>(2.0f);
                odom_anchor_config_.tight_box.ground_cell_size_m =
                    tb["ground_cell_size_m"].as<float>(0.50f);
                odom_anchor_config_.tight_box.ground_min_cells =
                    tb["ground_min_cells"].as<int>(4);
                odom_anchor_config_.tight_box.ground_min_points_per_cell =
                    tb["ground_min_points_per_cell"].as<int>(3);
                odom_anchor_config_.tight_box.ground_min_quadrants =
                    tb["ground_min_quadrants"].as<int>(3);
                odom_anchor_config_.tight_box.ground_allow_opposite_sides =
                    tb["ground_allow_opposite_sides"].as<bool>(true);
                odom_anchor_config_.tight_box.ground_max_range_m =
                    tb["ground_max_range_m"].as<float>(0.15f);
                odom_anchor_config_.tight_box.ground_expected_height_enabled =
                    tb["ground_expected_height_enabled"].as<bool>(true);
                odom_anchor_config_.tight_box.ground_expected_height_m =
                    tb["ground_expected_height_m"].as<float>(0.0f);
                odom_anchor_config_.tight_box.ground_max_expected_height_delta_m =
                    tb["ground_max_expected_height_delta_m"].as<float>(0.30f);
                odom_anchor_config_.tight_box.empty_max_hag_candidate_points =
                    std::clamp(
                        tb["empty_max_hag_candidate_points"].as<int>(2),
                        0, 20);
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
                odom_anchor_config_.tight_box.component_cluster_tolerance_m =
                    std::max(0.05F,
                        tb["component_cluster_tolerance_m"].as<float>(0.20F));
                odom_anchor_config_.tight_box.component_merge_longitudinal_gap_m =
                    std::max(0.0F,
                        tb["component_merge_longitudinal_gap_m"].as<float>(0.60F));
                odom_anchor_config_.tight_box.component_merge_lateral_gap_m =
                    std::max(0.0F,
                        tb["component_merge_lateral_gap_m"].as<float>(0.35F));
                odom_anchor_config_.tight_box.component_merge_max_yaw_difference_deg =
                    std::clamp(
                        tb["component_merge_max_yaw_difference_deg"].as<float>(15.0F),
                        1.0F, 45.0F);
                odom_anchor_config_.tight_box.component_merge_min_z_overlap_ratio =
                    std::clamp(
                        tb["component_merge_min_z_overlap_ratio"].as<float>(0.30F),
                        0.0F, 1.0F);
                odom_anchor_config_.tight_box.component_merge_max_components =
                    std::clamp(
                        tb["component_merge_max_components"].as<int>(3), 1, 3);
                odom_anchor_config_.tight_box.orientation_enabled =
                    tb["orientation_enabled"].as<bool>(true);
                odom_anchor_config_.tight_box.orientation_min_points =
                    std::max(3, tb["orientation_min_points"].as<int>(20));
                odom_anchor_config_.tight_box.orientation_min_geometric_aspect_ratio =
                    std::max(1.0F, tb["orientation_min_geometric_aspect_ratio"]
                        .as<float>(1.20F));
                odom_anchor_config_.tight_box.orientation_min_eigenvalue_ratio =
                    std::max(1.0F, tb["orientation_min_eigenvalue_ratio"]
                        .as<float>(1.44F));
                odom_anchor_config_.tight_box.orientation_min_concentration =
                    std::clamp(tb["orientation_min_concentration"]
                        .as<float>(0.70F), 0.0F, 1.0F);
                odom_anchor_config_.tight_box.orientation_min_confirm_frames =
                    std::max(3, tb["orientation_min_confirm_frames"]
                        .as<int>(3));
                odom_anchor_config_.tight_box.orientation_max_yaw_spread_deg =
                    std::clamp(tb["orientation_max_yaw_spread_deg"]
                        .as<float>(12.0F), 1.0F, 45.0F);

                ROS_DEBUG("[TightBoxConfig] enabled=%d symmetry=%s hag_filter=%d percentile=[%.2f,%.2f] sub_cluster=%d oriented=%d",
                         odom_anchor_config_.tight_box.enabled ? 1 : 0,
                         odom_anchor_config_.tight_box.anchor_symmetry_mode.c_str(),
                         odom_anchor_config_.tight_box.hag_filter_enabled ? 1 : 0,
                         odom_anchor_config_.tight_box.percentile_low,
                         odom_anchor_config_.tight_box.percentile_high,
                         odom_anchor_config_.tight_box.sub_cluster_enabled ? 1 : 0,
                         odom_anchor_config_.tight_box.orientation_enabled ? 1 : 0);
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
                    std::clamp(cw["minimum_roi_coverage_ratio"].as<float>(0.05F),
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
                odom_anchor_config_.cargo_warning.clear_alarm_code = cw["clear_alarm_code"].as<int>(14);
                if (odom_anchor_config_.cargo_warning.clear_alarm_code !=
                        lidar_slam2_msgs::CargoSafetyStatus::CODE_CLEAR ||
                    odom_anchor_config_.cargo_warning.level1_alarm_code !=
                        lidar_slam2_msgs::CargoSafetyStatus::CODE_LEVEL1_WARNING ||
                    odom_anchor_config_.cargo_warning.level2_alarm_code !=
                        lidar_slam2_msgs::CargoSafetyStatus::CODE_LEVEL2_WARNING) {
                    ROS_ERROR("[CargoWarningConfig] invalid status codes; forcing 14/17/18 contract");
                    cargo_safety_config_error_ = true;
                    odom_anchor_config_.cargo_warning.clear_alarm_code =
                        lidar_slam2_msgs::CargoSafetyStatus::CODE_CLEAR;
                    odom_anchor_config_.cargo_warning.level1_alarm_code =
                        lidar_slam2_msgs::CargoSafetyStatus::CODE_LEVEL1_WARNING;
                    odom_anchor_config_.cargo_warning.level2_alarm_code =
                        lidar_slam2_msgs::CargoSafetyStatus::CODE_LEVEL2_WARNING;
                }

                ROS_DEBUG("[CargoWarningConfig] enabled=%d publish_alarm=%d debug_marker=%d level1_dist=%.1f level2_dist=%.1f clearance=%.2f",
                         odom_anchor_config_.cargo_warning.enabled ? 1 : 0,
                         odom_anchor_config_.cargo_warning.publish_alarm_msg ? 1 : 0,
                         odom_anchor_config_.cargo_warning.publish_debug_marker ? 1 : 0,
                         odom_anchor_config_.cargo_warning.level1_distance_m,
                         odom_anchor_config_.cargo_warning.level2_distance_m,
                         odom_anchor_config_.cargo_warning.min_vertical_clearance_m);
            }

            ROS_DEBUG("[OdomAnchorBoxConfig] enabled=%d anchor=(%.2f,%.2f) detect_rate=%.1f marker_rate=%.1f debug_points=%d global_payload=%d cargobox_v2=%d dynamic_eraser=%d",
                     odom_anchor_config_.enabled ? 1 : 0,
                     odom_anchor_config_.anchor_x, odom_anchor_config_.anchor_y,
                     odom_anchor_config_.detect_rate_hz, odom_anchor_config_.marker_rate_hz,
                     odom_anchor_config_.publish_debug_points ? 1 : 0,
                     odom_anchor_config_.use_global_payload_tracker ? 1 : 0,
                     odom_anchor_config_.use_cargobox_v2 ? 1 : 0,
                     odom_anchor_config_.use_dynamic_history_eraser ? 1 : 0);
        }

        // ConfigFinal 日志
        ROS_DEBUG("[ConfigFinal] hook_cargo_removal=%d source=config",
                 hook_lock_config_.enable_hook_cargo_removal ? 1 : 0);

        // Runtime Diagnostics 配置
        if (config["debug"] && config["debug"]["runtime_diagnostics"]) {
            auto diag = config["debug"]["runtime_diagnostics"];
            runtime_diag_config_.enabled = diag["enabled"].as<bool>(false);
            runtime_diag_config_.console_health_enabled =
                diag["console_health_enabled"].as<bool>(false);
            runtime_diag_config_.console_risk_enabled =
                diag["console_risk_enabled"].as<bool>(false);
            runtime_diag_config_.cargo_console_enabled =
                diag["cargo_console_enabled"].as<bool>(true);
            runtime_diag_config_.health_period_sec =
                diag["health_period_sec"].as<double>(10.0);
            runtime_diag_config_.risk_repeat_period_sec =
                diag["risk_repeat_period_sec"].as<double>(10.0);
            runtime_diag_config_.csv_enabled = diag["csv_enabled"].as<bool>(true);
            runtime_diag_config_.csv_flush_period_sec = diag["csv_flush_period_sec"].as<double>(1.0);
            runtime_diag_config_.warn_consecutive_overrun_frames = diag["warn_consecutive_overrun_frames"].as<int>(3);
            runtime_diag_config_.warn_prediction_only_frames = diag["warn_prediction_only_frames"].as<int>(3);
            runtime_diag_config_.warn_target_fallback_frames = diag["warn_target_fallback_frames"].as<int>(3);
            runtime_diag_config_.warn_cargo_bottom_jump_m = diag["warn_cargo_bottom_jump_m"].as<double>(0.20);
            runtime_diag_config_.warn_cargo_height_jump_m = diag["warn_cargo_height_jump_m"].as<double>(0.20);

            diag_output_dir_ = diag["output_dir"].as<std::string>(
                "/tmp/ndt_slam_runtime_data");

            ROS_DEBUG("[RuntimeDiagnostics] enabled=%d health_console=%d risk_console=%d cargo_console=%d health_period=%.1f risk_repeat=%.1f csv=%d csv_flush=%.1f",
                     runtime_diag_config_.enabled ? 1 : 0,
                     runtime_diag_config_.console_health_enabled ? 1 : 0,
                     runtime_diag_config_.console_risk_enabled ? 1 : 0,
                     runtime_diag_config_.cargo_console_enabled ? 1 : 0,
                     runtime_diag_config_.health_period_sec,
                     runtime_diag_config_.risk_repeat_period_sec,
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
        const YAML::Node cargo_safety = config["cargo_safety"];
        safety_config.level1_distance_m = cargo_safety
            ? cargo_safety["level1_distance_m"].as<float>(3.0F) : 3.0F;
        safety_config.level2_distance_m = cargo_safety
            ? cargo_safety["level2_distance_m"].as<float>(5.0F) : 5.0F;
        safety_config.minimum_vertical_clearance_m = cargo_safety
            ? cargo_safety["minimum_vertical_clearance_m"].as<float>(0.80F)
            : 0.80F;
        safety_config.cargo_bottom_extra_margin_m =
            odom_anchor_config_.cargo_warning.cargo_bottom_extra_margin_m;
        safety_config.obstacle_top_percentile =
            odom_anchor_config_.cargo_warning.obstacle_top_percentile;
        safety_config.obstacle_cluster_tolerance_m =
            odom_anchor_config_.cargo_warning.obstacle_cluster_tolerance_m;
        safety_config.obstacle_min_cluster_points =
            static_cast<std::size_t>(std::max(
                1, odom_anchor_config_.cargo_warning.obstacle_min_points));
        safety_config.maximum_obstacle_cloud_age_sec = cargo_safety
            ? cargo_safety["maximum_obstacle_cloud_age_sec"].as<double>(0.50)
            : 0.50;
        safety_config.minimum_roi_finite_points =
            static_cast<std::size_t>(
                odom_anchor_config_.cargo_warning.minimum_roi_finite_points);
        safety_config.minimum_roi_coverage_ratio = cargo_safety
            ? cargo_safety["minimum_roi_coverage_ratio"].as<float>(0.05F)
            : 0.05F;
        // Runtime identity/motion classification is the sole self-removal
        // authority. CargoSafetyEvaluator is intentionally stateless and
        // never applies a second geometry-only deletion volume.

        const auto nearly_equal = [](double lhs, double rhs) {
            return std::isfinite(lhs) && std::abs(lhs - rhs) <= 1.0e-6;
        };
        bool contract_valid =
            nearly_equal(safety_config.level1_distance_m, 3.0) &&
            nearly_equal(safety_config.level2_distance_m, 5.0) &&
            nearly_equal(safety_config.minimum_vertical_clearance_m, 0.80) &&
            nearly_equal(safety_config.minimum_roi_coverage_ratio, 0.05) &&
            nearly_equal(safety_config.maximum_obstacle_cloud_age_sec, 0.50);

        const YAML::Node status_codes = config["status_codes"];
        if (status_codes) {
            contract_valid = contract_valid &&
                status_codes["clear"].as<int>(14) == 14 &&
                status_codes["level1"].as<int>(17) == 17 &&
                status_codes["level2"].as<int>(18) == 18 &&
                status_codes["system_not_ready"].as<int>(30) == 30 &&
                status_codes["localization_invalid"].as<int>(31) == 31 &&
                status_codes["gravity_invalid"].as<int>(32) == 32 &&
                status_codes["cargo_invalid"].as<int>(33) == 33 &&
                status_codes["obstacle_invalid"].as<int>(34) == 34 &&
                status_codes["internal_error"].as<int>(35) == 35;
        }

        if (!contract_valid) {
            ROS_ERROR("[CargoSafetyConfig] invalid safety/status contract; "
                      "restoring 3.0/5.0/0.80 and status codes 14/17/18/30-35");
            cargo_safety_config_error_ = true;
            safety_config.level1_distance_m = 3.0F;
            safety_config.level2_distance_m = 5.0F;
            safety_config.minimum_vertical_clearance_m = 0.80F;
            safety_config.minimum_roi_coverage_ratio = 0.05F;
            safety_config.maximum_obstacle_cloud_age_sec = 0.50;
        }
        cargo_safety_evaluator_.setConfig(safety_config);
        CargoSafetyTemporalConfig temporal_config;
        if (cargo_safety) {
            temporal_config.hazard_confirm_frames = std::max(
                2, cargo_safety["hazard_confirm_frames"].as<int>(3));
            temporal_config.clear_confirm_frames = std::max(
                2, cargo_safety["clear_confirm_frames"].as<int>(2));
            temporal_config.minimum_hazard_cluster_points =
                static_cast<std::size_t>(std::max(
                    1, cargo_safety["minimum_hazard_cluster_points"]
                           .as<int>(20)));
            temporal_config.maximum_evidence_gap_sec = std::max(
                0.10, cargo_safety["maximum_evidence_gap_sec"]
                          .as<double>(0.60));
            temporal_config.maximum_centroid_step_m = std::max(
                0.05F, cargo_safety["maximum_centroid_step_m"]
                           .as<float>(0.75F));
            temporal_config.maximum_distance_step_m = std::max(
                0.05F, cargo_safety["maximum_distance_step_m"]
                           .as<float>(0.75F));
            temporal_config.maximum_clearance_step_m = std::max(
                0.05F, cargo_safety["maximum_clearance_step_m"]
                           .as<float>(0.75F));
        }
        cargo_safety_temporal_filter_.setConfig(temporal_config);
        CargoObstacleTrackerConfig obstacle_tracker_config;
        obstacle_tracker_config.confirm_frames =
            temporal_config.hazard_confirm_frames;
        obstacle_tracker_config.minimum_points =
            temporal_config.minimum_hazard_cluster_points;
        obstacle_tracker_config.maximum_observation_gap_sec =
            temporal_config.maximum_evidence_gap_sec;
        if (cargo_safety) {
            obstacle_tracker_config.association_max_centroid_distance_m =
                std::max(0.05F,
                    cargo_safety["obstacle_track_association_distance_m"]
                        .as<float>(0.75F));
            obstacle_tracker_config.association_max_top_step_m =
                std::max(0.05F,
                    cargo_safety["obstacle_track_association_top_step_m"]
                        .as<float>(0.75F));
            obstacle_tracker_config.stale_track_sec = std::max(
                obstacle_tracker_config.maximum_observation_gap_sec,
                cargo_safety["obstacle_track_stale_sec"].as<double>(1.00));
            obstacle_tracker_config.require_static_cargo_for_warning =
                cargo_safety["require_static_cargo_for_warning"]
                    .as<bool>(true);
            obstacle_tracker_config.static_cargo_min_voxel_points =
                static_cast<std::size_t>(std::max(
                    static_cast<int>(obstacle_tracker_config.minimum_points),
                    cargo_safety["static_cargo_min_voxel_points"]
                        .as<int>(80)));
            obstacle_tracker_config.static_cargo_min_raw_equivalent_points =
                static_cast<std::size_t>(std::max(
                    0, cargo_safety[
                           "static_cargo_min_raw_equivalent_points"]
                           .as<int>(0)));
            obstacle_tracker_config.static_cargo_min_xy_area_m2 = std::max(
                0.01F, cargo_safety["static_cargo_min_xy_area_m2"]
                           .as<float>(0.50F));
            obstacle_tracker_config.static_cargo_min_long_side_m = std::max(
                0.10F, cargo_safety["static_cargo_min_long_side_m"]
                           .as<float>(0.80F));
            obstacle_tracker_config.static_cargo_min_height_span_m = std::max(
                0.10F, cargo_safety["static_cargo_min_height_span_m"]
                           .as<float>(0.40F));
            obstacle_tracker_config.static_cargo_min_occupied_cells =
                static_cast<std::size_t>(std::max(
                    1, cargo_safety["static_cargo_min_occupied_cells"]
                           .as<int>(12)));
            obstacle_tracker_config.static_cargo_confirm_frames = std::max(
                obstacle_tracker_config.confirm_frames,
                cargo_safety["static_cargo_confirm_frames"].as<int>(8));
            obstacle_tracker_config.static_cargo_confirm_sec = std::max(
                0.0, cargo_safety["static_cargo_confirm_sec"]
                         .as<double>(1.0));
            obstacle_tracker_config.static_velocity_threshold_mps =
                std::max(0.0F,
                    cargo_safety["obstacle_track_static_velocity_mps"]
                        .as<float>(0.08F));
        }
        cargo_obstacle_tracker_.setConfig(obstacle_tracker_config);
        if (cargo_safety) {
            cargo_motion_corridor_config_.enabled =
                cargo_safety["motion_corridor_enabled"].as<bool>(true);
            cargo_motion_corridor_config_.immediate_near_field_m =
                std::max(0.0F,
                    cargo_safety[
                        "motion_corridor_immediate_near_field_m"]
                        .as<float>(0.30F));
            cargo_motion_corridor_config_.minimum_motion_speed_mps =
                std::max(0.0F,
                    cargo_safety["motion_corridor_minimum_speed_mps"]
                        .as<float>(0.05F));
            cargo_motion_corridor_config_.prediction_horizon_sec =
                std::max(0.5F,
                    cargo_safety[
                        "motion_corridor_prediction_horizon_sec"]
                        .as<float>(3.0F));
            cargo_motion_corridor_config_.lateral_margin_m =
                std::max(0.0F,
                    cargo_safety["motion_corridor_lateral_margin_m"]
                        .as<float>(0.30F));
            cargo_motion_corridor_config_.rear_margin_m =
                std::max(0.0F,
                    cargo_safety["motion_corridor_rear_margin_m"]
                        .as<float>(0.30F));
            cargo_motion_corridor_config_.velocity_alpha = std::clamp(
                cargo_safety["motion_corridor_velocity_alpha"]
                    .as<float>(0.35F),
                0.0F, 1.0F);
            cargo_motion_corridor_config_.maximum_velocity_sample_gap_sec =
                std::max(0.10,
                    cargo_safety[
                        "motion_corridor_maximum_velocity_sample_gap_sec"]
                        .as<double>(0.80));
            cargo_residual_classifier_config_.validation_shell_m =
                std::max(0.0F,
                    cargo_safety["residual_validation_shell_min_m"]
                        .as<float>(0.30F));
            cargo_residual_classifier_config_.minimum_inside_xy_ratio =
                std::clamp(
                    cargo_safety["residual_minimum_inside_xy_ratio"]
                        .as<float>(0.60F),
                    0.0F, 1.0F);
            cargo_residual_classifier_config_.minimum_identity_match_ratio =
                std::clamp(
                    cargo_safety["residual_minimum_identity_match_ratio"]
                        .as<float>(0.35F),
                    0.0F, 1.0F);
            cargo_residual_classifier_config_.minimum_surface_band_ratio =
                std::clamp(
                    cargo_safety["residual_minimum_surface_band_ratio"]
                        .as<float>(0.50F),
                    0.0F, 1.0F);
            cargo_residual_classifier_config_.minimum_motion_match_score =
                std::clamp(
                    cargo_safety["residual_minimum_motion_match_score"]
                        .as<float>(0.70F),
                    0.0F, 1.0F);
            cargo_residual_surface_band_below_m_ = std::max(
                0.0F,
                cargo_safety["residual_surface_band_below_m"]
                    .as<float>(0.60F));
            cargo_residual_surface_band_above_m_ = std::max(
                0.0F,
                cargo_safety["residual_surface_band_above_m"]
                    .as<float>(0.20F));
            cargo_safety_console_period_sec_ = std::max(
                0.5,
                cargo_safety["safety_console_period_sec"]
                    .as<double>(2.0));
            cargo_safety_pending_error_sec_ = std::max(
                0.5,
                cargo_safety["safety_pending_error_sec"]
                    .as<double>(1.0));
        }

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parse error: %s", e.what());
        cargo_safety_config_error_ = true;
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
    const bool source_time_rollback =
        !msg->header.stamp.isZero() &&
        !hook_load_snapshot_.source_stamp.isZero() &&
        msg->header.stamp.toSec() + 1.0e-6 <
            hook_load_snapshot_.source_stamp.toSec();
    if (source_time_rollback) {
        // A restarted publisher or replayed bag establishes a new epoch.
        // Origin evidence from the previous epoch must never cross this edge.
        pending_origin_height_valid_ = false;
        pending_origin_height_m_ = 0.0F;
        pending_origin_center_base_.setZero();
        pending_origin_stamp_ = ros::Time();
        empty_hook_height_history_.clear();
        ROS_WARN("[OriginBinding] reset on Gravity source-time rollback "
                 "previous=%.6f current=%.6f",
                 hook_load_snapshot_.source_stamp.toSec(),
                 msg->header.stamp.toSec());
    }
    const bool source_stamp_valid = !msg->header.stamp.isZero() &&
        !source_time_rollback;
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
        std::vector<OriginHeightSample> recent;
        recent.reserve(empty_hook_height_history_.size());
        for (const auto& sample : empty_hook_height_history_) {
            const double age = (msg->header.stamp - sample.stamp).toSec();
            if (std::isfinite(age) && age >= 0.0 &&
                age <= origin_history_max_age_sec_ &&
                std::isfinite(sample.confidence) &&
                sample.confidence >= origin_min_confidence_ &&
                sample.size_xy.allFinite() &&
                sample.size_xy.minCoeff() > 0.0F) {
                recent.push_back(sample);
            }
        }
        if (recent.size() >= 3U) {
            Eigen::Vector2f center_map_mean = Eigen::Vector2f::Zero();
            Eigen::Vector2f center_base_mean = Eigen::Vector2f::Zero();
            for (const auto& sample : recent) {
                center_map_mean += sample.center_map;
                center_base_mean += sample.center_base;
            }
            center_map_mean /= static_cast<float>(recent.size());
            center_base_mean /= static_cast<float>(recent.size());
            float maximum_spread = 0.0F;
            for (const auto& sample : recent) {
                maximum_spread = std::max(
                    maximum_spread,
                    (sample.center_map - center_map_mean).norm());
            }
            std::vector<float> heights;
            heights.reserve(recent.size());
            for (const auto& sample : recent) heights.push_back(sample.height_m);
            const auto middle = heights.begin() +
                static_cast<std::ptrdiff_t>(heights.size() / 2U);
            std::nth_element(heights.begin(), middle, heights.end());
            pending_origin_height_m_ = *middle;
            const auto [height_min, height_max] =
                std::minmax_element(heights.begin(), heights.end());
            std::vector<float> absolute_deviations;
            absolute_deviations.reserve(heights.size());
            for (const float height : heights) {
                absolute_deviations.push_back(
                    std::abs(height - pending_origin_height_m_));
            }
            const auto deviation_middle = absolute_deviations.begin() +
                static_cast<std::ptrdiff_t>(absolute_deviations.size() / 2U);
            std::nth_element(absolute_deviations.begin(), deviation_middle,
                             absolute_deviations.end());
            const float height_mad = *deviation_middle;

            Eigen::Vector2f size_mean = Eigen::Vector2f::Zero();
            for (const auto& sample : recent) size_mean += sample.size_xy;
            size_mean /= static_cast<float>(recent.size());
            float maximum_size_relative_deviation = 0.0F;
            for (const auto& sample : recent) {
                const Eigen::Array2f relative =
                    (sample.size_xy - size_mean).cwiseAbs().array() /
                    size_mean.cwiseMax(
                        Eigen::Vector2f::Constant(0.05F)).array();
                maximum_size_relative_deviation = std::max(
                    maximum_size_relative_deviation, relative.maxCoeff());
            }
            pending_origin_center_base_ = center_base_mean;
            pending_origin_stamp_ = msg->header.stamp;
            pending_origin_height_valid_ =
                maximum_spread <= origin_history_max_position_spread_m_ &&
                height_mad <= origin_height_max_mad_m_ &&
                (*height_max - *height_min) <= origin_height_max_range_m_ &&
                maximum_size_relative_deviation <=
                    origin_size_max_relative_deviation_ &&
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
    if (source_stamp_advanced || source_time_rollback ||
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
        HookLoadSnapshot disabled;
        disabled.valid = false;
        disabled.state = lidar_slam2_msgs::HookLoadState::STATE_UNKNOWN;
        disabled.reason = "hook_signal_disabled";
        return disabled;
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
    return snapshot;
}

bool NdtSlamNode::shouldRemoveHookCargo() const {
    if (!hook_lock_config_.enable_hook_cargo_removal) return false;
    const HookLoadSnapshot hook = currentHookLoadSnapshot();
    const double authorization_age =
        (ros::Time::now() - formal_cargo_removal_stamp_).toSec();
    const bool gravity_permits_removal =
        hook_load_signal_role_ != HookLoadSignalRole::REQUIRED ||
        (hook.valid &&
         hook.state == lidar_slam2_msgs::HookLoadState::STATE_LOADED);
    return gravity_permits_removal &&
           formal_cargo_removal_authorized_ &&
           relocalization_pose_reliable_ &&
           formal_cargo_removal_track_id_ == cargo_fusion_track_id_ &&
           std::isfinite(authorization_age) && authorization_age >= 0.0 &&
           authorization_age <= formal_cargo_removal_max_age_sec_ &&
           (cargo_state_.state == CargoState::LOCKED ||
            cargo_state_.state == CargoState::LOST) &&
           current_rigid_cargo_geometry_.valid &&
           cargo_state_.valid_geometry && cargo_state_.valid_height &&
           cargo_state_.center_base.allFinite() && cargo_state_.size.allFinite() &&
           cargo_state_.size.x() > 0.0F && cargo_state_.size.y() > 0.0F &&
           std::isfinite(cargo_state_.bottom_z) &&
           std::isfinite(cargo_state_.top_z) &&
           cargo_state_.top_z > cargo_state_.bottom_z;
}

bool NdtSlamNode::hookAllowsMapCommit() const {
    const HookLoadSnapshot hook = currentHookLoadSnapshot();
    const HookLoadMapCommitDecision decision = evaluateHookLoadMapCommit({
        hook_load_signal_role_, hook.valid,
        static_cast<HookLoadState>(hook.state), shouldRemoveHookCargo(), false});
    return decision.allow_commit;
}

void NdtSlamNode::recordEmptyHookOriginHeight(
    float height_m, const Eigen::Vector2f& center_base,
    const Eigen::Vector2f& center_map,
    const Eigen::Vector2f& size_xy, float confidence,
    const ros::Time& stamp) {
    if (!std::isfinite(height_m) ||
        !center_base.allFinite() || !center_map.allFinite() ||
        !size_xy.allFinite() || size_xy.minCoeff() <= 0.0F ||
        !std::isfinite(confidence) || confidence < 0.0F || confidence > 1.0F ||
        stamp.isZero() ||
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
    if (!empty_hook_height_history_.empty() &&
        stamp <= empty_hook_height_history_.back().stamp) {
        empty_hook_height_history_.clear();
    }
    empty_hook_height_history_.push_back(
        OriginHeightSample{
            height_m, center_base, center_map, size_xy, confidence, stamp});
    while (empty_hook_height_history_.size() >
           empty_hook_height_history_max_samples_) {
        empty_hook_height_history_.pop_front();
    }
}

void NdtSlamNode::resetCargoForHookState(bool preserve_origin_height) {
    clearHookLock();
    retired_cargo_shape_ = LockedCargoShape{};
    retired_cargo_center_base_.setZero();
    retired_cargo_velocity_base_.setZero();
    retired_cargo_stamp_ = ros::Time();
    retired_cargo_signature_valid_ = false;
    lidar_no_cargo_evidence_.reset("cargo_lifecycle_reset");
    cargo_state_ = CargoState{};
    hook_fixed_cargo_ = HookCargoDetection{};
    hook_fixed_bottom_ = HookCargoBottomEstimate{};
    hook_observation_associated_current_ = false;
    cargo_bottom_fusion_.reset();
    cargo_fusion_track_active_ = false;
    formal_cargo_removal_authorized_ = false;
    formal_cargo_removal_track_id_ = 0U;
    formal_cargo_removal_stamp_ = ros::Time();
    cargo_origin_height_valid_ = false;
    cargo_origin_height_m_ = 0.0F;
    cargo_origin_height_track_id_ = 0U;
    last_cargo_bottom_result_ = CargoBottomResult{};
    last_cargo_safety_result_ = CargoSafetyResult{};
    confirmed_cargo_safety_result_ = CargoSafetyResult{};
    cargo_safety_temporal_filter_.reset();
    cargo_obstacle_tracker_.reset();
    cargo_map_motion_sample_valid_ = false;
    cargo_previous_center_map_.setZero();
    cargo_velocity_map_.setZero();
    cargo_previous_center_stamp_sec_ = 0.0;
    cargo_safety_spatial_mode_ = "RADIAL_FALLBACK";
    cargo_corridor_eligible_clusters_ = 0U;
    cargo_corridor_rejected_clusters_ = 0U;
    cargo_residual_self_clusters_ = 0U;
    cargo_residual_unknown_clusters_ = 0U;
    has_stable_height_ = false;
    stable_height_ = 0.0F;
    if (!preserve_origin_height) {
        std::lock_guard<std::mutex> lock(hook_load_state_mutex_);
        pending_origin_height_valid_ = false;
        pending_origin_height_m_ = 0.0F;
        pending_origin_center_base_.setZero();
        pending_origin_stamp_ = ros::Time();
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

bool NdtSlamNode::localizationInputPending() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return !cloud_queue_.empty();
}

void NdtSlamNode::requestMapMaintenance() {
    bool notify = false;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        ++map_maintenance_commit_count_;
        if (!map_maintenance_has_run_.load(std::memory_order_acquire) ||
            map_maintenance_commit_count_ >=
                map_maintenance_interval_commits_) {
            map_maintenance_commit_count_ = 0;
            map_maintenance_pending_ = true;
            clean_map_rebuild_pending_ = true;
            notify = true;
        }
    }
    if (notify) queue_cv_.notify_one();
}

void NdtSlamNode::requestMapPublication(const ros::Time& stamp) {
    {
        std::lock_guard<std::mutex> lock(map_publication_mutex_);
        ++map_publication_requested_version_;
        if (map_publication_requested_version_ == 0U) {
            ++map_publication_requested_version_;
        }
        map_publication_stamp_ = stamp;
    }
    map_publication_cv_.notify_one();
}

void NdtSlamNode::mapPublicationThread() {
    while (true) {
        std::uint64_t requested_version = 0U;
        ros::Time stamp;
        {
            std::unique_lock<std::mutex> lock(map_publication_mutex_);
            map_publication_cv_.wait(lock, [this]() {
                return map_publication_shutdown_ ||
                    map_publication_requested_version_ >
                        map_publication_completed_version_;
            });
            if (map_publication_shutdown_) return;
            requested_version = map_publication_requested_version_;
            stamp = map_publication_stamp_;
        }

        const auto publish_start = std::chrono::steady_clock::now();
        const MapPublicationSnapshot snapshot =
            captureMapPublicationSnapshot(requested_version, stamp);
        publishMapPublicationSnapshot(snapshot);
        const double publish_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - publish_start).count();
        {
            std::lock_guard<std::mutex> lock(map_publication_mutex_);
            map_publication_completed_version_ = std::max(
                map_publication_completed_version_, requested_version);
        }
        ROS_DEBUG("[MapPublish] request=%llu generation=%llu stamp=%.3f "
                  "duration_ms=%.1f",
                 static_cast<unsigned long long>(requested_version),
                 static_cast<unsigned long long>(snapshot.generation),
                 snapshot.stamp.toSec(), publish_ms);
    }
}

NdtSlamNode::MapPublicationSnapshot
NdtSlamNode::captureMapPublicationSnapshot(
    std::uint64_t version, const ros::Time& requested_stamp) {
    MapPublicationSnapshot snapshot;
    snapshot.request_version = version;
    // All pointers come from one sealed, immutable content generation. The
    // publication worker never combines current raw maps with an older clean
    // layer.
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        snapshot.generation = latest_completed_map_bundle_.generation;
        snapshot.stamp = latest_completed_map_bundle_.source_stamp.isZero()
            ? (requested_stamp.isZero() ? ros::Time::now() : requested_stamp)
            : latest_completed_map_bundle_.source_stamp;
        snapshot.registration = latest_completed_map_bundle_.registration;
        snapshot.display = latest_completed_map_bundle_.display;
        snapshot.ground = latest_completed_map_bundle_.ground;
        snapshot.objects = latest_completed_map_bundle_.objects;
        snapshot.objects_clean =
            latest_completed_map_bundle_.objects_clean;
    }
    return snapshot;
}

void NdtSlamNode::publishMapPublicationSnapshot(
    const MapPublicationSnapshot& snapshot) {
    if (!snapshot.registration || !snapshot.display || !snapshot.ground ||
        !snapshot.objects || !snapshot.objects_clean) {
        ROS_ERROR_THROTTLE(
            5.0, "[MapPublish] incomplete immutable layer bundle");
        return;
    }
    const auto make_message = [this, &snapshot](
        const pcl::PointCloud<pcl::PointXYZ>& cloud) {
        sensor_msgs::PointCloud2 message;
        pcl::toROSMsg(cloud, message);
        message.header.seq = static_cast<std::uint32_t>(
            snapshot.generation & 0xffffffffU);
        message.header.stamp = snapshot.stamp;
        message.header.frame_id = map_frame_;
        return message;
    };
    map_pub_.publish(make_message(*snapshot.registration));
    display_map_pub_.publish(make_message(*snapshot.display));
    ground_map_pub_.publish(make_message(*snapshot.ground));
    objects_map_pub_.publish(make_message(*snapshot.objects));
    objects_clean_map_pub_.publish(make_message(*snapshot.objects_clean));
}

void NdtSlamNode::advanceMapLayerGenerationLocked() {
    ++map_layer_generation_;
    if (map_layer_generation_ == 0U) {
        ++map_layer_generation_;
    }
}

void NdtSlamNode::sealCurrentMapLayerBundleLocked(const ros::Time& stamp) {
    const auto clone = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(
            cloud ? new pcl::PointCloud<pcl::PointXYZ>(*cloud)
                  : new pcl::PointCloud<pcl::PointXYZ>);
    };
    latest_completed_map_bundle_.valid = true;
    latest_completed_map_bundle_.generation = map_layer_generation_;
    latest_completed_map_bundle_.objects_version =
        objects_map_content_version_;
    latest_completed_map_bundle_.lifecycle_epoch =
        map_rebuild_generation_.load(std::memory_order_acquire);
    latest_completed_map_bundle_.source_stamp =
        stamp.isZero() ? ros::Time::now() : stamp;
    latest_completed_map_bundle_.registration = clone(global_map_);
    latest_completed_map_bundle_.display = clone(display_map_);
    latest_completed_map_bundle_.ground = clone(ground_map_);
    latest_completed_map_bundle_.objects = clone(objects_map_);
    latest_completed_map_bundle_.objects_clean = clone(objects_clean_map_);
}

void NdtSlamNode::advanceObjectsMapContentVersionLocked() {
    ++objects_map_content_version_;
    if (objects_map_content_version_ == 0U) {
        ++objects_map_content_version_;
    }
}

void NdtSlamNode::startCleanMapRebuildJob() {
    if (clean_map_rebuild_running_.load(std::memory_order_acquire)) {
        // The in-flight immutable snapshot remains publishable even if a newer
        // objects map exists. Remember the newer raw/deny/protect generation
        // so a follow-up build converges to the current working map.
        clean_map_rebuild_pending_ = true;
        return;
    }
    if (clean_map_rebuild_thread_.joinable()) {
        clean_map_rebuild_thread_.join();
    }

    constexpr float kCleanCellSize = 0.15F;
    const double current_time = ros::Time::now().toSec();
    CleanMapBuildInput input;
    input.cell_size_m = kCleanCellSize;
    std::uint64_t source_objects_version = 0U;
    MapLayerBundle source_bundle;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        const auto clone = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(
                cloud ? new pcl::PointCloud<pcl::PointXYZ>(*cloud)
                      : new pcl::PointCloud<pcl::PointXYZ>);
        };
        source_objects_version = objects_map_content_version_;
        source_bundle.valid = true;
        source_bundle.generation = map_layer_generation_;
        source_bundle.objects_version = source_objects_version;
        source_bundle.lifecycle_epoch =
            map_rebuild_generation_.load(std::memory_order_acquire);
        source_bundle.source_stamp.fromSec(current_time);
        source_bundle.registration = clone(global_map_);
        source_bundle.display = clone(display_map_);
        source_bundle.ground = clone(ground_map_);
        source_bundle.objects = clone(objects_map_);
    }
    if (source_bundle.objects) {
        input.object_points.reserve(source_bundle.objects->size());
        for (const auto& point : source_bundle.objects->points) {
            input.object_points.emplace_back(point.x, point.y, point.z);
        }
    }
    if (rebuild_payload_candidate_) {
        input.payload_candidate_points.reserve(
            rebuild_payload_candidate_->size());
        for (const auto& point : rebuild_payload_candidate_->points) {
            input.payload_candidate_points.emplace_back(
                point.x, point.y, point.z);
        }
    }
    for (const auto& item : bev_observation_count_) {
        input.observation_counts.emplace(
            CleanMapCell{item.first.x, item.first.y}, item.second);
    }

    if (dynamic_event_config_.enabled &&
        dynamic_event_config_.clean_deny_enabled) {
        input.deny_cells = dynamic_event_manager_.getDynamicDenyCells(
            kCleanCellSize, current_time);
        input.protect_cells = dynamic_event_manager_.getStaticProtectCells(
            kCleanCellSize, current_time);
        if (dynamic_event_config_.enable_cargo_history_clean) {
            for (const auto& item : cargo_deny_history_) {
                const double age = current_time - item.second.last_seen_time;
                if (std::isfinite(age) && age >= 0.0 &&
                    age < cargo_deny_ttl_) {
                    input.deny_cells.insert(item.first);
                }
            }
        }
    }

    input.use_human_deny = human_filter_config_.enabled &&
        dynamic_event_config_.enable_human_history_clean;
    if (input.use_human_deny) {
        input.human_deny_cells = human_filter_.getDenyCellsSnapshot(
            kCleanCellSize, current_time);
    }

    input.use_3d_deny = dynamic_event_config_.enabled &&
        !dynamic_deny_volume_map_.empty();
    if (input.use_3d_deny &&
        std::isfinite(dynamic_deny_resolution_) &&
        dynamic_deny_resolution_ > 0.0) {
        for (const auto& item : dynamic_deny_volume_map_) {
            const double x_min = item.first.first * dynamic_deny_resolution_;
            const double y_min = item.first.second * dynamic_deny_resolution_;
            const double x_max = x_min + dynamic_deny_resolution_;
            const double y_max = y_min + dynamic_deny_resolution_;
            const int clean_x_min = static_cast<int>(
                std::floor(x_min / kCleanCellSize));
            const int clean_y_min = static_cast<int>(
                std::floor(y_min / kCleanCellSize));
            const int clean_x_max = static_cast<int>(
                std::floor((x_max - 1e-9) / kCleanCellSize));
            const int clean_y_max = static_cast<int>(
                std::floor((y_max - 1e-9) / kCleanCellSize));
            for (const auto& volume : item.second) {
                const double age = current_time - volume.stamp;
                if (!std::isfinite(age) || age < 0.0 ||
                    age >= dynamic_deny_ttl_) {
                    continue;
                }
                for (int x = clean_x_min; x <= clean_x_max; ++x) {
                    for (int y = clean_y_min; y <= clean_y_max; ++y) {
                        input.deny_ranges[{x, y}].push_back(
                            {volume.z_min, volume.z_max});
                    }
                }
            }
        }
    }

    clean_map_rebuild_running_.store(true, std::memory_order_release);
    clean_map_rebuild_thread_ = std::thread(
        [this, source_objects_version, source_bundle = std::move(source_bundle),
         input = std::move(input)]() mutable {
            CleanMapWorkerResult result;
            result.source_objects_version = source_objects_version;
            result.bundle = std::move(source_bundle);
            const auto started = std::chrono::steady_clock::now();
            try {
                result.build = buildCleanMapFromSnapshot(input);
                result.valid = result.build.valid;
                if (result.valid) {
                    auto clean_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                        new pcl::PointCloud<pcl::PointXYZ>);
                    clean_cloud->reserve(result.build.clean_points.size());
                    for (const Eigen::Vector3f& point :
                         result.build.clean_points) {
                        pcl::PointXYZ pcl_point;
                        pcl_point.x = point.x();
                        pcl_point.y = point.y();
                        pcl_point.z = point.z();
                        clean_cloud->push_back(pcl_point);
                    }
                    clean_cloud->width = static_cast<std::uint32_t>(
                        clean_cloud->size());
                    clean_cloud->height = 1U;
                    clean_cloud->is_dense = false;
                    result.bundle.objects_clean = clean_cloud;
                }
            } catch (const std::exception& error) {
                result.build.reason =
                    std::string("worker_exception:") + error.what();
            } catch (...) {
                result.build.reason = "worker_exception:unknown";
            }
            result.duration_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
            {
                std::lock_guard<std::mutex> lock(
                    clean_map_rebuild_result_mutex_);
                clean_map_worker_result_ = std::move(result);
            }
            clean_map_rebuild_result_ready_.store(
                true, std::memory_order_release);
            clean_map_rebuild_running_.store(
                false, std::memory_order_release);
            queue_cv_.notify_one();
        });
}

void NdtSlamNode::consumeCleanMapRebuildResult(const ros::Time& stamp) {
    if (!clean_map_rebuild_result_ready_.exchange(
            false, std::memory_order_acq_rel)) {
        return;
    }
    CleanMapWorkerResult result;
    {
        std::lock_guard<std::mutex> lock(clean_map_rebuild_result_mutex_);
        result = std::move(clean_map_worker_result_);
    }
    if (result.bundle.lifecycle_epoch !=
        map_rebuild_generation_.load(std::memory_order_acquire)) {
        ROS_DEBUG("[CleanMapWorker] discarded previous lifecycle epoch");
        return;
    }
    std::uint64_t current_objects_version = 0U;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_objects_version = objects_map_content_version_;
    }
    const CleanMapBuildAction initial_action = evaluateCleanMapBuildAction(
        result.valid,
        clean_map_rebuild_pending_.load(std::memory_order_acquire),
        result.source_objects_version, current_objects_version);
    if (initial_action == CleanMapBuildAction::DISCARD_INVALID) {
        ROS_DEBUG("[CleanMapWorker] rejected reason=%s duration_ms=%.1f",
                  result.build.reason.c_str(), result.duration_ms);
        return;
    }

    bool installed_as_current = false;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        const CleanMapBuildAction final_action = evaluateCleanMapBuildAction(
            result.valid, false, result.source_objects_version,
            objects_map_content_version_);
        if (final_action == CleanMapBuildAction::APPLY &&
            result.bundle.objects_clean) {
            objects_clean_map_.reset(
                new pcl::PointCloud<pcl::PointXYZ>(
                    *result.bundle.objects_clean));
            ++clean_map_content_version_;
            if (clean_map_content_version_ == 0U) {
                ++clean_map_content_version_;
            }
            installed_as_current = true;
        } else if (final_action ==
                   CleanMapBuildAction::PUBLISH_SNAPSHOT_ONLY) {
            clean_map_rebuild_pending_ = true;
            map_maintenance_pending_ = true;
        }
        // Publish the completed raw+clean snapshot even when the working map
        // advanced during the build. This removes clean-worker starvation
        // without ever installing stale clean content into the working map.
        advanceMapLayerGenerationLocked();
        result.bundle.generation = map_layer_generation_;
        result.bundle.valid = true;
        latest_completed_map_bundle_ = result.bundle;
    }

    last_commit_clean_map_ms_ = result.duration_ms;
    map_maintenance_has_run_.store(true, std::memory_order_release);
    shadow_target_pending_ = shadow_target_pending_ || installed_as_current;
    map_maintenance_pending_ = true;
    requestMapPublication(
        result.bundle.source_stamp.isZero()
            ? stamp : result.bundle.source_stamp);
    ROS_DEBUG("[CleanMapWorker] source=%llu points=%zu cells=%d/%d "
              "denied=%d protected=%d installed_current=%d duration_ms=%.1f",
              static_cast<unsigned long long>(
                  result.source_objects_version),
              result.build.clean_points.size(), result.build.passed_cells,
              result.build.total_cells, result.build.denied_cells,
              result.build.protected_cells,
              installed_as_current ? 1 : 0, result.duration_ms);
}

void NdtSlamNode::runMapMaintenanceIfIdle(bool force_timeslice) {
    consumeCleanMapRebuildResult(last_stamp_);
    if (clean_rebuild_requested_from_worker_.exchange(
            false, std::memory_order_acq_rel)) {
        map_maintenance_pending_ = true;
        clean_map_rebuild_pending_ = true;
        shadow_target_pending_ = true;
    }
    if (!map_maintenance_pending_ ||
        (!force_timeslice && localizationInputPending())) return;

    map_maintenance_pending_ = false;
    const bool clean_rebuild_requested = clean_map_rebuild_pending_;
    if (clean_map_rebuild_pending_) {
        clean_map_rebuild_pending_ = false;
        startCleanMapRebuildJob();
    }

    // A forced timeslice completes exactly one expensive reconstruction phase.
    // Remaining stateful owner-thread tasks stay pending for the next bounded
    // timeslice; map serialization has already moved to its own worker.
    if (clean_rebuild_requested && localizationInputPending()) {
        if (release_keyframes_pending_ || flush_tiles_pending_ ||
            runtime_status_pending_ || memory_guard_pending_ ||
            active_map_rebuild_pending_.load(std::memory_order_acquire) ||
            loop_closure_pending_) {
            map_maintenance_pending_ = true;
        }
        return;
    }

    if (localization_target_enabled_ && shadow_target_pending_ &&
        clean_map_content_version_ != shadow_target_source_version_) {
        if (updateLocalizationTarget(objects_clean_map_, shadow_target_pose_)) {
            swapLocalizationTargetBuffers();
            shadow_target_source_version_ = clean_map_content_version_;
            shadow_target_pending_ = false;
            if (force_timeslice && localizationInputPending()) {
                map_maintenance_pending_ = true;
                return;
            }
        } else {
            map_maintenance_pending_ = true;
            return;
        }
    }

    if (loop_closure_enabled_ && loop_closure_pending_) {
        if (!force_timeslice && localizationInputPending()) {
            map_maintenance_pending_ = true;
            return;
        }
        loop_closure_pending_ = false;
        processLoopClosure();
        if (force_timeslice && localizationInputPending()) {
            map_maintenance_pending_ = true;
            return;
        }
    } else if (!loop_closure_enabled_) {
        loop_closure_pending_ = false;
    }

    const auto yield_if_localization_waiting =
        [this, force_timeslice]() {
        if (!localizationInputPending() || force_timeslice) return false;
        map_maintenance_pending_ = true;
        return true;
    };
    const auto finish_forced_phase = [this, force_timeslice]() {
        if (!force_timeslice || !localizationInputPending()) return false;
        map_maintenance_pending_ = true;
        return true;
    };
    if (yield_if_localization_waiting()) return;
    if (release_keyframes_pending_) {
        release_keyframes_pending_ = false;
        releaseOldKeyframeClouds();
        if (finish_forced_phase()) return;
    }
    if (yield_if_localization_waiting()) return;
    if (flush_tiles_pending_) {
        flush_tiles_pending_ = false;
        flushDirtyTiles();
        if (finish_forced_phase()) return;
    }
    if (yield_if_localization_waiting()) return;
    if (runtime_status_pending_) {
        runtime_status_pending_ = false;
        writeRuntimeStatus();
        if (finish_forced_phase()) return;
    }
    if (yield_if_localization_waiting()) return;
    if (memory_guard_pending_) {
        memory_guard_pending_ = false;
        checkMemoryGuard();
        if (finish_forced_phase()) return;
    }
    if (yield_if_localization_waiting()) return;
    if (active_map_rebuild_pending_.exchange(
            false, std::memory_order_acq_rel)) {
        rebuildActiveMapFromRecentKeyframes();
        if (finish_forced_phase()) return;
    }

    ROS_DEBUG("[MapMaintenance] clean_ms=%.1f publish_ms=%.1f clean_points=%zu",
              last_commit_clean_map_ms_, last_commit_display_map_ms_,
              objects_clean_map_ ? objects_clean_map_->size() : 0U);
}

void NdtSlamNode::processCloudThread() {
    ROS_DEBUG("Processing thread started");

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
        bool run_map_maintenance = false;
        bool force_map_maintenance_timeslice = false;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]() {
                return !cloud_queue_.empty() || map_maintenance_pending_ ||
                       clean_rebuild_requested_from_worker_.load(
                           std::memory_order_acquire) ||
                       clean_map_rebuild_result_ready_.load(
                           std::memory_order_acquire) ||
                       loop_closure_result_ready_.load(
                           std::memory_order_acquire) || shutdown_;
            });
            if (shutdown_) break;
            const bool owner_task_pending = map_maintenance_pending_ ||
                    clean_rebuild_requested_from_worker_.load(
                        std::memory_order_acquire) ||
                    clean_map_rebuild_result_ready_.load(
                        std::memory_order_acquire) ||
                    loop_closure_result_ready_.load(
                        std::memory_order_acquire);
            force_map_maintenance_timeslice = owner_task_pending &&
                !cloud_queue_.empty() &&
                map_maintenance_deferral_frames_ >=
                    map_maintenance_max_deferral_frames_;
            if (cloud_queue_.empty() || force_map_maintenance_timeslice) {
                run_map_maintenance = owner_task_pending;
                if (run_map_maintenance) {
                    map_maintenance_deferral_frames_ = 0;
                }
            } else {
                CloudQueueEntry entry = std::move(cloud_queue_.front());
                cloud_queue_.pop_front();
                msg = std::move(entry.message);
                diag_queue_age_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - entry.enqueued_at).count();
                if (owner_task_pending) {
                    ++map_maintenance_deferral_frames_;
                } else {
                    map_maintenance_deferral_frames_ = 0;
                }
            }
        }

        std::unique_lock<std::mutex> runtime_state_lock(
            runtime_state_mutex_);
        consumeMapCommitCompletion();
        if (run_map_maintenance) {
            consumeLoopClosureResult(last_stamp_);
            consumeCleanMapRebuildResult(last_stamp_);
            runMapMaintenanceIfIdle(force_map_maintenance_timeslice);
            continue;
        }

        if (!msg) continue;
        consumeLoopClosureResult(msg->header.stamp);
        consumeCleanMapRebuildResult(msg->header.stamp);
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
            const bool lidar_time_rollback =
                !msg->header.stamp.isZero() &&
                msg->header.stamp.toSec() + 1.0e-6 <
                    last_processed_frame_stamp_.toSec();
            if (lidar_time_rollback) {
                handleLidarTimeRollback(
                    last_processed_frame_stamp_, msg->header.stamp);
            }
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
        const bool hook_detection_due =
            odom_anchor_config_.enabled &&
            shouldRunOdomAnchorDetect(msg->header.stamp);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr hook_input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& p : input_cloud->points) {
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                double range = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
                if (range > 0.5 && range < 30.0) {
                    filtered_cloud->push_back(p);
                    if (hook_detection_due) hook_input_cloud->push_back(p);
                }
            }
        }

        // Keep the due-frame input unvoxelized here. The detector crops its ROI
        // first and performs the single 0.05 m voxel pass on the much smaller ROI.

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
        if (hook_load_signal_role_ == HookLoadSignalRole::REQUIRED &&
            hook_load.state != last_processed_hook_load_state_) {
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
        const bool hook_allows_tracking =
            hook_load_signal_role_ != HookLoadSignalRole::REQUIRED ||
            (hook_load.valid && hook_load.state ==
                lidar_slam2_msgs::HookLoadState::STATE_LOADED);
        const bool hook_is_empty = hook_load.valid &&
            hook_load.state ==
                lidar_slam2_msgs::HookLoadState::STATE_EMPTY;
        const bool localization_evidence_valid =
            relocalization_pose_reliable_ &&
            current_pose_.translation().allFinite() &&
            current_pose_.so3().matrix().allFinite();
        if (!localization_evidence_valid) {
            lidar_no_cargo_evidence_.reset("localization_invalid");
        }

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
        if (!skip_hook_this_frame && hook_detection_due &&
            hook_input_cloud && !hook_input_cloud->empty()) {
            hook_fixed_cargo_ = detectCargoAroundOdomAnchor(hook_input_cloud, msg->header.stamp);
                hook_fixed_bottom_ = estimateCargoBottom(hook_fixed_cargo_);
                hook_fixed_cargo_.lidar_lift_evidence =
                    hook_fixed_cargo_.valid &&
                    hook_fixed_cargo_.ground_reference_valid &&
                    hook_fixed_bottom_.valid &&
                    hook_fixed_bottom_.source == "points_visible_side" &&
                    std::isfinite(hook_fixed_bottom_.bottom_z_base) &&
                    std::isfinite(hook_fixed_cargo_.ground_z) &&
                    hook_fixed_bottom_.bottom_z_base -
                            hook_fixed_cargo_.ground_z >=
                        hook_lock_config_.suspended_min_ground_clearance_m;
                if (hook_is_empty && localization_evidence_valid &&
                    !hook_fixed_cargo_.lidar_lift_evidence &&
                    hook_fixed_bottom_.valid &&
                    hook_fixed_cargo_.core_points_base &&
                    hook_fixed_bottom_.source == "points_visible_side") {
                        const Eigen::Vector3d origin_center_map =
                            current_pose_ *
                            hook_fixed_cargo_.center_base.cast<double>();
                        recordEmptyHookOriginHeight(
                            hook_fixed_bottom_.height,
                            hook_fixed_cargo_.center_base.head<2>(),
                            origin_center_map.head<2>().cast<float>(),
                            hook_fixed_cargo_.size_visible.head<2>(),
                            std::min(
                                1.0F,
                                static_cast<float>(
                                    hook_fixed_cargo_.core_points_base->size()) /
                                    static_cast<float>(std::max(
                                        1, hook_lock_config_.lock_strong_min_points))),
                            msg->header.stamp);
                }
                if (hook_allows_tracking) {
                    updateHookCargoLock(
                        hook_fixed_cargo_, hook_fixed_bottom_,
                        msg->header.stamp);
                } else {
                    hook_observation_associated_current_ = false;
                    hook_observation_association_stamp_ = msg->header.stamp;
                }
                lidar_no_cargo_evidence_.update({
                    true, hook_fixed_cargo_.outcome,
                    localization_evidence_valid,
                    hook_lock_.state != HookCargoLockState::EMPTY,
                    msg->header.stamp.toSec()});

                // 发布 selected_core_points（默认关闭）
                if (odom_anchor_config_.publish_selected_core_points && hook_fixed_cargo_.valid) {
                    publishSelectedCorePoints(hook_fixed_cargo_, msg->header.stamp);
                }
                const auto publish_cargo_debug_cloud = [this, &msg](
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                    ros::Publisher& publisher) {
                    if (publisher.getNumSubscribers() == 0U) return;
                    sensor_msgs::PointCloud2 debug_message;
                    if (cloud && !cloud->empty()) {
                        pcl::toROSMsg(*cloud, debug_message);
                    }
                    debug_message.header.stamp = msg->header.stamp;
                    debug_message.header.frame_id = "base_link";
                    publisher.publish(debug_message);
                };
                publish_cargo_debug_cloud(
                    hook_fixed_cargo_.candidate_components_base,
                    cargo_candidate_components_pub_);
                publish_cargo_debug_cloud(
                    hook_fixed_cargo_.core_points_base,
                    cargo_selected_candidate_pub_);

                visualization_msgs::Marker provisional_marker;
                provisional_marker.header.stamp = msg->header.stamp;
                provisional_marker.header.frame_id = "base_link";
                provisional_marker.ns = "cargo_provisional_obb";
                provisional_marker.id = 1;
                provisional_marker.type = visualization_msgs::Marker::CUBE;
                provisional_marker.action =
                    hook_lock_.state ==
                            HookCargoLockState::GEOMETRY_CONFIRMING &&
                        hook_fixed_cargo_.valid &&
                        hook_fixed_cargo_.oriented_footprint_valid
                    ? visualization_msgs::Marker::ADD
                    : visualization_msgs::Marker::DELETE;
                provisional_marker.pose.orientation.w = 1.0;
                if (provisional_marker.action ==
                    visualization_msgs::Marker::ADD) {
                    provisional_marker.pose.position.x =
                        hook_fixed_cargo_.footprint_center_base.x();
                    provisional_marker.pose.position.y =
                        hook_fixed_cargo_.footprint_center_base.y();
                    provisional_marker.pose.position.z =
                        hook_fixed_cargo_.center_base.z();
                    const float half_yaw =
                        0.5F * hook_fixed_cargo_.footprint_yaw_base_rad;
                    provisional_marker.pose.orientation.z =
                        std::sin(half_yaw);
                    provisional_marker.pose.orientation.w =
                        std::cos(half_yaw);
                    provisional_marker.scale.x =
                        hook_fixed_cargo_.footprint_length_width.x();
                    provisional_marker.scale.y =
                        hook_fixed_cargo_.footprint_length_width.y();
                    provisional_marker.scale.z = std::max(
                        0.05F, hook_fixed_cargo_.visible_height);
                    provisional_marker.color.r = 1.0F;
                    provisional_marker.color.g = 0.75F;
                    provisional_marker.color.b = 0.0F;
                    provisional_marker.color.a = 0.45F;
                }
                cargo_predicted_obb_pub_.publish(provisional_marker);

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
        } else if (!skip_hook_this_frame && hook_detection_due) {
            // Detection ran, but an empty/invalid input has no explicit
            // negative evidence and therefore remains UNKNOWN.
            last_anchor_detect_stamp_ = msg->header.stamp;
            hook_fixed_cargo_ = HookCargoDetection{};
            hook_fixed_bottom_ = HookCargoBottomEstimate{};
            if (hook_allows_tracking) {
                updateHookCargoLock(
                    hook_fixed_cargo_, hook_fixed_bottom_, msg->header.stamp);
            } else {
                hook_observation_associated_current_ = false;
                hook_observation_association_stamp_ = msg->header.stamp;
            }
            lidar_no_cargo_evidence_.update({
                true, CargoObservationOutcome::UNKNOWN,
                localization_evidence_valid,
                hook_lock_.state != HookCargoLockState::EMPTY,
                msg->header.stamp.toSec()});
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

        // Human/dynamic classification must see both channel-safe points and
        // uncertain payload candidates. Channel classification alone cannot
        // allow a person in the work channel to bypass the human filter.
        pcl::PointCloud<pcl::PointXYZ>::Ptr human_filter_input(
            new pcl::PointCloud<pcl::PointXYZ>(*safe_objects));
        if (payload_candidates && !payload_candidates->empty()) {
            *human_filter_input += *payload_candidates;
        }

        if (human_filter_config_.enabled) {
            // 获取 T_map_base（当前位姿）
            Eigen::Matrix4d T_map_base = current_pose_.matrix();

            // 获取时间戳
            double timestamp = msg->header.stamp.toSec();

            // 处理人体过滤
            human_filter_.processFrame(human_filter_input, T_map_base, timestamp,
                                       human_safe_objects, human_candidates,
                                       human_dynamic, human_pending);

            // 发布调试话题（每 10 帧一次）
            static int hf_debug_count = 0;
            hf_debug_count++;
            if (hf_debug_count % 10 == 1) {
                ROS_DEBUG("[HumanFilter] input=%lu, safe=%lu, candidate=%lu, dynamic=%lu, pending=%lu",
                         human_filter_input->size(), human_safe_objects->size(),
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
            // 人体过滤未启用时仍保留完整的通道输入；后续会恢复通道
            // 分区，以确保不确定候选只按低权重进入 NDT。
            human_safe_objects = human_filter_input;
        }

        // v8-stable-r3: 构建配准用点云（NDT 输入减负）
        // 使用 buildRegistrationCloud 替代原来的 objects x4 + ground full
        diag_stage.human_filter_ms = elapsedMs(human_filter_start);
        const auto registration_build_start = DiagClock::now();
        // Restore the channel partition only after human filtering. Static
        // objects retain normal object weighting; surviving uncertain cargo
        // candidates enter exactly once and are never silently removed before
        // formal fused-cargo authorization.
        const RegistrationObjectPartition registration_partition =
            partitionRegistrationObjects(
                *human_safe_objects, *payload_candidates);
        if (payload_candidates && !payload_candidates->empty()) {
            channel_candidate_points_.fetch_add(
                payload_candidates->size(), std::memory_order_relaxed);
            candidate_human_filtered_points_.fetch_add(
                registration_partition.candidate_human_filtered_points,
                std::memory_order_relaxed);
            if (!shouldRemoveHookCargo()) {
                candidate_kept_before_auth_.fetch_add(
                    registration_partition.candidate_survivor_points,
                    std::memory_order_relaxed);
            }
        }
        RegistrationCloudBuildResult registration_build_result =
            buildRegistrationCloud(
                registration_partition.static_objects, ground_cloud,
                registration_partition.uncertain_candidates);
        pcl::PointCloud<pcl::PointXYZ>::Ptr registration_cloud =
            registration_build_result.cloud;

        // ========== Hook locked box 剔除（NDT 输入）==========
        if (shouldRemoveHookCargo()) {
            size_t hook_removed_count = 0;
            const CargoObbFootprint removal_footprint =
                toCargoObbFootprint(current_rigid_cargo_geometry_);
            registration_build_result =
                excludeCargoObbFromRegistrationCloud(
                    registration_build_result, removal_footprint,
                    0.10F, 0.10F, registration_cloud_config_,
                    &hook_removed_count);
            registration_cloud = registration_build_result.cloud;
            if (hook_removed_count > 0U) {
                formal_box_removed_points_.fetch_add(
                    hook_removed_count, std::memory_order_relaxed);
                if (debug_cfg_.debug_registration_removal) {
                    ROS_INFO_THROTTLE(
                        debug_cfg_.summary_interval_sec,
                        "[RegistrationCargoRemoval] removed=%zu after=%zu "
                        "mode=%s reason=%s",
                        hook_removed_count, registration_cloud->size(),
                        registration_build_result.mode.c_str(),
                        registration_build_result.reason.c_str());
                }
            }
        } else if (debug_cfg_.debug_hook_removal) {
            ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec,
                "[HookCargoRemoval] enabled=0 reason=safety_gate_closed");
        }

        last_registration_build_result_ = registration_build_result;
        last_source_points_ =
            static_cast<int>(registration_build_result.total_points);
        const bool registration_mode_changed =
            registration_build_result.mode != last_registration_console_mode_;
        if (registration_mode_changed) {
            if (registration_build_result.valid) {
                ROS_DEBUG("[RegistrationInput] mode=%s valid=1 static=%zu uncertain=%zu ground=%zu total=%zu ground_fraction=%.3f reason=%s",
                         registration_build_result.mode.c_str(),
                         registration_build_result.static_object_points,
                         registration_build_result.uncertain_candidate_points,
                         registration_build_result.ground_points,
                         registration_build_result.total_points,
                         registration_build_result.ground_fraction,
                         registration_build_result.reason.c_str());
            } else {
                ROS_DEBUG("[RegistrationInput] mode=%s valid=0 static=%zu uncertain=%zu ground=%zu total=%zu ground_fraction=%.3f reason=%s",
                         registration_build_result.mode.c_str(),
                         registration_build_result.static_object_points,
                         registration_build_result.uncertain_candidate_points,
                         registration_build_result.ground_points,
                         registration_build_result.total_points,
                         registration_build_result.ground_fraction,
                         registration_build_result.reason.c_str());
            }
            last_registration_console_mode_ =
                registration_build_result.mode;
        } else {
            ROS_DEBUG_THROTTLE(
                5.0,
                "[RegistrationInput] mode=%s valid=%d static=%zu uncertain=%zu ground=%zu total=%zu ground_fraction=%.3f reason=%s",
                registration_build_result.mode.c_str(),
                registration_build_result.valid ? 1 : 0,
                registration_build_result.static_object_points,
                registration_build_result.uncertain_candidate_points,
                registration_build_result.ground_points,
                registration_build_result.total_points,
                registration_build_result.ground_fraction,
                registration_build_result.reason.c_str());
        }

        std::vector<Eigen::Vector2d> observability_structure_points;
        observability_structure_points.reserve(
            registration_build_result.structure_cloud->size());
        for (const auto& point :
             registration_build_result.structure_cloud->points) {
            if (std::isfinite(point.x) && std::isfinite(point.y)) {
                observability_structure_points.emplace_back(
                    point.x, point.y);
            }
        }
        NdtObservability frame_ndt_observability =
            estimateNdtObservabilityFromStructure(
                observability_structure_points,
                ndt_observability_config_);
        // The structure proxy is estimated in the registration source/base
        // frame, while the EKF state and NDT translation measurement live in
        // the map frame. Rotate with the same predicted pose used to seed NDT.
        Sophus::SE3d observability_frame_pose = current_pose_;
        if (crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized()) {
            observability_frame_pose = crane_motion_ekf_.predictPoseReadOnly(
                current_pose_, last_sensor_dt_);
        }
        const Eigen::Matrix3d observability_rotation =
            observability_frame_pose.so3().matrix();
        const double observability_yaw = std::atan2(
            observability_rotation(1, 0), observability_rotation(0, 0));
        frame_ndt_observability = rotateNdtObservability(
            frame_ndt_observability, observability_yaw);
        if (!ndt_observability_config_.enabled) {
            frame_ndt_observability.degenerate = false;
            frame_ndt_observability.severely_degenerate = false;
            frame_ndt_observability.reason = "disabled";
        }
        last_ndt_observability_ = frame_ndt_observability;
        static std::string last_observability_reason;
        if (frame_ndt_observability.reason != last_observability_reason) {
            ROS_DEBUG("[NDTObservability] valid=%d degenerate=%d severe=%d ratio=%.4f strong=%.3f weak=%.3f weak_dir=(%.3f,%.3f) normals=%zu reason=%s proxy=static_local_xy_normals",
                     frame_ndt_observability.valid ? 1 : 0,
                     frame_ndt_observability.degenerate ? 1 : 0,
                     frame_ndt_observability.severely_degenerate ? 1 : 0,
                     frame_ndt_observability.eigenvalue_ratio,
                     frame_ndt_observability.strong_eigenvalue,
                     frame_ndt_observability.weak_eigenvalue,
                     frame_ndt_observability.weak_direction.x(),
                     frame_ndt_observability.weak_direction.y(),
                     frame_ndt_observability.local_normals,
                     frame_ndt_observability.reason.c_str());
            last_observability_reason = frame_ndt_observability.reason;
        } else {
            ROS_DEBUG_THROTTLE(
                5.0,
                "[NDTObservability] valid=%d degenerate=%d severe=%d ratio=%.4f normals=%zu reason=%s proxy=static_local_xy_normals",
                frame_ndt_observability.valid ? 1 : 0,
                frame_ndt_observability.degenerate ? 1 : 0,
                frame_ndt_observability.severely_degenerate ? 1 : 0,
                frame_ndt_observability.eigenvalue_ratio,
                frame_ndt_observability.local_normals,
                frame_ndt_observability.reason.c_str());
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
            ROS_DEBUG("SLAM initialized: total=%lu, feature=%lu, ground=%lu, reg=%lu",
                     filtered_cloud->size(), feature_cloud->size(), ground_cloud->size(), registration_cloud->size());
        }

        // ========== 阶段 5：NDT_OMP 配准 ==========
        // Apply only a twice-confirmed asynchronous recovery result before the
        // current NDT prediction so this frame immediately refines it.
        consumeRelocalizationResult(processing_frame_index, msg->header.stamp);

        Sophus::SE3d new_pose = current_pose_;
        bool registration_success = false;
        bool ndt_attempted_this_frame = false;
        std::string ndt_execution_state = "NDT_NOT_ATTEMPTED";
        bool ndt_safe_pose_valid_this_frame = false;
        bool frame_ndt_accepted = false;
        bool frame_prediction_only = true;
        bool frame_registration_quality_valid = false;
        bool frame_severe_degeneracy =
            frame_ndt_observability.severely_degenerate;
        Sophus::SE3d frame_raw_ndt_pose = current_pose_;
        double frame_raw_increment_m = 0.0;
        static Sophus::SE3d last_local_map_pose = Sophus::SE3d();
        static int frames_since_last_update = 0;

        try {
            const bool observability_unavailable =
                ndt_observability_config_.enabled &&
                !frame_ndt_observability.valid;
            const bool bootstrap_quality_invalid =
                !bootstrap_local_map_complete_ &&
                ndt_observability_config_.enabled &&
                frame_ndt_observability.severely_degenerate;
            if (!registration_build_result.valid ||
                observability_unavailable || bootstrap_quality_invalid) {
                ndt_execution_state = observability_unavailable ||
                        bootstrap_quality_invalid
                    ? "NDT_SKIPPED_INSUFFICIENT_OBSERVABILITY"
                    : "NDT_SKIPPED_INSUFFICIENT_STRUCTURE";
                last_ndt_converged_ = false;
                last_ndt_iterations_ = 0;
                last_ndt_fitness_ =
                    std::numeric_limits<double>::infinity();
                last_raw_step_ = 0.0;
                last_source_points_ =
                    static_cast<int>(registration_cloud->size());
                if (crane_motion_ekf_enabled_ &&
                    crane_motion_ekf_.initialized()) {
                    const auto ekf_start = DiagClock::now();
                    new_pose = crane_motion_ekf_.predictWithoutMeasurement(
                        current_pose_, msg->header.stamp,
                        (observability_unavailable || bootstrap_quality_invalid)
                            ? "INSUFFICIENT_OBSERVABILITY"
                            : "INSUFFICIENT_STRUCTURE");
                    registration_success = true;
                    ekf_reject_count_.fetch_add(
                        1, std::memory_order_relaxed);
                    diag_stage.ekf_ms += elapsedMs(ekf_start);
                    diag_ekf_pose = new_pose;
                    logSO3GuardPose(
                        "ekf_pose", processing_frame_index, new_pose);
                }
            } else if (!bootstrap_local_map_complete_) {
                ndt_execution_state = "NDT_SKIPPED_BOOTSTRAP";
                last_ndt_converged_ = false;
                last_ndt_iterations_ = 0;
                last_ndt_fitness_ =
                    std::numeric_limits<double>::infinity();
                // One-shot bootstrap. Only validated, observable registration
                // source frames reach this branch. Once complete, a later map
                // shrink cannot silently reopen this write bypass.
                *local_map_ += *registration_cloud;
                ++local_map_version_;
                ++bootstrap_local_map_frames_;
                registration_success = true;

                const std::size_t bootstrap_min_points = std::max<std::size_t>(
                    500U, registration_cloud_config_.min_registration_points);
                if (local_map_->size() >= bootstrap_min_points) {
                    bootstrap_local_map_complete_ = true;
                    localization_target_state_ =
                        LocalizationTargetState::BUILDING_TARGET;
                    ROS_DEBUG("Local map bootstrap complete: points=%lu frames=%d",
                              local_map_->size(), bootstrap_local_map_frames_);
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
                    diag_initial_guess_pose = predicted;
                } else {
                    initial_guess = current_pose_.matrix().cast<float>();
                    diag_initial_guess_pose = current_pose_;
                }
                diag_have_initial_guess = true;

                last_source_points_ = static_cast<int>(registration_cloud->size());

                // 计算 initial_guess 到 current_pose_ 的距离（用于诊断）
                Eigen::Vector3f initial_pos = initial_guess.block<3,1>(0,3);
                Eigen::Vector3f current_pos = current_pose_.matrix().cast<float>().block<3,1>(0,3);
                last_init_dist_ = (initial_pos - current_pos).norm();

                auto ndt_start = std::chrono::steady_clock::now();
                ndt_attempt_count_.fetch_add(1, std::memory_order_relaxed);
                ndt_attempted_this_frame = true;
                ndt_execution_state = "NDT_ATTEMPTED";
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
                        // PCL registration output is approximate. Never pass
                        // its matrix to Sophus' asserting matrix constructor.
                        const Eigen::Matrix4d ndt_matrix = result.cast<double>();
                        logSO3GuardStage(
                            "raw_ndt_matrix", processing_frame_index,
                            ndt_matrix.block<3, 3>(0, 0),
                            ndt_matrix.allFinite(), ndt_matrix.allFinite(),
                            "external_ndt_matrix");
                        const SafeSE3Result converted =
                            makeSafeSE3FromMatrix(ndt_matrix);
                        if (!converted.valid) {
                            logSO3GuardStage(
                                "ndt_safe_se3", processing_frame_index,
                                ndt_matrix.block<3, 3>(0, 0),
                                ndt_matrix.allFinite(), false,
                                converted.diagnostics.reason);
                            ROS_ERROR_THROTTLE(
                                1.0,
                                "[SO3Guard] rejected NDT measurement frame=%llu "
                                "reason=%s input_orth=%.17g input_det=%.17g "
                                "projected_orth=%.17g projected_det=%.17g",
                                static_cast<unsigned long long>(
                                    processing_frame_index),
                                converted.diagnostics.reason.c_str(),
                                converted.diagnostics.input_orthogonality_error,
                                converted.diagnostics.input_determinant,
                                converted.diagnostics.projected_orthogonality_error,
                                converted.diagnostics.projected_determinant);
                            if (crane_motion_ekf_enabled_ &&
                                crane_motion_ekf_.initialized()) {
                                const auto ekf_start = DiagClock::now();
                                new_pose =
                                    crane_motion_ekf_.predictWithoutMeasurement(
                                        current_pose_, msg->header.stamp,
                                        "NDT_SAFE_SE3_REJECTED");
                                registration_success = true;
                                ekf_reject_count_.fetch_add(
                                    1, std::memory_order_relaxed);
                                diag_stage.ekf_ms += elapsedMs(ekf_start);
                                diag_ekf_pose = new_pose;
                                logSO3GuardPose(
                                    "ekf_pose", processing_frame_index,
                                    new_pose);
                            }
                        } else {
                        ndt_safe_pose_valid_this_frame = true;
                        logSO3GuardPose(
                            "projected_ndt_rotation", processing_frame_index,
                            converted.pose);
                        new_pose = converted.pose;
                        logSO3GuardPose(
                            "ndt_safe_se3", processing_frame_index, new_pose);
                        diag_raw_ndt_pose = new_pose;
                        diag_have_raw_ndt_pose = true;
                        frame_raw_ndt_pose = new_pose;
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
                                    ndt_time_ms,
                                    &frame_ndt_observability);
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
                                    "lat=%.3f tan=%.3f R=%.4f obs_ratio=%.4f weak_inflate=%.1f P=%.4f mode=%s accept=%d predict=%d limited=%d reject=%s",
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
                                    ekf_status.observability_ratio,
                                    ekf_status.weak_variance_inflation,
                                    ekf_status.p_trace,
                                    ekf_status.diagonal_mode ? "DIAG" : "NORMAL",
                                    ekf_status.ndt_accepted ? 1 : 0,
                                    ekf_status.prediction_only ? 1 : 0,
                                    ekf_status.step_limited ? 1 : 0,
                                    ekf_status.reject_reason.c_str());
                            }
                        }

                        frame_ndt_accepted = ndt_accepted;
                        frame_prediction_only = crane_motion_ekf_enabled_
                            ? crane_motion_ekf_.status().prediction_only
                            : !ndt_accepted;
                        frame_registration_quality_valid =
                            ndt_accepted && std::isfinite(fitness_score) &&
                            fitness_score <= map_commit_max_fitness_ &&
                            registration_build_result.structure_quality_valid &&
                            (!ndt_observability_config_.enabled ||
                             frame_ndt_observability.valid);
                        frame_raw_increment_m =
                            diag_raw_ndt_step_from_previous;

                        // v8-stable-r3: 使用 EKF 输出作为 new_pose
                        diag_stage.ekf_ms += elapsedMs(ekf_start);
                        new_pose = ekf_pose;
                        diag_ekf_pose = ekf_pose;
                        logSO3GuardPose(
                            "ekf_pose", processing_frame_index, ekf_pose);

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

                        }
                    } else if (crane_motion_ekf_enabled_ &&
                               crane_motion_ekf_.initialized()) {
                        const auto ekf_start = DiagClock::now();
                        const Eigen::Matrix4d invalid_ndt_matrix =
                            result.cast<double>();
                        logSO3GuardStage(
                            "raw_ndt_matrix", processing_frame_index,
                            invalid_ndt_matrix.block<3, 3>(0, 0),
                            invalid_ndt_matrix.allFinite(), false,
                            "zero_or_nan_ndt_transform");
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
                        logSO3GuardPose(
                            "ekf_pose", processing_frame_index, new_pose);
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
                        logSO3GuardPose(
                            "ekf_pose", processing_frame_index, new_pose);
                    }
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("NDT_OMP exception: %s", e.what());
        }

        if (ndt_attempted_this_frame) {
            ndt_execution_state = last_ndt_converged_
                ? "NDT_ATTEMPTED_CONVERGED"
                : "NDT_ATTEMPTED_NOT_CONVERGED";
        }

        StationaryMotionInput motion_input;
        motion_input.stamp_sec = msg->header.stamp.toSec();
        motion_input.ndt_converged = last_ndt_converged_;
        motion_input.ndt_accepted = frame_ndt_accepted;
        motion_input.prediction_only = frame_prediction_only;
        motion_input.registration_quality_valid =
            frame_registration_quality_valid;
        motion_input.severe_degeneracy = frame_severe_degeneracy;
        motion_input.raw_position = frame_raw_ndt_pose.translation().head<2>();
        motion_input.filtered_position = new_pose.translation().head<2>();
        motion_input.filtered_velocity =
            crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized()
                ? crane_motion_ekf_.status().velocity
                : Eigen::Vector2d::Zero();
        motion_input.raw_increment_m = frame_raw_increment_m;
        motion_input.allowed_physical_step_m =
            crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized()
                ? std::max(1.0e-6,
                           crane_motion_ekf_.status().max_allowed_step)
                : std::max(1.0e-6,
                           crane_motion_ekf_cfg_.max_step_max_m);

        if (motion_gate_enabled_ && registration_success) {
            stationary_motion_decision_ = updateStationaryMotionState(
                motion_input, new_pose, new_pose);
        } else {
            stationary_motion_decision_ = StationaryMotionDecision{};
            stationary_motion_decision_.state = RuntimeMotionState::MOVING;
            stationary_motion_decision_.allow_local_map_update =
                frame_ndt_accepted && frame_registration_quality_valid;
            stationary_motion_decision_.allow_persistent_map_commit =
                stationary_motion_decision_.allow_local_map_update;
            stationary_motion_decision_.constrained_position =
                new_pose.translation().head<2>();
            stationary_motion_decision_.reason = motion_gate_enabled_
                ? "NO_RUNTIME_POSE"
                : "POLICY_DISABLED";
            allow_runtime_local_map_update_ =
                stationary_motion_decision_.allow_local_map_update;
            allow_persistent_map_commit_ =
                stationary_motion_decision_.allow_persistent_map_commit;
        }

        // Runtime local-map writes are authorized independently from
        // persistent MapCommit. STATIONARY_HOLD, MOVING_CONFIRM, CATCH_UP,
        // prediction-only, and severe-degeneracy frames all stop here.
        if (registration_success) {
            ++frames_since_last_update;
            const Sophus::SE3d delta =
                last_local_map_pose.inverse() * new_pose;
            const double move_dist = delta.translation().norm();
            const double move_rot = delta.so3().log().norm();
            const bool catch_up_blocks_local_map =
                stationary_motion_decision_.state ==
                    RuntimeMotionState::CATCH_UP;
            if (!allow_runtime_local_map_update_ ||
                catch_up_blocks_local_map ||
                !relocalization_pose_reliable_) {
                ROS_DEBUG("[LocalMap] blocked state=%s reason=%s",
                          runtimeMotionStateName(
                              stationary_motion_decision_.state),
                          stationary_motion_decision_.reason.c_str());
            } else if (move_dist > 0.5 || move_rot > 0.08 ||
                       frames_since_last_update > 15) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(
                    new pcl::PointCloud<pcl::PointXYZ>);
                pcl::transformPointCloud(
                    *registration_cloud, *transformed,
                    new_pose.matrix().cast<float>());
                *local_map_ += *transformed;

                const Eigen::Vector3d current_pos = new_pose.translation();
                pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(
                    new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& p : local_map_->points) {
                    const double dx = p.x - current_pos.x();
                    const double dy = p.y - current_pos.y();
                    const double dz = p.z - current_pos.z();
                    if (dx * dx + dy * dy + dz * dz < 225.0) {
                        cropped->push_back(p);
                    }
                }

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
            logSO3GuardPose(
                "crane_constrained_pose", processing_frame_index,
                constrained_pose);
            if (!constrained_pose.translation().allFinite() ||
                !constrained_pose.so3().matrix().allFinite()) {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[SO3Guard] refusing non-finite constrained pose frame=%llu",
                    static_cast<unsigned long long>(processing_frame_index));
                registration_success = false;
            }
        }

        if (registration_success) {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            current_pose_ = constrained_pose;  // 发布约束后的 pose
        }

        if (ndt_attempted_this_frame) {
            const bool frame_ndt_healthy =
                last_ndt_converged_ && ndt_safe_pose_valid_this_frame &&
                std::isfinite(last_ndt_fitness_) &&
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
            Sophus::SE3d final_pose =
                selectPublishedPose(constrained_pose, publish_time);
            if (!final_pose.translation().allFinite() ||
                !final_pose.so3().matrix().allFinite()) {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[SO3Guard] published-pose selector returned non-finite "
                    "pose frame=%llu; using constrained pose",
                    static_cast<unsigned long long>(processing_frame_index));
                final_pose = constrained_pose;
            }
            logSO3GuardPose(
                "published_pose", processing_frame_index, final_pose);
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
                    feature_cloud, filtered_cloud, final_pose, publish_time,
                    msg->header.stamp,
                    diag_queue_age_ms * 0.001 +
                        elapsedMs(start_time) * 0.001);
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

            ROS_DEBUG_THROTTLE(
                5.0,
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
                enqueueMapCommitJob(filtered_cloud, final_pose, publish_time);
                diag_stage.map_commit_ms = elapsedMs(map_commit_start);
                // Filtering and map writes now run on the bounded worker.
                // Only a completed, current-epoch job advances the owner-side
                // target and gate references.
                diag_stage.clean_map_ms = 0.0;
                diag_stage.display_map_ms = 0.0;
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
            ndt_rec.registration_mode = registration_build_result.mode;
            ndt_rec.static_object_points = static_cast<int>(
                registration_build_result.static_object_points);
            ndt_rec.uncertain_candidate_points = static_cast<int>(
                registration_build_result.uncertain_candidate_points);
            ndt_rec.ground_points = static_cast<int>(
                registration_build_result.ground_points);
            ndt_rec.ground_fraction =
                registration_build_result.ground_fraction;
            ndt_rec.structure_quality_valid =
                registration_build_result.structure_quality_valid;
            ndt_rec.observability_valid = frame_ndt_observability.valid;
            ndt_rec.observability_degenerate =
                frame_ndt_observability.degenerate;
            ndt_rec.observability_severe =
                frame_ndt_observability.severely_degenerate;
            ndt_rec.observability_strong_eigenvalue =
                frame_ndt_observability.strong_eigenvalue;
            ndt_rec.observability_weak_eigenvalue =
                frame_ndt_observability.weak_eigenvalue;
            ndt_rec.observability_ratio =
                frame_ndt_observability.eigenvalue_ratio;
            ndt_rec.observability_weak_direction_x =
                frame_ndt_observability.weak_direction.x();
            ndt_rec.observability_weak_direction_y =
                frame_ndt_observability.weak_direction.y();
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
            ndt_rec.ndt_execution_state = ndt_execution_state;
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
            if (!ndt_attempted_this_frame) {
                runtime_diag_.clearConsoleRisk(
                    "NDT_NOT_CONVERGED", "NDT_RISK");
            } else {
                // NDT 风险检测
                if (!last_ndt_converged_) {
                    runtime_diag_.logNdtRiskNotConverged(
                        diag_frame_index_, diag_last_cloud_stamp_,
                        last_ndt_fitness_, last_ndt_iterations_,
                        last_actual_target_source_, last_target_points_,
                        last_source_points_, last_ndt_time_ms_,
                        average_process_time_ms_);
                } else {
                    runtime_diag_.clearConsoleRisk(
                        "NDT_NOT_CONVERGED", "NDT_RISK");
                }
            }
            const double robust_fitness_median =
                runtime_diag_.fitnessStats().median();
            const double robust_fitness_mad =
                runtime_diag_.fitnessStats().mad();
            const bool fitness_warm =
                runtime_diag_.fitnessStats().count() >= 30U;
            const double rolling_fitness_threshold = std::max(
                robust_fitness_median * 2.0,
                robust_fitness_median +
                    5.0 * std::max(1.0e-4, robust_fitness_mad));
            const bool fitness_spike_candidate =
                ndt_attempted_this_frame && last_ndt_converged_ &&
                std::isfinite(last_ndt_fitness_) && fitness_warm &&
                last_ndt_fitness_ >= map_commit_max_fitness_ &&
                last_ndt_fitness_ >= rolling_fitness_threshold;
            static int fitness_spike_enter_frames = 0;
            static int fitness_spike_clear_frames = 0;
            static bool fitness_spike_active = false;
            if (fitness_spike_candidate) {
                ++fitness_spike_enter_frames;
                fitness_spike_clear_frames = 0;
                if (fitness_spike_enter_frames >= 3) {
                    fitness_spike_active = true;
                    runtime_diag_.logNdtRiskFitnessSpike(
                        diag_frame_index_, diag_last_cloud_stamp_,
                        last_ndt_fitness_, robust_fitness_median,
                        robust_fitness_mad, map_commit_max_fitness_,
                        last_ndt_converged_, last_raw_step_, diag_innovation);
                }
            } else if (ndt_attempted_this_frame) {
                fitness_spike_enter_frames = 0;
                if (fitness_spike_active &&
                    ++fitness_spike_clear_frames >= 3) {
                    runtime_diag_.clearConsoleRisk(
                        "NDT_FITNESS_SPIKE", "NDT_RISK");
                    fitness_spike_active = false;
                    fitness_spike_clear_frames = 0;
                }
            }

            // Frame overrun 检测
            // The budget comes from callback sensor cadence. Processed sensor
            // dt is already distorted after drops and must never define PASS.
            // Prediction-only 检测
            if (diag_prediction_only) {
                runtime_diag_.incrementPredictionOnly();
                diag_consecutive_prediction_only_++;
                runtime_diag_.logEkfRiskPredictionOnly(
                    diag_frame_index_, diag_last_cloud_stamp_, diag_reject_reason,
                    last_ndt_fitness_, last_ndt_converged_, diag_innovation,
                    diag_consecutive_prediction_only_);
                if (diag_consecutive_prediction_only_ > 3) {
                    runtime_diag_.logEkfRiskPredictionStreak(
                        diag_consecutive_prediction_only_, 0, diag_last_valid_ndt_stamp_);
                }
            } else {
                runtime_diag_.clearConsoleRisk(
                    "EKF_PREDICTION_ONLY", "EKF_RISK");
                runtime_diag_.clearConsoleRisk(
                    "EKF_PREDICTION_STREAK", "EKF_RISK");
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
            } else {
                runtime_diag_.clearConsoleRisk(
                    "ODOM_RAW_STEP", "ODOM_RISK");
            }

            // Output step violation 检测
            if (diag_max_allowed_step > 0.0 &&
                diag_output_step > diag_max_allowed_step + 0.0001) {
                runtime_diag_.incrementOutputStepViolation();
                runtime_diag_.logOdomRiskOutputStepViolation(
                    diag_frame_index_, diag_last_cloud_stamp_,
                    diag_output_dx, diag_output_dy, 0.0,
                    diag_output_step, diag_max_allowed_step);
            } else {
                runtime_diag_.clearConsoleRisk(
                    "ODOM_OUTPUT_STEP", "ODOM_RISK");
            }

            // 周期性健康日志
            static ros::Time last_diag_health_time;
            if ((ros::Time::now() - last_diag_health_time).toSec() >= 5.0) {
                logNdtHealthPeriodic();
                last_diag_health_time = ros::Time::now();
            }

            // Cargo CSV is frame-by-frame. RuntimeDiagnostics performs its own
            // console throttling, so operational output remains sparse.
            logCargoHealthPeriodic();

            // Create the pending record now; total_ms is finalized at the true
            // loop tail after periodic maintenance and risk output.
            average_process_time_ms_ = elapsedMs(start_time);
            last_total_ms_ = average_process_time_ms_;
            ndt_rec.total_ms = average_process_time_ms_;
            diag_pending_ndt_record_ = std::move(ndt_rec);
            diag_pending_ndt_record_valid_ = true;
        }

        ROS_DEBUG_THROTTLE(5.0,
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
            ROS_DEBUG("[Status] frames=%d/%d, pose=(%.2f, %.2f, %.2f), "
                     "keyframes=%d, tiles_flushed=%d, "
                     "local_map=%zu, active_map=%zu, process=%.2fs",
                     success_frames, total_frames, pos.x(), pos.y(), pos.z(),
                     keyframe_count_.load(std::memory_order_relaxed),
                     flushed_tile_count_.load(std::memory_order_relaxed),
                     local_map_->size(), global_map_->size(), elapsed);
            last_log_time = ros::Time::now();

            // ========== 长期建图维护 ==========
            if (longterm_mapping_enabled_) {
                // 更新关键帧统计
                total_keyframes_ = static_cast<int>(
                    loop_closure_detector_.getKeyFrameCount());
                active_keyframes_ = std::min(total_keyframes_, max_active_keyframes_);

                // 定期释放旧关键帧
                static int release_check_count = 0;
                release_check_count++;
                if (release_check_count >= keyframe_release_interval_) {
                    release_keyframes_pending_ = true;
                    map_maintenance_pending_ = true;
                    release_check_count = 0;
                }

                // 定期 flush dirty tiles
                if (persistent_map_enabled_) {
                    double time_since_flush = (ros::Time::now() - last_flush_time_).toSec();
                    if (time_since_flush >= flush_interval_sec_ || dirty_tile_count_ >= max_dirty_tiles_) {
                        flush_tiles_pending_ = true;
                        map_maintenance_pending_ = true;
                    }

                    // 定期写入 runtime status（每 5 秒）
                    static ros::Time last_status_write_time;
                    double time_since_status = (ros::Time::now() - last_status_write_time).toSec();
                    if (time_since_status >= 5.0) {
                        runtime_status_pending_ = true;
                        map_maintenance_pending_ = true;
                        last_status_write_time = ros::Time::now();
                    }
                }

                // 内存保护检查
                if (memory_guard_enabled_) {
                    double time_since_check = (ros::Time::now() - last_memory_check_time_).toSec();
                    if (time_since_check >= memory_check_interval_sec_) {
                        memory_guard_pending_ = true;
                        map_maintenance_pending_ = true;
                        last_memory_check_time_ = ros::Time::now();
                    }
                }

                // 定期重建 active map（每 10 个关键帧）
                if (longterm_mapping_enabled_ && keyframe_count_ > 0 && keyframe_count_ % rebuild_every_keyframes_ == 0) {
                    active_map_rebuild_pending_ = true;
                    map_maintenance_pending_ = true;
                }
                if (map_maintenance_pending_) queue_cv_.notify_one();
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
            PipelineRiskRecord pipeline_risk;
            pipeline_risk.frame = diag_pending_ndt_record_.frame_index;
            pipeline_risk.stamp = diag_pending_ndt_record_.cloud_stamp;
            pipeline_risk.frame_budget_ms = frame_budget_ms;
            pipeline_risk.total_ms = average_process_time_ms_;
            pipeline_risk.preprocess_ms =
                diag_pending_ndt_record_.preprocess_ms;
            pipeline_risk.target_prepare_ms =
                diag_pending_ndt_record_.target_prepare_ms;
            pipeline_risk.ndt_align_ms =
                diag_pending_ndt_record_.ndt_align_ms;
            pipeline_risk.ekf_ms = diag_pending_ndt_record_.ekf_ms;
            pipeline_risk.map_commit_ms =
                diag_pending_ndt_record_.map_commit_ms;
            pipeline_risk.processed_hz = final_rate.processed_hz;
            pipeline_risk.input_hz = final_rate.callback_hz;
            pipeline_risk.drop_ratio = final_rate.drop_ratio;
            if (frame_budget_ms > 0.0 &&
                average_process_time_ms_ > frame_budget_ms) {
                std::size_t final_queue_size = 0U;
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    final_queue_size = cloud_queue_.size();
                }
                runtime_diag_.incrementFrameOverrun();
                pipeline_risk.consecutive_overruns =
                    runtime_diag_.consecutiveOverruns();
                pipeline_risk.estimated_backlog_frames =
                    static_cast<double>(final_queue_size);
                const bool sustained =
                    pipeline_risk.consecutive_overruns >=
                    runtime_diag_config_.warn_consecutive_overrun_frames;
                pipeline_risk.reason = sustained
                    ? "SUSTAINED_OVERRUN" : "FRAME_OVERRUN";
                pipeline_risk.level = sustained ? 2 : 1;
                runtime_diag_.updatePipelineRisk(pipeline_risk);
            } else {
                pipeline_risk.consecutive_overruns =
                    runtime_diag_.consecutiveOverruns();
                runtime_diag_.clearPipelineRisk(pipeline_risk);
                runtime_diag_.resetConsecutiveOverruns();
            }
            average_process_time_ms_ = elapsedMs(start_time);
            last_total_ms_ = average_process_time_ms_;
            diag_pending_ndt_record_.total_ms = average_process_time_ms_;
        }
    }

    ROS_DEBUG("Processing thread stopped");
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
        ROS_DEBUG("Feature: total=%lu, feature=%lu, ground=%lu, reg=%lu",
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
// Runtime motion constraints are applied inside the EKF before publication;
// the later MapCommit gate remains read-only.
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
// Runtime motion constraints are applied inside the EKF before publication;
// the later MapCommit gate remains read-only.
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

    ROS_DEBUG("Published initial TF: map -> %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
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
                    const Eigen::Matrix4d refined_matrix =
                        raw.cast<double>();
                    const SafeSE3Result converted =
                        makeSafeSE3FromMatrix(refined_matrix);
                    logSO3GuardStage(
                        "icp_matrix", job.frame_index,
                        refined_matrix.block<3, 3>(0, 0),
                        refined_matrix.allFinite(), converted.valid,
                        converted.diagnostics.reason);
                    if (!converted.valid) {
                        ROS_WARN_THROTTLE(
                            1.0,
                            "[SO3Guard] rejected ICP result frame=%llu "
                            "reason=%s input_orth=%.17g input_det=%.17g "
                            "projected_orth=%.17g projected_det=%.17g",
                            static_cast<unsigned long long>(job.frame_index),
                            converted.diagnostics.reason.c_str(),
                            converted.diagnostics.input_orthogonality_error,
                            converted.diagnostics.input_determinant,
                            converted.diagnostics.projected_orthogonality_error,
                            converted.diagnostics.projected_determinant);
                    } else {
                    const Sophus::SE3d refined = converted.pose;

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

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_cloud_ = transformed_cloud.makeShared();
    }

    ++frame_count_;
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

    // Capture before launching. If reset/load advances the generation before
    // this worker starts running, the old request must remain stale instead of
    // adopting the replacement map's generation.
    const std::uint64_t generation =
        map_rebuild_generation_.load(std::memory_order_acquire);
    rebuild_thread_ = std::thread([this, generation]() {
        auto start = std::chrono::steady_clock::now();
        bool succeeded = false;
        try {
            rebuildGlobalMapFromSnapshot(generation);
            succeeded = generation == map_rebuild_generation_.load(
                std::memory_order_acquire);
        } catch (const std::exception& error) {
            ROS_ERROR("[AsyncRebuild] worker exception: %s", error.what());
        } catch (...) {
            ROS_ERROR("[AsyncRebuild] worker exception: unknown");
        }

        auto end = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(end - start).count();
        if (succeeded) {
            ROS_INFO("[AsyncRebuild] all maps rebuilt in %.2fs", elapsed);
        } else if (generation != map_rebuild_generation_.load(
                       std::memory_order_acquire)) {
            ROS_INFO("[AsyncRebuild] stale generation discarded in %.2fs",
                     elapsed);
        } else {
            ROS_ERROR("[AsyncRebuild] rebuild failed in %.2fs", elapsed);
        }
        rebuild_running_.store(false);
    });
}

void NdtSlamNode::rebuildGlobalMapFromSnapshot(
    std::uint64_t worker_generation) {
    std::lock_guard<std::mutex> rebuild_lock(
        map_rebuild_execution_mutex_);
    if (worker_generation != map_rebuild_generation_.load(
            std::memory_order_acquire)) return;
    const auto voxel_filter = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                 double leaf) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(
            new pcl::PointCloud<pcl::PointXYZ>);
        if (input->size() <= 100U) {
            *output = *input;
            return output;
        }
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setInputCloud(input);
        filter.setLeafSize(leaf, leaf, leaf);
        filter.filter(*output);
        return output;
    };

    for (;;) {
        // Clear before taking the snapshot. Any keyframe committed after the
        // snapshot sets this flag before it attempts map_mutex_.
        rebuild_pending_.store(false, std::memory_order_release);
        const auto keyframes = loop_closure_detector_.getKeyFramesSnapshot();
        auto new_global = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        auto new_display = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        auto new_ground = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
        auto new_objects = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>);

        std::size_t incomplete_layers = 0U;
        const auto append_in_range = [this](
            const pcl::PointCloud<pcl::PointXYZ>& source,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& destination) {
            for (const auto& point : source.points) {
                if (std::isfinite(point.x) && std::isfinite(point.y) &&
                    std::isfinite(point.z) &&
                    std::abs(point.x) <= max_map_size_ &&
                    std::abs(point.y) <= max_map_size_ &&
                    std::abs(point.z) <= max_map_size_) {
                    destination->push_back(point);
                }
            }
        };

        for (const auto& keyframe : keyframes) {
            const Sophus::SE3d pose = keyframe.has_refined_pose_
                ? keyframe.pose_refined_ : keyframe.pose_;
            const Eigen::Matrix4f transform = pose.matrix().cast<float>();
            pcl::PointCloud<pcl::PointXYZ> ground_map;
            pcl::PointCloud<pcl::PointXYZ> objects_map;

            if (keyframe.ground_points && keyframe.objects_filtered &&
                (!keyframe.ground_points->empty() ||
                 !keyframe.objects_filtered->empty())) {
                pcl::transformPointCloud(
                    *keyframe.ground_points, ground_map, transform);
                pcl::transformPointCloud(
                    *keyframe.objects_filtered, objects_map, transform);
            } else if (keyframe.cloud_ && !keyframe.cloud_->empty()) {
                // Loaded legacy databases have no layer metadata. Re-split
                // them statelessly and retain every object candidate; a
                // background rebuild must never run the stateful human filter
                // or silently authorize cargo removal.
                pcl::PointCloud<pcl::PointXYZ> ground_base;
                pcl::PointCloud<pcl::PointXYZ> objects_base;
                separateGroundByGrid(
                    *keyframe.cloud_, ground_base, objects_base);
                pcl::transformPointCloud(
                    ground_base, ground_map, transform);
                pcl::transformPointCloud(
                    objects_base, objects_map, transform);
                ++incomplete_layers;
            } else {
                ++incomplete_layers;
                continue;
            }

            append_in_range(ground_map, new_ground);
            append_in_range(objects_map, new_objects);
            append_in_range(ground_map, new_global);
            append_in_range(objects_map, new_global);
            append_in_range(ground_map, new_display);
            append_in_range(objects_map, new_display);
        }

        new_global = voxel_filter(new_global, voxel_size_);
        new_display = voxel_filter(new_display, display_voxel_size_);
        new_ground = voxel_filter(new_ground, ground_voxel_size_);
        new_objects = voxel_filter(new_objects, objects_voxel_size_);
        const std::size_t registration_size = new_global->size();
        const std::size_t display_size = new_display->size();
        const std::size_t ground_size = new_ground->size();
        const std::size_t objects_size = new_objects->size();

        bool retry = false;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            if (worker_generation != map_rebuild_generation_.load(
                    std::memory_order_acquire)) {
                ROS_WARN("[SnapshotRebuild] stale generation discarded");
                return;
            }
            retry = rebuild_pending_.exchange(
                false, std::memory_order_acq_rel);
            if (!retry) {
                global_map_.swap(new_global);
                display_map_.swap(new_display);
                ground_map_.swap(new_ground);
                objects_map_.swap(new_objects);
                advanceMapLayerGenerationLocked();
                advanceObjectsMapContentVersionLocked();
                rebuild_objects_filtered_.reset(
                    new pcl::PointCloud<pcl::PointXYZ>(*objects_map_));
            }
        }
        if (retry) {
            ROS_DEBUG("[SnapshotRebuild] keyframe changed; retry latest snapshot");
            continue;
        }

        clean_rebuild_requested_from_worker_.store(
            true, std::memory_order_release);
        queue_cv_.notify_one();
        ROS_INFO("[SnapshotRebuild] keyframes=%zu incomplete_layers=%zu "
                 "registration=%zu display=%zu ground=%zu objects=%zu",
                 keyframes.size(), incomplete_layers,
                 registration_size, display_size, ground_size, objects_size);
        return;
    }
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
        ROS_DEBUG("[MapCommit] keyframe #%d added | pos=(%.1f, %.1f, %.1f) | tiles=%d",
                 keyframe_count_.load(std::memory_order_relaxed),
                 pos.x(), pos.y(), pos.z(),
                 flushed_tile_count_.load(std::memory_order_relaxed));

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
                          tile_key.c_str(),
                          dirty_tile_count_.load(std::memory_order_relaxed));
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

        requestMapPublication(ros::Time::now());

        // 显示地图每3个关键帧更新一次，解耦实时处理和可视化
        static int display_publish_counter = 0;
        display_publish_counter++;
        if (display_publish_counter >= 3) {
            display_publish_counter = 0;
            // clean map 异步构建（不阻塞主处理线程）
            startCleanMapRebuildJob();
        }
    }

    if (keyframe_count_ % loop_detection_interval_ == 0) {
        ROS_DEBUG("Performing loop closure detection...");
        processLoopClosure();
    }
}
#endif  // 旧代码结束

void NdtSlamNode::publishCurrentCloud() {
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr snapshot;
    sensor_msgs::PointCloud2 cloud_msg;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (current_cloud_->empty()) return;
        snapshot = current_cloud_;
    }
    pcl::toROSMsg(*snapshot, cloud_msg);
    cloud_msg.header.stamp = last_stamp_;
    cloud_msg.header.frame_id = map_frame_;
    current_cloud_pub_.publish(cloud_msg);
}

void NdtSlamNode::processLoopClosure() {
    if (!loop_closure_enabled_) return;
    if (loop_closure_running_.exchange(true, std::memory_order_acq_rel)) {
        loop_closure_pending_ = true;
        return;
    }
    if (loop_closure_thread_.joinable()) loop_closure_thread_.join();

    const auto snapshot = loop_closure_detector_.getKeyFramesSnapshot();
    if (snapshot.size() < 30U) {
        loop_closure_running_.store(false, std::memory_order_release);
        return;
    }
    const std::uint64_t pose_version =
        keyframe_pose_version_.load(std::memory_order_acquire);

    loop_closure_thread_ = std::thread(
        [this, snapshot, pose_version]() mutable {
            LoopClosureResult result;
            try {
                result.pose_version = pose_version;
                result.snapshot_last_id = snapshot.back().id_;
                result.snapshot_last_pose = snapshot.back().pose_;
                result.candidate = loop_closure_detector_.detectLoop(snapshot);
                if (result.candidate.current_keyframe_id < 0 ||
                    result.candidate.candidate_keyframe_id < 0) {
                    result.reason = "no_candidate";
                } else {
                    PoseGraphOptimizer optimizer;
                    for (const auto& keyframe : snapshot) {
                        optimizer.addKeyFrame(keyframe);
                    }
                    const auto information =
                        Eigen::Matrix<double, 6, 6>::Identity();
                    for (std::size_t i = 0; i + 1U < snapshot.size(); ++i) {
                        optimizer.addOdometryEdge(
                            snapshot[i].id_, snapshot[i + 1U].id_,
                            snapshot[i].pose_.inverse() * snapshot[i + 1U].pose_,
                            information);
                    }
                    optimizer.addLoopEdge(
                        result.candidate.candidate_keyframe_id,
                        result.candidate.current_keyframe_id,
                        result.candidate.relative_pose, information);
                    if (optimizer.optimize(10)) {
                        result.optimized_keyframes.assign(
                            snapshot.begin(), snapshot.end());
                        optimizer.updateKeyFramePoses(result.optimized_keyframes);
                        if (!result.optimized_keyframes.empty()) {
                            result.optimized_last_pose =
                                result.optimized_keyframes.back().pose_;
                            result.valid = true;
                            result.reason = "optimized";
                        }
                    } else {
                        result.reason = "optimization_failed";
                    }
                }
            } catch (const std::exception& error) {
                result.valid = false;
                result.reason = std::string("worker_exception:") + error.what();
            } catch (...) {
                result.valid = false;
                result.reason = "worker_exception:unknown";
            }
            {
                std::lock_guard<std::mutex> lock(
                    loop_closure_result_mutex_);
                loop_closure_result_ = std::move(result);
            }
            loop_closure_result_ready_.store(
                true, std::memory_order_release);
            loop_closure_running_.store(false, std::memory_order_release);
            queue_cv_.notify_one();
        });
}

void NdtSlamNode::consumeLoopClosureResult(const ros::Time& stamp) {
    if (!loop_closure_result_ready_.exchange(
            false, std::memory_order_acq_rel)) return;
    LoopClosureResult result;
    {
        std::lock_guard<std::mutex> lock(loop_closure_result_mutex_);
        result = std::move(loop_closure_result_);
    }
    if (!result.valid) {
        ROS_DEBUG("[LoopWorker] result=%s", result.reason.c_str());
        return;
    }
    if (result.pose_version !=
        keyframe_pose_version_.load(std::memory_order_acquire)) {
        ROS_WARN("[LoopWorker] stale pose version discarded");
        loop_closure_pending_ = true;
        map_maintenance_pending_ = true;
        return;
    }
    const std::pair<int, int> loop_pair = {
        result.candidate.candidate_keyframe_id,
        result.candidate.current_keyframe_id};
    {
        std::lock_guard<std::mutex> lock(processed_loops_mutex_);
        if (!processed_loops_.insert(loop_pair).second) return;
    }

    const Sophus::SE3d correction =
        result.optimized_last_pose * result.snapshot_last_pose.inverse();
    loop_closure_detector_.applyOptimizedPoses(
        result.optimized_keyframes, result.snapshot_last_id, correction);
    keyframe_pose_version_.fetch_add(1U, std::memory_order_acq_rel);
    updatePoseFromLoopClosure(correction * current_pose_, stamp);
    asyncRebuildGlobalMap();
    ROS_WARN("[LoopWorker] applied loop %d<->%d at frame boundary",
             loop_pair.first, loop_pair.second);
}

void NdtSlamNode::updatePoseFromLoopClosure(
    const Sophus::SE3d& new_pose, const ros::Time& stamp) {
    Sophus::SE3d previous_pose;
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        previous_pose = current_pose_;
        current_pose_ = new_pose;
        relocalized_pose_ = current_pose_;
        tracking_lost_ = false;
    }
    if (crane_motion_ekf_enabled_) {
        crane_motion_ekf_.initialize(current_pose_, stamp);
    }
    const Sophus::SE3d local_correction =
        current_pose_ * previous_pose.inverse();
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (local_map_ && !local_map_->empty()) {
            auto corrected_local = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(
                *local_map_, *corrected_local,
                local_correction.matrix().cast<float>());
            local_map_.swap(corrected_local);
            ++local_map_version_;
        }
    }
    filtered_yaw_initialized_ = false;
    path_msg_.poses.clear();
    runtime_path_msg_.poses.clear();
    has_last_path_pose_ = false;
    formal_cargo_removal_authorized_ = false;
    {
        std::lock_guard<std::mutex> lock(localization_target_mutex_);
        const auto transform_target = [&local_correction](
            pcl::PointCloud<pcl::PointXYZ>::Ptr& target) {
            if (!target || target->empty()) return;
            auto corrected = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(
                *target, *corrected,
                local_correction.matrix().cast<float>());
            target.swap(corrected);
        };
        transform_target(localization_target_front_);
        transform_target(localization_target_back_);
        transform_target(localization_target_snapshot_);
        ++localization_target_version_;
        cached_target_valid_ = false;
        cached_target_version_ = 0;
        last_bound_ndt_target_.reset();
        last_bound_ndt_target_version_ = 0;
        last_bound_ndt_target_source_ = "none";
        last_target_reason_ = "loop_closure_rebind";
    }
    resetCargoAfterPoseDiscontinuity();
    relocalization_pose_reliable_ = false;
    relocalization_good_frames_ = 0;
    relocalization_bad_frames_ = 0;
    relocalization_state_ = RelocalizationState::DEGRADED;
    publishRelocalizationSafetyInvalid(stamp, "loop_closure_pose_jump");
    relocalization_invalid_safety_published_ = true;
    tracking_cv_.notify_all();
}

bool NdtSlamNode::resetService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    ROS_INFO("Resetting SLAM system...");
    std::lock_guard<std::mutex> runtime_state_lock(runtime_state_mutex_);
    std::lock_guard<std::mutex> map_commit_lifecycle_lock(
        map_commit_lifecycle_mutex_);
    keyframe_pose_version_.fetch_add(1U, std::memory_order_acq_rel);
    loop_closure_result_ready_.store(false, std::memory_order_release);

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        current_pose_ = Sophus::SE3d();
        relocalized_pose_ = Sophus::SE3d();
        published_pose_ = Sophus::SE3d();
        initialized_ = false;
        tracking_lost_ = false;
    }

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_rebuild_generation_.fetch_add(1U, std::memory_order_acq_rel);
        global_map_->clear();
        display_map_->clear();
        ground_map_->clear();
        objects_map_->clear();
        objects_clean_map_->clear();
        advanceMapLayerGenerationLocked();
        advanceObjectsMapContentVersionLocked();
        sealCurrentMapLayerBundleLocked(ros::Time::now());
        rebuild_objects_filtered_->clear();
        rebuild_payload_candidate_->clear();
        rebuild_payload_dynamic_->clear();
        rebuild_human_candidate_->clear();
        rebuild_human_dynamic_->clear();
        rebuild_human_pending_->clear();
        rebuild_ground_raw_->clear();
        current_cloud_->clear();
        local_map_->clear();
        local_map_version_ = 0;
        bootstrap_local_map_complete_ = false;
        bootstrap_local_map_frames_ = 0;
    }
    {
        std::lock_guard<std::mutex> lock(map_commit_queue_mutex_);
        map_commit_queue_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(map_commit_completion_mutex_);
        map_commit_completion_.pending = false;
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
    crane_motion_ekf_.reset();
    filtered_yaw_initialized_ = false;
    path_msg_.poses.clear();
    runtime_path_msg_.poses.clear();
    has_last_path_pose_ = false;
    last_processed_frame_stamp_ = ros::Time();
    last_processed_frame_size_ = 0U;
    last_processed_frame_hash_ = 0U;
    has_last_raw_ndt_pose_ = false;
    has_commit_gate_reference_ = false;
    resetStationaryState("reset_service");
    last_keyframe_pose_for_gate_ = Sophus::SE3d();
    last_keyframe_time_for_gate_ = ros::Time();
    moved_frame_count_ = 0;
    delta_translation_ = 0.0;
    delta_yaw_ = 0.0;
    last_stamp_ = ros::Time();
    last_tf_stamp_ = ros::Time();
    rebuild_pending_.store(false, std::memory_order_release);
    clean_rebuild_requested_from_worker_.store(
        false, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        cloud_queue_.clear();
        map_maintenance_pending_ = false;
        clean_map_rebuild_pending_ = false;
        loop_closure_pending_ = false;
    }
    active_map_rebuild_pending_.store(false, std::memory_order_release);
    active_map_rebuild_dirty_.store(false, std::memory_order_release);
    last_active_map_rebuild_time_sec_.store(0.0, std::memory_order_release);
    loop_closure_detector_.clear();
    {
        std::lock_guard<std::mutex> lock(processed_loops_mutex_);
        processed_loops_.clear();
    }
    resetCargoForHookState(false);
    payload_tracker_.reset();
    selected_payload_track_id_ = -1;
    has_selected_payload_track_ = false;
    selected_payload_stamp_ = ros::Time();
    human_filter_.reset();
    dynamic_event_manager_.reset();
    cargo_swept_history_.clear();
    new_cargo_volumes_this_frame_.clear();
    cargo_deny_history_.clear();
    bev_observation_count_.clear();
    cargo_fusion_track_id_ = 0U;
    relocalization_force_global_.store(false, std::memory_order_release);
    relocalization_state_ = RelocalizationState::IDLE;
    relocalization_pose_reliable_ = true;
    relocalization_invalid_safety_published_ = false;
    relocalization_bad_frames_ = 0;
    relocalization_good_frames_ = 0;
    relocalization_confirmation_count_ = 0;
    relocalization_confirmation_pose_ = Sophus::SE3d();
    relocalization_last_submit_frame_ = 0;
    relocalization_last_result_frame_ = 0;
    relocalization_cooldown_until_frame_ = 0;

    cargo_marker_lifecycle_.reset();
    const ros::Time reset_stamp = ros::Time::now();
    publishCargoFusionMarker(
        CargoBottomResult{}, reset_stamp, false, true);
    requestMapPublication(reset_stamp);

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_snapshot(
        new pcl::PointCloud<pcl::PointXYZ>);
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        *map_snapshot = *global_map_;
    }

    if (map_snapshot->empty()) {
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
        int result = writer.writeBinary(file_path, *map_snapshot);
        response.success = (result == 0);
        response.message = response.success ? "Map saved" : "Failed to save";
        response.num_points = map_snapshot->size();
        response.saved_file_path = file_path;

        if (response.success) {
            ROS_INFO("[SaveMap] using_filtered_keyframes=true, placed_cargo_masks=%zu, dynamic_events=%zu",
                     dynamic_event_manager_.getPlacedSessions().size(),
                     dynamic_event_manager_.getActiveCount());
            ROS_INFO("Map saved: %s, points: %lu", file_path.c_str(), map_snapshot->size());

            // 同时保存关键帧数据库
            std::string session_dir = file_path.substr(0, file_path.find_last_of("/\\"));
            if (session_dir.empty()) session_dir = ".";
            session_dir += "/session_" + std::to_string(ros::Time::now().toSec());

            // 更新关键帧质量指标
            updateKeyFrameMetrics();

            // 保存关键帧数据库
            loop_closure_detector_.saveKeyFrameDatabase(session_dir);

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

        std::lock_guard<std::mutex> runtime_state_lock(runtime_state_mutex_);
        std::lock_guard<std::mutex> map_commit_lifecycle_lock(
            map_commit_lifecycle_mutex_);

        const ros::Time load_stamp = ros::Time::now();
        keyframe_pose_version_.fetch_add(1U, std::memory_order_acq_rel);
        loop_closure_result_ready_.store(false, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            current_pose_ = Sophus::SE3d();
            relocalized_pose_ = Sophus::SE3d();
            published_pose_ = Sophus::SE3d();
            initialized_ = false;
            tracking_lost_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            map_rebuild_generation_.fetch_add(
                1U, std::memory_order_acq_rel);
            global_map_ = loaded_cloud;
            display_map_.reset(
                new pcl::PointCloud<pcl::PointXYZ>(*loaded_cloud));
            ground_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            objects_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            objects_clean_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            advanceMapLayerGenerationLocked();
            advanceObjectsMapContentVersionLocked();
            sealCurrentMapLayerBundleLocked(load_stamp);
            current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            local_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
            rebuild_objects_filtered_.reset(
                new pcl::PointCloud<pcl::PointXYZ>);
            rebuild_payload_candidate_.reset(
                new pcl::PointCloud<pcl::PointXYZ>);
            rebuild_payload_dynamic_.reset(
                new pcl::PointCloud<pcl::PointXYZ>);
            rebuild_human_candidate_.reset(
                new pcl::PointCloud<pcl::PointXYZ>);
            rebuild_human_dynamic_.reset(
                new pcl::PointCloud<pcl::PointXYZ>);
            rebuild_human_pending_.reset(
                new pcl::PointCloud<pcl::PointXYZ>);
            rebuild_ground_raw_.reset(
                new pcl::PointCloud<pcl::PointXYZ>);
            local_map_version_ = 0;
            // A loaded map starts in relocalization/target mode. It must not
            // seed a new local map through the startup bootstrap bypass.
            bootstrap_local_map_complete_ = true;
            bootstrap_local_map_frames_ = 0;
        }
        {
            std::lock_guard<std::mutex> lock(map_commit_queue_mutex_);
            map_commit_queue_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(map_commit_completion_mutex_);
            map_commit_completion_.pending = false;
        }
        {
            std::lock_guard<std::mutex> lock(localization_target_mutex_);
            localization_target_front_.reset(
                new pcl::PointCloud<pcl::PointXYZ>);
            localization_target_back_.reset(
                new pcl::PointCloud<pcl::PointXYZ>);
            localization_target_snapshot_.reset(
                new pcl::PointCloud<pcl::PointXYZ>);
            ++localization_target_version_;
            localization_target_snapshot_version_ = 0;
            localization_target_ready_ = false;
            localization_target_state_ =
                LocalizationTargetState::BOOTSTRAP_LOCAL_MAP;
            cached_target_valid_ = false;
            cached_target_version_ = 0;
            cached_target_points_ = 0;
            crop_frames_since_update_ = 0;
            last_bound_ndt_target_.reset();
            last_bound_ndt_target_version_ = 0;
            last_bound_ndt_target_source_ = "none";
            last_actual_target_source_ = "bootstrap_local_map";
            last_target_reason_ = "map_loaded";
            target_version_ = 0;
            target_rebuild_count_ = 0;
            setInputTarget_count_ = 0;
        }

        // The matcher may retain a target shared_ptr internally. Recreate it
        // so no old-map target can survive the generation transition.
        ndt_.reset(new pclomp::NormalDistributionsTransform<
                   pcl::PointXYZ, pcl::PointXYZ>());
        ndt_->setResolution(ndt_resolution_);
        ndt_->setStepSize(ndt_step_size_);
        ndt_->setTransformationEpsilon(ndt_transformation_epsilon_);
        ndt_->setMaximumIterations(ndt_max_iterations_);
        ndt_->setNumThreads(ndt_num_threads_);
        if (ndt_neighbor_search_method_ == "DIRECT7") {
            ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        } else if (ndt_neighbor_search_method_ == "DIRECT1") {
            ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT1);
        } else {
            ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
        }

        crane_motion_ekf_.reset();
        frame_count_ = 0;
        filtered_yaw_initialized_ = false;
        path_msg_.poses.clear();
        runtime_path_msg_.poses.clear();
        has_last_path_pose_ = false;
        last_processed_frame_stamp_ = ros::Time();
        last_processed_frame_size_ = 0U;
        last_processed_frame_hash_ = 0U;
        has_last_raw_ndt_pose_ = false;
        has_commit_gate_reference_ = false;
        resetStationaryState("load_map");
        last_keyframe_pose_for_gate_ = Sophus::SE3d();
        last_keyframe_time_for_gate_ = ros::Time();
        moved_frame_count_ = 0;
        delta_translation_ = 0.0;
        delta_yaw_ = 0.0;
        last_stamp_ = ros::Time();
        last_tf_stamp_ = ros::Time();

        loop_closure_detector_.clear();
        keyframe_count_ = 0;
        {
            std::lock_guard<std::mutex> lock(processed_loops_mutex_);
            processed_loops_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            cloud_queue_.clear();
            map_maintenance_pending_ = false;
            clean_map_rebuild_pending_ = false;
            loop_closure_pending_ = false;
        }
        rebuild_pending_.store(false, std::memory_order_release);
        clean_rebuild_requested_from_worker_.store(
            false, std::memory_order_release);
        active_map_rebuild_pending_.store(false, std::memory_order_release);
        active_map_rebuild_dirty_.store(false, std::memory_order_release);
        last_active_map_rebuild_time_sec_.store(
            0.0, std::memory_order_release);

        human_filter_.reset();
        dynamic_event_manager_.reset();
        payload_tracker_.reset();
        selected_payload_track_id_ = -1;
        has_selected_payload_track_ = false;
        selected_payload_stamp_ = ros::Time();
        resetCargoForHookState(false);
        cargo_fusion_track_id_ = 0U;
        cargo_swept_history_.clear();
        new_cargo_volumes_this_frame_.clear();
        cargo_deny_history_.clear();
        bev_observation_count_.clear();

        relocalization_confirmation_count_ = 0;
        relocalization_confirmation_pose_ = Sophus::SE3d();
        relocalization_last_submit_frame_ = 0;
        relocalization_last_result_frame_ = 0;
        relocalization_bad_frames_ = relocalization_enabled_
            ? relocalization_global_trigger_frames_ : 0;
        relocalization_good_frames_ = 0;
        relocalization_cooldown_until_frame_ = 0;
        relocalization_force_global_.store(
            relocalization_enabled_, std::memory_order_release);
        relocalization_pose_reliable_ = !relocalization_enabled_;
        relocalization_state_ = relocalization_enabled_
            ? RelocalizationState::DEGRADED
            : RelocalizationState::IDLE;
        relocalization_invalid_safety_published_ = relocalization_enabled_;
        cargo_marker_lifecycle_.reset();
        if (relocalization_enabled_) {
            publishRelocalizationSafetyInvalid(
                load_stamp, "runtime_map_replaced");
            publishRelocalizationStatus(
                "DEGRADED", "runtime_map_replaced_global_search_required");
        }

        response.success = true;
        response.message = relocalization_enabled_
            ? "Map loaded; global relocalization required"
            : "Map loaded; runtime state reset";
        response.num_points = loaded_cloud->size();

        requestMapPublication(load_stamp);

        ROS_INFO("Map loaded: %s, points: %lu", file_path.c_str(), loaded_cloud->size());
    } catch (const std::exception& e) {
        response.success = false;
        response.message = std::string("Exception: ") + e.what();
        response.num_points = 0;
    }

    return true;
}

bool NdtSlamNode::rebuildMapService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    ROS_INFO("Rebuilding map from keyframes with edge-preserving fusion...");
    std::lock_guard<std::mutex> runtime_state_lock(runtime_state_mutex_);
    std::lock_guard<std::mutex> map_commit_lifecycle_lock(
        map_commit_lifecycle_mutex_);
    keyframe_pose_version_.fetch_add(1U, std::memory_order_acq_rel);
    loop_closure_result_ready_.store(false, std::memory_order_release);
    map_rebuild_generation_.fetch_add(1U, std::memory_order_acq_rel);
    {
        std::lock_guard<std::mutex> lock(processed_loops_mutex_);
        processed_loops_.clear();
    }

    const std::string session_dir = persistent_map_root_dir_ +
        "/rebuild_" + std::to_string(ros::Time::now().toSec());

    // 先加载关键帧数据库
    if (loop_closure_detector_.getKeyFrameCount() == 0U) {
        ROS_ERROR("No keyframes loaded; rebuild requires an explicitly loaded "
                  "keyframe database under persistent_map.root_dir");
        return false;
    }

    rebuildGlobalMapFromSnapshot(
        map_rebuild_generation_.load(std::memory_order_acquire));
    saveMultiLayerMaps(session_dir);

    ROS_INFO("Map rebuilt successfully. Output: %s", session_dir.c_str());

    return true;
}

void NdtSlamNode::updateKeyFrameMetrics() {
    const auto kf_list = loop_closure_detector_.getKeyFramesSnapshot();
    std::vector<std::pair<std::uint64_t, KeyFrameMetrics>> updates;
    updates.reserve(kf_list.size());

    for (auto& kf : kf_list) {
        // 计算质量指标
        KeyFrameMetrics metrics;

        // 地面/非地面分割统计
        if (!kf.cloud_ || kf.cloud_->empty()) continue;
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
        updates.emplace_back(kf.id_, metrics);
    }

    loop_closure_detector_.applyKeyFrameMetrics(updates);

    ROS_INFO("Updated metrics for %zu keyframes", kf_list.size());
}

void NdtSlamNode::saveMultiLayerMaps(const std::string& session_dir) {
    try {
        std::filesystem::path session_path(session_dir);

        // 创建目录
        std::filesystem::create_directories(session_path);

        // Capture one coherent generation of every map pointer. PCD I/O can
        // take seconds and must not race background pointer swaps or hold the
        // live map mutex for the duration of disk writes.
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr display_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr objects_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clean_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr payload_candidate_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr payload_dynamic_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr human_candidate_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr human_dynamic_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr human_pending_snapshot;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_raw_snapshot;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            const auto clone = [](const auto& map) {
                return map
                    ? pcl::PointCloud<pcl::PointXYZ>::Ptr(
                          new pcl::PointCloud<pcl::PointXYZ>(*map))
                    : pcl::PointCloud<pcl::PointXYZ>::Ptr(
                          new pcl::PointCloud<pcl::PointXYZ>);
            };
            // Formal layers are saved from one completed immutable bundle.
            // This prevents raw N+1 from being serialized with clean N.
            global_snapshot = clone(
                latest_completed_map_bundle_.registration);
            display_snapshot = clone(
                latest_completed_map_bundle_.display);
            ground_snapshot = clone(latest_completed_map_bundle_.ground);
            objects_snapshot = clone(latest_completed_map_bundle_.objects);
            clean_snapshot = clone(
                latest_completed_map_bundle_.objects_clean);
            // Diagnostic layers are not members of the formal bundle and are
            // intentionally captured from their current working snapshots.
            filtered_snapshot = clone(rebuild_objects_filtered_);
            payload_candidate_snapshot = clone(rebuild_payload_candidate_);
            payload_dynamic_snapshot = clone(rebuild_payload_dynamic_);
            human_candidate_snapshot = clone(rebuild_human_candidate_);
            human_dynamic_snapshot = clone(rebuild_human_dynamic_);
            human_pending_snapshot = clone(rebuild_human_pending_);
            ground_raw_snapshot = clone(rebuild_ground_raw_);
        }

        // 保存各层地图
        auto saveMap = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& map, const std::string& filename) {
            if (map && !map->empty()) {
                std::string filepath = session_path / filename;
                const int status = pcl::io::savePCDFileBinary(filepath, *map);
                if (status < 0) {
                    ROS_ERROR("Failed to save %s (status=%d)",
                              filename.c_str(), status);
                } else {
                    ROS_INFO("Saved %s: %zu points",
                             filename.c_str(), map->size());
                }
            }
        };

        // ========== 正式地图层 ==========
        saveMap(global_snapshot, "map_registration.pcd");
        saveMap(display_snapshot, "map_display.pcd");
        saveMap(ground_snapshot, "map_ground.pcd");
        saveMap(objects_snapshot, "map_objects_raw.pcd");
        saveMap(clean_snapshot, "map_objects_clean.pcd");

        // ========== 调试/检测用 PCD ==========
        saveMap(filtered_snapshot, "map_objects_filtered.pcd");
        saveMap(payload_candidate_snapshot, "map_payload_candidate.pcd");
        saveMap(payload_dynamic_snapshot, "map_payload_dynamic.pcd");
        saveMap(human_candidate_snapshot, "map_human_candidate.pcd");
        saveMap(human_dynamic_snapshot, "map_human_dynamic.pcd");
        saveMap(human_pending_snapshot, "map_human_pending.pcd");
        saveMap(ground_raw_snapshot, "map_ground_raw.pcd");

        // 全量显示地图（ground + filtered_objects）
        pcl::PointCloud<pcl::PointXYZ>::Ptr display_full(new pcl::PointCloud<pcl::PointXYZ>);
        if (ground_raw_snapshot && !ground_raw_snapshot->empty()) {
            *display_full += *ground_raw_snapshot;
        }
        if (filtered_snapshot && !filtered_snapshot->empty()) {
            *display_full += *filtered_snapshot;
        }
        saveMap(display_full, "map_display_full.pcd");

        int total_saved = 0;
        if (global_snapshot && !global_snapshot->empty()) total_saved++;
        if (display_snapshot && !display_snapshot->empty()) total_saved++;
        if (ground_snapshot && !ground_snapshot->empty()) total_saved++;
        if (objects_snapshot && !objects_snapshot->empty()) total_saved++;
        if (clean_snapshot && !clean_snapshot->empty()) total_saved++;
        if (filtered_snapshot && !filtered_snapshot->empty()) total_saved++;
        if (payload_candidate_snapshot && !payload_candidate_snapshot->empty()) total_saved++;
        if (payload_dynamic_snapshot && !payload_dynamic_snapshot->empty()) total_saved++;
        if (human_candidate_snapshot && !human_candidate_snapshot->empty()) total_saved++;
        if (human_dynamic_snapshot && !human_dynamic_snapshot->empty()) total_saved++;
        if (human_pending_snapshot && !human_pending_snapshot->empty()) total_saved++;
        if (ground_raw_snapshot && !ground_raw_snapshot->empty()) total_saved++;
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
                for (const auto& session : placed_sessions) {
                    ROS_INFO("[SaveMapMaskConfirm]   session=%d, bbox=(%.1f,%.1f,%.1f)-(%.1f,%.1f,%.1f), placed_time=%.2f",
                             session.id,
                             session.placed_bbox.min_pt.x(), session.placed_bbox.min_pt.y(), session.placed_bbox.min_pt.z(),
                             session.placed_bbox.max_pt.x(), session.placed_bbox.max_pt.y(), session.placed_bbox.max_pt.z(),
                             session.placed_time);
                }
            }

            // 检查 objects_clean_map 是否包含 placed cargo 点
            if (clean_snapshot && !clean_snapshot->empty()) {
                int placed_points = 0;
                for (const auto& p : clean_snapshot->points) {
                    for (const auto& session : placed_sessions) {
                        if (session.placed_bbox.contains(p)) {
                            placed_points++;
                            break;
                        }
                    }
                }
                ROS_INFO("[SaveMapMaskConfirm] objects_clean_map contains %d placed cargo points out of %zu total",
                         placed_points, clean_snapshot->size());
            }

            // 检查 display_map 是否包含 placed cargo 点
            if (display_snapshot && !display_snapshot->empty()) {
                int placed_points = 0;
                for (const auto& p : display_snapshot->points) {
                    for (const auto& session : placed_sessions) {
                        if (session.placed_bbox.contains(p)) {
                            placed_points++;
                            break;
                        }
                    }
                }
                ROS_INFO("[SaveMapMaskConfirm] display_map contains %d placed cargo points out of %zu total",
                         placed_points, display_snapshot->size());
            }
        } else {
            ROS_INFO("[SaveMapMaskConfirm] dynamic_events disabled, no mask applied");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception saving multi-layer maps: %s", e.what());
    }
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
    if (result.map_generation !=
            map_rebuild_generation_.load(std::memory_order_acquire) ||
        result.pose_version !=
            keyframe_pose_version_.load(std::memory_order_acquire)) {
        relocalization_confirmation_count_ = 0;
        publishRelocalizationStatus(
            "DEGRADED", "stale_map_or_pose_generation_discarded");
        return;
    }
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
    job.map_generation =
        map_rebuild_generation_.load(std::memory_order_acquire);
    job.pose_version =
        keyframe_pose_version_.load(std::memory_order_acquire);
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
    has_selected_payload_track_ = false;
    selected_payload_stamp_ = ros::Time();
    if (cargoTrackRetained()) {
        // The locked shape and live base/hook pose are independent of the map
        // transform. Reset only transform-dependent evidence; the same track
        // is reprojected with the recovered T_map_base.
        cargo_bottom_fusion_.reset();
        current_rigid_cargo_geometry_ = RigidCargoGeometry{};
        last_cargo_bottom_result_ = CargoBottomResult{};
        last_cargo_safety_result_ = CargoSafetyResult{};
        confirmed_cargo_safety_result_ = CargoSafetyResult{};
        cargo_safety_temporal_filter_.reset();
        cargo_obstacle_tracker_.reset();
        formal_cargo_removal_authorized_ = false;
        formal_cargo_removal_stamp_ = ros::Time();
        last_cargo_pipeline_stamp_ = ros::Time();
        hook_observation_associated_current_ = false;
        cargo_fusion_track_active_ = true;
        return;
    }
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
    status.cargo_valid = false;
    status.cargo_source =
        lidar_slam2_msgs::CargoBottomEstimate::SOURCE_INVALID;
    status.hook_signal_valid = hook.valid;
    status.hook_load_state = hook.state;
    status.hook_voltage = hook.voltage;
    status.no_cargo_confirmed = false;
    status.obstacle_valid = false;
    status = composeCargoSafetyStatus(
        status, false, CargoSafetyFault::NONE, 0U, false,
        "relocalization:" + reason);
    cargo_raw_warning_code_ = 0;
    cargo_confirmed_warning_code_ = 0;
    cargo_temporal_candidate_code_ = 0;
    cargo_temporal_candidate_count_ = 0;
    cargo_used_previous_confirmation_ = false;
    cargo_raw_safety_status_pub_.publish(status);
    std_msgs::Int32 raw_status_code_msg;
    raw_status_code_msg.data = status.requested_alarm_code;
    cargo_raw_status_code_pub_.publish(raw_status_code_msg);
    cargo_safety_status_pub_.publish(status);
    logCargoSafetyStatus(status);
    publishCargoFusionMarker(
        CargoBottomResult{}, stamp, false, false);
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

StationaryMotionDecision NdtSlamNode::updateStationaryMotionState(
    const StationaryMotionInput& input,
    const Sophus::SE3d& pose_template,
    Sophus::SE3d& constrained_pose) {
    StationaryMotionDecision decision = stationary_motion_policy_.update(input);
    const RuntimeMotionState next_state = decision.state;

    if (previous_runtime_motion_state_ == RuntimeMotionState::MOVING &&
        next_state == RuntimeMotionState::STATIONARY_HOLD) {
        enterStationaryState(input, decision.reason);
    } else if (previous_runtime_motion_state_ != RuntimeMotionState::MOVING &&
               next_state == RuntimeMotionState::MOVING) {
        exitStationaryState(decision.reason);
    }

    if (next_state != previous_runtime_motion_state_) {
        ROS_INFO("[MotionState] %s -> %s reason=%s local_map=%d persistent_map=%d",
                 runtimeMotionStateName(previous_runtime_motion_state_),
                 runtimeMotionStateName(next_state), decision.reason.c_str(),
                 decision.allow_local_map_update ? 1 : 0,
                 decision.allow_persistent_map_commit ? 1 : 0);
    }

    const bool catch_up_blocks_maps =
        next_state == RuntimeMotionState::CATCH_UP;
    allow_runtime_local_map_update_ =
        decision.allow_local_map_update && !catch_up_blocks_maps;
    allow_persistent_map_commit_ =
        decision.allow_persistent_map_commit && !catch_up_blocks_maps;
    if (allow_runtime_local_map_update_) {
        local_map_update_allowed_count_.fetch_add(
            1, std::memory_order_relaxed);
    } else {
        local_map_update_blocked_count_.fetch_add(
            1, std::memory_order_relaxed);
    }
    if (allow_persistent_map_commit_) {
        persistent_map_commit_allowed_count_.fetch_add(
            1, std::memory_order_relaxed);
    } else {
        persistent_map_commit_blocked_count_.fetch_add(
            1, std::memory_order_relaxed);
    }

    if (decision.apply_position_constraint &&
        crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized()) {
        ros::Time constraint_stamp;
        constraint_stamp.fromSec(input.stamp_sec);
        constrained_pose = crane_motion_ekf_.applyStationaryConstraint(
            pose_template, decision.constrained_position,
            constraint_stamp);
        decision.constrained_position =
            constrained_pose.translation().head<2>();
    }

    previous_runtime_motion_state_ = next_state;
    return decision;
}

void NdtSlamNode::enterStationaryState(
    const StationaryMotionInput& input,
    const std::string& reason) {
    is_stationary_ = true;
    motion_gate_stationary_ = true;
    stationary_frame_count_ = 0;
    ROS_INFO("[MotionState] enter_stationary anchor=(%.3f,%.3f) reason=%s",
             input.filtered_position.x(), input.filtered_position.y(),
             reason.c_str());
}

void NdtSlamNode::exitStationaryState(const std::string& reason) {
    is_stationary_ = false;
    motion_gate_stationary_ = false;
    stationary_frame_count_ = 0;
    ROS_INFO("[MotionState] exit_stationary reason=%s", reason.c_str());
}

void NdtSlamNode::resetStationaryState(const std::string& reason) {
    stationary_motion_policy_.reset();
    stationary_motion_decision_ = StationaryMotionDecision{};
    previous_runtime_motion_state_ = RuntimeMotionState::MOVING;
    is_stationary_ = false;
    motion_gate_stationary_ = false;
    stationary_frame_count_ = 0;
    allow_runtime_local_map_update_ = false;
    allow_persistent_map_commit_ = false;
    ROS_INFO("[MotionState] reset reason=%s", reason.c_str());
}

void NdtSlamNode::handleLidarTimeRollback(
    const ros::Time& previous_stamp,
    const ros::Time& current_stamp) {
    {
        std::lock_guard<std::mutex> lock(hook_load_state_mutex_);
        pending_origin_height_valid_ = false;
        pending_origin_height_m_ = 0.0F;
        pending_origin_center_base_.setZero();
        pending_origin_stamp_ = ros::Time();
        empty_hook_height_history_.clear();
    }
    resetCargoForHookState(false);
    cargo_marker_lifecycle_.reset();
    last_cargo_pipeline_stamp_ = ros::Time();
    last_cargo_warning_stamp_ = ros::Time();
    last_anchor_detect_stamp_ = ros::Time();
    last_anchor_marker_stamp_ = ros::Time();
    last_anchor_summary_stamp_ = ros::Time();

    crane_motion_ekf_.reset();
    resetStationaryState("lidar_source_time_rollback");
    has_last_raw_ndt_pose_ = false;
    has_commit_gate_reference_ = false;
    filtered_yaw_initialized_ = false;
    last_processed_stamp_ = -1.0;
    last_stamp_ = current_stamp;
    last_tf_stamp_ = ros::Time();
    diag_last_cloud_stamp_ = 0.0;
    diag_last_valid_ndt_stamp_ = 0.0;
    diag_consecutive_prediction_only_ = 0;
    runtime_diag_.resetTimeEpoch();

    relocalization_bad_frames_ = 0;
    relocalization_good_frames_ = 0;
    relocalization_confirmation_count_ = 0;
    relocalization_last_submit_frame_ = 0;
    relocalization_last_result_frame_ = 0;
    relocalization_cooldown_until_frame_ = 0;
    relocalization_force_global_.store(false, std::memory_order_release);
    relocalization_state_ = RelocalizationState::IDLE;
    relocalization_pose_reliable_ = true;
    relocalization_invalid_safety_published_ = false;

    ROS_WARN("[TIME_EPOCH_RESET] source=lidar previous=%.6f current=%.6f "
             "ekf=reset motion=reset cargo=reset diagnostics=reset",
             previous_stamp.toSec(), current_stamp.toSec());
}

// CRITICAL RUNTIME CHAIN - DO NOT MODIFY
// a7be4bf runtime pose chain must stay unchanged:
// NDT/refined/EKF -> publishOdometry -> TF -> publishRuntimePath.
// Runtime motion constraints are applied inside the EKF before publication;
// the later MapCommit gate remains read-only.
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

    ROS_DEBUG_THROTTLE(
        5.0,
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

bool NdtSlamNode::shouldCommitKeyframe(
    const Sophus::SE3d& current_pose,
    const ros::Time& current_time) {
    if (!motion_gate_enabled_) {
        return true;
    }

    // CATCH_UP is an explicit no-write state even if a stale decision flag is
    // accidentally carried across a future refactor.
    if (!allow_persistent_map_commit_ ||
        stationary_motion_decision_.state == RuntimeMotionState::CATCH_UP) {
        return false;
    }

    if (last_keyframe_pose_for_gate_.translation().norm() < 0.001) {
        last_keyframe_pose_for_gate_ = current_pose;
        last_keyframe_time_for_gate_ = current_time;
        return true;
    }

    const Sophus::SE3d delta =
        last_keyframe_pose_for_gate_.inverse() * current_pose;
    const double translation = delta.translation().norm();
    const double rotation_deg =
        delta.so3().log().norm() * 180.0 / M_PI;
    const double elapsed_sec =
        (current_time - last_keyframe_time_for_gate_).toSec();
    delta_translation_ = translation;
    delta_yaw_ = rotation_deg;

    const bool moved_enough =
        translation >= motion_gate_min_translation_m_ ||
        rotation_deg >= motion_gate_min_rotation_deg_;
    if (moved_enough && elapsed_sec >= motion_gate_min_time_sec_) {
        last_keyframe_pose_for_gate_ = current_pose;
        last_keyframe_time_for_gate_ = current_time;
        ++moved_frame_count_;
        return true;
    }
    return false;
}


void NdtSlamNode::releaseOldKeyframeClouds() {
    if (max_active_keyframes_ <= 0) return;

    const std::size_t keyframe_count =
        loop_closure_detector_.getKeyFrameCount();

    // 释放超出窗口的旧关键帧的点云
    const std::size_t release_count =
        loop_closure_detector_.releaseCloudsBeforeActiveWindow(
            static_cast<std::size_t>(max_active_keyframes_));

    if (release_count > 0) {
        ROS_INFO("[LongTerm] Released %zu old keyframe clouds, active window: %zu",
                 release_count,
                 std::min(keyframe_count,
                          static_cast<std::size_t>(max_active_keyframes_)));
    }
}

void NdtSlamNode::flushDirtyTiles() {
    if (!persistent_map_enabled_) return;
    {
        std::lock_guard<std::mutex> lock(failed_tile_flush_mutex_);
        std::lock_guard<std::mutex> dirty_lock(dirty_tiles_mutex_);
        for (auto& [tile_key, failed] : failed_tile_flush_batch_) {
            auto& target = dirty_tiles_[tile_key];
            const auto merge_cloud = [](auto& destination, const auto& source) {
                if (!source || source->empty()) return;
                if (!destination) {
                    destination.reset(new pcl::PointCloud<pcl::PointXYZ>);
                }
                *destination += *source;
            };
            merge_cloud(target.registration, failed.registration);
            merge_cloud(target.display, failed.display);
            merge_cloud(target.ground, failed.ground);
            merge_cloud(target.objects, failed.objects);
        }
        failed_tile_flush_batch_.clear();
        dirty_tile_count_.store(
            static_cast<int>(dirty_tiles_.size()),
            std::memory_order_release);
    }
    if (tile_flush_running_.load(std::memory_order_acquire)) return;

    // 检查磁盘保护
    if (!checkDiskGuard()) {
        ROS_WARN_THROTTLE(60, "[DiskGuard] Skipping flush, disk low");
        return;
    }

    if (tile_flush_thread_.joinable()) tile_flush_thread_.join();

    std::map<std::string, TileLayers> flush_batch;
    {
        std::lock_guard<std::mutex> dirty_lock(dirty_tiles_mutex_);
        if (dirty_tiles_.empty()) return;
        flush_batch.swap(dirty_tiles_);
        dirty_tile_count_.store(0, std::memory_order_release);
    }
    last_flush_time_ = ros::Time::now();
    last_flush_time_local_ = last_flush_time_;
    tile_flush_running_.store(true, std::memory_order_release);

    tile_flush_thread_ = std::thread(
        [this, flush_batch = std::move(flush_batch)]() mutable {
    try {

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
        if (pcl::io::savePCDFileBinary(tmp_path, *filtered) != 0) {
            throw std::runtime_error("failed to write temporary tile: " +
                                     tmp_path);
        }
        // POSIX rename atomically replaces an existing destination. Check the
        // return code so a failed replacement is retained for retry.
        if (std::rename(tmp_path.c_str(), filepath.c_str()) != 0) {
            throw std::runtime_error("failed to replace tile: " + filepath);
        }

        return filtered->size();
    };

    int flushed = 0;
    std::map<std::string, TileLayers> failed_layers;
    for (auto& [tile_key, tile_layers] : flush_batch) {
        bool tile_ok = true;
        TileLayers retry;
        // 写入 registration layer
        if (tile_layers.registration && !tile_layers.registration->empty()) {
            std::string filepath = reg_dir + "/" + tile_key + ".pcd";
            try {
                mergeAndWrite(tile_layers.registration, filepath, tile_voxel_registration_);
            } catch (const std::exception& error) {
                retry.registration = tile_layers.registration;
                tile_ok = false;
                ROS_ERROR("[TileFlush] %s registration: %s",
                          tile_key.c_str(), error.what());
            }
        }

        // 写入 display layer
        if (tile_layers.display && !tile_layers.display->empty()) {
            std::string filepath = disp_dir + "/" + tile_key + ".pcd";
            try {
                mergeAndWrite(tile_layers.display, filepath, tile_voxel_display_);
            } catch (const std::exception& error) {
                retry.display = tile_layers.display;
                tile_ok = false;
                ROS_ERROR("[TileFlush] %s display: %s",
                          tile_key.c_str(), error.what());
            }
        }

        // 写入 ground layer
        if (tile_layers.ground && !tile_layers.ground->empty()) {
            std::string filepath = gnd_dir + "/" + tile_key + ".pcd";
            try {
                mergeAndWrite(tile_layers.ground, filepath, tile_voxel_ground_);
            } catch (const std::exception& error) {
                retry.ground = tile_layers.ground;
                tile_ok = false;
                ROS_ERROR("[TileFlush] %s ground: %s",
                          tile_key.c_str(), error.what());
            }
        }

        // 写入 objects layer
        if (tile_layers.objects && !tile_layers.objects->empty()) {
            std::string filepath = obj_dir + "/" + tile_key + ".pcd";
            try {
                mergeAndWrite(tile_layers.objects, filepath, tile_voxel_objects_);
            } catch (const std::exception& error) {
                retry.objects = tile_layers.objects;
                tile_ok = false;
                ROS_ERROR("[TileFlush] %s objects: %s",
                          tile_key.c_str(), error.what());
            }
        }

        if (tile_ok) {
            ++flushed;
        } else {
            failed_layers.emplace(tile_key, std::move(retry));
        }
    }

    if (!failed_layers.empty()) {
        std::lock_guard<std::mutex> lock(failed_tile_flush_mutex_);
        for (auto& entry : failed_layers) {
            failed_tile_flush_batch_[entry.first] = std::move(entry.second);
        }
    }

    const int total_flushed =
        flushed_tile_count_.fetch_add(flushed, std::memory_order_relaxed) +
        flushed;

    ROS_INFO("[TileFlush] %d tiles flushed to disk | total_flushed=%d",
             flushed, total_flushed);
    } catch (const std::exception& error) {
        ROS_ERROR("[TileFlush] worker failed: %s", error.what());
        std::lock_guard<std::mutex> lock(failed_tile_flush_mutex_);
        for (auto& entry : flush_batch) {
            failed_tile_flush_batch_[entry.first] = std::move(entry.second);
        }
    }
    tile_flush_running_.store(false, std::memory_order_release);
        });
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
    std::size_t global_pts = 0U;
    std::size_t display_pts = 0U;
    std::size_t ground_pts = 0U;
    std::size_t objects_pts = 0U;
    std::size_t local_pts = 0U;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        global_pts = global_map_ ? global_map_->size() : 0U;
        display_pts = display_map_ ? display_map_->size() : 0U;
        ground_pts = ground_map_ ? ground_map_->size() : 0U;
        objects_pts = objects_map_ ? objects_map_->size() : 0U;
        local_pts = local_map_ ? local_map_->size() : 0U;
    }

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
    f << "  \"dirty_tile_count\": "
      << dirty_tile_count_.load(std::memory_order_relaxed) << ",\n";
    f << "  \"flushed_tile_count\": "
      << flushed_tile_count_.load(std::memory_order_relaxed) << ",\n";
    f << "  \"channel_candidate_points\": "
      << channel_candidate_points_.load(std::memory_order_relaxed) << ",\n";
    f << "  \"candidate_removed_before_auth\": "
      << candidate_removed_before_auth_.load(std::memory_order_relaxed)
      << ",\n";
    f << "  \"candidate_kept_before_auth\": "
      << candidate_kept_before_auth_.load(std::memory_order_relaxed) << ",\n";
    f << "  \"candidate_human_filtered_points\": "
      << candidate_human_filtered_points_.load(std::memory_order_relaxed)
      << ",\n";
    f << "  \"formal_box_removed_points\": "
      << formal_box_removed_points_.load(std::memory_order_relaxed) << ",\n";
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
    f << "  \"last_active_map_rebuild_sec\": "
      << last_active_map_rebuild_time_sec_.load(std::memory_order_relaxed)
      << ",\n";
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

    auto forceVoxel = [](pcl::PointCloud<pcl::PointXYZ>::Ptr& map,
                         double voxel, const char* name) {
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
            return true;
        }
        return false;
    };

    bool layer_changed = forceVoxel(global_map_, 0.5, "global_map_");
    layer_changed = forceVoxel(display_map_, 0.5, "display_map_") ||
        layer_changed;
    layer_changed = forceVoxel(ground_map_, 0.3, "ground_map_") ||
        layer_changed;
    const bool objects_changed = objects_map_ && objects_map_->size() > 1000U;
    layer_changed = forceVoxel(objects_map_, 0.3, "objects_map_") ||
        layer_changed;
    if (layer_changed) {
        advanceMapLayerGenerationLocked();
    }
    if (objects_changed) {
        advanceObjectsMapContentVersionLocked();
    }
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

    if (active_map_rebuild_running_.exchange(
            true, std::memory_order_acq_rel)) {
        ROS_WARN_THROTTLE(10, "[ActiveMap] rebuild already running, skip");
        return;
    }
    if (active_map_rebuild_thread_.joinable()) {
        active_map_rebuild_thread_.join();
    }
    const std::uint64_t expected_generation =
        map_rebuild_generation_.load(std::memory_order_acquire);
    active_map_rebuild_thread_ = std::thread(
        [this, expected_generation]() {
        try {
            std::lock_guard<std::mutex> rebuild_lock(
                map_rebuild_execution_mutex_);
            const auto voxel_filter = [](
                const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                double leaf) {
                auto output = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZ>);
                if (input->size() <= 100U) {
                    *output = *input;
                    return output;
                }
                pcl::VoxelGrid<pcl::PointXYZ> filter;
                filter.setInputCloud(input);
                filter.setLeafSize(leaf, leaf, leaf);
                filter.filter(*output);
                return output;
            };

            for (;;) {
                if (expected_generation != map_rebuild_generation_.load(
                        std::memory_order_acquire)) break;
                active_map_rebuild_dirty_.store(
                    false, std::memory_order_release);
                const auto keyframes =
                    loop_closure_detector_.getKeyFramesSnapshot();
                const std::size_t active_limit = static_cast<std::size_t>(
                    std::max(1, max_active_keyframes_));
                std::vector<KeyFrame> recent;
                recent.reserve(std::min(
                    keyframes.size(), active_limit));
                for (auto it = keyframes.rbegin();
                     it != keyframes.rend() &&
                     recent.size() < active_limit; ++it) {
                    const bool has_layers =
                        (it->ground_points && !it->ground_points->empty()) ||
                        (it->objects_filtered &&
                         !it->objects_filtered->empty());
                    const bool has_raw = it->cloud_ && !it->cloud_->empty();
                    if (has_layers || has_raw) recent.push_back(*it);
                }
                if (recent.empty()) break;

                auto new_global = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZ>);
                auto new_display = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZ>);
                auto new_ground = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZ>);
                auto new_objects = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                    new pcl::PointCloud<pcl::PointXYZ>);
                const auto append_in_range = [this](
                    const pcl::PointCloud<pcl::PointXYZ>& source,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& destination) {
                    for (const auto& point : source.points) {
                        if (std::isfinite(point.x) &&
                            std::isfinite(point.y) &&
                            std::isfinite(point.z) &&
                            std::abs(point.x) <= max_map_size_ &&
                            std::abs(point.y) <= max_map_size_ &&
                            std::abs(point.z) <= max_map_size_) {
                            destination->push_back(point);
                        }
                    }
                };

                std::size_t legacy_splits = 0U;
                for (const auto& keyframe : recent) {
                    const Sophus::SE3d pose = keyframe.has_refined_pose_
                        ? keyframe.pose_refined_ : keyframe.pose_;
                    const Eigen::Matrix4f transform =
                        pose.matrix().cast<float>();
                    pcl::PointCloud<pcl::PointXYZ> ground_map;
                    pcl::PointCloud<pcl::PointXYZ> objects_map;
                    if (keyframe.ground_points &&
                        keyframe.objects_filtered &&
                        (!keyframe.ground_points->empty() ||
                         !keyframe.objects_filtered->empty())) {
                        pcl::transformPointCloud(
                            *keyframe.ground_points, ground_map, transform);
                        pcl::transformPointCloud(
                            *keyframe.objects_filtered, objects_map, transform);
                    } else if (keyframe.cloud_ &&
                               !keyframe.cloud_->empty()) {
                        pcl::PointCloud<pcl::PointXYZ> ground_base;
                        pcl::PointCloud<pcl::PointXYZ> objects_base;
                        separateGroundByGrid(
                            *keyframe.cloud_, ground_base, objects_base);
                        pcl::transformPointCloud(
                            ground_base, ground_map, transform);
                        pcl::transformPointCloud(
                            objects_base, objects_map, transform);
                        ++legacy_splits;
                    } else {
                        continue;
                    }
                    append_in_range(ground_map, new_ground);
                    append_in_range(objects_map, new_objects);
                    append_in_range(ground_map, new_global);
                    append_in_range(objects_map, new_global);
                    append_in_range(ground_map, new_display);
                    append_in_range(objects_map, new_display);
                }

                new_global = voxel_filter(new_global, voxel_size_);
                new_display = voxel_filter(
                    new_display, display_voxel_size_);
                new_ground = voxel_filter(new_ground, ground_voxel_size_);
                new_objects = voxel_filter(
                    new_objects, objects_voxel_size_);
                const std::size_t global_size = new_global->size();
                const std::size_t display_size = new_display->size();
                const std::size_t ground_size = new_ground->size();
                const std::size_t objects_size = new_objects->size();

                bool retry = false;
                {
                    std::lock_guard<std::mutex> lock(map_mutex_);
                    if (expected_generation !=
                        map_rebuild_generation_.load(
                            std::memory_order_acquire)) break;
                    retry = active_map_rebuild_dirty_.exchange(
                        false, std::memory_order_acq_rel);
                    if (!retry) {
                        global_map_.swap(new_global);
                        display_map_.swap(new_display);
                        ground_map_.swap(new_ground);
                        objects_map_.swap(new_objects);
                        advanceMapLayerGenerationLocked();
                        advanceObjectsMapContentVersionLocked();
                        rebuild_objects_filtered_.reset(
                            new pcl::PointCloud<pcl::PointXYZ>(*objects_map_));
                    }
                }
                if (retry) continue;

                last_active_map_rebuild_time_sec_.store(
                    ros::Time::now().toSec(), std::memory_order_release);
                clean_rebuild_requested_from_worker_.store(
                    true, std::memory_order_release);
                queue_cv_.notify_one();
                ROS_INFO("[ActiveMap] keyframes=%zu legacy=%zu "
                         "registration=%zu display=%zu ground=%zu objects=%zu",
                         recent.size(), legacy_splits, global_size,
                         display_size, ground_size, objects_size);
                break;
            }
        } catch (const std::exception& error) {
            ROS_ERROR("[ActiveMap] worker exception: %s", error.what());
        } catch (...) {
            ROS_ERROR("[ActiveMap] worker exception: unknown");
        }
        active_map_rebuild_running_.store(false, std::memory_order_release);
    });
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
    result.candidate_components_base.reset(
        new pcl::PointCloud<pcl::PointXYZ>);

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

    bool adaptive_search_valid = false;
    Eigen::Vector2f adaptive_center(cx, cy);
    float adaptive_yaw = 0.0F;
    float adaptive_half_long = 0.0F;
    float adaptive_half_short = 0.0F;
    if (!hook_lock_.provisional_observations.empty()) {
        const CargoCandidateDescriptor& provisional =
            hook_lock_.provisional_observations.back();
        adaptive_search_valid = provisional.center.allFinite();
        adaptive_center = provisional.center.head<2>();
        adaptive_yaw = provisional.yaw_rad;
        adaptive_half_long = 0.5F * provisional.size.x() +
            odom_anchor_config_.tight_box
                .component_merge_longitudinal_gap_m;
        adaptive_half_short = 0.5F * provisional.size.y() +
            odom_anchor_config_.tight_box.component_merge_lateral_gap_m;
    } else if (cargoTrackRetained() && hook_lock_.locked_shape.valid &&
               hook_lock_.live_pose.valid) {
        adaptive_search_valid = true;
        adaptive_center = hook_lock_.live_pose.center_base.head<2>();
        adaptive_yaw = hook_lock_.locked_shape.yaw_base_rad;
        adaptive_half_long = 0.5F * hook_lock_.locked_shape.length_m +
            odom_anchor_config_.tight_box
                .component_merge_longitudinal_gap_m;
        adaptive_half_short = 0.5F * hook_lock_.locked_shape.width_m +
            odom_anchor_config_.tight_box.component_merge_lateral_gap_m;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : cloud_base->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const bool inside_fixed = p.x >= x_min && p.x <= x_max &&
            p.y >= y_min && p.y <= y_max;
        bool inside_adaptive = false;
        if (adaptive_search_valid) {
            const Eigen::Vector2f delta(
                p.x - adaptive_center.x(), p.y - adaptive_center.y());
            const float cosine = std::cos(adaptive_yaw);
            const float sine = std::sin(adaptive_yaw);
            const float local_long =
                cosine * delta.x() + sine * delta.y();
            const float local_short =
                -sine * delta.x() + cosine * delta.y();
            inside_adaptive = std::abs(local_long) <= adaptive_half_long &&
                std::abs(local_short) <= adaptive_half_short;
        }
        if ((inside_fixed || inside_adaptive) &&
            p.z >= z_min && p.z <= z_max) {
            crop_cloud->push_back(p);
        }
    }
    result.roi_finite_points = crop_cloud->size();

    // A negative cargo observation is accepted only when the external ring
    // proves that the LiDAR actually observed the ROI surroundings. Detector
    // failures with residual HAG points remain UNKNOWN.
    const auto& tight = odom_anchor_config_.tight_box;
    if (tight.hag_filter_enabled) {
        ExternalGroundConfig ground_config;
        ground_config.ring_width_m = tight.ground_ring_width_m;
        ground_config.cell_size_m = tight.ground_cell_size_m;
        ground_config.minimum_cells = tight.ground_min_cells;
        ground_config.minimum_points_per_cell =
            tight.ground_min_points_per_cell;
        ground_config.minimum_quadrants = tight.ground_min_quadrants;
        ground_config.allow_opposite_sides = tight.ground_allow_opposite_sides;
        ground_config.maximum_range_m = tight.ground_max_range_m;
        ground_config.expected_height_enabled =
            tight.ground_expected_height_enabled;
        ground_config.expected_height_m = tight.ground_expected_height_m;
        ground_config.maximum_expected_height_delta_m =
            tight.ground_max_expected_height_delta_m;
        const ExternalGroundEstimate ground = estimateExternalGround(
            *cloud_base, cx, cy,
            odom_anchor_config_.search_half_x,
            odom_anchor_config_.search_half_y, ground_config);
        result.ground_z = ground.z_m;
        result.ground_reference_valid = ground.valid;
        result.roi_coverage_valid = ground.valid;
    }

    const auto classify_outcome = [&result, &tight](bool cargo_detected) {
        result.outcome = classifyCargoObservationOutcome({
            true, result.ground_reference_valid,
            result.roi_coverage_valid, result.hag_candidate_points,
            static_cast<std::size_t>(
                std::max(0, tight.empty_max_hag_candidate_points)),
            cargo_detected});
    };

    if (crop_cloud->empty()) {
        classify_outcome(false);
        result.reject_reason =
            result.outcome == CargoObservationOutcome::EMPTY_CONFIRMED
                ? "empty_confirmed_no_hag_candidates"
                : "crop_empty_unknown";
        ROS_DEBUG_THROTTLE(
            1.0, "[OdomAnchorDetect] input=%zu crop=0 ground_valid=%d "
            "outcome=%u anchor=(%.2f,%.2f)",
            cloud_base->size(), result.ground_reference_valid ? 1 : 0,
            static_cast<unsigned int>(result.outcome), cx, cy);
        return result;
    }

    result.raw_roi_min_z = minimumFiniteZ(*crop_cloud);
    if (!std::isfinite(result.raw_roi_min_z)) {
        result.reject_reason = "raw_roi_no_finite_z";
        return result;
    }

    // HAG 预过滤（如果启用）
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (tight.hag_filter_enabled) {
        if (result.ground_reference_valid) {
            for (const auto& point : crop_cloud->points) {
                const float hag = point.z - result.ground_z;
                if (hag >= tight.hag_min_m && hag <= tight.hag_max_m) {
                    filtered_cloud->push_back(point);
                }
            }
        } else {
            *filtered_cloud = *crop_cloud;
            ROS_DEBUG_THROTTLE(
                2.0, "[OdomAnchorDetect] no reliable external ground; "
                "bypass HAG prefilter");
        }
    } else {
        filtered_cloud = crop_cloud;
    }
    result.hag_candidate_points = filtered_cloud->size();
    classify_outcome(false);

    if (!filtered_cloud->empty()) {
        result.hag_filtered_min_z = minimumFiniteZ(*filtered_cloud);
        const float bottom_input_delta =
            result.hag_filtered_min_z - result.raw_roi_min_z;
        if (!std::isfinite(result.hag_filtered_min_z) ||
            !std::isfinite(bottom_input_delta)) {
            result.reject_reason = "hag_no_finite_z";
            return result;
        }
        ROS_DEBUG_THROTTLE(
            1.0, "[CargoBottomInput] raw_min=%.3f hag_min=%.3f "
            "ground_valid=%d ground_z=%.3f delta=%.3f",
            result.raw_roi_min_z, result.hag_filtered_min_z,
            result.ground_reference_valid ? 1 : 0, result.ground_z,
            bottom_input_delta);
    }

    if (filtered_cloud->empty()) {
        result.reject_reason =
            result.outcome == CargoObservationOutcome::EMPTY_CONFIRMED
                ? "empty_confirmed_no_hag_candidates"
                : "hag_filter_empty_unknown";
        ROS_DEBUG_THROTTLE(1.0, "[OdomAnchorDetect] input=%zu crop=%zu hag_filter=0",
                          cloud_base->size(), crop_cloud->size());
        return result;
    }

    if (result.outcome == CargoObservationOutcome::EMPTY_CONFIRMED) {
        result.reject_reason = "empty_confirmed_noise_floor";
        return result;
    }

    // 体素降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setInputCloud(filtered_cloud);
    vf.setLeafSize(0.05f, 0.05f, 0.05f);
    try {
        vf.filter(*voxel_cloud);
    } catch (const std::exception& error) {
        result.reject_reason =
            std::string("voxel_filter_failed:") + error.what();
        return result;
    } catch (...) {
        result.reject_reason = "voxel_filter_failed:unknown";
        return result;
    }

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
    ec.setClusterTolerance(
        odom_anchor_config_.tight_box.component_cluster_tolerance_m);
    ec.setMinClusterSize(odom_anchor_config_.weak_min_points);
    ec.setMaxClusterSize(8000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(voxel_cloud);
    try {
        ec.extract(cluster_indices);
    } catch (const std::exception& error) {
        result.reject_reason =
            std::string("clustering_failed:") + error.what();
        return result;
    } catch (...) {
        result.reject_reason = "clustering_failed:unknown";
        return result;
    }

    if (cluster_indices.empty()) {
        result.reject_reason = "no_clusters";
        ROS_DEBUG_THROTTLE(1.0, "[OdomAnchorDetect] input=%zu crop=%zu filtered=%zu voxel=%zu clusters=0",
                          cloud_base->size(), crop_cloud->size(), filtered_cloud->size(), voxel_cloud->size());
        return result;
    }

    // 选择最大簇
    // Build and score every disconnected component independently. The old
    // largest-cluster shortcut could repeatedly select a stable rack edge and
    // then report perfect yaw concentration for the wrong physical object.
    result.candidate_count = cluster_indices.size();
    CargoCandidateIdentityContext identity_context;
    identity_context.hook_center = anchor;
    identity_context.hook_region_radius_m = std::hypot(
        odom_anchor_config_.search_half_x,
        odom_anchor_config_.search_half_y);
    identity_context.strong_point_count = static_cast<std::size_t>(
        std::max(1, hook_lock_config_.lock_strong_min_points));
    identity_context.association_radius_m =
        hook_lock_config_.locked_update_max_center_dist;
    if (cargoTrackRetained() && hook_lock_.live_pose.valid &&
        hook_lock_.locked_shape.valid) {
        identity_context.predicted_track_valid = true;
        const double dt = std::min(
            static_cast<double>(
                hook_lock_config_.formal_xy_evidence_hold_sec),
            std::max(
                0.0,
                stamp.toSec() -
                    hook_lock_.live_pose.evidence_stamp_sec));
        const double decayed_dt =
            hook_lock_config_.lost_velocity_decay_tau_sec *
            (1.0 - std::exp(
                -dt / hook_lock_config_.lost_velocity_decay_tau_sec));
        identity_context.predicted_center =
            hook_lock_.live_pose.center_base +
            hook_lock_.live_pose_velocity_base *
                static_cast<float>(decayed_dt);
        identity_context.predicted_size = Eigen::Vector3f(
            hook_lock_.locked_shape.length_m,
            hook_lock_.locked_shape.width_m,
            hook_lock_.locked_shape.height_m);
        identity_context.predicted_yaw_rad =
            hook_lock_.locked_shape.yaw_base_rad;
        identity_context.association_radius_m = std::min(
            hook_lock_.state == HookCargoLockState::LOST_HOLD
                ? hook_lock_config_.reacquisition_max_xy_gate_m
                : hook_lock_config_.association_max_xy_gate_m,
            identity_context.association_radius_m +
                hook_lock_config_.velocity_model_uncertainty_mps *
                    static_cast<float>(dt) +
                hook_lock_.live_pose.position_uncertainty_m +
                hook_lock_.horizontal_tracking_residual_m);
    } else if (!hook_lock_.provisional_observations.empty()) {
        const CargoCandidateDescriptor& previous =
            hook_lock_.provisional_observations.back();
        identity_context.predicted_track_valid = true;
        identity_context.predicted_center = previous.center;
        identity_context.predicted_size = previous.size;
        identity_context.predicted_yaw_rad = previous.yaw_rad;
        identity_context.association_radius_m =
            hook_lock_config_.lock_max_center_step_m;
    } else if (retired_cargo_signature_valid_ &&
               retired_cargo_shape_.valid &&
               !retired_cargo_stamp_.isZero() &&
               (stamp - retired_cargo_stamp_).toSec() >= 0.0 &&
               (stamp - retired_cargo_stamp_).toSec() <=
                   hook_lock_config_.lost_clear_sec) {
        const double retired_dt = std::min(
            static_cast<double>(
                hook_lock_config_.formal_xy_evidence_hold_sec),
            (stamp - retired_cargo_stamp_).toSec());
        const double retired_decayed_dt =
            hook_lock_config_.lost_velocity_decay_tau_sec *
            (1.0 - std::exp(
                -retired_dt /
                    hook_lock_config_.lost_velocity_decay_tau_sec));
        identity_context.predicted_track_valid = true;
        identity_context.predicted_center =
            retired_cargo_center_base_ + retired_cargo_velocity_base_ *
                static_cast<float>(retired_decayed_dt);
        identity_context.predicted_size = Eigen::Vector3f(
            retired_cargo_shape_.length_m,
            retired_cargo_shape_.width_m,
            retired_cargo_shape_.height_m);
        identity_context.predicted_yaw_rad =
            retired_cargo_shape_.yaw_base_rad;
        identity_context.association_radius_m =
            hook_lock_config_.reacquisition_max_xy_gate_m;
    }

    CargoOrientedFootprintConfig candidate_footprint_config;
    candidate_footprint_config.minimum_points =
        static_cast<std::size_t>(std::max(
            3, odom_anchor_config_.tight_box.orientation_min_points));
    candidate_footprint_config.percentile_low =
        odom_anchor_config_.tight_box.percentile_low;
    candidate_footprint_config.percentile_high =
        odom_anchor_config_.tight_box.percentile_high;
    candidate_footprint_config.margin_m =
        odom_anchor_config_.tight_box.margin_xy_m;
    candidate_footprint_config.minimum_geometric_aspect_ratio =
        odom_anchor_config_.tight_box.orientation_min_geometric_aspect_ratio;
    candidate_footprint_config.minimum_eigenvalue_ratio =
        odom_anchor_config_.tight_box.orientation_min_eigenvalue_ratio;
    candidate_footprint_config.minimum_long_side_m = std::max(
        odom_anchor_config_.min_size_x, odom_anchor_config_.min_size_y);
    candidate_footprint_config.minimum_short_side_m = std::min(
        odom_anchor_config_.min_size_x, odom_anchor_config_.min_size_y);
    candidate_footprint_config.maximum_long_side_m = std::max(
        odom_anchor_config_.max_size_x, odom_anchor_config_.max_size_y);
    candidate_footprint_config.maximum_short_side_m = std::min(
        odom_anchor_config_.max_size_x, odom_anchor_config_.max_size_y);

    const bool collect_candidate_debug =
        cargo_candidate_components_pub_.getNumSubscribers() > 0U;
    struct CandidateComponentData {
        pcl::PointIndices indices;
        CargoCandidateDescriptor descriptor;
        float min_z = 0.0F;
        float max_z = 0.0F;
    };
    std::vector<CandidateComponentData> components;
    components.reserve(cluster_indices.size());
    for (std::size_t component_index = 0U;
         component_index < cluster_indices.size(); ++component_index) {
        std::vector<Eigen::Vector2f> footprint_points;
        std::vector<float> component_z;
        footprint_points.reserve(
            cluster_indices[component_index].indices.size());
        component_z.reserve(cluster_indices[component_index].indices.size());
        for (int point_index : cluster_indices[component_index].indices) {
            const pcl::PointXYZ& point = voxel_cloud->points[point_index];
            if (collect_candidate_debug) {
                result.candidate_components_base->push_back(point);
            }
            footprint_points.emplace_back(point.x, point.y);
            component_z.push_back(point.z);
        }
        std::sort(component_z.begin(), component_z.end());
        const CargoOrientedFootprint footprint =
            estimateCargoOrientedFootprint(
                footprint_points, candidate_footprint_config);
        if (!footprint.valid || component_z.size() < 3U) continue;
        const float z_low = component_z[static_cast<std::size_t>(
            (component_z.size() - 1U) *
            odom_anchor_config_.tight_box.percentile_low)];
        const float z_high = component_z[static_cast<std::size_t>(
            (component_z.size() - 1U) *
            odom_anchor_config_.tight_box.percentile_high)];
        CandidateComponentData component;
        component.indices = cluster_indices[component_index];
        component.min_z = z_low;
        component.max_z = z_high;
        component.descriptor.component_id =
            static_cast<int>(components.size());
        component.descriptor.center = Eigen::Vector3f(
            footprint.center_base.x(), footprint.center_base.y(),
            0.5F * (z_low + z_high));
        component.descriptor.size = Eigen::Vector3f(
            footprint.size_long_short.x(), footprint.size_long_short.y(),
            std::max(0.01F, z_high - z_low));
        component.descriptor.yaw_rad = footprint.yaw_base_rad;
        component.descriptor.orientation_confidence =
            footprint.orientation_confidence;
        component.descriptor.point_count =
            cluster_indices[component_index].indices.size();
        component.descriptor.suspension_evidence =
            result.ground_reference_valid &&
            z_low - result.ground_z >=
                hook_lock_config_.suspended_min_ground_clearance_m;
        components.push_back(component);
    }
    if (components.empty()) {
        result.reject_reason = "no_identity_valid_component";
        return result;
    }

    std::vector<CargoComponentFragment> fragments;
    fragments.reserve(components.size());
    for (const CandidateComponentData& component : components) {
        CargoComponentFragment fragment;
        fragment.center = component.descriptor.center.head<2>();
        fragment.length_m = component.descriptor.size.x();
        fragment.width_m = component.descriptor.size.y();
        fragment.yaw_rad = component.descriptor.yaw_rad;
        fragment.min_z = component.min_z;
        fragment.max_z = component.max_z;
        fragment.point_count = component.descriptor.point_count;
        fragments.push_back(fragment);
    }
    CargoComponentFusionConfig fusion_config;
    fusion_config.maximum_axial_yaw_difference_rad =
        odom_anchor_config_.tight_box
            .component_merge_max_yaw_difference_deg *
        3.14159265358979323846F / 180.0F;
    fusion_config.maximum_longitudinal_gap_m =
        odom_anchor_config_.tight_box.component_merge_longitudinal_gap_m;
    fusion_config.maximum_lateral_gap_m =
        odom_anchor_config_.tight_box.component_merge_lateral_gap_m;
    fusion_config.minimum_z_overlap_ratio =
        odom_anchor_config_.tight_box.component_merge_min_z_overlap_ratio;
    fusion_config.maximum_combined_long_side_m =
        odom_anchor_config_.max_size_x;
    fusion_config.maximum_combined_short_side_m =
        odom_anchor_config_.max_size_y;
    fusion_config.maximum_components = static_cast<std::size_t>(
        odom_anchor_config_.tight_box.component_merge_max_components);
    const std::vector<CargoComponentHypothesis> component_hypotheses =
        buildCargoComponentHypotheses(fragments, fusion_config);

    std::vector<CargoCandidateIdentityScore> component_scores;
    std::vector<pcl::PointIndices> hypothesis_point_indices;
    std::vector<std::size_t> hypothesis_component_counts;
    component_scores.reserve(component_hypotheses.size());
    hypothesis_point_indices.reserve(component_hypotheses.size());
    hypothesis_component_counts.reserve(component_hypotheses.size());
    for (const CargoComponentHypothesis& hypothesis :
         component_hypotheses) {
        std::vector<Eigen::Vector2f> footprint_points;
        std::vector<float> component_z;
        pcl::PointIndices combined_indices;
        for (std::size_t component_index :
             hypothesis.component_indices) {
            if (component_index >= components.size()) continue;
            for (int point_index : components[component_index].indices.indices) {
                const pcl::PointXYZ& point = voxel_cloud->points[point_index];
                footprint_points.emplace_back(point.x, point.y);
                component_z.push_back(point.z);
                combined_indices.indices.push_back(point_index);
            }
        }
        std::sort(component_z.begin(), component_z.end());
        const CargoOrientedFootprint footprint =
            estimateCargoOrientedFootprint(
                footprint_points, candidate_footprint_config);
        if (!footprint.valid || component_z.size() < 3U) continue;
        const float z_low = component_z[static_cast<std::size_t>(
            (component_z.size() - 1U) *
            odom_anchor_config_.tight_box.percentile_low)];
        const float z_high = component_z[static_cast<std::size_t>(
            (component_z.size() - 1U) *
            odom_anchor_config_.tight_box.percentile_high)];
        CargoCandidateDescriptor descriptor;
        descriptor.component_id = static_cast<int>(component_scores.size());
        descriptor.center = Eigen::Vector3f(
            footprint.center_base.x(), footprint.center_base.y(),
            0.5F * (z_low + z_high));
        descriptor.size = Eigen::Vector3f(
            footprint.size_long_short.x(), footprint.size_long_short.y(),
            std::max(0.01F, z_high - z_low));
        descriptor.yaw_rad = footprint.yaw_base_rad;
        descriptor.orientation_confidence =
            footprint.orientation_confidence;
        descriptor.point_count = combined_indices.indices.size();
        descriptor.suspension_evidence =
            result.ground_reference_valid &&
            z_low - result.ground_z >=
                hook_lock_config_.suspended_min_ground_clearance_m;
        const CargoCandidateIdentityScore identity =
            scoreCargoCandidateIdentity(descriptor, identity_context);
        component_scores.push_back(identity);
        hypothesis_point_indices.push_back(std::move(combined_indices));
        hypothesis_component_counts.push_back(
            hypothesis.component_indices.size());
        ROS_DEBUG_THROTTLE(
            2.0,
            "[CargoCandidate] id=%d points=%zu center=(%.2f,%.2f,%.2f) "
            "size=(%.2f,%.2f,%.2f) yaw=%.1f hook=%.2f predicted=%.2f "
            "overlap=%.2f shape=%.2f motion=%.2f suspension=%.2f "
            "identity=%.2f overall=%.2f",
            descriptor.component_id, descriptor.point_count,
            descriptor.center.x(), descriptor.center.y(),
            descriptor.center.z(), descriptor.size.x(), descriptor.size.y(),
            descriptor.size.z(), descriptor.yaw_rad * 180.0F /
                3.14159265358979323846F,
            identity.hook_distance_score,
            identity.predicted_center_score, identity.overlap_score,
            identity.shape_confidence, identity.motion_confidence,
            identity.suspension_confidence, identity.identity_confidence,
            identity.overall_lock_confidence);
    }
    const CargoCandidateRanking candidate_ranking =
        rankCargoCandidateIdentityScores(component_scores);
    if (!candidate_ranking.valid) {
        result.reject_reason = "no_identity_valid_component";
        return result;
    }
    const CargoCandidateIdentityScore& selected_identity =
        candidate_ranking.top1;
    const std::size_t selected_hypothesis_index = static_cast<std::size_t>(
        selected_identity.component_id);
    if (selected_hypothesis_index >= hypothesis_point_indices.size()) {
        result.reject_reason = "selected_hypothesis_index_invalid";
        return result;
    }
    result.selected_candidate_id =
        static_cast<int>(selected_hypothesis_index);
    result.candidate_count = component_scores.size();
    result.merged_component_count =
        hypothesis_component_counts[selected_hypothesis_index];
    result.candidate_top1_score = candidate_ranking.top1_rank;
    result.candidate_top2_score = candidate_ranking.top2_rank;
    result.candidate_score_margin = candidate_ranking.margin;
    result.identity_confidence = selected_identity.identity_confidence;
    result.shape_confidence = selected_identity.shape_confidence;
    result.motion_confidence = selected_identity.motion_confidence;
    result.suspension_confidence = selected_identity.suspension_confidence;
    result.overall_lock_confidence =
        selected_identity.overall_lock_confidence;
    const pcl::PointIndices& best_cluster =
        hypothesis_point_indices[selected_hypothesis_index];

    // 构建结果点云
    for (int idx : best_cluster.indices) {
        result.core_points_base->push_back(voxel_cloud->points[idx]);
    }

    // Geometry must consume the exact component selected by identity ranking.
    // A second nearest-anchor subcluster pass used to replace long cargo with
    // one local fragment and could freeze a completely different OBB/yaw.

    // Estimate the one formal footprint consumed by locking, bottom fusion,
    // safety, markers and authorized removal. A separate visual-only yaw is
    // deliberately forbidden because it recreates divergent geometry.
    if (odom_anchor_config_.tight_box.orientation_enabled) {
        std::vector<Eigen::Vector2f> footprint_points;
        footprint_points.reserve(result.core_points_base->size());
        for (const auto& point : result.core_points_base->points) {
            footprint_points.emplace_back(point.x, point.y);
        }
        CargoOrientedFootprintConfig footprint_config;
        footprint_config.minimum_points = static_cast<std::size_t>(std::max(
            3, odom_anchor_config_.tight_box.orientation_min_points));
        footprint_config.percentile_low =
            odom_anchor_config_.tight_box.percentile_low;
        footprint_config.percentile_high =
            odom_anchor_config_.tight_box.percentile_high;
        footprint_config.margin_m =
            odom_anchor_config_.tight_box.margin_xy_m;
        footprint_config.minimum_geometric_aspect_ratio =
            odom_anchor_config_.tight_box
                .orientation_min_geometric_aspect_ratio;
        footprint_config.minimum_eigenvalue_ratio =
            odom_anchor_config_.tight_box
                .orientation_min_eigenvalue_ratio;
        footprint_config.minimum_long_side_m = std::max(
            odom_anchor_config_.min_size_x, odom_anchor_config_.min_size_y);
        footprint_config.minimum_short_side_m = std::min(
            odom_anchor_config_.min_size_x, odom_anchor_config_.min_size_y);
        footprint_config.maximum_long_side_m = std::max(
            odom_anchor_config_.max_size_x, odom_anchor_config_.max_size_y);
        footprint_config.maximum_short_side_m = std::min(
            odom_anchor_config_.max_size_x, odom_anchor_config_.max_size_y);
        const CargoOrientedFootprint footprint =
            estimateCargoOrientedFootprint(footprint_points, footprint_config);
        result.oriented_footprint_valid = footprint.valid;
        if (footprint.valid) {
            result.footprint_center_base = footprint.center_base;
            result.footprint_length_width = footprint.size_long_short;
            result.footprint_yaw_base_rad = footprint.yaw_base_rad;
            result.orientation_confidence =
                footprint.orientation_confidence;
            result.visible_long_axis_span_m =
                footprint.size_long_short.x();
            constexpr float kCoverageBinM = 0.15F;
            std::set<int> occupied_long_bins;
            std::set<int> occupied_short_bins;
            const float cosine = std::cos(footprint.yaw_base_rad);
            const float sine = std::sin(footprint.yaw_base_rad);
            for (const pcl::PointXYZ& point :
                 result.core_points_base->points) {
                const Eigen::Vector2f delta(
                    point.x - footprint.center_base.x(),
                    point.y - footprint.center_base.y());
                const float local_long =
                    cosine * delta.x() + sine * delta.y();
                const float local_short =
                    -sine * delta.x() + cosine * delta.y();
                occupied_long_bins.insert(static_cast<int>(
                    std::floor(local_long / kCoverageBinM)));
                occupied_short_bins.insert(static_cast<int>(
                    std::floor(local_short / kCoverageBinM)));
            }
            const int expected_long_bins = std::max(
                1, static_cast<int>(std::ceil(
                    footprint.size_long_short.x() / kCoverageBinM)));
            const int expected_short_bins = std::max(
                1, static_cast<int>(std::ceil(
                    footprint.size_long_short.y() / kCoverageBinM)));
            result.long_axis_coverage_ratio = std::clamp(
                static_cast<float>(occupied_long_bins.size()) /
                    static_cast<float>(expected_long_bins),
                0.0F, 1.0F);
            result.short_axis_coverage_ratio = std::clamp(
                static_cast<float>(occupied_short_bins.size()) /
                    static_cast<float>(expected_short_bins),
                0.0F, 1.0F);
        }
        ROS_DEBUG_THROTTLE(
            2.0,
            "[CargoFootprint] valid=%d size=(%.2f,%.2f) yaw_deg=%.1f "
            "eigen_ratio=%.2f aspect=%.2f confidence=%.2f points=%zu "
            "reason=%s",
            footprint.valid ? 1 : 0, footprint.size_long_short.x(),
            footprint.size_long_short.y(),
            footprint.yaw_base_rad * 180.0F /
                3.14159265358979323846F,
            footprint.eigenvalue_ratio,
            footprint.geometric_aspect_ratio,
            footprint.orientation_confidence, footprint.finite_points,
            footprint.reason.c_str());
    }

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

    if (result.oriented_footprint_valid) {
        center_x = result.footprint_center_base.x();
        center_y = result.footprint_center_base.y();
        sx = result.footprint_length_width.x();
        sy = result.footprint_length_width.y();
    }

    result.center_base = Eigen::Vector3f(center_x, center_y, (z05 + z95) * 0.5f);
    result.size_visible = Eigen::Vector3f(sx, sy, sz);
    result.z05 = z05;
    result.z50 = (z05 + z95) * 0.5f;
    result.z95 = z95;
    result.visible_height = z95 - z05;
    result.xy_area = sx * sy;
    if (result.oriented_footprint_valid) {
        CargoCandidateDescriptor selected_descriptor;
        selected_descriptor.component_id =
            std::max(0, result.selected_candidate_id);
        selected_descriptor.center = result.center_base;
        selected_descriptor.size = Eigen::Vector3f(
            result.footprint_length_width.x(),
            result.footprint_length_width.y(),
            std::max(0.01F, result.visible_height));
        selected_descriptor.yaw_rad = result.footprint_yaw_base_rad;
        selected_descriptor.orientation_confidence =
            result.orientation_confidence;
        selected_descriptor.point_count = result.core_points_base->size();
        selected_descriptor.suspension_evidence =
            result.ground_reference_valid &&
            z05 - result.ground_z >=
                hook_lock_config_.suspended_min_ground_clearance_m;
        const CargoCandidateIdentityScore final_identity =
            scoreCargoCandidateIdentity(
                selected_descriptor, identity_context);
        if (final_identity.valid) {
            result.identity_confidence =
                final_identity.identity_confidence;
            result.shape_confidence = final_identity.shape_confidence;
            result.motion_confidence = final_identity.motion_confidence;
            result.suspension_confidence =
                final_identity.suspension_confidence;
            result.overall_lock_confidence =
                final_identity.overall_lock_confidence;
        }
    }
    result.valid = true;
    classify_outcome(true);

    if (debug_cfg_.debug_tight_box) {
        ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec,
            "[TightBox] raw=%zu hag=%zu voxel=%zu clusters=%zu sub_cluster=%s selected_points=%zu anchor=(%.2f,%.2f) center=(%.2f,%.2f) size=(%.2f,%.2f,%.2f) z=[%.2f,%.2f] mode=%s",
            cloud_base->size(), filtered_cloud->size(), voxel_cloud->size(),
            result.candidate_count,
            "disabled_identity_contract",
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

bool NdtSlamNode::isCompactLockStrongDetection(
    const HookCargoDetection& det) const {
    if (!hook_lock_config_.compact_lock_enabled || !det.valid ||
        !det.core_points_base) return false;
    return classifyCargoLockProfile(
        det.core_points_base->size(), det.visible_height, det.xy_area,
        true,
        hook_lock_config_.lock_strong_min_points,
        hook_lock_config_.lock_min_visible_height,
        hook_lock_config_.lock_min_xy_area,
        hook_lock_config_.compact_lock_enabled,
        hook_lock_config_.compact_min_points,
        hook_lock_config_.compact_min_visible_height,
        hook_lock_config_.compact_min_xy_area) ==
        CargoLockProfile::COMPACT_BODY;
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

    if (!hook_lock_.locked_shape.valid ||
        (!hook_lock_.live_pose.valid && !hook_lock_.has_last_accepted)) {
        *reject_reason = "association_reference_unavailable";
        return false;
    }

    // Restore the d9 stable association semantics: the rigid shape is frozen,
    // while each fresh component is compared with the last filtered live
    // center. A velocity extrapolation must not drag the gate onto another
    // nearby component after one noisy observation.
    Eigen::Vector2f reference_center;
    if (hook_lock_.live_pose.valid) {
        reference_center = hook_lock_.live_pose.center_base.head<2>();
    } else {
        reference_center = hook_lock_.last_accepted_center.head<2>();
    }
    Eigen::Vector2f detected_center;
    if (det.oriented_footprint_valid) {
        detected_center = det.footprint_center_base;
    } else {
        detected_center = det.center_base.head<2>();
    }
    const float center_distance =
        (detected_center - reference_center).norm();
    const bool strict_reacquisition =
        hook_lock_.state == HookCargoLockState::LOST_HOLD;
    const float center_gate = strict_reacquisition
        ? std::min(hook_lock_config_.locked_update_max_center_dist,
                   hook_lock_config_.reacquisition_max_xy_gate_m)
        : hook_lock_config_.locked_update_max_center_dist;
    hook_lock_.association_xy_gate_m = center_gate;
    hook_lock_.association_z_gate_m =
        std::numeric_limits<float>::infinity();
    if (!std::isfinite(center_distance) || center_distance > center_gate) {
        *reject_reason = "center_too_far";
        return false;
    }

    if (det.oriented_footprint_valid) {
        const float detected_length = det.footprint_length_width.x();
        const float detected_width = det.footprint_length_width.y();
        const float locked_length = hook_lock_.locked_shape.length_m;
        const float locked_width = hook_lock_.locked_shape.width_m;
        if (!std::isfinite(detected_length) ||
            !std::isfinite(detected_width) || detected_length <= 0.0F ||
            detected_width <= 0.0F || locked_length <= 0.0F ||
            locked_width <= 0.0F) {
            *reject_reason = "invalid_association_geometry";
            return false;
        }
        const float length_error =
            std::abs(detected_length - locked_length) / locked_length;
        const float width_error =
            std::abs(detected_width - locked_width) / locked_width;
        if (length_error > hook_lock_config_.size_change_max_ratio ||
            width_error > hook_lock_config_.size_change_max_ratio) {
            *reject_reason = "rigid_shape_size_mismatch";
            return false;
        }
        hook_lock_.observed_yaw_rad = det.footprint_yaw_base_rad;
        hook_lock_.yaw_residual_rad = cargoAxialYawDifference(
            det.footprint_yaw_base_rad,
            hook_lock_.locked_shape.yaw_base_rad);
        // Yaw is intentionally diagnostic-only after lock. Partial visible
        // faces can flip the raw PCA axis; they must not drop an otherwise
        // continuous rigid cargo track or rotate the frozen formal box.
        hook_lock_.yaw_used_as_hard_gate = false;
    }

    bool partial_height_mismatch = false;
    if (bottom.valid && hook_lock_.locked_shape.height_m > 0.0F &&
        std::isfinite(bottom.height)) {
        const float height_error = std::abs(
            bottom.height - hook_lock_.locked_shape.height_m) /
            hook_lock_.locked_shape.height_m;
        partial_height_mismatch =
            height_error > hook_lock_config_.size_change_max_ratio;
    }

    // A partial side view changes raw z05/z95 span without changing the rigid
    // cargo. Keep it associated; frozen thickness and the robust top surface
    // determine vertical pose instead of dropping into LOST_HOLD.
    *reject_reason = hook_lock_.yaw_residual_rad > 0.35F
        ? "consistent_soft_yaw_mismatch"
        : (partial_height_mismatch
               ? "consistent_partial_height_observation"
               : "consistent");
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

void NdtSlamNode::updateLockedHeightAfterAssociation(
    const HookCargoBottomEstimate& bottom, const ros::Time& stamp) {
    const LockedCargoHeightAction action = evaluateLockedCargoHeightAction(
        hook_lock_config_.freeze_geometry_after_lock,
        hook_lock_.has_good_height, bottom.valid);
    switch (action) {
    case LockedCargoHeightAction::IGNORE_INVALID:
        return;
    case LockedCargoHeightAction::INITIALIZE_ONCE:
        updateLockedHeight(bottom, stamp, true);
        hook_lock_.locked_shape.height_m = bottom.height;
        hook_lock_.locked_shape.valid =
            hook_lock_.locked_shape.length_m > 0.0F &&
            hook_lock_.locked_shape.width_m > 0.0F &&
            hook_lock_.locked_shape.height_m > 0.0F;
        ROS_WARN("[CargoLock] frozen height initialized after lock "
                 "bottom=%.2f top=%.2f",
                 bottom.bottom_z_base, bottom.top_z_base);
        return;
    case LockedCargoHeightAction::UPDATE_ADAPTIVE:
        updateLockedHeight(bottom, stamp, false);
        return;
    case LockedCargoHeightAction::REFRESH_FROZEN:
        hook_lock_.last_good_height_stamp = stamp;
        hook_lock_.bottom_uncertainty = std::min(
            hook_lock_.bottom_uncertainty, bottom.uncertainty);
        return;
    }
}

bool NdtSlamNode::cargoTrackRetained() const {
    return hook_lock_.state == HookCargoLockState::LOCKED ||
        hook_lock_.state == HookCargoLockState::LOST_HOLD;
}

void NdtSlamNode::updateLiveCargoPose(
    const HookCargoDetection& det,
    const HookCargoBottomEstimate& bottom,
    const ros::Time& stamp,
    CargoPoseSource source) {
    if (!det.valid || stamp.isZero()) return;
    Eigen::Vector3f measured = det.center_base;
    if (det.oriented_footprint_valid) {
        measured.x() = det.footprint_center_base.x();
        measured.y() = det.footprint_center_base.y();
    }
    const double stamp_sec = stamp.toSec();
    if (hook_lock_.live_pose.valid &&
        stamp_sec <= hook_lock_.live_pose.evidence_stamp_sec + 1.0e-4) {
        // A repeated physical sample is not a new motion observation.
        return;
    }
    const double measurement_dt = hook_lock_.live_pose.valid
        ? std::max(
              0.0, stamp_sec - hook_lock_.live_pose.evidence_stamp_sec)
        : 0.0;
    const float predicted_z = hook_lock_.live_pose.valid
        ? hook_lock_.live_pose.center_base.z() +
              std::clamp(
                  hook_lock_.live_pose_velocity_base.z(),
                  -hook_lock_config_.live_pose_max_z_speed_mps,
                  hook_lock_config_.live_pose_max_z_speed_mps) *
                  static_cast<float>(measurement_dt)
        : det.center_base.z();
    CargoVerticalPoseSource vertical_source =
        CargoVerticalPoseSource::PREDICTION;
    const bool frozen_height_valid = hook_lock_.locked_shape.valid &&
        hook_lock_.locked_shape.height_m > 0.0F;
    const bool direct_bottom = frozen_height_valid && bottom.valid &&
        bottom.source == "points_visible_side" &&
        std::isfinite(bottom.bottom_z_base);
    const CargoTopSurfaceHeightResult vertical_measurement =
        evaluateCargoTopSurfaceHeight({
            frozen_height_valid
                ? hook_lock_.locked_shape.height_m : 0.0F,
            hook_lock_config_.track_vertical_from_top_surface &&
                std::isfinite(det.z95),
            det.z95,
            direct_bottom,
            bottom.bottom_z_base,
            hook_lock_config_.top_bottom_center_agreement_m});
    const bool accepted_direct_bottom = direct_bottom &&
        (!vertical_measurement.used_top_surface ||
         vertical_measurement.bottom_corroborated);
    const bool freeze_vertical =
        hook_lock_config_.freeze_vertical_position_after_lock &&
        hook_lock_.live_pose.valid &&
        hook_lock_.live_pose.center_base.allFinite();
    if (freeze_vertical) {
        // The lifting operation treats the cargo as one rigid body at a
        // stable working height. Raw bottom/top extrema remain diagnostics,
        // but fragmented returns cannot pull the formal box above/below the
        // cargo or create a negative-bottom excursion.
        measured.z() = hook_lock_.live_pose.center_base.z();
        vertical_source = CargoVerticalPoseSource::DISPLAY_FROZEN;
        if (accepted_direct_bottom) {
            hook_lock_.direct_bottom_evidence_stamp = stamp;
        }
    } else if (vertical_measurement.valid &&
               vertical_measurement.used_top_surface) {
        // The overhead sensors consistently see the upper face while the
        // lower edge is frequently occluded. Derive absolute bottom/center
        // from robust z95 and the thickness frozen from the pre-lift window.
        measured.z() = vertical_measurement.center_z_base;
        vertical_source = CargoVerticalPoseSource::DIRECT_TOP;
        if (accepted_direct_bottom) {
            hook_lock_.direct_bottom_evidence_stamp = stamp;
        }
    } else if (vertical_measurement.valid) {
        measured.z() = vertical_measurement.center_z_base;
        vertical_source = CargoVerticalPoseSource::DIRECT_BOTTOM;
        hook_lock_.direct_bottom_evidence_stamp = stamp;
    } else if (frozen_height_valid && det.core_points_base &&
               !det.core_points_base->empty() &&
               hook_lock_.live_pose.valid) {
        std::vector<float> supported_z;
        supported_z.reserve(det.core_points_base->size());
        const float cosine = std::cos(hook_lock_.locked_shape.yaw_base_rad);
        const float sine = std::sin(hook_lock_.locked_shape.yaw_base_rad);
        for (const pcl::PointXYZ& point : det.core_points_base->points) {
            if (!pcl::isFinite(point)) continue;
            const Eigen::Vector2f delta = point.getVector3fMap().head<2>() -
                hook_lock_.live_pose.center_base.head<2>();
            const float local_x = cosine * delta.x() + sine * delta.y();
            const float local_y = -sine * delta.x() + cosine * delta.y();
            if (std::abs(local_x) <=
                    0.5F * hook_lock_.locked_shape.length_m + 0.15F &&
                std::abs(local_y) <=
                    0.5F * hook_lock_.locked_shape.width_m + 0.15F &&
                std::abs(point.z - predicted_z) <=
                    0.5F * hook_lock_.locked_shape.height_m + 0.20F) {
                supported_z.push_back(point.z);
            }
        }
        if (!supported_z.empty()) {
            const auto middle = supported_z.begin() +
                static_cast<std::ptrdiff_t>(supported_z.size() / 2U);
            std::nth_element(supported_z.begin(), middle, supported_z.end());
            const float weak_residual = std::clamp(
                *middle - predicted_z, -0.08F, 0.08F);
            measured.z() = predicted_z + weak_residual;
            vertical_source =
                CargoVerticalPoseSource::LOCKED_OBB_POINT_SUPPORT;
        } else {
            measured.z() = predicted_z;
        }
    } else {
        measured.z() = predicted_z;
    }
    if (!measured.allFinite()) return;
    if (!hook_lock_.live_pose.valid ||
        !hook_lock_.live_pose.center_base.allFinite()) {
        hook_lock_.live_pose.center_base = measured;
        hook_lock_.live_pose_velocity_base.setZero();
        hook_lock_.live_pose_predicted_base = measured;
        hook_lock_.live_pose_innovation_base.setZero();
        hook_lock_.live_pose_residual_base.setZero();
        hook_lock_.live_pose_dt_sec = 0.0;
    } else {
        const double dt = stamp_sec -
            hook_lock_.live_pose.evidence_stamp_sec;
        if (!std::isfinite(dt) || dt <= 0.0) return;
        const Eigen::Vector3f previous = hook_lock_.live_pose.center_base;
        const CargoLivePoseStepResult step = updateCargoLivePoseStep({
            previous, hook_lock_.live_pose_velocity_base, measured, dt,
            hook_lock_config_.live_pose_center_alpha,
            hook_lock_config_.live_pose_velocity_alpha,
            hook_lock_config_.live_pose_max_xy_speed_mps,
            hook_lock_config_.live_pose_max_z_speed_mps,
            hook_lock_config_.live_pose_step_margin_m});
        if (!step.valid) return;
        hook_lock_.live_pose.center_base = step.filtered_center;
        hook_lock_.live_pose_velocity_base = step.filtered_velocity;
        hook_lock_.live_pose_predicted_base = step.predicted_center;
        hook_lock_.live_pose_innovation_base = step.measurement_residual;
        hook_lock_.live_pose_residual_base = step.tracking_residual;
        hook_lock_.live_pose_dt_sec = dt;
        if (freeze_vertical) {
            hook_lock_.live_pose.center_base.z() = previous.z();
            hook_lock_.live_pose_velocity_base.z() = 0.0F;
            hook_lock_.live_pose_predicted_base.z() = previous.z();
            hook_lock_.live_pose_innovation_base.z() = 0.0F;
            hook_lock_.live_pose_residual_base.z() = 0.0F;
        }
    }
    hook_lock_.live_pose_measured_base = measured;
    hook_lock_.live_pose.valid = true;
    hook_lock_.live_pose.evidence_stamp_sec = stamp_sec;
    hook_lock_.live_pose.evaluation_stamp_sec = stamp_sec;
    hook_lock_.live_pose.source = source;
    hook_lock_.live_pose.vertical_source = vertical_source;
    const float horizontal_residual =
        hook_lock_.live_pose_residual_base.head<2>().norm();
    const float vertical_residual =
        std::abs(hook_lock_.live_pose_residual_base.z());
    hook_lock_.horizontal_tracking_residual_m = std::max(
        horizontal_residual,
        hook_lock_config_.residual_uncertainty_decay *
            hook_lock_.horizontal_tracking_residual_m);
    hook_lock_.vertical_tracking_residual_m = std::max(
        vertical_residual,
        hook_lock_config_.residual_uncertainty_decay *
            hook_lock_.vertical_tracking_residual_m);
    hook_lock_.live_pose.position_uncertainty_m = std::max(
        0.08F, hook_lock_.horizontal_tracking_residual_m);
    const double direct_bottom_age =
        hook_lock_.direct_bottom_evidence_stamp.isZero()
            ? hook_lock_config_.direct_bottom_soft_stale_sec
            : std::max(
                  0.0, (stamp -
                        hook_lock_.direct_bottom_evidence_stamp).toSec());
    const float bottom_age_uncertainty = std::min(
        hook_lock_config_.bottom_max_uncertainty,
        0.10F + 0.05F * static_cast<float>(std::max(
            0.0, direct_bottom_age -
                hook_lock_config_.direct_bottom_soft_stale_sec)));
    hook_lock_.vertical_pose_uncertainty_m = std::max(
        accepted_direct_bottom ? bottom.uncertainty : bottom_age_uncertainty,
        hook_lock_.vertical_tracking_residual_m);
    // Only a physical vertical measurement advances formal Z authority.
    // XY-only association, prediction and display retention must not refresh
    // this timestamp.
    const bool formal_vertical_measurement =
        vertical_source == CargoVerticalPoseSource::DIRECT_TOP ||
        vertical_source == CargoVerticalPoseSource::DIRECT_BOTTOM ||
        vertical_source ==
            CargoVerticalPoseSource::LOCKED_OBB_POINT_SUPPORT;
    if (formal_vertical_measurement) {
        hook_lock_.live_vertical_pose_valid = true;
        hook_lock_.live_vertical_pose_evidence_stamp = stamp;
    }
    hook_lock_.direct_bottom_support_valid = accepted_direct_bottom;
    hook_lock_.locked_center_base = hook_lock_.live_pose.center_base;
}

RigidCargoGeometry NdtSlamNode::buildCurrentRigidCargoGeometryForPose(
    const Sophus::SE3d& pose_map_base,
    const ros::Time& stamp) {
    LiveCargoPose live_pose = hook_lock_.live_pose;
    const bool current_association =
        hook_observation_associated_current_ &&
        hook_observation_association_stamp_ == stamp;
    if (cargoTrackRetained() && live_pose.valid && !current_association) {
        live_pose = propagateHeldCargoPose(
            live_pose, hook_lock_.live_pose_velocity_base, stamp.toSec(),
            hook_lock_config_.formal_xy_evidence_hold_sec,
            hook_lock_config_.live_pose_max_xy_speed_mps,
            hook_lock_config_.live_pose_max_z_speed_mps,
            hook_lock_config_.lost_velocity_decay_tau_sec,
            hook_lock_config_.lost_position_uncertainty_per_sec,
            hook_lock_config_.lost_position_uncertainty_max_m);
    }
    live_pose.evaluation_stamp_sec = stamp.toSec();
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    transform.matrix() = pose_map_base.matrix().cast<float>();
    current_rigid_cargo_geometry_ = ndt_slam::buildCurrentRigidCargoGeometry(
        hook_lock_.locked_shape, live_pose, transform,
        cargo_fusion_track_id_,
        std::max(hook_lock_.horizontal_tracking_residual_m,
                 live_pose.position_uncertainty_m),
        std::max(hook_lock_.vertical_tracking_residual_m,
                 hook_lock_.vertical_pose_uncertainty_m));
    current_rigid_cargo_geometry_.height_evidence_stamp_sec =
        hook_lock_.live_vertical_pose_valid &&
                !hook_lock_.live_vertical_pose_evidence_stamp.isZero()
            ? hook_lock_.live_vertical_pose_evidence_stamp.toSec()
            : (!hook_lock_.locked_stamp.isZero()
                   ? hook_lock_.locked_stamp.toSec()
                   : live_pose.evidence_stamp_sec);
    return current_rigid_cargo_geometry_;
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
    if (hook_lock_.locked_shape.valid && hook_lock_.live_pose.valid) {
        const ros::Time retirement_stamp =
            !hook_observation_association_stamp_.isZero() &&
                    (hook_lock_.last_seen_stamp.isZero() ||
                     hook_observation_association_stamp_ >=
                         hook_lock_.last_seen_stamp)
                ? hook_observation_association_stamp_
                : hook_lock_.last_seen_stamp;
        const double prediction_dt = retirement_stamp.isZero()
            ? 0.0
            : std::max(
                  0.0, retirement_stamp.toSec() -
                      hook_lock_.live_pose.evidence_stamp_sec);
        retired_cargo_shape_ = hook_lock_.locked_shape;
        retired_cargo_center_base_ = hook_lock_.live_pose.center_base +
            hook_lock_.live_pose_velocity_base *
                static_cast<float>(prediction_dt);
        retired_cargo_velocity_base_ =
            hook_lock_.live_pose_velocity_base;
        retired_cargo_stamp_ = retirement_stamp;
        retired_cargo_signature_valid_ = true;
    }
    hook_lock_.state = HookCargoLockState::EMPTY;
    hook_lock_.confirm_count = 0;
    hook_lock_.weak_count = 0;
    hook_lock_.lost_count = 0;
    hook_lock_.candidate_started_stamp = ros::Time();
    hook_lock_.last_any_candidate_stamp = ros::Time();
    hook_lock_.last_identity_consistent_stamp = ros::Time();
    hook_lock_.last_candidate_progress_stamp = ros::Time();
    hook_lock_.candidate_gap_frames = 0;
    hook_lock_.candidate_progress_count = 0;
    hook_lock_.challenger_confirm_count = 0;
    hook_lock_.challenger_candidate_id = -1;
    hook_lock_.has_locked_size = false;
    hook_lock_.locked_shape = LockedCargoShape{};
    hook_lock_.live_pose = LiveCargoPose{};
    hook_lock_.live_pose_velocity_base.setZero();
    hook_lock_.live_pose_measured_base.setZero();
    hook_lock_.live_pose_predicted_base.setZero();
    hook_lock_.live_pose_innovation_base.setZero();
    hook_lock_.live_pose_residual_base.setZero();
    hook_lock_.live_pose_dt_sec = 0.0;
    hook_lock_.horizontal_tracking_residual_m = 0.0F;
    hook_lock_.vertical_tracking_residual_m = 0.0F;
    hook_lock_.vertical_pose_uncertainty_m = 0.30F;
    hook_lock_.shape_height_valid = false;
    hook_lock_.live_vertical_pose_valid = false;
    hook_lock_.direct_bottom_support_valid = false;
    hook_lock_.live_vertical_pose_evidence_stamp = ros::Time();
    hook_lock_.direct_bottom_evidence_stamp = ros::Time();
    hook_lock_.locked_obb_support = CargoFrozenObbSupport{};
    hook_lock_.association_xy_gate_m = 0.0F;
    hook_lock_.association_z_gate_m = 0.0F;
    hook_lock_.observed_yaw_rad = 0.0F;
    hook_lock_.yaw_residual_rad = 0.0F;
    hook_lock_.yaw_used_as_hard_gate = false;
    hook_lock_.rearm_start_stamp = ros::Time();
    hook_lock_.rearm_gravity_state =
        lidar_slam2_msgs::HookLoadState::STATE_UNKNOWN;
    current_rigid_cargo_geometry_ = RigidCargoGeometry{};
    previous_self_mask_geometry_ = RigidCargoGeometry{};
    accepted_self_mask_geometry_ = RigidCargoGeometry{};
    hook_lock_.has_good_height = false;
    hook_lock_.size_update_count = 0;
    hook_lock_.init_size_buffer.clear();
    hook_lock_.init_oriented_size_buffer.clear();
    hook_lock_.init_oriented_yaw_buffer.clear();
    hook_lock_.init_orientation_confidence_buffer.clear();
    hook_lock_.provisional_observations.clear();
    hook_lock_.provisional_scores.clear();
    hook_lock_.provisional_summary = CargoProvisionalLockSummary{};
    hook_lock_.provisional_track_id = 0U;
    hook_lock_.provisional_velocity_base.setZero();
    hook_lock_.provisional_origin_center_base.setZero();
    hook_lock_.provisional_last_evidence_stamp_sec = 0.0;
    hook_lock_.suspension_confirm_count = 0;
    hook_lock_.lift_confirm_count = 0;
    hook_lock_.ground_clearance_m =
        std::numeric_limits<float>::quiet_NaN();
    hook_lock_.lift_from_origin_m = 0.0F;
    hook_lock_.lock_authority_source = CargoLockAuthoritySource::NONE;
    hook_lock_.size_candidate_buffer.clear();
    hook_lock_.last_accepted_core_points.reset();
    hook_lock_.last_accepted_center.setZero();
    hook_lock_.last_accepted_size.setZero();
    hook_lock_.candidate_compact_profile = false;
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
    const bool large_lock_observation = isLockStrongDetection(det, bottom);
    const bool compact_lock_observation =
        isCompactLockStrongDetection(det);
    // REQUIRED transitions are evaluated while holding the same mutex used by
    // the Gravity callback. This closes the outer-snapshot/inner-transition
    // race instead of relying on two independently timed snapshots.
    std::unique_lock<std::mutex> hook_policy_guard(
        hook_load_state_mutex_, std::defer_lock);
    HookLoadSnapshot hook;
    if (hook_load_signal_role_ == HookLoadSignalRole::REQUIRED &&
        hook_load_signal_enabled_) {
        hook_policy_guard.lock();
        hook = hook_load_snapshot_;
        const double now = ros::WallTime::now().toSec();
        const double receipt_age = now - hook.receipt_wall_sec;
        const double progress_age = now - hook.source_progress_wall_sec;
        if (!std::isfinite(receipt_age) || !std::isfinite(progress_age) ||
            hook.receipt_wall_sec <= 0.0 ||
            hook.source_progress_wall_sec <= 0.0 ||
            receipt_age < 0.0 || progress_age < 0.0 ||
            receipt_age > hook_load_state_stale_timeout_sec_ ||
            progress_age > hook_load_state_stale_timeout_sec_) {
            hook.valid = false;
            hook.state = lidar_slam2_msgs::HookLoadState::STATE_UNKNOWN;
            hook.reason = "consumer_signal_stale";
        }
    } else {
        hook = currentHookLoadSnapshot();
    }
    const HookLoadState gravity_state =
        hook.state == lidar_slam2_msgs::HookLoadState::STATE_LOADED
            ? HookLoadState::LOADED
            : (hook.state == lidar_slam2_msgs::HookLoadState::STATE_EMPTY
                ? HookLoadState::EMPTY
                : (hook.state == lidar_slam2_msgs::HookLoadState::STATE_INHIBIT
                    ? HookLoadState::INHIBIT : HookLoadState::UNKNOWN));
    const SuspendedCargoLockDecision candidate_policy =
        evaluateSuspendedCargoLock({
            hook_load_signal_role_, hook.valid, gravity_state,
            hook_lock_config_.lock_confirm_frames,
            det.lidar_lift_evidence});
    bool compact_size_continuous = true;
    if (compact_lock_observation && hook_lock_.has_last_accepted) {
        for (int axis = 0; axis < 3; ++axis) {
            const float reference = std::max(
                0.10F, hook_lock_.last_accepted_size[axis]);
            const float relative_step = std::abs(
                det.size_visible[axis] - reference) / reference;
            if (relative_step >
                hook_lock_config_.compact_max_size_relative_step) {
                compact_size_continuous = false;
                break;
            }
        }
    }

    ROS_DEBUG_THROTTLE(2.0,
        "[UpdateHookCargoLock] state=%d det.valid=%d points=%zu bottom.valid=%d",
        static_cast<int>(hook_lock_.state),
        det.valid ? 1 : 0,
        det.core_points_base ? det.core_points_base->size() : 0,
        bottom.valid ? 1 : 0);

    switch (hook_lock_.state) {
    case HookCargoLockState::EMPTY:
        if (candidate_policy.allow_candidate &&
            isStrongDetection(det, bottom)) {
            hook_lock_.state = HookCargoLockState::CANDIDATE;
            hook_lock_.confirm_count = 0;
            hook_lock_.init_size_buffer.clear();
            hook_lock_.init_oriented_size_buffer.clear();
            hook_lock_.init_oriented_yaw_buffer.clear();
            hook_lock_.init_orientation_confidence_buffer.clear();
            hook_lock_.last_accepted_center = det.center_base;
            hook_lock_.last_accepted_size = det.size_visible;
            hook_lock_.has_last_accepted = true;
            hook_lock_.last_seen_stamp = stamp;
            hook_lock_.candidate_started_stamp = stamp;
            hook_lock_.last_any_candidate_stamp = stamp;
            hook_lock_.last_identity_consistent_stamp = stamp;
            hook_lock_.last_candidate_progress_stamp = stamp;
            hook_lock_.candidate_progress_count = 1;
            hook_lock_.candidate_gap_frames = 0;
            ROS_INFO("[CargoLock] EMPTY->CANDIDATE confirm=0 points=%zu bottom=%.2f top=%.2f",
                     det.core_points_base ? det.core_points_base->size() : 0,
                     bottom.valid ? bottom.bottom_z_base : -1.0f,
                     bottom.valid ? bottom.top_z_base : -1.0f);
        }
        break;

    case HookCargoLockState::CANDIDATE:
        if (!candidate_policy.allow_candidate) {
            clearHookLock();
            ROS_WARN_THROTTLE(
                1.0, "[CargoLock] CANDIDATE->EMPTY policy=%s",
                candidate_policy.reason.c_str());
            break;
        }
        if (det.valid) hook_lock_.last_any_candidate_stamp = stamp;
        const bool candidate_formal_observation =
            det.oriented_footprint_valid &&
            bottom.valid &&
            (large_lock_observation ||
             (compact_lock_observation && compact_size_continuous));
        const bool candidate_spatially_consistent =
            candidate_formal_observation &&
            (det.center_base.head<2>() -
             hook_lock_.last_accepted_center.head<2>()).norm() <=
                hook_lock_config_.lock_max_center_step_m;
        if (candidate_spatially_consistent) {
            ++hook_lock_.candidate_progress_count;
            hook_lock_.candidate_gap_frames = 0;
            hook_lock_.last_identity_consistent_stamp = stamp;
            hook_lock_.last_candidate_progress_stamp = stamp;
            hook_lock_.last_accepted_center = det.center_base;
            hook_lock_.last_accepted_size = det.size_visible;
        } else if (det.valid) {
            ++hook_lock_.candidate_gap_frames;
        }
        // Two independent consistent observations are enough to begin the
        // 12-frame robust window. Ambiguous top-1 margins are resolved by the
        // window instead of blocking discovery for tens of seconds.
        if (candidate_spatially_consistent &&
            hook_lock_.candidate_progress_count >= 2) {
            hook_lock_.state = HookCargoLockState::GEOMETRY_CONFIRMING;
            hook_lock_.confirm_count = 1;
            hook_lock_.weak_count = 0;
            hook_lock_.candidate_compact_profile =
                !large_lock_observation && compact_lock_observation;
            hook_lock_.provisional_observations.clear();
            hook_lock_.provisional_scores.clear();
            ++next_provisional_track_id_;
            if (next_provisional_track_id_ == 0U) ++next_provisional_track_id_;
            hook_lock_.provisional_track_id = next_provisional_track_id_;
            CargoCandidateDescriptor observation;
            observation.component_id = std::max(
                0, det.selected_candidate_id);
            observation.center = det.center_base;
            observation.center.x() = det.footprint_center_base.x();
            observation.center.y() = det.footprint_center_base.y();
            observation.size = Eigen::Vector3f(
                det.footprint_length_width.x(),
                det.footprint_length_width.y(), bottom.height);
            observation.yaw_rad = det.footprint_yaw_base_rad;
            observation.orientation_confidence =
                det.orientation_confidence;
            observation.point_count = det.core_points_base
                ? det.core_points_base->size() : 0U;
            observation.suspension_evidence =
                det.lidar_lift_evidence;
            CargoCandidateIdentityScore score;
            score.valid = true;
            score.component_id = observation.component_id;
            score.shape_confidence = det.shape_confidence;
            score.motion_confidence = det.motion_confidence;
            score.suspension_confidence = det.suspension_confidence;
            score.identity_confidence = det.identity_confidence;
            score.overall_lock_confidence =
                det.overall_lock_confidence;
            score.reason = "detector_selected_component";
            hook_lock_.provisional_observations.push_back(observation);
            hook_lock_.provisional_scores.push_back(score);
            hook_lock_.provisional_origin_center_base = observation.center;
            hook_lock_.provisional_velocity_base.setZero();
            hook_lock_.provisional_last_evidence_stamp_sec = stamp.toSec();
            hook_lock_.ground_clearance_m =
                det.ground_reference_valid &&
                        std::isfinite(det.ground_z)
                    ? bottom.bottom_z_base - det.ground_z
                    : std::numeric_limits<float>::quiet_NaN();
            hook_lock_.suspension_confirm_count =
                det.lidar_lift_evidence ? 1 : 0;
            hook_lock_.lift_confirm_count = 0;
            hook_lock_.lift_from_origin_m = 0.0F;
            hook_lock_.last_accepted_center = observation.center;
            hook_lock_.last_accepted_size = observation.size;
            hook_lock_.last_seen_stamp = stamp;
            hook_lock_.last_candidate_progress_stamp = stamp;
            ROS_INFO(
                "[CargoLock] CANDIDATE->GEOMETRY_CONFIRMING "
                "track=%llu candidate=%d identity=%.2f overall=%.2f "
                "score_margin=%.2f",
                static_cast<unsigned long long>(
                    hook_lock_.provisional_track_id),
                observation.component_id, score.identity_confidence,
                score.overall_lock_confidence,
                det.candidate_score_margin);
            break;
        }
        const double candidate_age = hook_lock_.candidate_started_stamp.isZero()
            ? 0.0 : (stamp - hook_lock_.candidate_started_stamp).toSec();
        const double progress_age =
            hook_lock_.last_candidate_progress_stamp.isZero()
                ? candidate_age
                : (stamp - hook_lock_.last_candidate_progress_stamp).toSec();
        if (candidate_age >=
            hook_lock_config_.candidate_absolute_timeout_sec) {
            clearHookLock();
            ROS_INFO(
                "[CargoLock] CANDIDATE->EMPTY reason=absolute_timeout");
        } else if (progress_age >=
                   hook_lock_config_.candidate_progress_timeout_sec) {
            // Weak points may keep candidate_present true, but cannot refresh
            // progress. Re-seed a real observation or return to EMPTY.
            if (candidate_formal_observation) {
                hook_lock_.candidate_progress_count = 1;
                hook_lock_.candidate_gap_frames = 0;
                hook_lock_.last_candidate_progress_stamp = stamp;
                hook_lock_.last_identity_consistent_stamp = stamp;
                hook_lock_.last_accepted_center = det.center_base;
                hook_lock_.last_accepted_size = det.size_visible;
                ROS_INFO_THROTTLE(
                    5.0, "[CargoLock] candidate hypothesis reseeded "
                         "reason=progress_timeout candidate=%d",
                    det.selected_candidate_id);
            } else {
                clearHookLock();
                ROS_INFO(
                    "[CargoLock] CANDIDATE->EMPTY reason=progress_timeout");
            }
        }
        // Orientation concentration alone cannot grant formal lock authority.
        break;

    case HookCargoLockState::GEOMETRY_CONFIRMING: {
        if (!candidate_policy.allow_candidate) {
            hook_lock_.state = HookCargoLockState::CANDIDATE;
            hook_lock_.confirm_count = 0;
            hook_lock_.provisional_observations.clear();
            hook_lock_.provisional_scores.clear();
            hook_lock_.provisional_summary =
                CargoProvisionalLockSummary{};
            ROS_WARN_THROTTLE(
                1.0,
                "[CargoLock] GEOMETRY_CONFIRMING->CANDIDATE policy=%s",
                candidate_policy.reason.c_str());
            break;
        }
        const double geometry_age = hook_lock_.candidate_started_stamp.isZero()
            ? 0.0 : (stamp - hook_lock_.candidate_started_stamp).toSec();
        const double geometry_progress_age =
            hook_lock_.last_candidate_progress_stamp.isZero()
                ? geometry_age
                : (stamp - hook_lock_.last_candidate_progress_stamp).toSec();
        if (geometry_age >=
                hook_lock_config_.candidate_absolute_timeout_sec ||
            geometry_progress_age >=
                hook_lock_config_.candidate_progress_timeout_sec) {
            hook_lock_.state = HookCargoLockState::CANDIDATE;
            hook_lock_.confirm_count = 0;
            hook_lock_.candidate_progress_count = 0;
            hook_lock_.candidate_gap_frames = 0;
            hook_lock_.candidate_started_stamp = stamp;
            hook_lock_.last_candidate_progress_stamp = stamp;
            hook_lock_.provisional_observations.clear();
            hook_lock_.provisional_scores.clear();
            hook_lock_.provisional_summary = CargoProvisionalLockSummary{};
            ROS_INFO(
                "[CargoLock] GEOMETRY_CONFIRMING->CANDIDATE "
                "reason=%s",
                geometry_age >=
                        hook_lock_config_.candidate_absolute_timeout_sec
                    ? "absolute_timeout" : "progress_timeout");
            break;
        }
        const bool formal_observation = det.oriented_footprint_valid &&
            bottom.valid &&
            (large_lock_observation ||
             (compact_lock_observation && compact_size_continuous));
        if (!formal_observation) {
            hook_lock_.weak_count++;
            hook_lock_.candidate_gap_frames++;
            if (hook_lock_.candidate_gap_frames >
                    hook_lock_config_.candidate_max_gap_frames ||
                hook_lock_.weak_count >
                    hook_lock_config_.candidate_max_weak_frames) {
                hook_lock_.state = HookCargoLockState::CANDIDATE;
                hook_lock_.confirm_count = 0;
                hook_lock_.provisional_observations.clear();
                hook_lock_.provisional_scores.clear();
                hook_lock_.provisional_summary =
                    CargoProvisionalLockSummary{};
                ROS_INFO(
                    "[CargoLock] GEOMETRY_CONFIRMING->CANDIDATE "
                    "reason=observation_gap");
            }
            break;
        }
        CargoCandidateDescriptor observation;
        observation.component_id = std::max(0, det.selected_candidate_id);
        observation.center = det.center_base;
        observation.center.x() = det.footprint_center_base.x();
        observation.center.y() = det.footprint_center_base.y();
        observation.size = Eigen::Vector3f(
            det.footprint_length_width.x(),
            det.footprint_length_width.y(), bottom.height);
        observation.yaw_rad = det.footprint_yaw_base_rad;
        observation.orientation_confidence = det.orientation_confidence;
        observation.point_count = det.core_points_base
            ? det.core_points_base->size() : 0U;
        observation.suspension_evidence = det.lidar_lift_evidence;
        CargoCandidateIdentityScore score;
        score.valid = true;
        score.component_id = observation.component_id;
        score.shape_confidence = det.shape_confidence;
        score.motion_confidence = det.motion_confidence;
        score.suspension_confidence = det.suspension_confidence;
        score.identity_confidence = det.identity_confidence;
        score.overall_lock_confidence = det.overall_lock_confidence;
        score.reason = "detector_selected_component";

        // Geometry confirmation uses consecutive physical observations, not a
        // velocity prediction or a hard raw-yaw match.  The latter caused a
        // partial face to clear the window; the now-empty window then produced
        // "not_evaluated" forever.  Shape CV and orientation concentration
        // remain part of the final robust summary below.
        const bool fresh_stamp =
            stamp.toSec() >
                hook_lock_.provisional_last_evidence_stamp_sec + 1.0e-4;
        const float provisional_center_step =
            hook_lock_.provisional_observations.empty()
                ? 0.0F
                : (observation.center.head<2>() -
                   hook_lock_.provisional_observations.back()
                       .center.head<2>()).norm();
        const bool same_provisional_identity = fresh_stamp &&
            !hook_lock_.provisional_observations.empty() &&
            std::isfinite(provisional_center_step) &&
            provisional_center_step <=
                hook_lock_config_.lock_max_center_step_m;
        if (!fresh_stamp) {
            break;
        }
        if (!same_provisional_identity) {
            if (hook_lock_.challenger_candidate_id ==
                observation.component_id) {
                ++hook_lock_.challenger_confirm_count;
            } else {
                hook_lock_.challenger_candidate_id =
                    observation.component_id;
                hook_lock_.challenger_confirm_count = 1;
            }
            ++hook_lock_.candidate_gap_frames;
            const bool switch_confirmed =
                det.candidate_score_margin >=
                    hook_lock_config_.candidate_switch_margin &&
                hook_lock_.challenger_confirm_count >=
                    hook_lock_config_.candidate_switch_confirm_frames;
            if (switch_confirmed) {
                hook_lock_.provisional_observations.clear();
                hook_lock_.provisional_scores.clear();
                hook_lock_.provisional_summary =
                    CargoProvisionalLockSummary{};
                hook_lock_.confirm_count = 0;
                hook_lock_.provisional_velocity_base.setZero();
                hook_lock_.suspension_confirm_count = 0;
                hook_lock_.lift_confirm_count = 0;
                hook_lock_.provisional_observations.push_back(observation);
                hook_lock_.provisional_scores.push_back(score);
                hook_lock_.provisional_origin_center_base =
                    observation.center;
                hook_lock_.provisional_last_evidence_stamp_sec =
                    stamp.toSec();
                hook_lock_.confirm_count = 1;
                hook_lock_.last_seen_stamp = stamp;
                hook_lock_.last_accepted_center = observation.center;
                hook_lock_.last_accepted_size = observation.size;
                hook_lock_.has_last_accepted = true;
                hook_lock_.candidate_gap_frames = 0;
                hook_lock_.challenger_confirm_count = 0;
                hook_lock_.challenger_candidate_id = -1;
                hook_lock_.last_candidate_progress_stamp = stamp;
            }
            ROS_DEBUG_THROTTLE(
                1.0,
                "[CargoLock] provisional challenger track=%llu "
                "center_step=%.2f candidate=%d score_margin=%.2f "
                "streak=%d switched=%d",
                static_cast<unsigned long long>(
                    hook_lock_.provisional_track_id),
                provisional_center_step, observation.component_id,
                det.candidate_score_margin,
                hook_lock_.challenger_confirm_count,
                switch_confirmed ? 1 : 0);
            if (!switch_confirmed &&
                hook_lock_.candidate_gap_frames >
                    hook_lock_config_.candidate_max_gap_frames) {
                hook_lock_.state = HookCargoLockState::CANDIDATE;
                hook_lock_.candidate_progress_count = 0;
                hook_lock_.last_candidate_progress_stamp = stamp;
                hook_lock_.provisional_observations.clear();
                hook_lock_.provisional_scores.clear();
                hook_lock_.provisional_summary =
                    CargoProvisionalLockSummary{};
            }
            break;
        }
        hook_lock_.candidate_gap_frames = 0;
        hook_lock_.challenger_confirm_count = 0;
        hook_lock_.challenger_candidate_id = -1;
        const double provisional_dt = stamp.toSec() -
            hook_lock_.provisional_last_evidence_stamp_sec;
        if (provisional_dt > 1.0e-4) {
            Eigen::Vector3f measured_velocity =
                (observation.center -
                 hook_lock_.provisional_observations.back().center) /
                static_cast<float>(provisional_dt);
            const float xy_speed = measured_velocity.head<2>().norm();
            if (xy_speed > hook_lock_config_.live_pose_max_xy_speed_mps &&
                xy_speed > 1.0e-5F) {
                measured_velocity.head<2>() *=
                    hook_lock_config_.live_pose_max_xy_speed_mps / xy_speed;
            }
            measured_velocity.z() = std::clamp(
                measured_velocity.z(),
                -hook_lock_config_.live_pose_max_z_speed_mps,
                hook_lock_config_.live_pose_max_z_speed_mps);
            hook_lock_.provisional_velocity_base =
                0.5F * hook_lock_.provisional_velocity_base +
                0.5F * measured_velocity;
        }
        hook_lock_.provisional_last_evidence_stamp_sec = stamp.toSec();
        hook_lock_.ground_clearance_m =
            det.ground_reference_valid && std::isfinite(det.ground_z)
                ? bottom.bottom_z_base - det.ground_z
                : std::numeric_limits<float>::quiet_NaN();
        hook_lock_.lift_from_origin_m = observation.center.z() -
            hook_lock_.provisional_origin_center_base.z();
        hook_lock_.suspension_confirm_count =
            det.lidar_lift_evidence &&
                    std::isfinite(hook_lock_.ground_clearance_m) &&
                    hook_lock_.ground_clearance_m >=
                        hook_lock_config_.suspended_min_ground_clearance_m
                ? hook_lock_.suspension_confirm_count + 1
                : 0;
        hook_lock_.lift_confirm_count =
            hook_lock_.lift_from_origin_m >=
                    hook_lock_config_.minimum_lift_from_origin_m
                ? hook_lock_.lift_confirm_count + 1
                : 0;
        hook_lock_.provisional_observations.push_back(observation);
        hook_lock_.provisional_scores.push_back(score);
        const std::size_t maximum_window = static_cast<std::size_t>(
            hook_lock_config_.candidate_window_frames);
        if (hook_lock_.provisional_observations.size() > maximum_window) {
            hook_lock_.provisional_observations.erase(
                hook_lock_.provisional_observations.begin());
            hook_lock_.provisional_scores.erase(
                hook_lock_.provisional_scores.begin());
        }
        hook_lock_.confirm_count++;
        hook_lock_.weak_count = 0;
        hook_lock_.last_seen_stamp = stamp;
        hook_lock_.last_identity_consistent_stamp = stamp;
        hook_lock_.last_candidate_progress_stamp = stamp;
        hook_lock_.last_accepted_center = observation.center;
        hook_lock_.last_accepted_size = observation.size;
        hook_lock_.has_last_accepted = true;
        const int role_required_frames =
            evaluateSuspendedCargoLock({
                hook_load_signal_role_, hook.valid, gravity_state,
                hook_lock_.candidate_compact_profile
                    ? hook_lock_config_.compact_confirm_frames
                    : hook_lock_config_.lock_confirm_frames,
                det.lidar_lift_evidence})
                .required_confirm_frames;
        CargoProvisionalLockConfig provisional_config;
        provisional_config.minimum_frames = static_cast<std::size_t>(
            std::max({hook_lock_config_.candidate_required_consistent_frames,
                      role_required_frames,
                      odom_anchor_config_.tight_box
                          .orientation_min_confirm_frames}));
        provisional_config.maximum_center_step_m =
            hook_lock_config_.lock_max_center_step_m;
        provisional_config.maximum_shape_cv =
            hook_lock_config_.maximum_provisional_shape_cv;
        provisional_config.minimum_orientation_concentration =
            odom_anchor_config_.tight_box.orientation_min_concentration;
        provisional_config.minimum_identity_confidence =
            hook_lock_config_.minimum_identity_confidence;
        provisional_config.minimum_overall_lock_confidence =
            hook_lock_config_.minimum_overall_lock_confidence;
        hook_lock_.provisional_summary = summarizeCargoProvisionalLock(
            hook_lock_.provisional_observations,
            hook_lock_.provisional_scores, provisional_config);
        const CargoPhysicalLockAuthorityDecision physical_authority =
            evaluateCargoPhysicalLockAuthority({
                hook_load_signal_role_, hook.valid, gravity_state,
                hook_lock_.ground_clearance_m,
                hook_lock_config_.suspended_min_ground_clearance_m,
                hook_lock_.lift_from_origin_m,
                hook_lock_config_.minimum_lift_from_origin_m,
                hook_lock_.suspension_confirm_count,
                hook_lock_.lift_confirm_count,
                hook_lock_config_.suspension_confirm_frames});
        ROS_DEBUG_THROTTLE(
            1.0,
            "[CargoProvisional] frames=%zu identity=%.2f shape=%.2f "
            "motion=%.2f orientation=%.2f suspension=%.2f overall=%.2f "
            "reason=%s",
            hook_lock_.provisional_observations.size(),
            hook_lock_.provisional_summary.identity_confidence,
            hook_lock_.provisional_summary.shape_confidence,
            hook_lock_.provisional_summary.motion_confidence,
            hook_lock_.provisional_summary.orientation_confidence,
            hook_lock_.provisional_summary.suspension_confidence,
            hook_lock_.provisional_summary.overall_lock_confidence,
            hook_lock_.provisional_summary.reason.c_str());
        if (!hook_lock_.provisional_summary.formal_lock_allowed ||
            !candidate_policy.allow_lock ||
            !physical_authority.allowed) {
            ROS_DEBUG_THROTTLE(
                1.0,
                "[CargoProvisionalAuthority] track=%llu geometry=%d "
                "authority=%s clearance=%.2f lift=%.2f "
                "suspended_frames=%d lift_frames=%d margin=%.2f reason=%s",
                static_cast<unsigned long long>(
                    hook_lock_.provisional_track_id),
                hook_lock_.provisional_summary.formal_lock_allowed ? 1 : 0,
                cargoLockAuthoritySourceName(physical_authority.source),
                hook_lock_.ground_clearance_m,
                hook_lock_.lift_from_origin_m,
                hook_lock_.suspension_confirm_count,
                hook_lock_.lift_confirm_count,
                det.candidate_score_margin,
                !candidate_policy.allow_lock
                    ? candidate_policy.reason.c_str()
                    : (physical_authority.allowed
                        ? hook_lock_.provisional_summary.reason.c_str()
                        : physical_authority.reason.c_str()));
            break;
        }

        hook_lock_.state = HookCargoLockState::LOCKED;
        hook_lock_.lock_authority_source = physical_authority.source;
        observation_associated = true;
        const CargoProvisionalLockSummary& summary =
            hook_lock_.provisional_summary;
        hook_lock_.locked_shape.length_m = summary.median_size.x();
        hook_lock_.locked_shape.width_m = summary.median_size.y();
        hook_lock_.locked_shape.height_m = summary.median_size.z();
        hook_lock_.locked_shape.yaw_base_rad =
            hook_lock_config_.axis_aligned_yaw_after_lock
                ? quantizeCargoAxialYawToOrthogonal(summary.axial_yaw_rad)
                : summary.axial_yaw_rad;
        hook_lock_.locked_shape.orientation_confidence =
            summary.orientation_confidence;
        hook_lock_.locked_shape.valid =
            hook_lock_.locked_shape.length_m >=
                hook_lock_.locked_shape.width_m &&
            hook_lock_.locked_shape.width_m > 0.0F &&
            hook_lock_.locked_shape.height_m > 0.0F;
        hook_lock_.shape_height_valid =
            hook_lock_.locked_shape.height_m > 0.0F;
        hook_lock_.locked_size = summary.median_size;
        hook_lock_.has_locked_size = hook_lock_.locked_shape.valid;
        hook_lock_.locked_stamp = stamp;
        hook_lock_.last_accepted_core_points = det.core_points_base;
        hook_lock_.last_accepted_center = observation.center;
        hook_lock_.last_accepted_size = observation.size;
        hook_lock_.has_last_accepted = true;
        HookCargoBottomEstimate lock_bottom = bottom;
        lock_bottom.valid = summary.median_center.allFinite() &&
            summary.median_size.z() > 0.0F;
        lock_bottom.bottom_z_base = summary.median_center.z() -
            0.5F * summary.median_size.z();
        lock_bottom.top_z_base = summary.median_center.z() +
            0.5F * summary.median_size.z();
        lock_bottom.height = summary.median_size.z();
        lock_bottom.source = "provisional_median";
        updateLockedHeight(lock_bottom, stamp, true);
        HookCargoDetection lock_detection = det;
        lock_detection.center_base = summary.median_center;
        lock_detection.footprint_center_base =
            summary.median_center.head<2>();
        lock_detection.z95 = lock_bottom.top_z_base;
        updateLiveCargoPose(
            lock_detection, lock_bottom, stamp,
            CargoPoseSource::CURRENT_ASSOCIATED_LIDAR);
        ROS_WARN(
            "[CARGO_STATE] GEOMETRY_CONFIRMING->LOCKED "
            "shape=(%.2f,%.2f,%.2f) yaw_deg=%.1f "
            "orientation=%.2f identity=%.2f shape_conf=%.2f "
            "motion=%.2f overall=%.2f center=(%.2f,%.2f,%.2f) "
            "authority=%s clearance=%.2f lift=%.2f margin=%.2f track=%llu",
            hook_lock_.locked_shape.length_m,
            hook_lock_.locked_shape.width_m,
            hook_lock_.locked_shape.height_m,
            hook_lock_.locked_shape.yaw_base_rad * 180.0F /
                3.14159265358979323846F,
            summary.orientation_confidence,
            summary.identity_confidence, summary.shape_confidence,
            summary.motion_confidence, summary.overall_lock_confidence,
            hook_lock_.live_pose.center_base.x(),
            hook_lock_.live_pose.center_base.y(),
            hook_lock_.live_pose.center_base.z(),
            cargoLockAuthoritySourceName(hook_lock_.lock_authority_source),
            hook_lock_.ground_clearance_m,
            hook_lock_.lift_from_origin_m,
            det.candidate_score_margin,
            static_cast<unsigned long long>(
                hook_lock_.provisional_track_id));
        break;
    }

    case HookCargoLockState::LOCKED:
        if (strong || weak) {
            // Association gate：检查检测是否与 locked box 一致
            std::string reject_reason;
            bool accepted = isDetectionConsistentWithLockedBox(det, bottom, &reject_reason);

            if (accepted) {
                observation_associated = true;
                hook_lock_.last_seen_stamp = stamp;
                // 更新高度和尺寸
                updateLockedHeightAfterAssociation(bottom, stamp);
                updateLiveCargoPose(
                    det, bottom, stamp,
                    CargoPoseSource::CURRENT_ASSOCIATED_LIDAR);
                if (!hook_lock_config_.freeze_geometry_after_lock && strong) {
                    maybeUpdateLockedSize(det, bottom);
                }

                // 更新 last accepted
                if (det.core_points_base && !det.core_points_base->empty()) {
                    hook_lock_.last_accepted_core_points = det.core_points_base;
                    hook_lock_.last_accepted_center = det.center_base;
                    hook_lock_.last_accepted_size = det.size_visible;
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
                updateLockedHeightAfterAssociation(bottom, stamp);
                updateLiveCargoPose(
                    det, bottom, stamp,
                    CargoPoseSource::CURRENT_ASSOCIATED_LIDAR);
                if (!hook_lock_config_.freeze_geometry_after_lock && strong) {
                    maybeUpdateLockedSize(det, bottom);
                }
                ROS_WARN("[CARGO_STATE] LOST_HOLD->LOCKED track=%llu",
                         static_cast<unsigned long long>(
                             cargo_fusion_track_id_));
            } else {
                growUncertainty();
                const double lost_age =
                    (stamp - hook_lock_.last_seen_stamp).toSec();
                ROS_DEBUG_THROTTLE(
                    2.0,
                    "[CargoLock] LOST_HOLD recovery rejected age=%.2f reason=%s",
                    lost_age, reject_reason.c_str());
                if (lost_age > hook_lock_config_.lost_clear_sec) {
                    clearHookLock();
                    hook_lock_.state =
                        HookCargoLockState::CLEAR_WAIT_REARM;
                    hook_lock_.rearm_start_stamp = stamp;
                    hook_lock_.rearm_gravity_state = hook.state;
                    ROS_WARN("[CARGO_STATE] LOST_HOLD->CLEAR reason=timeout");
                }
            }
        } else {
            double lost_age = (stamp - hook_lock_.last_seen_stamp).toSec();
            growUncertainty();
            if (lost_age > hook_lock_config_.lost_clear_sec) {
                clearHookLock();
                hook_lock_.state = HookCargoLockState::CLEAR_WAIT_REARM;
                hook_lock_.rearm_start_stamp = stamp;
                hook_lock_.rearm_gravity_state = hook.state;
                ROS_WARN("[CARGO_STATE] LOST_HOLD->CLEAR reason=timeout "
                         "age=%.1f", lost_age);
            }
        }
        break;

    case HookCargoLockState::CLEAR_WAIT_REARM: {
        const LidarNoCargoEvidenceResult empty_evidence =
            lidar_no_cargo_evidence_.result();
        const double rearm_age = hook_lock_.rearm_start_stamp.isZero()
            ? 0.0 : std::max(
                0.0, (stamp - hook_lock_.rearm_start_stamp).toSec());
        const CargoRearmDecision rearm = evaluateCargoRearm({
            empty_evidence.confirmed,
            rearm_age,
            hook_lock_config_.rearm_empty_confirm_sec,
            hook.valid,
            gravity_state,
            static_cast<HookLoadState>(
                hook_lock_.rearm_gravity_state),
            det.valid,
            det.lidar_lift_evidence,
            det.candidate_score_margin,
            hook_lock_config_.minimum_candidate_score_margin,
            det.identity_confidence,
            hook_lock_config_.minimum_identity_confidence});
        if (rearm.allowed) {
            hook_lock_.state = HookCargoLockState::EMPTY;
            hook_lock_.rearm_start_stamp = ros::Time();
            retired_cargo_shape_ = LockedCargoShape{};
            retired_cargo_center_base_.setZero();
            retired_cargo_velocity_base_.setZero();
            retired_cargo_stamp_ = ros::Time();
            retired_cargo_signature_valid_ = false;
            ROS_INFO(
                "[CargoLock] CLEAR_WAIT_REARM->EMPTY reason=%s age=%.2f",
                rearm.reason.c_str(), rearm_age);
        }
        break;
    }
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
    case HookCargoLockState::CLEAR_WAIT_REARM:
        cargo_state_.state = CargoState::EMPTY;
        cargo_state_.valid_geometry = false;
        cargo_state_.valid_height = false;
        break;

    case HookCargoLockState::CANDIDATE:
    case HookCargoLockState::GEOMETRY_CONFIRMING:
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

            if (hook_lock_config_.freeze_geometry_after_lock &&
                hook_lock_.has_locked_size) {
                cargo_state_.center_base = hook_lock_.live_pose.center_base;
                cargo_state_.size = Eigen::Vector3f(
                    hook_lock_.locked_shape.length_m,
                    hook_lock_.locked_shape.width_m,
                    hook_lock_.locked_shape.height_m);
                cargo_state_.valid_geometry =
                    hook_lock_.locked_shape.length_m > 0.0F &&
                    hook_lock_.locked_shape.width_m > 0.0F &&
                    hook_lock_.live_pose.valid &&
                    hook_lock_.live_pose.center_base.allFinite();
                cargo_state_.source = "rigid_shape_live_pose";
            } else {
                // Legacy adaptive mode remains available for deployments where
                // the tracked object is not rigidly suspended.
                if (cargo_state_.valid_geometry) {
                    constexpr float kCenterAlpha = 0.25F;
                    constexpr float kMaxCenterStep = 0.08F;
                    const Eigen::Vector3f center_diff =
                        det.center_base - cargo_state_.center_base;
                    for (int i = 0; i < 2; ++i) {
                        const float step = std::clamp(
                            center_diff(i), -kMaxCenterStep, kMaxCenterStep);
                        cargo_state_.center_base(i) += kCenterAlpha * step;
                    }

                    constexpr float kSizeAlpha = 0.30F;
                    constexpr float kMaxSizeStep = 0.10F;
                    const Eigen::Vector3f size_diff =
                        det.size_visible - cargo_state_.size;
                    for (int i = 0; i < 3; ++i) {
                        const float step = std::clamp(
                            size_diff(i), -kMaxSizeStep, kMaxSizeStep);
                        cargo_state_.size(i) += kSizeAlpha * step;
                    }
                } else {
                    cargo_state_.center_base = det.center_base;
                    cargo_state_.size = det.size_visible;
                }
                cargo_state_.valid_geometry = true;
                cargo_state_.source = "tight_box";
            }
        }

        // 更新高度（带稳定保护）
        if (hook_lock_config_.freeze_geometry_after_lock &&
            hook_lock_.locked_shape.valid &&
            hook_lock_.live_pose.valid && hook_lock_.has_good_height) {
            cargo_state_.bottom_z = hook_lock_.live_pose.center_base.z() -
                0.5F * hook_lock_.locked_shape.height_m;
            cargo_state_.top_z = hook_lock_.live_pose.center_base.z() +
                0.5F * hook_lock_.locked_shape.height_m;
            cargo_state_.bottom_unc = hook_lock_.bottom_uncertainty;
            cargo_state_.center_base = hook_lock_.live_pose.center_base;
            cargo_state_.size.z() = hook_lock_.locked_shape.height_m;
            cargo_state_.valid_height = true;
            hook_lock_.stable_bottom_z = cargo_state_.bottom_z;
            hook_lock_.stable_top_z = cargo_state_.top_z;
            hook_lock_.stable_height = hook_lock_.locked_shape.height_m;
        } else if (bottom.valid) {
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
            // TightBox z is geometry-only evidence. It must never become an
            // authorized physical bottom before CargoBottomFusion accepts it.
            cargo_state_.bottom_z = det.z05;
            cargo_state_.top_z = det.z95;
            cargo_state_.bottom_unc = 0.18f;
            cargo_state_.valid_height = false;
            cargo_state_.source = "tight_box_z_unverified";
        }

        // 计算 bottom_safe_z
        if (cargo_state_.valid_height) {
            cargo_state_.bottom_safe_z = cargo_state_.bottom_z - cargo_state_.bottom_unc - 0.05f;
        }

        // ========== 同步 CargoState 回 hook_lock_ ==========
        // 让所有下游（Marker、Removal、Warning）使用同一份数据
        if (cargo_state_.valid_geometry &&
            !hook_lock_config_.freeze_geometry_after_lock) {
            hook_lock_.locked_center_base = cargo_state_.center_base;
            hook_lock_.last_accepted_center = cargo_state_.center_base;
            hook_lock_.locked_size = cargo_state_.size;
            hook_lock_.has_locked_size = true;
        }

        if (cargo_state_.valid_height &&
            !hook_lock_config_.freeze_geometry_after_lock) {
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
    const CargoBottomResult& bottom, const ros::Time& stamp,
    bool explicit_empty, bool localization_valid) {
    CargoBoxGeometry visual_geometry = bottom.geometry;
    const RigidCargoGeometry rigid_geometry =
        buildCurrentRigidCargoGeometryForPose(current_pose_, stamp);
    const bool rigid_suspended_track = cargoTrackRetained() &&
        rigid_geometry.valid;
    if (rigid_suspended_track) {
        visual_geometry = CargoBoxGeometry{};
        visual_geometry.bottom_z_base = rigid_geometry.bottom_z_base;
        visual_geometry.top_z_base = rigid_geometry.top_z_base;
        visual_geometry.center_base = rigid_geometry.pose.center_base;
        visual_geometry.size_base = Eigen::Vector3f(
            rigid_geometry.shape.length_m,
            rigid_geometry.shape.width_m,
            rigid_geometry.shape.height_m);
        visual_geometry.corners_base = rigid_geometry.corners_base;
        visual_geometry.corners_map = rigid_geometry.corners_map;
        visual_geometry.center_map.setZero();
        for (const Eigen::Vector3f& corner : rigid_geometry.corners_map) {
            visual_geometry.center_map += corner;
        }
        visual_geometry.center_map /= 8.0F;
        visual_geometry.size_map = rigid_geometry.aabb_max_map -
            rigid_geometry.aabb_min_map;
        visual_geometry.bottom_z_map = rigid_geometry.aabb_min_map.z();
        visual_geometry.top_z_map = rigid_geometry.aabb_max_map.z();
        visual_geometry.valid = true;
    }

    CargoMarkerLifecycleInput lifecycle_input;
    lifecycle_input.stamp_sec = stamp.toSec();
    lifecycle_input.explicit_empty = explicit_empty;
    lifecycle_input.localization_valid = localization_valid;
    lifecycle_input.geometry_valid = rigid_suspended_track ||
        (bottom.geometry_valid && bottom.geometry.valid);
    lifecycle_input.safety_height_valid = bottom.valid;
    lifecycle_input.geometry = visual_geometry;
    CargoMarkerLifecycleDecision lifecycle =
        cargo_marker_lifecycle_.update(lifecycle_input);

    if (lifecycle.show && lifecycle.using_last_good_geometry &&
        localization_valid) {
        // The cargo is fixed in base/hook coordinates. During a short bottom
        // evidence gap, keep following the runtime pose instead of freezing
        // the last box in map coordinates.
        for (std::size_t i = 0U;
             i < lifecycle.geometry.corners_base.size(); ++i) {
            lifecycle.geometry.corners_map[i] =
                (current_pose_ *
                 lifecycle.geometry.corners_base[i].cast<double>()).cast<float>();
        }
    }

    if (cargo_fused_box_marker_pub_.getNumSubscribers() == 0U) return;

    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = "cargo_fused_box";
    marker.id = 0;
    marker.action = lifecycle.show
        ? visualization_msgs::Marker::ADD
        : visualization_msgs::Marker::DELETE;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    if (lifecycle.style == CargoMarkerStyle::LOCALIZATION_DEGRADED) {
        marker.color.r = 0.65F;
        marker.color.g = 0.65F;
        marker.color.b = 0.65F;
    } else if (lifecycle.style == CargoMarkerStyle::HEIGHT_DEGRADED) {
        marker.color.r = 1.0F;
        marker.color.g = 0.55F;
        marker.color.b = 0.05F;
    } else {
        marker.color.r = bottom.source == CargoBottomSource::POINTS
            ? 0.1F : 1.0F;
        marker.color.g = bottom.source == CargoBottomSource::POINTS
            ? 1.0F : 0.7F;
        marker.color.b = 0.1F;
    }
    marker.color.a = 0.95F;
    if (lifecycle.show) {
        constexpr int kEdges[12][2] = {
            {0, 1}, {1, 3}, {3, 2}, {2, 0},
            {4, 5}, {5, 7}, {7, 6}, {6, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7}};
        marker.points.reserve(24U);
        for (const auto& edge : kEdges) {
            for (int endpoint : edge) {
                geometry_msgs::Point point;
                const Eigen::Vector3f& corner =
                    lifecycle.geometry.corners_map[
                        static_cast<std::size_t>(endpoint)];
                point.x = corner.x();
                point.y = corner.y();
                point.z = corner.z();
                marker.points.push_back(point);
            }
        }
    }
    cargo_fused_box_marker_pub_.publish(marker);
}

lidar_slam2_msgs::CargoSafetyStatus NdtSlamNode::composeCargoSafetyStatus(
    lidar_slam2_msgs::CargoSafetyStatus status,
    bool visual_conflict,
    CargoSafetyFault evaluator_fault,
    std::uint16_t warning_code,
    bool warning_valid,
    const std::string& evidence_reason,
    bool evidence_initialized) const {
    using Status = lidar_slam2_msgs::CargoSafetyStatus;

    const bool system_ready = running_.load() &&
        !status.header.stamp.isZero() && evidence_initialized;
    const bool localization_valid =
        !tracking_lost_.load() && relocalization_pose_reliable_ &&
        (relocalization_state_ == RelocalizationState::IDLE ||
         !relocalization_enabled_);
    const bool gravity_valid = status.hook_signal_valid &&
        std::isfinite(status.hook_voltage) &&
        (status.hook_load_state == Status::HOOK_EMPTY ||
         status.hook_load_state == Status::HOOK_LOADED);
    status.hook_signal_role =
        static_cast<std::uint8_t>(hook_load_signal_role_);
    HookLoadEvidenceInput evidence_input;
    evidence_input.role = hook_load_signal_role_;
    evidence_input.gravity_valid = gravity_valid;
    evidence_input.gravity_state =
        static_cast<HookLoadState>(status.hook_load_state);
    evidence_input.lidar_cargo_valid = status.cargo_valid;
    evidence_input.lidar_no_cargo_confirmed =
        status.no_cargo_confirmed && !visual_conflict;
    evidence_input.lidar_track_locked = status.cargo_valid;
    evidence_input.lidar_geometry_valid = status.cargo_valid;
    evidence_input.lidar_height_valid = status.cargo_valid;
    const HookLoadEvidenceDecision hook_evidence =
        evaluateHookLoadEvidence(evidence_input);
    status.hook_signal_conflict = hook_evidence.gravity_conflict;
    const bool hook_empty = gravity_valid &&
        status.hook_load_state == Status::HOOK_EMPTY;
    const bool safe_empty = hook_evidence.lidar_empty_accepted;
    const bool lidar_cargo_accepted = hook_evidence.lidar_cargo_accepted;

    const bool cargo_fault =
        (hook_load_signal_role_ == HookLoadSignalRole::REQUIRED &&
         hook_empty && visual_conflict) ||
        (!safe_empty &&
         evaluator_fault == CargoSafetyFault::CARGO_HEIGHT_INVALID);
    const bool obstacle_fault = lidar_cargo_accepted &&
        (!status.obstacle_valid ||
         evaluator_fault == CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID);
    const bool warning_code_valid =
        warning_code == CargoSafetyEvaluator::kSafeCode ||
        warning_code == CargoSafetyEvaluator::kLevel1Code ||
        warning_code == CargoSafetyEvaluator::kLevel2Code;
    const bool warning_contract_error = lidar_cargo_accepted &&
        evaluator_fault == CargoSafetyFault::NONE &&
        (!warning_valid || !warning_code_valid);
    const bool finite_contract_error =
        (status.cargo_valid &&
         (!std::isfinite(status.cargo_bottom_z_map) ||
          !std::isfinite(status.cargo_bottom_uncertainty_m))) ||
        (status.obstacle_valid && status.obstacle_count > 0U &&
         (!std::isfinite(status.nearest_obstacle_distance_m) ||
          !std::isfinite(status.obstacle_top_z_map) ||
          !std::isfinite(status.obstacle_uncertainty_m) ||
          !std::isfinite(status.conservative_vertical_clearance_m)));
    const bool internal_fault = cargo_safety_config_error_ ||
        !hook_load_signal_role_config_valid_ ||
        evaluator_fault == CargoSafetyFault::INTERNAL_ERROR ||
        warning_contract_error || finite_contract_error;

    static_assert(Status::CODE_CLEAR == CargoSafetyProtocol::kClear,
                  "Cargo safety clear-code mismatch");
    static_assert(Status::CODE_LEVEL1_WARNING ==
                      CargoSafetyProtocol::kLevel1Warning &&
                  Status::CODE_LEVEL2_WARNING ==
                      CargoSafetyProtocol::kLevel2Warning &&
                  Status::CODE_SYSTEM_NOT_READY ==
                      CargoSafetyProtocol::kSystemNotReady &&
                  Status::CODE_LOCALIZATION_INVALID ==
                      CargoSafetyProtocol::kLocalizationInvalid &&
                  Status::CODE_GRAVITY_INVALID ==
                      CargoSafetyProtocol::kGravityInvalid &&
                  Status::CODE_CARGO_INVALID ==
                      CargoSafetyProtocol::kCargoInvalid &&
                  Status::CODE_OBSTACLE_INVALID ==
                      CargoSafetyProtocol::kObstacleInvalid &&
                  Status::CODE_INTERNAL_ERROR ==
                      CargoSafetyProtocol::kInternalError,
                  "Cargo safety status-code mismatch");
    static_assert(Status::FAULT_STATUS_STALE ==
                      CargoSafetyProtocol::kFaultStatusStale &&
                  Status::FAULT_LOCALIZATION ==
                      CargoSafetyProtocol::kFaultLocalization &&
                  Status::FAULT_GRAVITY ==
                      CargoSafetyProtocol::kFaultGravity &&
                  Status::FAULT_CARGO == CargoSafetyProtocol::kFaultCargo &&
                  Status::FAULT_OBSTACLE ==
                      CargoSafetyProtocol::kFaultObstacle &&
                  Status::FAULT_INTERNAL ==
                      CargoSafetyProtocol::kFaultInternal,
                  "Cargo safety fault-mask mismatch");
    static_assert(Status::HOOK_ROLE_DISABLED ==
                      static_cast<std::uint8_t>(HookLoadSignalRole::DISABLED) &&
                  Status::HOOK_ROLE_REQUIRED ==
                      static_cast<std::uint8_t>(HookLoadSignalRole::REQUIRED) &&
                  Status::HOOK_ROLE_AUXILIARY ==
                      static_cast<std::uint8_t>(HookLoadSignalRole::AUXILIARY),
                  "Hook load role schema mismatch");

    CargoSafetyDecisionInput decision_input;
    decision_input.system_ready = system_ready;
    decision_input.localization_valid = localization_valid;
    decision_input.hook_signal_role = hook_load_signal_role_;
    decision_input.gravity_valid = gravity_valid;
    decision_input.gravity_conflict = hook_evidence.gravity_conflict;
    decision_input.safe_empty = safe_empty;
    decision_input.hook_loaded = lidar_cargo_accepted;
    decision_input.cargo_fault = cargo_fault;
    decision_input.obstacle_fault = obstacle_fault;
    decision_input.internal_fault = internal_fault;
    decision_input.warning_valid = warning_valid && warning_code_valid;
    decision_input.warning_code = warning_code;
    decision_input.evidence_reason = cargo_safety_config_error_
        ? "cargo_safety_config_invalid" :
          (warning_contract_error ? "warning_contract_invalid" :
           (finite_contract_error ? "non_finite_safety_status" :
            (hook_evidence.gravity_conflict
                ? std::string("gravity_lidar_conflict:") + evidence_reason
                : evidence_reason)));
    const CargoSafetyDecision decision =
        composeCargoSafetyDecision(decision_input);

    status.localization_valid = localization_valid;
    status.valid = decision.valid;
    status.warning_valid = decision.warning_valid;
    status.requested_alarm_code = decision.requested_code;
    status.warning_code = decision.warning_code;
    status.fault_code = decision.fault_code;
    status.fault_mask = decision.fault_mask;
    status.reason = decision.reason;
    return status;
}

void NdtSlamNode::logCargoSafetyStatus(
    const lidar_slam2_msgs::CargoSafetyStatus& status) {
    const bool code_changed =
        status.requested_alarm_code != cargo_last_requested_code_;
    const bool pending_evidence =
        status.requested_alarm_code ==
            CargoSafetyProtocol::kObstacleInvalid &&
        (status.reason.find("pending") != std::string::npos ||
         status.reason.find("too_sparse") != std::string::npos ||
         status.reason.find("spatial_discontinuity") != std::string::npos ||
         status.reason.find("source_unresolved") != std::string::npos ||
         status.reason.find("evidence_gap") != std::string::npos);
    const bool noisy_pending =
        status.reason.find("too_sparse") != std::string::npos ||
        status.reason.find("spatial_discontinuity") != std::string::npos ||
        status.reason.find("source_unresolved") != std::string::npos;
    cargo_last_requested_code_ = status.requested_alarm_code;
    cargo_last_safety_reason_ = status.reason;

    if (pending_evidence) {
        if (cargo_safety_pending_since_stamp_.isZero() || code_changed) {
            cargo_safety_pending_since_stamp_ = status.header.stamp;
            cargo_safety_pending_error_reported_ = false;
        }
    } else {
        cargo_safety_pending_since_stamp_ = ros::Time();
        cargo_safety_pending_error_reported_ = false;
    }
    const double elapsed = cargo_last_safety_console_stamp_.isZero()
        ? std::numeric_limits<double>::infinity()
        : (status.header.stamp - cargo_last_safety_console_stamp_).toSec();
    const bool periodic = !std::isfinite(elapsed) || elapsed < 0.0 ||
        elapsed >= cargo_safety_console_period_sec_;
    const double pending_age = pending_evidence &&
            !cargo_safety_pending_since_stamp_.isZero()
        ? std::max(
              0.0,
              (status.header.stamp - cargo_safety_pending_since_stamp_)
                  .toSec())
        : 0.0;
    const bool persistent_due = pending_evidence &&
        pending_age >= cargo_safety_pending_error_sec_ &&
        !cargo_safety_pending_error_reported_;
    const bool one_shot_hard_fault =
        status.requested_alarm_code ==
            CargoSafetyProtocol::kCargoInvalid ||
        status.requested_alarm_code ==
            CargoSafetyProtocol::kLocalizationInvalid ||
        status.requested_alarm_code ==
            CargoSafetyProtocol::kGravityInvalid ||
        status.requested_alarm_code ==
            CargoSafetyProtocol::kInternalError;
    if (pending_evidence && !periodic && !code_changed && !persistent_due) {
        return;
    }
    if (one_shot_hard_fault && !code_changed) return;
    if (!pending_evidence && !code_changed && !periodic) return;

    std::ostringstream message;
    message << "[CARGO_SAFETY] code=" << status.requested_alarm_code
            << " reason=" << status.reason
            << " track=" << status.cargo_track_id
            << " cargo_valid=" << (status.cargo_valid ? 1 : 0)
            << " roi_finite=" << cargo_obstacle_roi_finite_points_
            << " coverage=" << std::fixed << std::setprecision(3)
            << cargo_obstacle_roi_coverage_ratio_
            << " self_removed=" << cargo_self_removed_points_
            << " identity_removed="
            << cargo_identity_self_removed_points_
            << " rigging_removed="
            << cargo_rigging_self_removed_points_
            << " external=" << cargo_external_obstacle_points_
            << " cluster_points=" << cargo_dangerous_cluster_points_
            << " distance=" << cargo_nearest_cluster_distance_m_
            << " clearance=" << cargo_conservative_clearance_m_
            << " nearest=(" << cargo_nearest_obstacle_point_.x() << ","
            << cargo_nearest_obstacle_point_.y() << ","
            << cargo_nearest_obstacle_point_.z() << ")"
            << " bottom=" << status.cargo_bottom_z_map
            << " bottom_unc=" << status.cargo_bottom_uncertainty_m
            << " obstacle_top=" << cargo_obstacle_top_z95_m_
            << " obstacle_unc=" << cargo_obstacle_uncertainty_m_
            << " horizontal_unc=" << cargo_horizontal_uncertainty_m_
            << " vertical_unc=" << cargo_vertical_uncertainty_m_
            << " spatial_mode=" << cargo_safety_spatial_mode_
            << " obstacle_track=" << cargo_obstacle_track_id_
            << " confirm=" << cargo_obstacle_track_confirm_count_
            << " pending_age=" << pending_age
            << " ground_valid="
            << (hook_fixed_cargo_.ground_reference_valid ? 1 : 0)
            << " ground_z=" << hook_fixed_cargo_.ground_z;
    if (pending_evidence &&
        pending_age >= cargo_safety_pending_error_sec_) {
        ROS_ERROR_STREAM("[CARGO_SAFETY_PERSISTENT] " << message.str());
        cargo_safety_pending_error_reported_ = true;
    } else if (pending_evidence && noisy_pending) {
        ROS_WARN_STREAM("[CARGO_SAFETY_PENDING] " << message.str());
    } else if (pending_evidence) {
        ROS_DEBUG_STREAM("[CARGO_SAFETY_PENDING] " << message.str());
    } else if (status.requested_alarm_code ==
                   CargoSafetyProtocol::kCargoInvalid) {
        ROS_ERROR_STREAM(message.str());
    } else if (status.requested_alarm_code == CargoSafetyProtocol::kLevel1Warning ||
        status.requested_alarm_code == CargoSafetyProtocol::kLevel2Warning ||
        status.requested_alarm_code >= CargoSafetyProtocol::kSystemNotReady) {
        ROS_WARN_STREAM(message.str());
    } else {
        ROS_INFO_STREAM(message.str());
    }
    cargo_last_safety_console_stamp_ = status.header.stamp;
}

void NdtSlamNode::publishHookOnlySafetyStatus(
    const HookLoadSnapshot& hook, const ros::Time& stamp,
    bool visual_conflict, const std::string& reason,
    bool evidence_initialized) {
    const bool lidar_no_cargo =
        lidar_no_cargo_evidence_.result().confirmed && !visual_conflict;
    const bool safe_empty =
        hook_load_signal_role_ == HookLoadSignalRole::REQUIRED
            ? (hook.valid &&
               hook.state == lidar_slam2_msgs::HookLoadState::STATE_EMPTY &&
               !visual_conflict)
            : lidar_no_cargo;

    lidar_slam2_msgs::CargoSafetyStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = map_frame_;
    status.schema_version = lidar_slam2_msgs::CargoSafetyStatus::SCHEMA_VERSION;
    status.cargo_valid = false;
    status.cargo_source =
        lidar_slam2_msgs::CargoBottomEstimate::SOURCE_INVALID;
    status.hook_signal_valid = hook.valid;
    status.hook_load_state = hook.state;
    status.hook_voltage = hook.voltage;
    status.no_cargo_confirmed = safe_empty;
    status.obstacle_valid = false;
    status.confidence = safe_empty ? 1.0F : 0.0F;
    status = composeCargoSafetyStatus(
        status, visual_conflict, CargoSafetyFault::NONE,
        safe_empty ? CargoSafetyEvaluator::kSafeCode : 0U,
        safe_empty, reason.empty() ? hook.reason : reason,
        evidence_initialized);
    cargo_raw_warning_code_ = status.warning_valid
        ? status.warning_code : 0;
    cargo_confirmed_warning_code_ = cargo_raw_warning_code_;
    cargo_temporal_candidate_code_ = 0;
    cargo_temporal_candidate_count_ = 0;
    cargo_used_previous_confirmation_ = false;
    cargo_raw_safety_status_pub_.publish(status);
    std_msgs::Int32 raw_status_code_msg;
    raw_status_code_msg.data = status.requested_alarm_code;
    cargo_raw_status_code_pub_.publish(raw_status_code_msg);
    cargo_obstacle_roi_finite_points_ = 0U;
    cargo_obstacle_roi_coverage_ratio_ = 0.0F;
    cargo_self_removed_points_ = 0U;
    cargo_identity_self_removed_points_ = 0U;
    cargo_rigging_self_removed_points_ = 0U;
    cargo_external_obstacle_points_ = 0U;
    cargo_dangerous_cluster_points_ = 0U;
    cargo_nearest_obstacle_point_.setZero();
    cargo_nearest_cluster_center_.setZero();
    cargo_nearest_cluster_distance_m_ =
        std::numeric_limits<float>::infinity();
    cargo_obstacle_top_z95_m_ =
        std::numeric_limits<float>::quiet_NaN();
    cargo_obstacle_uncertainty_m_ =
        std::numeric_limits<float>::quiet_NaN();
    cargo_conservative_clearance_m_ =
        std::numeric_limits<float>::quiet_NaN();
    cargo_horizontal_uncertainty_m_ = 0.0F;
    cargo_vertical_uncertainty_m_ = 0.0F;
    cargo_self_margin_xy_m_ = 0.0F;
    cargo_self_margin_z_m_ = 0.0F;
    cargo_safety_status_pub_.publish(status);
    logCargoSafetyStatus(status);

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
    const bool localization_valid =
        !tracking_lost_.load() && relocalization_pose_reliable_;
    publishCargoFusionMarker(
        CargoBottomResult{}, stamp, safe_empty, localization_valid);
    publishPayloadTrackInfoInvalid(status.reason);
}

void NdtSlamNode::updateAndPublishCargoSafetyPipeline(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& obstacle_cloud_base,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& observation_cloud_base,
    const Sophus::SE3d& pose_map_base,
    const ros::Time& stamp,
    const ros::Time& obstacle_cloud_stamp,
    double processing_age_sec) {
    last_cargo_pipeline_stamp_ = stamp;
    const HookLoadSnapshot hook = currentHookLoadSnapshot();
    const bool visual_conflict = hook_fixed_cargo_.valid;
    const bool required_role =
        hook_load_signal_role_ == HookLoadSignalRole::REQUIRED;
    if (required_role &&
        (!hook.valid ||
         hook.state == lidar_slam2_msgs::HookLoadState::STATE_UNKNOWN ||
         hook.state == lidar_slam2_msgs::HookLoadState::STATE_INHIBIT)) {
        publishHookOnlySafetyStatus(
            hook, stamp, visual_conflict,
            std::string("hook_signal_invalid:") + hook.reason);
        return;
    }
    if (required_role &&
        hook.state == lidar_slam2_msgs::HookLoadState::STATE_EMPTY) {
        publishHookOnlySafetyStatus(
            hook, stamp, visual_conflict,
            visual_conflict ? "empty_hook_visual_conflict"
                            : "empty_hook_no_cargo_confirmed");
        return;
    }
    const bool lidar_no_cargo_confirmed =
        lidar_no_cargo_evidence_.result().confirmed && !visual_conflict;
    if (!required_role && lidar_no_cargo_confirmed) {
        publishHookOnlySafetyStatus(
            hook, stamp, false,
            hook.valid ? "lidar_no_cargo_gravity_diagnostic"
                       : "lidar_no_cargo_gravity_unavailable");
        return;
    }

    // LOCKED and LOST_HOLD are one retained rigid track. Missing current
    // points increase uncertainty; they do not create a new track or disable
    // formal safety before explicit empty/lost_clear.
    const bool active_track = cargoTrackRetained() &&
        cargo_state_.valid_geometry;
    if (active_track && !cargo_fusion_track_active_) {
        ++cargo_fusion_track_id_;
        if (cargo_fusion_track_id_ == 0U) ++cargo_fusion_track_id_;
        cargo_bottom_fusion_.reset();
        cargo_origin_height_valid_ = false;
        cargo_origin_height_m_ = 0.0F;
        cargo_origin_height_track_id_ = cargo_fusion_track_id_;
    } else if (!active_track && cargo_fusion_track_active_) {
        cargo_bottom_fusion_.reset();
        cargo_origin_height_valid_ = false;
        formal_cargo_removal_authorized_ = false;
    }
    cargo_fusion_track_active_ = active_track;
    if (!active_track) {
        // Before either a retained cargo track or an independently confirmed
        // EMPTY observation exists, the safety evidence epoch has not
        // started.  Report 30 (not-ready), not 33 (an established cargo whose
        // live pose/height has become invalid).
        publishHookOnlySafetyStatus(
            hook, stamp, visual_conflict,
            visual_conflict ? "cargo_candidate_not_yet_authoritative"
                            : "cargo_track_not_initialized",
            false);
        return;
    }

    // Gravity may become LOADED after a lift-evidence track has already
    // started. Consume the EMPTY-phase origin as soon as it becomes available,
    // not only on the first active-track frame.
    if (active_track && !cargo_origin_height_valid_) {
        std::lock_guard<std::mutex> lock(hook_load_state_mutex_);
        if (pending_origin_height_valid_) {
            const bool track_center_finite =
                cargo_state_.center_base.head<2>().allFinite();
            const bool origin_center_finite =
                pending_origin_center_base_.allFinite();
            const double origin_distance =
                track_center_finite && origin_center_finite
                    ? static_cast<double>(
                          (cargo_state_.center_base.head<2>() -
                           pending_origin_center_base_).norm())
                    : std::numeric_limits<double>::quiet_NaN();
            PendingOriginAction action = std::isfinite(pending_origin_height_m_)
                ? evaluatePendingOriginBinding({
                      true, stamp.toSec(), pending_origin_stamp_.toSec(),
                      origin_history_max_age_sec_,
                      origin_future_stamp_tolerance_sec_,
                      track_center_finite, origin_center_finite,
                      origin_distance,
                      static_cast<double>(origin_match_max_distance_m_)})
                : PendingOriginAction::DISCARD_INVALID;

            const auto clear_pending_origin = [this]() {
                pending_origin_height_valid_ = false;
                pending_origin_height_m_ = 0.0F;
                pending_origin_center_base_.setZero();
                pending_origin_stamp_ = ros::Time();
            };
            switch (action) {
                case PendingOriginAction::KEEP_WAITING_FOR_LIDAR_TIME:
                    ROS_DEBUG_THROTTLE(
                        1.0,
                        "[OriginBinding] action=%s lidar=%.6f origin=%.6f",
                        pendingOriginActionName(action), stamp.toSec(),
                        pending_origin_stamp_.toSec());
                    break;
                case PendingOriginAction::ATTACH:
                    cargo_origin_height_valid_ = true;
                    cargo_origin_height_m_ = pending_origin_height_m_;
                    cargo_origin_height_track_id_ = cargo_fusion_track_id_;
                    if (hook_lock_.locked_shape.valid &&
                        hook_lock_.live_pose.valid &&
                        std::isfinite(cargo_origin_height_m_) &&
                        cargo_origin_height_m_ >=
                            cargo_bottom_fusion_.config()
                                .minimum_prior_height &&
                        cargo_origin_height_m_ <=
                            cargo_bottom_fusion_.config()
                                .maximum_prior_height) {
                        // Calibrate the one frozen thickness from the stable
                        // EMPTY-phase history. Preserve the currently observed
                        // top face while moving the derived center/bottom.
                        const float previous_height =
                            hook_lock_.locked_shape.height_m;
                        const float previous_top =
                            hook_lock_.live_pose.center_base.z() +
                            0.5F * previous_height;
                        hook_lock_.locked_shape.height_m =
                            cargo_origin_height_m_;
                        hook_lock_.locked_size.z() = cargo_origin_height_m_;
                        hook_lock_.stable_height = cargo_origin_height_m_;
                        hook_lock_.stable_top_z = previous_top;
                        hook_lock_.stable_bottom_z = previous_top -
                            cargo_origin_height_m_;
                        hook_lock_.live_pose.center_base.z() =
                            previous_top - 0.5F * cargo_origin_height_m_;
                        hook_lock_.live_pose_measured_base.z() =
                            hook_lock_.live_pose.center_base.z();
                        hook_lock_.live_pose_predicted_base.z() =
                            hook_lock_.live_pose.center_base.z();
                        hook_lock_.live_pose_velocity_base.z() = 0.0F;
                    }
                    ROS_INFO(
                        "[OriginBinding] action=%s track=%llu age=%.3f "
                        "distance=%.3f frozen_thickness=%.3f",
                        pendingOriginActionName(action),
                        static_cast<unsigned long long>(
                            cargo_fusion_track_id_),
                        std::max(0.0,
                                 (stamp - pending_origin_stamp_).toSec()),
                        origin_distance, cargo_origin_height_m_);
                    clear_pending_origin();
                    break;
                case PendingOriginAction::DISCARD_EXPIRED:
                case PendingOriginAction::DISCARD_SPATIAL_MISMATCH:
                case PendingOriginAction::DISCARD_INVALID:
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[OriginBinding] action=%s lidar=%.6f origin=%.6f "
                        "distance=%.3f",
                        pendingOriginActionName(action), stamp.toSec(),
                        pending_origin_stamp_.toSec(), origin_distance);
                    clear_pending_origin();
                    break;
                case PendingOriginAction::NONE:
                    break;
            }
        }
    }

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
        observation.footprint_valid =
            hook_lock_.locked_shape.length_m > 0.0F &&
            hook_lock_.locked_shape.width_m > 0.0F;
        observation.footprint_center_base =
            hook_lock_.live_pose.center_base.head<2>();
        observation.footprint_size_xy = Eigen::Vector2f(
            hook_lock_.locked_shape.length_m,
            hook_lock_.locked_shape.width_m);
        observation.footprint_yaw_base_rad =
            hook_lock_.locked_shape.yaw_base_rad;
        observation.track_center_valid = hook_lock_.live_pose.valid &&
            hook_lock_.live_pose.center_base.allFinite();
        observation.track_center_base =
            hook_lock_.live_pose.center_base;
        const bool origin_height_matches_track = cargo_origin_height_valid_ &&
            cargo_origin_height_track_id_ == cargo_fusion_track_id_;
        observation.prior_height_valid = origin_height_matches_track;
        observation.prior_height_m = cargo_origin_height_m_;
        observation.origin_height_valid = origin_height_matches_track;
        observation.origin_height_m = cargo_origin_height_m_;
        // MAP_STATIC and MAP_DIFF stay invalid until an independent map-backed
        // observation is available. Never label the origin prior as map evidence.
        observation.map_static_height_valid = false;
        observation.map_diff_height_valid = false;
    }

    last_cargo_bottom_result_ = cargo_bottom_fusion_.update(observation);
    last_cargo_pipeline_stamp_ = stamp;
    const bool current_bottom_evidence =
        hook_lock_.live_vertical_pose_valid &&
        !hook_lock_.live_vertical_pose_evidence_stamp.isZero() &&
        std::abs((hook_lock_.live_vertical_pose_evidence_stamp - stamp)
                     .toSec()) <= 1.0e-4 &&
        (hook_lock_.live_pose.vertical_source ==
             CargoVerticalPoseSource::DIRECT_TOP ||
         hook_lock_.live_pose.vertical_source ==
             CargoVerticalPoseSource::DIRECT_BOTTOM ||
         hook_lock_.live_pose.vertical_source ==
             CargoVerticalPoseSource::LOCKED_OBB_POINT_SUPPORT);
    RigidCargoGeometry rigid_geometry =
        buildCurrentRigidCargoGeometryForPose(pose_map_base, stamp);
    const CargoFormalUseDecision formal_use = evaluateCargoFormalUse(
        rigid_geometry.valid,
        hook_lock_.state == HookCargoLockState::LOST_HOLD,
        stamp.toSec(), rigid_geometry.pose_evidence_stamp_sec,
        rigid_geometry.height_evidence_stamp_sec,
        hook_lock_config_.formal_xy_evidence_hold_sec,
        hook_lock_config_.formal_vertical_evidence_hold_sec,
        rigid_geometry.horizontal_uncertainty_m);
    if (rigid_geometry.valid) {
        CargoBoxGeometry formal_geometry;
        formal_geometry.valid = true;
        formal_geometry.center_base = rigid_geometry.pose.center_base;
        formal_geometry.size_base = Eigen::Vector3f(
            rigid_geometry.shape.length_m,
            rigid_geometry.shape.width_m,
            rigid_geometry.shape.height_m);
        formal_geometry.bottom_z_base = rigid_geometry.bottom_z_base;
        formal_geometry.top_z_base = rigid_geometry.top_z_base;
        formal_geometry.corners_base = rigid_geometry.corners_base;
        formal_geometry.corners_map = rigid_geometry.corners_map;
        formal_geometry.center_map.setZero();
        for (const auto& corner : rigid_geometry.corners_map) {
            formal_geometry.center_map += corner;
        }
        formal_geometry.center_map /= 8.0F;
        formal_geometry.size_map = rigid_geometry.aabb_max_map -
            rigid_geometry.aabb_min_map;
        formal_geometry.bottom_z_map = rigid_geometry.aabb_min_map.z();
        formal_geometry.top_z_map = rigid_geometry.aabb_max_map.z();
        last_cargo_bottom_result_.geometry = formal_geometry;
        last_cargo_bottom_result_.geometry_valid = true;
        last_cargo_bottom_result_.height_valid =
            formal_use.formal_safety_valid;
        last_cargo_bottom_result_.valid =
            last_cargo_bottom_result_.valid &&
            formal_use.formal_safety_valid;
        last_cargo_bottom_result_.height = rigid_geometry.shape.height_m;
        last_cargo_bottom_result_.track_id = cargo_fusion_track_id_;
        // stamp_sec is the evaluation time for the message. The safety
        // evaluator uses evidence_stamp_sec and must never see a held sample
        // refreshed to the current tick.
        last_cargo_bottom_result_.stamp_sec = stamp.toSec();
        // Frozen thickness persists, but bottom position authority advances
        // only with DIRECT_TOP/DIRECT_BOTTOM/LOCKED_OBB_POINT_SUPPORT.
        last_cargo_bottom_result_.evidence_stamp_sec =
            rigid_geometry.height_evidence_stamp_sec;
        if (!current_bottom_evidence) {
            last_cargo_bottom_result_.valid =
                formal_use.formal_safety_valid;
            last_cargo_bottom_result_.source =
                CargoBottomSource::RECENT_STABLE;
            last_cargo_bottom_result_.source_name = "RECENT_STABLE";
            last_cargo_bottom_result_.reason = formal_use.reason;
            last_cargo_bottom_result_.uncertainty =
                rigid_geometry.vertical_uncertainty_m;
            last_cargo_bottom_result_.confidence = std::max(
                0.20F, hook_lock_.locked_shape.orientation_confidence *
                    (hook_lock_.state == HookCargoLockState::LOST_HOLD
                         ? 0.55F : 0.75F));
        }
        cargo_state_.center_base = rigid_geometry.pose.center_base;
        cargo_state_.size = formal_geometry.size_base;
        cargo_state_.bottom_z = rigid_geometry.bottom_z_base;
        cargo_state_.top_z = rigid_geometry.top_z_base;
        cargo_state_.bottom_unc =
            rigid_geometry.vertical_uncertainty_m;
        cargo_state_.bottom_safe_z = cargo_state_.bottom_z -
            cargo_state_.bottom_unc - 0.05F;
        cargo_state_.valid_geometry = true;
        cargo_state_.valid_height = formal_use.formal_safety_valid;
        cargo_state_.source = std::string("rigid:") +
            cargoPoseSourceName(rigid_geometry.pose.source);
        formal_cargo_removal_authorized_ = active_track &&
            formal_use.formal_removal_valid;
        formal_cargo_removal_track_id_ = cargo_fusion_track_id_;
        formal_cargo_removal_stamp_.fromSec(std::min(
            rigid_geometry.pose_evidence_stamp_sec,
            rigid_geometry.height_evidence_stamp_sec));
    } else {
        cargo_state_.valid_height = false;
        cargo_state_.bottom_unc =
            cargo_bottom_fusion_.config().invalid_uncertainty;
        cargo_state_.source = "rigid:INVALID";
        formal_cargo_removal_authorized_ = false;
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
    visualization_msgs::Marker predicted_obb_marker;
    predicted_obb_marker.header.stamp = stamp;
    predicted_obb_marker.header.frame_id = "base_link";
    predicted_obb_marker.ns = "cargo_predicted_obb";
    predicted_obb_marker.id = 0;
    predicted_obb_marker.type = visualization_msgs::Marker::CUBE;
    predicted_obb_marker.action = rigid_geometry.valid
        ? visualization_msgs::Marker::ADD
        : visualization_msgs::Marker::DELETE;
    predicted_obb_marker.pose.orientation.w = 1.0;
    if (rigid_geometry.valid) {
        predicted_obb_marker.pose.position.x =
            rigid_geometry.pose.center_base.x();
        predicted_obb_marker.pose.position.y =
            rigid_geometry.pose.center_base.y();
        predicted_obb_marker.pose.position.z =
            rigid_geometry.pose.center_base.z();
        const float half_yaw =
            0.5F * rigid_geometry.shape.yaw_base_rad;
        predicted_obb_marker.pose.orientation.z = std::sin(half_yaw);
        predicted_obb_marker.pose.orientation.w = std::cos(half_yaw);
        predicted_obb_marker.scale.x = rigid_geometry.shape.length_m;
        predicted_obb_marker.scale.y = rigid_geometry.shape.width_m;
        predicted_obb_marker.scale.z = rigid_geometry.shape.height_m;
        predicted_obb_marker.color.r = 0.10F;
        predicted_obb_marker.color.g = 0.75F;
        predicted_obb_marker.color.b = 1.0F;
        predicted_obb_marker.color.a = 0.30F;
    }
    cargo_predicted_obb_pub_.publish(predicted_obb_marker);

    CargoSafetyInput safety_input;
    safety_input.evaluation_time_sec = stamp.toSec();
    safety_input.height.valid = last_cargo_bottom_result_.valid;
    safety_input.height.stale = false;
    safety_input.height.stamp_sec =
        last_cargo_bottom_result_.evidence_stamp_sec;
    safety_input.height.bottom_z =
        last_cargo_bottom_result_.geometry.bottom_z_base;
    safety_input.height.bottom_uncertainty_m =
        last_cargo_bottom_result_.uncertainty;
    if (rigid_geometry.valid) {
        safety_input.footprint_base = toCargoObbFootprint(
            rigid_geometry, formal_use.horizontal_uncertainty_m);
    }
    Eigen::Vector2f cargo_center_map = Eigen::Vector2f::Zero();
    bool cargo_map_velocity_valid = false;
    if (rigid_geometry.valid &&
        rigid_geometry.pose.center_base.allFinite()) {
        const Eigen::Vector3d center_map_3d = pose_map_base *
            rigid_geometry.pose.center_base.cast<double>();
        cargo_center_map = center_map_3d.head<2>().cast<float>();
        const double current_stamp_sec = stamp.toSec();
        if (cargo_map_motion_sample_valid_) {
            const double sample_dt_sec =
                current_stamp_sec - cargo_previous_center_stamp_sec_;
            if (sample_dt_sec > 1.0e-4 &&
                sample_dt_sec <=
                    cargo_motion_corridor_config_
                        .maximum_velocity_sample_gap_sec &&
                cargo_previous_center_map_.allFinite()) {
                const Eigen::Vector2f measured_velocity =
                    (cargo_center_map - cargo_previous_center_map_) /
                    static_cast<float>(sample_dt_sec);
                const float alpha =
                    cargo_motion_corridor_config_.velocity_alpha;
                cargo_velocity_map_ =
                    (1.0F - alpha) * cargo_velocity_map_ +
                    alpha * measured_velocity;
                cargo_map_velocity_valid = cargo_velocity_map_.allFinite();
            } else if (sample_dt_sec < -1.0e-4 ||
                       sample_dt_sec >
                           cargo_motion_corridor_config_
                               .maximum_velocity_sample_gap_sec) {
                cargo_velocity_map_.setZero();
            }
        }
        cargo_previous_center_map_ = cargo_center_map;
        cargo_previous_center_stamp_sec_ = current_stamp_sec;
        cargo_map_motion_sample_valid_ = true;

        // The cargo is rigidly carried by the crane for the dominant map
        // motion. Prefer the EKF velocity when it is backed by a current NDT
        // update: it remains a valid zero-speed observation while stationary
        // and does not turn small cargo-centre fitting jitter into a false
        // swept-corridor direction. The centre finite difference above is
        // retained as the fallback for relative cargo motion or unavailable
        // localization velocity.
        const CraneMotionEKFStatus& ekf_status = crane_motion_ekf_.status();
        if (ekf_status.initialized && ekf_status.ndt_accepted &&
            !ekf_status.prediction_only &&
            ekf_status.velocity.allFinite()) {
            cargo_velocity_map_ =
                ekf_status.velocity.cast<float>();
            cargo_map_velocity_valid = true;
        }
    } else {
        cargo_map_motion_sample_valid_ = false;
        cargo_velocity_map_.setZero();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_roi(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::set<std::pair<int, int>> observed_cells;
    std::size_t roi_finite_points = 0U;
    float coverage_ratio = 0.0F;
    if (last_cargo_bottom_result_.geometry_valid) {
        const float radius = cargo_safety_evaluator_.config().level2_distance_m;
        const float uncertainty = formal_use.horizontal_uncertainty_m;
        const float min_x = rigid_geometry.aabb_min_base.x() - radius -
            uncertainty;
        const float max_x = rigid_geometry.aabb_max_base.x() + radius +
            uncertainty;
        const float min_y = rigid_geometry.aabb_min_base.y() - radius -
            uncertainty;
        const float max_y = rigid_geometry.aabb_max_base.y() + radius +
            uncertainty;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr self_removed_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr external_obstacle_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    const CargoObbFootprint predicted_self_footprint =
        rigid_geometry.valid
            ? toCargoObbFootprint(rigid_geometry)
            : CargoObbFootprint{};
    const CargoObbFootprint previous_self_footprint =
        previous_self_mask_geometry_.valid &&
                previous_self_mask_geometry_.track_id == rigid_geometry.track_id
            ? toCargoObbFootprint(previous_self_mask_geometry_)
            : CargoObbFootprint{};
    const CargoObbFootprint accepted_self_footprint =
        accepted_self_mask_geometry_.valid &&
                accepted_self_mask_geometry_.track_id == rigid_geometry.track_id
            ? toCargoObbFootprint(accepted_self_mask_geometry_)
            : CargoObbFootprint{};
    const float self_margin_xy = std::min(
        hook_lock_config_.self_cargo_max_margin_xy_m,
        hook_lock_config_.self_cargo_base_margin_xy_m +
            hook_lock_.horizontal_tracking_residual_m +
            rigid_geometry.horizontal_uncertainty_m);
    const float self_margin_z = std::min(
        hook_lock_config_.self_cargo_max_margin_z_m,
        hook_lock_config_.self_cargo_base_margin_z_m +
            hook_lock_.vertical_tracking_residual_m +
            0.5F * rigid_geometry.vertical_uncertainty_m);
    cargo_obstacle_roi_finite_points_ = roi_finite_points;
    cargo_obstacle_roi_coverage_ratio_ = coverage_ratio;
    cargo_self_margin_xy_m_ = self_margin_xy;
    cargo_self_margin_z_m_ = self_margin_z;
    cargo_horizontal_uncertainty_m_ =
        rigid_geometry.horizontal_uncertainty_m;
    cargo_vertical_uncertainty_m_ =
        rigid_geometry.vertical_uncertainty_m;

    pcl::KdTreeFLANN<pcl::PointXYZ> identity_self_tree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr identity_self_reference;
    if (detection_is_current && hook_fixed_cargo_.core_points_base &&
        !hook_fixed_cargo_.core_points_base->empty()) {
        identity_self_reference = hook_fixed_cargo_.core_points_base;
    } else if (hook_lock_.has_last_accepted &&
               hook_lock_.last_accepted_core_points &&
               !hook_lock_.last_accepted_core_points->empty() &&
               rigid_geometry.pose.center_base.allFinite()) {
        // Detection is intentionally rate-limited.  Between detector frames,
        // translate the last identity-selected component with the retained
        // rigid pose instead of dropping the identity mask for one frame.
        identity_self_reference.reset(
            new pcl::PointCloud<pcl::PointXYZ>);
        identity_self_reference->reserve(
            hook_lock_.last_accepted_core_points->size());
        const Eigen::Vector3f translation =
            rigid_geometry.pose.center_base -
            hook_lock_.last_accepted_center;
        for (const pcl::PointXYZ& source :
             hook_lock_.last_accepted_core_points->points) {
            pcl::PointXYZ projected = source;
            projected.x += translation.x();
            projected.y += translation.y();
            projected.z += translation.z();
            identity_self_reference->push_back(projected);
        }
    }
    const bool identity_self_mask_valid = identity_self_reference &&
        !identity_self_reference->empty();
    if (identity_self_mask_valid) {
        identity_self_tree.setInputCloud(identity_self_reference);
    }
    std::size_t identity_self_removed = 0U;
    std::size_t rigging_self_removed = 0U;
    const Eigen::Vector2f hook_anchor = getCargoAnchorXY();
    const float rigging_lower_z = predicted_self_footprint.valid
        ? predicted_self_footprint.max_z - self_margin_z
        : std::numeric_limits<float>::infinity();
    const float rigging_upper_z = std::max(
        rigging_lower_z + 0.10F,
        static_cast<float>(odom_anchor_config_.search_z_max));
    std::vector<int> identity_nearest_index(1);
    std::vector<float> identity_nearest_distance_sq(1);
    // Identity correspondence gets one additional voxel shell. Geometry
    // alone still uses the conservative margins below.
    const float identity_margin_xy = self_margin_xy +
        hook_lock_config_.self_cargo_point_match_radius_m;
    const float identity_margin_z = self_margin_z +
        hook_lock_config_.self_cargo_point_match_radius_m;
    self_removed_cloud->reserve(obstacle_roi->size());
    external_obstacle_cloud->reserve(obstacle_roi->size());
    for (const pcl::PointXYZ& point : obstacle_roi->points) {
        const Eigen::Vector3f point_base = point.getVector3fMap();
        const bool inside_predicted = predicted_self_footprint.valid &&
            containsPointInCargoObbBase(
                point_base, predicted_self_footprint,
                self_margin_xy, self_margin_z);
        const bool inside_previous = previous_self_footprint.valid &&
            containsPointInCargoObbBase(
                point_base, previous_self_footprint,
                self_margin_xy, self_margin_z);
        const bool inside_accepted = accepted_self_footprint.valid &&
            containsPointInCargoObbBase(
                point_base, accepted_self_footprint,
                self_margin_xy, self_margin_z);
        const bool inside_swept_previous =
            previous_self_footprint.valid && predicted_self_footprint.valid &&
            containsPointInSweptCargoObbBase(
                point_base, previous_self_footprint,
                predicted_self_footprint, self_margin_xy, self_margin_z);
        const bool inside_swept_accepted =
            accepted_self_footprint.valid && predicted_self_footprint.valid &&
            containsPointInSweptCargoObbBase(
                point_base, accepted_self_footprint,
                predicted_self_footprint, self_margin_xy, self_margin_z);
        // Geometry alone removes only the conservative formal OBB. Identity
        // correspondence gets one additional voxel shell, allowing real
        // cargo surface returns shifted by downsampling to be recovered
        // without blindly expanding the geometric self mask.
        const bool inside_identity_predicted =
            predicted_self_footprint.valid &&
            containsPointInCargoObbBase(
                point_base, predicted_self_footprint,
                identity_margin_xy, identity_margin_z);
        const bool inside_identity_previous =
            previous_self_footprint.valid &&
            containsPointInCargoObbBase(
                point_base, previous_self_footprint,
                identity_margin_xy, identity_margin_z);
        const bool inside_identity_accepted =
            accepted_self_footprint.valid &&
            containsPointInCargoObbBase(
                point_base, accepted_self_footprint,
                identity_margin_xy, identity_margin_z);
        const bool inside_identity_swept_previous =
            previous_self_footprint.valid && predicted_self_footprint.valid &&
            containsPointInSweptCargoObbBase(
                point_base, previous_self_footprint,
                predicted_self_footprint,
                identity_margin_xy, identity_margin_z);
        const bool inside_identity_swept_accepted =
            accepted_self_footprint.valid && predicted_self_footprint.valid &&
            containsPointInSweptCargoObbBase(
                point_base, accepted_self_footprint,
                predicted_self_footprint,
                identity_margin_xy, identity_margin_z);
        const bool inside_identity_neighborhood =
            inside_identity_predicted || inside_identity_previous ||
            inside_identity_accepted ||
            inside_identity_swept_previous ||
            inside_identity_swept_accepted;
        bool matches_current_identity = false;
        if (identity_self_mask_valid && inside_identity_neighborhood &&
            identity_self_tree.nearestKSearch(
                point, 1, identity_nearest_index,
                identity_nearest_distance_sq) > 0 &&
            !identity_nearest_distance_sq.empty()) {
            matches_current_identity = isCargoIdentityPointMatch(
                identity_nearest_distance_sq.front(),
                hook_lock_config_.self_cargo_point_match_radius_m,
                inside_identity_neighborhood);
        }
        bool inside_rigging = false;
        if (predicted_self_footprint.valid &&
            point.z >= rigging_lower_z &&
            point.z <= rigging_upper_z + self_margin_z) {
            const float alpha = std::clamp(
                (point.z - rigging_lower_z) /
                    std::max(0.10F, rigging_upper_z - rigging_lower_z),
                0.0F, 1.0F);
            const Eigen::Vector2f expected_xy =
                (1.0F - alpha) * predicted_self_footprint.center_base +
                alpha * hook_anchor;
            inside_rigging =
                (point_base.head<2>() - expected_xy).norm() <=
                hook_lock_config_.self_rigging_radius_m;
        }
        if (matches_current_identity || inside_rigging ||
            inside_predicted || inside_previous ||
            inside_accepted ||
            inside_swept_previous || inside_swept_accepted) {
            self_removed_cloud->push_back(point);
            if (matches_current_identity) ++identity_self_removed;
            if (inside_rigging) ++rigging_self_removed;
        } else {
            external_obstacle_cloud->push_back(point);
        }
    }
    if (rigid_geometry.valid) {
        previous_self_mask_geometry_ = rigid_geometry;
        if (hook_observation_associated_current_) {
            accepted_self_mask_geometry_ = rigid_geometry;
        }
    }
    cargo_self_removed_points_ = self_removed_cloud->size();
    cargo_identity_self_removed_points_ = identity_self_removed;
    cargo_rigging_self_removed_points_ = rigging_self_removed;
    cargo_external_obstacle_points_ = external_obstacle_cloud->size();
    const auto publish_safety_debug_cloud = [&stamp](
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        ros::Publisher& publisher) {
        if (publisher.getNumSubscribers() == 0U) return;
        sensor_msgs::PointCloud2 message;
        if (cloud && !cloud->empty()) pcl::toROSMsg(*cloud, message);
        message.header.stamp = stamp;
        message.header.frame_id = "base_link";
        publisher.publish(message);
    };
    safety_input.obstacle_cloud_base = external_obstacle_cloud;
    safety_input.obstacle_observation_valid =
        static_cast<bool>(obstacle_cloud_base) &&
        static_cast<bool>(observation_cloud_base) &&
        last_cargo_bottom_result_.geometry_valid;
    const double stamp_delta =
        (stamp - obstacle_cloud_stamp).toSec();
    const double sensor_age =
        (ros::Time::now() - obstacle_cloud_stamp).toSec();
    constexpr double kFutureStampToleranceSec = 0.05;
    if (!std::isfinite(stamp_delta) ||
        stamp_delta < -kFutureStampToleranceSec ||
        !std::isfinite(sensor_age) ||
        sensor_age < -kFutureStampToleranceSec ||
        !std::isfinite(processing_age_sec) || processing_age_sec < 0.0) {
        safety_input.obstacle_cloud_age_sec =
            std::numeric_limits<double>::infinity();
    } else {
        safety_input.obstacle_cloud_age_sec = std::max(
            {std::max(0.0, stamp_delta), std::max(0.0, sensor_age),
             processing_age_sec});
    }
    safety_input.obstacle_roi_finite_points = roi_finite_points;
    safety_input.obstacle_roi_coverage_ratio = coverage_ratio;
    const CargoSafetyResult radial_safety_result =
        cargo_safety_evaluator_.evaluate(safety_input);
    CargoSafetyResult raw_cargo_safety_result = radial_safety_result;
    cargo_corridor_eligible_clusters_ = 0U;
    cargo_corridor_rejected_clusters_ = 0U;
    const float cargo_map_speed = cargo_velocity_map_.norm();
    const bool motion_velocity_authoritative =
        cargo_motion_corridor_config_.enabled &&
        cargo_map_velocity_valid && std::isfinite(cargo_map_speed);
    const bool motion_corridor_authoritative =
        motion_velocity_authoritative && cargo_map_speed >=
            cargo_motion_corridor_config_.minimum_motion_speed_mps;
    cargo_safety_spatial_mode_ = cargoSafetySpatialModeName(
        motion_corridor_authoritative
            ? CargoSafetySpatialMode::MOTION_CORRIDOR
            : (motion_velocity_authoritative
                   ? CargoSafetySpatialMode::STATIONARY_GUARD
                   : CargoSafetySpatialMode::RADIAL_FALLBACK));
    if (radial_safety_result.input_valid &&
        radial_safety_result.warning_valid &&
        radial_safety_result.fault == CargoSafetyFault::NONE &&
        (radial_safety_result.warning_code ==
             CargoSafetyEvaluator::kLevel1Code ||
         radial_safety_result.warning_code ==
             CargoSafetyEvaluator::kLevel2Code)) {
        raw_cargo_safety_result.cluster_evidence.clear();
        raw_cargo_safety_result.has_cluster_evidence = false;
        raw_cargo_safety_result.evaluated_cluster_count = 0U;
        const Eigen::Matrix3d map_base_rotation =
            pose_map_base.so3().matrix();
        const float map_base_yaw = static_cast<float>(std::atan2(
            map_base_rotation(1, 0), map_base_rotation(0, 0)));
        const auto more_dangerous = [](
            const CargoSafetyClusterEvidence& candidate,
            const CargoSafetyClusterEvidence& current) {
            const int candidate_priority =
                candidate.warning_code == CargoSafetyEvaluator::kLevel1Code
                    ? 2 : 1;
            const int current_priority =
                current.warning_code == CargoSafetyEvaluator::kLevel1Code
                    ? 2 : 1;
            if (candidate_priority != current_priority) {
                return candidate_priority > current_priority;
            }
            if (candidate.conservative_clearance_m !=
                current.conservative_clearance_m) {
                return candidate.conservative_clearance_m <
                    current.conservative_clearance_m;
            }
            return candidate.footprint_distance_m <
                current.footprint_distance_m;
        };
        for (const CargoSafetyClusterEvidence& evidence :
             radial_safety_result.cluster_evidence) {
            if (evidence.warning_code !=
                    CargoSafetyEvaluator::kLevel1Code &&
                evidence.warning_code !=
                    CargoSafetyEvaluator::kLevel2Code) {
                continue;
            }
            const Eigen::Vector3d nearest_map_3d = pose_map_base *
                evidence.nearest_point_base.getVector3fMap().cast<double>();
            const Eigen::Vector3d centroid_map_3d = pose_map_base *
                evidence.centroid_base.getVector3fMap().cast<double>();
            CargoMotionCorridorInput corridor_input;
            corridor_input.cargo_center_map = cargo_center_map;
            corridor_input.cargo_velocity_map = cargo_velocity_map_;
            corridor_input.velocity_valid = cargo_map_velocity_valid;
            corridor_input.cargo_length_m =
                rigid_geometry.shape.length_m;
            corridor_input.cargo_width_m =
                rigid_geometry.shape.width_m;
            corridor_input.cargo_yaw_map_rad = map_base_yaw +
                rigid_geometry.shape.yaw_base_rad;
            corridor_input.horizontal_uncertainty_m =
                formal_use.horizontal_uncertainty_m;
            corridor_input.obstacle_nearest_map =
                nearest_map_3d.head<2>().cast<float>();
            corridor_input.obstacle_centroid_map =
                centroid_map_3d.head<2>().cast<float>();
            corridor_input.current_footprint_distance_m =
                evidence.footprint_distance_m;
            const CargoMotionCorridorDecision corridor_decision =
                evaluateCargoMotionCorridor(
                    cargo_motion_corridor_config_, corridor_input);
            cargo_safety_spatial_mode_ =
                cargoSafetySpatialModeName(corridor_decision.mode);
            if (!corridor_decision.eligible) {
                ++cargo_corridor_rejected_clusters_;
                continue;
            }
            ++cargo_corridor_eligible_clusters_;
            raw_cargo_safety_result.cluster_evidence.push_back(evidence);
            ++raw_cargo_safety_result.evaluated_cluster_count;
            if (!raw_cargo_safety_result.has_cluster_evidence ||
                more_dangerous(
                    evidence,
                    raw_cargo_safety_result.most_dangerous_cluster)) {
                raw_cargo_safety_result.most_dangerous_cluster = evidence;
                raw_cargo_safety_result.has_cluster_evidence = true;
            }
        }
        if (raw_cargo_safety_result.has_cluster_evidence) {
            raw_cargo_safety_result.warning_code =
                raw_cargo_safety_result.most_dangerous_cluster.warning_code;
            raw_cargo_safety_result.reason =
                cargo_safety_spatial_mode_ == "MOTION_CORRIDOR"
                    ? "hazard_inside_motion_corridor"
                    : "hazard_radial_fallback";
        } else {
            raw_cargo_safety_result.warning_code =
                CargoSafetyEvaluator::kSafeCode;
            raw_cargo_safety_result.warning_valid = true;
            raw_cargo_safety_result.fault = CargoSafetyFault::NONE;
            raw_cargo_safety_result.reason =
                "clear_no_hazard_in_motion_corridor";
        }
    }
    cargo_residual_self_clusters_ = 0U;
    cargo_residual_unknown_clusters_ = 0U;
    // Cluster point_indices always refer to the evaluator input. Preserve that
    // index space even if residual self-points are removed for publication.
    const pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cluster_source_cloud =
        external_obstacle_cloud;
    std::set<int> residual_self_point_indices;
    if (raw_cargo_safety_result.input_valid &&
        raw_cargo_safety_result.warning_valid &&
        raw_cargo_safety_result.fault == CargoSafetyFault::NONE &&
        (raw_cargo_safety_result.warning_code ==
             CargoSafetyEvaluator::kLevel1Code ||
         raw_cargo_safety_result.warning_code ==
             CargoSafetyEvaluator::kLevel2Code)) {
        std::vector<CargoSafetyClusterEvidence> classified_clusters;
        classified_clusters.reserve(
            raw_cargo_safety_result.cluster_evidence.size());
        bool has_validated_hazard = false;
        bool has_unresolved_residual = false;
        CargoSafetyClusterEvidence most_dangerous_validated;
        const float identity_radius_sq =
            hook_lock_config_.self_cargo_point_match_radius_m *
            hook_lock_config_.self_cargo_point_match_radius_m;
        for (CargoSafetyClusterEvidence evidence :
             raw_cargo_safety_result.cluster_evidence) {
            if (evidence.warning_code !=
                    CargoSafetyEvaluator::kLevel1Code &&
                evidence.warning_code !=
                    CargoSafetyEvaluator::kLevel2Code) {
                continue;
            }
            std::size_t valid_cluster_points = 0U;
            std::size_t inside_xy_points = 0U;
            std::size_t identity_points = 0U;
            std::size_t surface_band_points = 0U;
            for (int point_index : evidence.point_indices) {
                if (point_index < 0 ||
                    static_cast<std::size_t>(point_index) >=
                        obstacle_cluster_source_cloud->size()) {
                    continue;
                }
                const pcl::PointXYZ& point =
                    obstacle_cluster_source_cloud->points[
                        static_cast<std::size_t>(point_index)];
                ++valid_cluster_points;
                if (predicted_self_footprint.valid &&
                    pointToCargoObbDistance2D(
                        Eigen::Vector2f(point.x, point.y),
                        predicted_self_footprint) <= 1.0e-4F) {
                    ++inside_xy_points;
                }
                if (predicted_self_footprint.valid &&
                    point.z >= predicted_self_footprint.min_z -
                        cargo_residual_surface_band_below_m_ &&
                    point.z <= predicted_self_footprint.max_z +
                        cargo_residual_surface_band_above_m_) {
                    ++surface_band_points;
                }
                if (identity_self_mask_valid &&
                    identity_self_tree.nearestKSearch(
                        point, 1, identity_nearest_index,
                        identity_nearest_distance_sq) > 0 &&
                    !identity_nearest_distance_sq.empty() &&
                    identity_nearest_distance_sq.front() <=
                        identity_radius_sq) {
                    ++identity_points;
                }
            }
            const float denominator = static_cast<float>(
                std::max<std::size_t>(1U, valid_cluster_points));
            evidence.inside_xy_obb_ratio =
                static_cast<float>(inside_xy_points) / denominator;
            evidence.identity_match_ratio =
                static_cast<float>(identity_points) / denominator;
            evidence.surface_band_ratio =
                static_cast<float>(surface_band_points) / denominator;

            const Eigen::Vector3d centroid_map_3d = pose_map_base *
                evidence.centroid_base.getVector3fMap().cast<double>();
            const CargoObstacleTrack* prior_track = nullptr;
            float prior_distance = std::numeric_limits<float>::infinity();
            for (const CargoObstacleTrack& track :
                 cargo_obstacle_tracker_.tracks()) {
                if (!track.confirmed) continue;
                const float distance =
                    (track.centroid_map -
                     centroid_map_3d.cast<float>()).norm();
                if (distance < prior_distance &&
                    distance <= cargo_obstacle_tracker_.config()
                        .association_max_centroid_distance_m) {
                    prior_distance = distance;
                    prior_track = &track;
                }
            }
            float motion_match_score = 0.0F;
            if (prior_track != nullptr && cargo_map_velocity_valid &&
                cargo_velocity_map_.norm() >=
                    cargo_motion_corridor_config_.minimum_motion_speed_mps) {
                const float scale = std::max(
                    0.10F, cargo_velocity_map_.norm());
                motion_match_score = std::clamp(
                    1.0F -
                        (prior_track->velocity_map.head<2>() -
                         cargo_velocity_map_).norm() / scale,
                    0.0F, 1.0F);
            }
            evidence.moves_with_cargo_score = motion_match_score;
            CargoResidualClassifierInput classifier_input;
            classifier_input.footprint_distance_m =
                evidence.footprint_distance_m;
            classifier_input.inside_xy_ratio =
                evidence.inside_xy_obb_ratio;
            classifier_input.identity_match_ratio =
                evidence.identity_match_ratio;
            classifier_input.surface_band_ratio =
                evidence.surface_band_ratio;
            classifier_input.moves_with_cargo_score = motion_match_score;
            classifier_input.independent_external_static_provenance =
                prior_track != nullptr && prior_track->static_obstacle &&
                prior_track->independent_external_provenance;
            CargoResidualClassifierConfig residual_config =
                cargo_residual_classifier_config_;
            residual_config.validation_shell_m = std::max({
                cargo_residual_classifier_config_.validation_shell_m,
                cargo_motion_corridor_config_.immediate_near_field_m,
                hook_fixed_config_.voxel_leaf +
                    formal_use.horizontal_uncertainty_m +
                    hook_lock_.horizontal_tracking_residual_m});
            const CargoResidualClassifierDecision classifier_decision =
                classifyCargoResidual(
                    residual_config, classifier_input);
            evidence.source_validated =
                classifier_decision.source_validated;
            evidence.source_reason = classifier_decision.reason;
            if (classifier_decision.classification ==
                CargoResidualClass::CARGO_SELF) {
                ++cargo_residual_self_clusters_;
                for (int point_index : evidence.point_indices) {
                    residual_self_point_indices.insert(point_index);
                }
                continue;
            }
            if (!classifier_decision.source_validated) {
                ++cargo_residual_unknown_clusters_;
                has_unresolved_residual = true;
            } else if (!has_validated_hazard) {
                most_dangerous_validated = evidence;
                has_validated_hazard = true;
            } else {
                const int candidate_priority =
                    evidence.warning_code ==
                            CargoSafetyEvaluator::kLevel1Code
                        ? 2 : 1;
                const int current_priority =
                    most_dangerous_validated.warning_code ==
                            CargoSafetyEvaluator::kLevel1Code
                        ? 2 : 1;
                if (candidate_priority > current_priority ||
                    (candidate_priority == current_priority &&
                     evidence.conservative_clearance_m <
                         most_dangerous_validated
                             .conservative_clearance_m)) {
                    most_dangerous_validated = evidence;
                }
            }
            classified_clusters.push_back(evidence);
        }
        raw_cargo_safety_result.cluster_evidence =
            std::move(classified_clusters);
        raw_cargo_safety_result.evaluated_cluster_count =
            raw_cargo_safety_result.cluster_evidence.size();
        if (has_validated_hazard) {
            raw_cargo_safety_result.has_cluster_evidence = true;
            raw_cargo_safety_result.most_dangerous_cluster =
                most_dangerous_validated;
            raw_cargo_safety_result.warning_code =
                most_dangerous_validated.warning_code;
            raw_cargo_safety_result.reason =
                most_dangerous_validated.source_reason;
        } else if (has_unresolved_residual) {
            raw_cargo_safety_result.input_valid = false;
            raw_cargo_safety_result.warning_valid = false;
            raw_cargo_safety_result.warning_code = 0U;
            raw_cargo_safety_result.has_cluster_evidence = true;
            raw_cargo_safety_result.fault =
                CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID;
            raw_cargo_safety_result.reason =
                "cargo_boundary_source_unresolved";
        } else {
            raw_cargo_safety_result.has_cluster_evidence = false;
            raw_cargo_safety_result.warning_valid = true;
            raw_cargo_safety_result.warning_code =
                CargoSafetyEvaluator::kSafeCode;
            raw_cargo_safety_result.fault = CargoSafetyFault::NONE;
            raw_cargo_safety_result.reason =
                "clear_cargo_residuals_classified_self";
        }
    }

    if (!residual_self_point_indices.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr classified_external(
            new pcl::PointCloud<pcl::PointXYZ>);
        classified_external->reserve(
            external_obstacle_cloud->size() -
            std::min<std::size_t>(
                external_obstacle_cloud->size(),
                residual_self_point_indices.size()));
        for (std::size_t point_index = 0U;
             point_index < external_obstacle_cloud->size(); ++point_index) {
            const pcl::PointXYZ& point =
                external_obstacle_cloud->points[point_index];
            if (residual_self_point_indices.count(
                    static_cast<int>(point_index)) > 0U) {
                self_removed_cloud->push_back(point);
            } else {
                classified_external->push_back(point);
            }
        }
        external_obstacle_cloud = classified_external;
        cargo_self_removed_points_ = self_removed_cloud->size();
        cargo_external_obstacle_points_ = external_obstacle_cloud->size();
    }
    publish_safety_debug_cloud(
        self_removed_cloud, cargo_self_removed_pub_);
    publish_safety_debug_cloud(
        external_obstacle_cloud, cargo_external_obstacle_pub_);
    cargo_raw_warning_code_ = raw_cargo_safety_result.warning_valid
        ? raw_cargo_safety_result.warning_code : 0;
    last_cargo_safety_result_ = raw_cargo_safety_result;
    std::vector<CargoObstacleObservation> obstacle_track_observations;
    if (radial_safety_result.input_valid &&
        radial_safety_result.warning_valid &&
        radial_safety_result.fault == CargoSafetyFault::NONE) {
        obstacle_track_observations.reserve(
            raw_cargo_safety_result.cluster_evidence.size());
        for (std::size_t evidence_index = 0U;
             evidence_index < raw_cargo_safety_result.cluster_evidence.size();
             ++evidence_index) {
            const CargoSafetyClusterEvidence& evidence =
                raw_cargo_safety_result.cluster_evidence[evidence_index];
            if (evidence.warning_code != CargoSafetyEvaluator::kLevel1Code &&
                evidence.warning_code != CargoSafetyEvaluator::kLevel2Code) {
                continue;
            }
            const Eigen::Vector3d centroid_map = pose_map_base *
                evidence.centroid_base.getVector3fMap().cast<double>();
            const Eigen::Vector3d top_map = pose_map_base * Eigen::Vector3d(
                evidence.centroid_base.x, evidence.centroid_base.y,
                evidence.obstacle_top_z95_m);
            CargoObstacleObservation observation;
            observation.source_index = evidence_index;
            observation.centroid_map = centroid_map.cast<float>();
            observation.top_z95_map = static_cast<float>(top_map.z());
            observation.footprint_distance_m =
                evidence.footprint_distance_m;
            observation.conservative_clearance_m =
                evidence.conservative_clearance_m;
            observation.point_count = evidence.point_count;
            // The subscribed obstacle cloud is already voxelized, so an
            // honest raw-return count is unavailable. Keep raw-equivalent at
            // zero; production can enable that gate only when an upstream raw
            // count is carried explicitly.
            observation.raw_equivalent_point_count = 0U;
            float min_x = std::numeric_limits<float>::infinity();
            float max_x = -std::numeric_limits<float>::infinity();
            float min_y = std::numeric_limits<float>::infinity();
            float max_y = -std::numeric_limits<float>::infinity();
            float min_z = std::numeric_limits<float>::infinity();
            float max_z = -std::numeric_limits<float>::infinity();
            std::set<std::pair<int, int>> occupied_cells;
            constexpr float kStaticCargoCellM = 0.25F;
            for (int point_index : evidence.point_indices) {
                if (point_index < 0 ||
                    static_cast<std::size_t>(point_index) >=
                        obstacle_cluster_source_cloud->size()) {
                    continue;
                }
                const pcl::PointXYZ& point =
                    obstacle_cluster_source_cloud->points[
                        static_cast<std::size_t>(point_index)];
                min_x = std::min(min_x, point.x);
                max_x = std::max(max_x, point.x);
                min_y = std::min(min_y, point.y);
                max_y = std::max(max_y, point.y);
                min_z = std::min(min_z, point.z);
                max_z = std::max(max_z, point.z);
                occupied_cells.emplace(
                    static_cast<int>(std::floor(point.x / kStaticCargoCellM)),
                    static_cast<int>(std::floor(point.y / kStaticCargoCellM)));
            }
            if (std::isfinite(min_x) && std::isfinite(max_x) &&
                std::isfinite(min_y) && std::isfinite(max_y) &&
                std::isfinite(min_z) && std::isfinite(max_z)) {
                const float extent_x = std::max(0.0F, max_x - min_x);
                const float extent_y = std::max(0.0F, max_y - min_y);
                observation.xy_area_m2 = extent_x * extent_y;
                observation.long_side_m = std::max(extent_x, extent_y);
                observation.height_span_m = std::max(0.0F, max_z - min_z);
                observation.occupied_cells = occupied_cells.size();
            }
            observation.warning_code = evidence.warning_code;
            observation.source_validated = evidence.source_validated;
            const float residual_validation_shell_m = std::max({
                cargo_residual_classifier_config_.validation_shell_m,
                cargo_motion_corridor_config_.immediate_near_field_m,
                hook_fixed_config_.voxel_leaf +
                    formal_use.horizontal_uncertainty_m +
                    hook_lock_.horizontal_tracking_residual_m});
            observation.independent_external_provenance =
                evidence.source_validated &&
                evidence.footprint_distance_m >
                    residual_validation_shell_m;
            obstacle_track_observations.push_back(observation);
        }
    }
    const CargoObstacleTrackerDecision obstacle_track_decision =
        cargo_obstacle_tracker_.update(
            stamp.toSec(), obstacle_track_observations);
    cargo_obstacle_track_id_ = obstacle_track_decision.selected_track_id;
    cargo_obstacle_track_age_sec_ =
        obstacle_track_decision.selected_track_age_sec;
    cargo_obstacle_track_confirm_count_ =
        obstacle_track_decision.selected_confirm_count;
    cargo_obstacle_track_static_ =
        obstacle_track_decision.selected_track_static;
    cargo_obstacle_track_velocity_map_ =
        obstacle_track_decision.selected_track_velocity;

    const auto make_obstacle_pending = [&](const std::string& reason) {
        last_cargo_safety_result_.input_valid = false;
        last_cargo_safety_result_.warning_valid = false;
        last_cargo_safety_result_.warning_code = 0U;
        last_cargo_safety_result_.fault =
            CargoSafetyFault::OBSTACLE_EVIDENCE_INVALID;
        last_cargo_safety_result_.reason = reason;
    };

    if (raw_cargo_safety_result.input_valid &&
        raw_cargo_safety_result.warning_valid &&
        raw_cargo_safety_result.fault == CargoSafetyFault::NONE &&
        (raw_cargo_safety_result.warning_code ==
             CargoSafetyEvaluator::kLevel1Code ||
         raw_cargo_safety_result.warning_code ==
             CargoSafetyEvaluator::kLevel2Code)) {
        cargo_safety_temporal_filter_.reset();
        confirmed_cargo_safety_result_ = CargoSafetyResult{};
        cargo_temporal_candidate_code_ =
            raw_cargo_safety_result.warning_code;
        cargo_temporal_candidate_count_ =
            obstacle_track_decision.selected_confirm_count;
        cargo_used_previous_confirmation_ = false;
        if (obstacle_track_decision.confirmed_hazard &&
            obstacle_track_decision.selected_source_index <
                raw_cargo_safety_result.cluster_evidence.size()) {
            const CargoSafetyClusterEvidence& selected =
                raw_cargo_safety_result.cluster_evidence[
                    obstacle_track_decision.selected_source_index];
            last_cargo_safety_result_.has_cluster_evidence = true;
            last_cargo_safety_result_.most_dangerous_cluster = selected;
            last_cargo_safety_result_.warning_code =
                obstacle_track_decision.warning_code;
            last_cargo_safety_result_.reason =
                obstacle_track_decision.reason;
            cargo_confirmed_warning_code_ =
                obstacle_track_decision.warning_code;
            confirmed_cargo_safety_result_ = last_cargo_safety_result_;
        } else {
            cargo_confirmed_warning_code_ = 0;
            make_obstacle_pending(obstacle_track_decision.reason);
        }
    } else if (raw_cargo_safety_result.input_valid &&
               raw_cargo_safety_result.warning_valid &&
               raw_cargo_safety_result.fault == CargoSafetyFault::NONE) {
        // CLEAR confirmation remains a separate two-frame lifecycle guard.
        CargoSafetyTemporalInput temporal_input;
        temporal_input.stamp_sec = stamp.toSec();
        temporal_input.raw_valid = true;
        temporal_input.raw_code = CargoSafetyEvaluator::kSafeCode;
        const CargoSafetyTemporalDecision temporal_decision =
            cargo_safety_temporal_filter_.update(temporal_input);
        cargo_confirmed_warning_code_ = temporal_decision.confirmed_code;
        cargo_temporal_candidate_code_ = temporal_decision.candidate_code;
        cargo_temporal_candidate_count_ = temporal_decision.evidence_count;
        cargo_used_previous_confirmation_ =
            temporal_decision.stable &&
            !temporal_decision.use_current_evidence;
        if (!temporal_decision.stable) {
            make_obstacle_pending(temporal_decision.reason);
        } else {
            last_cargo_safety_result_.warning_code =
                temporal_decision.code;
            last_cargo_safety_result_.reason = temporal_decision.reason;
            confirmed_cargo_safety_result_ = last_cargo_safety_result_;
        }
    } else {
        // Faults remain immediate and cannot be used as temporal evidence.
        cargo_safety_temporal_filter_.reset();
        confirmed_cargo_safety_result_ = CargoSafetyResult{};
        cargo_confirmed_warning_code_ = 0;
        cargo_temporal_candidate_code_ = 0;
        cargo_temporal_candidate_count_ = 0;
        cargo_used_previous_confirmation_ = false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr dangerous_cluster_debug(
        new pcl::PointCloud<pcl::PointXYZ>);
    cargo_nearest_cluster_center_.setZero();
    cargo_nearest_obstacle_point_.setZero();
    cargo_nearest_cluster_distance_m_ =
        std::numeric_limits<float>::infinity();
    cargo_dangerous_cluster_points_ = 0U;
    cargo_obstacle_top_z95_m_ =
        std::numeric_limits<float>::quiet_NaN();
    cargo_obstacle_uncertainty_m_ =
        std::numeric_limits<float>::quiet_NaN();
    cargo_conservative_clearance_m_ =
        std::numeric_limits<float>::quiet_NaN();
    if (last_cargo_safety_result_.has_cluster_evidence) {
        const CargoSafetyClusterEvidence& evidence =
            last_cargo_safety_result_.most_dangerous_cluster;
        const Eigen::Vector3f seed =
            evidence.nearest_point_base.getVector3fMap();
        cargo_nearest_cluster_center_ =
            evidence.centroid_base.getVector3fMap();
        const float debug_radius = std::max(
            0.05F,
            cargo_safety_evaluator_.config()
                .obstacle_cluster_tolerance_m);
        for (const pcl::PointXYZ& point :
             external_obstacle_cloud->points) {
            if ((point.getVector3fMap() - seed).norm() <= debug_radius) {
                dangerous_cluster_debug->push_back(point);
            }
        }
        cargo_nearest_cluster_distance_m_ =
            evidence.footprint_distance_m;
        cargo_dangerous_cluster_points_ = evidence.point_count;
        cargo_nearest_obstacle_point_ = seed;
        cargo_obstacle_top_z95_m_ = evidence.obstacle_top_z95_m;
        cargo_obstacle_uncertainty_m_ =
            evidence.obstacle_uncertainty_m;
        cargo_conservative_clearance_m_ =
            evidence.conservative_clearance_m;
    }
    publish_safety_debug_cloud(
        dangerous_cluster_debug, cargo_most_dangerous_cluster_pub_);

    const auto build_safety_message = [&](const CargoSafetyResult& result) {
        lidar_slam2_msgs::CargoSafetyStatus message;
        message.header.stamp = stamp;
        message.header.frame_id = map_frame_;
        message.schema_version =
            lidar_slam2_msgs::CargoSafetyStatus::SCHEMA_VERSION;
        message.cargo_valid = last_cargo_bottom_result_.valid;
        message.cargo_track_id = bottom_msg.track_id;
        message.cargo_source = bottom_msg.source;
        message.hook_signal_valid = hook.valid;
        message.hook_load_state = hook.state;
        message.hook_voltage = hook.voltage;
        message.no_cargo_confirmed = false;
        message.cargo_bottom_z_map = bottom_msg.bottom_z_map;
        message.cargo_bottom_uncertainty_m = bottom_msg.uncertainty_m;
        message.obstacle_valid = result.input_valid &&
            result.fault == CargoSafetyFault::NONE;
        message.obstacle_count = static_cast<std::uint32_t>(
            std::min<std::size_t>(
                result.evaluated_cluster_count,
                std::numeric_limits<std::uint32_t>::max()));
        if (result.has_cluster_evidence) {
            const auto& evidence = result.most_dangerous_cluster;
            message.nearest_obstacle_distance_m =
                evidence.footprint_distance_m;
            const Eigen::Vector3d obstacle_base(
                evidence.nearest_point_base.x,
                evidence.nearest_point_base.y,
                evidence.obstacle_top_z95_m);
            const Eigen::Vector3d obstacle_map =
                pose_map_base * obstacle_base;
            message.obstacle_top_z_map =
                static_cast<float>(obstacle_map.z());
            message.obstacle_uncertainty_m =
                evidence.obstacle_uncertainty_m;
            message.conservative_vertical_clearance_m =
                evidence.conservative_clearance_m;
        }
        message.confidence = last_cargo_bottom_result_.valid &&
                             result.input_valid
            ? last_cargo_bottom_result_.confidence : 0.0F;
        const std::string reason = last_cargo_bottom_result_.valid
            ? result.reason
            : std::string("cargo_bottom_invalid:") +
                  last_cargo_bottom_result_.reason;
        return composeCargoSafetyStatus(
            message, false, result.fault, result.warning_code,
            result.warning_valid, reason);
    };

    const lidar_slam2_msgs::CargoSafetyStatus raw_safety_msg =
        build_safety_message(raw_cargo_safety_result);
    cargo_raw_safety_status_pub_.publish(raw_safety_msg);
    std_msgs::Int32 raw_status_code_msg;
    raw_status_code_msg.data = raw_safety_msg.requested_alarm_code;
    cargo_raw_status_code_pub_.publish(raw_status_code_msg);

    lidar_slam2_msgs::CargoSafetyStatus safety_msg =
        build_safety_message(last_cargo_safety_result_);
    cargo_safety_status_pub_.publish(safety_msg);
    logCargoSafetyStatus(safety_msg);
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

        ROS_DEBUG_THROTTLE(1.0,
            "[CargoSourceMode] mode=hook_local_roi global_dynamic_track=debug_only source=%s",
            box_source == 1.0f ? "V2_CORE" : "LAST_GOOD");

        ROS_DEBUG_THROTTLE(1.0,
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

        ROS_DEBUG_THROTTLE(1.0,
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

        ROS_DEBUG_THROTTLE(1.0,
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

    ROS_DEBUG_THROTTLE(1.0,
        "[CargoTargetHardCheck] source=hook_roi selected=-1 payload=-1 match=0 reason=%s state=%d",
        reason.c_str(), static_cast<int>(hook_lock_.state));

    payload_track_info_pub_.publish(msg);
}

// ========== 构建锁定的 odom-fixed cargo box ==========
void NdtSlamNode::buildLockedOdomFixedCargoBox(const ros::Time& stamp) {
    // 只有在 LOCKED 或 LOST_HOLD 状态下才构建 box
    if (hook_lock_.state != HookCargoLockState::LOCKED &&
        hook_lock_.state != HookCargoLockState::LOST_HOLD) {
        return;
    }

    if (!hook_lock_.has_locked_size) {
        return;
    }

    // 强制使用 anchor
    auto anchor = getCargoAnchorXY();
    float cx = anchor.x();
    float cy = anchor.y();

    Eigen::Vector3f size = hook_lock_.locked_size;
    float zmin = hook_lock_.stable_bottom_z;
    float zmax = hook_lock_.stable_top_z;

    // 构建 odom-fixed bbox
    Eigen::Vector3f bbox_min(cx - size.x() * 0.5f, cy - size.y() * 0.5f, zmin);
    Eigen::Vector3f bbox_max(cx + size.x() * 0.5f, cy + size.y() * 0.5f, zmax);
    Eigen::Vector3f center(cx, cy, 0.5f * (zmin + zmax));

    // 发布 CargoMarkerBaseLink
    ROS_DEBUG_THROTTLE(2.0,
        "[CargoMarkerBaseLink] center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) z=[%.2f,%.2f]",
        center.x(), center.y(), center.z(),
        size.x(), size.y(), size.z(),
        zmin, zmax);
}

// ========== payload_precise_box_info 发布 ==========

// 注：payload_precise_box_info 发布逻辑已移至 HookFixedCargoDetector
// 当前版本使用 HookFixedCargoDetector 作为主要来源，不再发布 payload_precise_box_info

// Commit C: 发布 payload_precise_box_info

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
    const ndt_slam::CranePoseRpy rpy = ndt_slam::cranePoseRpy(r);
    roll = rpy.roll;
    pitch = rpy.pitch;
    yaw = rpy.yaw;
}

Sophus::SE3d NdtSlamNode::applyCraneMotionConstraint(const Sophus::SE3d& raw_pose, const std::string& stage) {
    if (!crane_constraint_enabled_) {
        return raw_pose;
    }

    Eigen::Vector3d t = raw_pose.translation();
    const ndt_slam::CranePoseRpy raw_rpy =
        ndt_slam::cranePoseRpy(raw_pose.so3());
    if (!t.allFinite() || !raw_rpy.valid) {
        crane_constraint_invalid_input_count_.fetch_add(1, std::memory_order_relaxed);
        ROS_ERROR_THROTTLE(2.0,
                           "[CraneConstraint:%s] rejected non-finite runtime pose",
                           stage.c_str());
        return current_pose_;
    }

    double roll = raw_rpy.roll;
    double pitch = raw_rpy.pitch;
    double yaw = raw_rpy.yaw;

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

    ndt_slam::CranePoseConstraintConfig config;
    config.enabled = true;
    config.lock_z = lock_z_;
    config.fixed_z = fixed_z_;
    config.constrain_z = constrain_z_;
    config.max_abs_z_drift = max_abs_z_drift_;
    config.lock_roll = lock_roll_;
    config.fixed_roll_rad = fixed_roll_ * M_PI / 180.0;
    config.constrain_roll = !lock_roll_;
    config.max_abs_roll_rad = max_roll_deg_ * M_PI / 180.0;
    config.lock_pitch = lock_pitch_;
    config.fixed_pitch_rad = fixed_pitch_ * M_PI / 180.0;
    config.constrain_pitch = !lock_pitch_;
    config.max_abs_pitch_rad = max_pitch_deg_ * M_PI / 180.0;
    config.lock_yaw = lock_yaw_;
    config.fixed_yaw_rad = fixed_yaw_ * M_PI / 180.0;
    config.constrain_yaw = constrain_yaw_;
    config.max_abs_yaw_delta_rad = max_yaw_deg_ * M_PI / 180.0;

    const ndt_slam::CranePoseConstraintResult result =
        ndt_slam::applyCranePoseConstraint(raw_pose, config, {false, 0.0});
    if (!result.valid) {
        crane_constraint_invalid_input_count_.fetch_add(1, std::memory_order_relaxed);
        ROS_ERROR_THROTTLE(2.0,
                           "[CraneConstraint:%s] rejected invalid pose: %s",
                           stage.c_str(), result.reason.c_str());
        return current_pose_;
    }
    if (result.fallback_used) {
        crane_constraint_fallback_count_.fetch_add(1, std::memory_order_relaxed);
        ROS_ERROR_THROTTLE(
            2.0,
            "[CraneConstraint:%s] reconstruction fallback to raw pose: %s",
            stage.c_str(), result.reason.c_str());
    }

    const Sophus::SE3d constrained_pose = result.pose;
    t = constrained_pose.translation();
    const ndt_slam::CranePoseRpy constrained_rpy =
        ndt_slam::cranePoseRpy(constrained_pose.so3());
    roll = constrained_rpy.roll;
    pitch = constrained_rpy.pitch;
    yaw = constrained_rpy.yaw;

    // 日志：debug_pose_flow 开启时输出
    if (debug_cfg_.debug_pose_flow) {
        ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec,
                          "[CraneConstraint:%s] raw_z=%.3f -> z=%.3f, "
                          "raw_rpy=(%.2f, %.2f, %.2f)deg -> rpy=(%.2f, %.2f, %.2f)deg",
                          stage.c_str(),
                          raw_z, t.z(),
                          raw_roll, raw_pitch, raw_yaw,
                          roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    }

    return constrained_pose;
}

// ============================================================================
// v8-stable-r3: SoftYawFilter
// ============================================================================

static inline double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static inline double clampDouble(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
}

double NdtSlamNode::updateSoftYaw(double raw_yaw,
                                  double speed_xy,
                                  bool is_stationary) {
    if (!soft_yaw_enabled_) {
        return raw_yaw;
    }

    if (!filtered_yaw_initialized_) {
        filtered_yaw_rad_ = raw_yaw;
        filtered_yaw_initialized_ = true;
        return filtered_yaw_rad_;
    }

    double alpha = is_stationary
        ? yaw_filter_alpha_stationary_
        : yaw_filter_alpha_moving_ +
          yaw_filter_alpha_speed_extra_ * std::min(speed_xy / 1.0, 1.0);

    alpha = clampDouble(alpha, 0.01, 0.80);

    const double max_step = is_stationary
        ? yaw_max_step_stationary_rad_
        : yaw_max_step_moving_rad_;

    double dyaw = normalizeAngle(raw_yaw - filtered_yaw_rad_);

    if (std::abs(dyaw) > yaw_warn_raw_filtered_diff_rad_) {
        ROS_WARN_THROTTLE(
            2.0,
            "[SoftYaw] raw-filter diff large: raw=%.2fdeg filtered=%.2fdeg diff=%.2fdeg",
            raw_yaw * 180.0 / M_PI,
            filtered_yaw_rad_ * 180.0 / M_PI,
            dyaw * 180.0 / M_PI);
    }

    dyaw = clampDouble(dyaw, -max_step, max_step);

    filtered_yaw_rad_ = normalizeAngle(filtered_yaw_rad_ + alpha * dyaw);

    return filtered_yaw_rad_;
}

Sophus::SE3d NdtSlamNode::applyCraneOutputConstraint(
    const Sophus::SE3d& pose_in,
    bool is_stationary,
    double speed_xy) {
    const ndt_slam::CranePoseRpy input_rpy =
        ndt_slam::cranePoseRpy(pose_in.so3());
    if (!pose_in.translation().allFinite() || !input_rpy.valid ||
        !std::isfinite(speed_xy)) {
        crane_constraint_invalid_input_count_.fetch_add(1, std::memory_order_relaxed);
        ROS_ERROR_THROTTLE(2.0,
                           "[CraneConstraint:runtime] rejected non-finite pose/context");
        return current_pose_;
    }

    const double filtered_yaw =
        updateSoftYaw(input_rpy.yaw, speed_xy, is_stationary);
    ndt_slam::CranePoseConstraintConfig config;
    config.enabled = true;
    config.lock_z = true;
    config.fixed_z = fixed_z_;
    config.lock_roll = true;
    config.fixed_roll_rad = 0.0;
    config.lock_pitch = true;
    config.fixed_pitch_rad = 0.0;
    config.lock_yaw = true;
    config.fixed_yaw_rad = filtered_yaw;

    const ndt_slam::CranePoseConstraintResult result =
        ndt_slam::applyCranePoseConstraint(
            pose_in, config, {is_stationary, speed_xy});
    if (!result.valid) {
        crane_constraint_invalid_input_count_.fetch_add(1, std::memory_order_relaxed);
        ROS_ERROR_THROTTLE(2.0,
                           "[CraneConstraint:runtime] rejected invalid pose: %s",
                           result.reason.c_str());
        return current_pose_;
    }
    if (result.fallback_used) {
        crane_constraint_fallback_count_.fetch_add(1, std::memory_order_relaxed);
        ROS_ERROR_THROTTLE(
            2.0,
            "[CraneConstraint:runtime] reconstruction fallback to raw pose: %s",
            result.reason.c_str());
    }
    return result.pose;
}

// ============================================================================
// v8-stable-r3: NDT 输入减负函数
// ============================================================================

pcl::PointCloud<pcl::PointXYZ>::Ptr NdtSlamNode::sampleCloudByRatio(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double ratio) {
    auto out = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (!cloud || cloud->empty() || ratio <= 0.0) {
        return out;
    }

    if (ratio >= 1.0) {
        *out = *cloud;
        return out;
    }

    const int step = std::max(1, static_cast<int>(std::round(1.0 / ratio)));
    out->reserve(cloud->size() / step + 1);

    for (size_t i = 0; i < cloud->size(); i += step) {
        out->push_back((*cloud)[i]);
    }

    out->width = out->size();
    out->height = 1;
    out->is_dense = false;

    return out;
}

void NdtSlamNode::voxelDownsampleInPlace(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double leaf) {
    if (!cloud || cloud->empty() || leaf <= 0.0) {
        return;
    }

    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setLeafSize(leaf, leaf, leaf);
    vf.setInputCloud(cloud);

    auto filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    vf.filter(*filtered);

    cloud.swap(filtered);
}

void NdtSlamNode::limitCloudUniformInPlace(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    int max_points) {
    if (!cloud || max_points <= 0 ||
        static_cast<int>(cloud->size()) <= max_points) {
        return;
    }

    auto limited = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    limited->reserve(max_points);

    const double step =
        static_cast<double>(cloud->size()) / static_cast<double>(max_points);

    for (int i = 0; i < max_points; ++i) {
        const size_t idx =
            std::min(static_cast<size_t>(std::floor(i * step)),
                     cloud->size() - 1);
        limited->push_back((*cloud)[idx]);
    }

    limited->width = limited->size();
    limited->height = 1;
    limited->is_dense = false;

    cloud.swap(limited);
}

// ============================================================================
// V3: Localization Target 管理
// ============================================================================

bool NdtSlamNode::updateLocalizationTarget(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud,
    const Sophus::SE3d& pose) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate(
        new pcl::PointCloud<pcl::PointXYZ>);

    if (objects_cloud) {
        const Eigen::Vector3d center = pose.translation();
        candidate->reserve(objects_cloud->size());
        std::size_t processed = 0U;
        for (const auto& p : objects_cloud->points) {
            if (++processed % 2048U == 0U && localizationInputPending()) {
                return false;
            }
            const double dx = p.x - center.x();
            const double dy = p.y - center.y();
            if (dx * dx + dy * dy < 900.0) {
                candidate->push_back(p);
            }
        }
    }

    // The source is a map-frame snapshot, not a per-keyframe delta.  Rebuild
    // from it instead of repeatedly appending the complete map to itself.
    if (!candidate->empty()) {
        if (localizationInputPending()) return false;
        pcl::VoxelGrid<pcl::PointXYZ> vf;
        vf.setInputCloud(candidate);
        vf.setLeafSize(localization_target_voxel_size_,
                       localization_target_voxel_size_,
                       localization_target_voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> filtered;
        vf.filter(filtered);
        candidate.reset(new pcl::PointCloud<pcl::PointXYZ>(filtered));
    }

    if (static_cast<int>(candidate->size()) > localization_target_max_points_) {
        limitCloudUniformInPlace(candidate, localization_target_max_points_);
    }

    std::lock_guard<std::mutex> lock(localization_target_mutex_);
    if (static_cast<int>(candidate->size()) < localization_target_min_points_) {
        const bool previously_ready = localization_target_ready_;
        localization_target_ready_ = false;
        localization_target_state_ = previously_ready
            ? LocalizationTargetState::TARGET_DEGRADED
            : LocalizationTargetState::BUILDING_TARGET;
        localization_target_back_ = candidate;
        cached_target_valid_ = false;
        cached_target_points_ = 0;
        last_target_reason_ = "candidate_below_min_points";
        ROS_DEBUG_THROTTLE(
            2.0,
            "[LocTarget] candidate rejected after crop/voxel: points=%zu min=%d; using local_map",
            candidate->size(), localization_target_min_points_);
        return true;
    }

    localization_target_back_ = candidate;
    localization_target_state_ = LocalizationTargetState::BUILDING_TARGET;
    last_target_reason_ = "candidate_ready_to_swap";
    return true;
}
bool NdtSlamNode::swapLocalizationTargetBuffers() {
    std::lock_guard<std::mutex> lock(localization_target_mutex_);

    if (!localization_target_back_ ||
        static_cast<int>(localization_target_back_->size()) <
            localization_target_min_points_) {
        localization_target_ready_ = false;
        localization_target_state_ = localization_target_snapshot_->empty()
            ? LocalizationTargetState::BUILDING_TARGET
            : LocalizationTargetState::TARGET_DEGRADED;
        cached_target_valid_ = false;
        last_target_reason_ = "swap_below_min_points";
        return false;
    }

    localization_target_front_ = localization_target_back_;
    localization_target_back_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    localization_target_snapshot_ = localization_target_front_;
    localization_target_version_++;
    localization_target_snapshot_version_ = localization_target_version_;
    localization_target_ready_ = true;
    localization_target_state_ = LocalizationTargetState::TARGET_READY;
    cached_target_valid_ = false;
    cached_target_points_ = 0;
    last_target_reason_ = "snapshot_swapped";

    ROS_DEBUG("[LocTarget] READY: version=%llu points=%zu min=%d",
             static_cast<unsigned long long>(localization_target_version_),
             localization_target_snapshot_->size(),
             localization_target_min_points_);
    return true;
}

bool NdtSlamNode::updateCroppedCachedTarget(const Sophus::SE3d& predicted_pose) {
    crop_frames_since_update_++;

    const Eigen::Vector3d pred_pos = predicted_pose.translation();
    const double pred_yaw = predicted_pose.so3().log().z();

    const double dist =
        (pred_pos.head<2>() - cached_center_xy_.head<2>()).norm();
    double yaw_diff = std::abs(pred_yaw - cached_yaw_);
    if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_snapshot;
    uint64_t snapshot_version = 0;
    {
        std::lock_guard<std::mutex> lock(localization_target_mutex_);
        target_snapshot = localization_target_snapshot_;
        snapshot_version = localization_target_snapshot_version_;
    }

    const bool version_changed = snapshot_version != cached_target_version_;
    const bool moved = dist > crop_update_distance_m_ ||
        yaw_diff > (crop_update_yaw_deg_ * M_PI / 180.0);
    const bool interval_elapsed =
        crop_frames_since_update_ >= crop_update_min_interval_frames_;
    const bool need_rebuild = !cached_target_valid_ || version_changed ||
        (interval_elapsed && moved);

    if (!need_rebuild) {
        return cached_target_valid_;
    }

    if (!target_snapshot ||
        static_cast<int>(target_snapshot->size()) <
            localization_target_min_points_) {
        cached_target_valid_ = false;
        cached_target_points_ = 0;
        localization_target_ready_ = false;
        localization_target_state_ = LocalizationTargetState::TARGET_DEGRADED;
        last_target_reason_ = "snapshot_below_min_points";
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
    cropped->reserve(target_snapshot->size());
    for (const auto& p : target_snapshot->points) {
        const double dx = p.x - pred_pos.x();
        const double dy = p.y - pred_pos.y();
        if (std::abs(dx) < crop_radius_x_ && std::abs(dy) < crop_radius_y_) {
            cropped->push_back(p);
        }
    }

    if (static_cast<int>(cropped->size()) < localization_target_min_points_) {
        cached_target_valid_ = false;
        cached_target_points_ = static_cast<int>(cropped->size());
        localization_target_ready_ = false;
        localization_target_state_ = LocalizationTargetState::TARGET_DEGRADED;
        last_target_reason_ = "crop_below_min_points";
        ROS_DEBUG_THROTTLE(
            2.0,
            "[LocTarget] crop invalidated: points=%zu min=%d; using local_map",
            cropped->size(), localization_target_min_points_);
        return false;
    }

    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setInputCloud(cropped);
    vf.setLeafSize(localization_target_voxel_size_,
                   localization_target_voxel_size_,
                   localization_target_voxel_size_);
    pcl::PointCloud<pcl::PointXYZ> filtered;
    vf.filter(filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered));
    if (static_cast<int>(filtered_ptr->size()) > localization_target_max_points_) {
        limitCloudUniformInPlace(filtered_ptr, localization_target_max_points_);
    }

    if (static_cast<int>(filtered_ptr->size()) < localization_target_min_points_) {
        cached_target_valid_ = false;
        cached_target_points_ = static_cast<int>(filtered_ptr->size());
        localization_target_ready_ = false;
        localization_target_state_ = LocalizationTargetState::TARGET_DEGRADED;
        last_target_reason_ = "crop_voxel_below_min_points";
        ROS_DEBUG_THROTTLE(
            2.0,
            "[LocTarget] voxelized crop invalidated: points=%zu min=%d; using local_map",
            filtered_ptr->size(), localization_target_min_points_);
        return false;
    }

    cached_target_ = filtered_ptr;
    cached_center_xy_ = pred_pos;
    cached_yaw_ = pred_yaw;
    cached_target_version_ = snapshot_version;
    cached_target_points_ = static_cast<int>(cached_target_->size());
    cached_target_valid_ = true;
    crop_frames_since_update_ = 0;
    localization_target_state_ = LocalizationTargetState::TARGET_READY;
    last_target_reason_ = "crop_ready";
    return true;
}

void NdtSlamNode::bindNdtInputTarget(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target,
    const std::string& source,
    uint64_t content_version,
    const std::string& reason) {
    if (!target || target->empty()) {
        last_target_points_ = 0;
        last_actual_target_source_ = "invalid";
        last_target_reason_ = "empty_target";
        ROS_ERROR_THROTTLE(1.0, "[LocTarget] refusing to bind an empty NDT target");
        return;
    }

    const bool changed = !last_bound_ndt_target_ ||
        last_bound_ndt_target_.get() != target.get() ||
        last_bound_ndt_target_version_ != content_version ||
        last_bound_ndt_target_source_ != source;

    if (changed) {
        ndt_->setInputTarget(target);
        last_bound_ndt_target_ = target;
        last_bound_ndt_target_version_ = content_version;
        last_bound_ndt_target_source_ = source;
        ++setInputTarget_count_;
        ++target_rebuild_count_;
        ROS_DEBUG("[LocTarget] bind source=%s version=%llu points=%zu reason=%s setInput=%d",
                 source.c_str(),
                 static_cast<unsigned long long>(content_version),
                 target->size(), reason.c_str(), setInputTarget_count_);
    }

    target_version_ = content_version;
    last_target_points_ = static_cast<int>(target->size());
    last_actual_target_source_ = source;
    last_target_reason_ = reason;
}

RegistrationCloudBuildResult NdtSlamNode::buildRegistrationCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& human_safe_objects,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& uncertain_candidates) {
    return buildStructurePreservingRegistrationCloud(
        human_safe_objects, uncertain_candidates, ground_cloud,
        registration_cloud_config_);
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

// CRITICAL RUNTIME CHAIN - DO NOT MODIFY
// a7be4bf runtime pose chain must stay unchanged:
// NDT/refined/EKF -> publishOdometry -> TF -> publishRuntimePath.
// Runtime motion constraints are applied inside the EKF before publication;
// the later MapCommit gate remains read-only.
// raw_ndt_pose is allowed only as MapCommit evidence.
// Do NOT route raw_pose/tracking_pose to odom/TF/runtime_path/current_cloud.
bool NdtSlamNode::commitKeyFrameWithDynamicFiltering(
    const MapCommitJob& job)
{
    if (!job.cloud || job.cloud->empty()) {
        ROS_WARN_THROTTLE(1.0, "[KeyFrameCommit] empty cloud, skip");
        return false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>(*job.cloud));
    const Sophus::SE3d& pose = job.pose;
    const ros::Time& stamp = job.stamp;
    // ------------------------------------------------------------------------
    // 0. 基础准备 + DuplicateFrameGuard（内容指纹）+ MotionGate
    // ------------------------------------------------------------------------
    // P0: DuplicateFrameGuard 使用内容指纹（在 NDT 之前拦截）
    auto sig = computeFrameSignature(cloud, stamp, pose);

    if (isDuplicateFrameBySignature(sig)) {
        skipped_duplicate_frames_++;
        ROS_WARN_THROTTLE(2.0,
            "[DuplicateFrameGuard] skip duplicate frame stamp=%.3f cloud_size=%zu hash=%lu skipped=%lu",
            sig.stamp, sig.cloud_size, sig.hash, skipped_duplicate_frames_);
        return false;
    }

    last_frame_signature_ = sig;
    last_processed_stamp_ = stamp.toSec();
    frame_seq_++;

    // [FrameStart] 日志：debug_frame_start 开启时输出
    if (debug_cfg_.debug_frame_start) {
        ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec,
            "[FrameStart] frame=%lu stamp=%.3f raw=%zu pose=(%.2f,%.2f,%.2f)",
            frame_seq_, stamp.toSec(), cloud->size(),
            pose.translation().x(), pose.translation().y(), pose.translation().z());
    }

    // MotionGate was evaluated by the caller before entering this expensive
    // MapCommit pipeline. It is never consulted by the runtime pose chain.

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

    // Gate: 旧 global cargo 链路（OdomAnchorBox 模式下默认关闭）
    bool run_legacy_cargo = payload_tracker_config_.enabled &&
                            channel_filter_config_.enabled &&
                            payload_candidates && !payload_candidates->empty() &&
                            (!odom_anchor_config_.enabled || odom_anchor_config_.use_global_payload_tracker);

    if (run_legacy_cargo)
    {
        // 3.1 先更新 PayloadTracker
        std::map<CellKey, float> empty_ground_model;
        payload_track_result = payload_tracker_.update(
            payload_candidates, T_map_base, stamp.toSec(), empty_ground_model);

        // 3.2 再对每个 track 估计 CargoBoxV2
        if (cargo_box_estimator_config_.enabled && odom_anchor_config_.use_cargobox_v2) {
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
                // P0-6: 计算 is_locked_track
                bool is_locked_track = track.observed_frames >= 3 ||
                                       track.has_last_core_box;
                bool box_valid = cargo_box_estimator_.estimateCargoBox(
                    cluster_base, ground_model,
                    core_box, remove_box, forbidden_box,
                    prev_core_box, is_locked_track);

                if (!box_valid) {
                    // [CargoBoxReject] 日志（DEBUG）
                    ROS_DEBUG("[CargoBoxReject] seq=%d track=%d reason=%d action=%s",
                              keyframe_count_ + 1,
                              track.track_id,
                              static_cast<int>(core_box.reject_reason),
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

                                ROS_INFO("[CargoFallbackActive] kf=%d track=%d reason=USE_LAST_GOOD_BOX age=%.2f reject_count=%d",
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
                        const bool payload_static =
                            track.state == TrackState::SUSPENDED_STATIC;
                        float alpha_center = payload_static ? 0.18f : 0.35f;
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

                            if (payload_static &&
                                (swing_radius > 0.80f || dz > 0.30f)) {
                                // 摆动过大，可能是 track 跳变，不跟随
                                ROS_WARN_THROTTLE(1.0,
                                    "[BoxFollowReject] track=%d reason=SWING_TOO_LARGE swing=%.2f dz=%.2f",
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

                        // [BoxFollow] 日志（INFO_THROTTLE）
                        ROS_INFO_THROTTLE(1.0,
                            "[BoxFollow] mode=%s stopped=%d track=%d center_base=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f)",
                            payload_static ? "SWING_FOLLOW" : "MOVING_TRACK",
                            payload_static ? 1 : 0,
                            track.track_id,
                            track.display_center_base.x(),
                            track.display_center_base.y(),
                            track.display_center_base.z(),
                            track.display_size.x(),
                            track.display_size.y(),
                            track.display_size.z());

                        // Commit B: 保存 selected payload track
                        selected_payload_track_id_ = track.track_id;
                        has_selected_payload_track_ = true;
                        selected_payload_stamp_ = stamp;
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

        // Compatibility output is owned by the formal fusion pipeline.

        // P1: 清理过期的 SUSPENDED_STATIC track
        payload_tracker_.cleanupStaleSuspendedStaticTracks(stamp.toSec());
    }

    // ------------------------------------------------------------------------
    // v8: 统一 active remove box 生成（从 CargoBoxV2 循环抽出）
    // ------------------------------------------------------------------------
    std::vector<CargoBox> active_cargo_remove_boxes_base;
    int moving_tracks = 0, static_tracks = 0;
    int current_valid_count = 0, last_good_count = 0, core_fallback_count = 0;
    int skipped_no_box = 0, skipped_no_overlap = 0, skipped_state = 0;
    int overlap_total = 0;
    const bool legacy_removal_authorized =
        run_legacy_cargo && job.lidar_removal_authorized;

    for (const auto& track : payload_tracker_.getTracks()) {
        if (!legacy_removal_authorized) break;
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
        } else {
            if (decision.reason == "NO_BOX") skipped_no_box++;
            else if (decision.reason == "NO_OVERLAP") skipped_no_overlap++;
            else if (decision.reason == "STATE_NOT_ACTIVE") skipped_state++;
        }
    }

    if (!legacy_removal_authorized) {
        active_cargo_remove_boxes_base.clear();
    }

    // v8-stable-r3: [Cargo] 日志（每 2 秒一次，DEBUG）
    if (run_legacy_cargo) {
        ROS_DEBUG_THROTTLE(2.0,
                 "[Cargo] tracks=%zu moving=%d static=%d active=%zu removed=%zu weak_skip=%d fallback=%d",
                 payload_tracker_.getTracks().size(),
                 moving_tracks, static_tracks,
                 active_cargo_remove_boxes_base.size(),
                 cargo_removed_base->size(),
                 skipped_no_box + skipped_no_overlap + skipped_state,
                 last_good_count + core_fallback_count);
    } else {
        ROS_DEBUG_THROTTLE(5.0, "[CargoLegacy] skipped reason=odom_anchor_mode");
    }

    // ------------------------------------------------------------------------
    // 4. CargoCommit：当前帧吊货点删除（必须在 MapCommit 前）
    // ------------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects_after_cargo_base(new pcl::PointCloud<pcl::PointXYZ>);
    const HookLoadMapCommitDecision hook_map_policy =
        evaluateHookLoadMapCommit({
            job.hook_role, job.hook_valid,
            static_cast<HookLoadState>(job.hook_state),
            job.lidar_removal_authorized && job.formal_footprint_valid,
            payload_candidates && !payload_candidates->empty()});
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cargo_commit_source =
        hook_map_policy.exclude_candidate_region
            ? objects_channel_safe : objects_base;

    removePointsInsideCargoRemoveBoxesBase(
        cargo_commit_source,
        active_cargo_remove_boxes_base,
        objects_after_cargo_base,
        cargo_removed_base);

    // ========== Hook locked box 剔除（必须在 MapCommit 前）==========
    if (hook_map_policy.use_formal_remove_box) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr objects_after_hook_base(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr hook_cargo_removed_base(new pcl::PointCloud<pcl::PointXYZ>);
        const CargoObbFootprint& removal_footprint = job.formal_footprint;

        for (const auto& p : objects_after_cargo_base->points) {
            if (containsPointInCargoObbBase(
                    Eigen::Vector3f(p.x, p.y, p.z), removal_footprint,
                    0.10F, 0.10F)) {
                hook_cargo_removed_base->push_back(p);
            } else {
                objects_after_hook_base->push_back(p);
            }
        }

        // 更新 objects_after_cargo_base
        objects_after_cargo_base = objects_after_hook_base;

        // 合并到 cargo_removed_base
        *cargo_removed_base += *hook_cargo_removed_base;

        ROS_DEBUG_THROTTLE(2.0, "[HookCargoRemoval] source=rigid_obb removed_commit=%zu center=(%.2f,%.2f) shape=(%.2f,%.2f,%.2f) yaw=%.1f",
                 hook_cargo_removed_base->size(),
                 removal_footprint.center_base.x(),
                 removal_footprint.center_base.y(),
                 removal_footprint.length_m,
                 removal_footprint.width_m,
                 removal_footprint.max_z - removal_footprint.min_z,
                 removal_footprint.yaw_base_rad * 180.0F /
                     3.14159265358979323846F);
    }

    // [CargoCommit] 日志
    // v8-stable-r3: 降为 DEBUG
    ROS_DEBUG("[CargoCommit] seq=%d source=objects_base before=%zu active_boxes=%zu removed=%zu after=%zu",
             keyframe_count_ + 1,
             cargo_commit_source->size(),
             active_cargo_remove_boxes_base.size(),
             cargo_removed_base->size(),
             objects_after_cargo_base->size());

    // 发布被删除的吊货点
    if (cargo_removed_base && !cargo_removed_base->empty()) {
        sensor_msgs::PointCloud2 removed_msg;
        pcl::toROSMsg(*cargo_removed_base, removed_msg);
        removed_msg.header.stamp = stamp;
        removed_msg.header.frame_id = "base_link";
        cargo_dynamic_removed_pub_.publish(removed_msg);
    }

    // v6: 构建 swept volumes 用于历史反删
    new_cargo_volumes_this_frame_.clear();
    for (const auto& box_base : active_cargo_remove_boxes_base) {
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

        // v8-stable-r3: 降为 DEBUG
        ROS_DEBUG("[CargoHistoryAdd] kf=%d volume=(%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f) fallback=%d history_size=%zu",
                 keyframe_count_ + 1,
                 vol.min_map.x(), vol.min_map.y(), vol.min_map.z(),
                 vol.max_map.x(), vol.max_map.y(), vol.max_map.z(),
                 vol.from_fallback ? 1 : 0,
                 cargo_swept_history_.size());
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

    // [MapCommitInput] 日志（要求的格式）
    // v8-stable-r3: 降为 DEBUG
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
    // ------------------------------------------------------------------------
    if (!loop_closure_detector_.addKeyFrame(
            pose, commit_cloud_base, stamp)) {
        return false;
    }

    keyframe_count_++;

    // [MapCommit] 日志（要求的格式）
    ROS_DEBUG("[MapCommit] seq=%d keyframe=%d commit_total=%zu commit_objects=%zu cargo_removed=%zu human_removed=%zu",
             keyframe_count_.load(std::memory_order_relaxed),
             keyframe_count_.load(std::memory_order_relaxed),
             commit_cloud_base->size(),
             objects_after_human_base->size(),
             cargo_removed_base->size(),
             human_dynamic_base->size());

    // ------------------------------------------------------------------------
    // 8. Map / Tile / Display 更新（只允许使用过滤后的点云）
    // ------------------------------------------------------------------------
    // 保存到 keyframe
    loop_closure_detector_.setLastKeyFrameLayers(
        objects_base, objects_after_human_base, ground_base);
    if (rebuild_running_.load(std::memory_order_acquire)) {
        // The full-map worker checks this under map_mutex_ immediately before
        // its atomic swap, so this keyframe cannot be overwritten.
        rebuild_pending_.store(true, std::memory_order_release);
    }
    if (active_map_rebuild_running_.load(std::memory_order_acquire)) {
        // Active and full rebuilds are serialized but keep independent dirty
        // flags so one scheduler cannot clear the other's retry request.
        active_map_rebuild_dirty_.store(true, std::memory_order_release);
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
        if (objects_added > 0U) {
            advanceObjectsMapContentVersionLocked();
        }
        if (registration_added > 0U || display_added > 0U ||
            ground_added > 0U || objects_added > 0U) {
            advanceMapLayerGenerationLocked();
        }
    }

    // [MapWrite] 日志：debug_map_commit 开启时输出
    if (debug_cfg_.debug_map_commit) {
        ROS_INFO("[MapWrite] seq=%d registration_added=%zu ground_added=%zu objects_added=%zu display_added=%zu",
                 keyframe_count_.load(std::memory_order_relaxed),
                 registration_added,
                 ground_added,
                 objects_added,
                 display_added);
    }

    // v6: DynamicHistoryEraser - 用 swept volume 反删 objects_map/display_map
    if (!new_cargo_volumes_this_frame_.empty()) {
        std::size_t erased_objects = 0U;
        std::size_t erased_display = 0U;
        std::size_t objects_left = 0U;
        std::size_t display_left = 0U;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            erased_objects = eraseDynamicPointsFromCloud(
                objects_map_, new_cargo_volumes_this_frame_);
            erased_display = eraseDynamicPointsFromCloud(
                display_map_, new_cargo_volumes_this_frame_);
            if (erased_objects > 0U) {
                advanceObjectsMapContentVersionLocked();
            }
            if (erased_objects > 0U || erased_display > 0U) {
                advanceMapLayerGenerationLocked();
            }
            objects_left = objects_map_->size();
            display_left = display_map_->size();
        }

        ROS_INFO("[DynamicHistoryEraser] kf=%d new_volumes=%zu erased_objects=%zu erased_display=%zu objects_left=%zu display_left=%zu",
                 keyframe_count_.load(std::memory_order_relaxed),
                 new_cargo_volumes_this_frame_.size(),
                 erased_objects,
                 erased_display,
                 objects_left,
                 display_left);
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
                  keyframe_count_.load(std::memory_order_relaxed),
                  objects_transformed.size(),
                  seen_cells.size(),
                  bev_observation_count_.size());
    }

    // 长期建图：写入 tiles
    if (longterm_mapping_enabled_ && persistent_map_enabled_ &&
        job.allow_persistent_map_commit) {
        Eigen::Vector3d kf_pos = pose.translation();
        int tile_x = std::floor(kf_pos.x() / tile_size_m_);
        int tile_y = std::floor(kf_pos.y() / tile_size_m_);
        std::string tile_key = "x" + std::to_string(tile_x) + "_y" + std::to_string(tile_y);

        std::lock_guard<std::mutex> dirty_lock(dirty_tiles_mutex_);
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

        dirty_tile_count_.store(
            static_cast<int>(dirty_tiles_.size()),
            std::memory_order_release);
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
    // 10. Clean-map maintenance is bounded by a forced owner-thread timeslice.
    // The clean worker publishes only after sealing raw and derived layers
    // into one immutable bundle. A raw commit must not republish an older
    // complete bundle under a new timestamp.
    // ------------------------------------------------------------------------
    requestMapMaintenance();
    last_clean_points_ = objects_clean_map_ ? objects_clean_map_->size() : 0;

    // 闭环检测
    if (loop_closure_enabled_ &&
        keyframe_count_ % loop_detection_interval_ == 0) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        loop_closure_pending_ = true;
        map_maintenance_pending_ = true;
        queue_cv_.notify_one();
    }

    // [PipelineSummary] 单行摘要（INFO，关键验收点）
    {
        int moving = 0, statik = 0, pend = 0;
        for (const auto& t : payload_tracker_.getTracks()) {
            if (t.state == TrackState::SUSPENDED_MOVING || t.state == TrackState::DYNAMIC_PAYLOAD) moving++;
            else if (t.state == TrackState::SUSPENDED_STATIC) statik++;
            else if (t.state == TrackState::PENDING_STATIC) pend++;
        }

        if (debug_cfg_.debug_map_commit) {
            ROS_INFO("[PipelineSummary] "
                     "frame=%lu kf=%d stamp=%.3f "
                     "raw=%zu ground=%zu raw_obj=%zu candidates=%zu "
                     "tracks=%zu moving=%d static=%d pending=%d "
                     "active_boxes=%zu cargo_removed=%zu human_removed=%zu "
                     "commit_obj=%zu clean_points=%zu",
                     frame_seq_,
                     keyframe_count_.load(std::memory_order_relaxed),
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
    return true;
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

    // 检查 overlap
    d.overlap = countPointsInsideBoxBase(objects_base, candidate);
    if (d.overlap < 5) {  // min_remove_overlap_points
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
// Cargo Warning 函数
// ============================================================================

NdtSlamNode::CargoWarningData NdtSlamNode::computeCargoWarning(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_base,
    const Eigen::Vector3f& cargo_center,
    const Eigen::Vector3f& cargo_size,
    float cargo_bottom_z,
    float cargo_bottom_uncertainty,
    const ros::Time& stamp) {

    CargoWarningData warning;
    warning.valid = false;
    warning.level = 0;
    warning.alarm_code = 0;
    warning.source = "tight_box";
    warning.reason = "no_obstacle";

    if (!odom_anchor_config_.cargo_warning.enabled) {
        return warning;
    }

    if (!cloud_base || cloud_base->empty()) {
        return warning;
    }

    const auto& config = odom_anchor_config_.cargo_warning;

    // 计算货物底部安全高度
    float cargo_bottom_safe = cargo_bottom_z;
    if (config.cargo_bottom_use_uncertainty) {
        cargo_bottom_safe -= cargo_bottom_uncertainty;
    }
    cargo_bottom_safe -= config.cargo_bottom_extra_margin_m;

    warning.cargo_bottom_z = cargo_bottom_z;
    warning.cargo_bottom_safe_z = cargo_bottom_safe;
    warning.cargo_top_z = cargo_bottom_z + cargo_size.z();
    warning.cargo_bottom_uncertainty = cargo_bottom_uncertainty;
    warning.cargo_center = cargo_center;
    warning.cargo_size = cargo_size;

    // 提取障碍物点（排除货物自身、地面和高处结构）
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    float half_x = cargo_size.x() * 0.5f;
    float half_y = cargo_size.y() * 0.5f;
    float margin_xy = config.self_cargo_margin_xy_m;
    float margin_z = config.self_cargo_margin_z_m;
    float cargo_top_z = cargo_bottom_z + cargo_size.z();

    // 估算地面高度
    float ground_z = cloud_base->points[0].z;
    for (const auto& p : cloud_base->points) {
        if (p.z < ground_z) ground_z = p.z;
    }

    size_t before_count = cloud_base->size();
    size_t ground_filtered = 0;
    size_t self_filtered = 0;
    size_t high_filtered = 0;

    for (const auto& p : cloud_base->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        // 排除地面点
        if (config.exclude_ground) {
            float hag = p.z - ground_z;
            if (hag < config.ground_hag_min_m) {
                ground_filtered++;
                continue;
            }
        }

        // 排除货物自身点（tight box 外扩 margin）
        if (config.exclude_self_cargo) {
            float dx = std::abs(p.x - cargo_center.x()) - (half_x + margin_xy);
            float dy = std::abs(p.y - cargo_center.y()) - (half_y + margin_xy);
            float dz_low = cargo_bottom_z - margin_z;
            float dz_high = cargo_top_z + margin_z;

            if (dx < 0 && dy < 0 && p.z >= dz_low && p.z <= dz_high) {
                self_filtered++;
                continue;  // 在货物区域内
            }
        }

        // 排除高处吊具/绳索/上方结构
        if (p.z > cargo_top_z + 0.30f) {
            high_filtered++;
            continue;
        }

        obstacle_cloud->push_back(p);
    }

    if (obstacle_cloud->size() < static_cast<size_t>(config.obstacle_min_points)) {
        warning.reason = "few_obstacles";
        warning.valid = true;
        return warning;
    }

    // 聚类障碍物点
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(obstacle_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(config.obstacle_cluster_tolerance_m);
    ec.setMinClusterSize(config.obstacle_min_points);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(obstacle_cloud);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
        warning.reason = "no_obstacle_clusters";
        warning.valid = true;
        return warning;
    }

    // 找到最近的危险障碍物
    float min_distance = std::numeric_limits<float>::max();
    float max_obstacle_top_z = -std::numeric_limits<float>::max();
    Eigen::Vector3f nearest_point = Eigen::Vector3f::Zero();
    uint32_t total_obstacle_points = 0;

    for (const auto& cluster : cluster_indices) {
        // 计算簇的 z95
        std::vector<float> cluster_z;
        Eigen::Vector3f cluster_center = Eigen::Vector3f::Zero();
        for (int idx : cluster.indices) {
            const auto& p = obstacle_cloud->points[idx];
            cluster_z.push_back(p.z);
            cluster_center += p.getVector3fMap();
        }
        cluster_center /= static_cast<float>(cluster.indices.size());

        std::sort(cluster_z.begin(), cluster_z.end());
        float z95 = cluster_z[static_cast<int>(cluster_z.size() * config.obstacle_top_percentile)];

        // 计算到货物 footprint 边界的距离
        float dx = std::max(std::abs(cluster_center.x() - cargo_center.x()) - half_x, 0.0f);
        float dy = std::max(std::abs(cluster_center.y() - cargo_center.y()) - half_y, 0.0f);
        float distance = std::sqrt(dx * dx + dy * dy);

        // 检查是否在预警范围内
        if (distance <= config.level2_distance_m) {
            total_obstacle_points += cluster.indices.size();

            if (distance < min_distance) {
                min_distance = distance;
                nearest_point = cluster_center;
            }

            if (z95 > max_obstacle_top_z) {
                max_obstacle_top_z = z95;
            }
        }
    }

    if (total_obstacle_points == 0) {
        warning.reason = "no_nearby_obstacles";
        warning.valid = true;
        return warning;
    }

    // 计算净空
    float clearance = cargo_bottom_safe - max_obstacle_top_z;

    warning.distance_to_footprint_m = min_distance;
    warning.clearance_m = clearance;
    warning.obstacle_top_z = max_obstacle_top_z;
    warning.obstacle_point_count = total_obstacle_points;
    warning.obstacle_nearest_point = nearest_point;
    warning.valid = true;

    // 判断报警等级
    if (min_distance <= config.level1_distance_m && clearance < config.min_vertical_clearance_m) {
        warning.level = 1;
        warning.alarm_code = config.level1_alarm_code;
        warning.reason = "level1_clearance_lt_0.80";
    } else if (min_distance <= config.level2_distance_m && clearance < config.min_vertical_clearance_m) {
        warning.level = 2;
        warning.alarm_code = config.level2_alarm_code;
        warning.reason = "level2_clearance_lt_0.80";
    } else {
        warning.level = 0;
        warning.alarm_code = config.clear_alarm_code;
        warning.reason = "clear";
    }

    // CargoWarningDebug 日志：debug_cargo_warning 开启时输出
    if (debug_cfg_.debug_cargo_warning) {
        ROS_INFO_THROTTLE(debug_cfg_.summary_interval_sec, "[CargoWarningDebug] before=%zu ground_filtered=%zu self_filtered=%zu high_filtered=%zu candidate=%zu clusters=%zu selected_dist=%.2f selected_top=%.2f clearance=%.2f level=%d",
                         before_count, ground_filtered, self_filtered, high_filtered,
                         obstacle_cloud->size(), cluster_indices.size(),
                         min_distance, max_obstacle_top_z, clearance, warning.level);
    }

    return warning;
}

void NdtSlamNode::publishCargoWarning(const CargoWarningData& warning, const ros::Time& stamp) {
    if (!odom_anchor_config_.cargo_warning.enabled) return;

    // Debounce 逻辑
    if (warning.level > 0) {
        cargo_warning_debounce_count_++;
        if (cargo_warning_debounce_count_ < odom_anchor_config_.cargo_warning.debounce_frames) {
            return;
        }
    } else {
        // 检查 clear_hold_sec
        if (last_cargo_warning_.level > 0 &&
            (stamp - last_cargo_warning_stamp_).toSec() < odom_anchor_config_.cargo_warning.clear_hold_sec) {
            return;
        }
        cargo_warning_debounce_count_ = 0;
    }

    last_cargo_warning_ = warning;
    last_cargo_warning_stamp_ = stamp;

    // 只有 publish_alarm_msg=true 时才发布正式报警消息
    if (odom_anchor_config_.cargo_warning.publish_alarm_msg) {
        std_msgs::String msg;
        std::ostringstream oss;
        oss << "{"
            << "\"valid\":" << (warning.valid ? "true" : "false") << ","
            << "\"level\":" << static_cast<int>(warning.level) << ","
            << "\"alarm_code\":" << warning.alarm_code << ","
            << "\"distance_to_footprint_m\":" << std::fixed << std::setprecision(2) << warning.distance_to_footprint_m << ","
            << "\"clearance_m\":" << std::fixed << std::setprecision(2) << warning.clearance_m << ","
            << "\"cargo_bottom_z\":" << std::fixed << std::setprecision(2) << warning.cargo_bottom_z << ","
            << "\"cargo_bottom_safe_z\":" << std::fixed << std::setprecision(2) << warning.cargo_bottom_safe_z << ","
            << "\"cargo_top_z\":" << std::fixed << std::setprecision(2) << warning.cargo_top_z << ","
            << "\"obstacle_top_z\":" << std::fixed << std::setprecision(2) << warning.obstacle_top_z << ","
            << "\"obstacle_point_count\":" << warning.obstacle_point_count << ","
            << "\"source\":\"" << warning.source << "\","
            << "\"reason\":\"" << warning.reason << "\""
            << "}";
        msg.data = oss.str();
        cargo_warning_pub_.publish(msg);
    }

    // 日志（始终输出，用于调试）
    if (debug_cfg_.debug_cargo_warning && warning.level > 0) {
        ROS_DEBUG_THROTTLE(10.0, "[CargoWarning] level=%d alarm=%d dist=%.2f clearance=%.2f cargo_bottom=%.2f obstacle_top=%.2f reason=%s",
                          warning.level, warning.alarm_code,
                          warning.distance_to_footprint_m, warning.clearance_m,
                          warning.cargo_bottom_z, warning.obstacle_top_z,
                          warning.reason.c_str());
    } else if (debug_cfg_.debug_cargo_warning) {
        ROS_DEBUG_THROTTLE(10.0, "[CargoWarning] level=0 reason=%s", warning.reason.c_str());
    }
}

void NdtSlamNode::publishCargoWarningMarkers(
    const Eigen::Vector3f& cargo_center,
    const Eigen::Vector3f& cargo_size,
    const CargoWarningData& warning,
    const ros::Time& stamp) {

    if (!odom_anchor_config_.cargo_warning.enabled) return;
    if (!odom_anchor_config_.cargo_warning.publish_debug_marker) return;

    const auto& config = odom_anchor_config_.cargo_warning;
    Eigen::Vector3f marker_center = cargo_center;
    Eigen::Vector3f marker_size = cargo_size;
    double marker_yaw_map_rad = 0.0;
    const RigidCargoGeometry rigid_geometry =
        buildCurrentRigidCargoGeometryForPose(current_pose_, stamp);
    if (rigid_geometry.valid) {
        marker_center.setZero();
        for (const Eigen::Vector3f& corner : rigid_geometry.corners_map) {
            marker_center += corner;
        }
        marker_center /= 8.0F;
        marker_size = Eigen::Vector3f(
            rigid_geometry.shape.length_m,
            rigid_geometry.shape.width_m,
            rigid_geometry.shape.height_m);
        const Eigen::Vector3f long_axis_map =
            rigid_geometry.corners_map[1] - rigid_geometry.corners_map[0];
        marker_yaw_map_rad = std::atan2(
            static_cast<double>(long_axis_map.y()),
            static_cast<double>(long_axis_map.x()));
    }
    const double marker_half_yaw = 0.5 * marker_yaw_map_rad;
    const double marker_orientation_z = std::sin(marker_half_yaw);
    const double marker_orientation_w = std::cos(marker_half_yaw);

    // 绿色 tight box marker
    visualization_msgs::Marker tight_box_marker;
    tight_box_marker.header.frame_id = map_frame_;
    tight_box_marker.header.stamp = stamp;
    tight_box_marker.ns = "cargo_tight_box";
    tight_box_marker.id = 0;
    tight_box_marker.type = visualization_msgs::Marker::CUBE;
    tight_box_marker.action = visualization_msgs::Marker::ADD;
    tight_box_marker.pose.position.x = marker_center.x();
    tight_box_marker.pose.position.y = marker_center.y();
    tight_box_marker.pose.position.z = marker_center.z();
    tight_box_marker.pose.orientation.z = marker_orientation_z;
    tight_box_marker.pose.orientation.w = marker_orientation_w;
    tight_box_marker.scale.x = marker_size.x();
    tight_box_marker.scale.y = marker_size.y();
    tight_box_marker.scale.z = marker_size.z();
    tight_box_marker.color.r = 0.0;
    tight_box_marker.color.g = 1.0;
    tight_box_marker.color.b = 0.0;
    tight_box_marker.color.a = 0.3;
    tight_box_marker.lifetime = ros::Duration(0.5);
    cargo_tight_box_marker_pub_.publish(tight_box_marker);

    // 黄色/红色预警范围 marker（只画水平 footprint）
    visualization_msgs::MarkerArray zone_markers;

    // 黄色 5m footprint
    visualization_msgs::Marker yellow_marker;
    yellow_marker.header.frame_id = map_frame_;
    yellow_marker.header.stamp = stamp;
    yellow_marker.ns = "cargo_warning_zone";
    yellow_marker.id = 1;
    yellow_marker.type = visualization_msgs::Marker::CUBE;
    yellow_marker.action = visualization_msgs::Marker::ADD;
    yellow_marker.pose.position.x = marker_center.x();
    yellow_marker.pose.position.y = marker_center.y();
    yellow_marker.pose.position.z = marker_center.z() - marker_size.z() * 0.5f + 0.01;
    yellow_marker.pose.orientation.z = marker_orientation_z;
    yellow_marker.pose.orientation.w = marker_orientation_w;
    yellow_marker.scale.x = marker_size.x() + 2.0f * config.level2_distance_m;
    yellow_marker.scale.y = marker_size.y() + 2.0f * config.level2_distance_m;
    yellow_marker.scale.z = 0.02;
    yellow_marker.color.r = 1.0;
    yellow_marker.color.g = 1.0;
    yellow_marker.color.b = 0.0;
    yellow_marker.color.a = 0.15;
    yellow_marker.lifetime = ros::Duration(0.5);
    zone_markers.markers.push_back(yellow_marker);

    // 红色 3m footprint
    visualization_msgs::Marker red_marker;
    red_marker.header.frame_id = map_frame_;
    red_marker.header.stamp = stamp;
    red_marker.ns = "cargo_warning_zone";
    red_marker.id = 2;
    red_marker.type = visualization_msgs::Marker::CUBE;
    red_marker.action = visualization_msgs::Marker::ADD;
    red_marker.pose.position.x = marker_center.x();
    red_marker.pose.position.y = marker_center.y();
    red_marker.pose.position.z = marker_center.z() - marker_size.z() * 0.5f + 0.02;
    red_marker.pose.orientation.z = marker_orientation_z;
    red_marker.pose.orientation.w = marker_orientation_w;
    red_marker.scale.x = marker_size.x() + 2.0f * config.level1_distance_m;
    red_marker.scale.y = marker_size.y() + 2.0f * config.level1_distance_m;
    red_marker.scale.z = 0.02;
    red_marker.color.r = 1.0;
    red_marker.color.g = 0.0;
    red_marker.color.b = 0.0;
    red_marker.color.a = 0.2;
    red_marker.lifetime = ros::Duration(0.5);
    zone_markers.markers.push_back(red_marker);

    cargo_warning_zone_marker_pub_.publish(zone_markers);

    // 白色最近危险障碍物点 marker
    if (warning.level > 0) {
        visualization_msgs::Marker obstacle_marker;
        obstacle_marker.header.frame_id = "map";
        obstacle_marker.header.stamp = stamp;
        obstacle_marker.ns = "cargo_warning_obstacle";
        obstacle_marker.id = 0;
        obstacle_marker.type = visualization_msgs::Marker::SPHERE;
        obstacle_marker.action = visualization_msgs::Marker::ADD;
        obstacle_marker.pose.position.x = warning.obstacle_nearest_point.x();
        obstacle_marker.pose.position.y = warning.obstacle_nearest_point.y();
        obstacle_marker.pose.position.z = warning.obstacle_nearest_point.z();
        obstacle_marker.pose.orientation.w = 1.0;
        obstacle_marker.scale.x = 0.3;
        obstacle_marker.scale.y = 0.3;
        obstacle_marker.scale.z = 0.3;
        obstacle_marker.color.r = 1.0;
        obstacle_marker.color.g = 1.0;
        obstacle_marker.color.b = 1.0;
        obstacle_marker.color.a = 0.8;
        obstacle_marker.lifetime = ros::Duration(0.5);
        cargo_warning_obstacle_marker_pub_.publish(obstacle_marker);
    }
}

// ========== Runtime Diagnostics 辅助函数 ==========

void NdtSlamNode::logStartupConfig() {
    if (!runtime_diag_.isEnabled()) return;

    std::map<std::string, std::string> params;
    params["git_sha"] = build_info::kGitSha;
    params["config_path"] = diag_output_dir_;
    params["mapping_mode"] = "longterm_mapping";
    params["cloud_topic"] = pointcloud_topic_;
    params["subscriber_queue"] = "10";
    params["processing_queue_capacity"] =
        std::to_string(localization_queue_capacity_);
    params["ndt_resolution"] = std::to_string(ndt_resolution_);
    params["ndt_step_size"] = std::to_string(ndt_step_size_);
    params["ndt_epsilon"] = std::to_string(ndt_transformation_epsilon_);
    params["ndt_max_iterations"] = std::to_string(ndt_max_iterations_);
    params["reject_high_fitness"] = "false";
    params["fitness_threshold"] = "2.00";
    params["map_commit_max_fitness"] = std::to_string(map_commit_max_fitness_);
    params["max_speed_mps"] = "2.00";
    params["max_step_safety_factor"] = "1.10";
    params["max_step_min_m"] = "0.05";
    params["max_step_max_m"] = "0.25";
    params["innovation_gate_m"] = "0.35";
    params["innovation_reject_m"] = "1.00";
    params["target_min_points"] = std::to_string(localization_target_min_points_);
    params["merger_max_pair_dt_sec"] = "0.060";
    params["merger_stale_timeout_sec"] = "0.120";
    params["formal_safety_status_enabled"] = "1";
    params["alarm_publisher_owner"] = "cargo_alarm_heartbeat_node";
    runtime_diag_.logRunConfig(params);
}

void NdtSlamNode::logBuildId() {
    if (!runtime_diag_.isEnabled()) return;

    std::map<std::string, std::string> params;
    params["git_sha"] = build_info::kGitSha;
    params["git_branch"] = build_info::kGitBranch;
    params["build_time"] = __DATE__ " " __TIME__;
    params["source_root"] = build_info::kSourceRoot;
    params["node_name"] = ros::this_node::getName();
    runtime_diag_.logBuildId(params);
}

void NdtSlamNode::logNdtHealthPeriodic() {
    if (!runtime_diag_.isEnabled()) return;

    const PipelineRateSnapshot rate = runtime_diag_.pipelineRateSnapshot(
        queue_overwrite_drop_count_.load(std::memory_order_relaxed));
    const std::uint64_t attempts =
        ndt_attempt_count_.load(std::memory_order_relaxed);
    const double converged_ratio = attempts > 0U
        ? static_cast<double>(
              ndt_converged_count_.load(std::memory_order_relaxed)) /
              static_cast<double>(attempts)
        : 0.0;
    double output_step = 0.0;
    double allowed_step = 0.0;
    if (crane_motion_ekf_enabled_ && crane_motion_ekf_.initialized()) {
        output_step = crane_motion_ekf_.status().output_step;
        allowed_step = crane_motion_ekf_.status().max_allowed_step;
    }

    runtime_diag_.logNdtHealth(
        diag_frame_index_, diag_last_cloud_stamp_,
        rate.callback_hz, rate.processed_hz,
        rate.callback_sensor_dt_p50_ms,
        last_total_ms_, last_ndt_time_ms_,
        last_actual_target_source_, last_target_points_,
        converged_ratio, last_ndt_fitness_,
        runtime_diag_.predictionOnlyCount(), diag_consecutive_prediction_only_,
        last_raw_step_, output_step, allowed_step);

    // 重置计数
    diag_processed_frame_count_ = 0;
    diag_converged_count_ = 0;
}

void NdtSlamNode::logCargoHealthPeriodic() {
    if (!runtime_diag_.isEnabled()) return;

    CargoFrameRecord rec;
    rec.stamp = !last_cargo_pipeline_stamp_.isZero()
        ? last_cargo_pipeline_stamp_.toSec()
        : cargo_state_.stamp.toSec();
    rec.track_id = static_cast<int>(std::min<std::uint64_t>(
        cargo_fusion_track_id_,
        static_cast<std::uint64_t>(std::numeric_limits<int>::max())));
    rec.center_x = cargo_state_.center_base.x();
    rec.center_y = cargo_state_.center_base.y();
    rec.center_z = cargo_state_.center_base.z();
    rec.measured_center_x = hook_lock_.live_pose_measured_base.x();
    rec.measured_center_y = hook_lock_.live_pose_measured_base.y();
    rec.measured_center_z = hook_lock_.live_pose_measured_base.z();
    rec.predicted_center_x = hook_lock_.live_pose_predicted_base.x();
    rec.predicted_center_y = hook_lock_.live_pose_predicted_base.y();
    rec.predicted_center_z = hook_lock_.live_pose_predicted_base.z();
    rec.center_residual_x = hook_lock_.live_pose_residual_base.x();
    rec.center_residual_y = hook_lock_.live_pose_residual_base.y();
    rec.center_residual_z = hook_lock_.live_pose_residual_base.z();
    rec.pose_sensor_dt_sec = hook_lock_.live_pose_dt_sec;
    rec.position_source = cargoPoseSourceName(
        current_rigid_cargo_geometry_.valid
            ? current_rigid_cargo_geometry_.pose.source
            : hook_lock_.live_pose.source);
    rec.vertical_position_source = cargoVerticalPoseSourceName(
        hook_lock_.live_pose.vertical_source);
    rec.observed_top_z = hook_fixed_cargo_.valid &&
            std::isfinite(hook_fixed_cargo_.z95)
        ? hook_fixed_cargo_.z95
        : std::numeric_limits<double>::quiet_NaN();
    rec.frozen_thickness_m = hook_lock_.locked_shape.valid
        ? hook_lock_.locked_shape.height_m : 0.0;
    const double evaluation_stamp = rec.stamp;
    rec.pose_evidence_age_sec =
        hook_lock_.live_pose.evidence_stamp_sec > 0.0
            ? std::max(0.0, evaluation_stamp -
                  hook_lock_.live_pose.evidence_stamp_sec)
            : 0.0;
    rec.height_evidence_age_sec =
        last_cargo_bottom_result_.evidence_stamp_sec > 0.0
            ? std::max(0.0, evaluation_stamp -
                  last_cargo_bottom_result_.evidence_stamp_sec)
            : 0.0;
    rec.size_x = cargo_state_.size.x();
    rec.size_y = cargo_state_.size.y();
    rec.size_z = cargo_state_.size.z();
    rec.footprint_yaw_deg = hook_lock_.locked_shape.valid
        ? hook_lock_.locked_shape.yaw_base_rad * 180.0 /
            3.14159265358979323846
        : 0.0;
    rec.raw_bottom_z = cargo_state_.bottom_z;
    rec.filtered_bottom_z = cargo_state_.bottom_z;
    rec.stable_bottom_z = cargo_state_.bottom_safe_z;
    rec.top_z = cargo_state_.top_z;
    rec.height_m = cargo_state_.top_z - cargo_state_.bottom_z;
    rec.bottom_valid = cargo_state_.valid_height;
    rec.height_valid = cargo_state_.valid_height;
    rec.observation_valid = hook_observation_associated_current_;
    rec.cluster_points = hook_fixed_cargo_.core_points_base
        ? static_cast<int>(std::min<std::size_t>(
            hook_fixed_cargo_.core_points_base->size(),
            static_cast<std::size_t>(std::numeric_limits<int>::max())))
        : 0;
    rec.candidate_count = static_cast<int>(std::min<std::size_t>(
        hook_fixed_cargo_.candidate_count,
        static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.selected_candidate_id =
        hook_fixed_cargo_.selected_candidate_id;
    rec.identity_score = hook_fixed_cargo_.identity_confidence;
    rec.orientation_confidence =
        hook_fixed_cargo_.orientation_confidence;
    rec.shape_confidence =
        hook_lock_.provisional_observations.empty()
            ? hook_fixed_cargo_.shape_confidence
            : hook_lock_.provisional_summary.shape_confidence;
    rec.motion_confidence =
        hook_lock_.provisional_observations.empty()
            ? hook_fixed_cargo_.motion_confidence
            : hook_lock_.provisional_summary.motion_confidence;
    rec.overall_lock_confidence =
        hook_lock_.provisional_observations.empty()
            ? hook_fixed_cargo_.overall_lock_confidence
            : hook_lock_.provisional_summary.overall_lock_confidence;
    rec.obstacle_roi_finite_points = static_cast<int>(
        std::min<std::size_t>(
            cargo_obstacle_roi_finite_points_,
            static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.obstacle_roi_coverage_ratio =
        cargo_obstacle_roi_coverage_ratio_;
    rec.self_removed_points = static_cast<int>(std::min<std::size_t>(
        cargo_self_removed_points_,
        static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.identity_self_removed_points = static_cast<int>(
        std::min<std::size_t>(
            cargo_identity_self_removed_points_,
            static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.rigging_self_removed_points = static_cast<int>(
        std::min<std::size_t>(
            cargo_rigging_self_removed_points_,
            static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.external_obstacle_points = static_cast<int>(std::min<std::size_t>(
        cargo_external_obstacle_points_,
        static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.self_margin_xy_m = cargo_self_margin_xy_m_;
    rec.self_margin_z_m = cargo_self_margin_z_m_;
    rec.horizontal_uncertainty_m = cargo_horizontal_uncertainty_m_;
    rec.vertical_uncertainty_m = cargo_vertical_uncertainty_m_;
    rec.ground_reference_valid =
        hook_fixed_cargo_.ground_reference_valid;
    rec.ground_z = hook_fixed_cargo_.ground_z;
    rec.dangerous_cluster_points = static_cast<int>(
        std::min<std::size_t>(
            cargo_dangerous_cluster_points_,
            static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.nearest_obstacle_x = cargo_nearest_obstacle_point_.x();
    rec.nearest_obstacle_y = cargo_nearest_obstacle_point_.y();
    rec.nearest_obstacle_z = cargo_nearest_obstacle_point_.z();
    rec.nearest_cluster_center_x = cargo_nearest_cluster_center_.x();
    rec.nearest_cluster_center_y = cargo_nearest_cluster_center_.y();
    rec.nearest_cluster_center_z = cargo_nearest_cluster_center_.z();
    rec.nearest_cluster_distance = cargo_nearest_cluster_distance_m_;
    rec.obstacle_top_z95_m = cargo_obstacle_top_z95_m_;
    rec.obstacle_uncertainty_m = cargo_obstacle_uncertainty_m_;
    rec.conservative_clearance_m = cargo_conservative_clearance_m_;
    rec.requested_alarm_code = cargo_last_requested_code_;
    rec.raw_warning_code = cargo_raw_warning_code_;
    rec.confirmed_warning_code = cargo_confirmed_warning_code_;
    rec.temporal_candidate_code = cargo_temporal_candidate_code_;
    rec.temporal_candidate_count = cargo_temporal_candidate_count_;
    // Previous-confirmation holding is deliberately disabled. Keep this
    // explicit field so Ubuntu CSV review can verify it remains zero.
    rec.temporal_hold_age_sec = 0.0;
    rec.used_previous_confirmation = cargo_used_previous_confirmation_;
    rec.obstacle_track_id = cargo_obstacle_track_id_;
    rec.obstacle_track_age_sec = cargo_obstacle_track_age_sec_;
    rec.obstacle_track_confirm_count =
        cargo_obstacle_track_confirm_count_;
    rec.obstacle_track_static = cargo_obstacle_track_static_;
    rec.obstacle_track_velocity_x =
        cargo_obstacle_track_velocity_map_.x();
    rec.obstacle_track_velocity_y =
        cargo_obstacle_track_velocity_map_.y();
    rec.obstacle_track_velocity_z =
        cargo_obstacle_track_velocity_map_.z();
    rec.safety_spatial_mode = cargo_safety_spatial_mode_;
    rec.cargo_map_speed_mps = cargo_velocity_map_.norm();
    rec.corridor_eligible_clusters = static_cast<int>(
        std::min<std::size_t>(
            cargo_corridor_eligible_clusters_,
            static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.corridor_rejected_clusters = static_cast<int>(
        std::min<std::size_t>(
            cargo_corridor_rejected_clusters_,
            static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.residual_self_clusters = static_cast<int>(
        std::min<std::size_t>(
            cargo_residual_self_clusters_,
            static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.residual_unknown_clusters = static_cast<int>(
        std::min<std::size_t>(
            cargo_residual_unknown_clusters_,
            static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.safety_reason = cargo_last_safety_reason_;
    rec.support_points = static_cast<int>(std::min<std::size_t>(
        last_cargo_bottom_result_.selected_stats.bottom_band_points,
        static_cast<std::size_t>(std::numeric_limits<int>::max())));
    rec.filter_accepted = last_cargo_bottom_result_.valid;
    rec.filter_reason = last_cargo_bottom_result_.reason;
    rec.lost_frames = hook_lock_.lost_count;

    switch (cargo_state_.state) {
        case CargoState::EMPTY: rec.track_state = "EMPTY"; break;
        case CargoState::CANDIDATE: rec.track_state = "CANDIDATE"; break;
        case CargoState::LOCKED: rec.track_state = "LOCKED"; break;
        case CargoState::LOST: rec.track_state = "LOST"; break;
    }

    switch (hook_lock_.state) {
        case HookCargoLockState::EMPTY: rec.lock_state = "EMPTY"; break;
        case HookCargoLockState::CANDIDATE:
            rec.lock_state = "CANDIDATE";
            break;
        case HookCargoLockState::GEOMETRY_CONFIRMING:
            rec.lock_state = "GEOMETRY_CONFIRMING";
            break;
        case HookCargoLockState::LOCKED: rec.lock_state = "LOCKED"; break;
        case HookCargoLockState::LOST_HOLD:
            rec.lock_state = "LOST_HOLD";
            break;
        case HookCargoLockState::CLEAR_WAIT_REARM:
            rec.lock_state = "CLEAR_WAIT_REARM";
            break;
    }

    runtime_diag_.logCargoHealth(rec);
    runtime_diag_.writeCargoFrame(rec);
}

} // namespace ndt_slam
