#include "lidar_slam2/OdometryNode.hpp"

#include <chrono>
#include <memory>
#include <sophus/se3.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

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

#include "kiss_icp/pipeline/KissICP.hpp"

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
            ROS_WARN("%s", ex.what());
        }
    }
    ROS_WARN("Failed to find tf. Reason=%s", err_msg.c_str());
    return Sophus::SE3d();
}
}  // namespace

namespace lidar_slam2 {
using utils::PointCloud2ToEigen;
using utils::GetTimestamps;

OdometryNode::OdometryNode(const ros::NodeHandle& nh)
    : nh_(nh) {
    initializeParameters();

    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 10, &OdometryNode::pointCloudCallback, this);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug_cloud", 10);
    mapping_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(mapping_pointcloud_topic_, 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    reset_srv_ = nh_.advertiseService("reset", &OdometryNode::resetService, this);
    set_pose_srv_ = nh_.advertiseService("set_pose", &OdometryNode::setPoseService, this);

    relocalize_client_ = nh_.serviceClient<std_srvs::Empty>(relocalize_service_);

    relocalization_sub_ = nh_.subscribe(relocalization_topic_, 10, &OdometryNode::relocalizationCallback, this);

    current_pose_ = Sophus::SE3d();

    kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(kiss_icp_config_);

    shutdown_ = false;
    process_thread_ = std::thread(&OdometryNode::processCloudThread, this);

    ROS_INFO("OdometryNode initialized with KISS-ICP");
    ROS_INFO("Set pose service initialized: ~/set_pose");
    ROS_INFO("Relocalization client initialized: %s", relocalize_service_.c_str());
    ROS_INFO("Relocalization subscriber initialized: %s", relocalization_topic_.c_str());
    ROS_INFO("Processing thread started");
}

OdometryNode::OdometryNode(const std::string& config_file_path, const ros::NodeHandle& nh)
    : nh_(nh) {
    initializeParameters(config_file_path);

    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 10, &OdometryNode::pointCloudCallback, this);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    debug_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug_cloud", 10);
    mapping_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(mapping_pointcloud_topic_, 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    reset_srv_ = nh_.advertiseService("reset", &OdometryNode::resetService, this);
    set_pose_srv_ = nh_.advertiseService("set_pose", &OdometryNode::setPoseService, this);

    relocalize_client_ = nh_.serviceClient<std_srvs::Empty>(relocalize_service_);

    relocalization_sub_ = nh_.subscribe(relocalization_topic_, 10, &OdometryNode::relocalizationCallback, this);

    current_pose_ = Sophus::SE3d();

    kiss_icp_ = std::make_unique<kiss_icp::pipeline::KissICP>(kiss_icp_config_);

    shutdown_ = false;
    process_thread_ = std::thread(&OdometryNode::processCloudThread, this);

    ROS_INFO("OdometryNode initialized with KISS-ICP (using config file: %s)", config_file_path.c_str());
    ROS_INFO("Set pose service initialized: ~/set_pose");
    ROS_INFO("Relocalization client initialized: %s", relocalize_service_.c_str());
    ROS_INFO("Relocalization subscriber initialized: %s", relocalization_topic_.c_str());
    ROS_INFO("Processing thread started");
}

OdometryNode::~OdometryNode() {
    shutdown_ = true;
    queue_cv_.notify_one();
    if (process_thread_.joinable()) {
        process_thread_.join();
    }
    ROS_INFO("Processing thread stopped");
}

void OdometryNode::initializeParameters(const std::string& config_file_path) {
    ROS_INFO("=== Initializing parameters (from %s) ===", config_file_path.c_str());

    try {
        YAML::Node config = YAML::LoadFile(config_file_path);

        if (config["pointcloud_topic"]) {
            pointcloud_topic_ = config["pointcloud_topic"].as<std::string>();
        }
        if (config["mapping_pointcloud_topic"]) {
            mapping_pointcloud_topic_ = config["mapping_pointcloud_topic"].as<std::string>();
        }
        if (config["odom_topic"]) {
            odom_topic_ = config["odom_topic"].as<std::string>();
        }
        if (config["odom_frame"]) {
            odom_frame_ = config["odom_frame"].as<std::string>();
        }
        if (config["base_frame"]) {
            base_frame_ = config["base_frame"].as<std::string>();
        }
        if (config["map_frame"]) {
            lidar_odom_frame_ = config["map_frame"].as<std::string>();
        }

        if (config["use_sim_time"]) {
            use_sim_time_ = config["use_sim_time"].as<bool>();
        }
        if (config["publish_odom_tf"]) {
            publish_odom_tf_ = config["publish_odom_tf"].as<bool>();
        }
        if (config["publish_debug_clouds"]) {
            publish_debug_clouds_ = config["publish_debug_clouds"].as<bool>();
        }
        if (config["position_covariance"]) {
            position_covariance_ = config["position_covariance"].as<double>();
        }
        if (config["orientation_covariance"]) {
            orientation_covariance_ = config["orientation_covariance"].as<double>();
        }

        if (config["odom_min_neighbors"]) {
            min_neighbors_ = config["odom_min_neighbors"].as<int>();
        }
        if (config["odom_neighbor_search_radius"]) {
            neighbor_search_radius_ = config["odom_neighbor_search_radius"].as<double>();
        }

        if (config["tracking"]) {
            if (config["tracking"]["inlier_ratio_threshold"]) {
                inlier_ratio_threshold_ = config["tracking"]["inlier_ratio_threshold"].as<double>();
            }
            if (config["tracking"]["mean_distance_threshold"]) {
                mean_distance_threshold_ = config["tracking"]["mean_distance_threshold"].as<double>();
            }
            if (config["tracking"]["model_deviation_threshold"]) {
                model_deviation_threshold_ = config["tracking"]["model_deviation_threshold"].as<double>();
            }
        }

        if (config["data"]) {
            if (config["data"]["deskew"]) {
                kiss_icp_config_.deskew = config["data"]["deskew"].as<bool>();
            }
            if (config["data"]["max_range"]) {
                kiss_icp_config_.max_range = config["data"]["max_range"].as<double>();
            }
            if (config["data"]["min_range"]) {
                kiss_icp_config_.min_range = config["data"]["min_range"].as<double>();
            }
        }
        if (config["mapping"]) {
            if (config["mapping"]["voxel_size"]) {
                kiss_icp_config_.voxel_size = config["mapping"]["voxel_size"].as<double>();
            }
            if (config["mapping"]["max_points_per_voxel"]) {
                kiss_icp_config_.max_points_per_voxel = config["mapping"]["max_points_per_voxel"].as<int>();
            }
        }
        if (config["adaptive_threshold"]) {
            if (config["adaptive_threshold"]["initial_threshold"]) {
                kiss_icp_config_.initial_threshold = config["adaptive_threshold"]["initial_threshold"].as<double>();
            }
            if (config["adaptive_threshold"]["min_motion_th"]) {
                kiss_icp_config_.min_motion_th = config["adaptive_threshold"]["min_motion_th"].as<double>();
            }
        }
        if (config["registration"]) {
            if (config["registration"]["max_num_iterations"]) {
                kiss_icp_config_.max_num_iterations = config["registration"]["max_num_iterations"].as<int>();
            }
            if (config["registration"]["convergence_criterion"]) {
                kiss_icp_config_.convergence_criterion = config["registration"]["convergence_criterion"].as<double>();
            }
            if (config["registration"]["max_num_threads"]) {
                kiss_icp_config_.max_num_threads = config["registration"]["max_num_threads"].as<int>();
            }
        }

        ROS_INFO("=== Odometry Node Parameters ===");
        ROS_INFO("PointCloud topic: %s", pointcloud_topic_.c_str());
        ROS_INFO("Odometry topic: %s", odom_topic_.c_str());
        ROS_INFO("Frames: odom_frame=%s, base_frame=%s, lidar_odom_frame=%s", odom_frame_.c_str(), base_frame_.c_str(), lidar_odom_frame_.c_str());
        ROS_INFO("Global config: use_sim_time=%d, publish_odom_tf=%d, publish_debug_clouds=%d", use_sim_time_, publish_odom_tf_, publish_debug_clouds_);
        ROS_INFO("Covariance config: position=%.2f, orientation=%.2f", position_covariance_, orientation_covariance_);
        ROS_INFO("KISS-ICP config:");
        ROS_INFO("  voxel_size=%.3f m", kiss_icp_config_.voxel_size);
        ROS_INFO("  max_iterations=%d", kiss_icp_config_.max_num_iterations);
        ROS_INFO("  convergence_threshold=%.6f", kiss_icp_config_.convergence_criterion);
        ROS_INFO("  valid_range: [%.2f, %.2f] m", kiss_icp_config_.min_range, kiss_icp_config_.max_range);
        ROS_INFO("  max_points_per_voxel=%d", kiss_icp_config_.max_points_per_voxel);
        ROS_INFO("  initial_threshold=%.2f", kiss_icp_config_.initial_threshold);
        ROS_INFO("  min_motion_th=%.2f", kiss_icp_config_.min_motion_th);
        ROS_INFO("  max_threads=%d", kiss_icp_config_.max_num_threads);
        ROS_INFO("  deskew=%d", kiss_icp_config_.deskew);
        ROS_INFO("Outlier filter: min_neighbors=%d, search_radius=%.2f m", min_neighbors_, neighbor_search_radius_);
        ROS_INFO("Tracking thresholds: inlier_ratio=%.2f, mean_distance=%.2f, model_deviation=%.2f",
                    inlier_ratio_threshold_, mean_distance_threshold_, model_deviation_threshold_);

        if (config["use_lidar2base_transform"]) {
            use_lidar2base_transform_ = config["use_lidar2base_transform"].as<bool>();
        }
        if (config["lidar2base_extrinsic"]) {
            auto extrinsic = config["lidar2base_extrinsic"].as<std::vector<double>>();
            if (extrinsic.size() == 16) {
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        lidar2base_transform_(i, j) = extrinsic[i * 4 + j];
                    }
                }
            }
        }
        ROS_INFO("lidar2base transform: enabled=%d", use_lidar2base_transform_);
        ROS_INFO("===============================");

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parse error: %s", e.what());
        ROS_ERROR("Using default parameters, continuing...");
    }
}

void OdometryNode::initializeParameters() {
    std::string config_file_path = "/home/ydkj/lidarslam_ws/src/lidar_slam2/config/slam_params.yaml";
    initializeParameters(config_file_path);
}

void OdometryNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        cloud_queue_.push(msg);
        ROS_DEBUG("PointCloud added to queue, queue size: %ld", cloud_queue_.size());
    }

    queue_cv_.notify_one();
}

void OdometryNode::processCloudThread() {
    ROS_INFO("Processing thread started");

    while (ros::ok() && !shutdown_) {
        sensor_msgs::PointCloud2::ConstPtr msg;

        if (tracking_lost_) {
            ROS_INFO("Tracking lost, waiting for relocalization result");
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
            ROS_DEBUG("Dequeued pointCloud, queue size: %ld", cloud_queue_.size());
        }

        if (msg) {
            ROS_INFO("=== PointCloud processing started ===");
            ROS_INFO("PointCloud timestamp: %d.%d", msg->header.stamp.sec, msg->header.stamp.nsec);

            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *input_cloud);

            ROS_INFO("Raw pointCloud size: %ld", input_cloud->size());

            if (input_cloud->empty()) {
                ROS_WARN("Empty pointCloud, skipping");
                continue;
            }

            if (use_lidar2base_transform_) {
                applyLidar2BaseTransform(input_cloud);
            }

            auto preprocess_start = std::chrono::high_resolution_clock::now();
            preprocessPointCloud(input_cloud);
            auto preprocess_end = std::chrono::high_resolution_clock::now();
            auto preprocess_duration = std::chrono::duration_cast<std::chrono::milliseconds>(preprocess_end - preprocess_start);

            ROS_INFO("Preprocessing done: processed points=%ld, time=%ldms", input_cloud->size(), preprocess_duration.count());

            bool should_init = false;
            {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                if (!initialized_) {
                    should_init = true;
                    initialized_ = true;
                }
            }

            if (should_init) {
                ROS_INFO("SLAM system initializing...");
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                current_pose_ = Sophus::SE3d();
                ROS_INFO("SLAM initialized, initial position: (0, 0, 0)");
            }

            std::string cloud_frame_id = msg->header.frame_id;
            std::vector<Eigen::Vector3d> frame;
            frame.reserve(input_cloud->size());
            for (const auto& point : input_cloud->points) {
                frame.emplace_back(point.x, point.y, point.z);
            }
            const auto timestamps = utils::GetTimestamps(msg);

            auto kiss_icp_start = std::chrono::high_resolution_clock::now();
            const auto& [preprocessed_frame, source] = kiss_icp_->RegisterFrame(frame, timestamps);
            auto kiss_icp_end = std::chrono::high_resolution_clock::now();
            auto kiss_icp_duration = std::chrono::duration_cast<std::chrono::milliseconds>(kiss_icp_end - kiss_icp_start);

            Sophus::SE3d new_pose = kiss_icp_->pose();

            ROS_INFO("KISS-ICP processing done, time=%ldms", kiss_icp_duration.count());

            const Eigen::Vector3d& translation = new_pose.translation();
            const Sophus::SO3d& so3 = new_pose.so3();

            if (std::isnan(translation.x()) || std::isnan(translation.y()) || std::isnan(translation.z())) {
                ROS_ERROR("KISS-ICP returned NaN translation: (%.4f, %.4f, %.4f)",
                         translation.x(), translation.y(), translation.z());
                ROS_ERROR("Resetting pose to identity matrix");
                {
                    std::lock_guard<std::mutex> lock(cloud_mutex_);
                    current_pose_ = Sophus::SE3d();
                }

                tracking_lost_ = true;
                ROS_WARN("Tracking lost, triggering global relocalization...");

                if (relocalize_client_) {
                    std_srvs::Empty srv;
                    if (relocalize_client_.call(srv)) {
                        ROS_INFO("Relocalization request sent");
                    } else {
                        ROS_WARN("Relocalization service call failed");
                    }
                } else {
                    ROS_WARN("Relocalization service client not initialized");
                }

                continue;
            }

            Eigen::Matrix3d rotation_matrix = so3.matrix();
            if (rotation_matrix.hasNaN() || !rotation_matrix.allFinite()) {
                ROS_ERROR("KISS-ICP returned rotation matrix with NaN or inf values");
                ROS_ERROR("Resetting pose to identity matrix");
                {
                    std::lock_guard<std::mutex> lock(cloud_mutex_);
                    current_pose_ = Sophus::SE3d();
                }

                tracking_lost_ = true;
                ROS_WARN("Tracking lost, triggering global relocalization...");

                if (relocalize_client_) {
                    std_srvs::Empty srv;
                    if (relocalize_client_.call(srv)) {
                        ROS_INFO("Relocalization request sent");
                    } else {
                        ROS_WARN("Relocalization service call failed");
                    }
                } else {
                    ROS_WARN("Relocalization service client not initialized");
                }

                continue;
            }

            const auto& match_quality = kiss_icp_->matchQuality();
            const auto& model_deviation = kiss_icp_->modelDeviation();

            double model_deviation_norm = model_deviation.so3().log().norm() + model_deviation.translation().norm();

            ROS_INFO("Match quality: inlier_ratio=%.3f, mean_distance=%.3f, model_deviation=%.3f",
                        match_quality.inlier_ratio, match_quality.mean_distance, model_deviation_norm);

            int lost_count = 0;
            if (match_quality.inlier_ratio < inlier_ratio_threshold_) lost_count++;
            if (match_quality.mean_distance > mean_distance_threshold_) lost_count++;
            if (model_deviation_norm > model_deviation_threshold_) lost_count++;

            if (lost_count >= 2) {
                if (!tracking_lost_) {
                    tracking_lost_ = true;
                    ROS_WARN("Tracking lost, triggering global relocalization...");

                    if (relocalize_client_) {
                        std_srvs::Empty srv;
                        if (relocalize_client_.call(srv)) {
                            ROS_INFO("Relocalization request sent");
                        } else {
                            ROS_WARN("Relocalization service call failed");
                        }
                    } else {
                        ROS_WARN("Relocalization service client not initialized");
                    }
                }
            } else {
                if (tracking_lost_) {
                    tracking_lost_ = false;
                    ROS_INFO("Tracking recovered");
                }
            }

            {
                std::lock_guard<std::mutex> lock(cloud_mutex_);
                current_pose_ = new_pose;
            }

            last_stamp_ = msg->header.stamp;

            if (!tracking_lost_) {
                publishOdometry(msg->header.stamp, msg->header.frame_id, new_pose);

                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(*input_cloud, cloud_msg);
                cloud_msg.header.frame_id = msg->header.frame_id;
                cloud_msg.header.stamp = msg->header.stamp;

                mapping_pointcloud_pub_.publish(cloud_msg);
                ROS_DEBUG("[Publish] Odometry and pointCloud published");

                ROS_DEBUG("=== PointCloud processing ended ===");
            }
        }
    }

    ROS_INFO("Processing thread exited");
}

void OdometryNode::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    auto start_time = std::chrono::high_resolution_clock::now();

    ROS_INFO("[Preprocess] Raw points: %ld", cloud->size());

    if (cloud->empty()) {
        return;
    }

    cloud = filterOutlierPoints(cloud);
    ROS_INFO("[Preprocess] After distance filter: %ld", cloud->size());

    if (neighbor_search_radius_ > 0 && min_neighbors_ > 0) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
        outlier_filter.setInputCloud(cloud);
        outlier_filter.setRadiusSearch(neighbor_search_radius_);
        outlier_filter.setMinNeighborsInRadius(min_neighbors_);
        pcl::PointCloud<pcl::PointXYZ> cleaned;
        outlier_filter.filter(cleaned);
        *cloud = cleaned;
        ROS_INFO("[Preprocess] After outlier filter: %ld", cloud->size());
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    ROS_INFO("[Preprocess] Total time: %ld ms", duration.count());
}

void OdometryNode::applyLidar2BaseTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty()) {
        return;
    }

    Eigen::Matrix4f transform = lidar2base_transform_.cast<float>();
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, transform);
    *cloud = transformed_cloud;

    ROS_DEBUG("Applied lidar2base transform, points: %ld", cloud->size());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OdometryNode::filterOutlierPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    ROS_DEBUG("[Far point filter] Starting filter, raw points: %ld", cloud->size());

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

    ROS_DEBUG("[Far point filter] After distance filter: %ld", filtered_cloud->size());

    if (neighbor_search_radius_ > 0 && min_neighbors_ > 0) {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
        outlier_filter.setInputCloud(filtered_cloud);
        outlier_filter.setRadiusSearch(neighbor_search_radius_);
        outlier_filter.setMinNeighborsInRadius(min_neighbors_);
        pcl::PointCloud<pcl::PointXYZ> cleaned;
        outlier_filter.filter(cleaned);
        *filtered_cloud = cleaned;
        ROS_DEBUG("[Far point filter] After outlier filter: %ld", filtered_cloud->size());
    }

    return filtered_cloud;
}

void OdometryNode::publishOdometry(const ros::Time& stamp, const std::string& cloud_frame_id, const Sophus::SE3d& pose_override) {
    const Sophus::SE3d current_pose_to_use = pose_override.matrix().isIdentity() ? [this]() -> Sophus::SE3d {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        return current_pose_;
    }() : pose_override;

    const auto pose = current_pose_to_use;

    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);
    const auto moving_frame = egocentric_estimation ? cloud_frame_id : base_frame_;
    const auto final_pose = egocentric_estimation ? pose : [&]() -> Sophus::SE3d {
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id, *tf2_buffer_);
        return cloud2base * pose * cloud2base.inverse();
    }();

    if (publish_odom_tf_) {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        if (invert_odom_tf_) {
            transform_msg.header.frame_id = moving_frame;
            transform_msg.child_frame_id = lidar_odom_frame_;
            transform_msg.transform = tf2::sophusToTransform(final_pose.inverse());
        } else {
            transform_msg.header.frame_id = lidar_odom_frame_;
            transform_msg.child_frame_id = moving_frame;
            transform_msg.transform = tf2::sophusToTransform(final_pose);
        }
        tf_broadcaster_->sendTransform(transform_msg);
    }

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = lidar_odom_frame_;
    odom_msg.child_frame_id = moving_frame;
    odom_msg.pose.pose = tf2::sophusToPose(final_pose);
    odom_msg.pose.covariance.fill(0.0);
    odom_msg.pose.covariance[0] = position_covariance_;
    odom_msg.pose.covariance[7] = position_covariance_;
    odom_msg.pose.covariance[14] = position_covariance_;
    odom_msg.pose.covariance[21] = orientation_covariance_;
    odom_msg.pose.covariance[28] = orientation_covariance_;
    odom_msg.pose.covariance[35] = orientation_covariance_;
    odom_pub_.publish(odom_msg);
    ROS_DEBUG("[Publish] Odometry published to topic: %s", odom_topic_.c_str());
}

void OdometryNode::publishTF(const ros::Time& stamp) {
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

bool OdometryNode::resetService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    initialized_ = false;

    if (kiss_icp_) {
        kiss_icp_->Reset();
    }

    ROS_INFO("SLAM reset");
    return true;
}

bool OdometryNode::setPoseService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    ROS_INFO("Setting pose...");
    ROS_INFO("Pose set successfully");
    return true;
}

void OdometryNode::relocalizationCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("Received relocalization result");

    const auto& pose = msg->pose.pose;
    Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    quaternion.normalize();

    Sophus::SE3d new_pose(quaternion, translation);

    updatePose(new_pose);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    tracking_lost_ = false;

    tracking_cv_.notify_one();

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = lidar_odom_frame_;
    transform.child_frame_id = "relocalization_pose";

    transform.transform.translation.x = translation.x();
    transform.transform.translation.y = translation.y();
    transform.transform.translation.z = translation.z();
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();

    tf_broadcaster_->sendTransform(transform);
    ROS_INFO("Published TF transform for relocalization pose: relocalization_pose");

    ROS_INFO("Relocalization successful, pose updated: position (%.3f, %.3f, %.3f)",
                translation.x(), translation.y(), translation.z());
}

void OdometryNode::updatePose(const Sophus::SE3d& new_pose) {
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        current_pose_ = new_pose;
    }
    if (kiss_icp_) {
        kiss_icp_->pose() = new_pose;
    }

    ros::Time now = ros::Time::now();
    publishOdometry(now, "rslidar", new_pose);

    ROS_INFO("Updated KISS-ICP pose: position=(%.3f, %.3f, %.3f)",
                new_pose.translation().x(), new_pose.translation().y(), new_pose.translation().z());
    ROS_INFO("Published odometry after relocalization");
}

} // namespace lidar_slam2
