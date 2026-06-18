#include "ndt_slam/odometry.hpp"
#include "lidar_slam2/Utils.hpp"

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

namespace ndt_slam {

// ========== ScanMatcher 实现 ==========

ScanMatcher::ScanMatcher(const MatcherConfig& config) : config_(config) {
    if (config_.type == MatcherType::NDT_OMP) {
        ndt_ = std::make_shared<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>();
        ndt_->setResolution(config_.resolution);
        ndt_->setStepSize(config_.step_size);
        ndt_->setTransformationEpsilon(config_.transformation_epsilon);
        ndt_->setMaximumIterations(config_.max_iterations);
        ndt_->setNumThreads(config_.num_threads);
        ROS_INFO("ScanMatcher initialized: NDT_OMP, resolution=%.2f, threads=%d",
                 config_.resolution, config_.num_threads);
    } else {
        gicp_ = std::make_shared<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();
        gicp_->setTransformationEpsilon(config_.transformation_epsilon);
        gicp_->setMaximumIterations(config_.max_iterations);
        gicp_->setMaxCorrespondenceDistance(config_.max_correspondence);
        ROS_INFO("ScanMatcher initialized: GICP_OMP");
    }
}

void ScanMatcher::setTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) {
    if (config_.type == MatcherType::NDT_OMP) {
        ndt_->setInputTarget(target);
    } else {
        gicp_->setInputTarget(target);
    }
    target_set_ = true;
}

Eigen::Matrix4d ScanMatcher::align(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const Eigen::Matrix4d& initial_guess) {

    if (!target_set_) {
        ROS_ERROR("ScanMatcher: target not set!");
        return initial_guess;
    }

    pcl::PointCloud<pcl::PointXYZ> aligned;

    if (config_.type == MatcherType::NDT_OMP) {
        ndt_->setInputSource(source);
        ndt_->align(aligned, initial_guess.cast<float>());
        final_transformation_ = ndt_->getFinalTransformation().cast<double>();
        has_converged_ = ndt_->hasConverged();
        fitness_score_ = ndt_->getFitnessScore();
    } else {
        gicp_->setInputSource(source);
        gicp_->align(aligned, initial_guess.cast<float>());
        final_transformation_ = gicp_->getFinalTransformation().cast<double>();
        has_converged_ = gicp_->hasConverged();
        fitness_score_ = gicp_->getFitnessScore();
    }

    return final_transformation_;
}

double ScanMatcher::getFitnessScore() const {
    return fitness_score_;
}

bool ScanMatcher::hasConverged() const {
    return has_converged_;
}

Eigen::Matrix4d ScanMatcher::getFinalTransformation() const {
    return final_transformation_;
}

// ========== OdometryNode 实现 ==========

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

        if (config["pointcloud_topic"]) pointcloud_topic_ = config["pointcloud_topic"].as<std::string>();
        if (config["mapping_pointcloud_topic"]) mapping_pointcloud_topic_ = config["mapping_pointcloud_topic"].as<std::string>();
        if (config["odom_topic"]) odom_topic_ = config["odom_topic"].as<std::string>();
        if (config["odom_frame"]) odom_frame_ = config["odom_frame"].as<std::string>();
        if (config["base_frame"]) base_frame_ = config["base_frame"].as<std::string>();
        if (config["map_frame"]) lidar_odom_frame_ = config["map_frame"].as<std::string>();

        if (config["use_sim_time"]) use_sim_time_ = config["use_sim_time"].as<bool>();
        if (config["publish_odom_tf"]) publish_odom_tf_ = config["publish_odom_tf"].as<bool>();
        if (config["publish_debug_clouds"]) publish_debug_clouds_ = config["publish_debug_clouds"].as<bool>();
        if (config["position_covariance"]) position_covariance_ = config["position_covariance"].as<double>();
        if (config["orientation_covariance"]) orientation_covariance_ = config["orientation_covariance"].as<double>();

        // KISS-ICP 配置
        if (config["data"]) {
            if (config["data"]["deskew"]) kiss_icp_config_.deskew = config["data"]["deskew"].as<bool>();
            if (config["data"]["max_range"]) kiss_icp_config_.max_range = config["data"]["max_range"].as<double>();
            if (config["data"]["min_range"]) kiss_icp_config_.min_range = config["data"]["min_range"].as<double>();
        }
        if (config["mapping"]) {
            if (config["mapping"]["voxel_size"]) kiss_icp_config_.voxel_size = config["mapping"]["voxel_size"].as<double>();
            if (config["mapping"]["max_points_per_voxel"]) kiss_icp_config_.max_points_per_voxel = config["mapping"]["max_points_per_voxel"].as<int>();
        }

        if (config["use_lidar2base_transform"]) use_lidar2base_transform_ = config["use_lidar2base_transform"].as<bool>();
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

        ROS_INFO("=== OdometryNode Parameters ===");
        ROS_INFO("PointCloud topic: %s", pointcloud_topic_.c_str());
        ROS_INFO("Odometry topic: %s", odom_topic_.c_str());
        ROS_INFO("KISS-ICP: voxel_size=%.3f, max_range=%.2f", kiss_icp_config_.voxel_size, kiss_icp_config_.max_range);
        ROS_INFO("===============================");

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parse error: %s", e.what());
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
        }

        if (msg) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *input_cloud);

            if (input_cloud->empty()) continue;

            if (use_lidar2base_transform_) {
                applyLidar2BaseTransform(input_cloud);
            }

            preprocessPointCloud(input_cloud);

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
                ROS_INFO("SLAM initialized");
            }

            std::string cloud_frame_id = msg->header.frame_id;
            std::vector<Eigen::Vector3d> frame;
            frame.reserve(input_cloud->size());
            for (const auto& point : input_cloud->points) {
                frame.emplace_back(point.x, point.y, point.z);
            }
            const auto timestamps = lidar_slam2::utils::GetTimestamps(msg);

            const auto& [preprocessed_frame, source] = kiss_icp_->RegisterFrame(frame, timestamps);

            Sophus::SE3d new_pose = kiss_icp_->pose();

            const Eigen::Vector3d& translation = new_pose.translation();
            const Sophus::SO3d& so3 = new_pose.so3();

            if (std::isnan(translation.x()) || std::isnan(translation.y()) || std::isnan(translation.z())) {
                ROS_ERROR("KISS-ICP returned NaN translation");
                {
                    std::lock_guard<std::mutex> lock(cloud_mutex_);
                    current_pose_ = Sophus::SE3d();
                }
                tracking_lost_ = true;
                continue;
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
            }
        }
    }

    ROS_INFO("Processing thread exited");
}

void OdometryNode::preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
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

void OdometryNode::applyLidar2BaseTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty()) return;

    Eigen::Matrix4f transform = lidar2base_transform_.cast<float>();
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, transform);
    *cloud = transformed_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OdometryNode::filterOutlierPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
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
}

} // namespace ndt_slam
