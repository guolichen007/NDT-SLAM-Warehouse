#include "lidar_slam2/LoopClosureNode.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace lidar_slam2 {

LoopClosureNode::LoopClosureNode(const ros::NodeHandle& nh)
    : nh_(nh) {
    try {
        initializeParameters();
    } catch (const std::exception& e) {
        std::cerr << "LoopClosureNode 初始化异常: " << e.what() << std::endl;
    }

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &LoopClosureNode::odomCallback, this);
    cloud_sub_ = nh_.subscribe(pointcloud_topic_, 10, &LoopClosureNode::pointCloudCallback, this);

    relocalize_srv_ = nh_.advertiseService("/loop_closure_node/relocalize", &LoopClosureNode::relocalizeService, this);

    ROS_INFO("Relocalization service created: /loop_closure_node/relocalize");

    relocalization_pub_ = nh_.advertise<nav_msgs::Odometry>(relocalization_topic_, 10);

    timer_ = nh_.createTimer(ros::Duration(5.0), &LoopClosureNode::timerCallback, this);

    last_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("LoopClosureNode initialized");
    ROS_INFO("Relocalization publisher initialized: %s", relocalization_topic_.c_str());
}

LoopClosureNode::LoopClosureNode(const std::string& config_file_path, const ros::NodeHandle& nh)
    : nh_(nh) {
    initializeParameters(config_file_path);

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &LoopClosureNode::odomCallback, this);
    cloud_sub_ = nh_.subscribe(pointcloud_topic_, 10, &LoopClosureNode::pointCloudCallback, this);

    relocalize_srv_ = nh_.advertiseService("/loop_closure_node/relocalize", &LoopClosureNode::relocalizeService, this);

    relocalization_pub_ = nh_.advertise<nav_msgs::Odometry>(relocalization_topic_, 10);

    last_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("LoopClosureNode initialized (using config file: %s)", config_file_path.c_str());
    ROS_INFO("Relocalization publisher initialized: %s", relocalization_topic_.c_str());
}

LoopClosureNode::~LoopClosureNode() {
}

void LoopClosureNode::initializeParameters(const std::string& config_file_path) {
    ROS_INFO("=== Initializing loop closure parameters (from %s) ===", config_file_path.c_str());

    try {
        YAML::Node config = YAML::LoadFile(config_file_path);

        if (config["odom_topic"]) {
            odom_topic_ = config["odom_topic"].as<std::string>();
        }
        if (config["pointcloud_topic"]) {
            pointcloud_topic_ = config["pointcloud_topic"].as<std::string>();
        }
        if (config["loop_detection_interval"]) {
            loop_detection_interval_ = config["loop_detection_interval"].as<int>();
        }

        loop_closure_detector_.configureFromYaml(config_file_path);

        ROS_INFO("=== Loop Closure Node Parameters ===");
        ROS_INFO("Odometry topic: %s", odom_topic_.c_str());
        ROS_INFO("PointCloud topic: %s", pointcloud_topic_.c_str());
        ROS_INFO("Loop detection interval: %d keyframes", loop_detection_interval_);
        ROS_INFO("===============================");

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parse error: %s", e.what());
        ROS_ERROR("Using default parameters, continuing...");
    }
}

void LoopClosureNode::initializeParameters() {
    std::string config_file_path = "/home/ydkj/lidarslam_ws/src/lidar_slam2/config/slam_params.yaml";
    initializeParameters(config_file_path);
}

void LoopClosureNode::timerCallback(const ros::TimerEvent&) {
    ROS_INFO("[Timer] Node running - keyframes: %zu, cloud points: %zu, initialized: %d",
                loop_closure_detector_.getKeyFrames().size(),
                last_cloud_->size(),
                initialized_);
}

void LoopClosureNode::odomCallback(const nav_msgs::Odometry::ConstPtr msg) {
    if (!initialized_) {
        initialized_ = true;
        ROS_INFO("Received first odometry message");
    }

    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond orientation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                   msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    last_pose_ = Sophus::SE3d(orientation, position);
    last_stamp_ = msg->header.stamp;
}

void LoopClosureNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    if (!initialized_) {
        ROS_WARN("Waiting for odometry message...");
        return;
    }

    try {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            ROS_WARN("Empty pointCloud, skipping");
            return;
        }

        *last_cloud_ = *cloud;

        ROS_INFO("PointCloud converted successfully, points: %ld", cloud->size());

        ROS_INFO("Preparing to add keyframe");
        loop_closure_detector_.addKeyFrame(last_pose_, cloud, msg->header.stamp);
        ROS_INFO("Keyframe added");

        const auto& keyframes = loop_closure_detector_.getKeyFrames();
        ROS_INFO("Current keyframe count: %ld", keyframes.size());
        if (keyframes.size() > 0 && keyframes.size() % loop_detection_interval_ == 0) {
            ROS_INFO("Performing loop closure detection");
            processLoopClosure();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error processing pointCloud: %s", e.what());
    }
}

void LoopClosureNode::processLoopClosure() {
    ROS_INFO("Performing loop closure detection...");

    LoopCandidate candidate = loop_closure_detector_.detectLoop();

    if (candidate.current_keyframe_id != -1 && candidate.candidate_keyframe_id != -1) {
        ROS_INFO("Loop found: keyframe %d <-> keyframe %d, similarity: %.3f",
                    candidate.current_keyframe_id, candidate.candidate_keyframe_id, candidate.similarity);

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
            ROS_INFO("Pose graph optimization successful");

            std::vector<KeyFrame> updated_keyframes(keyframes.begin(), keyframes.end());
            pose_graph_optimizer_.updateKeyFramePoses(updated_keyframes);
        } else {
            ROS_WARN("Pose graph optimization failed");
        }
    } else {
        ROS_INFO("No loop found");
    }
}

bool LoopClosureNode::relocalizeService(std_srvs::Empty::Request& request,
                                      std_srvs::Empty::Response& response) {
    ROS_INFO("Received relocalization request!");
    ROS_INFO("Current keyframe count: %zu", loop_closure_detector_.getKeyFrames().size());
    ROS_INFO("Current pointCloud points: %zu", last_cloud_->size());

    if (last_cloud_->empty()) {
        ROS_WARN("No pointCloud data available, cannot perform relocalization");
        return true;
    }

    Sophus::SE3d relocalized_pose = loop_closure_detector_.globalRelocalization(last_cloud_);

    if (relocalized_pose.so3().matrix().allFinite()) {
        ROS_INFO("Global relocalization successful: position (%.3f, %.3f, %.3f)",
                    relocalized_pose.translation().x(), relocalized_pose.translation().y(), relocalized_pose.translation().z());

        relocalized_pose_ = relocalized_pose;

        nav_msgs::Odometry relocalization_msg;
        relocalization_msg.header.stamp = ros::Time::now();
        relocalization_msg.header.frame_id = "odom";
        relocalization_msg.child_frame_id = "base_link";

        relocalization_msg.pose.pose.position.x = relocalized_pose.translation().x();
        relocalization_msg.pose.pose.position.y = relocalized_pose.translation().y();
        relocalization_msg.pose.pose.position.z = relocalized_pose.translation().z();

        Eigen::Quaterniond quat = relocalized_pose.so3().unit_quaternion();
        relocalization_msg.pose.pose.orientation.x = quat.x();
        relocalization_msg.pose.pose.orientation.y = quat.y();
        relocalization_msg.pose.pose.orientation.z = quat.z();
        relocalization_msg.pose.pose.orientation.w = quat.w();

        for (int i = 0; i < 6; ++i) {
            relocalization_msg.pose.covariance[i * 6 + i] = 0.1;
        }

        relocalization_pub_.publish(relocalization_msg);
        ROS_INFO("Relocalization result published to: %s", relocalization_topic_.c_str());
    } else {
        ROS_WARN("Global relocalization failed");
    }
    return true;
}

} // namespace lidar_slam2
