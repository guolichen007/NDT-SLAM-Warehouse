#include "ndt_slam/mapping.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace ndt_slam {

// ========== KeyFrameManager 实现 ==========

void KeyFrameManager::configureFromYaml(const YAML::Node& config) {
    translation_threshold_ = config["translation_threshold"] ? config["translation_threshold"].as<double>() : 0.8;
    double rotation_deg = config["rotation_threshold"] ? config["rotation_threshold"].as<double>() : 8.0;
    rotation_threshold_ = rotation_deg * M_PI / 180.0;
    time_threshold_ = config["time_threshold"] ? config["time_threshold"].as<double>() : 1.5;
    max_keyframes_ = config["max_keyframes"] ? config["max_keyframes"].as<int>() : 0;
}

void KeyFrameManager::configure(double translation_threshold, double rotation_threshold,
                                double time_threshold, int max_keyframes) {
    translation_threshold_ = translation_threshold;
    rotation_threshold_ = rotation_threshold * M_PI / 180.0;
    time_threshold_ = time_threshold;
    max_keyframes_ = max_keyframes;
}

uint64_t KeyFrameManager::computeSpatialHash(const Eigen::Vector3d& position, double cell_size) const {
    int64_t x = static_cast<int64_t>(std::floor(position.x() / cell_size));
    int64_t y = static_cast<int64_t>(std::floor(position.y() / cell_size));
    int64_t z = static_cast<int64_t>(std::floor(position.z() / cell_size));
    return static_cast<uint64_t>((x * 73856093) ^ (y * 19349663) ^ (z * 83492791));
}

std::vector<int> KeyFrameManager::getNearbyKeyFrames(const Eigen::Vector3d& position, double radius) const {
    std::vector<int> result;
    for (int i = 0; i < keyframes_.size(); ++i) {
        double dist = (keyframes_[i].pose_.translation() - position).norm();
        if (dist < radius) {
            result.push_back(i);
        }
    }
    return result;
}

const KeyFrame* KeyFrameManager::getKeyFrameById(uint64_t id) const {
    for (const auto& kf : keyframes_) {
        if (kf.id_ == id) {
            return &kf;
        }
    }
    return nullptr;
}

bool KeyFrameManager::isKeyFrame(const Sophus::SE3d& current_pose, const ros::Time& current_time) {
    if (keyframes_.empty()) {
        return true;
    }

    Eigen::Vector3d translation_diff = current_pose.translation() - last_keyframe_pose_.translation();
    double translation_distance = translation_diff.norm();

    Sophus::SO3d rotation_diff = last_keyframe_pose_.so3().inverse() * current_pose.so3();
    double rotation_angle = rotation_diff.log().norm();

    double time_diff = (current_time - last_keyframe_time_).toSec();

    if (translation_distance > translation_threshold_ ||
        rotation_angle > rotation_threshold_ ||
        time_diff > time_threshold_) {
        return true;
    }

    return false;
}

void KeyFrameManager::addKeyFrame(const Sophus::SE3d& pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Time& stamp) {
    if (max_keyframes_ > 0 && keyframes_.size() >= static_cast<size_t>(max_keyframes_)) {
        keyframes_.pop_front();
    }

    uint64_t new_id = last_keyframe_id_ + 1;
    KeyFrame keyframe(new_id, stamp, pose, cloud);
    keyframes_.push_back(keyframe);

    uint64_t hash = computeSpatialHash(pose.translation(), 8.0);
    spatial_index_[hash].push_back(new_id);

    last_keyframe_id_ = new_id;
    last_keyframe_time_ = stamp;
    last_keyframe_pose_ = pose;
}

bool KeyFrameManager::saveKeyFrameDatabase(const std::string& session_dir) const {
    try {
        std::filesystem::path session_path(session_dir);
        std::filesystem::path keyframes_dir = session_path / "keyframes";
        std::filesystem::create_directories(keyframes_dir);

        for (const auto& kf : keyframes_) {
            std::string pcd_filename = keyframes_dir / (std::to_string(kf.id_) + ".pcd");
            pcl::io::savePCDFileBinary(pcd_filename, *kf.cloud_);
        }

        std::string poses_raw_file = session_path / "poses_raw.txt";
        std::ofstream ofs_raw(poses_raw_file);
        if (!ofs_raw.is_open()) {
            ROS_ERROR("Failed to open poses_raw.txt for writing");
            return false;
        }
        ofs_raw << std::fixed << std::setprecision(6);
        for (const auto& kf : keyframes_) {
            Eigen::Vector3d t = kf.pose_.translation();
            Eigen::Quaterniond q(kf.pose_.unit_quaternion());
            ofs_raw << kf.id_ << " "
                    << kf.stamp_.sec << "." << std::setw(9) << std::setfill('0') << kf.stamp_.nsec << " "
                    << t.x() << " " << t.y() << " " << t.z() << " "
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        ofs_raw.close();

        ROS_INFO("Saved keyframe database: %zu keyframes to %s", keyframes_.size(), session_dir.c_str());
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception saving keyframe database: %s", e.what());
        return false;
    }
}

bool KeyFrameManager::loadKeyFrameDatabase(const std::string& session_dir) {
    try {
        std::filesystem::path session_path(session_dir);
        std::filesystem::path keyframes_dir = session_path / "keyframes";

        if (!std::filesystem::exists(keyframes_dir)) {
            ROS_ERROR("Keyframes directory does not exist: %s", keyframes_dir.c_str());
            return false;
        }

        std::string poses_file = session_path / "poses_raw.txt";
        std::ifstream ifs(poses_file);
        if (!ifs.is_open()) {
            ROS_ERROR("Failed to open poses_raw.txt");
            return false;
        }

        keyframes_.clear();
        last_keyframe_id_ = 0;

        std::string line;
        while (std::getline(ifs, line)) {
            std::istringstream iss(line);
            uint64_t id;
            double timestamp;
            double tx, ty, tz, qx, qy, qz, qw;

            if (!(iss >> id >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
                continue;
            }

            std::string pcd_file = keyframes_dir / (std::to_string(id) + ".pcd");
            if (!std::filesystem::exists(pcd_file)) {
                continue;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile(pcd_file, *cloud) < 0) {
                continue;
            }

            Sophus::SE3d pose(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
            ros::Time stamp;
            stamp.sec = static_cast<uint32_t>(timestamp);
            stamp.nsec = static_cast<uint32_t>((timestamp - stamp.sec) * 1e9);

            KeyFrame kf(id, stamp, pose, cloud);
            keyframes_.push_back(kf);

            if (id > last_keyframe_id_) {
                last_keyframe_id_ = id;
            }
        }
        ifs.close();

        if (!keyframes_.empty()) {
            last_keyframe_pose_ = keyframes_.back().pose_;
            last_keyframe_time_ = keyframes_.back().stamp_;
        }

        ROS_INFO("Loaded keyframe database: %zu keyframes from %s", keyframes_.size(), session_dir.c_str());
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception loading keyframe database: %s", e.what());
        return false;
    }
}

bool KeyFrameManager::saveOptimizedPoses(const std::string& filepath) const {
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        ROS_ERROR("Failed to open file for writing: %s", filepath.c_str());
        return false;
    }

    ofs << std::fixed << std::setprecision(6);
    for (const auto& kf : keyframes_) {
        Sophus::SE3d pose = kf.has_refined_pose_ ? kf.pose_refined_ : kf.pose_;
        Eigen::Vector3d t = pose.translation();
        Eigen::Quaterniond q(pose.unit_quaternion());
        ofs << kf.id_ << " "
            << kf.stamp_.sec << "." << std::setw(9) << std::setfill('0') << kf.stamp_.nsec << " "
            << t.x() << " " << t.y() << " " << t.z() << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    ofs.close();

    ROS_INFO("Saved optimized poses: %zu keyframes to %s", keyframes_.size(), filepath.c_str());
    return true;
}

bool KeyFrameManager::loadOptimizedPoses(const std::string& filepath) {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open file for reading: %s", filepath.c_str());
        return false;
    }

    std::string line;
    while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        uint64_t id;
        double timestamp;
        double tx, ty, tz, qx, qy, qz, qw;

        if (!(iss >> id >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
            continue;
        }

        for (auto& kf : keyframes_) {
            if (kf.id_ == id) {
                kf.pose_refined_ = Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
                kf.has_refined_pose_ = true;
                break;
            }
        }
    }
    ifs.close();

    ROS_INFO("Loaded optimized poses from %s", filepath.c_str());
    return true;
}

void KeyFrameManager::updateKeyFramePose(uint64_t id, const Sophus::SE3d& new_pose) {
    for (auto& kf : keyframes_) {
        if (kf.id_ == id) {
            kf.pose_refined_ = new_pose;
            kf.has_refined_pose_ = true;
            break;
        }
    }
}

// ========== MappingNode 实现 ==========

MappingNode::MappingNode(const ros::NodeHandle& nh)
    : nh_(nh) {

    initializeParameters();
    ROS_INFO("MappingNode parameters init.");

    global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    last_position_ = Eigen::Vector3d::Zero();
    last_orientation_ = Eigen::Quaterniond::Identity();
    publishInitialTransform();

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MappingNode::odomCallback, this);
    mapping_pointcloud_sub_ = nh_.subscribe(mapping_pointcloud_topic_, 10, &MappingNode::pointCloudCallback, this);

    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 10);
    current_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(current_cloud_topic_, 10);

    save_map_srv_ = nh_.advertiseService("/mapping_node/save_map", &MappingNode::saveMapService, this);
    load_map_srv_ = nh_.advertiseService("/mapping_node/load_map", &MappingNode::loadMapService, this);

    if (num_worker_threads_ > 0) {
        for (int i = 0; i < num_worker_threads_; ++i) {
            worker_threads_.emplace_back(&MappingNode::processingWorker, this);
        }
        ROS_INFO("MappingNode initialized with %d worker threads (async mode)", num_worker_threads_);
    } else {
        ROS_INFO("MappingNode initialized (sync mode)");
    }
}

MappingNode::MappingNode(const std::string& config_file_path, const ros::NodeHandle& nh)
    : nh_(nh) {

    initializeParameters(config_file_path);
    ROS_INFO("MappingNode parameters init.");

    global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    last_position_ = Eigen::Vector3d::Zero();
    last_orientation_ = Eigen::Quaterniond::Identity();
    publishInitialTransform();

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MappingNode::odomCallback, this);
    mapping_pointcloud_sub_ = nh_.subscribe(mapping_pointcloud_topic_, 10, &MappingNode::pointCloudCallback, this);

    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_, 10);
    current_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(current_cloud_topic_, 10);

    save_map_srv_ = nh_.advertiseService("/mapping_node/save_map", &MappingNode::saveMapService, this);
    load_map_srv_ = nh_.advertiseService("/mapping_node/load_map", &MappingNode::loadMapService, this);

    if (num_worker_threads_ > 0) {
        for (int i = 0; i < num_worker_threads_; ++i) {
            worker_threads_.emplace_back(&MappingNode::processingWorker, this);
        }
        ROS_INFO("MappingNode initialized with %d worker threads (async mode)", num_worker_threads_);
    } else {
        ROS_INFO("MappingNode initialized (sync mode)");
    }
}

MappingNode::~MappingNode() {
    running_ = false;
    task_cv_.notify_all();
    for (auto& worker : worker_threads_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void MappingNode::processingWorker() {
    ROS_INFO("Worker thread started");

    while (ros::ok() && running_) {
        MapTask task;

        {
            std::unique_lock<std::mutex> lock(task_queue_mutex_);
            task_cv_.wait(lock, [this] {
                return !task_queue_.empty() || !running_;
            });

            if (!running_ && task_queue_.empty()) {
                break;
            }

            if (!task_queue_.empty()) {
                task = std::move(task_queue_.front());
                task_queue_.pop();
            } else {
                continue;
            }
        }

        addPointsToMap(task.cloud, task.position, task.orientation);

        frame_count_++;
        if (frame_count_ % map_update_interval_ == 0) {
            publishMap();
        }
    }

    ROS_INFO("Worker thread finished");
}

void MappingNode::initializeParameters() {
    std::string config_file_path = "/home/ydkj/lidarslam_ws/src/lidar_slam2/config/slam_params.yaml";
    initializeParameters(config_file_path);
}

void MappingNode::initializeParameters(const std::string& config_file_path) {
    ROS_INFO("=== Initializing mapping parameters (from %s) ===", config_file_path.c_str());

    try {
        YAML::Node config = YAML::LoadFile(config_file_path);

        if (config["odom_topic"]) odom_topic_ = config["odom_topic"].as<std::string>();
        if (config["mapping_pointcloud_topic"]) mapping_pointcloud_topic_ = config["mapping_pointcloud_topic"].as<std::string>();
        if (config["map_topic"]) map_topic_ = config["map_topic"].as<std::string>();
        if (config["current_cloud_topic"]) current_cloud_topic_ = config["current_cloud_topic"].as<std::string>();
        if (config["map_frame"]) map_frame_ = config["map_frame"].as<std::string>();

        if (config["map_voxel_size"]) voxel_size_ = config["map_voxel_size"].as<double>();
        if (config["max_map_size"]) max_map_size_ = config["max_map_size"].as<double>();
        if (config["use_voxel_filter"]) use_voxel_filter_ = config["use_voxel_filter"].as<bool>();
        if (config["map_update_interval"]) map_update_interval_ = config["map_update_interval"].as<int>();
        if (config["num_worker_threads"]) num_worker_threads_ = config["num_worker_threads"].as<int>();

        ROS_INFO("=== MappingNode Parameters ===");
        ROS_INFO("Voxel size: %.3f m", voxel_size_);
        ROS_INFO("Max map range: %.1f m", max_map_size_);
        ROS_INFO("==============================");

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parse error: %s", e.what());
    }
}

void MappingNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (!has_first_odom_) {
        has_first_odom_ = true;
        ROS_INFO("Received first odometry message");
    }

    last_position_ << msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z;

    last_orientation_ = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    if (tf_broadcaster_) {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = msg->header.stamp;
        transform_msg.header.frame_id = map_frame_;
        transform_msg.child_frame_id = odom_frame_;

        transform_msg.transform.translation.x = last_position_.x();
        transform_msg.transform.translation.y = last_position_.y();
        transform_msg.transform.translation.z = last_position_.z();
        transform_msg.transform.rotation.x = last_orientation_.x();
        transform_msg.transform.rotation.y = last_orientation_.y();
        transform_msg.transform.rotation.z = last_orientation_.z();
        transform_msg.transform.rotation.w = last_orientation_.w();

        tf_broadcaster_->sendTransform(transform_msg);
    }
}

void MappingNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!has_first_odom_) {
        ROS_WARN("[PointCloud callback] Waiting for odometry message...");
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);

    if (input_cloud->empty()) {
        ROS_WARN("[PointCloud callback] Empty pointCloud, skipping");
        return;
    }

    Eigen::Vector3d current_position;
    Eigen::Quaterniond current_orientation;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_position = last_position_;
        current_orientation = last_orientation_;
    }

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = current_orientation.toRotationMatrix();
    transform.block<3, 1>(0, 3) = current_position;
    pcl::transformPointCloud(*input_cloud, transformed_cloud, transform.cast<float>());

    sensor_msgs::PointCloud2 current_cloud_msg;
    pcl::toROSMsg(transformed_cloud, current_cloud_msg);
    current_cloud_msg.header.frame_id = map_frame_;
    current_cloud_msg.header.stamp = msg->header.stamp;
    current_cloud_pub_.publish(current_cloud_msg);

    if (num_worker_threads_ > 0) {
        MapTask task;
        task.cloud = input_cloud;
        task.position = current_position;
        task.orientation = current_orientation;
        task.stamp = msg->header.stamp;

        {
            std::lock_guard<std::mutex> lock(task_queue_mutex_);
            task_queue_.push(std::move(task));
        }
        task_cv_.notify_one();
    } else {
        addPointsToMap(input_cloud, current_position, current_orientation);
        publishMap();
    }
}

void MappingNode::addPointsToMap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                  const Eigen::Vector3d& position,
                                  const Eigen::Quaterniond& orientation) {
    std::lock_guard<std::mutex> lock(map_mutex_);

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    transform.block<3, 1>(0, 3) = position;

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*cloud, transformed_cloud, transform.cast<float>());

    for (const auto& point : transformed_cloud.points) {
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

    if (use_voxel_filter_ && global_map_->size() > 5000) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(global_map_);
        voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        pcl::PointCloud<pcl::PointXYZ> filtered;
        voxel_filter.filter(filtered);
        *global_map_ = filtered;
    }
}

void MappingNode::publishInitialTransform() {
    if (!tf_broadcaster_) {
        return;
    }

    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = map_frame_;
    transform_msg.child_frame_id = odom_frame_;

    transform_msg.transform.translation.x = 0.0;
    transform_msg.transform.translation.y = 0.0;
    transform_msg.transform.translation.z = 0.0;
    transform_msg.transform.rotation.x = 0.0;
    transform_msg.transform.rotation.y = 0.0;
    transform_msg.transform.rotation.z = 0.0;
    transform_msg.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform_msg);
    ROS_INFO("Published initial transform: %s -> %s", map_frame_.c_str(), odom_frame_.c_str());
}

void MappingNode::publishMap() {
    if (global_map_->empty()) {
        ROS_WARN("[Publish map] Map is empty, skipping publish");
        return;
    }

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*global_map_, map_msg);
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = ros::Time::now();

    map_pub_.publish(map_msg);
}

bool MappingNode::saveMapService(lidar_slam2_msgs::SaveMap::Request& request,
                                 lidar_slam2_msgs::SaveMap::Response& response) {
    ROS_INFO("Received save map request: %s", request.file_path.c_str());

    std::lock_guard<std::mutex> lock(map_mutex_);

    if (global_map_->empty()) {
        response.success = false;
        response.message = "Map is empty, cannot save";
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

        if (result == 0) {
            response.success = true;
            response.message = "Map saved successfully";
            response.saved_file_path = file_path;
            response.num_points = global_map_->size();
            ROS_INFO("Map saved successfully: %s, points: %ld", file_path.c_str(), global_map_->size());
        } else {
            response.success = false;
            response.message = "Failed to write file";
            response.num_points = 0;
        }
    } catch (const std::exception& e) {
        response.success = false;
        response.message = std::string("Save error: ") + e.what();
        response.num_points = 0;
    }
    return true;
}

bool MappingNode::loadMapService(lidar_slam2_msgs::LoadMap::Request& request,
                                 lidar_slam2_msgs::LoadMap::Response& response) {
    ROS_INFO("Received load map request: %s", request.file_path.c_str());

    std::string file_path = request.file_path;
    if (file_path.empty()) {
        response.success = false;
        response.message = "File path is empty";
        response.num_points = 0;
        return true;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    try {
        pcl::PCDReader reader;
        int result = reader.read(file_path, *loaded_cloud);

        if (result < 0) {
            response.success = false;
            response.message = "Failed to read file";
            response.num_points = 0;
            return true;
        }

        if (request.load_as_current) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            global_map_ = loaded_cloud;
            ROS_INFO("Replaced current map, loaded points: %ld", loaded_cloud->size());
            publishMap();
        }

        response.success = true;
        response.message = "Map loaded successfully";
        response.num_points = loaded_cloud->size();
        ROS_INFO("Map loaded successfully: %s, points: %ld", file_path.c_str(), loaded_cloud->size());

    } catch (const std::exception& e) {
        response.success = false;
        response.message = std::string("Load error: ") + e.what();
        response.num_points = 0;
    }
    return true;
}

} // namespace ndt_slam
