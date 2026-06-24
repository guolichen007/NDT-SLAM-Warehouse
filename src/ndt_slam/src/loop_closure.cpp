#include "ndt_slam/loop_closure.hpp"
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace ndt_slam {

// ========== ScanContext 实现 ==========

Eigen::MatrixXd ScanContext::generate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const Eigen::Vector3d& origin) {
    Eigen::MatrixXd sc = Eigen::MatrixXd::Zero(num_rings_, num_sectors_);

    if (!cloud || cloud->empty()) {
        return sc;
    }

    try {
        for (const auto& point : cloud->points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }

            Eigen::Vector3d p(point.x, point.y, point.z);
            Eigen::Vector3d rel_p = p - origin;

            double range = rel_p.norm();
            if (range > max_range_) continue;

            double angle = std::atan2(rel_p.y(), rel_p.x());
            if (angle < 0) angle += 2 * M_PI;

            int ring_idx = static_cast<int>((range / max_range_) * num_rings_);
            int sector_idx = static_cast<int>((angle / (2 * M_PI)) * num_sectors_);

            if (ring_idx >= num_rings_) ring_idx = num_rings_ - 1;
            if (sector_idx >= num_sectors_) sector_idx = num_sectors_ - 1;

            if (ring_idx >= 0 && ring_idx < num_rings_ && sector_idx >= 0 && sector_idx < num_sectors_) {
                if (rel_p.z() > sc(ring_idx, sector_idx)) {
                    sc(ring_idx, sector_idx) = rel_p.z();
                }
            }
        }
    } catch (const std::exception& e) {
        // 捕获异常，避免程序崩溃
    }

    return sc;
}

double ScanContext::calculateSimilarity(const Eigen::MatrixXd& sc1, const Eigen::MatrixXd& sc2) {
    double diff = (sc1 - sc2).norm();
    double max_diff = std::sqrt(sc1.rows() * sc1.cols()) * 5.0;
    double similarity = 1.0 - std::min(diff / max_diff, 1.0);
    return similarity;
}

double ScanContext::calculateSimilarityWithShift(const Eigen::MatrixXd& sc1, const Eigen::MatrixXd& sc2, int sector_shift) const {
    if (sc1.rows() != sc2.rows() || sc1.cols() != sc2.cols()) {
        return 0.0;
    }

    int rows = sc1.rows();
    int cols = sc1.cols();

    // 将 sc2 的列向右循环移位 shift 个位置
    double total_diff = 0.0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int shifted_c = (c + sector_shift + cols) % cols;
            double diff = sc1(r, c) - sc2(r, shifted_c);
            total_diff += diff * diff;
        }
    }

    double max_diff = rows * cols * 25.0;  // 假设最大值 5.0
    double similarity = 1.0 - std::min(std::sqrt(total_diff) / std::sqrt(max_diff), 1.0);
    return similarity;
}

int ScanContext::findBestMatch(const Eigen::MatrixXd& current_sc, const std::vector<Eigen::MatrixXd>& sc_list) {
    if (sc_list.empty()) return -1;

    double best_similarity = -1.0;
    int best_idx = -1;

    for (int i = 0; i < sc_list.size(); ++i) {
        double similarity = calculateSimilarity(current_sc, sc_list[i]);
        if (similarity > best_similarity) {
            best_similarity = similarity;
            best_idx = i;
        }
    }

    return best_idx;
}

// ========== PoseGraphOptimizer 实现 ==========

PoseGraphOptimizer::PoseGraphOptimizer() {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>())
    );

    optimizer_ = std::make_unique<g2o::SparseOptimizer>();
    optimizer_->setAlgorithm(solver);
    optimizer_->setVerbose(false);

    odometry_information_.setIdentity();
    odometry_information_(0, 0) = 100.0;
    odometry_information_(1, 1) = 100.0;
    odometry_information_(2, 2) = 100.0;
    odometry_information_(3, 3) = 100.0;
    odometry_information_(4, 4) = 100.0;
    odometry_information_(5, 5) = 100.0;

    loop_information_.setIdentity();
    loop_information_(0, 0) = 100.0;
    loop_information_(1, 1) = 100.0;
    loop_information_(2, 2) = 100.0;
    loop_information_(3, 3) = 100.0;
    loop_information_(4, 4) = 100.0;
    loop_information_(5, 5) = 100.0;
}

void PoseGraphOptimizer::addKeyFrame(const KeyFrame& keyframe) {
    if (vertices_.find(keyframe.id_) != vertices_.end()) {
        return;
    }

    g2o::VertexSE3* vertex = new g2o::VertexSE3();
    vertex->setId(keyframe.id_);

    Eigen::Isometry3d pose;
    pose = keyframe.pose_.matrix();
    vertex->setEstimate(pose);

    if (keyframe.id_ == 0) {
        vertex->setFixed(true);
    }

    optimizer_->addVertex(vertex);
    vertices_[keyframe.id_] = vertex;
}

void PoseGraphOptimizer::addOdometryEdge(int from_id, int to_id, const Sophus::SE3d& relative_pose, const Eigen::Matrix<double, 6, 6>& information) {
    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end()) {
        return;
    }

    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->setVertex(0, vertices_[from_id]);
    edge->setVertex(1, vertices_[to_id]);

    Eigen::Isometry3d relative_pose_isometry;
    relative_pose_isometry = relative_pose.matrix();
    edge->setMeasurement(relative_pose_isometry);

    edge->setInformation(information);

    optimizer_->addEdge(edge);
}

void PoseGraphOptimizer::addLoopEdge(int from_id, int to_id, const Sophus::SE3d& relative_pose, const Eigen::Matrix<double, 6, 6>& information) {
    if (vertices_.find(from_id) == vertices_.end() || vertices_.find(to_id) == vertices_.end()) {
        return;
    }

    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    edge->setVertex(0, vertices_[from_id]);
    edge->setVertex(1, vertices_[to_id]);

    Eigen::Isometry3d relative_pose_isometry;
    relative_pose_isometry = relative_pose.matrix();
    edge->setMeasurement(relative_pose_isometry);

    edge->setInformation(information);

    g2o::RobustKernelHuber* robust_kernel = new g2o::RobustKernelHuber();
    robust_kernel->setDelta(1.0);
    edge->setRobustKernel(robust_kernel);

    optimizer_->addEdge(edge);
}

bool PoseGraphOptimizer::optimize(int iterations) {
    if (optimizer_->vertices().size() < 2) {
        return false;
    }

    optimizer_->initializeOptimization();
    int result = optimizer_->optimize(iterations);

    return result > 0;
}

Sophus::SE3d PoseGraphOptimizer::getOptimizedPose(int keyframe_id) const {
    auto it = vertices_.find(keyframe_id);
    if (it == vertices_.end()) {
        return Sophus::SE3d();
    }

    g2o::VertexSE3* vertex = it->second;
    Eigen::Isometry3d pose = vertex->estimate();

    return Sophus::SE3d(pose.matrix());
}

void PoseGraphOptimizer::updateKeyFramePoses(std::vector<KeyFrame>& keyframes) {
    for (auto& keyframe : keyframes) {
        Sophus::SE3d optimized_pose = getOptimizedPose(keyframe.id_);
        if (optimized_pose.so3().matrix().allFinite()) {
            keyframe.pose_ = optimized_pose;
        }
    }
}

// ========== LoopClosureDetector 实现 ==========

void LoopClosureDetector::configureFromYaml(const std::string& config_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_file_path);

        if (config["scan_context"]) {
            YAML::Node sc = config["scan_context"];

            int num_rings = sc["num_rings"] ? sc["num_rings"].as<int>() : 20;
            int num_sectors = sc["num_sectors"] ? sc["num_sectors"].as<int>() : 60;
            double max_range = sc["max_range"] ? sc["max_range"].as<double>() : 80.0;

            scan_context_.configure(num_rings, num_sectors, max_range);

            spatial_search_radius_ = sc["spatial_search_radius"] ? sc["spatial_search_radius"].as<double>() : 8.0;
            similarity_threshold_ = sc["similarity_threshold"] ? sc["similarity_threshold"].as<double>() : 0.8;

            translation_threshold_ = sc["translation_threshold"] ? sc["translation_threshold"].as<double>() : 1.0;
            double rotation_deg = sc["rotation_threshold"] ? sc["rotation_threshold"].as<double>() : 10.0;
            rotation_threshold_ = rotation_deg * M_PI / 180.0;
        }

        if (config["keyframe"]) {
            YAML::Node kf = config["keyframe"];
            keyframe_manager_.configureFromYaml(kf);
        }

    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parse error (LoopClosureDetector): " << e.what() << std::endl;
    }
}

void LoopClosureDetector::configure(int num_rings, int num_sectors, double max_range,
                                    double spatial_search_radius, double similarity_threshold,
                                    double translation_threshold, double rotation_threshold) {
    scan_context_.configure(num_rings, num_sectors, max_range);
    spatial_search_radius_ = spatial_search_radius;
    similarity_threshold_ = similarity_threshold;
    translation_threshold_ = translation_threshold;
    rotation_threshold_ = rotation_threshold;
}

void LoopClosureDetector::addKeyFrame(const Sophus::SE3d& pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Time& stamp) {
    try {
        if (keyframe_manager_.isKeyFrame(pose, stamp)) {
            keyframe_manager_.addKeyFrame(pose, cloud, stamp);

            const auto& keyframes = keyframe_manager_.getKeyFrames();
            if (keyframes.empty()) {
                return;
            }
            KeyFrame& last_keyframe = const_cast<KeyFrame&>(keyframes.back());
            // 修正：统一使用 Zero() 作为原点，保持坐标基准一致
            last_keyframe.scan_context_ = scan_context_.generate(cloud, Eigen::Vector3d::Zero());

            scan_context_list_.push_back(last_keyframe.scan_context_);
        }
    } catch (const std::exception& e) {
        // 捕获异常，避免程序崩溃
    }
}

LoopCandidate LoopClosureDetector::detectLoop() {
    LoopCandidate candidate;
    candidate.current_keyframe_id = -1;
    candidate.candidate_keyframe_id = -1;

    const auto& keyframes = keyframe_manager_.getKeyFrames();
    if (keyframes.size() < 30) return candidate;

    const KeyFrame& current_keyframe = keyframes.back();

    const int min_keyframe_gap = 25;
    const double min_loop_distance = 5.0;
    std::vector<int> spatial_candidates;
    for (size_t i = 0; i < keyframes.size() - min_keyframe_gap; ++i) {
        const KeyFrame& kf = keyframes[i];
        double distance = (current_keyframe.pose_.translation() - kf.pose_.translation()).norm();
        if (distance < spatial_search_radius_ && distance > min_loop_distance) {
            spatial_candidates.push_back(i);
        }
    }

    if (spatial_candidates.empty()) return candidate;

    std::vector<Eigen::MatrixXd> candidate_scs;
    for (int i : spatial_candidates) {
        candidate_scs.push_back(keyframes[i].scan_context_);
    }

    int best_candidate_idx = scan_context_.findBestMatch(current_keyframe.scan_context_, candidate_scs);
    if (best_candidate_idx == -1) return candidate;

    int actual_idx = spatial_candidates[best_candidate_idx];
    const KeyFrame& candidate_keyframe = keyframes[actual_idx];

    double similarity = scan_context_.calculateSimilarity(current_keyframe.scan_context_, candidate_keyframe.scan_context_);
    if (similarity < similarity_threshold_) return candidate;

    Sophus::SE3d initial_guess = candidate_keyframe.pose_.inverse() * current_keyframe.pose_;
    Sophus::SE3d refined_pose = refinePose(current_keyframe.cloud_, candidate_keyframe.cloud_, initial_guess);

    Sophus::SE3d odometry_pose = candidate_keyframe.pose_.inverse() * current_keyframe.pose_;
    if (!checkConsistency(refined_pose, odometry_pose)) return candidate;

    candidate.current_keyframe_id = current_keyframe.id_;
    candidate.candidate_keyframe_id = candidate_keyframe.id_;
    candidate.relative_pose = refined_pose;
    candidate.similarity = similarity;

    return candidate;
}

bool LoopClosureDetector::checkConsistency(const Sophus::SE3d& loop_pose, const Sophus::SE3d& odometry_pose) {
    double translation_diff = (loop_pose.translation() - odometry_pose.translation()).norm();

    Sophus::SO3d rotation_diff = odometry_pose.so3().inverse() * loop_pose.so3();
    double rotation_diff_angle = rotation_diff.log().norm();

    if (translation_diff < translation_threshold_ && rotation_diff_angle < rotation_threshold_) {
        return true;
    }

    return false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr filterInvalidPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            filtered_cloud->points.push_back(point);
        }
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;
    return filtered_cloud;
}

Sophus::SE3d LoopClosureDetector::refinePose(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                                             const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                                             const Sophus::SE3d& initial_guess) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source = filterInvalidPoints(source);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target = filterInvalidPoints(target);

    if (filtered_source->empty() || filtered_target->empty()) {
        return initial_guess;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(filtered_source);
    icp.setInputTarget(filtered_target);

    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, initial_guess.matrix().cast<float>());

    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    Eigen::Matrix4d transformation_double = transformation.cast<double>();

    Eigen::Matrix3d R = transformation_double.block<3, 3>(0, 0);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();

    if (R_ortho.determinant() < 0) {
        R_ortho.col(0) *= -1;
    }

    transformation_double.block<3, 3>(0, 0) = R_ortho;

    return Sophus::SE3d(transformation_double);
}

Sophus::SE3d LoopClosureDetector::globalRelocalization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    const auto& keyframes = keyframe_manager_.getKeyFrames();
    if (keyframes.empty()) {
        return Sophus::SE3d();
    }

    Eigen::MatrixXd current_sc = scan_context_.generate(cloud, Eigen::Vector3d::Zero());

    double best_similarity = -1.0;
    int best_keyframe_idx = -1;

    for (size_t i = 0; i < keyframes.size(); ++i) {
        const KeyFrame& keyframe = keyframes[i];
        double similarity = scan_context_.calculateSimilarity(current_sc, keyframe.scan_context_);
        if (similarity > best_similarity) {
            best_similarity = similarity;
            best_keyframe_idx = i;
        }
    }

    if (best_keyframe_idx == -1 || best_similarity < 0.7) {
        ROS_WARN("Global relocalization: best_similarity=%.3f < 0.7", best_similarity);
        return Sophus::SE3d();
    }

    const KeyFrame& best_keyframe = keyframes[best_keyframe_idx];

    Sophus::SE3d initial_guess = best_keyframe.pose_;
    Sophus::SE3d relative_transform = refinePose(cloud, best_keyframe.cloud_, initial_guess);

    Sophus::SE3d refined_pose = initial_guess * relative_transform;

    Eigen::Matrix3d R = refined_pose.so3().matrix();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();

    if (R_ortho.determinant() < 0) {
        R_ortho.col(0) *= -1;
    }

    Sophus::SE3d ortho_pose(R_ortho, refined_pose.translation());

    return ortho_pose;
}

void LoopClosureDetector::updateKeyFramePoses(const std::vector<KeyFrame>& updated_keyframes) {
    auto& keyframes = const_cast<std::deque<KeyFrame>&>(keyframe_manager_.getKeyFrames());

    for (const auto& updated_kf : updated_keyframes) {
        for (auto& kf : keyframes) {
            if (kf.id_ == updated_kf.id_) {
                kf.pose_ = updated_kf.pose_;
                break;
            }
        }
    }
}

// ========== LoopClosureNode 实现 ==========

LoopClosureNode::LoopClosureNode(const ros::NodeHandle& nh)
    : nh_(nh) {
    try {
        initializeParameters();
    } catch (const std::exception& e) {
        std::cerr << "LoopClosureNode init exception: " << e.what() << std::endl;
    }

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &LoopClosureNode::odomCallback, this);
    cloud_sub_ = nh_.subscribe(pointcloud_topic_, 10, &LoopClosureNode::pointCloudCallback, this);

    relocalize_srv_ = nh_.advertiseService("/loop_closure_node/relocalize", &LoopClosureNode::relocalizeService, this);
    relocalization_pub_ = nh_.advertise<nav_msgs::Odometry>(relocalization_topic_, 10);

    timer_ = nh_.createTimer(ros::Duration(5.0), &LoopClosureNode::timerCallback, this);

    last_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("LoopClosureNode initialized");
}

LoopClosureNode::LoopClosureNode(const std::string& config_file_path, const ros::NodeHandle& nh)
    : nh_(nh) {
    initializeParameters(config_file_path);

    odom_sub_ = nh_.subscribe(odom_topic_, 10, &LoopClosureNode::odomCallback, this);
    cloud_sub_ = nh_.subscribe(pointcloud_topic_, 10, &LoopClosureNode::pointCloudCallback, this);

    relocalize_srv_ = nh_.advertiseService("/loop_closure_node/relocalize", &LoopClosureNode::relocalizeService, this);
    relocalization_pub_ = nh_.advertise<nav_msgs::Odometry>(relocalization_topic_, 10);

    last_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("LoopClosureNode initialized (config: %s)", config_file_path.c_str());
}

LoopClosureNode::~LoopClosureNode() {
}

void LoopClosureNode::initializeParameters(const std::string& config_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_file_path);

        if (config["odom_topic"]) odom_topic_ = config["odom_topic"].as<std::string>();
        if (config["pointcloud_topic"]) pointcloud_topic_ = config["pointcloud_topic"].as<std::string>();
        if (config["loop_detection_interval"]) loop_detection_interval_ = config["loop_detection_interval"].as<int>();

        loop_closure_detector_.configureFromYaml(config_file_path);

        ROS_INFO("LoopClosureNode: odom=%s, cloud=%s, interval=%d",
                 odom_topic_.c_str(), pointcloud_topic_.c_str(), loop_detection_interval_);

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parse error: %s", e.what());
    }
}

void LoopClosureNode::initializeParameters() {
    std::string config_file_path = "/home/ydkj/lidarslam_ws/src/lidar_slam2/config/slam_params.yaml";
    initializeParameters(config_file_path);
}

void LoopClosureNode::timerCallback(const ros::TimerEvent&) {
    ROS_INFO("[Timer] keyframes=%zu, cloud=%zu, init=%d",
                loop_closure_detector_.getKeyFrames().size(),
                last_cloud_->size(),
                initialized_);
}

void LoopClosureNode::odomCallback(const nav_msgs::Odometry::ConstPtr msg) {
    if (!initialized_) {
        initialized_ = true;
    }

    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaterniond orientation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                   msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    last_pose_ = Sophus::SE3d(orientation, position);
    last_stamp_ = msg->header.stamp;
}

void LoopClosureNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr msg) {
    if (!initialized_) return;

    try {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        *last_cloud_ = *cloud;

        loop_closure_detector_.addKeyFrame(last_pose_, cloud, msg->header.stamp);

        const auto& keyframes = loop_closure_detector_.getKeyFrames();
        if (keyframes.size() > 0 && keyframes.size() % loop_detection_interval_ == 0) {
            processLoopClosure();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error processing pointCloud: %s", e.what());
    }
}

void LoopClosureNode::processLoopClosure() {
    LoopCandidate candidate = loop_closure_detector_.detectLoop();

    if (candidate.current_keyframe_id != -1 && candidate.candidate_keyframe_id != -1) {
        ROS_INFO("Loop found: %d <-> %d, similarity: %.3f",
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
        }
    }
}

bool LoopClosureNode::relocalizeService(std_srvs::Empty::Request& request,
                                      std_srvs::Empty::Response& response) {
    ROS_INFO("Received relocalization request!");

    if (last_cloud_->empty()) {
        ROS_WARN("No pointCloud data available");
        return true;
    }

    Sophus::SE3d relocalized_pose = loop_closure_detector_.globalRelocalization(last_cloud_);

    if (relocalized_pose.so3().matrix().allFinite()) {
        ROS_INFO("Global relocalization successful: (%.3f, %.3f, %.3f)",
                    relocalized_pose.translation().x(),
                    relocalized_pose.translation().y(),
                    relocalized_pose.translation().z());

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
    } else {
        ROS_WARN("Global relocalization failed");
    }
    return true;
}

// ========== 重定位相关实现 ==========

std::vector<RelocCandidate> LoopClosureDetector::findRelocalizationCandidates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    int top_k,
    double min_score,
    bool use_yaw_alignment,
    int yaw_search_sectors) {

    std::vector<RelocCandidate> candidates;

    const auto& keyframes = keyframe_manager_.getKeyFrames();
    if (!cloud || cloud->empty() || keyframes.empty()) {
        return candidates;
    }

    Eigen::MatrixXd current_sc = scan_context_.generate(cloud, Eigen::Vector3d::Zero());

    for (size_t i = 0; i < keyframes.size(); ++i) {
        const auto& kf = keyframes[i];

        if (kf.scan_context_.rows() == 0 || kf.scan_context_.cols() == 0) {
            continue;
        }

        double best_score = -1.0;
        int best_shift = 0;

        int min_shift = use_yaw_alignment ? -yaw_search_sectors : 0;
        int max_shift = use_yaw_alignment ? yaw_search_sectors : 0;

        for (int shift = min_shift; shift <= max_shift; ++shift) {
            double score = scan_context_.calculateSimilarityWithShift(
                current_sc, kf.scan_context_, shift);

            if (score > best_score) {
                best_score = score;
                best_shift = shift;
            }
        }

        if (best_score < min_score) {
            continue;
        }

        RelocCandidate c;
        c.keyframe_index = static_cast<int>(i);
        c.keyframe_id = kf.id_;
        c.score = best_score;
        c.yaw_shift = best_shift;
        c.yaw_rad = best_shift * 2.0 * M_PI / scan_context_.getNumSectors();

        // 天车约束：粗位姿直接来自 keyframe
        c.coarse_pose = kf.pose_;

        // 默认不应用大 yaw 修正
        if (use_yaw_alignment && yaw_search_sectors > 0) {
            Sophus::SE3d yaw_correction(
                Sophus::SO3d::rotZ(c.yaw_rad),
                Eigen::Vector3d::Zero());
            c.coarse_pose = kf.pose_ * yaw_correction;
        }

        candidates.push_back(c);
    }

    // 按 score 降序排序
    std::sort(candidates.begin(), candidates.end(),
              [](const RelocCandidate& a, const RelocCandidate& b) {
                  return a.score > b.score;
              });

    // 截取 top_k
    if (candidates.size() > static_cast<size_t>(top_k)) {
        candidates.resize(top_k);
    }

    return candidates;
}

bool LoopClosureDetector::loadOrBuildScanContextDatabase(const std::string& database_dir) {
    // 尝试加载已有的 ScanContext 数据库
    bool loaded = keyframe_manager_.loadScanContextDatabase(
        database_dir,
        scan_context_.getNumRings(),
        scan_context_.getNumSectors());

    if (loaded) {
        // 同步到 scan_context_list_
        const auto& keyframes = keyframe_manager_.getKeyFrames();
        scan_context_list_.clear();
        for (const auto& kf : keyframes) {
            scan_context_list_.push_back(kf.scan_context_);
        }
        return true;
    }

    ROS_WARN("[ScanContextDB] load failed, rebuild from keyframe clouds");

    // 从 keyframe PCD 重新生成
    auto& keyframes = keyframe_manager_.getKeyFramesNonConst();
    scan_context_list_.clear();

    for (auto& kf : keyframes) {
        if (kf.cloud_ && !kf.cloud_->empty()) {
            kf.scan_context_ = scan_context_.generate(kf.cloud_, Eigen::Vector3d::Zero());
            scan_context_list_.push_back(kf.scan_context_);
        }
    }

    // 保存到磁盘
    keyframe_manager_.saveScanContextDatabase(
        database_dir,
        scan_context_.getNumRings(),
        scan_context_.getNumSectors(),
        scan_context_.getMaxRange());

    return true;
}

} // namespace ndt_slam
