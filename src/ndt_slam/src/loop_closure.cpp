#include "ndt_slam/loop_closure.hpp"
#include "ndt_slam/rigid_transform_conversion.hpp"
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <utility>

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
        ROS_ERROR_THROTTLE(1.0, "[KeyFrame] add failed: %s", e.what());
    }

    return sc;
}

double ScanContext::calculateSimilarity(
    const Eigen::MatrixXd& sc1, const Eigen::MatrixXd& sc2) const {
    return calculateSimilarityWithShift(sc1, sc2).first;
}

std::pair<double, int> ScanContext::calculateSimilarityWithShift(
    const Eigen::MatrixXd& query, const Eigen::MatrixXd& candidate) const {
    if (query.rows() != candidate.rows() ||
        query.cols() != candidate.cols() || query.size() == 0) {
        return {0.0, 0};
    }
    const double query_norm = query.norm();
    const double candidate_norm = candidate.norm();
    if (query_norm < 1.0e-12 || candidate_norm < 1.0e-12) {
        return {0.0, 0};
    }

    double best_similarity = -1.0;
    int best_shift = 0;
    for (int shift = 0; shift < query.cols(); ++shift) {
        double dot = 0.0;
        for (int sector = 0; sector < query.cols(); ++sector) {
            const int candidate_sector = (sector + shift) % query.cols();
            dot += query.col(sector).dot(candidate.col(candidate_sector));
        }
        const double similarity = dot / (query_norm * candidate_norm);
        if (similarity > best_similarity) {
            best_similarity = similarity;
            best_shift = shift;
        }
    }
    return {std::clamp(best_similarity, 0.0, 1.0), best_shift};
}

int ScanContext::findBestMatch(
    const Eigen::MatrixXd& current_sc,
    const std::vector<Eigen::MatrixXd>& sc_list) const {
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

    const SafeSE3Result converted =
        makeSafeSE3FromMatrix(pose.matrix());
    if (!converted.valid) {
        ROS_ERROR("[SO3Guard] rejected optimized pose keyframe=%d reason=%s",
                  keyframe_id, converted.diagnostics.reason.c_str());
        return Sophus::SE3d();
    }
    return converted.pose;
}

void PoseGraphOptimizer::updateKeyFramePoses(std::vector<KeyFrame>& keyframes) {
    for (auto& keyframe : keyframes) {
        const auto vertex_it = vertices_.find(keyframe.id_);
        if (vertex_it == vertices_.end()) continue;
        const SafeSE3Result converted = makeSafeSE3FromMatrix(
            vertex_it->second->estimate().matrix());
        if (!converted.valid) {
            ROS_ERROR(
                "[SO3Guard] preserving keyframe=%llu after invalid optimized "
                "pose reason=%s",
                static_cast<unsigned long long>(keyframe.id_),
                converted.diagnostics.reason.c_str());
            continue;
        }
        keyframe.pose_ = converted.pose;
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

            loop_icp_max_correspondence_m_ = std::clamp(sc["loop_icp_max_correspondence_m"].as<double>(1.0), 0.10, 5.0);
            loop_icp_max_fitness_ = std::clamp(sc["loop_icp_max_fitness"].as<double>(0.50), 0.01, 5.0);
            loop_icp_max_correction_translation_m_ = std::clamp(sc["loop_icp_max_correction_translation_m"].as<double>(1.0), 0.05, 5.0);
            loop_icp_max_correction_rotation_rad_ =
                std::clamp(sc["loop_icp_max_correction_yaw_deg"].as<double>(15.0), 1.0, 90.0) * M_PI / 180.0;
            global_icp_max_correspondence_m_ = std::clamp(sc["global_icp_max_correspondence_m"].as<double>(1.5), 0.10, 5.0);
            global_icp_max_fitness_ = std::clamp(sc["global_icp_max_fitness"].as<double>(0.80), 0.01, 5.0);
            global_icp_max_correction_translation_m_ = std::clamp(sc["global_icp_max_correction_translation_m"].as<double>(1.5), 0.05, 5.0);
            global_icp_max_correction_rotation_rad_ =
                std::clamp(sc["global_icp_max_correction_yaw_deg"].as<double>(20.0), 1.0, 90.0) * M_PI / 180.0;
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

bool LoopClosureDetector::addKeyFrame(
    const Sophus::SE3d& pose,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const ros::Time& stamp) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    try {
        if (keyframe_manager_.isKeyFrame(pose, stamp)) {
            keyframe_manager_.addKeyFrame(pose, cloud, stamp);

            const auto& keyframes = keyframe_manager_.getKeyFrames();
            if (keyframes.empty()) {
                return false;
            }
            KeyFrame& last_keyframe = const_cast<KeyFrame&>(keyframes.back());
            // Keyframe clouds are stored in base_link coordinates. Their scan
            // contexts must therefore use the sensor origin, exactly like a
            // live relocalization query.
            last_keyframe.scan_context_ =
                scan_context_.generate(cloud, Eigen::Vector3d::Zero());

            scan_context_list_.push_back(last_keyframe.scan_context_);
            return true;
        }
    } catch (const std::exception& e) {
        // 捕获异常，避免程序崩溃
    }
    return false;
}

LoopCandidate LoopClosureDetector::detectLoop() {
    return detectLoop(getKeyFramesSnapshot());
}

LoopCandidate LoopClosureDetector::detectLoop(
    const std::deque<KeyFrame>& keyframes) const {
    LoopCandidate candidate;
    candidate.current_keyframe_id = -1;
    candidate.candidate_keyframe_id = -1;

    if (keyframes.size() < 30) return candidate;

    const KeyFrame& current_keyframe = keyframes.back();
    if (!current_keyframe.cloud_ || current_keyframe.cloud_->empty()) {
        return candidate;
    }

    const int min_keyframe_gap = 25;
    const double min_loop_distance = 5.0;
    std::vector<int> spatial_candidates;
    for (size_t i = 0; i < keyframes.size() - min_keyframe_gap; ++i) {
        const KeyFrame& kf = keyframes[i];
        if (!kf.cloud_ || kf.cloud_->empty()) continue;
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
    const auto refined_pose = refinePose(
        current_keyframe.cloud_, candidate_keyframe.cloud_, initial_guess,
        false);
    if (!refined_pose) return candidate;

    Sophus::SE3d odometry_pose = candidate_keyframe.pose_.inverse() * current_keyframe.pose_;
    if (!checkConsistency(*refined_pose, odometry_pose)) return candidate;

    candidate.current_keyframe_id = current_keyframe.id_;
    candidate.candidate_keyframe_id = candidate_keyframe.id_;
    candidate.relative_pose = *refined_pose;
    candidate.similarity = similarity;

    return candidate;
}

bool LoopClosureDetector::checkConsistency(
    const Sophus::SE3d& loop_pose,
    const Sophus::SE3d& odometry_pose) const {
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

std::optional<Sophus::SE3d> LoopClosureDetector::refinePose(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    const Sophus::SE3d& initial_guess,
    bool global_relocalization) const {
    if (!source || !target || source->empty() || target->empty()) {
        return std::nullopt;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source = filterInvalidPoints(source);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target = filterInvalidPoints(target);

    if (filtered_source->empty() || filtered_target->empty()) {
        return std::nullopt;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(filtered_source);
    icp.setInputTarget(filtered_target);

    icp.setMaximumIterations(50);
    const double maximum_correspondence = global_relocalization
        ? global_icp_max_correspondence_m_
        : loop_icp_max_correspondence_m_;
    const double maximum_fitness = global_relocalization
        ? global_icp_max_fitness_ : loop_icp_max_fitness_;
    const double maximum_translation = global_relocalization
        ? global_icp_max_correction_translation_m_
        : loop_icp_max_correction_translation_m_;
    const double maximum_rotation = global_relocalization
        ? global_icp_max_correction_rotation_rad_
        : loop_icp_max_correction_rotation_rad_;
    icp.setMaxCorrespondenceDistance(maximum_correspondence);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, initial_guess.matrix().cast<float>());

    if (!icp.hasConverged()) return std::nullopt;
    const double fitness = icp.getFitnessScore();
    if (!std::isfinite(fitness) || fitness > maximum_fitness) {
        return std::nullopt;
    }

    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    if (!transformation.allFinite()) return std::nullopt;
    const SafeSE3Result converted =
        makeSafeSE3FromMatrix(transformation.cast<double>());
    if (!converted.valid) {
        ROS_WARN("[SO3Guard] rejected loop ICP result reason=%s",
                 converted.diagnostics.reason.c_str());
        return std::nullopt;
    }
    const Sophus::SE3d refined = converted.pose;
    const Sophus::SE3d correction = initial_guess.inverse() * refined;
    if (correction.translation().norm() > maximum_translation ||
        correction.so3().log().norm() > maximum_rotation) {
        return std::nullopt;
    }
    return refined;
}

std::optional<Sophus::SE3d> LoopClosureDetector::globalRelocalization(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    const auto hints = findRelocalizationHints(cloud, 1, 0.70);
    if (hints.empty()) return std::nullopt;

    const auto keyframes = getKeyFramesSnapshot();
    const auto it = std::find_if(
        keyframes.begin(), keyframes.end(), [&hints](const KeyFrame& keyframe) {
            return keyframe.id_ == hints.front().keyframe_id;
        });
    if (it == keyframes.end() || !it->cloud_ || it->cloud_->empty()) {
        return std::nullopt;
    }

    // Keyframe clouds are base-frame observations. Transform the candidate
    // target into map coordinates before using an absolute map pose as ICP's
    // initial guess. The old implementation mixed these frames and then
    // multiplied the keyframe pose twice.
    auto target_map = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*it->cloud_, *target_map,
                             hints.front().pose.matrix().cast<float>());
    const Eigen::Matrix3d hint_rotation = hints.front().pose.so3().matrix();
    const double hint_yaw = std::atan2(hint_rotation(1, 0),
                                       hint_rotation(0, 0));
    const Sophus::SE3d initial_guess(
        Eigen::AngleAxisd(hint_yaw + hints.front().yaw_offset_rad,
                          Eigen::Vector3d::UnitZ()).toRotationMatrix(),
        hints.front().pose.translation());
    return refinePose(cloud, target_map, initial_guess, true);
}

std::vector<RelocalizationHint> LoopClosureDetector::findRelocalizationHints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::size_t max_candidates, double min_similarity) {
    std::vector<RelocalizationHint> hints;
    if (!cloud || cloud->empty() || max_candidates == 0U) return hints;

    const auto keyframes = getKeyFramesSnapshot();
    if (keyframes.empty()) return hints;
    const Eigen::MatrixXd query =
        scan_context_.generate(cloud, Eigen::Vector3d::Zero());
    if (query.squaredNorm() < 1.0e-12) return hints;

    hints.reserve(keyframes.size());
    for (const auto& keyframe : keyframes) {
        if (keyframe.scan_context_.rows() != query.rows() ||
            keyframe.scan_context_.cols() != query.cols()) continue;
        if (keyframe.scan_context_.squaredNorm() < 1.0e-12) continue;
        const auto similarity_shift = scan_context_.calculateSimilarityWithShift(
            query, keyframe.scan_context_);
        const double similarity = similarity_shift.first;
        if (!std::isfinite(similarity) || similarity < min_similarity) continue;
        int signed_shift = similarity_shift.second;
        if (signed_shift > query.cols() / 2) signed_shift -= query.cols();
        const double yaw_offset = static_cast<double>(signed_shift) *
            2.0 * M_PI / static_cast<double>(query.cols());
        hints.push_back(
            {keyframe.pose_, keyframe.id_, similarity, yaw_offset});
    }
    std::stable_sort(hints.begin(), hints.end(),
        [](const RelocalizationHint& lhs, const RelocalizationHint& rhs) {
            return lhs.similarity > rhs.similarity;
        });
    if (hints.size() > max_candidates) hints.resize(max_candidates);
    return hints;
}

void LoopClosureDetector::rebuildScanContexts() {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    auto& keyframes = const_cast<std::deque<KeyFrame>&>(
        keyframe_manager_.getKeyFrames());
    scan_context_list_.clear();
    scan_context_list_.reserve(keyframes.size());
    for (auto& keyframe : keyframes) {
        keyframe.scan_context_ = scan_context_.generate(
            keyframe.cloud_, Eigen::Vector3d::Zero());
        scan_context_list_.push_back(keyframe.scan_context_);
    }
}

void LoopClosureDetector::updateKeyFramePoses(const std::vector<KeyFrame>& updated_keyframes) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
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

// ========== Thread-safe keyframe snapshot and commit APIs ==========

std::deque<KeyFrame> LoopClosureDetector::getKeyFramesSnapshot() const {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    return keyframe_manager_.getKeyFrames();
}

std::size_t LoopClosureDetector::getKeyFrameCount() const {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    return keyframe_manager_.getKeyFrameCount();
}

void LoopClosureDetector::resetTemporalGateForSourceEpoch(
    const ros::Time& new_epoch_stamp) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    keyframe_manager_.resetTemporalGateForSourceEpoch(new_epoch_stamp);
}

void LoopClosureDetector::clear() {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    keyframe_manager_.clear();
    scan_context_list_.clear();
}

bool LoopClosureDetector::saveKeyFrameDatabase(
    const std::string& session_dir) const {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    return keyframe_manager_.saveKeyFrameDatabase(session_dir);
}

bool LoopClosureDetector::loadKeyFrameDatabase(
    const std::string& session_dir) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    if (!keyframe_manager_.loadKeyFrameDatabase(session_dir)) return false;
    auto& keyframes = keyframe_manager_.getKeyFramesNonConst();
    scan_context_list_.clear();
    scan_context_list_.reserve(keyframes.size());
    for (auto& keyframe : keyframes) {
        if (!keyframe.cloud_ || keyframe.cloud_->empty()) continue;
        keyframe.scan_context_ = scan_context_.generate(
            keyframe.cloud_, Eigen::Vector3d::Zero());
        scan_context_list_.push_back(keyframe.scan_context_);
    }
    return true;
}

void LoopClosureDetector::installKeyFrameDatabase(
    std::deque<KeyFrame> keyframes) {
    PreparedKeyFrameDatabase prepared;
    std::string reason;
    if (!prepareKeyFrameDatabase(
            std::move(keyframes), &prepared, &reason)) {
        ROS_ERROR("Prepared keyframe install rejected: %s", reason.c_str());
        return;
    }
    installPreparedKeyFrameDatabase(std::move(prepared));
}

bool LoopClosureDetector::prepareKeyFrameDatabase(
    std::deque<KeyFrame> keyframes,
    PreparedKeyFrameDatabase* prepared,
    std::string* reason) const {
    if (!prepared) {
        if (reason) *reason = "prepared_output_missing";
        return false;
    }
    *prepared = PreparedKeyFrameDatabase{};
    ScanContext generator;
    generator.configure(
        scan_context_.getNumRings(), scan_context_.getNumSectors(),
        scan_context_.getMaxRange());
    prepared->scan_contexts.reserve(keyframes.size());
    for (auto& keyframe : keyframes) {
        if (!keyframe.cloud_ || keyframe.cloud_->empty()) {
            if (reason) *reason = "keyframe_cloud_missing";
            return false;
        }
        keyframe.scan_context_ = generator.generate(
            keyframe.cloud_, Eigen::Vector3d::Zero());
        if (keyframe.scan_context_.size() == 0) {
            if (reason) *reason = "keyframe_scan_context_invalid";
            return false;
        }
        prepared->scan_contexts.push_back(keyframe.scan_context_);
    }
    prepared->keyframes = std::move(keyframes);
    prepared->valid = true;
    if (reason) *reason = "keyframe_database_prepared";
    return true;
}

void LoopClosureDetector::installPreparedKeyFrameDatabase(
    PreparedKeyFrameDatabase&& prepared) noexcept {
    if (!prepared.valid) return;
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    keyframe_manager_.replaceKeyFrames(std::move(prepared.keyframes));
    scan_context_list_.swap(prepared.scan_contexts);
}

void LoopClosureDetector::applyKeyFrameMetrics(
    const std::vector<std::pair<std::uint64_t, KeyFrameMetrics>>& metrics) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    std::map<std::uint64_t, KeyFrameMetrics> by_id(
        metrics.begin(), metrics.end());
    for (auto& keyframe : keyframe_manager_.getKeyFramesNonConst()) {
        const auto found = by_id.find(keyframe.id_);
        if (found != by_id.end()) keyframe.metrics_ = found->second;
    }
}

bool LoopClosureDetector::setLastKeyFrameLayers(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_raw,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_filtered,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_points) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    auto& keyframes = keyframe_manager_.getKeyFramesNonConst();
    if (keyframes.empty()) return false;
    auto& keyframe = keyframes.back();
    keyframe.objects_raw = objects_raw;
    keyframe.objects_filtered = objects_filtered;
    keyframe.ground_points = ground_points;
    keyframe.dirty_dynamic = false;
    return true;
}

std::size_t LoopClosureDetector::releaseCloudsBeforeActiveWindow(
    std::size_t max_active) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    auto& keyframes = keyframe_manager_.getKeyFramesNonConst();
    if (keyframes.size() <= max_active) return 0U;
    std::size_t released = 0U;
    const std::size_t end = keyframes.size() - max_active;
    for (std::size_t i = 0; i < end; ++i) {
        if (keyframes[i].cloud_ && !keyframes[i].cloud_->empty()) {
            keyframes[i].cloud_.reset();
            ++released;
        }
    }
    return released;
}

void LoopClosureDetector::applyOptimizedPoses(
    const std::vector<KeyFrame>& optimized,
    std::uint64_t snapshot_last_id,
    const Sophus::SE3d& correction_for_newer_frames) {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    auto& keyframes = keyframe_manager_.getKeyFramesNonConst();
    std::map<std::uint64_t, Sophus::SE3d> optimized_by_id;
    for (const auto& keyframe : optimized) {
        optimized_by_id.emplace(keyframe.id_, keyframe.pose_);
    }
    for (auto& keyframe : keyframes) {
        const auto found = optimized_by_id.find(keyframe.id_);
        if (found != optimized_by_id.end()) {
            keyframe.pose_ = found->second;
            keyframe.pose_refined_ = found->second;
            keyframe.has_refined_pose_ = true;
            keyframe.dirty_pose = false;
        } else if (keyframe.id_ > snapshot_last_id) {
            keyframe.pose_ = correction_for_newer_frames * keyframe.pose_;
            if (keyframe.has_refined_pose_) {
                keyframe.pose_refined_ =
                    correction_for_newer_frames * keyframe.pose_refined_;
            }
            keyframe.dirty_pose = false;
        }
    }
}

// ========== LoopClosureNode implementation ==========

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
                loop_closure_detector_.getKeyFrameCount(),
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
    if (!orientation.coeffs().allFinite() || !position.allFinite() ||
        !std::isfinite(orientation.norm()) ||
        orientation.norm() <= 1.0e-12) {
        ROS_WARN_THROTTLE(1.0, "[SO3Guard] rejected invalid odometry pose");
        return;
    }
    orientation.normalize();
    const SafeSE3Result converted = makeSafeSE3(
        orientation.toRotationMatrix(), position);
    if (!converted.valid) {
        ROS_WARN_THROTTLE(
            1.0, "[SO3Guard] rejected odometry pose reason=%s",
            converted.diagnostics.reason.c_str());
        return;
    }
    last_pose_ = converted.pose;
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

        const auto keyframes = loop_closure_detector_.getKeyFramesSnapshot();
        if (!keyframes.empty() &&
            keyframes.size() % loop_detection_interval_ == 0) {
            processLoopClosure();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error processing pointCloud: %s", e.what());
    }
}

void LoopClosureNode::processLoopClosure() {
    LoopCandidate candidate = loop_closure_detector_.detectLoop();

    if (candidate.current_keyframe_id != -1 &&
        candidate.candidate_keyframe_id != -1) {
        ROS_INFO("Loop found: %d <-> %d, similarity: %.3f",
                 candidate.current_keyframe_id,
                 candidate.candidate_keyframe_id,
                 candidate.similarity);

        const auto keyframes = loop_closure_detector_.getKeyFramesSnapshot();
        for (const auto& keyframe : keyframes) {
            pose_graph_optimizer_.addKeyFrame(keyframe);
        }

        for (std::size_t i = 0; i + 1 < keyframes.size(); ++i) {
            const KeyFrame& first = keyframes[i];
            const KeyFrame& second = keyframes[i + 1];
            const Sophus::SE3d relative_pose =
                first.pose_.inverse() * second.pose_;
            const Eigen::Matrix<double, 6, 6> information =
                Eigen::Matrix<double, 6, 6>::Identity();
            pose_graph_optimizer_.addOdometryEdge(
                first.id_, second.id_, relative_pose, information);
        }

        const Eigen::Matrix<double, 6, 6> loop_information =
            Eigen::Matrix<double, 6, 6>::Identity();
        pose_graph_optimizer_.addLoopEdge(
            candidate.candidate_keyframe_id,
            candidate.current_keyframe_id,
            candidate.relative_pose,
            loop_information);

        if (pose_graph_optimizer_.optimize(10)) {
            ROS_INFO("Pose graph optimization successful");
            std::vector<KeyFrame> updated_keyframes(
                keyframes.begin(), keyframes.end());
            pose_graph_optimizer_.updateKeyFramePoses(updated_keyframes);
        }
    }
}

bool LoopClosureNode::relocalizeService(
    std_srvs::Empty::Request& request,
    std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    ROS_INFO("Received relocalization request!");

    if (last_cloud_->empty()) {
        ROS_WARN("No pointCloud data available");
        return false;
    }

    const auto relocalized_pose =
        loop_closure_detector_.globalRelocalization(last_cloud_);
    if (!relocalized_pose) {
        ROS_WARN("Global relocalization failed");
        return false;
    }

    ROS_INFO("Global relocalization successful: (%.3f, %.3f, %.3f)",
             relocalized_pose->translation().x(),
             relocalized_pose->translation().y(),
             relocalized_pose->translation().z());
    const Sophus::SE3d& pose = *relocalized_pose;
    relocalized_pose_ = pose;

    nav_msgs::Odometry relocalization_msg;
    relocalization_msg.header.stamp = ros::Time::now();
    relocalization_msg.header.frame_id = "odom";
    relocalization_msg.child_frame_id = "base_link";
    relocalization_msg.pose.pose.position.x =
        pose.translation().x();
    relocalization_msg.pose.pose.position.y =
        pose.translation().y();
    relocalization_msg.pose.pose.position.z =
        pose.translation().z();

    const Eigen::Quaterniond quaternion =
        pose.so3().unit_quaternion();
    relocalization_msg.pose.pose.orientation.x = quaternion.x();
    relocalization_msg.pose.pose.orientation.y = quaternion.y();
    relocalization_msg.pose.pose.orientation.z = quaternion.z();
    relocalization_msg.pose.pose.orientation.w = quaternion.w();
    for (int i = 0; i < 6; ++i) {
        relocalization_msg.pose.covariance[i * 6 + i] = 0.1;
    }
    relocalization_pub_.publish(relocalization_msg);
    return true;
}

}  // namespace ndt_slam
