#include "lidar_slam2/LoopClosureDetector.hpp"
#include <pcl/registration/icp.h>
#include <yaml-cpp/yaml.h>

namespace lidar_slam2 {

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
        std::cerr << "YAML 解析错误 (LoopClosureDetector): " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "配置错误 (LoopClosureDetector): " << e.what() << std::endl;
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
        // 检查是否是关键帧
        if (keyframe_manager_.isKeyFrame(pose, stamp)) {
            // 添加关键帧
            keyframe_manager_.addKeyFrame(pose, cloud, stamp);
            
            // 生成ScanContext描述子
            const auto& keyframes = keyframe_manager_.getKeyFrames();
            if (keyframes.empty()) {
                return;
            }
            KeyFrame& last_keyframe = const_cast<KeyFrame&>(keyframes.back());
            last_keyframe.scan_context_ = scan_context_.generate(cloud, pose.translation());
            
            // 添加到ScanContext列表
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
    // 需要至少30个关键帧才开始检测闭环，避免初始化阶段误触发
    if (keyframes.size() < 30) return candidate;

    // 获取当前关键帧
    const KeyFrame& current_keyframe = keyframes.back();

    // 空间搜索：当前位置N米内的历史关键帧
    // 但要排除最近的关键帧（避免相邻关键帧被误识别为闭环）
    const int min_keyframe_gap = 25;  // 至少间隔25个关键帧（约12m运动）
    const double min_loop_distance = 5.0;  // 闭环候选至少距离当前位置5m
    std::vector<int> spatial_candidates;
    for (size_t i = 0; i < keyframes.size() - min_keyframe_gap; ++i) {
        const KeyFrame& kf = keyframes[i];
        double distance = (current_keyframe.pose_.translation() - kf.pose_.translation()).norm();
        // 必须在搜索半径内，且距离不能太近（太近说明还没绕回来）
        if (distance < spatial_search_radius_ && distance > min_loop_distance) {
            spatial_candidates.push_back(i);
        }
    }

    if (spatial_candidates.empty()) return candidate;
    
    // 使用ScanContext找最相似的候选
    std::vector<Eigen::MatrixXd> candidate_scs;
    for (int i : spatial_candidates) {
        candidate_scs.push_back(keyframes[i].scan_context_);
    }
    
    int best_candidate_idx = scan_context_.findBestMatch(current_keyframe.scan_context_, candidate_scs);
    if (best_candidate_idx == -1) return candidate;
    
    int actual_idx = spatial_candidates[best_candidate_idx];
    const KeyFrame& candidate_keyframe = keyframes[actual_idx];
    
    // 计算相似度
    double similarity = scan_context_.calculateSimilarity(current_keyframe.scan_context_, candidate_keyframe.scan_context_);
    if (similarity < similarity_threshold_) return candidate;
    
    // 使用ICP进行精配准
    Sophus::SE3d initial_guess = candidate_keyframe.pose_.inverse() * current_keyframe.pose_;
    Sophus::SE3d refined_pose = refinePose(current_keyframe.cloud_, candidate_keyframe.cloud_, initial_guess);
    
    // 一致性检查
    Sophus::SE3d odometry_pose = candidate_keyframe.pose_.inverse() * current_keyframe.pose_;
    if (!checkConsistency(refined_pose, odometry_pose)) return candidate;
    
    // 填充回环候选
    candidate.current_keyframe_id = current_keyframe.id_;
    candidate.candidate_keyframe_id = candidate_keyframe.id_;
    candidate.relative_pose = refined_pose;
    candidate.similarity = similarity;
    
    return candidate;
}

bool LoopClosureDetector::checkConsistency(const Sophus::SE3d& loop_pose, const Sophus::SE3d& odometry_pose) {
    // 计算平移差
    double translation_diff = (loop_pose.translation() - odometry_pose.translation()).norm();
    
    // 计算旋转差
    Sophus::SO3d rotation_diff = odometry_pose.so3().inverse() * loop_pose.so3();
    double rotation_diff_angle = rotation_diff.log().norm();
    
    // 检查是否在阈值范围内
    if (translation_diff < translation_threshold_ && rotation_diff_angle < rotation_threshold_) {
        return true;
    }
    
    return false;
}

// 过滤无效点（NaN或Inf）
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
    // 过滤无效点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source = filterInvalidPoints(source);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target = filterInvalidPoints(target);
    
    // 检查过滤后点云是否为空
    if (filtered_source->empty() || filtered_target->empty()) {
        return initial_guess;
    }
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(filtered_source);
    icp.setInputTarget(filtered_target);
    
    // 设置ICP参数
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    
    // 执行ICP
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, initial_guess.matrix().cast<float>());
    
    // 获取变换矩阵
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    Eigen::Matrix4d transformation_double = transformation.cast<double>();
    
    // 提取旋转矩阵
    Eigen::Matrix3d R = transformation_double.block<3, 3>(0, 0);
    
    // 对旋转矩阵进行正交化处理
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();
    
    // 确保旋转矩阵的行列式为1（右手坐标系）
    if (R_ortho.determinant() < 0) {
        R_ortho.col(0) *= -1;
    }
    
    // 更新变换矩阵
    transformation_double.block<3, 3>(0, 0) = R_ortho;
    
    return Sophus::SE3d(transformation_double);
}

Sophus::SE3d LoopClosureDetector::globalRelocalization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // 获取所有关键帧
    const auto& keyframes = keyframe_manager_.getKeyFrames();
    if (keyframes.empty()) {
        return Sophus::SE3d();
    }
    
    // 生成当前点云的ScanContext描述子
    Eigen::MatrixXd current_sc = scan_context_.generate(cloud, Eigen::Vector3d::Zero());
    
    // 遍历所有关键帧，找到最相似的
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
    
    if (best_keyframe_idx == -1 || best_similarity < 0.7) { // 相似度阈值
        ROS_WARN("Global relocalization: best_similarity=%.3f < 0.7, keyframe_count=%zu", 
                 best_similarity, keyframes.size());
        return Sophus::SE3d();
    }
    
    // 获取最相似的关键帧
    const KeyFrame& best_keyframe = keyframes[best_keyframe_idx];
    ROS_INFO("Global relocalization: best_keyframe_idx=%d, similarity=%.3f, keyframe_pose=(%.3f, %.3f, %.3f)",
             best_keyframe_idx, best_similarity,
             best_keyframe.pose_.translation().x(),
             best_keyframe.pose_.translation().y(),
             best_keyframe.pose_.translation().z());
    
    // 使用ICP进行精配准
    Sophus::SE3d initial_guess = best_keyframe.pose_;
    Sophus::SE3d relative_transform = refinePose(cloud, best_keyframe.cloud_, initial_guess);
    
    ROS_INFO("Global relocalization: relative_transform=(%.3f, %.3f, %.3f)",
             relative_transform.translation().x(),
             relative_transform.translation().y(),
             relative_transform.translation().z());
    
    // ICP返回的是相对变换（从当前帧到关键帧），需要结合关键帧的绝对位姿
    Sophus::SE3d refined_pose = initial_guess * relative_transform;
    
    ROS_INFO("Global relocalization: refined_pose=(%.3f, %.3f, %.3f)",
             refined_pose.translation().x(),
             refined_pose.translation().y(),
             refined_pose.translation().z());
    
    // 确保旋转矩阵正交
    Eigen::Matrix3d R = refined_pose.so3().matrix();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();
    
    // 确保旋转矩阵的行列式为1（右手坐标系）
    if (R_ortho.determinant() < 0) {
        R_ortho.col(0) *= -1;
    }
    
    // 创建正交化后的位姿
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
    
    ROS_INFO("Updated %zu keyframe poses", updated_keyframes.size());
}

} // namespace lidar_slam2
