#include <filesystem>
#include "ndt_slam/keyframe_manager.hpp"
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
            if (kf.cloud_ && !kf.cloud_->empty()) {
                pcl::io::savePCDFileBinary(pcd_filename, *kf.cloud_);
            } else {
                ROS_DEBUG("[KeyFrame] Skip saving released keyframe cloud: id=%lu", kf.id_);
            }
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

// ========== ScanContext 数据库持久化 ==========

bool KeyFrameManager::saveScanContextDatabase(const std::string& session_dir, int num_rings, int num_sectors, double max_range) const {
    try {
        std::filesystem::path session_path(session_dir);

        // 保存 meta.yaml
        std::string meta_file = session_path / "scan_context_meta.yaml";
        std::ofstream ofs_meta(meta_file);
        if (!ofs_meta.is_open()) {
            ROS_ERROR("[ScanContextDB] Failed to open %s for writing", meta_file.c_str());
            return false;
        }
        ofs_meta << "version: 1\n";
        ofs_meta << "num_rings: " << num_rings << "\n";
        ofs_meta << "num_sectors: " << num_sectors << "\n";
        ofs_meta << "max_range: " << max_range << "\n";
        ofs_meta << "count: " << keyframes_.size() << "\n";
        ofs_meta.close();

        // 保存 poses.txt
        std::string poses_file = session_path / "scan_context_poses.txt";
        std::ofstream ofs_poses(poses_file);
        if (!ofs_poses.is_open()) {
            ROS_ERROR("[ScanContextDB] Failed to open %s for writing", poses_file.c_str());
            return false;
        }
        ofs_poses << std::fixed << std::setprecision(6);
        for (const auto& kf : keyframes_) {
            Eigen::Vector3d t = kf.pose_.translation();
            Eigen::Quaterniond q(kf.pose_.unit_quaternion());
            ofs_poses << kf.id_ << " "
                      << kf.stamp_.sec << "." << std::setw(9) << std::setfill('0') << kf.stamp_.nsec << " "
                      << t.x() << " " << t.y() << " " << t.z() << " "
                      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
        ofs_poses.close();

        // 保存 scan_contexts.bin
        std::string bin_file = session_path / "scan_contexts.bin";
        std::ofstream ofs_bin(bin_file, std::ios::binary);
        if (!ofs_bin.is_open()) {
            ROS_ERROR("[ScanContextDB] Failed to open %s for writing", bin_file.c_str());
            return false;
        }

        uint32_t magic = 0x53435458;  // "SCTX"
        uint32_t version = 1;
        uint32_t count = static_cast<uint32_t>(keyframes_.size());
        uint32_t rows = static_cast<uint32_t>(num_rings);
        uint32_t cols = static_cast<uint32_t>(num_sectors);

        ofs_bin.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
        ofs_bin.write(reinterpret_cast<const char*>(&version), sizeof(version));
        ofs_bin.write(reinterpret_cast<const char*>(&count), sizeof(count));
        ofs_bin.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
        ofs_bin.write(reinterpret_cast<const char*>(&cols), sizeof(cols));

        for (const auto& kf : keyframes_) {
            if (kf.scan_context_.rows() == rows && kf.scan_context_.cols() == cols) {
                ofs_bin.write(reinterpret_cast<const char*>(kf.scan_context_.data()),
                              rows * cols * sizeof(double));
            } else {
                // 写入全零
                std::vector<double> zeros(rows * cols, 0.0);
                ofs_bin.write(reinterpret_cast<const char*>(zeros.data()),
                              rows * cols * sizeof(double));
            }
        }
        ofs_bin.close();

        ROS_INFO("[ScanContextDB] Saved %zu keyframes to %s", keyframes_.size(), session_dir.c_str());
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("[ScanContextDB] Exception: %s", e.what());
        return false;
    }
}

bool KeyFrameManager::loadScanContextDatabase(const std::string& session_dir, int expected_rings, int expected_sectors) {
    try {
        std::filesystem::path session_path(session_dir);

        // 检查文件是否存在
        std::string bin_file = session_path / "scan_contexts.bin";
        if (!std::filesystem::exists(bin_file)) {
            ROS_WARN("[ScanContextDB] scan_contexts.bin not found in %s", session_dir.c_str());
            return false;
        }

        // 读取 bin 文件
        std::ifstream ifs_bin(bin_file, std::ios::binary);
        if (!ifs_bin.is_open()) {
            ROS_ERROR("[ScanContextDB] Failed to open %s", bin_file.c_str());
            return false;
        }

        uint32_t magic, version, count, rows, cols;
        ifs_bin.read(reinterpret_cast<char*>(&magic), sizeof(magic));
        ifs_bin.read(reinterpret_cast<char*>(&version), sizeof(version));
        ifs_bin.read(reinterpret_cast<char*>(&count), sizeof(count));
        ifs_bin.read(reinterpret_cast<char*>(&rows), sizeof(rows));
        ifs_bin.read(reinterpret_cast<char*>(&cols), sizeof(cols));

        if (magic != 0x53435458) {
            ROS_ERROR("[ScanContextDB] Invalid magic number");
            return false;
        }

        if (rows != static_cast<uint32_t>(expected_rings) || cols != static_cast<uint32_t>(expected_sectors)) {
            ROS_ERROR("[ScanContextDB] Dimension mismatch: expected %dx%d, got %ux%u",
                      expected_rings, expected_sectors, rows, cols);
            return false;
        }

        if (count != static_cast<uint32_t>(keyframes_.size())) {
            ROS_WARN("[ScanContextDB] Count mismatch: keyframes=%zu, scan_contexts=%u",
                     keyframes_.size(), count);
            // 继续，取较小值
        }

        size_t load_count = std::min(static_cast<size_t>(count), keyframes_.size());
        for (size_t i = 0; i < load_count; ++i) {
            keyframes_[i].scan_context_ = Eigen::MatrixXd(rows, cols);
            ifs_bin.read(reinterpret_cast<char*>(keyframes_[i].scan_context_.data()),
                         rows * cols * sizeof(double));
        }
        ifs_bin.close();

        ROS_INFO("[ScanContextDB] Loaded %zu scan contexts from %s", load_count, session_dir.c_str());
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("[ScanContextDB] Exception: %s", e.what());
        return false;
    }
}

} // namespace ndt_slam
