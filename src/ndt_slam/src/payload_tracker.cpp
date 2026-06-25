#include <ndt_slam/payload_tracker.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

namespace ndt_slam {

void PayloadTrackManager::configure(const PayloadTrackerConfig& config) {
    config_ = config;
}

void PayloadTrackManager::configureFromYaml(const std::string& config_file) {
    try {
        YAML::Node yaml = YAML::LoadFile(config_file);
        auto pt = yaml["payload_tracker"];
        if (pt) {
            config_.enabled = pt["enabled"].as<bool>(true);
            config_.bev_resolution = pt["bev_resolution"].as<double>(0.25);
            config_.min_cluster_points = pt["min_cluster_points"].as<int>(30);
            config_.min_cluster_area = pt["min_cluster_area"].as<double>(0.20);
            config_.cluster_tolerance = pt["cluster_tolerance"].as<double>(0.5);
            config_.max_match_distance = pt["max_match_distance"].as<double>(0.8);
            config_.max_match_bbox_ratio = pt["max_match_bbox_ratio"].as<double>(0.5);
            config_.window_sec = pt["window_sec"].as<double>(3.0);
            config_.motion_confirm_frames = pt["motion_confirm_frames"].as<int>(3);
            config_.displacement_thresh = pt["displacement_thresh"].as<double>(0.25);
            config_.velocity_thresh = pt["velocity_thresh"].as<double>(0.05);
            config_.direction_consistency_thresh = pt["direction_consistency_thresh"].as<double>(0.65);
            config_.base_stability_std_thresh = pt["base_stability_std_thresh"].as<double>(0.35);
            config_.max_missed_frames = pt["max_missed_frames"].as<int>(5);
        }
    } catch (const std::exception& e) {
        ROS_WARN("[PayloadTracker] Failed to load config: %s, using defaults", e.what());
    }
}

void PayloadTrackManager::reset() {
    tracks_.clear();
    next_track_id_ = 0;
}

// ========== Cluster 提取 ==========

std::vector<ClusterInfo> PayloadTrackManager::extractClusters(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::map<CellKey, float>& ground_model) {

    std::vector<ClusterInfo> clusters;
    if (!cloud || cloud->size() < (size_t)config_.min_cluster_points) return clusters;

    // KD-Tree 用于欧式聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(config_.cluster_tolerance);
    ec.setMinClusterSize(config_.min_cluster_points);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto& indices : cluster_indices) {
        ClusterInfo info;
        info.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        info.centroid = Eigen::Vector3f::Zero();
        info.bbox_min = Eigen::Vector3f(1e9, 1e9, 1e9);
        info.bbox_max = Eigen::Vector3f(-1e9, -1e9, -1e9);
        info.hag_min = 1e9f;
        info.hag_max = -1e9f;

        for (int idx : indices.indices) {
            const auto& p = cloud->points[idx];
            info.cloud->push_back(p);
            info.centroid += Eigen::Vector3f(p.x, p.y, p.z);
            info.bbox_min[0] = std::min(info.bbox_min[0], p.x);
            info.bbox_min[1] = std::min(info.bbox_min[1], p.y);
            info.bbox_min[2] = std::min(info.bbox_min[2], p.z);
            info.bbox_max[0] = std::max(info.bbox_max[0], p.x);
            info.bbox_max[1] = std::max(info.bbox_max[1], p.y);
            info.bbox_max[2] = std::max(info.bbox_max[2], p.z);

            // 计算 HAG（height above ground）
            float ground_z = 0;
            if (!ground_model.empty()) {
                CellKey ck{(int)std::floor(p.x / 1.5f), (int)std::floor(p.y / 1.5f)};
                auto it = ground_model.find(ck);
                if (it != ground_model.end()) ground_z = it->second;
            }
            float hag = p.z - ground_z;
            info.hag_min = std::min(info.hag_min, hag);
            info.hag_max = std::max(info.hag_max, hag);
        }

        info.centroid /= info.cloud->size();
        info.point_count = info.cloud->size();

        // 过滤太小的 cluster
        float area = (info.bbox_max[0] - info.bbox_min[0]) *
                     (info.bbox_max[1] - info.bbox_min[1]);
        if (area >= config_.min_cluster_area) {
            clusters.push_back(info);
        }
    }

    return clusters;
}

// ========== 匹配 ==========

MatchResult PayloadTrackManager::matchClusters(
    const std::vector<ClusterInfo>& clusters) {

    MatchResult result;
    std::vector<bool> track_matched(tracks_.size(), false);
    std::vector<bool> cluster_matched(clusters.size(), false);

    // 贪心匹配：对每个活跃 track，找最近的未匹配 cluster
    for (size_t ti = 0; ti < tracks_.size(); ti++) {
        if (tracks_[ti].state == TrackState::EXPIRED) continue;

        double best_dist = config_.max_match_distance;
        int best_ci = -1;

        for (size_t ci = 0; ci < clusters.size(); ci++) {
            if (cluster_matched[ci]) continue;

            // 距离约束
            double dist = (tracks_[ti].centroid_map - clusters[ci].centroid).norm();
            if (dist > config_.max_match_distance) continue;

            // 包围盒尺寸约束
            Eigen::Vector3f track_size = tracks_[ti].bbox_max_map - tracks_[ti].bbox_min_map;
            Eigen::Vector3f cluster_size = clusters[ci].bbox_max - clusters[ci].bbox_min;
            float size_diff = (track_size - cluster_size).cwiseAbs().maxCoeff();
            float size_max = track_size.cwiseMax(cluster_size).maxCoeff();
            if (size_max > 0.1f && size_diff / size_max > config_.max_match_bbox_ratio)
                continue;

            if (dist < best_dist) {
                best_dist = dist;
                best_ci = ci;
            }
        }

        if (best_ci >= 0) {
            result.matches.push_back({(int)ti, best_ci});
            track_matched[ti] = true;
            cluster_matched[best_ci] = true;
        }
    }

    // 收集未匹配
    for (size_t ti = 0; ti < tracks_.size(); ti++) {
        if (!track_matched[ti]) result.unmatched_tracks.push_back((int)ti);
    }
    for (size_t ci = 0; ci < clusters.size(); ci++) {
        if (!cluster_matched[ci]) result.unmatched_clusters.push_back((int)ci);
    }

    return result;
}

// ========== 运动统计（双坐标系版本）==========

float PayloadTrackManager::computeBaseStability(const std::deque<TrackPoint>& trajectory_base) {
    if (trajectory_base.size() < 3) return 0;

    // 计算 base_link 下位置的标准差
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& tp : trajectory_base) {
        mean += tp.position;
    }
    mean /= trajectory_base.size();

    Eigen::Vector3f variance = Eigen::Vector3f::Zero();
    for (const auto& tp : trajectory_base) {
        Eigen::Vector3f diff = tp.position - mean;
        variance += diff.cwiseProduct(diff);
    }
    variance /= trajectory_base.size();

    // 返回标准差的模（越小越稳定）
    return std::sqrt(variance.x() + variance.y() + variance.z());
}

void PayloadTrackManager::updateMotionStats(ObjectTrack& track) {
    // ========== map 坐标系运动统计 ==========
    auto& traj_map = track.trajectory_map;
    if (traj_map.size() < 2) {
        track.map_displacement = 0;
        track.velocity = 0;
        track.direction_consistency = 0;
    } else {
        // 取最近 window_sec 内的轨迹点
        double window_start = traj_map.back().stamp - config_.window_sec;
        std::vector<Eigen::Vector3f> window_points;
        for (auto& tp : traj_map) {
            if (tp.stamp >= window_start) window_points.push_back(tp.position);
        }
        if (window_points.size() >= 2) {
            // 累计位移 = 首尾直线距离
            track.map_displacement = (window_points.back() - window_points.front()).norm();

            // 速度 = 位移 / 时间
            size_t start_idx = traj_map.size() - window_points.size();
            double dt = traj_map.back().stamp - traj_map[start_idx].stamp;
            track.velocity = (dt > 0.01) ? track.map_displacement / dt : 0;

            // 方向一致性
            Eigen::Vector3f main_dir = window_points.back() - window_points.front();
            if (main_dir.norm() > 0.01f) {
                main_dir.normalize();
                float cos_sum = 0;
                int valid_segments = 0;
                for (size_t i = 1; i < window_points.size(); i++) {
                    Eigen::Vector3f seg = window_points[i] - window_points[i - 1];
                    if (seg.norm() > 0.01f) {
                        cos_sum += main_dir.dot(seg.normalized());
                        valid_segments++;
                    }
                }
                track.direction_consistency = (valid_segments > 0) ?
                    cos_sum / valid_segments : 0;
            } else {
                track.direction_consistency = 0;
            }
        } else {
            track.map_displacement = 0;
            track.velocity = 0;
            track.direction_consistency = 0;
        }
    }

    // ========== base_link 坐标系稳定性统计 ==========
    track.base_center_std = computeBaseStability(track.trajectory_base);

    // 横向稳定性（y 方向标准差）
    if (track.trajectory_base.size() >= 3) {
        float mean_y = 0;
        for (const auto& tp : track.trajectory_base) {
            mean_y += tp.position.y();
        }
        mean_y /= track.trajectory_base.size();

        float var_y = 0;
        for (const auto& tp : track.trajectory_base) {
            float diff = tp.position.y() - mean_y;
            var_y += diff * diff;
        }
        var_y /= track.trajectory_base.size();
        track.base_lateral_std = std::sqrt(var_y);
    } else {
        track.base_lateral_std = 0;
    }
}

// ========== 状态转换（双坐标系判断）==========
//
// 核心逻辑：
//   base_link 下稳定 + map 下连续移动 = 吊货（DYNAMIC_PAYLOAD）
//   base_link 下移动 + map 下稳定 = 仓库静态结构
//
// 天车场景中，吊货与雷达同线：
//   - 在 base_link 下，吊货相对雷达位置稳定（base_center_std 小）
//   - 在 map 下，吊货随天车移动（map_displacement 大）
//   - 这正好与仓库固定结构相反

bool PayloadTrackManager::isCargoSizeValid(const ObjectTrack& track) const {
    // 检查 bbox 尺寸是否在合理范围内
    float length = track.bbox_max_map.x() - track.bbox_min_map.x();
    float width = track.bbox_max_map.y() - track.bbox_min_map.y();
    float height = track.bbox_max_map.z() - track.bbox_min_map.z();

    // 合理范围：长 0.6-8m，宽 0.3-3.5m，高 0.2-3m
    bool valid = (length >= 0.6f && length <= 8.0f &&
                  width >= 0.3f && width <= 3.5f &&
                  height >= 0.2f && height <= 3.0f);

    return valid;
}

void PayloadTrackManager::checkStateTransition(ObjectTrack& track) {
    switch (track.state) {
    case TrackState::NEW:
        // 出现几帧后 → PENDING_STATIC
        if (track.observed_frames >= 2) {
            track.state = TrackState::PENDING_STATIC;
        }
        break;

    case TrackState::PENDING_STATIC: {
        // 双坐标系判断
        bool base_stable = (track.base_center_std < config_.base_stability_std_thresh);
        bool map_moving = (track.observed_frames >= config_.motion_confirm_frames &&
                           track.map_displacement > config_.displacement_thresh &&
                           track.velocity > config_.velocity_thresh &&
                           track.direction_consistency > config_.direction_consistency_thresh);

        // HAG 判断（悬浮识别）
        bool has_ground_gap = (track.hag_min > config_.min_floating_gap);
        bool size_valid = isCargoSizeValid(track);

        if (base_stable && size_valid) {
            if (has_ground_gap && map_moving) {
                // 悬浮 + 移动 = SUSPENDED_MOVING
                track.state = TrackState::SUSPENDED_MOVING;
                ROS_WARN("[PayloadTracker] track %d → SUSPENDED_MOVING! "
                         "hag_min=%.2f, base_std=%.2f, map_disp=%.2f vel=%.2f",
                         track.track_id, track.hag_min, track.base_center_std,
                         track.map_displacement, track.velocity);
            } else if (has_ground_gap && track.observed_frames >= config_.min_suspended_observed_frames) {
                // 悬浮 + 静止 = SUSPENDED_STATIC
                track.state = TrackState::SUSPENDED_STATIC;
                ROS_WARN("[PayloadTracker] track %d → SUSPENDED_STATIC! "
                         "hag_min=%.2f, base_std=%.2f, frames=%d",
                         track.track_id, track.hag_min, track.base_center_std,
                         track.observed_frames);
            } else if (map_moving) {
                // 地面 + 移动 = DYNAMIC_PAYLOAD
                track.state = TrackState::DYNAMIC_PAYLOAD;
                ROS_WARN("[PayloadTracker] track %d → DYNAMIC_PAYLOAD! "
                         "hag_min=%.2f, base_std=%.2f, map_disp=%.2f vel=%.2f",
                         track.track_id, track.hag_min, track.base_center_std,
                         track.map_displacement, track.velocity);
            }
        }
        break;
    }

    case TrackState::DYNAMIC_PAYLOAD:
        // 已确认动态，检查是否应该升级为悬浮
        if (config_.suspended_detection_enabled && track.hag_min > config_.strong_floating_gap) {
            track.state = TrackState::SUSPENDED_MOVING;
            ROS_WARN("[PayloadTracker] track %d upgraded to SUSPENDED_MOVING (hag_min=%.2f)",
                     track.track_id, track.hag_min);
        }
        break;

    case TrackState::SUSPENDED_STATIC:
        // 悬浮静止，检查是否开始移动
        if (track.velocity > config_.velocity_thresh * 2) {
            track.state = TrackState::SUSPENDED_MOVING;
            ROS_WARN("[PayloadTracker] track %d → SUSPENDED_MOVING (started moving)",
                     track.track_id);
        }
        break;

    case TrackState::SUSPENDED_MOVING:
        // 悬浮移动，检查是否停止
        if (config_.keep_suspended_when_stopped &&
            track.velocity < config_.velocity_thresh &&
            track.map_displacement < config_.displacement_thresh) {
            track.state = TrackState::SUSPENDED_STATIC;
            ROS_WARN("[PayloadTracker] track %d → SUSPENDED_STATIC (stopped)",
                     track.track_id);
        }
        break;

    case TrackState::EXPIRED:
        break;
    }
}

// ========== 主更新函数（双坐标系版本）==========

TrackResult PayloadTrackManager::update(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& candidates_base,
    const Eigen::Matrix4d& T_map_base,
    double stamp,
    const std::map<CellKey, float>& ground_model) {

    TrackResult result;
    result.dynamic_payload.reset(new pcl::PointCloud<pcl::PointXYZ>);
    result.static_confirmed.reset(new pcl::PointCloud<pcl::PointXYZ>);
    result.pending.reset(new pcl::PointCloud<pcl::PointXYZ>);
    result.active_tracks = 0;
    result.dynamic_tracks = 0;
    result.pending_tracks = 0;

    if (!config_.enabled || !candidates_base || candidates_base->empty()) {
        return result;
    }

    // 1. 在 base_link 下提取 cluster
    auto clusters_base = extractClusters(candidates_base, ground_model);

    // 2. 将 cluster 变换到 map 坐标系
    Eigen::Matrix3d R = T_map_base.block<3,3>(0,0);
    Eigen::Vector3d t = T_map_base.block<3,1>(0,3);

    std::vector<ClusterInfo> clusters_map(clusters_base.size());
    for (size_t i = 0; i < clusters_base.size(); i++) {
        clusters_map[i] = clusters_base[i];  // 复制基本信息

        // 变换 centroid 到 map
        Eigen::Vector3f centroid_base = clusters_base[i].centroid;
        Eigen::Vector3d centroid_map_d = R * centroid_base.cast<double>() + t;
        clusters_map[i].centroid = centroid_map_d.cast<float>();

        // 变换 bbox 到 map
        clusters_map[i].bbox_min = (R * clusters_base[i].bbox_min.cast<double>() + t).cast<float>();
        clusters_map[i].bbox_max = (R * clusters_base[i].bbox_max.cast<double>() + t).cast<float>();

        // cloud 保持 base_link 坐标系不变（用于后续过滤）
        clusters_map[i].cloud = clusters_base[i].cloud;
    }

    // 3. 匹配（使用 map 坐标系的 cluster）
    auto match = matchClusters(clusters_map);

    // 4. 更新匹配到的 track
    for (auto& [ti, ci] : match.matches) {
        auto& track = tracks_[ti];
        auto& cluster_map = clusters_map[ci];
        auto& cluster_base = clusters_base[ci];

        // 更新 map 坐标系位置
        track.centroid_map = cluster_map.centroid;
        track.bbox_min_map = cluster_map.bbox_min;
        track.bbox_max_map = cluster_map.bbox_max;

        // 更新 base_link 坐标系位置
        track.centroid_base = cluster_base.centroid;
        track.bbox_min_base = cluster_base.bbox_min;
        track.bbox_max_base = cluster_base.bbox_max;

        track.hag_min = cluster_base.hag_min;
        track.hag_max = cluster_base.hag_max;
        track.last_seen_time = stamp;
        track.observed_frames++;
        track.missed_frames = 0;

        // 添加轨迹点（双坐标系）
        track.trajectory_map.push_back({stamp, cluster_map.centroid});
        track.trajectory_base.push_back({stamp, cluster_base.centroid});
        while (track.trajectory_map.size() > 200) {
            track.trajectory_map.pop_front();
        }
        while (track.trajectory_base.size() > 200) {
            track.trajectory_base.pop_front();
        }

        // 保存点云历史
        track.cloud_history.push_back(cluster_base.cloud);
        while (track.cloud_history.size() > 50) {
            track.cloud_history.pop_front();
        }

        // 更新运动统计（双坐标系）
        updateMotionStats(track);

        // 检查状态转换
        checkStateTransition(track);

        // 根据状态分流点云
        switch (track.state) {
        case TrackState::NEW:
        case TrackState::PENDING_STATIC:
            *result.pending += *cluster_base.cloud;
            break;
        case TrackState::DYNAMIC_PAYLOAD:
            *result.dynamic_payload += *cluster_base.cloud;
            break;
        case TrackState::EXPIRED:
            *result.static_confirmed += *cluster_base.cloud;
            break;
        }
    }

    // 5. 未匹配的 track → missed_frames++
    for (int ti : match.unmatched_tracks) {
        tracks_[ti].missed_frames++;
        if (tracks_[ti].missed_frames > config_.max_missed_frames) {
            tracks_[ti].state = TrackState::EXPIRED;
        }
    }

    // 6. 未匹配的 cluster → 新建 track
    for (int ci : match.unmatched_clusters) {
        ObjectTrack new_track;
        new_track.track_id = next_track_id_++;
        new_track.state = TrackState::NEW;

        // map 坐标系
        new_track.centroid_map = clusters_map[ci].centroid;
        new_track.bbox_min_map = clusters_map[ci].bbox_min;
        new_track.bbox_max_map = clusters_map[ci].bbox_max;

        // base_link 坐标系
        new_track.centroid_base = clusters_base[ci].centroid;
        new_track.bbox_min_base = clusters_base[ci].bbox_min;
        new_track.bbox_max_base = clusters_base[ci].bbox_max;

        new_track.hag_min = clusters_base[ci].hag_min;
        new_track.hag_max = clusters_base[ci].hag_max;
        new_track.observed_frames = 1;
        new_track.missed_frames = 0;
        new_track.first_seen_time = stamp;
        new_track.last_seen_time = stamp;
        new_track.map_displacement = 0;
        new_track.velocity = 0;
        new_track.direction_consistency = 0;
        new_track.base_center_std = 0;
        new_track.base_lateral_std = 0;

        new_track.trajectory_map.push_back({stamp, clusters_map[ci].centroid});
        new_track.trajectory_base.push_back({stamp, clusters_base[ci].centroid});
        new_track.cloud_history.push_back(clusters_base[ci].cloud);
        tracks_.push_back(new_track);

        // 新建 track 默认进 pending
        *result.pending += *clusters_base[ci].cloud;
    }

    // 7. 清理 EXPIRED tracks（消失超过 20 帧后彻底删除）
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
            [](const ObjectTrack& t) {
                return t.state == TrackState::EXPIRED && t.missed_frames > 20;
            }),
        tracks_.end());

    // 8. 统计
    for (const auto& t : tracks_) {
        if (t.state != TrackState::EXPIRED) {
            result.active_tracks++;
            if (t.state == TrackState::DYNAMIC_PAYLOAD) result.dynamic_tracks++;
            if (t.state == TrackState::PENDING_STATIC || t.state == TrackState::NEW)
                result.pending_tracks++;
        }
    }

    return result;
}

// ========== 获取动态轨迹 ==========

std::vector<ObjectTrack> PayloadTrackManager::getDynamicTracks() const {
    std::vector<ObjectTrack> dynamic;
    for (const auto& t : tracks_) {
        if (t.state == TrackState::DYNAMIC_PAYLOAD) {
            dynamic.push_back(t);
        }
    }
    return dynamic;
}

bool PayloadTrackManager::getBestDynamicPayloadTrack(PayloadTrackInfo& out) const {
    const ObjectTrack* best = nullptr;
    int best_score = -1;

    for (const auto& t : tracks_) {
        if (t.state != TrackState::DYNAMIC_PAYLOAD) continue;

        // 评分：observed_frames * direction_consistency
        int score = static_cast<int>(t.observed_frames * t.direction_consistency * 100);
        if (score > best_score) {
            best_score = score;
            best = &t;
        }
    }

    if (!best) return false;

    out.track_id = best->track_id;
    out.state = 2;  // DYNAMIC
    out.centroid_map = best->centroid_map;
    out.bbox_min_map = best->bbox_min_map;
    out.bbox_max_map = best->bbox_max_map;
    // 使用最近一帧的实际点数，而不是 observed_frames
    out.point_count = best->cloud_history.empty() ? 0 : best->cloud_history.back()->size();
    out.direction_consistency = best->direction_consistency;
    out.map_displacement = best->map_displacement;

    // 计算 track_duration
    out.track_duration = static_cast<float>(best->last_seen_time - best->first_seen_time);

    // 从轨迹计算速度向量
    if (best->trajectory_map.size() >= 2) {
        const auto& last = best->trajectory_map.back();
        const auto& prev = best->trajectory_map[best->trajectory_map.size() - 2];
        Eigen::Vector3f dir = (last.position - prev.position);
        float dist = dir.norm();
        if (dist > 0.001f) {
            dir /= dist;
            out.velocity_map = dir * best->velocity;
        }
    }

    return true;
}

} // namespace ndt_slam
