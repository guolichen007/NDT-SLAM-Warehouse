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

            // P0.5 新增：base_link 优先匹配参数
            config_.max_match_distance_base = pt["max_match_distance_base"].as<double>(0.80);
            config_.max_match_residual_map = pt["max_match_residual_map"].as<double>(1.20);
            config_.max_match_bbox_ratio_base = pt["max_match_bbox_ratio_base"].as<double>(0.70);

            // P0.5 新增：EMA 平滑参数
            config_.centroid_base_alpha = pt["centroid_base_alpha"].as<double>(0.60);
            config_.size_base_alpha = pt["size_base_alpha"].as<double>(0.25);
            config_.lock_switch_confirm_frames = pt["lock_switch_confirm_frames"].as<int>(3);
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
    const std::vector<ClusterInfo>& clusters_map,
    const std::vector<ClusterInfo>& clusters_base,
    const Eigen::Matrix4d& T_map_base) {

    MatchResult result;
    std::vector<bool> track_matched(tracks_.size(), false);
    std::vector<bool> cluster_matched(clusters_map.size(), false);

    // P0.5 新增：base_link 优先匹配
    // 天车场景中，吊货相对雷达稳定，仓库固定物体相对 base_link 移动
    // 所以优先用 base_link 坐标匹配，map 坐标只做 odom 预测残差校验

    Eigen::Matrix3d R = T_map_base.block<3,3>(0,0);
    Eigen::Vector3d t = T_map_base.block<3,1>(0,3);

    for (size_t ti = 0; ti < tracks_.size(); ti++) {
        if (tracks_[ti].state == TrackState::EXPIRED) continue;

        double best_score = 1e9;
        int best_ci = -1;

        for (size_t ci = 0; ci < clusters_map.size(); ci++) {
            if (cluster_matched[ci]) continue;

            // 1. base_link 下距离匹配（主要判据）
            double dist_base = (tracks_[ti].centroid_base - clusters_base[ci].centroid).norm();
            if (dist_base > config_.max_match_distance_base) continue;

            // 2. map 下 odom 预测残差（辅助判据）
            // 用上一帧的 T_map_base 预测当前 track 在 map 下的位置
            Eigen::Vector3d centroid_map_last_d = tracks_[ti].centroid_map.cast<double>();
            // 如果有上一帧的 T_map_base，用 odom delta 预测
            if (tracks_[ti].last_T_map_base != Eigen::Matrix4d::Identity()) {
                Eigen::Matrix4d T_delta = T_map_base * tracks_[ti].last_T_map_base.inverse();
                centroid_map_last_d = T_delta.block<3,3>(0,0) * centroid_map_last_d + T_delta.block<3,1>(0,3);
            }
            double residual_map = (centroid_map_last_d - clusters_map[ci].centroid.cast<double>()).norm();
            if (residual_map > config_.max_match_residual_map) continue;

            // 3. 包围盒尺寸约束（base_link 下）
            Eigen::Vector3f track_size = tracks_[ti].bbox_max_base - tracks_[ti].bbox_min_base;
            Eigen::Vector3f cluster_size = clusters_base[ci].bbox_max - clusters_base[ci].bbox_min;
            float size_diff = (track_size - cluster_size).cwiseAbs().maxCoeff();
            float size_max = track_size.cwiseMax(cluster_size).maxCoeff();
            if (size_max > 0.1f && size_diff / size_max > config_.max_match_bbox_ratio_base)
                continue;

            // 综合评分：base 距离权重 2.0，map 残差权重 1.0
            double score = 2.0 * dist_base + 1.0 * residual_map;
            if (score < best_score) {
                best_score = score;
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
    for (size_t ci = 0; ci < clusters_map.size(); ci++) {
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

    // 3. 匹配（P0.5: base_link 优先匹配）
    auto match = matchClusters(clusters_map, clusters_base, T_map_base);

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

        // P0.5 新增：更新 base_link 下的 EMA 平滑
        updateBaseEma(track, cluster_base);

        // P0.5 新增：用当前 T_map_base 把 base bbox 转成 map bbox
        transformBaseBboxToMap(track, T_map_base);
        track.last_bbox_time = stamp;

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
        case TrackState::SUSPENDED_STATIC:
            // 悬挂静止不等于地图静态，吊着不动的货物仍不能进入 permanent static map
            *result.pending += *cluster_base.cloud;
            break;
        case TrackState::DYNAMIC_PAYLOAD:
        case TrackState::SUSPENDED_MOVING:
            *result.dynamic_payload += *cluster_base.cloud;
            break;
        case TrackState::EXPIRED:
            // 第一版不要直接塞进 static_confirmed，避免动态货物丢失后被写入静态地图
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

        // P0.5 新增：初始化 EMA 字段
        new_track.centroid_base_ema = clusters_base[ci].centroid;
        new_track.size_ema = clusters_base[ci].bbox_max - clusters_base[ci].bbox_min;
        new_track.bbox_min_base_ema = clusters_base[ci].bbox_min;
        new_track.bbox_max_base_ema = clusters_base[ci].bbox_max;

        // 初始化 display bbox（与当前帧一致）
        new_track.centroid_map_display = clusters_map[ci].centroid;
        new_track.bbox_min_map_display = clusters_map[ci].bbox_min;
        new_track.bbox_max_map_display = clusters_map[ci].bbox_max;

        new_track.last_T_map_base = T_map_base;

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
        if (t.state == TrackState::DYNAMIC_PAYLOAD ||
            t.state == TrackState::SUSPENDED_MOVING ||
            t.state == TrackState::SUSPENDED_STATIC) {
            dynamic.push_back(t);
        }
    }
    return dynamic;
}

bool PayloadTrackManager::getBestDynamicPayloadTrack(PayloadTrackInfo& out) const {
    const ObjectTrack* best = nullptr;
    int best_score = -1;

    for (const auto& t : tracks_) {
        // 包含 DYNAMIC_PAYLOAD、SUSPENDED_MOVING、SUSPENDED_STATIC
        if (t.state != TrackState::DYNAMIC_PAYLOAD &&
            t.state != TrackState::SUSPENDED_MOVING &&
            t.state != TrackState::SUSPENDED_STATIC) continue;

        // 评分：observed_frames * direction_consistency
        int score = static_cast<int>(t.observed_frames * t.direction_consistency * 100);
        if (score > best_score) {
            best_score = score;
            best = &t;
        }
    }

    if (!best) return false;

    out.track_id = best->track_id;

    // 统一状态编码：TrackState -> PayloadSemanticState
    // NEW/PENDING_STATIC -> SUSPENDED_CANDIDATE (2)
    // DYNAMIC_PAYLOAD/SUSPENDED_MOVING -> SUSPENDED_MOVING (3)
    // SUSPENDED_STATIC -> SUSPENDED_STATIC (4)
    // EXPIRED -> LOST (7)
    switch (best->state) {
        case TrackState::NEW:
        case TrackState::PENDING_STATIC:
            out.state = 2;  // SUSPENDED_CANDIDATE
            break;
        case TrackState::DYNAMIC_PAYLOAD:
        case TrackState::SUSPENDED_MOVING:
            out.state = 3;  // SUSPENDED_MOVING
            break;
        case TrackState::SUSPENDED_STATIC:
            out.state = 4;  // SUSPENDED_STATIC
            break;
        case TrackState::EXPIRED:
            out.state = 7;  // LOST
            break;
        default:
            out.state = 0;  // UNKNOWN
            break;
    }

    // P0.5: 使用 display bbox（由 base EMA 转换而来）
    out.centroid_map = best->centroid_map_display;
    out.bbox_min_map = best->bbox_min_map_display;
    out.bbox_max_map = best->bbox_max_map_display;
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

    // P3: 复制 CargoBoxV2 的 core_box 信息
    if (best->has_last_core_box) {
        out.has_core_box = true;
        out.core_box_base = best->last_core_box;
    } else {
        out.has_core_box = false;
    }

    return true;
}

// ========== P0.5 新增：base_link 下的 EMA 平滑 ==========

void PayloadTrackManager::updateBaseEma(ObjectTrack& track, const ClusterInfo& cluster_base) {
    float alpha_c = config_.centroid_base_alpha;
    float alpha_s = config_.size_base_alpha;

    // 第一帧初始化
    if (track.centroid_base_ema.isZero() && track.observed_frames <= 1) {
        track.centroid_base_ema = cluster_base.centroid;
        track.size_ema = cluster_base.bbox_max - cluster_base.bbox_min;
        track.bbox_min_base_ema = cluster_base.bbox_min;
        track.bbox_max_base_ema = cluster_base.bbox_max;
        return;
    }

    // 中心点 EMA
    track.centroid_base_ema = alpha_c * cluster_base.centroid +
                              (1.0f - alpha_c) * track.centroid_base_ema;

    // 尺寸 EMA
    Eigen::Vector3f cluster_size = cluster_base.bbox_max - cluster_base.bbox_min;
    track.size_ema = alpha_s * cluster_size + (1.0f - alpha_s) * track.size_ema;

    // bbox min/max 由中心和尺寸推导
    Eigen::Vector3f half_size = track.size_ema / 2.0f;
    track.bbox_min_base_ema = track.centroid_base_ema - half_size;
    track.bbox_max_base_ema = track.centroid_base_ema + half_size;
}

void PayloadTrackManager::transformBaseBboxToMap(ObjectTrack& track, const Eigen::Matrix4d& T_map_base) {
    Eigen::Matrix3d R = T_map_base.block<3,3>(0,0);
    Eigen::Vector3d t = T_map_base.block<3,1>(0,3);

    // 转换中心点
    Eigen::Vector3d centroid_map_d = R * track.centroid_base_ema.cast<double>() + t;
    track.centroid_map_display = centroid_map_d.cast<float>();

    // 转换 bbox min/max（8 个角点都转换，然后取 min/max）
    Eigen::Vector3f corners[2] = {track.bbox_min_base_ema, track.bbox_max_base_ema};
    Eigen::Vector3f map_min(1e9, 1e9, 1e9);
    Eigen::Vector3f map_max(-1e9, -1e9, -1e9);

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                Eigen::Vector3f corner(corners[i].x(), corners[j].y(), corners[k].z());
                Eigen::Vector3d corner_map = R * corner.cast<double>() + t;
                Eigen::Vector3f corner_map_f = corner_map.cast<float>();
                map_min = map_min.cwiseMin(corner_map_f);
                map_max = map_max.cwiseMax(corner_map_f);
            }
        }
    }

    track.bbox_min_map_display = map_min;
    track.bbox_max_map_display = map_max;

    // 保存当前 T_map_base
    track.last_T_map_base = T_map_base;
}

// ============================================================================
// P1: 清理过期的 SUSPENDED_STATIC track
// ============================================================================

void PayloadTrackManager::cleanupStaleSuspendedStaticTracks(double current_time)
{
    const double suspended_static_ttl = 20.0;
    const int suspended_static_max_reinit_reject = 6;
    const int suspended_static_min_core_points = 25;

    int expired_count = 0;

    for (auto& track : tracks_) {
        if (track.state != TrackState::SUSPENDED_STATIC) {
            continue;
        }

        double age_since_seen = current_time - track.last_seen_time;

        bool timeout = age_since_seen > suspended_static_ttl;
        bool too_many_reject = track.size_jump_count > suspended_static_max_reinit_reject;
        bool weak_core = track.has_last_core_box &&
                         track.last_core_box.suspended_points < suspended_static_min_core_points;

        if (timeout || too_many_reject || weak_core) {
            ROS_INFO("[TrackCleanup] expire suspended_static track=%d age=%.1f reject_count=%d core_pts=%d reason=%s",
                     track.track_id,
                     age_since_seen,
                     track.size_jump_count,
                     track.has_last_core_box ? track.last_core_box.suspended_points : 0,
                     timeout ? "timeout" : (too_many_reject ? "too_many_reject" : "weak_core"));

            track.state = TrackState::EXPIRED;
            expired_count++;
        }
    }

    // 压缩已过期的 track
    tracks_.erase(
        std::remove_if(
            tracks_.begin(),
            tracks_.end(),
            [&](const ObjectTrack& t) {
                return t.state == TrackState::EXPIRED &&
                       current_time - t.last_seen_time > 2.0;
            }),
        tracks_.end());

    if (expired_count > 0) {
        ROS_INFO("[TrackCleanup] expired=%d active_tracks=%zu", expired_count, tracks_.size());
    }
}

} // namespace ndt_slam
