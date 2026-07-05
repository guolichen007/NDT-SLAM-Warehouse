#include "ndt_slam/human_object_filter.hpp"
#include <ros/ros.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <algorithm>
#include <cmath>

namespace ndt_slam {

void HumanObjectDynamicFilter::initialize(const HumanObjectFilterConfig& filter_config,
                                          const HumanTrackingConfig& tracking_config,
                                          const HumanEraserConfig& eraser_config) {
    std::lock_guard<std::mutex> lock(mutex_);
    filter_config_ = filter_config;
    tracking_config_ = tracking_config;
    eraser_config_ = eraser_config;
}

void HumanObjectDynamicFilter::processFrame(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud_base,
    const Eigen::Matrix4d& T_map_base,
    double timestamp,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& safe_objects_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& human_candidate_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& human_dynamic_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& human_pending_out) {

    if (!filter_config_.enabled) {
        *safe_objects_out = *objects_cloud_base;
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // 初始化输出
    safe_objects_out->clear();
    human_candidate_out->clear();
    human_dynamic_out->clear();
    human_pending_out->clear();

    // ============================================================
    // 核心思路改变：
    // 旧逻辑：HAG 筛选 → 所有 HAG 范围内的点都是候选 → safe 太少
    // 新逻辑：BEV 聚类 → 判断每个聚类是否符合人体特征 → 只有人体聚类是候选
    // ============================================================

    // Step 1: 对所有 objects 点做 BEV 聚类（在 base_link 下）
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    clusterBEV(objects_cloud_base, clusters);

    // Step 2: 判断每个聚类是否符合人体特征
    // P1: 区分 strong human（>=30 points）和 weak transient（5-30 points）
    std::vector<int> human_cluster_indices;
    std::vector<Eigen::Vector3d> cluster_centroids;
    std::vector<Eigen::Vector3d> cluster_bbox_mins;
    std::vector<Eigen::Vector3d> cluster_bbox_maxs;
    std::vector<bool> cluster_is_strong;  // P1: 标记是否为 strong human

    for (size_t i = 0; i < clusters.size(); i++) {
        Eigen::Vector3d centroid, bbox_min, bbox_max;
        if (isHumanLikeCluster(clusters[i], centroid, bbox_min, bbox_max)) {
            human_cluster_indices.push_back(i);
            cluster_centroids.push_back(centroid);
            cluster_bbox_mins.push_back(bbox_min);
            cluster_bbox_maxs.push_back(bbox_max);

            // P1: 判断是 strong 还是 weak
            bool is_strong = (clusters[i]->size() >= static_cast<size_t>(filter_config_.min_points_strong));
            cluster_is_strong.push_back(is_strong);

            // P1: 将 weak transient 也写入 deny history
            // 这样即使 weak cluster 没有被跟踪，也会被拒绝进入地图
            if (!is_strong) {
                // weak transient：写入 deny cells
                addDenyCells(bbox_min, bbox_max, timestamp, human_deny_ttl_);
                ROS_DEBUG("[HumanFilterV2] weak transient cluster: points=%zu, added to deny history",
                          clusters[i]->size());
            }
        }
    }

    // Step 3: 将人体候选聚类转到 map 坐标系，用于跟踪
    std::vector<HumanTrack> current_detections;
    for (size_t j = 0; j < human_cluster_indices.size(); j++) {
        int idx = human_cluster_indices[j];
        Eigen::Vector3d centroid_base = cluster_centroids[j];
        Eigen::Vector3d centroid_map = (T_map_base * centroid_base.homogeneous()).head<3>();

        HumanTrack detection;
        detection.centroid_base = centroid_base;
        detection.centroid_map = centroid_map;
        detection.bbox_min = cluster_bbox_mins[j];
        detection.bbox_max = cluster_bbox_maxs[j];
        detection.point_count = clusters[idx]->size();
        detection.height = cluster_bbox_maxs[j].z() - cluster_bbox_mins[j].z();
        detection.area = (cluster_bbox_maxs[j].x() - cluster_bbox_mins[j].x()) *
                         (cluster_bbox_maxs[j].y() - cluster_bbox_mins[j].y());
        current_detections.push_back(detection);
    }

    // Step 4: 更新跟踪
    updateTracks(current_detections, timestamp);

    // Step 5: 分类输出
    // 收集所有人体聚类的点索引
    std::set<int> human_point_indices;

    // 人体候选点（当前帧检测到的人体聚类）
    for (size_t j = 0; j < human_cluster_indices.size(); j++) {
        int idx = human_cluster_indices[j];
        for (const auto& point : clusters[idx]->points) {
            human_candidate_out->push_back(point);
        }
    }

    // 动态人体和待确认人体（从跟踪中获取，使用 map 坐标系下的 bbox）
    for (const auto& point : objects_cloud_base->points) {
        Eigen::Vector3d pt_map = (T_map_base * point.getVector4fMap().cast<double>()).head<3>();

        bool is_dynamic = false;
        bool is_pending = false;

        for (const auto& track_pair : active_tracks_) {
            const HumanTrack& track = track_pair.second;
            if (track.state == HumanTrackState::DYNAMIC_CONFIRMED) {
                if (pt_map.x() >= track.bbox_min.x() - 0.5 &&
                    pt_map.x() <= track.bbox_max.x() + 0.5 &&
                    pt_map.y() >= track.bbox_min.y() - 0.5 &&
                    pt_map.y() <= track.bbox_max.y() + 0.5) {
                    is_dynamic = true;
                    break;
                }
            }
        }

        if (!is_dynamic) {
            for (const auto& track_pair : active_tracks_) {
                const HumanTrack& track = track_pair.second;
                if (track.state == HumanTrackState::PENDING ||
                    track.state == HumanTrackState::NEW) {
                    if (pt_map.x() >= track.bbox_min.x() - 0.5 &&
                        pt_map.x() <= track.bbox_max.x() + 0.5 &&
                        pt_map.y() >= track.bbox_min.y() - 0.5 &&
                        pt_map.y() <= track.bbox_max.y() + 0.5) {
                        is_pending = true;
                        break;
                    }
                }
            }
        }

        if (is_dynamic) {
            human_dynamic_out->push_back(point);
        } else if (is_pending) {
            human_pending_out->push_back(point);
        }
    }

    // safe_objects = 所有 objects - 人体候选聚类的点
    // 收集人体聚类中的所有点
    pcl::PointCloud<pcl::PointXYZ>::Ptr human_cluster_points(new pcl::PointCloud<pcl::PointXYZ>);
    for (int idx : human_cluster_indices) {
        *human_cluster_points += *clusters[idx];
    }

    // 从 objects 中移除人体聚类的点
    // 使用简单的 bbox 匹配（因为聚类是在 base_link 下做的）
    for (const auto& point : objects_cloud_base->points) {
        bool in_human_cluster = false;
        for (size_t j = 0; j < human_cluster_indices.size(); j++) {
            int idx = human_cluster_indices[j];
            const auto& bmin = cluster_bbox_mins[j];
            const auto& bmax = cluster_bbox_maxs[j];
            if (point.x >= bmin.x() - 0.1 && point.x <= bmax.x() + 0.1 &&
                point.y >= bmin.y() - 0.1 && point.y <= bmax.y() + 0.1 &&
                point.z >= bmin.z() - 0.1 && point.z <= bmax.z() + 0.1) {
                in_human_cluster = true;
                break;
            }
        }
        if (!in_human_cluster) {
            safe_objects_out->push_back(point);
        }
    }

    // 清理过期跟踪
    cleanupExpiredTracks(timestamp);

    // P1: 清理过期的 deny cells
    cleanupExpiredDenyCells(timestamp);

    // P1: 输出统计信息
    int strong_count = 0, weak_count = 0;
    for (size_t j = 0; j < human_cluster_indices.size(); j++) {
        if (cluster_is_strong[j]) {
            strong_count++;
        } else {
            weak_count++;
        }
    }

    if (strong_count > 0 || weak_count > 0) {
        ROS_DEBUG("[HumanFilterV2] strong=%d, weak=%d, removed=%zu, deny_cells=%d",
                  strong_count, weak_count, human_cluster_indices.size(), getDenyCellCount());
    }
}

void HumanObjectDynamicFilter::clusterBEV(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {

    clusters.clear();

    if (cloud->empty()) return;

    // BEV 网格化
    std::map<std::pair<int, int>, std::vector<int>> bev_grid;

    for (size_t i = 0; i < cloud->size(); i++) {
        auto key = bevKey(cloud->points[i].x, cloud->points[i].y);
        bev_grid[key].push_back(i);
    }

    // 连通分量标记（flood fill）
    std::map<std::pair<int, int>, int> labels;
    int current_label = 0;

    for (const auto& cell : bev_grid) {
        if (labels.find(cell.first) != labels.end()) continue;

        std::vector<std::pair<int, int>> stack;
        stack.push_back(cell.first);
        labels[cell.first] = current_label;

        while (!stack.empty()) {
            auto current = stack.back();
            stack.pop_back();

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;

                    std::pair<int, int> neighbor = {current.first + dx, current.second + dy};

                    if (bev_grid.find(neighbor) != bev_grid.end() &&
                        labels.find(neighbor) == labels.end()) {
                        double dist = std::sqrt(dx * dx + dy * dy) * filter_config_.bev_resolution;
                        if (dist <= filter_config_.merge_gap_m) {
                            labels[neighbor] = current_label;
                            stack.push_back(neighbor);
                        }
                    }
                }
            }
        }
        current_label++;
    }

    // 生成聚类
    clusters.resize(current_label);
    for (int i = 0; i < current_label; i++) {
        clusters[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    for (const auto& label_pair : labels) {
        for (int idx : bev_grid[label_pair.first]) {
            clusters[label_pair.second]->push_back(cloud->points[idx]);
        }
    }
}

bool HumanObjectDynamicFilter::isHumanLikeCluster(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
    Eigen::Vector3d& centroid,
    Eigen::Vector3d& bbox_min,
    Eigen::Vector3d& bbox_max) {

    if (cluster->size() < static_cast<size_t>(filter_config_.min_points) ||
        cluster->size() > static_cast<size_t>(filter_config_.max_points)) {
        return false;
    }

    // 计算 centroid 和 bbox
    Eigen::Vector4f centroid_4f;
    pcl::compute3DCentroid(*cluster, centroid_4f);
    centroid = centroid_4f.head<3>().cast<double>();

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);

    bbox_min = Eigen::Vector3d(min_pt.x, min_pt.y, min_pt.z);
    bbox_max = Eigen::Vector3d(max_pt.x, max_pt.y, max_pt.z);

    double width = bbox_max.x() - bbox_min.x();
    double length = bbox_max.y() - bbox_min.y();
    double height = bbox_max.z() - bbox_min.z();
    double area = width * length;

    // 检查高度约束（相对于聚类自身底部）
    if (height < filter_config_.min_cluster_height ||
        height > filter_config_.max_cluster_height) {
        return false;
    }

    // 检查面积约束
    if (area < filter_config_.min_area_m2 ||
        area > filter_config_.max_area_m2) {
        return false;
    }

    // 检查宽度和长度约束
    if (width > filter_config_.max_width_m ||
        length > filter_config_.max_length_m) {
        return false;
    }

    return true;
}

void HumanObjectDynamicFilter::updateTracks(
    const std::vector<HumanTrack>& current_detections,
    double timestamp) {

    // 标记所有现有跟踪为未匹配
    for (auto& track_pair : active_tracks_) {
        track_pair.second.missed_frames++;
    }

    // 匹配当前检测到已有跟踪
    std::vector<bool> matched(current_detections.size(), false);

    for (size_t i = 0; i < current_detections.size(); i++) {
        int track_id = matchToExistingTrack(current_detections[i]);

        if (track_id >= 0) {
            auto& track = active_tracks_[track_id];
            track.centroid_map = current_detections[i].centroid_map;
            track.centroid_base = current_detections[i].centroid_base;
            track.bbox_min = current_detections[i].bbox_min;
            track.bbox_max = current_detections[i].bbox_max;
            track.point_count = current_detections[i].point_count;
            track.height = current_detections[i].height;
            track.area = current_detections[i].area;
            track.missed_frames = 0;
            track.observed_frames++;
            track.last_seen_time = timestamp;

            track.centroid_map_history.push_back(current_detections[i].centroid_map);
            track.timestamp_history.push_back(timestamp);
            track.bbox_min_history.push_back(current_detections[i].bbox_min);
            track.bbox_max_history.push_back(current_detections[i].bbox_max);

            size_t max_history = static_cast<size_t>(tracking_config_.window_sec * 10);
            while (track.centroid_map_history.size() > max_history) {
                track.centroid_map_history.pop_front();
                track.timestamp_history.pop_front();
                track.bbox_min_history.pop_front();
                track.bbox_max_history.pop_front();
            }

            track.track_z_min = std::min(track.track_z_min, current_detections[i].bbox_min.z());
            track.track_z_max = std::max(track.track_z_max, current_detections[i].bbox_max.z());

            if (track.centroid_map_history.size() >= 2) {
                Eigen::Vector3d first = track.centroid_map_history.front();
                Eigen::Vector3d last = track.centroid_map_history.back();
                double dt = track.timestamp_history.back() - track.timestamp_history.front();

                track.map_displacement = (last - first).norm();
                if (dt > 0) {
                    track.velocity = track.map_displacement / dt;
                }
            }

            if (track.state == HumanTrackState::NEW ||
                track.state == HumanTrackState::PENDING) {
                if (isDynamicHuman(track)) {
                    track.state = HumanTrackState::DYNAMIC_CONFIRMED;
                    generateTrajectoryCapsule(track);
                } else if (track.observed_frames >= tracking_config_.confirm_frames) {
                    track.state = HumanTrackState::PENDING;
                }
            }

            matched[i] = true;
        }
    }

    // 创建新的跟踪
    for (size_t i = 0; i < current_detections.size(); i++) {
        if (!matched[i]) {
            HumanTrack new_track;
            new_track.id = next_track_id_++;
            new_track.state = HumanTrackState::NEW;
            new_track.centroid_map = current_detections[i].centroid_map;
            new_track.centroid_base = current_detections[i].centroid_base;
            new_track.bbox_min = current_detections[i].bbox_min;
            new_track.bbox_max = current_detections[i].bbox_max;
            new_track.point_count = current_detections[i].point_count;
            new_track.height = current_detections[i].height;
            new_track.area = current_detections[i].area;
            new_track.observed_frames = 1;
            new_track.missed_frames = 0;
            new_track.first_seen_time = timestamp;
            new_track.last_seen_time = timestamp;
            new_track.velocity = 0.0;
            new_track.map_displacement = 0.0;

            new_track.track_z_min = current_detections[i].bbox_min.z();
            new_track.track_z_max = current_detections[i].bbox_max.z();
            new_track.track_hag_min = filter_config_.min_hag;
            new_track.track_hag_max = filter_config_.max_hag;

            new_track.centroid_map_history.push_back(current_detections[i].centroid_map);
            new_track.timestamp_history.push_back(timestamp);
            new_track.bbox_min_history.push_back(current_detections[i].bbox_min);
            new_track.bbox_max_history.push_back(current_detections[i].bbox_max);

            active_tracks_[new_track.id] = new_track;
        }
    }
}

int HumanObjectDynamicFilter::matchToExistingTrack(const HumanTrack& detection) {
    int best_id = -1;
    double best_distance = tracking_config_.max_match_distance_m;

    for (const auto& track_pair : active_tracks_) {
        const HumanTrack& track = track_pair.second;

        if (track.missed_frames > tracking_config_.max_missed_frames) continue;

        double distance = (detection.centroid_map - track.centroid_map).norm();
        if (distance < best_distance) {
            best_distance = distance;
            best_id = track.id;
        }
    }

    return best_id;
}

bool HumanObjectDynamicFilter::isDynamicHuman(const HumanTrack& track) const {
    if (track.centroid_map_history.size() < static_cast<size_t>(tracking_config_.confirm_frames)) {
        return false;
    }

    double time_window = track.last_seen_time - track.first_seen_time;
    if (time_window < tracking_config_.window_sec) {
        return false;
    }

    if (track.map_displacement > tracking_config_.map_displacement_thresh_m ||
        track.velocity > tracking_config_.velocity_thresh_mps) {
        return true;
    }

    return false;
}

void HumanObjectDynamicFilter::generateTrajectoryCapsule(const HumanTrack& track) {
    TrajectoryCapsule capsule;
    capsule.track_id = track.id;
    capsule.centerline = track.centroid_map_history;
    capsule.radius = eraser_config_.capsule_radius_m;

    if (eraser_config_.use_track_height_range) {
        capsule.z_min = track.track_z_min - eraser_config_.z_margin_m;
        capsule.z_max = track.track_z_max + eraser_config_.z_margin_m;
        capsule.hag_min = track.track_hag_min - eraser_config_.hag_margin_m;
        capsule.hag_max = track.track_hag_max + eraser_config_.hag_margin_m;
    } else {
        capsule.z_min = filter_config_.min_cluster_height - eraser_config_.z_margin_m;
        capsule.z_max = filter_config_.max_cluster_height + eraser_config_.z_margin_m;
        capsule.hag_min = filter_config_.min_hag - eraser_config_.hag_margin_m;
        capsule.hag_max = filter_config_.max_hag + eraser_config_.hag_margin_m;
    }

    capsule.pre_guard_sec = eraser_config_.pre_guard_sec;
    capsule.post_guard_sec = eraser_config_.post_guard_sec;
    capsule.start_time = track.first_seen_time - eraser_config_.pre_guard_sec;
    capsule.end_time = track.last_seen_time + eraser_config_.post_guard_sec;

    trajectory_capsules_.push_back(capsule);
}

void HumanObjectDynamicFilter::eraseHumanHistory(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_map,
    double current_time) {

    if (!eraser_config_.enabled || trajectory_capsules_.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& point : objects_map->points) {
        bool should_erase = false;

        for (const auto& capsule : trajectory_capsules_) {
            if (isPointInCapsule(point, capsule, current_time)) {
                should_erase = true;
                break;
            }
        }

        if (!should_erase) {
            filtered_map->push_back(point);
        }
    }

    *objects_map = *filtered_map;
}

bool HumanObjectDynamicFilter::isPointInCapsule(
    const pcl::PointXYZ& point,
    const TrajectoryCapsule& capsule,
    double current_time) const {

    if (current_time < capsule.start_time || current_time > capsule.end_time) {
        return false;
    }

    if (point.z < capsule.z_min || point.z > capsule.z_max) {
        return false;
    }

    double min_dist_sq = std::numeric_limits<double>::max();

    for (const auto& center : capsule.centerline) {
        double dx = point.x - center.x();
        double dy = point.y - center.y();
        double dist_sq = dx * dx + dy * dy;
        min_dist_sq = std::min(min_dist_sq, dist_sq);
    }

    return min_dist_sq <= capsule.radius * capsule.radius;
}

void HumanObjectDynamicFilter::cleanupExpiredTracks(double current_time) {
    std::vector<int> expired_ids;

    for (const auto& track_pair : active_tracks_) {
        const HumanTrack& track = track_pair.second;

        if (track.missed_frames > tracking_config_.max_missed_frames) {
            expired_ids.push_back(track.id);
        }
    }

    for (int id : expired_ids) {
        active_tracks_.erase(id);
    }

    trajectory_capsules_.erase(
        std::remove_if(trajectory_capsules_.begin(), trajectory_capsules_.end(),
                       [current_time](const TrajectoryCapsule& c) {
                           return current_time > c.end_time + 10.0;
                       }),
        trajectory_capsules_.end());
}

int HumanObjectDynamicFilter::getActiveTrackCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_tracks_.size();
}

int HumanObjectDynamicFilter::getDynamicHumanCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    int count = 0;
    for (const auto& track_pair : active_tracks_) {
        if (track_pair.second.state == HumanTrackState::DYNAMIC_CONFIRMED) {
            count++;
        }
    }
    return count;
}

std::pair<int, int> HumanObjectDynamicFilter::bevKey(double x, double y) const {
    return {static_cast<int>(std::floor(x / filter_config_.bev_resolution)),
            static_cast<int>(std::floor(y / filter_config_.bev_resolution))};
}

bool HumanObjectDynamicFilter::isPointInPolygonPrism(
    const pcl::PointXYZ& point,
    const std::vector<Eigen::Vector2d>& polygon,
    double z_min, double z_max) const {

    if (point.z < z_min || point.z > z_max) {
        return false;
    }

    int n = polygon.size();
    if (n < 3) return false;

    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if ((polygon[i].y() > point.y) != (polygon[j].y() > point.y) &&
            point.x < (polygon[j].x() - polygon[i].x()) * (point.y - polygon[i].y()) /
                      (polygon[j].y() - polygon[i].y()) + polygon[i].x()) {
            inside = !inside;
        }
    }

    return inside;
}

// ========== P1: Deny History 管理 ==========

void HumanObjectDynamicFilter::addDenyCells(
    const Eigen::Vector3d& bbox_min, const Eigen::Vector3d& bbox_max,
    double current_time, double ttl_sec) {

    double bev_res = filter_config_.bev_resolution;
    int x_min = std::floor(bbox_min.x() / bev_res);
    int x_max = std::floor(bbox_max.x() / bev_res);
    int y_min = std::floor(bbox_min.y() / bev_res);
    int y_max = std::floor(bbox_max.y() / bev_res);

    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {
            auto key = std::make_pair(x, y);
            auto it = human_deny_cells_.find(key);
            if (it != human_deny_cells_.end()) {
                // 更新已有条目
                it->second.last_seen_time = current_time;
                it->second.hit_count++;
            } else {
                // 创建新条目
                DenyCellEntry entry;
                entry.first_seen_time = current_time;
                entry.last_seen_time = current_time;
                entry.hit_count = 1;
                human_deny_cells_[key] = entry;
            }
        }
    }
}

bool HumanObjectDynamicFilter::isCellDenied(double x, double y, double current_time) const {
    int bev_x = std::floor(x / filter_config_.bev_resolution);
    int bev_y = std::floor(y / filter_config_.bev_resolution);
    auto key = std::make_pair(bev_x, bev_y);

    auto it = human_deny_cells_.find(key);
    if (it == human_deny_cells_.end()) {
        return false;
    }

    // 检查是否过期
    double age = current_time - it->second.last_seen_time;
    return age < human_deny_ttl_;
}

void HumanObjectDynamicFilter::cleanupExpiredDenyCells(double current_time) {
    std::vector<std::pair<int,int>> expired_keys;

    for (const auto& entry : human_deny_cells_) {
        double age = current_time - entry.second.last_seen_time;
        if (age >= human_deny_ttl_) {
            expired_keys.push_back(entry.first);
        }
    }

    for (const auto& key : expired_keys) {
        human_deny_cells_.erase(key);
    }
}

int HumanObjectDynamicFilter::getDenyCellCount() const {
    return human_deny_cells_.size();
}

} // namespace ndt_slam
