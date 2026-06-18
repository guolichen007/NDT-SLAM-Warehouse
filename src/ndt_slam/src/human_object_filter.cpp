#include "ndt_slam/human_object_filter.hpp"
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

    // Step 1: 检测人体候选
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidates(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr safe_objects(new pcl::PointCloud<pcl::PointXYZ>);
    detectHumanCandidates(objects_cloud_base, candidates, safe_objects);

    // Step 2: 将候选转到 map 坐标系
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidates_map(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*candidates, *candidates_map, T_map_base);

    // Step 3: 聚类并生成跟踪检测
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    clusterBEV(candidates_map, clusters);

    std::vector<HumanTrack> current_detections;
    for (size_t i = 0; i < clusters.size(); i++) {
        Eigen::Vector3d centroid, bbox_min, bbox_max;
        if (isHumanLikeCluster(clusters[i], centroid, bbox_min, bbox_max)) {
            HumanTrack detection;
            detection.centroid_map = centroid;
            detection.centroid_base = Eigen::Vector3d(
                (T_map_base.inverse() * centroid.homogeneous()).head<3>());
            detection.bbox_min = bbox_min;
            detection.bbox_max = bbox_max;
            detection.point_count = clusters[i]->size();
            detection.height = bbox_max.z() - bbox_min.z();
            detection.area = (bbox_max.x() - bbox_min.x()) * (bbox_max.y() - bbox_min.y());
            current_detections.push_back(detection);
        }
    }

    // Step 4: 更新跟踪
    updateTracks(current_detections, timestamp);

    // Step 5: 分类输出
    // 将候选点分类到不同输出
    for (const auto& point : candidates_map->points) {
        bool assigned = false;

        // 检查是否属于动态人体
        for (const auto& track_pair : active_tracks_) {
            const HumanTrack& track = track_pair.second;
            if (track.state == HumanTrackState::DYNAMIC_CONFIRMED) {
                // 检查点是否在跟踪的 bbox 内
                if (point.x >= track.bbox_min.x() - 0.3 &&
                    point.x <= track.bbox_max.x() + 0.3 &&
                    point.y >= track.bbox_min.y() - 0.3 &&
                    point.y <= track.bbox_max.y() + 0.3 &&
                    point.z >= track.bbox_min.z() - 0.3 &&
                    point.z <= track.bbox_max.z() + 0.3) {
                    human_dynamic_out->push_back(point);
                    assigned = true;
                    break;
                }
            }
        }

        if (!assigned) {
            // 检查是否属于 pending 人体
            for (const auto& track_pair : active_tracks_) {
                const HumanTrack& track = track_pair.second;
                if (track.state == HumanTrackState::PENDING ||
                    track.state == HumanTrackState::NEW) {
                    if (point.x >= track.bbox_min.x() - 0.3 &&
                        point.x <= track.bbox_max.x() + 0.3 &&
                        point.y >= track.bbox_min.y() - 0.3 &&
                        point.y <= track.bbox_max.y() + 0.3 &&
                        point.z >= track.bbox_min.z() - 0.3 &&
                        point.z <= track.bbox_max.z() + 0.3) {
                        human_pending_out->push_back(point);
                        assigned = true;
                        break;
                    }
                }
            }
        }

        if (!assigned) {
            human_candidate_out->push_back(point);
        }
    }

    // safe_objects 已经在 detectHumanCandidates 中生成
    *safe_objects_out = *safe_objects;

    // 清理过期跟踪
    cleanupExpiredTracks(timestamp);
}

void HumanObjectDynamicFilter::detectHumanCandidates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& candidates_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& safe_out) {

    candidates_out->clear();
    safe_out->clear();

    // 按 HAG 高度筛选
    // 假设地面高度为 0（base_link 坐标系下）
    // 实际应用中可能需要更精确的地面高度估计
    double ground_z = 0.0;

    for (const auto& point : cloud->points) {
        double hag = point.z - ground_z;

        if (hag >= filter_config_.min_hag && hag <= filter_config_.max_hag) {
            // 在人体高度范围内，可能是人体候选
            candidates_out->push_back(point);
        } else {
            // 不在人体高度范围内，安全点
            safe_out->push_back(point);
        }
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

    // 连通分量标记（简单的 flood fill）
    std::map<std::pair<int, int>, int> labels;
    int current_label = 0;

    for (const auto& cell : bev_grid) {
        if (labels.find(cell.first) != labels.end()) continue;

        // 新的连通分量
        std::vector<std::pair<int, int>> stack;
        stack.push_back(cell.first);
        labels[cell.first] = current_label;

        while (!stack.empty()) {
            auto current = stack.back();
            stack.pop_back();

            // 检查 8 邻域
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;

                    std::pair<int, int> neighbor = {current.first + dx, current.second + dy};

                    if (bev_grid.find(neighbor) != bev_grid.end() &&
                        labels.find(neighbor) == labels.end()) {
                        // 检查距离
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

    // 检查尺寸约束
    if (height < filter_config_.min_cluster_height ||
        height > filter_config_.max_cluster_height) {
        return false;
    }

    if (area < filter_config_.min_area_m2 ||
        area > filter_config_.max_area_m2) {
        return false;
    }

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
            // 更新已有跟踪
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

            // 更新历史
            track.centroid_map_history.push_back(current_detections[i].centroid_map);
            track.timestamp_history.push_back(timestamp);
            track.bbox_min_history.push_back(current_detections[i].bbox_min);
            track.bbox_max_history.push_back(current_detections[i].bbox_max);

            // 限制历史长度
            size_t max_history = static_cast<size_t>(tracking_config_.window_sec * 10); // 假设 10Hz
            while (track.centroid_map_history.size() > max_history) {
                track.centroid_map_history.pop_front();
                track.timestamp_history.pop_front();
                track.bbox_min_history.pop_front();
                track.bbox_max_history.pop_front();
            }

            // 更新高度范围
            track.track_z_min = std::min(track.track_z_min, current_detections[i].bbox_min.z());
            track.track_z_max = std::max(track.track_z_max, current_detections[i].bbox_max.z());

            // 计算速度和位移
            if (track.centroid_map_history.size() >= 2) {
                Eigen::Vector3d first = track.centroid_map_history.front();
                Eigen::Vector3d last = track.centroid_map_history.back();
                double dt = track.timestamp_history.back() - track.timestamp_history.front();

                track.map_displacement = (last - first).norm();
                if (dt > 0) {
                    track.velocity = track.map_displacement / dt;
                }
            }

            // 判断状态转换
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

            // 初始化高度范围
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

    // 检查时间窗口
    double time_window = track.last_seen_time - track.first_seen_time;
    if (time_window < tracking_config_.window_sec) {
        return false;
    }

    // 检查位移和速度
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

    // 检查时间范围
    if (current_time < capsule.start_time || current_time > capsule.end_time) {
        return false;
    }

    // 检查高度范围
    if (point.z < capsule.z_min || point.z > capsule.z_max) {
        return false;
    }

    // 检查与轨迹中心线的距离
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

    // 清理过期的轨迹胶囊
    trajectory_capsules_.erase(
        std::remove_if(trajectory_capsules_.begin(), trajectory_capsules_.end(),
                       [current_time](const TrajectoryCapsule& c) {
                           return current_time > c.end_time + 10.0; // 保留 10 秒后清理
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

    // 简单的射线法判断点是否在多边形内
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

} // namespace ndt_slam
