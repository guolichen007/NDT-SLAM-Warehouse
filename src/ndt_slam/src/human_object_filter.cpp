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
    // Historical capsule deletion is no longer a product authority. Keep the
    // parameter for configuration compatibility without retaining state.
    (void)eraser_config;
}

void HumanObjectDynamicFilter::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    active_tracks_.clear();
    next_track_id_ = 0;
    has_last_map_update_stamp_ = false;
    last_map_update_stamp_sec_ = 0.0;
    last_map_update_cloud_instance_id_ = 0U;
    diagnostics_ = HumanMapAuthorityDiagnostics{};
}

HumanFrameClassification HumanObjectDynamicFilter::classifyFrame(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud_base,
    const SourceFrameIdentity& source_frame_identity,
    float ownership_voxel_size_m,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& safe_objects_out,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& human_candidate_out) const {
    HumanFrameClassification result;
    safe_objects_out->clear();
    human_candidate_out->clear();
    if (!objects_cloud_base || !source_frame_identity.valid() ||
        !std::isfinite(ownership_voxel_size_m) ||
        ownership_voxel_size_m <= 0.0F) {
        return result;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!filter_config_.enabled) {
        *safe_objects_out = *objects_cloud_base;
        return result;
    }

    result.source_stamp_sec = source_frame_identity.sensor_source_stamp_sec;
    result.source_cloud_instance_id =
        source_frame_identity.processing_frame_index;
    result.source_frame_identity = source_frame_identity;
    result.owned_points.source_frame_identity = source_frame_identity;
    result.owned_points.voxel_size_m = ownership_voxel_size_m;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    clusterBEV(objects_cloud_base, clusters);
    for (const auto& cluster : clusters) {
        Eigen::Vector3d centroid;
        Eigen::Vector3d bbox_min;
        Eigen::Vector3d bbox_max;
        if (!isHumanLikeCluster(cluster, centroid, bbox_min, bbox_max)) {
            continue;
        }
        HumanClusterObservation observation;
        observation.centroid_base = centroid;
        observation.bbox_min_base = bbox_min;
        observation.bbox_max_base = bbox_max;
        observation.point_count = static_cast<int>(cluster->size());
        observation.strong = cluster->size() >=
            static_cast<std::size_t>(filter_config_.min_points_strong);
        result.clusters.push_back(observation);
        for (const pcl::PointXYZ& point : cluster->points) {
            SourcePointKey exact_key;
            if (makeSourcePointKey(point, &exact_key)) {
                result.owned_points.exact_points.insert(exact_key);
            }
            PointOwnershipVoxel voxel;
            if (makePointOwnershipVoxel(
                    point, ownership_voxel_size_m, &voxel)) {
                result.owned_points.voxels.insert(voxel);
            }
            human_candidate_out->push_back(point);
        }
    }

    result.owned_points.valid = true;
    for (const pcl::PointXYZ& point : objects_cloud_base->points) {
        if (!result.owned_points.owns(point)) {
            safe_objects_out->push_back(point);
        }
    }
    result.valid = true;
    ++diagnostics_.classification_count;
    diagnostics_.registration_owned_point_count +=
        result.owned_points.exact_points.size();
    return result;
}

HumanMapFilterSnapshot HumanObjectDynamicFilter::updateMapTracks(
    const HumanFrameClassification& classification,
    const Eigen::Matrix4d& T_map_base,
    float static_learning_cell_size_m) {
    HumanMapFilterSnapshot snapshot;
    if (!classification.valid ||
        !classification.source_frame_identity.valid() ||
        classification.source_cloud_instance_id !=
            classification.source_frame_identity.processing_frame_index ||
        !sameSourceFrameIdentity(
            classification.owned_points.source_frame_identity,
            classification.source_frame_identity) ||
        std::abs(classification.source_stamp_sec -
                 classification.source_frame_identity.sensor_source_stamp_sec) >
            1.0e-6 ||
        !std::isfinite(classification.source_stamp_sec) ||
        classification.source_stamp_sec <= 0.0 ||
        classification.source_cloud_instance_id == 0U ||
        !T_map_base.allFinite() ||
        !std::isfinite(static_learning_cell_size_m) ||
        static_learning_cell_size_m <= 0.0F) {
        return snapshot;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    constexpr double kStampEpsilonSec = 1.0e-4;
    if (has_last_map_update_stamp_ &&
        (classification.source_stamp_sec <=
             last_map_update_stamp_sec_ + kStampEpsilonSec ||
         classification.source_cloud_instance_id <=
             last_map_update_cloud_instance_id_)) {
        const bool duplicate =
            classification.source_cloud_instance_id ==
                last_map_update_cloud_instance_id_ &&
            std::abs(classification.source_stamp_sec -
                     last_map_update_stamp_sec_) <= kStampEpsilonSec;
        if (duplicate) {
            ++diagnostics_.duplicate_update_reject_count;
        } else {
            ++diagnostics_.out_of_order_update_reject_count;
        }
        return snapshot;
    }

    std::vector<HumanTrack> detections;
    detections.reserve(classification.clusters.size());
    for (const HumanClusterObservation& cluster : classification.clusters) {
        HumanTrack detection{};
        detection.centroid_base = cluster.centroid_base;
        detection.centroid_map =
            (T_map_base * cluster.centroid_base.homogeneous()).head<3>();

        Eigen::Vector3d map_min = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::infinity());
        Eigen::Vector3d map_max = Eigen::Vector3d::Constant(
            -std::numeric_limits<double>::infinity());
        for (int x_index = 0; x_index < 2; ++x_index) {
            for (int y_index = 0; y_index < 2; ++y_index) {
                for (int z_index = 0; z_index < 2; ++z_index) {
                    const Eigen::Vector3d corner(
                        x_index == 0 ? cluster.bbox_min_base.x()
                                     : cluster.bbox_max_base.x(),
                        y_index == 0 ? cluster.bbox_min_base.y()
                                     : cluster.bbox_max_base.y(),
                        z_index == 0 ? cluster.bbox_min_base.z()
                                     : cluster.bbox_max_base.z());
                    const Eigen::Vector3d transformed =
                        (T_map_base * corner.homogeneous()).head<3>();
                    map_min = map_min.cwiseMin(transformed);
                    map_max = map_max.cwiseMax(transformed);
                }
            }
        }
        detection.bbox_min = map_min;
        detection.bbox_max = map_max;
        detection.point_count = cluster.point_count;
        detection.height = map_max.z() - map_min.z();
        detection.area = (map_max.x() - map_min.x()) *
            (map_max.y() - map_min.y());
        detections.push_back(detection);
    }

    updateTracks(detections, classification.source_stamp_sec);
    cleanupExpiredTracks(classification.source_stamp_sec);
    has_last_map_update_stamp_ = true;
    last_map_update_stamp_sec_ = classification.source_stamp_sec;
    last_map_update_cloud_instance_id_ =
        classification.source_cloud_instance_id;

    snapshot.valid = true;
    snapshot.source_stamp_sec = classification.source_stamp_sec;
    snapshot.source_cloud_instance_id =
        classification.source_cloud_instance_id;
    snapshot.source_frame_identity = classification.source_frame_identity;
    snapshot.owned_points = classification.owned_points;
    snapshot.static_learning_blocks.cell_size_m =
        static_learning_cell_size_m;
    std::vector<HumanTrack> live_tracks;
    live_tracks.reserve(active_tracks_.size());
    for (const auto& item : active_tracks_) {
        live_tracks.push_back(item.second);
    }
    snapshot.static_learning_blocks.human_cells =
        buildStaticLearningBlockCells(
            live_tracks, static_learning_cell_size_m);
    snapshot.active_track_count = static_cast<int>(active_tracks_.size());
    for (const auto& item : active_tracks_) {
        if (item.second.state == HumanTrackState::DYNAMIC_CONFIRMED) {
            ++snapshot.dynamic_track_count;
        }
    }
    ++diagnostics_.map_track_update_count;
    diagnostics_.map_owned_point_count +=
        snapshot.owned_points.exact_points.size();
    diagnostics_.static_learning_block_cell_count =
        snapshot.static_learning_blocks.human_cells.size();
    diagnostics_.static_learning_block_high_water = std::max(
        diagnostics_.static_learning_block_high_water,
        snapshot.static_learning_blocks.human_cells.size());
    diagnostics_.track_high_water = std::max(
        diagnostics_.track_high_water, active_tracks_.size());
    return snapshot;
}

void HumanObjectDynamicFilter::clusterBEV(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) const {

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
    Eigen::Vector3d& bbox_max) const {

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

            size_t max_history = static_cast<size_t>(tracking_config_.window_sec * 10);
            while (track.centroid_map_history.size() > max_history) {
                track.centroid_map_history.pop_front();
                track.timestamp_history.pop_front();
            }

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

            new_track.centroid_map_history.push_back(current_detections[i].centroid_map);
            new_track.timestamp_history.push_back(timestamp);

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

    (void)current_time;
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

HumanMapAuthorityDiagnostics HumanObjectDynamicFilter::diagnostics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return diagnostics_;
}

std::set<std::pair<int, int>>
HumanObjectDynamicFilter::buildStaticLearningBlockCells(
    const std::vector<HumanTrack>& detections,
    float cell_size_m) const {
    std::set<std::pair<int, int>> cells;
    if (!std::isfinite(cell_size_m) || cell_size_m <= 0.0F) {
        return cells;
    }
    for (const HumanTrack& detection : detections) {
        const int x_min = static_cast<int>(
            std::floor(detection.bbox_min.x() / cell_size_m));
        const int x_max = static_cast<int>(
            std::floor(detection.bbox_max.x() / cell_size_m));
        const int y_min = static_cast<int>(
            std::floor(detection.bbox_min.y() / cell_size_m));
        const int y_max = static_cast<int>(
            std::floor(detection.bbox_max.y() / cell_size_m));
        for (int x = x_min; x <= x_max; ++x) {
            for (int y = y_min; y <= y_max; ++y) {
                cells.emplace(x, y);
            }
        }
    }
    return cells;
}

std::pair<int, int> HumanObjectDynamicFilter::bevKey(double x, double y) const {
    return {static_cast<int>(std::floor(x / filter_config_.bev_resolution)),
            static_cast<int>(std::floor(y / filter_config_.bev_resolution))};
}

} // namespace ndt_slam
