#include <ndt_slam/base_payload_channel_filter.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <cmath>
#include <map>
#include <vector>
#include <unordered_set>
#include <set>

namespace ndt_slam {

void BasePayloadChannelFilter::configure(const BasePayloadChannelConfig& config) {
    config_ = config;
}

void BasePayloadChannelFilter::configureFromYaml(const std::string& config_file) {
    try {
        YAML::Node yaml = YAML::LoadFile(config_file);
        auto ch = yaml["base_payload_channel"];
        if (ch) {
            config_.enabled = ch["enabled"].as<bool>(true);
            config_.lateral_center = ch["lateral_center"].as<double>(0.0);
            config_.lateral_half_width = ch["lateral_half_width"].as<double>(2.0);
            config_.longitudinal_min = ch["longitudinal_min"].as<double>(-8.0);
            config_.longitudinal_max = ch["longitudinal_max"].as<double>(8.0);
            config_.min_object_hag = ch["min_object_hag"].as<double>(0.6);
            config_.cluster_bev_resolution = ch["cluster_bev_resolution"].as<double>(0.25);
            config_.min_payload_points = ch["min_payload_points"].as<int>(80);
            config_.min_payload_area_m2 = ch["min_payload_area_m2"].as<double>(0.5);
            config_.max_payload_area_m2 = ch["max_payload_area_m2"].as<double>(80.0);
            config_.expand_xy = ch["expand_xy"].as<double>(0.5);
            config_.expand_z = ch["expand_z"].as<double>(0.4);
            config_.include_weak_points_in_bbox = ch["include_weak_points_in_bbox"].as<bool>(true);

            ROS_INFO("[BasePayloadChannel] loaded config: enabled=%d, "
                     "lateral=[%.1f±%.1f], longitudinal=[%.1f, %.1f], min_hag=%.2f",
                     config_.enabled ? 1 : 0,
                     config_.lateral_center, config_.lateral_half_width,
                     config_.longitudinal_min, config_.longitudinal_max,
                     config_.min_object_hag);
        } else {
            ROS_WARN("[BasePayloadChannel] no 'base_payload_channel' section in yaml, using defaults");
        }
    } catch (const std::exception& e) {
        ROS_WARN("[BasePayloadChannel] Failed to load config: %s, using defaults", e.what());
    }
}

bool BasePayloadChannelFilter::isInChannel(const pcl::PointXYZ& p, float ground_z) const {
    // 检查是否在通道范围内
    if (p.x < config_.longitudinal_min || p.x > config_.longitudinal_max) {
        return false;
    }

    double dy = std::abs(p.y - config_.lateral_center);
    if (dy > config_.lateral_half_width) {
        return false;
    }

    // 检查高度：必须高于地面 min_object_hag
    float hag = p.z - ground_z;
    if (hag < config_.min_object_hag) {
        return false;
    }

    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BasePayloadChannelFilter::extractPayloadCandidates(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& channel_points,
    const std::map<CellKey, float>& ground_model,
    int& cluster_count) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr candidates(new pcl::PointCloud<pcl::PointXYZ>);
    cluster_count = 0;

    if (!channel_points || channel_points->size() < (size_t)config_.min_payload_points) {
        return candidates;
    }

    // BEV 网格聚类
    struct BevCell {
        std::vector<int> indices;
        float max_hag = 0;
        int count = 0;
    };

    std::map<std::pair<int,int>, BevCell> bev_grid;
    for (int i = 0; i < (int)channel_points->size(); i++) {
        const auto& p = channel_points->points[i];
        int cx = (int)std::floor(p.x / config_.cluster_bev_resolution);
        int cy = (int)std::floor(p.y / config_.cluster_bev_resolution);
        auto key = std::make_pair(cx, cy);

        float ground_z = 0;
        CellKey ck{(int)std::floor(p.x / 1.5f), (int)std::floor(p.y / 1.5f)};
        auto it = ground_model.find(ck);
        if (it != ground_model.end()) ground_z = it->second;

        float hag = p.z - ground_z;
        bev_grid[key].indices.push_back(i);
        bev_grid[key].count++;
        if (hag > bev_grid[key].max_hag) {
            bev_grid[key].max_hag = hag;
        }
    }

    // 连通分量分析（相邻 BEV 格子合并为簇）
    std::map<std::pair<int,int>, int> cell_to_cluster;
    int next_cluster = 0;

    // 为每个有足够点的格子分配初始簇
    for (auto& [key, cell] : bev_grid) {
        if (cell.count >= 3 && cell.max_hag >= config_.min_object_hag) {
            cell_to_cluster[key] = next_cluster++;
        }
    }

    // 合并相邻簇（简单的 union-find）
    std::vector<int> parent(next_cluster);
    for (int i = 0; i < next_cluster; i++) parent[i] = i;

    auto find_fn = [&](int x) -> int {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    auto unite_fn = [&](int a, int b) {
        a = find_fn(a); b = find_fn(b);
        if (a != b) parent[a] = b;
    };

    // 检查相邻格子
    int dx[] = {1, 0, 1, -1};
    int dy[] = {0, 1, 1, 1};
    for (auto& [key, cid] : cell_to_cluster) {
        for (int d = 0; d < 4; d++) {
            auto neighbor = std::make_pair(key.first + dx[d], key.second + dy[d]);
            auto it = cell_to_cluster.find(neighbor);
            if (it != cell_to_cluster.end()) {
                unite_fn(cid, it->second);
            }
        }
    }

    // 收集每个最终簇的点
    std::map<int, std::vector<int>> cluster_points;
    for (auto& [key, cid] : cell_to_cluster) {
        int root = find_fn(cid);
        for (int idx : bev_grid[key].indices) {
            cluster_points[root].push_back(idx);
        }
    }

    // 过滤：按点数和面积，收集膨胀后的 bbox
    struct ExpandedBBox {
        float min_x, max_x, min_y, max_y, min_z, max_z;
    };
    std::vector<ExpandedBBox> expanded_bboxes;

    for (auto& [root, indices] : cluster_points) {
        if ((int)indices.size() < config_.min_payload_points) continue;

        // 计算 bbox
        float min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
        float min_z = 1e9, max_z = -1e9;
        for (int idx : indices) {
            const auto& p = channel_points->points[idx];
            if (p.x < min_x) min_x = p.x;
            if (p.x > max_x) max_x = p.x;
            if (p.y < min_y) min_y = p.y;
            if (p.y > max_y) max_y = p.y;
            if (p.z < min_z) min_z = p.z;
            if (p.z > max_z) max_z = p.z;
        }

        float area = (max_x - min_x) * (max_y - min_y);
        if (area < config_.min_payload_area_m2 || area > config_.max_payload_area_m2) continue;

        // 添加簇内点到候选
        for (int idx : indices) {
            candidates->push_back(channel_points->points[idx]);
        }

        // 记录膨胀后的 bbox
        float ex = config_.expand_xy;
        float ez = config_.expand_z;
        expanded_bboxes.push_back({min_x - ex, max_x + ex,
                                   min_y - ex, max_y + ex,
                                   min_z - ez, max_z + ez});

        cluster_count++;
    }

    // 如果有膨胀 bbox，把 bbox 内的所有通道点也加入候选
    if (config_.include_weak_points_in_bbox && !expanded_bboxes.empty()) {
        // 用 set 记录已添加的点索引，避免重复
        std::set<int> added_indices;
        for (const auto& p : candidates->points) {
            // 找到对应的索引
            for (int i = 0; i < (int)channel_points->size(); i++) {
                if (channel_points->points[i].x == p.x &&
                    channel_points->points[i].y == p.y &&
                    channel_points->points[i].z == p.z) {
                    added_indices.insert(i);
                    break;
                }
            }
        }

        // 检查所有通道点是否在任何膨胀 bbox 内
        for (int i = 0; i < (int)channel_points->size(); i++) {
            if (added_indices.count(i)) continue;

            const auto& p = channel_points->points[i];
            for (const auto& bbox : expanded_bboxes) {
                if (p.x >= bbox.min_x && p.x <= bbox.max_x &&
                    p.y >= bbox.min_y && p.y <= bbox.max_y &&
                    p.z >= bbox.min_z && p.z <= bbox.max_z) {
                    candidates->push_back(p);
                    added_indices.insert(i);
                    break;
                }
            }
        }
    }

    return candidates;
}

ChannelFilterResult BasePayloadChannelFilter::filter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud_base,
    const std::map<CellKey, float>& ground_model) {

    ChannelFilterResult result;
    result.safe_objects.reset(new pcl::PointCloud<pcl::PointXYZ>);
    result.payload_candidates.reset(new pcl::PointCloud<pcl::PointXYZ>);
    result.channel_all_points.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (!config_.enabled || !objects_cloud_base || objects_cloud_base->empty()) {
        if (objects_cloud_base) *result.safe_objects = *objects_cloud_base;
        result.safe_points = objects_cloud_base ? objects_cloud_base->size() : 0;
        return result;
    }

    // 分离通道内和通道外的点
    // 关键：channel 外的点直接进 safe_objects，不受膨胀影响
    pcl::PointCloud<pcl::PointXYZ>::Ptr channel_points(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& p : objects_cloud_base->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        // 获取局部地面高度
        float ground_z = 0;
        CellKey ck{(int)std::floor(p.x / 1.5f), (int)std::floor(p.y / 1.5f)};
        auto it = ground_model.find(ck);
        if (it != ground_model.end()) ground_z = it->second;

        if (isInChannel(p, ground_z)) {
            channel_points->push_back(p);
        } else {
            // channel 外的点直接进 safe_objects
            result.safe_objects->push_back(p);
        }
    }

    *result.channel_all_points = *channel_points;
    result.channel_points = channel_points->size();

    // 从通道内点中提取吊货候选簇（含膨胀，但只在 channel 内）
    int cluster_count = 0;
    auto candidates = extractPayloadCandidates(channel_points, ground_model, cluster_count);

    // 构建候选点的快速查找集合
    struct PointHash {
        size_t operator()(const pcl::PointXYZ& p) const {
            // 使用量化坐标避免浮点精度问题
            int ix = (int)std::round(p.x * 100);
            int iy = (int)std::round(p.y * 100);
            int iz = (int)std::round(p.z * 100);
            return std::hash<int>()(ix) ^ (std::hash<int>()(iy) << 1) ^ (std::hash<int>()(iz) << 2);
        }
    };
    struct PointEqual {
        bool operator()(const pcl::PointXYZ& a, const pcl::PointXYZ& b) const {
            return std::abs(a.x - b.x) < 0.01f &&
                   std::abs(a.y - b.y) < 0.01f &&
                   std::abs(a.z - b.z) < 0.01f;
        }
    };
    std::unordered_set<pcl::PointXYZ, PointHash, PointEqual> candidate_set;
    for (const auto& p : candidates->points) {
        candidate_set.insert(p);
    }

    // 将 channel 内的点分为 candidate 和 safe
    for (const auto& p : channel_points->points) {
        if (candidate_set.count(p)) {
            result.payload_candidates->push_back(p);
        } else {
            result.safe_objects->push_back(p);
        }
    }

    result.candidate_points = result.payload_candidates->size();
    result.candidate_clusters = cluster_count;
    result.safe_points = result.safe_objects->size();

    return result;
}

} // namespace ndt_slam
