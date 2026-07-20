#include "ndt_slam/point_cloud_processing.hpp"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace ndt_slam {

PointCloudProcessing::PointCloudProcessing(const PointCloudProcessingConfig& config)
    : config_(config) {
}

void PointCloudProcessing::configure(const PointCloudProcessingConfig& config) {
    config_ = config;
}

void PointCloudProcessing::configureFromYaml(const std::string& config_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_file_path);

        if (config["grid_cell_size"]) {
            config_.grid_cell_size = config["grid_cell_size"].as<double>();
        }
        if (config["height_above_ground"]) {
            config_.height_above_ground = config["height_above_ground"].as<double>();
        }
        if (config["max_map_size"]) {
            config_.max_map_size = config["max_map_size"].as<double>();
        }

        // 体素大小
        if (config["map_voxel_size"]) {
            config_.voxel_size_registration = config["map_voxel_size"].as<double>();
        }
        if (config["display_voxel_size"]) {
            config_.voxel_size_display = config["display_voxel_size"].as<double>();
        }
        if (config["ground_voxel_size"]) {
            config_.voxel_size_ground = config["ground_voxel_size"].as<double>();
        }
        if (config["objects_voxel_size"]) {
            config_.voxel_size_objects = config["objects_voxel_size"].as<double>();
        }

        // 自适应过滤参数
        if (config["adaptive_filter"]) {
            auto af = config["adaptive_filter"];
            if (af["enabled"]) config_.adaptive_filter.enabled = af["enabled"].as<bool>();
            if (af["distance_thresholds"]) {
                auto dt = af["distance_thresholds"];
                if (dt["near"]) config_.adaptive_filter.near_distance = dt["near"].as<double>();
                if (dt["mid"]) config_.adaptive_filter.mid_distance = dt["mid"].as<double>();
            }

            if (af["sor"]) {
                auto sor = af["sor"];
                if (sor["near"]) {
                    if (sor["near"]["mean_k"]) config_.adaptive_filter.sor_near.mean_k = sor["near"]["mean_k"].as<int>();
                    if (sor["near"]["stddev_mul_thresh"]) config_.adaptive_filter.sor_near.stddev_mul_thresh = sor["near"]["stddev_mul_thresh"].as<double>();
                }
                if (sor["mid"]) {
                    if (sor["mid"]["mean_k"]) config_.adaptive_filter.sor_mid.mean_k = sor["mid"]["mean_k"].as<int>();
                    if (sor["mid"]["stddev_mul_thresh"]) config_.adaptive_filter.sor_mid.stddev_mul_thresh = sor["mid"]["stddev_mul_thresh"].as<double>();
                }
                if (sor["far"]) {
                    if (sor["far"]["mean_k"]) config_.adaptive_filter.sor_far.mean_k = sor["far"]["mean_k"].as<int>();
                    if (sor["far"]["stddev_mul_thresh"]) config_.adaptive_filter.sor_far.stddev_mul_thresh = sor["far"]["stddev_mul_thresh"].as<double>();
                }
            }

            if (af["bev_min_height"]) {
                auto bev = af["bev_min_height"];
                if (bev["near"]) config_.adaptive_filter.bev_min_height_near = bev["near"].as<double>();
                if (bev["mid"]) config_.adaptive_filter.bev_min_height_mid = bev["mid"].as<double>();
                if (bev["far"]) config_.adaptive_filter.bev_min_height_far = bev["far"].as<double>();
            }

            if (af["min_observations"]) {
                auto mo = af["min_observations"];
                if (mo["near"]) config_.adaptive_filter.min_observations_near = mo["near"].as<int>();
                if (mo["mid"]) config_.adaptive_filter.min_observations_mid = mo["mid"].as<int>();
                if (mo["far"]) config_.adaptive_filter.min_observations_far = mo["far"].as<int>();
            }
        }

        ROS_INFO("PointCloudProcessing configured from %s", config_file_path.c_str());
        ROS_INFO("  grid_cell_size: %.2f", config_.grid_cell_size);
        ROS_INFO("  height_above_ground: %.2f", config_.height_above_ground);
        ROS_INFO("  adaptive_filter: %s", config_.adaptive_filter.enabled ? "enabled" : "disabled");

    } catch (const YAML::Exception& e) {
        ROS_ERROR("YAML parse error in PointCloudProcessing: %s", e.what());
    }
}

void PointCloudProcessing::separateGroundByGrid(const pcl::PointCloud<pcl::PointXYZ>& input,
                                                 pcl::PointCloud<pcl::PointXYZ>& ground_out,
                                                 pcl::PointCloud<pcl::PointXYZ>& objects_out) {
    if (input.empty()) return;

    // 第一步：按 XY 网格分组，每个格子收集 z 值
    std::map<CellKey, std::vector<float>> cell_z_values;
    std::map<CellKey, std::vector<int>> cell_indices;

    for (int i = 0; i < (int)input.size(); ++i) {
        const auto& p = input.points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        CellKey key{(int)std::floor(p.x / config_.grid_cell_size), (int)std::floor(p.y / config_.grid_cell_size)};
        cell_z_values[key].push_back(p.z);
        cell_indices[key].push_back(i);
    }

    // 第二步：每个格子计算局部地面高度（第 20 百分位）
    std::map<CellKey, float> cell_ground_z;
    for (auto& [key, z_vals] : cell_z_values) {
        if (z_vals.size() < 3) {
            cell_ground_z[key] = *std::min_element(z_vals.begin(), z_vals.end());
        } else {
            std::sort(z_vals.begin(), z_vals.end());
            cell_ground_z[key] = z_vals[z_vals.size() / 5];  // 第20百分位
        }
    }

    // 更新局部地面模型
    cell_ground_z_ = cell_ground_z;

    // 第三步：根据局部地面高度分类
    for (auto& [key, indices] : cell_indices) {
        float local_ground_z = cell_ground_z[key];
        for (int idx : indices) {
            const auto& p = input.points[idx];
            float height_above_ground = p.z - local_ground_z;
            if (height_above_ground < config_.height_above_ground) {
                ground_out.push_back(p);
            } else {
                objects_out.push_back(p);
            }
        }
    }

    // 第四步：自适应 SOR 过滤
    if (config_.adaptive_filter.enabled && objects_out.size() > 50) {
        objects_out = *filterNoiseAdaptive(objects_out.makeShared());
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::extractGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    separateGroundByGrid(*cloud, *ground, *objects);
    return ground;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::extractObjects(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    separateGroundByGrid(*cloud, *ground, *objects);
    return objects;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::filterLowPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                                            float height_threshold) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : cloud->points) {
        if (p.z >= height_threshold) {
            filtered->push_back(p);
        }
    }
    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::filterNoise(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                                        int mean_k,
                                                                        double stddev_mul_thresh) {
    if (cloud->size() < 100) {
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul_thresh);
    sor.filter(*filtered);

    // 如果过滤后点数过少（<30%），说明场景本身点云稀疏，跳过过滤
    if (filtered->size() < cloud->size() * 0.3) {
        ROS_DEBUG("Filter too aggressive (%lu -> %lu), skipping",
                  cloud->size(), filtered->size());
        return cloud;
    }

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::filterNoiseAdaptive(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!config_.adaptive_filter.enabled || cloud->size() < 50) {
        return cloud;
    }

    // 按距离分组：近处（<10m）、中距离（10-20m）、远处（>20m）
    pcl::PointCloud<pcl::PointXYZ> near_cloud, mid_cloud, far_cloud;

    for (const auto& p : cloud->points) {
        float dist = computeDistance(p);
        if (dist < config_.adaptive_filter.near_distance) {
            near_cloud.push_back(p);
        } else if (dist < config_.adaptive_filter.mid_distance) {
            mid_cloud.push_back(p);
        } else {
            far_cloud.push_back(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZ> filtered_objects;

    // 近处：严格过滤
    if (near_cloud.size() > 30) {
        auto near_ptr = near_cloud.makeShared();
        auto result = filterNoise(near_ptr, config_.adaptive_filter.sor_near.mean_k,
                                   config_.adaptive_filter.sor_near.stddev_mul_thresh);
        for (const auto& p : result->points) filtered_objects.push_back(p);
    } else {
        for (const auto& p : near_cloud.points) filtered_objects.push_back(p);
    }

    // 中距离：适中过滤
    if (mid_cloud.size() > 20) {
        auto mid_ptr = mid_cloud.makeShared();
        auto result = filterNoise(mid_ptr, config_.adaptive_filter.sor_mid.mean_k,
                                   config_.adaptive_filter.sor_mid.stddev_mul_thresh);
        for (const auto& p : result->points) filtered_objects.push_back(p);
    } else {
        for (const auto& p : mid_cloud.points) filtered_objects.push_back(p);
    }

    // 远处：宽松过滤
    if (far_cloud.size() > 10) {
        auto far_ptr = far_cloud.makeShared();
        auto result = filterNoise(far_ptr, config_.adaptive_filter.sor_far.mean_k,
                                   config_.adaptive_filter.sor_far.stddev_mul_thresh);
        for (const auto& p : result->points) filtered_objects.push_back(p);
    } else {
        for (const auto& p : far_cloud.points) filtered_objects.push_back(p);
    }

    return filtered_objects.makeShared();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::edgePreservingVoxelGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                                                    double voxel_size) {
    if (!cloud || cloud->empty() || cloud->size() < 100) {
        return cloud;
    }

    // 使用简化的边缘保留融合：只保留每个 voxel 的第一个点
    struct VoxelKey {
        int x, y, z;
        bool operator==(const VoxelKey& o) const { return x==o.x && y==o.y && z==o.z; }
    };
    struct VoxelHash {
        size_t operator()(const VoxelKey& k) const {
            return ((k.x * 73856093) ^ (k.y * 19349663) ^ (k.z * 83492791));
        }
    };

    std::unordered_map<VoxelKey, std::vector<Eigen::Vector3d>, VoxelHash> voxels;

    for (const auto& p : cloud->points) {
        VoxelKey key{
            static_cast<int>(std::floor(p.x / voxel_size)),
            static_cast<int>(std::floor(p.y / voxel_size)),
            static_cast<int>(std::floor(p.z / voxel_size))
        };
        voxels[key].push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto& [key, points] : voxels) {
        if (points.empty()) continue;

        // 计算质心
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& p : points) {
            centroid += p;
        }
        centroid /= points.size();

        // 保留最远的点（边缘点）
        double max_dist = 0;
        Eigen::Vector3d edge_point = points[0];
        for (const auto& p : points) {
            double dist = (p - centroid).norm();
            if (dist > max_dist) {
                max_dist = dist;
                edge_point = p;
            }
        }
        result->points.push_back(pcl::PointXYZ(edge_point.x(), edge_point.y(), edge_point.z()));
    }

    result->width = result->points.size();
    result->height = 1;
    result->is_dense = true;

    return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::voxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                                            double voxel_size) {
    if (!cloud || cloud->empty() || cloud->size() < 100) {
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setInputCloud(cloud);
    vf.setLeafSize(voxel_size, voxel_size, voxel_size);
    vf.filter(*filtered);

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::filterByRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                                          double max_range) {
    if (!cloud || cloud->empty()) {
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : cloud->points) {
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
            double range = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
            if (range > 0.5 && range < max_range) {
                filtered->push_back(p);
            }
        }
    }

    return filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::filterNearField(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                                            double near_field_radius,
                                                                            double near_field_z_min,
                                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points) {
    if (!cloud || cloud->empty()) {
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        double xy_dist = std::sqrt(p.x * p.x + p.y * p.y);
        if (xy_dist < near_field_radius && p.z > near_field_z_min) {
            if (removed_points) {
                removed_points->push_back(p);
            }
        } else {
            filtered->push_back(p);
        }
    }

    return filtered;
}

float PointCloudProcessing::getLocalGroundHeight(float x, float y) const {
    CellKey key{(int)std::floor(x / config_.grid_cell_size), (int)std::floor(y / config_.grid_cell_size)};
    auto it = cell_ground_z_.find(key);
    if (it != cell_ground_z_.end()) {
        return it->second;
    }
    return 0.0f;
}

void PointCloudProcessing::updateLocalGroundModel(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    std::map<CellKey, std::vector<float>> cell_z_values;

    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        CellKey key{(int)std::floor(p.x / config_.grid_cell_size), (int)std::floor(p.y / config_.grid_cell_size)};
        cell_z_values[key].push_back(p.z);
    }

    for (auto& [key, z_vals] : cell_z_values) {
        if (z_vals.size() < 3) {
            cell_ground_z_[key] = *std::min_element(z_vals.begin(), z_vals.end());
        } else {
            std::sort(z_vals.begin(), z_vals.end());
            cell_ground_z_[key] = z_vals[z_vals.size() / 5];
        }
    }
}

float PointCloudProcessing::computeDistance(const pcl::PointXYZ& point) const {
    return std::sqrt(point.x * point.x + point.y * point.y);
}

// ========== 动态货物过滤实现 ==========

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessing::filterPayloadByTracking(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud,
    const Eigen::Matrix4d& current_pose,
    pcl::PointCloud<pcl::PointXYZ>::Ptr payload_out) {

    if (!objects_cloud || objects_cloud->empty()) return objects_cloud;

    // 点云自检测模式：
    // 1. 对 objects 点云做聚类
    // 2. 找大尺寸 cluster
    // 3. 如果 cluster 在连续多帧中相对 base_link 位置稳定，但在 map 中移动明显，判定为吊着的货物

    // 简化实现：使用高度和距离判断
    // 起重机吊着的货物通常：
    // - 高度 > 1.5m（高于大部分静止货物）
    // - 距离传感器较近（在吊钩附近）
    // - 形状长条状

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 提取 base_link 坐标系下的点（需要转换）
    Eigen::Matrix3d R = current_pose.block<3,3>(0,0);
    Eigen::Vector3d t = current_pose.block<3,1>(0,3);
    Eigen::Matrix3d R_inv = R.transpose();
    Eigen::Vector3d t_inv = -R_inv * t;

    // 动态货物过滤策略（简化稳定版）
    //
    // 问题：起重机吊着的货物会进入静态地图，形成长条拖影
    // 解决：使用连续帧统计，识别并剔除移动的大物体
    //
    // 策略：
    // 1. 对 objects 点云做 3D 体素化（0.5m 体素）
    // 2. 统计每个体素被观测的次数
    // 3. 如果一个体素在连续帧中位置变化大，判定为移动物体
    // 4. 移动物体的点不进入静态地图
    //
    // 简化实现：使用 BEV 网格统计，跟踪网格级别的移动

    static int filter_call_count = 0;
    static std::map<std::pair<int,int>, int> bev_observation_count;  // BEV网格 -> 观测次数
    static std::map<std::pair<int,int>, Eigen::Vector3d> bev_last_center;  // BEV网格 -> 上一帧中心
    filter_call_count++;

    const double bev_cell_size = 1.0;  // BEV 网格大小

    // 统计 BEV 网格
    std::map<std::pair<int,int>, std::vector<Eigen::Vector3d>> bev_cells;
    for (const auto& p : objects_cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        int cx = static_cast<int>(std::floor(p.x / bev_cell_size));
        int cy = static_cast<int>(std::floor(p.y / bev_cell_size));
        bev_cells[{cx, cy}].push_back(Eigen::Vector3d(p.x, p.y, p.z));
    }

    // 分析每个 BEV 网格
    std::set<std::pair<int,int>> moving_cells;
    for (auto& [cell, points] : bev_cells) {
        if (points.size() < 20) continue;  // 太小的网格跳过

        // 计算网格中心
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (const auto& p : points) center += p;
        center /= points.size();

        // 与上一帧对比
        auto it = bev_last_center.find(cell);
        if (it != bev_last_center.end()) {
            double motion = (center - it->second).norm();
            // 如果网格移动 > 1.0m，判定为移动物体
            if (motion > 1.0) {
                moving_cells.insert(cell);
            }
        }

        bev_last_center[cell] = center;
        bev_observation_count[cell]++;
    }

    // 过滤：移除移动网格的点
    for (const auto& p : objects_cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

        int cx = static_cast<int>(std::floor(p.x / bev_cell_size));
        int cy = static_cast<int>(std::floor(p.y / bev_cell_size));
        bool is_moving = moving_cells.count({cx, cy}) > 0;

        if (is_moving) {
            if (payload_out) payload_out->push_back(p);
        } else {
            filtered->push_back(p);
        }
    }

    // 每次调用都输出统计
    if (filter_call_count % 5 == 1) {
        ROS_INFO("[PayloadFilter] call=%d, objects_in=%zu, payload_removed=%zu, objects_out=%zu, moving_cells=%zu",
                 filter_call_count, objects_cloud->size(),
                 payload_out ? payload_out->size() : 0, filtered->size(),
                 moving_cells.size());
    }

    return filtered;
}

} // namespace ndt_slam
