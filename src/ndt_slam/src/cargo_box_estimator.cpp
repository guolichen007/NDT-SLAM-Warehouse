#include <ndt_slam/cargo_box_estimator.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <queue>

namespace ndt_slam {

void CargoBoxEstimator::configure(const CargoBoxEstimatorConfig& config) {
    config_ = config;
}

void CargoBoxEstimator::configureFromYaml(const std::string& config_file) {
    try {
        YAML::Node yaml = YAML::LoadFile(config_file);
        auto cb = yaml["cargo_box_estimator"];
        if (cb) {
            config_.enabled = cb["enabled"].as<bool>(true);
            config_.min_cluster_points = cb["min_cluster_points"].as<int>(40);
            config_.max_cluster_points = cb["max_cluster_points"].as<int>(20000);

            // BEV 参数
            config_.bev_resolution = cb["bev_resolution"].as<double>(0.10);
            config_.bev_min_points_per_cell = cb["bev_min_points_per_cell"].as<int>(2);
            config_.bev_min_component_cells = cb["bev_min_component_cells"].as<int>(4);
            config_.bev_connectivity = cb["bev_connectivity"].as<int>(8);

            // HAG 参数
            config_.min_hag_for_candidate = cb["min_hag_for_candidate"].as<double>(0.25);
            config_.min_payload_bottom_hag = cb["min_payload_bottom_hag"].as<double>(0.45);
            config_.z_hist_bin = cb["z_hist_bin"].as<double>(0.10);
            config_.min_z_band_points = cb["min_z_band_points"].as<int>(20);
            config_.dense_bin_ratio = cb["dense_bin_ratio"].as<double>(0.15);
            config_.dense_bin_min_points = cb["dense_bin_min_points"].as<int>(5);
            config_.min_dense_band_height = cb["min_dense_band_height"].as<double>(0.10);
            config_.max_dense_band_height = cb["max_dense_band_height"].as<double>(3.50);

            // 分位数参数
            config_.xy_percentile_low = cb["xy_percentile_low"].as<double>(2.0);
            config_.xy_percentile_high = cb["xy_percentile_high"].as<double>(98.0);
            config_.z_percentile_low = cb["z_percentile_low"].as<double>(5.0);
            config_.z_percentile_high = cb["z_percentile_high"].as<double>(98.0);

            config_.use_crane_axis_obb = cb["use_crane_axis_obb"].as<bool>(true);

            // 尺寸限制
            config_.max_width = cb["max_width"].as<double>(6.0);
            config_.max_length = cb["max_length"].as<double>(16.0);
            config_.max_height = cb["max_height"].as<double>(5.0);

            // 绝对尺寸约束
            config_.max_core_length = cb["max_core_length"].as<double>(12.0);
            config_.max_core_width = cb["max_core_width"].as<double>(4.0);
            config_.max_core_height = cb["max_core_height"].as<double>(4.0);
            config_.min_core_length = cb["min_core_length"].as<double>(0.30);
            config_.min_core_width = cb["min_core_width"].as<double>(0.30);
            config_.min_core_height = cb["min_core_height"].as<double>(0.10);

            // 初始化质量约束
            config_.init_max_area = cb["init_max_area"].as<double>(30.0);
            config_.init_max_aspect_ratio = cb["init_max_aspect_ratio"].as<double>(8.0);
            config_.init_min_bottom_hag = cb["init_min_bottom_hag"].as<double>(0.45);
            config_.init_min_core_points = cb["init_min_core_points"].as<int>(40);

            // 尺寸增长率限制
            config_.max_size_growth_ratio = cb["max_size_growth_ratio"].as<double>(1.35);

            // 最小悬空高度
            config_.min_suspended_hag = cb["min_suspended_hag"].as<double>(0.35);

            // 最小 core points
            config_.min_core_points = cb["min_core_points"].as<int>(30);

            // P0-6: 候选/确认两级阈值
            config_.min_candidate_core_points = cb["min_candidate_core_points"].as<int>(5);
            config_.min_confirm_core_points = cb["min_confirm_core_points"].as<int>(25);
            config_.min_update_core_points = cb["min_update_core_points"].as<int>(8);
            config_.min_confirm_observed_frames = cb["min_confirm_observed_frames"].as<int>(3);

            // 显示框扩展
            config_.core_expand_xy = cb["core_expand_xy"].as<double>(0.05);
            config_.core_expand_z_down = cb["core_expand_z_down"].as<double>(0.03);
            config_.core_expand_z_up = cb["core_expand_z_up"].as<double>(0.10);

            // 删除框扩展
            config_.remove_expand_xy = cb["remove_expand_xy"].as<double>(0.25);
            config_.remove_expand_z_down = cb["remove_expand_z_down"].as<double>(0.05);
            config_.remove_expand_z_up = cb["remove_expand_z_up"].as<double>(0.20);

            // 禁行区扩展
            config_.forbidden_expand_xy = cb["forbidden_expand_xy"].as<double>(0.50);
            config_.forbidden_height = cb["forbidden_height"].as<double>(3.0);
        }
    } catch (const std::exception& e) {
        ROS_WARN("[CargoBoxEstimator] Failed to load config: %s, using defaults", e.what());
    }
}

float CargoBoxEstimator::percentile(std::vector<float>& values, float p) {
    if (values.empty()) return 0.0f;
    std::sort(values.begin(), values.end());
    float idx = p / 100.0f * (values.size() - 1);
    int lower = static_cast<int>(std::floor(idx));
    int upper = static_cast<int>(std::ceil(idx));
    if (lower == upper) return values[lower];
    float frac = idx - lower;
    return values[lower] * (1.0f - frac) + values[upper] * frac;
}

// ========== BEV 连通域 ==========

std::vector<BevComponent> CargoBoxEstimator::buildBevComponents(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
    const SimpleGroundModel& ground_model) {

    std::vector<BevComponent> components;
    if (!points || points->empty()) return components;

    // 1. 建立 BEV grid
    std::map<std::pair<int,int>, BevCell> bev_grid;

    for (size_t i = 0; i < points->size(); i++) {
        const auto& p = points->points[i];
        float ground_z = ground_model.getGroundZ(p.x, p.y);
        float hag = p.z - ground_z;

        // 只保留 HAG >= min_hag_for_candidate 的点
        if (hag < config_.min_hag_for_candidate) continue;

        int cell_x = static_cast<int>(std::floor(p.x / config_.bev_resolution));
        int cell_y = static_cast<int>(std::floor(p.y / config_.bev_resolution));
        auto key = std::make_pair(cell_x, cell_y);

        auto& cell = bev_grid[key];
        cell.point_count++;
        cell.point_indices.push_back(i);
        cell.max_hag = std::max(cell.max_hag, hag);
        cell.mean_hag = (cell.mean_hag * (cell.point_count - 1) + hag) / cell.point_count;
        cell.min_z = std::min(cell.min_z, p.z);
        cell.max_z = std::max(cell.max_z, p.z);
    }

    // 2. 过滤 cell：point_count >= bev_min_points_per_cell 且 max_hag >= min_suspended_hag
    std::set<std::pair<int,int>> valid_cells;
    for (const auto& [key, cell] : bev_grid) {
        if (cell.point_count >= config_.bev_min_points_per_cell &&
            cell.max_hag >= config_.min_suspended_hag) {
            valid_cells.insert(key);
        }
    }

    // 3. 8 邻域连通域
    std::map<std::pair<int,int>, int> cell_labels;
    int current_label = 0;

    for (const auto& cell : valid_cells) {
        if (cell_labels.find(cell) != cell_labels.end()) continue;

        // BFS
        std::queue<std::pair<int,int>> q;
        q.push(cell);
        cell_labels[cell] = current_label;

        while (!q.empty()) {
            auto current = q.front();
            q.pop();

            // 检查邻域
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;

                    // 如果是 4 邻域，跳过对角线
                    if (config_.bev_connectivity == 4 && std::abs(dx) + std::abs(dy) > 1) continue;

                    std::pair<int,int> neighbor = {current.first + dx, current.second + dy};

                    if (valid_cells.find(neighbor) != valid_cells.end() &&
                        cell_labels.find(neighbor) == cell_labels.end()) {
                        cell_labels[neighbor] = current_label;
                        q.push(neighbor);
                    }
                }
            }
        }
        current_label++;
    }

    // 4. 构建 component
    components.resize(current_label);
    for (int i = 0; i < current_label; i++) {
        components[i].id = i;
    }

    for (const auto& [cell, label] : cell_labels) {
        auto& comp = components[label];
        comp.cells.insert(cell);

        // 添加该 cell 的所有点
        auto it = bev_grid.find(cell);
        if (it != bev_grid.end()) {
            for (int idx : it->second.point_indices) {
                comp.points->push_back(points->points[idx]);
            }
            comp.max_hag = std::max(comp.max_hag, it->second.max_hag);
        }
    }

    // 5. 过滤太小的 component
    std::vector<BevComponent> filtered_components;
    for (auto& comp : components) {
        if (comp.cells.size() >= static_cast<size_t>(config_.bev_min_component_cells)) {
            comp.point_count = comp.points->size();

            // 计算质心
            if (comp.point_count > 0) {
                Eigen::Vector3f sum = Eigen::Vector3f::Zero();
                for (const auto& p : comp.points->points) {
                    sum += Eigen::Vector3f(p.x, p.y, p.z);
                }
                comp.centroid = sum / comp.point_count;
            }

            filtered_components.push_back(comp);
        }
    }

    return filtered_components;
}

// ========== HAG z-density slicing ==========

bool CargoBoxEstimator::findDenseBandByHagHistogram(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
    float ground_z,
    float& band_min_hag,
    float& band_max_hag,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& core_points_out) {

    if (!points || points->size() < static_cast<size_t>(config_.min_z_band_points)) {
        return false;
    }

    // 1. 计算每个点的 HAG
    std::vector<float> hag_values;
    for (const auto& p : points->points) {
        float hag = p.z - ground_z;
        if (hag >= config_.min_hag_for_candidate) {
            hag_values.push_back(hag);
        }
    }

    if (hag_values.size() < static_cast<size_t>(config_.min_z_band_points)) {
        ROS_WARN("[CargoBoxV2] FAIL: hag_values.size()=%zu < min_z_band_points=%d (input=%zu)",
                  hag_values.size(), config_.min_z_band_points, points->size());
        return false;
    }

    // 2. 找到 HAG 的范围
    float hag_min = *std::min_element(hag_values.begin(), hag_values.end());
    float hag_max = *std::max_element(hag_values.begin(), hag_values.end());

    // 3. 创建 HAG histogram
    int num_bins = static_cast<int>(std::ceil((hag_max - hag_min) / config_.z_hist_bin)) + 1;
    std::vector<int> histogram(num_bins, 0);

    for (float hag : hag_values) {
        int bin = static_cast<int>((hag - hag_min) / config_.z_hist_bin);
        if (bin >= 0 && bin < num_bins) {
            histogram[bin]++;
        }
    }

    // 4. 找到最大 bin 的计数
    int max_bin_count = *std::max_element(histogram.begin(), histogram.end());

    // 5. 计算 dense threshold
    int dense_thresh = std::max(config_.dense_bin_min_points,
                                static_cast<int>(max_bin_count * config_.dense_bin_ratio));

    // 6. 找连续 dense band
    int best_start = 0, best_end = 0, best_count = 0;
    int current_start = -1, current_count = 0;

    for (int i = 0; i < num_bins; i++) {
        if (histogram[i] >= dense_thresh) {
            if (current_start < 0) {
                current_start = i;
            }
            current_count += histogram[i];

            if (current_count > best_count) {
                best_count = current_count;
                best_start = current_start;
                best_end = i;
            }
        } else {
            current_start = -1;
            current_count = 0;
        }
    }

    if (best_count < config_.min_z_band_points) {
        ROS_WARN("[CargoBoxV2] FAIL: best_count=%d < min_z_band_points=%d (hag_vals=%zu, bins=%d, dense_thresh=%d)",
                  best_count, config_.min_z_band_points, hag_values.size(), num_bins, dense_thresh);
        return false;
    }

    // 7. 转换回 HAG 坐标
    band_min_hag = hag_min + best_start * config_.z_hist_bin;
    band_max_hag = hag_min + (best_end + 1) * config_.z_hist_bin;

    // 检查 dense band 高度范围（带 epsilon 的闭区间）
    float band_height = band_max_hag - band_min_hag;
    const float eps = 1e-3f;
    if (band_height + eps < config_.min_dense_band_height ||
        band_height - eps > config_.max_dense_band_height) {
        ROS_DEBUG("[CargoBoxV2] band_height=%.3f range=[%.3f, %.3f] band=[%.2f, %.2f] accepted=0",
                  band_height, config_.min_dense_band_height, config_.max_dense_band_height,
                  band_min_hag, band_max_hag);
        return false;
    }

    // 8. 只用 dense band 内的点作为 core_points
    core_points_out.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : points->points) {
        float hag = p.z - ground_z;
        if (hag >= band_min_hag && hag <= band_max_hag) {
            core_points_out->push_back(p);
        }
    }

    if (core_points_out->size() < static_cast<size_t>(config_.min_core_points)) {
        ROS_WARN("[CargoBoxV2] FAIL: core_points_out.size()=%zu < min_core_points=%d (band=[%.2f,%.2f])",
                  core_points_out->size(), config_.min_core_points, band_min_hag, band_max_hag);
        return false;
    }
    return true;
}

// ========== 框验证 ==========

RejectReason CargoBoxEstimator::validateBox(
    const CargoBox& box,
    int core_points_count,
    float bottom_hag) {

    // v14: 检查绝对尺寸 - 拆分具体原因
    if (box.size.x() > config_.max_core_length) {
        return RejectReason::TOO_LARGE_X;
    }
    if (box.size.y() > config_.max_core_width) {
        return RejectReason::TOO_LARGE_Y;
    }
    if (box.size.z() > config_.max_core_height) {
        return RejectReason::TOO_LARGE_Z;
    }

    // v14: 检查最小尺寸 - 拆分具体原因
    if (box.size.x() < config_.min_core_length) {
        return RejectReason::TOO_SMALL_X;
    }
    if (box.size.y() < config_.min_core_width) {
        return RejectReason::TOO_SMALL_Y;
    }
    if (box.size.z() < config_.min_core_height) {
        return RejectReason::TOO_SMALL_Z;
    }

    // 检查面积
    float area = box.size.x() * box.size.y();
    if (area > config_.init_max_area) {
        return RejectReason::INIT_AREA;
    }

    // 检查长宽比
    float aspect = std::max(box.size.x(), box.size.y()) /
                   std::max(std::min(box.size.x(), box.size.y()), 0.1f);
    if (aspect > config_.init_max_aspect_ratio) {
        return RejectReason::ASPECT_RATIO;
    }

    // v14: 检查悬空高度 - 改为 GROUND_TOUCH
    if (bottom_hag < config_.init_min_bottom_hag) {
        return RejectReason::GROUND_TOUCH;
    }

    // 检查 core points 数量
    if (core_points_count < config_.init_min_core_points) {
        return RejectReason::LOW_CORE_POINTS;
    }

    return RejectReason::NONE;
}

// ========== 主函数 ==========

void CargoBoxEstimator::computeCoreBbox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
    float ground_z,
    CargoBox& box) {

    if (points->empty()) {
        box.valid = false;
        return;
    }

    // 提取 x, y, z 值
    std::vector<float> xs, ys, zs;
    for (const auto& p : points->points) {
        xs.push_back(p.x);
        ys.push_back(p.y);
        zs.push_back(p.z);
    }

    // 使用分位数计算 bbox
    float x_min = percentile(xs, config_.xy_percentile_low);
    float x_max = percentile(xs, config_.xy_percentile_high);
    float y_min = percentile(ys, config_.xy_percentile_low);
    float y_max = percentile(ys, config_.xy_percentile_high);
    float z_min = percentile(zs, config_.z_percentile_low);
    float z_max = percentile(zs, config_.z_percentile_high);

    // z_bottom 不能低于 local_ground_z + min_payload_bottom_hag
    z_min = std::max(z_min, ground_z + config_.min_payload_bottom_hag);

    // 计算中心和尺寸
    box.center = Eigen::Vector3f((x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2);
    box.size = Eigen::Vector3f(x_max - x_min, y_max - y_min, z_max - z_min);
    box.bbox_min = Eigen::Vector3f(x_min, y_min, z_min);
    box.bbox_max = Eigen::Vector3f(x_max, y_max, z_max);
    box.valid = true;
}

void CargoBoxEstimator::computeCraneAxisOBB(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
    Eigen::Vector3f& center,
    Eigen::Vector3f& size) {

    // 天车轴向约束 OBB：只允许沿 0° 和 90° 方向
    // 使用分位数而不是 min/max，避免离群点撑大框体

    if (points->empty()) return;

    // 提取 x, y 坐标
    std::vector<float> xs, ys;
    for (const auto& p : points->points) {
        xs.push_back(p.x);
        ys.push_back(p.y);
    }

    // 方向 1: 0° (X 轴) - 使用分位数
    float x_min_0 = percentile(xs, config_.xy_percentile_low);
    float x_max_0 = percentile(xs, config_.xy_percentile_high);
    float y_min_0 = percentile(ys, config_.xy_percentile_low);
    float y_max_0 = percentile(ys, config_.xy_percentile_high);
    float area_0 = (x_max_0 - x_min_0) * (y_max_0 - y_min_0);

    // 方向 2: 90° (Y 轴) - 交换 X 和 Y
    std::vector<float> rotated_xs, rotated_ys;
    for (const auto& p : points->points) {
        // 旋转 90°: (x,y) -> (-y,x)
        rotated_xs.push_back(-p.y);
        rotated_ys.push_back(p.x);
    }
    float x_min_90 = percentile(rotated_xs, config_.xy_percentile_low);
    float x_max_90 = percentile(rotated_xs, config_.xy_percentile_high);
    float y_min_90 = percentile(rotated_ys, config_.xy_percentile_low);
    float y_max_90 = percentile(rotated_ys, config_.xy_percentile_high);
    float area_90 = (x_max_90 - x_min_90) * (y_max_90 - y_min_90);

    // 计算质心（用于 z）
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& p : points->points) {
        centroid += Eigen::Vector3f(p.x, p.y, p.z);
    }
    centroid /= points->size();

    // 选择面积更小的方向
    if (area_0 <= area_90) {
        // 使用 0° 方向
        center = Eigen::Vector3f((x_min_0 + x_max_0) / 2, (y_min_0 + y_max_0) / 2, centroid.z());
        size = Eigen::Vector3f(x_max_0 - x_min_0, y_max_0 - y_min_0, 0);
    } else {
        // 使用 90° 方向，需要旋转回
        float cx = (x_min_90 + x_max_90) / 2;
        float cy = (y_min_90 + y_max_90) / 2;
        // 旋转回: (x,y) -> (y,-x)
        center = Eigen::Vector3f(cy, -cx, centroid.z());
        size = Eigen::Vector3f(x_max_90 - x_min_90, y_max_90 - y_min_90, 0);
    }
}

void CargoBoxEstimator::expandBox(
    CargoBox& box,
    float xy_expand,
    float z_down_expand,
    float z_up_expand) {

    box.bbox_min.x() -= xy_expand;
    box.bbox_min.y() -= xy_expand;
    box.bbox_min.z() -= z_down_expand;

    box.bbox_max.x() += xy_expand;
    box.bbox_max.y() += xy_expand;
    box.bbox_max.z() += z_up_expand;

    // 更新中心和尺寸
    box.center = (box.bbox_min + box.bbox_max) / 2;
    box.size = box.bbox_max - box.bbox_min;
}

bool CargoBoxEstimator::estimateCargoBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
    const SimpleGroundModel& ground_model,
    CargoBox& core_box,
    CargoBox& remove_box,
    CargoBox& forbidden_box,
    const CargoBox* prev_core_box) {

    if (!config_.enabled || !cluster || cluster->size() < static_cast<size_t>(config_.min_cluster_points)) {
        return false;
    }

    // 初始化调试点云
    hag_filtered_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    core_points_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    rejected_low_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    components_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    int input_pts = cluster->size();

    // ========== Step 1: BEV 连通域拆分 ==========
    auto components = buildBevComponents(cluster, ground_model);

    ROS_DEBUG("[CargoBoxV2] input_pts=%d, bev_components=%zu", input_pts, components.size());

    if (components.empty()) {
        ROS_WARN("[CargoBoxV2] FAIL: No valid BEV components (input=%d)", input_pts);
        return false;
    }

    // 生成 component 可视化（不同颜色）
    // 这里简单用 z 值区分颜色
    for (const auto& comp : components) {
        for (const auto& p : comp.points->points) {
            pcl::PointXYZ colored_p = p;
            colored_p.z = comp.id * 0.5f;  // 用 z 值区分 component
            components_cloud_->push_back(colored_p);
        }
    }

    // ========== Step 2: 选择最像吊货的 component ==========
    int selected_index = -1;  // 在 components 数组中的索引
    float best_score = -1.0f;

    for (size_t i = 0; i < components.size(); i++) {
        const auto& comp = components[i];
        // 计算 component 的质量分数
        float score = 0.0f;

        // 点数分数（适中的点数更好）
        float pts_ratio = static_cast<float>(comp.point_count) / input_pts;
        score += std::min(pts_ratio * 10.0f, 3.0f);

        // HAG 分数（更高的 HAG 更可能是吊货）
        score += std::min(comp.max_hag * 2.0f, 4.0f);

        // 紧凑度分数（更紧凑的 component 更好）
        if (comp.points->size() > 1) {
            Eigen::Vector3f min_pt, max_pt;
            min_pt = Eigen::Vector3f(1e9, 1e9, 1e9);
            max_pt = Eigen::Vector3f(-1e9, -1e9, -1e9);
            for (const auto& p : comp.points->points) {
                min_pt.x() = std::min(min_pt.x(), p.x);
                min_pt.y() = std::min(min_pt.y(), p.y);
                min_pt.z() = std::min(min_pt.z(), p.z);
                max_pt.x() = std::max(max_pt.x(), p.x);
                max_pt.y() = std::max(max_pt.y(), p.y);
                max_pt.z() = std::max(max_pt.z(), p.z);
            }
            Eigen::Vector3f size = max_pt - min_pt;
            float volume = size.x() * size.y() * size.z();
            float density = comp.point_count / std::max(volume, 0.01f);
            score += std::min(density * 0.1f, 3.0f);
        }

        // P0-7: track 一致性分数
        if (prev_core_box && prev_core_box->valid) {
            // 计算 component centroid 与上一帧 core_box 的距离
            float dist = (comp.centroid - prev_core_box->center).norm();
            // 距离越近，分数越高（最多加 5 分）
            float consistency_score = std::max(0.0f, 5.0f - dist * 2.0f);
            score += consistency_score;
        }

        if (score > best_score) {
            best_score = score;
            selected_index = i;
        }
    }

    if (selected_index < 0) {
        ROS_WARN("[CargoBoxV2] FAIL: No suitable component selected (comps=%zu)", components.size());
        return false;
    }

    const auto& selected_component = components[selected_index];
    ROS_DEBUG("[CargoBoxV2] selected_component=%d, pts=%d, max_hag=%.2f, score=%.2f",
             selected_component.id, selected_component.point_count,
             selected_component.max_hag, best_score);

    // ========== Step 3: HAG z-density slicing ==========
    float ground_z = ground_model.getGroundZ(selected_component.centroid.x(), selected_component.centroid.y());

    // 保存 HAG 过滤后的点云
    *hag_filtered_cloud_ = *selected_component.points;

    float band_min_hag, band_max_hag;
    pcl::PointCloud<pcl::PointXYZ>::Ptr core_points(new pcl::PointCloud<pcl::PointXYZ>);

    if (!findDenseBandByHagHistogram(selected_component.points, ground_z,
                                     band_min_hag, band_max_hag, core_points)) {
        ROS_DEBUG("[CargoBoxV2] Failed to find dense band");
        core_box.reject_reason = RejectReason::NO_DENSE_BAND;
        return false;
    }

    // 保存 core points
    *core_points_cloud_ = *core_points;

    int core_pts = core_points->size();
    ROS_DEBUG("[CargoBoxV2] core_pts=%d, band_hag=[%.2f, %.2f], ground_z=%.2f",
             core_pts, band_min_hag, band_max_hag, ground_z);

    // ========== Step 4: 计算 core box ==========
    if (config_.use_crane_axis_obb) {
        // 使用轴向约束 OBB
        Eigen::Vector3f obb_center, obb_size;
        computeCraneAxisOBB(core_points, obb_center, obb_size);

        // 使用分位数计算 z
        std::vector<float> zs;
        for (const auto& p : core_points->points) {
            zs.push_back(p.z);
        }
        float z_min = percentile(zs, config_.z_percentile_low);
        float z_max = percentile(zs, config_.z_percentile_high);
        z_min = std::max(z_min, ground_z + config_.min_payload_bottom_hag);

        core_box.center = Eigen::Vector3f(obb_center.x(), obb_center.y(), (z_min + z_max) / 2);
        core_box.size = Eigen::Vector3f(obb_size.x(), obb_size.y(), z_max - z_min);
        core_box.bbox_min = Eigen::Vector3f(
            core_box.center.x() - core_box.size.x() / 2,
            core_box.center.y() - core_box.size.y() / 2,
            z_min);
        core_box.bbox_max = Eigen::Vector3f(
            core_box.center.x() + core_box.size.x() / 2,
            core_box.center.y() + core_box.size.y() / 2,
            z_max);
    } else {
        // 使用普通分位数 bbox
        computeCoreBbox(core_points, ground_z, core_box);
    }

    // 计算 bottom_hag
    float bottom_hag = core_box.bbox_min.z() - ground_z;

    // ========== Step 5: 验证框 ==========
    RejectReason reject_reason = validateBox(core_box, core_pts, bottom_hag);
    if (reject_reason != RejectReason::NONE) {
        ROS_WARN("[CargoBoxV2] Rejected: reason=%d, size=(%.2f,%.2f,%.2f), pts=%d, hag=%.2f",
                 (int)reject_reason, core_box.size.x(), core_box.size.y(), core_box.size.z(),
                 core_pts, bottom_hag);
        core_box.reject_reason = reject_reason;
        return false;
    }

    // ========== Step 6: 尺寸增长率检查（已移到调用方 per-track 检查） ==========
    // P0-1: size jump 检查现在由调用方在 per-track 基础上执行
    // CargoBoxEstimator 不再维护任何帧间状态

    // ========== Step 7: 设置 core box 属性 ==========
    core_box.valid = true;
    core_box.type = CargoBoxType::CORE;
    core_box.reject_reason = RejectReason::NONE;
    core_box.suspended_points = core_pts;
    core_box.bottom_hag = bottom_hag;
    core_box.z_band_min = band_min_hag + ground_z;
    core_box.z_band_max = band_max_hag + ground_z;
    core_box.component_id = selected_component.id;
    core_box.component_count = components.size();

    // ========== Step 8: 生成 remove box ==========
    remove_box = core_box;
    remove_box.type = CargoBoxType::REMOVE;
    expandBox(remove_box, config_.remove_expand_xy, config_.remove_expand_z_down, config_.remove_expand_z_up);

    // ========== Step 9: 生成 forbidden zone ==========
    forbidden_box = core_box;
    forbidden_box.type = CargoBoxType::FORBIDDEN;
    forbidden_box.bbox_min.z() = ground_z;  // 扩展到地面
    forbidden_box.bbox_max.z() = ground_z + config_.forbidden_height;
    expandBox(forbidden_box, config_.forbidden_expand_xy, 0, 0);

    ROS_DEBUG("[CargoBoxV2] core_box: size=(%.2f,%.2f,%.2f), bottom_hag=%.2f, "
             "remove_box: size=(%.2f,%.2f,%.2f), forbidden_zone: size=(%.2f,%.2f,%.2f)",
             core_box.size.x(), core_box.size.y(), core_box.size.z(), bottom_hag,
             remove_box.size.x(), remove_box.size.y(), remove_box.size.z(),
             forbidden_box.size.x(), forbidden_box.size.y(), forbidden_box.size.z());

    return true;
}

} // namespace ndt_slam
