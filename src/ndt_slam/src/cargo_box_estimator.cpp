#include <ndt_slam/cargo_box_estimator.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <numeric>

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

            config_.min_hag_for_candidate = cb["min_hag_for_candidate"].as<double>(0.25);
            config_.min_payload_bottom_hag = cb["min_payload_bottom_hag"].as<double>(0.45);
            config_.z_hist_bin = cb["z_hist_bin"].as<double>(0.10);
            config_.min_z_band_points = cb["min_z_band_points"].as<int>(20);

            config_.xy_percentile_low = cb["xy_percentile_low"].as<double>(2.0);
            config_.xy_percentile_high = cb["xy_percentile_high"].as<double>(98.0);
            config_.z_percentile_low = cb["z_percentile_low"].as<double>(5.0);
            config_.z_percentile_high = cb["z_percentile_high"].as<double>(98.0);

            config_.use_crane_axis_obb = cb["use_crane_axis_obb"].as<bool>(true);

            config_.max_width = cb["max_width"].as<double>(6.0);
            config_.max_length = cb["max_length"].as<double>(16.0);
            config_.max_height = cb["max_height"].as<double>(5.0);

            config_.core_expand_xy = cb["core_expand_xy"].as<double>(0.05);
            config_.core_expand_z_down = cb["core_expand_z_down"].as<double>(0.03);
            config_.core_expand_z_up = cb["core_expand_z_up"].as<double>(0.10);

            config_.remove_expand_xy = cb["remove_expand_xy"].as<double>(0.25);
            config_.remove_expand_z_down = cb["remove_expand_z_down"].as<double>(0.05);
            config_.remove_expand_z_up = cb["remove_expand_z_up"].as<double>(0.20);

            config_.forbidden_expand_xy = cb["forbidden_expand_xy"].as<double>(0.50);
            config_.forbidden_height = cb["forbidden_height"].as<double>(3.0);

            // 尺寸增长率限制
            config_.max_size_growth_ratio = cb["max_size_growth_ratio"].as<double>(1.35);

            // 最小悬空高度
            config_.min_suspended_hag = cb["min_suspended_hag"].as<double>(0.35);
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

bool CargoBoxEstimator::findSuspendedDenseBand(
    const std::vector<float>& z_values,
    float ground_z,
    float& band_min,
    float& band_max) {

    if (z_values.size() < static_cast<size_t>(config_.min_z_band_points)) {
        return false;
    }

    // 计算 HAG (height above ground)
    std::vector<float> hag_values;
    for (float z : z_values) {
        float hag = z - ground_z;
        if (hag >= config_.min_hag_for_candidate) {
            hag_values.push_back(hag);
        }
    }

    if (hag_values.size() < static_cast<size_t>(config_.min_z_band_points)) {
        return false;
    }

    // 找到 HAG 的范围
    float hag_min = *std::min_element(hag_values.begin(), hag_values.end());
    float hag_max = *std::max_element(hag_values.begin(), hag_values.end());

    // 创建 z histogram
    int num_bins = static_cast<int>(std::ceil((hag_max - hag_min) / config_.z_hist_bin)) + 1;
    std::vector<int> histogram(num_bins, 0);

    for (float hag : hag_values) {
        int bin = static_cast<int>((hag - hag_min) / config_.z_hist_bin);
        if (bin >= 0 && bin < num_bins) {
            histogram[bin]++;
        }
    }

    // 寻找连续高密度区间
    // 策略：找到包含最多点的连续区间
    int best_start = 0, best_end = 0, best_count = 0;
    int current_start = 0, current_count = 0;

    for (int i = 0; i < num_bins; i++) {
        if (histogram[i] >= 3) {  // 至少 3 个点才算有效
            if (current_count == 0) {
                current_start = i;
            }
            current_count += histogram[i];

            if (current_count > best_count) {
                best_count = current_count;
                best_start = current_start;
                best_end = i;
            }
        } else {
            current_count = 0;
        }
    }

    if (best_count < config_.min_z_band_points) {
        return false;
    }

    // 转换回 z 坐标
    band_min = hag_min + best_start * config_.z_hist_bin + ground_z;
    band_max = hag_min + (best_end + 1) * config_.z_hist_bin + ground_z;

    return true;
}

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
    CargoBox& forbidden_box) {

    if (!config_.enabled || !cluster || cluster->size() < static_cast<size_t>(config_.min_cluster_points)) {
        return false;
    }

    // 1. 计算每个点的 HAG，并过滤
    pcl::PointCloud<pcl::PointXYZ>::Ptr suspended_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_low_points(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<float> z_values;

    // 获取地面高度
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& p : cluster->points) {
        centroid += Eigen::Vector3f(p.x, p.y, p.z);
    }
    centroid /= cluster->size();
    float ground_z = ground_model.getGroundZ(centroid.x(), centroid.y());

    for (const auto& p : cluster->points) {
        float hag = p.z - ground_z;
        if (hag >= config_.min_hag_for_candidate) {
            suspended_points->push_back(p);
            z_values.push_back(p.z);
        } else {
            removed_low_points->push_back(p);
        }
    }

    core_box.removed_low_points = removed_low_points->size();

    // 2. z histogram 寻找悬空主体层
    float band_min, band_max;
    if (!findSuspendedDenseBand(z_values, ground_z, band_min, band_max)) {
        ROS_DEBUG("[CargoBoxEstimator] Failed to find suspended dense band");
        return false;
    }

    // 3. 只用悬空主体层的点计算 core bbox
    pcl::PointCloud<pcl::PointXYZ>::Ptr core_points(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : suspended_points->points) {
        if (p.z >= band_min && p.z <= band_max) {
            core_points->push_back(p);
        }
    }

    if (core_points->size() < static_cast<size_t>(config_.min_z_band_points)) {
        ROS_DEBUG("[CargoBoxEstimator] Too few core points: %zu", core_points->size());
        return false;
    }

    // 4. 计算 core bbox
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

    // 尺寸限制
    if (core_box.size.x() > config_.max_length || core_box.size.y() > config_.max_width ||
        core_box.size.z() > config_.max_height) {
        ROS_DEBUG("[CargoBoxEstimator] Box too large: %.1f x %.1f x %.1f",
                  core_box.size.x(), core_box.size.y(), core_box.size.z());
        return false;
    }

    // 计算 bottom_hag
    float bottom_hag = core_box.bbox_min.z() - ground_z;

    // 悬空高度检查：bottom_hag 必须 >= min_suspended_hag
    if (bottom_hag < config_.min_suspended_hag) {
        ROS_DEBUG("[CargoBoxEstimator] Rejected by ground contact: bottom_hag=%.2f < %.2f",
                  bottom_hag, config_.min_suspended_hag);
        return false;
    }

    // 尺寸增长率检查：防止框体突然变大（可能是合并到旁边货物）
    if (has_last_size_) {
        float growth_x = core_box.size.x() / std::max(last_core_size_.x(), 0.1f);
        float growth_y = core_box.size.y() / std::max(last_core_size_.y(), 0.1f);
        float growth_z = core_box.size.z() / std::max(last_core_size_.z(), 0.1f);
        float max_growth = std::max({growth_x, growth_y, growth_z});

        if (max_growth > config_.max_size_growth_ratio) {
            ROS_WARN("[CargoBoxEstimator] Rejected by size jump: growth=%.2f > %.2f (last_size=(%.2f,%.2f,%.2f), new_size=(%.2f,%.2f,%.2f))",
                     max_growth, config_.max_size_growth_ratio,
                     last_core_size_.x(), last_core_size_.y(), last_core_size_.z(),
                     core_box.size.x(), core_box.size.y(), core_box.size.z());
            return false;
        }
    }

    // 更新上一帧 size
    last_core_size_ = core_box.size;
    has_last_size_ = true;

    core_box.valid = true;
    core_box.type = CargoBoxType::CORE;
    core_box.suspended_points = core_points->size();
    core_box.bottom_hag = bottom_hag;
    core_box.z_band_min = band_min;
    core_box.z_band_max = band_max;

    // 5. 计算 remove box（扩展）
    remove_box = core_box;
    remove_box.type = CargoBoxType::REMOVE;
    expandBox(remove_box, config_.remove_expand_xy, config_.remove_expand_z_down, config_.remove_expand_z_up);

    // 6. 计算 forbidden box（扩展到地面）
    forbidden_box = core_box;
    forbidden_box.type = CargoBoxType::FORBIDDEN;
    forbidden_box.bbox_min.z() = ground_z;  // 扩展到地面
    forbidden_box.bbox_max.z() = ground_z + config_.forbidden_height;
    expandBox(forbidden_box, config_.forbidden_expand_xy, 0, 0);

    ROS_INFO("[CargoBoxEstimator] core pts=%zu, bottom_hag=%.2f, size=(%.2f,%.2f,%.2f), "
             "z_band=(%.2f,%.2f), removed_low=%d, ground_z=%.2f",
             core_points->size(), core_box.bottom_hag,
             core_box.size.x(), core_box.size.y(), core_box.size.z(),
             band_min, band_max, core_box.removed_low_points, ground_z);

    return true;
}

} // namespace ndt_slam
