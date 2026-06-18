#include <ndt_slam/payload_gate.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

namespace ndt_slam {

void PayloadCandidateGate::configure(const PayloadGateConfig& config) {
    config_ = config;
}

void PayloadCandidateGate::configureFromYaml(const std::string& config_file) {
    try {
        YAML::Node yaml = YAML::LoadFile(config_file);
        auto pg = yaml["payload_gate"];
        if (pg) {
            config_.enabled = pg["enabled"].as<bool>(true);

            auto roi = pg["roi"];
            if (roi) {
                config_.roi_x_min = roi["x_min"].as<double>(-50.0);
                config_.roi_x_max = roi["x_max"].as<double>(50.0);
                config_.roi_y_min = roi["y_min"].as<double>(-5.0);
                config_.roi_y_max = roi["y_max"].as<double>(5.0);
            }
        }
    } catch (const std::exception& e) {
        ROS_WARN("[PayloadGate] Failed to load config: %s, using defaults", e.what());
    }
}

void PayloadCandidateGate::updateHookState(
    const PointCloudProcessing::HookState& hook_state) {
    hook_state_ = hook_state;
}

GatedObjects PayloadCandidateGate::gate(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& objects_cloud) {

    GatedObjects result;
    result.static_objects.reset(new pcl::PointCloud<pcl::PointXYZ>);
    result.payload_candidates.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (!config_.enabled || !objects_cloud || objects_cloud->empty()) {
        if (objects_cloud) *result.static_objects = *objects_cloud;
        return result;
    }

    // PLC 扩展：如果有吊钩状态，使用吊钩 ROI 替代固定 ROI
    bool use_plc_roi = hook_state_.valid && hook_state_.is_carrying;
    Eigen::Vector3d hook_pos = hook_state_.position;
    double half_l = hook_state_.payload_length / 2.0 + 0.5;
    double half_w = hook_state_.payload_width / 2.0 + 0.5;
    double z_min = hook_pos.z() - hook_state_.payload_height;
    double z_max = hook_pos.z() + 1.0;

    for (const auto& p : objects_cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;

        bool in_roi = false;

        if (use_plc_roi) {
            // PLC 模式：使用吊钩位置 + 货物尺寸
            double dx = p.x - hook_pos.x();
            double dy = p.y - hook_pos.y();
            in_roi = (std::abs(dx) < half_l &&
                      std::abs(dy) < half_w &&
                      p.z > z_min && p.z < z_max);
        } else {
            // 点云自检测模式：使用固定 ROI
            in_roi = (p.x >= config_.roi_x_min && p.x <= config_.roi_x_max &&
                      p.y >= config_.roi_y_min && p.y <= config_.roi_y_max);
        }

        if (in_roi) {
            result.payload_candidates->push_back(p);
        } else {
            result.static_objects->push_back(p);
        }
    }

    return result;
}

} // namespace ndt_slam
