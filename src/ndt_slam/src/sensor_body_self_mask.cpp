#include <ndt_slam/sensor_body_self_mask.hpp>

#include <algorithm>
#include <cmath>

namespace ndt_slam {

void SensorBodySelfMask::configure(const SensorBodySelfMaskConfig& config) {
    config_ = config;
    config_.half_extent = config_.half_extent.cwiseMax(Eigen::Vector3f::Zero());
    config_.maximum_removed_ratio = std::clamp(
        config_.maximum_removed_ratio, 0.0, 1.0);
}

const SensorBodySelfMaskConfig& SensorBodySelfMask::config() const {
    return config_;
}

SensorBodySelfMaskResult SensorBodySelfMask::filter(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::string& cloud_frame) const {
    SensorBodySelfMaskResult result;
    result.input_points = cloud.size();
    result.commissioned = config_.commissioned;
    result.frame_matches = !config_.frame_id.empty() &&
        cloud_frame == config_.frame_id;
    const bool geometry_configured =
        config_.half_extent.allFinite() &&
        config_.half_extent.minCoeff() > 0.0F &&
        std::isfinite(config_.yaw_rad) &&
        std::isfinite(config_.maximum_removed_ratio) &&
        config_.maximum_removed_ratio > 0.0 &&
        config_.maximum_removed_ratio < 1.0;
    result.kept->reserve(cloud.size());
    result.removed->reserve(cloud.size());

    if (!config_.enabled || !config_.commissioned ||
        !geometry_configured || !result.frame_matches) {
        *result.kept = cloud;
        result.ratio_within_limit = true;
        // Disabled is a preview-only state. The fail-closed mapping profile
        // must never treat a missing mask as commissioned calibration.
        result.mapping_ready = false;
        result.reason = !config_.enabled
            ? "disabled"
            : (!config_.commissioned
                ? "self_mask_not_commissioned"
                : (!geometry_configured
                    ? "self_mask_geometry_invalid"
                    : "self_mask_frame_mismatch"));
        return result;
    }

    const float cosine = std::cos(config_.yaw_rad);
    const float sine = std::sin(config_.yaw_rad);
    for (const pcl::PointXYZ& point : cloud.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            result.kept->push_back(point);
            continue;
        }
        const Eigen::Vector3f relative(
            point.x - config_.center.x(),
            point.y - config_.center.y(),
            point.z - config_.center.z());
        // Rotate into the OBB axes (inverse yaw).
        const float local_x = cosine * relative.x() + sine * relative.y();
        const float local_y = -sine * relative.x() + cosine * relative.y();
        const bool inside =
            std::abs(local_x) <= config_.half_extent.x() &&
            std::abs(local_y) <= config_.half_extent.y() &&
            std::abs(relative.z()) <= config_.half_extent.z();
        (inside ? result.removed : result.kept)->push_back(point);
    }

    result.removed_ratio = result.input_points == 0U
        ? 0.0
        : static_cast<double>(result.removed->size()) /
            static_cast<double>(result.input_points);
    result.ratio_within_limit =
        result.removed_ratio <= config_.maximum_removed_ratio;
    result.mapping_ready = result.ratio_within_limit;
    result.reason = result.ratio_within_limit
        ? "commissioned"
        : "self_mask_removed_ratio_exceeded";
    return result;
}

}  // namespace ndt_slam
