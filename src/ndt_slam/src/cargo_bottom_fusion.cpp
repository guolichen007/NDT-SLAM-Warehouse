#include "ndt_slam/cargo_bottom_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <utility>

namespace ndt_slam {
namespace {

bool finitePoint(const Eigen::Vector3f& point) {
    return point.allFinite();
}

bool validRigidTransform(const Eigen::Isometry3f& transform) {
    if (!transform.matrix().allFinite()) return false;
    const Eigen::Matrix3f rotation = transform.linear();
    const Eigen::Matrix3f orthogonality =
        rotation.transpose() * rotation - Eigen::Matrix3f::Identity();
    return orthogonality.cwiseAbs().maxCoeff() <= 1.0e-3F &&
           std::abs(rotation.determinant() - 1.0F) <= 1.0e-3F;
}

bool finiteNonNegative(float value) {
    return std::isfinite(value) && value >= 0.0F;
}

bool finiteNonNegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

bool unitInterval(float value) {
    return std::isfinite(value) && value >= 0.0F && value <= 1.0F;
}

bool validConfig(const CargoBottomFusionConfig& config, std::string* reason) {
    auto reject = [&](const char* field) {
        if (reason != nullptr) *reason = field;
        return false;
    };
    if (!finiteNonNegative(config.accumulation_window_sec))
        return reject("accumulation_window_sec");
    if (!std::isfinite(config.stale_reset_sec) || config.stale_reset_sec <= 0.0)
        return reject("stale_reset_sec");
    if (!finiteNonNegative(config.stable_hold_sec))
        return reject("stable_hold_sec");
    if (!finiteNonNegative(config.max_transform_skew_sec))
        return reject("max_transform_skew_sec");
    if (!finiteNonNegative(config.backwards_tolerance_sec))
        return reject("backwards_tolerance_sec");
    if (config.max_accumulated_points == 0U)
        return reject("max_accumulated_points");
    if (config.points_min_points == 0U)
        return reject("minimum_points");
    if (!finiteNonNegative(config.points_min_visible_height))
        return reject("minimum_visible_height");
    if (config.points_min_bottom_band_points == 0U ||
        config.points_min_bottom_band_xy_cells == 0U)
        return reject("minimum_bottom_support");
    if (!unitInterval(config.points_min_bottom_band_point_ratio) ||
        !unitInterval(config.points_min_bottom_band_xy_cell_ratio) ||
        !unitInterval(config.points_min_bottom_span_ratio))
        return reject("bottom_support_ratio");
    if (config.map_diff_min_points == 0U ||
        config.map_static_min_points == 0U ||
        !finiteNonNegative(config.map_min_visible_height) ||
        config.map_min_bottom_band_points == 0U ||
        config.map_min_bottom_band_xy_cells == 0U)
        return reject("map_minimum_support");
    if (!unitInterval(config.map_min_bottom_band_point_ratio) ||
        !unitInterval(config.map_min_bottom_band_xy_cell_ratio))
        return reject("map_support_ratio");
    if (!std::isfinite(config.bottom_band_height) ||
        config.bottom_band_height <= 0.0F ||
        !std::isfinite(config.xy_cell_size) || config.xy_cell_size <= 0.0F ||
        config.points_min_vertical_bins == 0U ||
        !std::isfinite(config.points_vertical_bin_size) ||
        config.points_vertical_bin_size <= 0.0F ||
        !finiteNonNegative(config.points_max_vertical_gap) ||
        !finiteNonNegative(config.prior_height_tolerance))
        return reject("support_geometry");
    if (!finiteNonNegative(config.footprint_margin) ||
        !std::isfinite(config.min_footprint_size) ||
        config.min_footprint_size <= 0.0F ||
        !std::isfinite(config.max_footprint_size) ||
        config.max_footprint_size < config.min_footprint_size)
        return reject("footprint_size");
    if (!std::isfinite(config.invalid_uncertainty) ||
        config.invalid_uncertainty <= 0.0F ||
        !finiteNonNegative(config.points_uncertainty_min) ||
        !finiteNonNegative(config.map_diff_uncertainty_min) ||
        !finiteNonNegative(config.map_static_uncertainty_min) ||
        !finiteNonNegative(config.origin_height_uncertainty_min) ||
        !finiteNonNegative(config.recent_stable_uncertainty_min) ||
        config.points_uncertainty_min > config.invalid_uncertainty ||
        config.map_diff_uncertainty_min > config.invalid_uncertainty ||
        config.map_static_uncertainty_min > config.invalid_uncertainty ||
        config.origin_height_uncertainty_min > config.invalid_uncertainty ||
        config.recent_stable_uncertainty_min > config.invalid_uncertainty)
        return reject("uncertainty_bounds");
    if (!finiteNonNegative(config.tail_uncertainty_gain) ||
        !finiteNonNegative(config.sparse_uncertainty_gain) ||
        !finiteNonNegative(config.stable_age_uncertainty_per_sec))
        return reject("uncertainty_gain");
    if (!unitInterval(config.points_confidence_base) ||
        !unitInterval(config.map_diff_confidence_base) ||
        !unitInterval(config.map_static_confidence_base) ||
        !unitInterval(config.origin_height_confidence_base) ||
        !unitInterval(config.recent_stable_confidence_base) ||
        !unitInterval(config.ema_alpha))
        return reject("confidence_or_ema");
    if (!finiteNonNegative(config.direct_update_max_jump) ||
        !finiteNonNegative(config.soft_update_max_jump) ||
        config.soft_update_max_jump < config.direct_update_max_jump ||
        !unitInterval(config.soft_update_alpha) ||
        config.large_jump_confirm_frames == 0U ||
        !finiteNonNegative(config.large_jump_confirmation_tolerance))
        return reject("source_transition_gate");
    if (!std::isfinite(config.minimum_prior_height) ||
        !std::isfinite(config.maximum_prior_height) ||
        config.minimum_prior_height <= 0.0F ||
        config.maximum_prior_height < config.minimum_prior_height)
        return reject("prior_height_bounds");
    return true;
}

float percentile(std::vector<float> values, float q) {
    if (values.empty()) {
        return 0.0F;
    }
    std::sort(values.begin(), values.end());
    const float position = std::clamp(q, 0.0F, 1.0F) *
                           static_cast<float>(values.size() - 1U);
    const std::size_t low = static_cast<std::size_t>(std::floor(position));
    const std::size_t high = static_cast<std::size_t>(std::ceil(position));
    const float fraction = position - static_cast<float>(low);
    return values[low] * (1.0F - fraction) + values[high] * fraction;
}

std::vector<Eigen::Vector3f> finitePoints(
    const std::vector<Eigen::Vector3f>& points) {
    std::vector<Eigen::Vector3f> output;
    output.reserve(points.size());
    for (const auto& point : points) {
        if (finitePoint(point)) {
            output.push_back(point);
        }
    }
    return output;
}

void limitPointsDeterministically(std::vector<Eigen::Vector3f>* points,
                                  std::size_t limit) {
    if (points == nullptr || points->size() <= limit) {
        return;
    }
    if (limit == 0U) {
        points->clear();
        return;
    }
    std::vector<Eigen::Vector3f> source = std::move(*points);
    points->clear();
    points->reserve(limit);
    if (limit == 1U) {
        points->push_back(source.front());
        return;
    }
    const long double source_span =
        static_cast<long double>(source.size() - 1U);
    const long double output_span = static_cast<long double>(limit - 1U);
    for (std::size_t i = 0; i < limit; ++i) {
        const std::size_t index = static_cast<std::size_t>(std::floor(
            static_cast<long double>(i) * source_span / output_span));
        points->push_back(source[index]);
    }
}

void filterToFootprint(std::vector<Eigen::Vector3f>* points,
                       const CargoBottomObservation& observation,
                       float margin) {
    if (!observation.footprint_valid ||
        !observation.footprint_center_base.allFinite() ||
        !observation.footprint_size_xy.allFinite() ||
        observation.footprint_size_xy.minCoeff() <= 0.0F) {
        return;
    }
    const Eigen::Vector2f half =
        0.5F * observation.footprint_size_xy.array().abs().matrix() +
        Eigen::Vector2f::Constant(std::max(0.0F, margin));
    const float cosine = std::cos(observation.footprint_yaw_base_rad);
    const float sine = std::sin(observation.footprint_yaw_base_rad);
    points->erase(
        std::remove_if(points->begin(), points->end(), [&](const Eigen::Vector3f& point) {
            const Eigen::Vector2f delta = point.head<2>() -
                observation.footprint_center_base;
            const Eigen::Vector2f local(
                cosine * delta.x() + sine * delta.y(),
                -sine * delta.x() + cosine * delta.y());
            return std::abs(local.x()) > half.x() ||
                std::abs(local.y()) > half.y();
        }),
        points->end());
}

CargoVerticalStats analyzeVertical(
    const std::vector<Eigen::Vector3f>& input,
    float bottom_band_height,
    float xy_cell_size,
    std::size_t min_points,
    float min_visible_height,
    std::size_t min_band_points,
    std::size_t min_band_cells,
    float min_band_point_ratio,
    float min_band_cell_ratio,
    const Eigen::Vector2f& footprint_center_xy,
    const Eigen::Vector2f& footprint_size_xy,
    float footprint_yaw_rad,
    float min_bottom_span_ratio,
    std::size_t min_vertical_bins,
    float vertical_bin_size,
    float max_vertical_gap,
    bool prior_height_valid,
    float prior_height,
    float prior_height_tolerance) {
    CargoVerticalStats stats;
    const auto points = finitePoints(input);
    stats.finite_points = points.size();
    if (points.size() < min_points || xy_cell_size <= 0.0F ||
        bottom_band_height <= 0.0F) {
        stats.reject_reason = points.size() < min_points
            ? "too_few_points"
            : "invalid_support_config";
        return stats;
    }

    std::vector<float> z_values;
    z_values.reserve(points.size());
    std::set<std::pair<int, int>> all_cells;
    Eigen::Vector2f all_min = Eigen::Vector2f::Constant(
        std::numeric_limits<float>::infinity());
    Eigen::Vector2f all_max = Eigen::Vector2f::Constant(
        -std::numeric_limits<float>::infinity());
    const bool oriented_coordinates = footprint_center_xy.allFinite() &&
        footprint_size_xy.minCoeff() > 0.0F &&
        std::isfinite(footprint_yaw_rad);
    const float cosine = std::cos(footprint_yaw_rad);
    const float sine = std::sin(footprint_yaw_rad);
    const auto footprintLocal = [&](const Eigen::Vector3f& point) {
        if (!oriented_coordinates) return point.head<2>().eval();
        const Eigen::Vector2f delta =
            point.head<2>() - footprint_center_xy;
        return Eigen::Vector2f(
            cosine * delta.x() + sine * delta.y(),
            -sine * delta.x() + cosine * delta.y());
    };
    for (const auto& point : points) {
        z_values.push_back(point.z());
        const Eigen::Vector2f local = footprintLocal(point);
        all_min = all_min.cwiseMin(local);
        all_max = all_max.cwiseMax(local);
        all_cells.emplace(
            static_cast<int>(std::floor(local.x() / xy_cell_size)),
            static_cast<int>(std::floor(local.y() / xy_cell_size)));
    }
    stats.z02 = percentile(z_values, 0.02F);
    stats.z05 = percentile(z_values, 0.05F);
    stats.z50 = percentile(z_values, 0.50F);
    stats.z95 = percentile(z_values, 0.95F);
    stats.visible_height = stats.z95 - stats.z05;
    stats.occupied_xy_cells = all_cells.size();

    // Build vertical continuity bins only from the robust body range
    // [z02, z95] to avoid tail outliers creating false gaps.
    const float continuity_min_z = stats.z02 - 1.0e-4F;
    const float continuity_max_z = stats.z95 + 1.0e-4F;
    std::set<int> vertical_bins;
    for (const auto& point : points) {
        if (point.z() < continuity_min_z ||
            point.z() > continuity_max_z) {
            continue;
        }
        vertical_bins.insert(
            static_cast<int>(std::floor(point.z() / vertical_bin_size)));
    }
    stats.occupied_vertical_bins = vertical_bins.size();
    if (vertical_bins.size() > 1U) {
        auto previous = vertical_bins.begin();
        for (auto current = std::next(previous); current != vertical_bins.end();
             ++current, ++previous) {
            const float gap = static_cast<float>(*current - *previous - 1) *
                              vertical_bin_size;
            stats.max_vertical_gap = std::max(stats.max_vertical_gap, gap);
        }
    }

    std::set<std::pair<int, int>> band_cells;
    Eigen::Vector2f band_min = Eigen::Vector2f::Constant(
        std::numeric_limits<float>::infinity());
    Eigen::Vector2f band_max = Eigen::Vector2f::Constant(
        -std::numeric_limits<float>::infinity());
    const float band_top = stats.z05 + bottom_band_height;
    for (const auto& point : points) {
        if (point.z() >= stats.z02 - 1.0e-4F && point.z() <= band_top) {
            ++stats.bottom_band_points;
            const Eigen::Vector2f local = footprintLocal(point);
            band_min = band_min.cwiseMin(local);
            band_max = band_max.cwiseMax(local);
            band_cells.emplace(
                static_cast<int>(std::floor(local.x() / xy_cell_size)),
                static_cast<int>(std::floor(local.y() / xy_cell_size)));
        }
    }
    stats.bottom_band_xy_cells = band_cells.size();
    stats.bottom_band_point_ratio = static_cast<float>(stats.bottom_band_points) /
                                    static_cast<float>(points.size());
    stats.bottom_band_xy_cell_ratio = all_cells.empty()
        ? 0.0F
        : static_cast<float>(band_cells.size()) /
          static_cast<float>(all_cells.size());
    if (stats.bottom_band_points > 0U) {
        stats.bottom_band_span_x = std::max(0.0F, band_max.x() - band_min.x());
        stats.bottom_band_span_y = std::max(0.0F, band_max.y() - band_min.y());
        const Eigen::Vector2f inferred_size = (all_max - all_min).cwiseMax(0.0F);
        const float denom_x = footprint_size_xy.x() > 0.0F
            ? footprint_size_xy.x() : inferred_size.x();
        const float denom_y = footprint_size_xy.y() > 0.0F
            ? footprint_size_xy.y() : inferred_size.y();
        stats.bottom_band_span_x_ratio = denom_x > 1.0e-4F
            ? stats.bottom_band_span_x / denom_x : 0.0F;
        stats.bottom_band_span_y_ratio = denom_y > 1.0e-4F
            ? stats.bottom_band_span_y / denom_y : 0.0F;
    }

    const bool prior_consistent = !prior_height_valid ||
        (std::isfinite(prior_height) &&
         std::abs(stats.visible_height - prior_height) <= prior_height_tolerance);

    stats.valid = std::isfinite(stats.z02) && std::isfinite(stats.z05) &&
                  std::isfinite(stats.z50) && std::isfinite(stats.z95) &&
                  stats.visible_height > 0.0F;
    stats.support_strong = stats.valid &&
        stats.visible_height >= min_visible_height &&
        stats.bottom_band_points >= min_band_points &&
        stats.bottom_band_xy_cells >= min_band_cells &&
        stats.bottom_band_point_ratio >= min_band_point_ratio &&
        stats.bottom_band_xy_cell_ratio >= min_band_cell_ratio &&
        stats.bottom_band_span_x_ratio >= min_bottom_span_ratio &&
        stats.bottom_band_span_y_ratio >= min_bottom_span_ratio &&
        stats.occupied_vertical_bins >= min_vertical_bins &&
        stats.max_vertical_gap <= max_vertical_gap &&
        prior_consistent;
    if (!stats.valid) {
        stats.reject_reason = "invalid_percentiles";
    } else if (stats.visible_height < min_visible_height) {
        stats.reject_reason = "visible_height_too_small";
    } else if (stats.bottom_band_points < min_band_points) {
        stats.reject_reason = "bottom_band_points_too_few";
    } else if (stats.bottom_band_xy_cells < min_band_cells) {
        stats.reject_reason = "bottom_band_cells_too_few";
    } else if (stats.bottom_band_point_ratio < min_band_point_ratio) {
        stats.reject_reason = "bottom_band_point_ratio_too_small";
    } else if (stats.bottom_band_xy_cell_ratio < min_band_cell_ratio) {
        stats.reject_reason = "bottom_band_cell_ratio_too_small";
    } else if (stats.bottom_band_span_x_ratio < min_bottom_span_ratio ||
               stats.bottom_band_span_y_ratio < min_bottom_span_ratio) {
        stats.reject_reason = "bottom_band_lateral_span_too_small";
    } else if (stats.occupied_vertical_bins < min_vertical_bins) {
        stats.reject_reason = "vertical_bins_too_few";
    } else if (stats.max_vertical_gap > max_vertical_gap) {
        stats.reject_reason = "vertical_continuity_gap";
    } else if (!prior_consistent) {
        stats.reject_reason = "prior_height_mismatch";
    } else {
        stats.reject_reason = "accepted";
    }
    return stats;
}

float candidateUncertainty(const CargoVerticalStats& stats,
                           float minimum,
                           const CargoBottomFusionConfig& config) {
    const float lower_tail = std::max(0.0F, stats.z05 - stats.z02);
    const float sparse = 1.0F - std::clamp(
        stats.bottom_band_xy_cell_ratio, 0.0F, 1.0F);
    return std::clamp(
        minimum + config.tail_uncertainty_gain * lower_tail +
            config.sparse_uncertainty_gain * sparse,
        minimum, config.invalid_uncertainty);
}

float candidateConfidence(const CargoVerticalStats& stats, float base) {
    const float point_support = std::clamp(
        stats.bottom_band_point_ratio / 0.20F, 0.0F, 1.0F);
    const float cell_support = std::clamp(
        stats.bottom_band_xy_cell_ratio / 0.40F, 0.0F, 1.0F);
    return std::clamp(base * (0.55F + 0.20F * point_support +
                              0.25F * cell_support),
                      0.0F, 1.0F);
}

bool inferFootprint(const std::vector<Eigen::Vector3f>& points,
                    Eigen::Vector2f* center,
                    Eigen::Vector2f* size) {
    std::vector<float> xs;
    std::vector<float> ys;
    xs.reserve(points.size());
    ys.reserve(points.size());
    for (const auto& point : points) {
        if (!finitePoint(point)) continue;
        xs.push_back(point.x());
        ys.push_back(point.y());
    }
    if (xs.size() < 4U) return false;
    const float x05 = percentile(xs, 0.05F);
    const float x95 = percentile(xs, 0.95F);
    const float y05 = percentile(ys, 0.05F);
    const float y95 = percentile(ys, 0.95F);
    *center = Eigen::Vector2f(0.5F * (x05 + x95), 0.5F * (y05 + y95));
    *size = Eigen::Vector2f(x95 - x05, y95 - y05);
    return center->allFinite() && size->allFinite();
}

CargoBoxGeometry makeGeometry(
    const CargoBottomObservation& observation,
    const std::vector<Eigen::Vector3f>& selected_points,
    float bottom_z,
    float top_z,
    const Eigen::Vector2f& memory_center,
    const Eigen::Vector2f& memory_size,
    const CargoBottomFusionConfig& config) {
    CargoBoxGeometry geometry;
    if (!validRigidTransform(observation.T_map_base) ||
        !std::isfinite(bottom_z) || !std::isfinite(top_z) || top_z <= bottom_z) {
        return geometry;
    }

    Eigen::Vector2f center = memory_center;
    Eigen::Vector2f size = memory_size;
    if (observation.footprint_valid &&
        observation.footprint_center_base.allFinite() &&
        observation.footprint_size_xy.allFinite() &&
        std::isfinite(observation.footprint_yaw_base_rad)) {
        center = observation.footprint_center_base;
        size = observation.footprint_size_xy.cwiseAbs();
    } else if (!inferFootprint(selected_points, &center, &size) &&
               (!center.allFinite() || !size.allFinite())) {
        return geometry;
    }
    size.array() += 2.0F * std::max(0.0F, config.footprint_margin);
    if (size.minCoeff() < config.min_footprint_size ||
        size.maxCoeff() > config.max_footprint_size) {
        return geometry;
    }

    geometry.bottom_z_base = bottom_z;
    geometry.top_z_base = top_z;
    geometry.center_base = Eigen::Vector3f(
        center.x(), center.y(), 0.5F * (bottom_z + top_z));
    geometry.size_base = Eigen::Vector3f(
        size.x(), size.y(), top_z - bottom_z);

    const float hx = 0.5F * size.x();
    const float hy = 0.5F * size.y();
    const float yaw = observation.footprint_valid
        ? observation.footprint_yaw_base_rad : 0.0F;
    const float cosine = std::cos(yaw);
    const float sine = std::sin(yaw);
    std::size_t index = 0;
    for (int z_side = 0; z_side < 2; ++z_side) {
        const float z = z_side == 0 ? bottom_z : top_z;
        for (int y_side = 0; y_side < 2; ++y_side) {
            for (int x_side = 0; x_side < 2; ++x_side) {
                const float local_x = x_side == 0 ? -hx : hx;
                const float local_y = y_side == 0 ? -hy : hy;
                const Eigen::Vector3f corner(
                    center.x() + cosine * local_x - sine * local_y,
                    center.y() + sine * local_x + cosine * local_y, z);
                geometry.corners_base[index] = corner;
                geometry.corners_map[index] = observation.T_map_base * corner;
                ++index;
            }
        }
    }

    Eigen::Vector3f minimum = geometry.corners_map[0];
    Eigen::Vector3f maximum = geometry.corners_map[0];
    geometry.center_map.setZero();
    for (const auto& corner : geometry.corners_map) {
        if (!corner.allFinite()) return CargoBoxGeometry{};
        minimum = minimum.cwiseMin(corner);
        maximum = maximum.cwiseMax(corner);
        geometry.center_map += corner;
    }
    geometry.center_map /= 8.0F;
    geometry.size_map = maximum - minimum;
    geometry.bottom_z_map = std::min(
        std::min(geometry.corners_map[0].z(), geometry.corners_map[1].z()),
        std::min(geometry.corners_map[2].z(), geometry.corners_map[3].z()));
    geometry.top_z_map = std::max(
        std::max(geometry.corners_map[4].z(), geometry.corners_map[5].z()),
        std::max(geometry.corners_map[6].z(), geometry.corners_map[7].z()));
    geometry.valid = geometry.center_base.allFinite() &&
                     geometry.center_map.allFinite() &&
                     geometry.size_base.allFinite() &&
                     geometry.size_map.allFinite() &&
                     geometry.size_base.minCoeff() > 0.0F &&
                     geometry.size_map.minCoeff() >= 0.0F &&
                     geometry.top_z_map >= geometry.bottom_z_map;
    return geometry;
}

}  // namespace

const char* cargoBottomSourceName(CargoBottomSource source) noexcept {
    switch (source) {
        case CargoBottomSource::POINTS: return "POINTS";
        case CargoBottomSource::MAP_DIFF: return "MAP_DIFF";
        case CargoBottomSource::MAP_STATIC: return "MAP_STATIC";
        case CargoBottomSource::RECENT_STABLE: return "RECENT_STABLE";
        case CargoBottomSource::ORIGIN_HEIGHT: return "ORIGIN_HEIGHT";
        case CargoBottomSource::INVALID: default: return "INVALID";
    }
}

int cargoBottomSourcePriority(CargoBottomSource source) noexcept {
    switch (source) {
        // Map difference and the pre-lift thickness both combine a stable
        // physical thickness with the current robust upper surface. For an
        // overhead LiDAR they are more trustworthy than an intermittently
        // visible lower edge in the current points.
        case CargoBottomSource::MAP_DIFF: return 6;
        case CargoBottomSource::ORIGIN_HEIGHT: return 5;
        case CargoBottomSource::POINTS: return 4;
        case CargoBottomSource::MAP_STATIC: return 3;
        case CargoBottomSource::RECENT_STABLE: return 1;
        default: return 0;
    }
}

CargoBottomFusion::CargoBottomFusion(const CargoBottomFusionConfig& config)
    : config_(config) {}

void CargoBottomFusion::reset() {
    has_track_ = false;
    track_id_ = 0;
    last_stamp_sec_ = 0.0;
    newest_points_stamp_sec_ = 0.0;
    accumulated_frames_.clear();
    accumulated_point_count_ = 0;
    resetTemporalState();
}

void CargoBottomFusion::setConfig(const CargoBottomFusionConfig& config) {
    config_ = config;
    reset();
}

void CargoBottomFusion::resetTemporalState() {
    newest_points_stamp_sec_ = 0.0;
    accumulated_frames_.clear();
    accumulated_point_count_ = 0U;
    ema_valid_ = false;
    ema_source_ = CargoBottomSource::INVALID;
    ema_uses_track_center_ = false;
    ema_bottom_z_base_ = 0.0F;
    ema_top_z_base_ = 0.0F;
    stable_ = StableEstimate{};
    final_valid_ = false;
    final_uses_track_center_ = false;
    final_bottom_value_ = 0.0F;
    final_top_value_ = 0.0F;
    final_source_ = CargoBottomSource::INVALID;
    pending_large_jump_ = false;
    pending_bottom_value_ = 0.0F;
    pending_top_value_ = 0.0F;
    pending_source_ = CargoBottomSource::INVALID;
    pending_large_jump_count_ = 0;
    last_result_available_ = false;
    last_result_ = CargoBottomResult{};
}

void CargoBottomFusion::purgeAccumulation(double stamp_sec) {
    const double cutoff = stamp_sec - config_.accumulation_window_sec;
    while (!accumulated_frames_.empty() &&
           accumulated_frames_.front().stamp_sec < cutoff) {
        accumulated_point_count_ -= accumulated_frames_.front().points_map.size();
        accumulated_frames_.pop_front();
    }
    enforceAccumulationLimit();
}

void CargoBottomFusion::enforceAccumulationLimit() {
    while (accumulated_frames_.size() > 1U &&
           accumulated_point_count_ > config_.max_accumulated_points) {
        accumulated_point_count_ -=
            accumulated_frames_.front().points_map.size();
        accumulated_frames_.pop_front();
    }
    if (!accumulated_frames_.empty() &&
        accumulated_frames_.front().points_map.size() >
            config_.max_accumulated_points) {
        limitPointsDeterministically(
            &accumulated_frames_.front().points_map,
            config_.max_accumulated_points);
    }
    accumulated_point_count_ = 0U;
    for (const auto& frame : accumulated_frames_) {
        accumulated_point_count_ += frame.points_map.size();
    }
}

void CargoBottomFusion::appendPoints(const CargoBottomObservation& observation) {
    if (observation.points_base.empty()) {
        return;
    }
    if (!accumulated_frames_.empty() &&
        observation.stamp_sec <=
            newest_points_stamp_sec_ + config_.backwards_tolerance_sec) {
        return;
    }
    AccumulatedFrame frame;
    frame.stamp_sec = observation.stamp_sec;
    frame.track_center_valid = observation.track_center_valid;
    frame.track_center_map = observation.track_center_valid
        ? (observation.T_map_base * observation.track_center_base)
        : Eigen::Vector3f::Zero();
    frame.points_map.reserve(observation.points_base.size());
    for (const auto& p : observation.points_base) {
        if (finitePoint(p)) {
            frame.points_map.push_back(observation.T_map_base * p);
        }
    }
    limitPointsDeterministically(&frame.points_map,
                                 config_.max_accumulated_points);
    if (!frame.points_map.empty()) {
        while (!accumulated_frames_.empty() &&
               accumulated_point_count_ + frame.points_map.size() >
                   config_.max_accumulated_points) {
            accumulated_point_count_ -=
                accumulated_frames_.front().points_map.size();
            accumulated_frames_.pop_front();
        }
        accumulated_frames_.push_back(std::move(frame));
        accumulated_point_count_ += accumulated_frames_.back().points_map.size();
        newest_points_stamp_sec_ = observation.stamp_sec;
        enforceAccumulationLimit();
    }
}

std::vector<Eigen::Vector3f> CargoBottomFusion::alignedAccumulatedPoints(
        const CargoBottomObservation& observation) const {
    std::vector<Eigen::Vector3f> result;
    if (accumulated_frames_.empty()) {
        return result;
    }
    const Eigen::Isometry3f T_base_map = observation.T_map_base.inverse();
    const Eigen::Vector3f current_center_map = observation.track_center_valid
        ? observation.T_map_base * observation.track_center_base
        : Eigen::Vector3f::Zero();
    result.reserve(accumulated_point_count_);
    for (const auto& frame : accumulated_frames_) {
        for (const auto& p : frame.points_map) {
            Eigen::Vector3f compensated_map = p;
            if (frame.track_center_valid && observation.track_center_valid) {
                compensated_map += current_center_map - frame.track_center_map;
            }
            result.push_back(T_base_map * compensated_map);
        }
    }
    return result;
}

CargoBottomResult CargoBottomFusion::update(const CargoBottomObservation& observation) {
    CargoBottomResult result;
    result.track_id = observation.track_id;
    result.stamp_sec = observation.stamp_sec;
    result.evidence_stamp_sec = observation.stamp_sec;
    std::string invalid_config_field;
    if (!validConfig(config_, &invalid_config_field)) {
        reset();
        result.reason = "invalid_config:" + invalid_config_field;
        return result;
    }
    result.uncertainty = config_.invalid_uncertainty;

    if (!observation.track_valid || !std::isfinite(observation.stamp_sec) ||
        observation.stamp_sec < 0.0 ||
        !validRigidTransform(observation.T_map_base)) {
        reset();
        result.reason = "invalid_track_time_or_transform";
        return result;
    }
    if (!std::isfinite(observation.transform_stamp_sec) ||
        observation.transform_stamp_sec < 0.0 ||
        std::abs(observation.transform_stamp_sec - observation.stamp_sec) >
            config_.max_transform_skew_sec) {
        result.reason = "transform_stamp_not_aligned";
        return result;
    }
    if (observation.footprint_valid &&
        (!observation.footprint_center_base.allFinite() ||
         !observation.footprint_size_xy.allFinite() ||
         observation.footprint_size_xy.minCoeff() <= 0.0F ||
         !std::isfinite(observation.footprint_yaw_base_rad))) {
        result.reason = "invalid_tracked_footprint";
        return result;
    }
    if (observation.track_center_valid &&
        !observation.track_center_base.allFinite()) {
        result.reason = "invalid_track_center";
        return result;
    }
    const auto validHeightPrior = [&](bool valid, float value) {
        return !valid || (std::isfinite(value) &&
            value >= config_.minimum_prior_height &&
            value <= config_.maximum_prior_height);
    };
    if (!validHeightPrior(observation.prior_height_valid,
                          observation.prior_height_m) ||
        !validHeightPrior(observation.map_diff_height_valid,
                          observation.map_diff_height_m) ||
        !validHeightPrior(observation.map_static_height_valid,
                          observation.map_static_height_m) ||
        !validHeightPrior(observation.origin_height_valid,
                          observation.origin_height_m) ||
        (observation.current_top_valid &&
         !std::isfinite(observation.current_top_z_base))) {
        result.reason = "invalid_height_prior";
        return result;
    }
    if (!has_track_ || track_id_ != observation.track_id) {
        resetTemporalState();
        has_track_ = true;
        track_id_ = observation.track_id;
    } else if (observation.stamp_sec + config_.backwards_tolerance_sec <
               last_stamp_sec_) {
        resetTemporalState();
        last_stamp_sec_ = observation.stamp_sec;
        result.reason = "time_rollback_reset";
        last_result_ = result;
        last_result_available_ = true;
        return result;
    } else if (observation.stamp_sec - last_stamp_sec_ > config_.stale_reset_sec) {
        resetTemporalState();
        last_stamp_sec_ = observation.stamp_sec;
        result.reason = "stale_gap_reset";
        last_result_ = result;
        last_result_available_ = true;
        return result;
    } else if (final_valid_ && final_uses_track_center_ &&
               !observation.track_center_valid) {
        resetTemporalState();
        last_stamp_sec_ = observation.stamp_sec;
        result.reason = "track_center_lost";
        last_result_ = result;
        last_result_available_ = true;
        return result;
    } else if (observation.stamp_sec <=
                   last_stamp_sec_ + config_.backwards_tolerance_sec &&
               last_result_available_) {
        return last_result_;
    }
    last_stamp_sec_ = observation.stamp_sec;

    appendPoints(observation);
    purgeAccumulation(observation.stamp_sec);
    std::vector<Eigen::Vector3f> points = alignedAccumulatedPoints(observation);
    filterToFootprint(&points, observation, config_.footprint_margin);
    result.accumulated_points = points.size();
    const Eigen::Vector2f footprint_size = observation.footprint_valid
        ? observation.footprint_size_xy.cwiseAbs().eval()
        : Eigen::Vector2f::Zero();
    const CargoVerticalStats accumulated_points_stats = analyzeVertical(
        points, config_.bottom_band_height, config_.xy_cell_size,
        config_.points_min_points, config_.points_min_visible_height,
        config_.points_min_bottom_band_points,
        config_.points_min_bottom_band_xy_cells,
        config_.points_min_bottom_band_point_ratio,
        config_.points_min_bottom_band_xy_cell_ratio,
        observation.footprint_center_base, footprint_size,
        observation.footprint_yaw_base_rad,
        config_.points_min_bottom_span_ratio,
        config_.points_min_vertical_bins, config_.points_vertical_bin_size,
        config_.points_max_vertical_gap, observation.prior_height_valid,
        observation.prior_height_m, config_.prior_height_tolerance);
    std::vector<Eigen::Vector3f> current_points =
        finitePoints(observation.points_base);
    filterToFootprint(&current_points, observation, config_.footprint_margin);
    const CargoVerticalStats current_points_stats = analyzeVertical(
        current_points, config_.bottom_band_height, config_.xy_cell_size,
        config_.points_min_points, config_.points_min_visible_height,
        config_.points_min_bottom_band_points,
        config_.points_min_bottom_band_xy_cells,
        config_.points_min_bottom_band_point_ratio,
        config_.points_min_bottom_band_xy_cell_ratio,
        observation.footprint_center_base, footprint_size,
        observation.footprint_yaw_base_rad,
        config_.points_min_bottom_span_ratio,
        config_.points_min_vertical_bins, config_.points_vertical_bin_size,
        config_.points_max_vertical_gap, observation.prior_height_valid,
        observation.prior_height_m, config_.prior_height_tolerance);
    const bool uses_current_frame_points = current_points_stats.support_strong;
    if (uses_current_frame_points) {
        // A complete current observation is safer than temporal accumulation:
        // history can trail a rapidly hoisted cargo even after base-motion
        // compensation.
        points = std::move(current_points);
        result.points_stats = current_points_stats;
    } else {
        result.points_stats = accumulated_points_stats;
    }

    auto heightPriorStats = [&](bool valid, float height) {
        CargoVerticalStats stats;
        if (!valid || !observation.current_top_valid) {
            stats.reject_reason = !valid
                ? "height_prior_unavailable"
                : "current_top_unavailable";
            return stats;
        }
        stats.valid = true;
        stats.support_strong = true;
        stats.z05 = observation.current_top_z_base - height;
        stats.z95 = observation.current_top_z_base;
        stats.z02 = stats.z05;
        stats.z50 = 0.5F * (stats.z05 + stats.z95);
        stats.visible_height = height;
        stats.reject_reason = "accepted_height_prior";
        return stats;
    };
    result.map_diff_stats = heightPriorStats(
        observation.map_diff_height_valid, observation.map_diff_height_m);
    result.map_static_stats = heightPriorStats(
        observation.map_static_height_valid, observation.map_static_height_m);
    result.origin_height_stats = heightPriorStats(
        observation.origin_height_valid, observation.origin_height_m);

    SelectedCandidate selected;
    auto selectFromPoints = [&](CargoBottomSource source,
                                const std::vector<Eigen::Vector3f>& candidate_points,
                                const CargoVerticalStats& stats,
                                float uncertainty_min,
                                float confidence_base,
                                const char* reason) {
        if (!stats.support_strong ||
            cargoBottomSourcePriority(source) <=
                cargoBottomSourcePriority(selected.source)) {
            return;
        }
        selected.valid = true;
        selected.source = source;
        selected.reason = reason;
        selected.points_base = candidate_points;
        selected.stats = stats;
        selected.bottom_z_base = stats.z05;
        selected.top_z_base = stats.z95;
        selected.uncertainty = candidateUncertainty(stats, uncertainty_min, config_);
        selected.confidence = candidateConfidence(stats, confidence_base);
    };
    selectFromPoints(CargoBottomSource::POINTS, points, result.points_stats,
                     config_.points_uncertainty_min,
                     config_.points_confidence_base,
                     uses_current_frame_points
                         ? "current_points_supported"
                         : "accumulated_points_supported");
    auto selectFromHeightPrior = [&](CargoBottomSource source,
                                     const CargoVerticalStats& stats,
                                     float uncertainty,
                                     float confidence,
                                     const char* reason) {
        if (!stats.support_strong ||
            cargoBottomSourcePriority(source) <=
                cargoBottomSourcePriority(selected.source)) {
            return;
        }
        selected.valid = true;
        selected.source = source;
        selected.reason = reason;
        selected.stats = stats;
        selected.bottom_z_base = stats.z05;
        selected.top_z_base = stats.z95;
        selected.uncertainty = uncertainty;
        selected.confidence = confidence;
    };
    selectFromHeightPrior(CargoBottomSource::MAP_DIFF, result.map_diff_stats,
                          config_.map_diff_uncertainty_min,
                          config_.map_diff_confidence_base,
                          "map_difference_height_prior");
    selectFromHeightPrior(CargoBottomSource::MAP_STATIC,
                          result.map_static_stats,
                          config_.map_static_uncertainty_min,
                          config_.map_static_confidence_base,
                          "static_map_height_prior");
    selectFromHeightPrior(CargoBottomSource::ORIGIN_HEIGHT,
                          result.origin_height_stats,
                          config_.origin_height_uncertainty_min,
                          config_.origin_height_confidence_base,
                          "pre_lift_frozen_height_prior");

    if (!selected.valid && stable_.valid &&
        stable_.track_id == observation.track_id) {
        const double age = observation.stamp_sec - stable_.stamp_sec;
        if (age >= -config_.backwards_tolerance_sec &&
            age <= config_.stable_hold_sec) {
            selected.valid = true;
            selected.source = CargoBottomSource::RECENT_STABLE;
            selected.reason = "recent_stable_hold";
            selected.stats = stable_.stats;
            selected.bottom_z_base = stable_.bottom_z_base;
            selected.top_z_base = stable_.top_z_base;
            if (stable_.track_center_valid && observation.track_center_valid) {
                selected.bottom_z_base = observation.track_center_base.z() +
                    stable_.bottom_offset_from_track_center;
                selected.top_z_base = observation.track_center_base.z() +
                    stable_.top_offset_from_track_center;
            }
            selected.uncertainty = std::clamp(
                std::max(stable_.uncertainty,
                         config_.recent_stable_uncertainty_min) +
                    static_cast<float>(age) *
                        config_.stable_age_uncertainty_per_sec,
                config_.recent_stable_uncertainty_min,
                config_.invalid_uncertainty);
            const float decay = config_.stable_hold_sec > 0.0
                ? std::clamp(1.0F - static_cast<float>(age / config_.stable_hold_sec),
                             0.0F, 1.0F)
                : 0.0F;
            selected.confidence = std::min(
                stable_.confidence, config_.recent_stable_confidence_base) * decay;
            selected.age_sec = age;
            result.evidence_stamp_sec = stable_.stamp_sec;
            selected.memory_center_base = observation.track_center_valid
                ? observation.track_center_base.head<2>()
                : stable_.center_base;
            selected.memory_size_xy = stable_.size_xy;
        }
    }

    if (!selected.valid) {
        result.reason = "no_supported_height_source:points=" +
            result.points_stats.reject_reason + ";map_diff=" +
            result.map_diff_stats.reject_reason + ";map_static=" +
            result.map_static_stats.reject_reason + ";origin_height=" +
            result.origin_height_stats.reject_reason;
        result.source_name = cargoBottomSourceName(result.source);
        last_result_ = result;
        last_result_available_ = true;
        return result;
    }

    if (selected.source != CargoBottomSource::RECENT_STABLE) {
        const bool use_track_center = observation.track_center_valid;
        float bottom_value = selected.bottom_z_base;
        float top_value = selected.top_z_base;
        if (use_track_center) {
            bottom_value -= observation.track_center_base.z();
            top_value -= observation.track_center_base.z();
        }
        if (ema_valid_ && ema_source_ == selected.source &&
            ema_uses_track_center_ == use_track_center) {
            const float alpha = std::clamp(config_.ema_alpha, 0.0F, 1.0F);
            bottom_value = alpha * bottom_value +
                (1.0F - alpha) * ema_bottom_z_base_;
            top_value = alpha * top_value +
                (1.0F - alpha) * ema_top_z_base_;
        }
        ema_valid_ = true;
        ema_source_ = selected.source;
        ema_uses_track_center_ = use_track_center;
        ema_bottom_z_base_ = bottom_value;
        ema_top_z_base_ = top_value;
        selected.bottom_z_base = use_track_center
            ? observation.track_center_base.z() + bottom_value
            : bottom_value;
        selected.top_z_base = use_track_center
            ? observation.track_center_base.z() + top_value
            : top_value;
    }

    // Apply one source-independent output gate after all source-specific
    // filtering.  A source change can therefore never bypass jump protection.
    bool final_transition_held = false;
    const auto finalAbsolute = [&](float value, bool uses_track_center) {
        return uses_track_center && observation.track_center_valid
            ? observation.track_center_base.z() + value
            : value;
    };
    const bool final_frame_compatible = final_valid_ &&
        (!final_uses_track_center_ || observation.track_center_valid);
    if (final_frame_compatible) {
        const float previous_bottom = finalAbsolute(
            final_bottom_value_, final_uses_track_center_);
        const float previous_top = finalAbsolute(
            final_top_value_, final_uses_track_center_);
        const float previous_height = previous_top - previous_bottom;
        const float candidate_height =
            selected.top_z_base - selected.bottom_z_base;
        const float bottom_jump =
            std::abs(selected.bottom_z_base - previous_bottom);
        const float top_jump = std::abs(selected.top_z_base - previous_top);
        const float height_jump =
            std::abs(candidate_height - previous_height);
        const float jump = std::max({bottom_jump, top_jump, height_jump});
        if (jump <= config_.direct_update_max_jump) {
            pending_large_jump_ = false;
            pending_large_jump_count_ = 0U;
        } else if (jump <= config_.soft_update_max_jump) {
            const float alpha = config_.soft_update_alpha;
            selected.bottom_z_base = alpha * selected.bottom_z_base +
                (1.0F - alpha) * previous_bottom;
            selected.top_z_base = alpha * selected.top_z_base +
                (1.0F - alpha) * previous_top;
            selected.uncertainty = std::clamp(
                selected.uncertainty + 0.5F * jump,
                selected.uncertainty, config_.invalid_uncertainty);
            selected.confidence *= 0.75F;
            selected.reason += ";source_transition_softened";
            pending_large_jump_ = false;
            pending_large_jump_count_ = 0U;
        } else {
            const bool same_pending = pending_large_jump_ &&
                pending_source_ == selected.source &&
                std::abs(pending_bottom_value_ - selected.bottom_z_base) <=
                    config_.large_jump_confirmation_tolerance &&
                std::abs(pending_top_value_ - selected.top_z_base) <=
                    config_.large_jump_confirmation_tolerance;
            if (same_pending) {
                ++pending_large_jump_count_;
            } else {
                pending_large_jump_ = true;
                pending_source_ = selected.source;
                pending_bottom_value_ = selected.bottom_z_base;
                pending_top_value_ = selected.top_z_base;
                pending_large_jump_count_ = 1U;
            }
            if (pending_large_jump_count_ <
                config_.large_jump_confirm_frames) {
                selected.bottom_z_base = previous_bottom;
                selected.top_z_base = previous_top;
                selected.source = final_source_;
                selected.reason = "large_jump_confirmation_pending";
                final_transition_held = true;
                if (stable_.valid) {
                    selected.stats = stable_.stats;
                    selected.memory_center_base = stable_.center_base;
                    selected.memory_size_xy = stable_.size_xy;
                    selected.confidence = stable_.confidence * 0.50F;
                    selected.age_sec =
                        observation.stamp_sec - stable_.stamp_sec;
                    result.evidence_stamp_sec = stable_.stamp_sec;
                } else {
                    selected.stats = CargoVerticalStats{};
                    selected.confidence = 0.0F;
                }
                selected.points_base.clear();
                selected.uncertainty = std::clamp(
                    std::max(stable_.uncertainty,
                             config_.recent_stable_uncertainty_min) +
                        std::min(jump, config_.invalid_uncertainty),
                    config_.recent_stable_uncertainty_min,
                    config_.invalid_uncertainty);
            } else {
                selected.bottom_z_base = pending_bottom_value_;
                selected.top_z_base = pending_top_value_;
                selected.reason += ";large_jump_confirmed";
                pending_large_jump_ = false;
                pending_large_jump_count_ = 0U;
            }
        }
    }

    final_valid_ = true;
    final_source_ = selected.source;
    final_uses_track_center_ = observation.track_center_valid;
    final_bottom_value_ = final_uses_track_center_
        ? selected.bottom_z_base - observation.track_center_base.z()
        : selected.bottom_z_base;
    final_top_value_ = final_uses_track_center_
        ? selected.top_z_base - observation.track_center_base.z()
        : selected.top_z_base;

    result.geometry = makeGeometry(
        observation, selected.points_base, selected.bottom_z_base,
        selected.top_z_base, selected.memory_center_base,
        selected.memory_size_xy, config_);
    result.source = selected.source;
    result.source_name = cargoBottomSourceName(selected.source);
    result.reason = selected.reason;
    result.selected_stats = selected.stats;
    result.uncertainty = selected.uncertainty;
    result.confidence = selected.confidence;
    result.source_age_sec = selected.age_sec;
    result.height = selected.top_z_base - selected.bottom_z_base;
    result.height_valid = std::isfinite(result.height) && result.height > 0.0F;
    result.geometry_valid = result.geometry.valid;
    result.valid = result.height_valid && result.geometry_valid;

    if (result.valid && selected.source != CargoBottomSource::RECENT_STABLE &&
        !final_transition_held) {
        stable_.valid = true;
        stable_.track_id = observation.track_id;
        stable_.stamp_sec = observation.stamp_sec;
        stable_.original_source = selected.source;
        stable_.bottom_z_base = selected.bottom_z_base;
        stable_.top_z_base = selected.top_z_base;
        stable_.uncertainty = selected.uncertainty;
        stable_.confidence = selected.confidence;
        stable_.center_base = result.geometry.center_base.head<2>();
        stable_.size_xy = (
            result.geometry.size_base.head<2>().array() -
            2.0F * std::max(0.0F, config_.footprint_margin)).max(
                config_.min_footprint_size).matrix();
        stable_.track_center_valid = observation.track_center_valid;
        if (stable_.track_center_valid) {
            stable_.bottom_offset_from_track_center =
                selected.bottom_z_base - observation.track_center_base.z();
            stable_.top_offset_from_track_center =
                selected.top_z_base - observation.track_center_base.z();
        }
        stable_.stats = selected.stats;
    }
    if (!result.valid) {
        result.reason += ";geometry_invalid";
    }
    last_result_ = result;
    last_result_available_ = true;
    return result;
}

}  // namespace ndt_slam
