#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace ndt_slam {

// Ordered by neither confidence nor fallback priority. Use
// cargoBottomSourcePriority() when comparing candidates; numeric values are
// kept stable for ROS messages.
enum class CargoBottomSource : std::uint8_t {
    INVALID = 0,
    POINTS = 1,
    MAP_DIFF = 2,
    MAP_STATIC = 3,
    RECENT_STABLE = 4,
    // Height frozen from stable pre-lift observations on the confirmed
    // EMPTY->LOADED hook transition. It is never learned from a post-lift
    // fallback result of the current track.
    ORIGIN_HEIGHT = 5,
    // A fresh, associated upper-surface observation combined with the
    // physical thickness frozen for this cargo track. This is a current
    // absolute bottom measurement; it is not a historical hold and does not
    // require the lower edge to be visible from an overhead LiDAR.
    DIRECT_TOP_FROZEN_THICKNESS = 6,
};

const char* cargoBottomSourceName(CargoBottomSource source) noexcept;
int cargoBottomSourcePriority(CargoBottomSource source) noexcept;

struct CargoVerticalStats {
    bool valid = false;
    bool support_strong = false;
    std::size_t finite_points = 0;
    float z02 = 0.0F;
    float z05 = 0.0F;
    float z50 = 0.0F;
    float z95 = 0.0F;
    float visible_height = 0.0F;
    std::size_t bottom_band_points = 0;
    std::size_t occupied_xy_cells = 0;
    std::size_t bottom_band_xy_cells = 0;
    float bottom_band_point_ratio = 0.0F;
    float bottom_band_xy_cell_ratio = 0.0F;
    float bottom_band_span_x = 0.0F;
    float bottom_band_span_y = 0.0F;
    float bottom_band_span_x_ratio = 0.0F;
    float bottom_band_span_y_ratio = 0.0F;
    std::size_t occupied_vertical_bins = 0;
    float max_vertical_gap = 0.0F;
    std::string reject_reason;
};

struct CargoBoxGeometry {
    bool valid = false;
    Eigen::Vector3f center_base = Eigen::Vector3f::Zero();
    Eigen::Vector3f size_base = Eigen::Vector3f::Zero();
    Eigen::Vector3f center_map = Eigen::Vector3f::Zero();
    // Axis-aligned extent of corners_map. The actual map orientation is retained
    // by corners_map, so consumers should use the corners for an oriented marker.
    Eigen::Vector3f size_map = Eigen::Vector3f::Zero();
    float bottom_z_base = 0.0F;
    float top_z_base = 0.0F;
    float bottom_z_map = 0.0F;
    float top_z_map = 0.0F;
    std::array<Eigen::Vector3f, 8> corners_base{};
    std::array<Eigen::Vector3f, 8> corners_map{};
};

struct CargoBottomFusionConfig {
    // Points are transformed with the pose carrying the same timestamp, stored in
    // map, then transformed back with the current pose. This removes base motion
    // from the short accumulation window.
    double accumulation_window_sec = 0.50;
    double stale_reset_sec = 0.80;
    double stable_hold_sec = 0.50;
    double max_transform_skew_sec = 0.03;
    double backwards_tolerance_sec = 1.0e-4;
    std::size_t max_accumulated_points = 20000;

    std::size_t points_min_points = 40;
    float points_min_visible_height = 0.18F;
    std::size_t points_min_bottom_band_points = 12;
    std::size_t points_min_bottom_band_xy_cells = 3;
    float points_min_bottom_band_point_ratio = 0.06F;
    float points_min_bottom_band_xy_cell_ratio = 0.10F;
    float points_min_bottom_span_ratio = 0.30F;
    std::size_t points_min_vertical_bins = 4;
    float points_vertical_bin_size = 0.10F;
    float points_max_vertical_gap = 0.30F;
    float prior_height_tolerance = 0.30F;

    std::size_t map_diff_min_points = 20;
    std::size_t map_static_min_points = 24;
    float map_min_visible_height = 0.10F;
    std::size_t map_min_bottom_band_points = 5;
    std::size_t map_min_bottom_band_xy_cells = 2;
    float map_min_bottom_band_point_ratio = 0.03F;
    float map_min_bottom_band_xy_cell_ratio = 0.05F;

    float bottom_band_height = 0.12F;
    float xy_cell_size = 0.10F;
    float footprint_margin = 0.12F;
    float min_footprint_size = 0.05F;
    float max_footprint_size = 10.0F;

    float points_uncertainty_min = 0.035F;
    float map_diff_uncertainty_min = 0.070F;
    float map_static_uncertainty_min = 0.120F;
    float origin_height_uncertainty_min = 0.100F;
    float direct_top_frozen_uncertainty_min = 0.060F;
    float recent_stable_uncertainty_min = 0.100F;
    float invalid_uncertainty = 1.0F;
    float tail_uncertainty_gain = 1.5F;
    float sparse_uncertainty_gain = 0.10F;
    float stable_age_uncertainty_per_sec = 0.20F;

    float points_confidence_base = 0.82F;
    float map_diff_confidence_base = 0.67F;
    float map_static_confidence_base = 0.52F;
    float origin_height_confidence_base = 0.58F;
    float direct_top_frozen_confidence_base = 0.74F;
    float recent_stable_confidence_base = 0.45F;
    float ema_alpha = 0.35F;

    // The final output filter is source-independent.  It prevents a source
    // switch from bypassing the per-source EMA and creating an instantaneous
    // bottom jump.
    float direct_update_max_jump = 0.15F;
    float soft_update_max_jump = 0.40F;
    float soft_update_alpha = 0.15F;
    std::size_t large_jump_confirm_frames = 3;
    float large_jump_confirmation_tolerance = 0.10F;

    float minimum_prior_height = 0.10F;
    float maximum_prior_height = 5.00F;
};

struct CargoBottomObservation {
    bool track_valid = false;
    std::uint64_t track_id = 0;
    double stamp_sec = 0.0;

    // transform_stamp_sec is deliberately explicit: the fusion rejects a pose
    // that is not time-aligned with points instead of silently creating a motion
    // trail. Callers normally set it to the cloud header timestamp.
    double transform_stamp_sec = std::numeric_limits<double>::quiet_NaN();
    Eigen::Isometry3f T_map_base = Eigen::Isometry3f::Identity();

    // PCL conversion at the call site is intentionally lightweight:
    //   points.emplace_back(p.x, p.y, p.z);
    std::vector<Eigen::Vector3f> points_base;

    // Map-derived evidence is a height prior captured at the lift origin, not
    // an old point cloud cropped by the cargo's current footprint.  Both
    // priors require a current top observation from the same sensor stamp.
    bool current_top_valid = false;
    bool current_top_support_valid = false;
    float current_top_z_base = 0.0F;
    // Frozen thickness belongs to track_id and may persist for the locked
    // track. It never refreshes current_top evidence time by itself.
    bool frozen_thickness_valid = false;
    float frozen_thickness_m = 0.0F;
    double frozen_thickness_stamp_sec = 0.0;
    float frozen_thickness_confidence = 0.0F;
    bool map_diff_height_valid = false;
    float map_diff_height_m = 0.0F;
    bool map_static_height_valid = false;
    float map_static_height_m = 0.0F;
    bool origin_height_valid = false;
    float origin_height_m = 0.0F;

    bool prior_height_valid = false;
    float prior_height_m = 0.0F;

    // If supplied, this tracked footprint is used for XY geometry and to reject
    // old accumulated points outside the current cargo box. Otherwise robust XY
    // bounds are inferred from the selected candidate.
    bool footprint_valid = false;
    Eigen::Vector2f footprint_center_base = Eigen::Vector2f::Zero();
    Eigen::Vector2f footprint_size_xy = Eigen::Vector2f::Zero();
    float footprint_yaw_base_rad = 0.0F;

    // Optional center of the same tracked cargo at stamp_sec.  When present it
    // lets the accumulator compensate cargo translation (including hoisting in
    // Z), rather than compensating only crane/base motion.  It must come from
    // the same detection and same timestamp as points_base.
    bool track_center_valid = false;
    Eigen::Vector3f track_center_base = Eigen::Vector3f::Zero();
};

struct CargoBottomResult {
    bool valid = false;
    bool height_valid = false;
    bool geometry_valid = false;
    std::uint64_t track_id = 0;
    double stamp_sec = 0.0;
    // Timestamp of the physical height evidence. For RECENT_STABLE and a held
    // large-jump transition this remains the stable sample timestamp.
    double evidence_stamp_sec = 0.0;
    CargoBottomSource source = CargoBottomSource::INVALID;
    std::string source_name = "INVALID";
    std::string reason = "not_initialized";

    float height = 0.0F;
    float uncertainty = 1.0F;
    float confidence = 0.0F;
    double source_age_sec = 0.0;
    std::size_t accumulated_points = 0;

    CargoVerticalStats selected_stats;
    CargoVerticalStats points_stats;
    CargoVerticalStats map_diff_stats;
    CargoVerticalStats map_static_stats;
    CargoVerticalStats origin_height_stats;
    CargoVerticalStats direct_top_frozen_stats;
    CargoBoxGeometry geometry;
};

// Stateful, single-threaded cargo-bottom estimator. A caller may keep one object
// for all tracks; track changes, stale input and backward time automatically reset
// all temporal state so height/EMA never leaks between cargoes.
class CargoBottomFusion {
public:
    explicit CargoBottomFusion(const CargoBottomFusionConfig& config = {});

    CargoBottomResult update(const CargoBottomObservation& observation);
    void reset();
    void setConfig(const CargoBottomFusionConfig& config);

    const CargoBottomFusionConfig& config() const noexcept { return config_; }
    std::size_t accumulatedPointCount() const noexcept { return accumulated_point_count_; }
    bool hasTrack() const noexcept { return has_track_; }
    std::uint64_t trackId() const noexcept { return track_id_; }

private:
    struct AccumulatedFrame {
        double stamp_sec = 0.0;
        std::vector<Eigen::Vector3f> points_map;
        bool track_center_valid = false;
        Eigen::Vector3f track_center_map = Eigen::Vector3f::Zero();
    };

    struct StableEstimate {
        bool valid = false;
        std::uint64_t track_id = 0;
        double stamp_sec = 0.0;
        CargoBottomSource original_source = CargoBottomSource::INVALID;
        float bottom_z_base = 0.0F;
        float top_z_base = 0.0F;
        float uncertainty = 1.0F;
        float confidence = 0.0F;
        Eigen::Vector2f center_base = Eigen::Vector2f::Zero();
        // Stored before footprint_margin is applied, so RECENT_STABLE does not
        // inflate the box again on every fallback.
        Eigen::Vector2f size_xy = Eigen::Vector2f::Zero();
        bool track_center_valid = false;
        float bottom_offset_from_track_center = 0.0F;
        float top_offset_from_track_center = 0.0F;
        CargoVerticalStats stats;
    };

    struct SelectedCandidate {
        bool valid = false;
        CargoBottomSource source = CargoBottomSource::INVALID;
        std::string reason;
        std::vector<Eigen::Vector3f> points_base;
        CargoVerticalStats stats;
        float bottom_z_base = 0.0F;
        float top_z_base = 0.0F;
        float uncertainty = 1.0F;
        float confidence = 0.0F;
        double age_sec = 0.0;
        Eigen::Vector2f memory_center_base = Eigen::Vector2f::Zero();
        Eigen::Vector2f memory_size_xy = Eigen::Vector2f::Zero();
    };

    CargoBottomFusionConfig config_;
    bool has_track_ = false;
    std::uint64_t track_id_ = 0;
    double last_stamp_sec_ = 0.0;
    double newest_points_stamp_sec_ = 0.0;
    std::deque<AccumulatedFrame> accumulated_frames_;
    std::size_t accumulated_point_count_ = 0;

    bool ema_valid_ = false;
    CargoBottomSource ema_source_ = CargoBottomSource::INVALID;
    bool ema_uses_track_center_ = false;
    // Absolute base-frame Z when ema_uses_track_center_ is false; offsets from
    // the current tracked center when it is true.
    float ema_bottom_z_base_ = 0.0F;
    float ema_top_z_base_ = 0.0F;
    StableEstimate stable_;

    bool final_valid_ = false;
    bool final_uses_track_center_ = false;
    float final_bottom_value_ = 0.0F;
    float final_top_value_ = 0.0F;
    CargoBottomSource final_source_ = CargoBottomSource::INVALID;
    bool pending_large_jump_ = false;
    float pending_bottom_value_ = 0.0F;
    float pending_top_value_ = 0.0F;
    CargoBottomSource pending_source_ = CargoBottomSource::INVALID;
    std::size_t pending_large_jump_count_ = 0;
    bool last_result_available_ = false;
    CargoBottomResult last_result_;

    void resetTemporalState();
    void purgeAccumulation(double stamp_sec);
    void enforceAccumulationLimit();
    void appendPoints(const CargoBottomObservation& observation);
    std::vector<Eigen::Vector3f> alignedAccumulatedPoints(
        const CargoBottomObservation& observation) const;
};

}  // namespace ndt_slam
