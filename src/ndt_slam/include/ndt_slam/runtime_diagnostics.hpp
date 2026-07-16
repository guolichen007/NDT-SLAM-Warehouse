#pragma once
// Runtime diagnostics for 1.0x and 1.5x acceptance testing.
// Observes only — does not change any algorithm result.

#include <string>
#include <fstream>
#include <deque>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>
#include <cstdint>
#include <limits>

namespace ndt_slam {

// Fixed-format risk tags — grep-friendly, never change the string.
struct RuntimeDiagnosticsConfig {
  bool enabled = false;
  bool console_health_enabled = false;
  bool console_risk_enabled = false;
  bool cargo_console_enabled = true;
  double health_period_sec = 10.0;
  double risk_repeat_period_sec = 10.0;
  bool csv_enabled = true;
  double csv_flush_period_sec = 1.0;
  int warn_consecutive_overrun_frames = 3;
  int warn_prediction_only_frames = 3;
  int warn_target_fallback_frames = 3;
  double warn_cargo_bottom_jump_m = 0.20;
  double warn_cargo_height_jump_m = 0.20;
};

struct PipelineRiskRecord {
  std::string reason;
  int level = 0;
  int frame = 0;
  double stamp = 0.0;
  double frame_budget_ms = 0.0;
  double total_ms = 0.0;
  double preprocess_ms = 0.0;
  double target_prepare_ms = 0.0;
  double ndt_align_ms = 0.0;
  double ekf_ms = 0.0;
  double map_commit_ms = 0.0;
  int consecutive_overruns = 0;
  double estimated_backlog_frames = 0.0;
  double processed_hz = 0.0;
  double input_hz = 0.0;
  double drop_ratio = 0.0;
};

struct PipelineRateSnapshot {
  uint64_t callback_total = 0;
  uint64_t processed_total = 0;
  uint64_t queue_drop_total = 0;
  double callback_sensor_dt_p50_ms = 0.0;
  double callback_sensor_dt_p95_ms = 0.0;
  double callback_wall_dt_p50_ms = 0.0;
  double processed_sensor_dt_p50_ms = 0.0;
  double processed_sensor_dt_p95_ms = 0.0;
  double processed_wall_dt_p50_ms = 0.0;
  double callback_sensor_dt_last_ms = 0.0;
  double callback_wall_dt_last_ms = 0.0;
  double processed_sensor_dt_last_ms = 0.0;
  double processed_wall_dt_last_ms = 0.0;
  double callback_hz = 0.0;
  double processed_hz = 0.0;
  double processed_ratio = 0.0;
  double drop_ratio = 0.0;
  double frame_budget_ms = 100.0;
};

struct RuntimeStageTimes {
  double ros_to_pcl_ms = 0.0;
  double near_filter_ms = 0.0;
  double hook_prepare_ms = 0.0;
  double cargo_detect_ms = 0.0;
  double cargo_warning_ms = 0.0;
  double slam_voxel_ms = 0.0;
  double ground_split_ms = 0.0;
  double channel_filter_ms = 0.0;
  double human_filter_ms = 0.0;
  double registration_build_ms = 0.0;
  double target_bind_ms = 0.0;
  double ndt_ms = 0.0;
  double ekf_ms = 0.0;
  double publish_odom_ms = 0.0;
  double current_cloud_ms = 0.0;
  double icp_prepare_ms = 0.0;
  double map_commit_ms = 0.0;
  double clean_map_ms = 0.0;
  double display_map_ms = 0.0;
  double shadow_target_ms = 0.0;
  double csv_log_ms = 0.0;
};

// Rolling statistics with fixed window to avoid unbounded memory growth.
class RollingStats {
public:
  explicit RollingStats(size_t window = 200) : window_(window) {}

  void add(double v) {
    buf_.push_back(v);
    if (buf_.size() > window_) buf_.pop_front();
  }

  double median() const {
    if (buf_.empty()) return 0.0;
    std::vector<double> s(buf_.begin(), buf_.end());
    std::sort(s.begin(), s.end());
    return s[s.size() / 2];
  }

  double p95() const {
    if (buf_.empty()) return 0.0;
    std::vector<double> s(buf_.begin(), buf_.end());
    std::sort(s.begin(), s.end());
    size_t idx = static_cast<size_t>(s.size() * 0.95);
    if (idx >= s.size()) idx = s.size() - 1;
    return s[idx];
  }

  double p99() const {
    if (buf_.empty()) return 0.0;
    std::vector<double> s(buf_.begin(), buf_.end());
    std::sort(s.begin(), s.end());
    size_t idx = static_cast<size_t>(s.size() * 0.99);
    if (idx >= s.size()) idx = s.size() - 1;
    return s[idx];
  }

  double max() const {
    if (buf_.empty()) return 0.0;
    return *std::max_element(buf_.begin(), buf_.end());
  }

  double mad() const {
    if (buf_.empty()) return 0.0;
    double med = median();
    std::vector<double> abs_dev;
    abs_dev.reserve(buf_.size());
    for (double v : buf_) abs_dev.push_back(std::abs(v - med));
    std::sort(abs_dev.begin(), abs_dev.end());
    return abs_dev[abs_dev.size() / 2];
  }

  size_t count() const { return buf_.size(); }

private:
  size_t window_;
  std::deque<double> buf_;
};

// NDT per-frame record for CSV output.
struct NdtFrameRecord {
  int frame_index = 0;
  double cloud_stamp = 0.0;
  double sensor_dt_ms = 0.0;
  double wall_interarrival_ms = 0.0;
  double callback_sensor_dt_ms = 0.0;
  double callback_wall_dt_ms = 0.0;
  double processed_sensor_dt_ms = 0.0;
  double processed_wall_dt_ms = 0.0;
  double queue_age_ms = 0.0;
  bool queue_degraded = false;
  int raw_points = 0;
  int merged_points = 0;
  int filtered_points = 0;
  int registration_points = 0;
  std::string registration_mode;
  int static_object_points = 0;
  int uncertain_candidate_points = 0;
  int ground_points = 0;
  double ground_fraction = 0.0;
  bool structure_quality_valid = false;
  bool observability_valid = false;
  bool observability_degenerate = false;
  bool observability_severe = false;
  double observability_strong_eigenvalue = 0.0;
  double observability_weak_eigenvalue = 0.0;
  double observability_ratio = 0.0;
  double observability_weak_direction_x = 0.0;
  double observability_weak_direction_y = 0.0;
  int target_points = 0;
  std::string target_source;
  int target_version = 0;
  bool target_reused = false;
  bool target_fallback = false;
  double preprocess_ms = 0.0;
  double target_prepare_ms = 0.0;
  double set_input_target_ms = 0.0;
  double ndt_align_ms = 0.0;
  double ekf_ms = 0.0;
  double map_commit_ms = 0.0;
  double total_ms = 0.0;
  RuntimeStageTimes stage;
  bool ndt_converged = false;
  std::string ndt_execution_state = "NDT_NOT_ATTEMPTED";
  int ndt_iterations = 0;
  double fitness = 0.0;
  double transformation_probability = 0.0;
  double raw_x = 0.0, raw_y = 0.0, raw_z = 0.0;
  double initial_guess_x = 0.0, initial_guess_y = 0.0, initial_guess_yaw_deg = 0.0;
  double raw_yaw_deg = 0.0;
  double ekf_x = 0.0, ekf_y = 0.0;
  double output_x = 0.0, output_y = 0.0, output_z = 0.0;
  double output_yaw_deg = 0.0;
  double raw_ndt_step_from_previous_m = 0.0;
  double ndt_correction_from_initial_guess_m = 0.0;
  double output_dx = 0.0, output_dy = 0.0;
  double output_yaw_step_deg = 0.0;
  double output_speed_mps = 0.0;
  double raw_step_m = 0.0;
  double output_step_m = 0.0;
  double allowed_step_m = 0.0;
  double innovation_m = 0.0;
  bool prediction_only = false;
  std::string prediction_reason;
  bool map_commit_allowed = false;
  std::string map_commit_reason;
  bool motion_gate_stationary = false;
  bool motion_gate_velocity_modified = false;
  bool motion_gate_map_commit_blocked = false;
  uint64_t motion_gate_check_count = 0;
  uint64_t motion_gate_block_count = 0;
  uint64_t motion_gate_violation_count = 0;
  bool icp_config_enabled = false;
  uint64_t icp_job_count = 0;
  uint64_t icp_stale_drop_count = 0;
  uint64_t icp_map_use_count = 0;
};

// Cargo per-frame record for CSV output.
struct CargoFrameRecord {
  double stamp = 0.0;
  std::string track_state;
  int track_id = -1;
  std::string lock_state;
  bool observation_valid = false;
  int cluster_points = 0;
  int support_points = 0;
  int candidate_count = 0;
  int selected_candidate_id = -1;
  double identity_score = 0.0;
  double orientation_confidence = 0.0;
  double shape_confidence = 0.0;
  double motion_confidence = 0.0;
  double overall_lock_confidence = 0.0;
  int obstacle_roi_finite_points = 0;
  double obstacle_roi_coverage_ratio = 0.0;
  int self_removed_points = 0;
  int identity_self_removed_points = 0;
  int rigging_self_removed_points = 0;
  int external_obstacle_points = 0;
  double self_margin_xy_m = 0.0;
  double self_margin_z_m = 0.0;
  double horizontal_uncertainty_m = 0.0;
  double vertical_uncertainty_m = 0.0;
  bool ground_reference_valid = false;
  double ground_z = 0.0;
  int dangerous_cluster_points = 0;
  double nearest_obstacle_x = 0.0;
  double nearest_obstacle_y = 0.0;
  double nearest_obstacle_z = 0.0;
  double nearest_cluster_center_x = 0.0;
  double nearest_cluster_center_y = 0.0;
  double nearest_cluster_center_z = 0.0;
  double nearest_cluster_distance =
      std::numeric_limits<double>::infinity();
  double obstacle_top_z95_m =
      std::numeric_limits<double>::quiet_NaN();
  double obstacle_uncertainty_m =
      std::numeric_limits<double>::quiet_NaN();
  double conservative_clearance_m =
      std::numeric_limits<double>::quiet_NaN();
  int requested_alarm_code = 30;
  std::string safety_reason = "system_not_ready";
  double center_x = 0.0, center_y = 0.0, center_z = 0.0;
  double measured_center_x = 0.0, measured_center_y = 0.0,
         measured_center_z = 0.0;
  double predicted_center_x = 0.0, predicted_center_y = 0.0,
         predicted_center_z = 0.0;
  double center_residual_x = 0.0, center_residual_y = 0.0,
         center_residual_z = 0.0;
  double pose_sensor_dt_sec = 0.0;
  std::string position_source;
  double pose_evidence_age_sec = 0.0;
  double height_evidence_age_sec = 0.0;
  double size_x = 0.0, size_y = 0.0, size_z = 0.0;
  double footprint_yaw_deg = 0.0;
  double raw_bottom_z = 0.0;
  double filtered_bottom_z = 0.0;
  double stable_bottom_z = 0.0;
  double top_z = 0.0;
  double height_m = 0.0;
  bool bottom_valid = false;
  bool height_valid = false;
  bool filter_accepted = false;
  std::string filter_reason;
  int lost_frames = 0;
  double odom_x = 0.0, odom_y = 0.0, odom_z = 0.0;
};

class RuntimeDiagnostics {
public:
  RuntimeDiagnostics();
  ~RuntimeDiagnostics();

  void configure(const RuntimeDiagnosticsConfig& cfg, const std::string& output_dir);
  void resetTimeEpoch();

  // Callback and processing rates are deliberately tracked separately.
  // Acceptance frame budget comes from callback sensor time, never from the
  // already-dropped processed stream.
  void recordCallback(double sensor_stamp_sec);
  void recordProcessed(double sensor_stamp_sec);
  PipelineRateSnapshot pipelineRateSnapshot(uint64_t queue_drop_total) const;
  void logPipelineRate(const PipelineRateSnapshot& rate,
                       size_t queue_size,
                       double oldest_age_ms);

  // ---- startup parameter dump (once) ----
  void logRunConfig(const std::map<std::string, std::string>& params);
  void logMergerCfg(const std::map<std::string, std::string>& params);
  void logBuildId(const std::map<std::string, std::string>& params);

  // ---- per-frame NDT CSV ----
  void writeNdtFrame(const NdtFrameRecord& rec);

  // ---- per-frame Cargo CSV ----
  void writeCargoFrame(const CargoFrameRecord& rec);

  // ---- periodic health ----
  void logNdtHealth(int frame, double stamp, double input_hz, double processed_hz,
                    double sensor_dt_ms, double total_ms_last, double ndt_ms_last,
                    const std::string& target_source, int target_points,
                    double converged_ratio, double fitness_last,
                    int prediction_only_count, int consecutive_prediction_only,
                    double raw_step_m, double output_step_m, double allowed_step_m);

  void logMergerHealth(int64_t received_201, int64_t received_203, int64_t paired,
                       int64_t single_201, int64_t single_203,
                       double pair_ratio, double pair_dt_ms_p50,
                       double pair_dt_ms_p95, double pair_dt_ms_max,
                       double output_hz, int64_t dropped, int64_t reused,
                       const std::string& last_mode);

  void logCargoHealth(const CargoFrameRecord& rec);

  // ---- risk outputs (greppable fixed format) ----
  void logNdtRiskNotConverged(int frame, double stamp, double fitness, int iterations,
                               const std::string& target_source, int target_points,
                               int input_points, double ndt_ms, double total_ms);
  void logNdtRiskFitnessSpike(int frame, double stamp, double fitness,
                               double rolling_median, double rolling_mad,
                               double configured_threshold, bool converged,
                               double raw_step_m, double innovation_m);
  void logNdtRiskTargetTooSmall(int frame, double stamp, const std::string& candidate_source,
                                 int candidate_points, int required_points,
                                 const std::string& fallback_source, int fallback_points);
  void logNdtRiskTargetFallbackStreak(int count, const std::string& current_source,
                                       const std::string& fallback_source);

  // Pipeline overrun is a stateful event: enter/change/repeat/clear.  Calling
  // this once per frame prevents FRAME_OVERRUN and SUSTAINED_OVERRUN from
  // being printed as two independent warnings for the same observation.
  void updatePipelineRisk(const PipelineRiskRecord& rec);
  void clearPipelineRisk(const PipelineRiskRecord& rec);

  // Clears a throttled non-pipeline risk and reports recovery once.  The key
  // is stable per risk type; output_tag is the existing greppable log family.
  void clearConsoleRisk(const std::string& key,
                        const std::string& output_tag);

  void logOdomRiskRawStepExceeded(int frame, double stamp, double sensor_dt_ms,
                                   double raw_dx, double raw_dy, double raw_dz,
                                   double raw_step_m, double allowed_step_m,
                                   double fitness, bool converged);
  void logOdomRiskOutputStepViolation(int frame, double stamp,
                                       double output_dx, double output_dy, double output_dz,
                                       double output_step_m, double allowed_step_m);

  void logEkfRiskPredictionOnly(int frame, double stamp, const std::string& cause,
                                 double fitness, bool converged, double innovation_m,
                                 int consecutive_count);
  void logEkfRiskPredictionStreak(int count, double duration_sec, double last_valid_stamp);
  void logEkfRiskRecovery(const std::string& recovery_cause, int high_fitness_frames,
                           int prediction_only_frames, double innovation_m,
                           double fitness, double covariance_before, double covariance_after);

  void logMapCommitBlocked(const std::string& reason, double fitness, bool converged,
                            bool prediction_only, bool step_valid,
                            const std::string& target_source);

  // ---- cargo risk outputs ----
  void logCargoRiskDetectionLost(double stamp, int track_id, const std::string& previous_state,
                                  const std::string& current_state, int lost_frames,
                                  double last_valid_bottom_z);
  void logCargoRiskInsufficientPoints(double stamp, int track_id, int cluster_points,
                                       int required_points);
  void logCargoRiskBottomJump(double stamp, int track_id, double previous_bottom_z,
                               double raw_bottom_z, double filtered_bottom_z,
                               double delta_m, bool filter_accepted,
                               const std::string& filter_reason);
  void logCargoRiskHeightJump(double stamp, int track_id, double previous_height_m,
                               double height_m, double delta_m, double bottom_z, double top_z);
  void logCargoRiskHeightInvalid(double stamp, int track_id, bool bottom_valid,
                                  bool top_valid, int cluster_points,
                                  const std::string& state);
  void logCargoRiskBoxGeometryJump(double stamp, int track_id, double center_delta_m,
                                    double size_delta_x, double size_delta_y,
                                    double size_delta_z, double odom_step_m);

  // ---- flush CSV buffers ----
  void flushCsv();

  // ---- accessors for stats ----
  const RollingStats& totalMsStats() const { return total_ms_stats_; }
  const RollingStats& ndtMsStats() const { return ndt_ms_stats_; }
  const RollingStats& fitnessStats() const { return fitness_stats_; }

  int frameOverrunCount() const { return frame_overrun_count_; }
  int consecutiveOverruns() const { return consecutive_overruns_; }
  int predictionOnlyCount() const { return prediction_only_count_; }
  int maxPredictionStreak() const { return max_prediction_streak_; }
  int targetFallbackCount() const { return target_fallback_count_; }
  int maxTargetFallbackStreak() const { return max_target_fallback_streak_; }
  int rawStepExceededCount() const { return raw_step_exceeded_count_; }
  int outputStepViolationCount() const { return output_step_violation_count_; }

  void incrementFrameOverrun() { frame_overrun_count_++; consecutive_overruns_++; }
  void resetConsecutiveOverruns() { consecutive_overruns_ = 0; }
  void incrementPredictionOnly() { prediction_only_count_++; consecutive_prediction_only_++; }
  void resetConsecutivePredictionOnly() {
    max_prediction_streak_ = std::max(max_prediction_streak_, consecutive_prediction_only_);
    consecutive_prediction_only_ = 0;
  }
  void incrementTargetFallback() { target_fallback_count_++; consecutive_target_fallback_++; }
  void resetConsecutiveTargetFallback() {
    max_target_fallback_streak_ = std::max(max_target_fallback_streak_, consecutive_target_fallback_);
    consecutive_target_fallback_ = 0;
  }
  void incrementRawStepExceeded() { raw_step_exceeded_count_++; }
  void incrementOutputStepViolation() { output_step_violation_count_++; }

  bool isEnabled() const { return cfg_.enabled; }

private:
  std::string timestamp() const;
  void maybeFlushCsv();
  void writePipelineRisk(const char* event,
                         const PipelineRiskRecord& rec,
                         const std::string& previous_reason = "");
  bool shouldEmitConsoleRisk(const std::string& key,
                             const std::string& reason);

  struct ConsoleRiskState {
    std::string reason;
    std::chrono::steady_clock::time_point last_emit;
  };

  RuntimeDiagnosticsConfig cfg_;
  std::string output_dir_;

  std::ofstream ndt_csv_;
  std::ofstream cargo_csv_;
  std::mutex csv_mutex_;

  RollingStats total_ms_stats_;
  RollingStats ndt_ms_stats_;
  RollingStats fitness_stats_;

  int frame_overrun_count_ = 0;
  int consecutive_overruns_ = 0;
  int prediction_only_count_ = 0;
  int consecutive_prediction_only_ = 0;
  int max_prediction_streak_ = 0;
  int target_fallback_count_ = 0;
  int consecutive_target_fallback_ = 0;
  int max_target_fallback_streak_ = 0;
  int raw_step_exceeded_count_ = 0;
  int output_step_violation_count_ = 0;

  std::chrono::steady_clock::time_point last_csv_flush_;
  std::chrono::steady_clock::time_point last_health_console_;
  std::chrono::steady_clock::time_point last_cargo_console_;
  std::chrono::steady_clock::time_point last_pipeline_console_;
  std::chrono::steady_clock::time_point last_pipeline_risk_console_;
  uint64_t last_pipeline_queue_drop_total_ = 0;
  uint64_t last_pipeline_processed_total_ = 0;
  int last_pipeline_overrun_count_ = 0;
  bool pipeline_risk_active_ = false;
  int pipeline_risk_level_ = 0;
  std::string pipeline_risk_reason_;
  std::mutex console_risk_mutex_;
  std::map<std::string, ConsoleRiskState> console_risk_states_;

  mutable std::mutex rate_mutex_;
  uint64_t callback_total_ = 0;
  uint64_t processed_total_ = 0;
  double last_callback_sensor_stamp_ = 0.0;
  double last_processed_sensor_stamp_ = 0.0;
  double callback_sensor_dt_last_ms_ = 0.0;
  double callback_wall_dt_last_ms_ = 0.0;
  double processed_sensor_dt_last_ms_ = 0.0;
  double processed_wall_dt_last_ms_ = 0.0;
  std::chrono::steady_clock::time_point last_callback_wall_;
  std::chrono::steady_clock::time_point last_processed_wall_;
  RollingStats callback_sensor_dt_ms_{500};
  RollingStats callback_wall_dt_ms_{500};
  RollingStats processed_sensor_dt_ms_{500};
  RollingStats processed_wall_dt_ms_{500};
};

}  // namespace ndt_slam
