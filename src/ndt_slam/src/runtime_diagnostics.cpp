#include "ndt_slam/runtime_diagnostics.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace ndt_slam {

RuntimeDiagnostics::RuntimeDiagnostics()
    : last_csv_flush_(std::chrono::steady_clock::now()),
      last_health_console_(std::chrono::steady_clock::now()),
      last_pipeline_console_(std::chrono::steady_clock::now()),
      last_callback_wall_(std::chrono::steady_clock::now()),
      last_processed_wall_(std::chrono::steady_clock::now()) {}

RuntimeDiagnostics::~RuntimeDiagnostics() {
  // 确保CSV文件被正确关闭
  std::lock_guard<std::mutex> lock(csv_mutex_);
  if (ndt_csv_.is_open()) {
    ndt_csv_.flush();
    ndt_csv_.close();
  }
  if (cargo_csv_.is_open()) {
    cargo_csv_.flush();
    cargo_csv_.close();
  }
}

void RuntimeDiagnostics::configure(const RuntimeDiagnosticsConfig& cfg,
                                    const std::string& output_dir) {
  cfg_ = cfg;
  output_dir_ = output_dir;

  if (cfg_.enabled && cfg_.csv_enabled) {
    std::lock_guard<std::mutex> lock(csv_mutex_);
    ndt_csv_.open(output_dir_ + "/runtime_frames.csv");
    ndt_csv_ << "frame_index,cloud_stamp,sensor_dt_ms,wall_interarrival_ms,"
             << "callback_sensor_dt_ms,callback_wall_dt_ms,processed_sensor_dt_ms,"
             << "processed_wall_dt_ms,queue_age_ms,queue_degraded,"
             << "raw_points,merged_points,filtered_points,registration_points,"
             << "registration_mode,static_object_points,uncertain_candidate_points,"
             << "ground_points,ground_fraction,structure_quality_valid,"
             << "observability_valid,observability_degenerate,observability_severe,"
             << "observability_strong_eigenvalue,observability_weak_eigenvalue,"
             << "observability_ratio,observability_weak_direction_x,"
             << "observability_weak_direction_y,"
             << "target_points,target_source,target_version,target_reused,target_fallback,"
             << "preprocess_ms,target_prepare_ms,set_input_target_ms,ndt_align_ms,"
             << "ekf_ms,map_commit_ms,total_ms,ros_to_pcl_ms,near_filter_ms,hook_prepare_ms,"
             << "cargo_detect_ms,cargo_warning_ms,slam_voxel_ms,ground_split_ms,"
             << "channel_filter_ms,human_filter_ms,registration_build_ms,target_bind_ms,"
             << "publish_odom_ms,current_cloud_ms,icp_prepare_ms,clean_map_ms,"
             << "display_map_ms,shadow_target_ms,csv_log_ms,"
             << "ndt_converged,ndt_iterations,fitness,transformation_probability,"
             << "initial_guess_x,initial_guess_y,initial_guess_yaw_deg,"
             << "raw_x,raw_y,raw_z,raw_yaw_deg,ekf_x,ekf_y,"
             << "output_x,output_y,output_z,output_yaw_deg,"
             << "ndt_correction_from_initial_guess_m,raw_ndt_step_from_previous_m,"
             << "raw_step_m,output_dx,output_dy,output_step_m,output_speed_mps,"
             << "output_yaw_step_deg,allowed_step_m,innovation_m,"
             << "prediction_only,prediction_reason,map_commit_allowed,map_commit_reason,"
             << "motion_gate_stationary,motion_gate_velocity_modified,"
             << "motion_gate_map_commit_blocked,motion_gate_check_count,"
             << "motion_gate_block_count,motion_gate_violation_count,"
             << "icp_config_enabled,icp_job_count,"
             << "icp_stale_drop_count,icp_map_use_count\n";

    cargo_csv_.open(output_dir_ + "/cargo_frames.csv");
    cargo_csv_ << "stamp,track_state,track_id,lock_state,observation_valid,"
               << "cluster_points,support_points,center_x,center_y,center_z,"
               << "size_x,size_y,size_z,raw_bottom_z,filtered_bottom_z,stable_bottom_z,"
               << "top_z,height_m,bottom_valid,height_valid,filter_accepted,filter_reason,"
               << "odom_x,odom_y,odom_z\n";
  }

  last_csv_flush_ = std::chrono::steady_clock::now();
  last_health_console_ = std::chrono::steady_clock::now();
  last_pipeline_console_ = std::chrono::steady_clock::now();
  last_pipeline_queue_drop_total_ = 0;
}

void RuntimeDiagnostics::recordCallback(double sensor_stamp_sec) {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(rate_mutex_);
  ++callback_total_;
  if (last_callback_sensor_stamp_ > 0.0) {
    const double dt_ms = (sensor_stamp_sec - last_callback_sensor_stamp_) * 1000.0;
    if (std::isfinite(dt_ms) && dt_ms > 0.01 && dt_ms < 2000.0) {
      callback_sensor_dt_ms_.add(dt_ms);
      callback_sensor_dt_last_ms_ = dt_ms;
    }
    const double wall_ms =
        std::chrono::duration<double, std::milli>(now - last_callback_wall_).count();
    if (std::isfinite(wall_ms) && wall_ms > 0.0 && wall_ms < 5000.0) {
      callback_wall_dt_ms_.add(wall_ms);
      callback_wall_dt_last_ms_ = wall_ms;
    }
  }
  last_callback_sensor_stamp_ = sensor_stamp_sec;
  last_callback_wall_ = now;
}

void RuntimeDiagnostics::recordProcessed(double sensor_stamp_sec) {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(rate_mutex_);
  ++processed_total_;
  if (last_processed_sensor_stamp_ > 0.0) {
    const double dt_ms = (sensor_stamp_sec - last_processed_sensor_stamp_) * 1000.0;
    if (std::isfinite(dt_ms) && dt_ms > 0.01 && dt_ms < 5000.0) {
      processed_sensor_dt_ms_.add(dt_ms);
      processed_sensor_dt_last_ms_ = dt_ms;
    }
    const double wall_ms =
        std::chrono::duration<double, std::milli>(now - last_processed_wall_).count();
    if (std::isfinite(wall_ms) && wall_ms > 0.0 && wall_ms < 10000.0) {
      processed_wall_dt_ms_.add(wall_ms);
      processed_wall_dt_last_ms_ = wall_ms;
    }
  }
  last_processed_sensor_stamp_ = sensor_stamp_sec;
  last_processed_wall_ = now;
}

PipelineRateSnapshot RuntimeDiagnostics::pipelineRateSnapshot(
    uint64_t queue_drop_total) const {
  std::lock_guard<std::mutex> lock(rate_mutex_);
  PipelineRateSnapshot rate;
  rate.callback_total = callback_total_;
  rate.processed_total = processed_total_;
  rate.queue_drop_total = queue_drop_total;
  rate.callback_sensor_dt_p50_ms = callback_sensor_dt_ms_.median();
  rate.callback_sensor_dt_p95_ms = callback_sensor_dt_ms_.p95();
  rate.callback_wall_dt_p50_ms = callback_wall_dt_ms_.median();
  rate.processed_sensor_dt_p50_ms = processed_sensor_dt_ms_.median();
  rate.processed_sensor_dt_p95_ms = processed_sensor_dt_ms_.p95();
  rate.processed_wall_dt_p50_ms = processed_wall_dt_ms_.median();
  rate.callback_sensor_dt_last_ms = callback_sensor_dt_last_ms_;
  rate.callback_wall_dt_last_ms = callback_wall_dt_last_ms_;
  rate.processed_sensor_dt_last_ms = processed_sensor_dt_last_ms_;
  rate.processed_wall_dt_last_ms = processed_wall_dt_last_ms_;
  if (rate.callback_sensor_dt_p50_ms > 1e-6) {
    rate.callback_hz = 1000.0 / rate.callback_sensor_dt_p50_ms;
    rate.frame_budget_ms = rate.callback_sensor_dt_p50_ms;
  }
  if (rate.processed_sensor_dt_p50_ms > 1e-6) {
    rate.processed_hz = 1000.0 / rate.processed_sensor_dt_p50_ms;
  }
  if (rate.callback_total > 0) {
    rate.processed_ratio = static_cast<double>(rate.processed_total) /
                           static_cast<double>(rate.callback_total);
    rate.drop_ratio = static_cast<double>(queue_drop_total) /
                      static_cast<double>(rate.callback_total);
  }
  return rate;
}

void RuntimeDiagnostics::logPipelineRate(
    const PipelineRateSnapshot& rate,
    size_t queue_size,
    double oldest_age_ms) {
  if (!cfg_.enabled) return;
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_pipeline_console_).count() <
      cfg_.console_period_sec) {
    return;
  }
  last_pipeline_console_ = now;
  const bool queue_dropped_since_last_report =
      rate.queue_drop_total > last_pipeline_queue_drop_total_;
  last_pipeline_queue_drop_total_ = rate.queue_drop_total;
  std::cout << "[PIPELINE_RATE] callback=" << rate.callback_total
            << " processed=" << rate.processed_total
            << " queue_drop=" << rate.queue_drop_total
            << " callback_hz=" << std::fixed << std::setprecision(2)
            << rate.callback_hz
            << " processed_hz=" << rate.processed_hz
            << " processed_ratio=" << rate.processed_ratio
            << " drop_ratio=" << rate.drop_ratio
            << " callback_sensor_dt_p50_ms=" << rate.callback_sensor_dt_p50_ms
            << " callback_sensor_dt_p95_ms=" << rate.callback_sensor_dt_p95_ms
            << " processed_sensor_dt_p50_ms=" << rate.processed_sensor_dt_p50_ms
            << " processed_sensor_dt_p95_ms=" << rate.processed_sensor_dt_p95_ms
            << " frame_budget_ms=" << rate.frame_budget_ms
            << " queue_size=" << queue_size
            << " oldest_age_ms=" << oldest_age_ms
            << " last_drop_reason="
            << (queue_dropped_since_last_report ? "overwrite_latest" : "none")
            << '\n';
}

std::string RuntimeDiagnostics::timestamp() const {
  auto now = std::chrono::system_clock::now();
  auto tt = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  std::ostringstream oss;
  oss << std::put_time(std::localtime(&tt), "%Y-%m-%d %H:%M:%S")
      << "." << std::setfill('0') << std::setw(3) << ms.count();
  return oss.str();
}

void RuntimeDiagnostics::logRunConfig(const std::map<std::string, std::string>& params) {
  if (!cfg_.enabled) return;
  std::cout << "[RUNCFG]" << std::endl;
  for (auto& kv : params) {
    std::cout << "  " << kv.first << "=" << kv.second << std::endl;
  }
}

void RuntimeDiagnostics::logMergerCfg(const std::map<std::string, std::string>& params) {
  if (!cfg_.enabled) return;
  std::cout << "[MERGER_CFG]" << std::endl;
  for (auto& kv : params) {
    std::cout << "  " << kv.first << "=" << kv.second << std::endl;
  }
}

void RuntimeDiagnostics::logBuildId(const std::map<std::string, std::string>& params) {
  if (!cfg_.enabled) return;
  std::cout << "[BUILD_ID]" << std::endl;
  for (auto& kv : params) {
    std::cout << "  " << kv.first << "=" << kv.second << std::endl;
  }
}

void RuntimeDiagnostics::writeNdtFrame(const NdtFrameRecord& rec) {
  if (!cfg_.enabled) return;

  std::lock_guard<std::mutex> lock(csv_mutex_);
  if (cfg_.csv_enabled && ndt_csv_.is_open()) {
    ndt_csv_ << rec.frame_index << ","
             << std::fixed << std::setprecision(6) << rec.cloud_stamp << ","
             << std::setprecision(1) << rec.sensor_dt_ms << ","
             << rec.wall_interarrival_ms << ","
             << rec.callback_sensor_dt_ms << ","
             << rec.callback_wall_dt_ms << ","
             << rec.processed_sensor_dt_ms << ","
             << rec.processed_wall_dt_ms << ","
             << rec.queue_age_ms << ","
             << (rec.queue_degraded ? 1 : 0) << ","
             << rec.raw_points << "," << rec.merged_points << ","
             << rec.filtered_points << "," << rec.registration_points << ","
             << rec.registration_mode << ","
             << rec.static_object_points << ","
             << rec.uncertain_candidate_points << ","
             << rec.ground_points << ","
             << rec.ground_fraction << ","
             << (rec.structure_quality_valid ? 1 : 0) << ","
             << (rec.observability_valid ? 1 : 0) << ","
             << (rec.observability_degenerate ? 1 : 0) << ","
             << (rec.observability_severe ? 1 : 0) << ","
             << rec.observability_strong_eigenvalue << ","
             << rec.observability_weak_eigenvalue << ","
             << rec.observability_ratio << ","
             << rec.observability_weak_direction_x << ","
             << rec.observability_weak_direction_y << ","
             << rec.target_points << "," << rec.target_source << ","
             << rec.target_version << ","
             << (rec.target_reused ? 1 : 0) << ","
             << (rec.target_fallback ? 1 : 0) << ","
             << std::setprecision(2)
             << rec.preprocess_ms << "," << rec.target_prepare_ms << ","
             << rec.set_input_target_ms << "," << rec.ndt_align_ms << ","
             << rec.ekf_ms << "," << rec.map_commit_ms << ","
             << rec.total_ms << ","
             << rec.stage.ros_to_pcl_ms << ","
             << rec.stage.near_filter_ms << ","
             << rec.stage.hook_prepare_ms << ","
             << rec.stage.cargo_detect_ms << ","
             << rec.stage.cargo_warning_ms << ","
             << rec.stage.slam_voxel_ms << ","
             << rec.stage.ground_split_ms << ","
             << rec.stage.channel_filter_ms << ","
             << rec.stage.human_filter_ms << ","
             << rec.stage.registration_build_ms << ","
             << rec.stage.target_bind_ms << ","
             << rec.stage.publish_odom_ms << ","
             << rec.stage.current_cloud_ms << ","
             << rec.stage.icp_prepare_ms << ","
             << rec.stage.clean_map_ms << ","
             << rec.stage.display_map_ms << ","
             << rec.stage.shadow_target_ms << ","
             << rec.stage.csv_log_ms << ","
             << (rec.ndt_converged ? 1 : 0) << "," << rec.ndt_iterations << ","
             << std::setprecision(6) << rec.fitness << ","
             << rec.transformation_probability << ","
             << rec.initial_guess_x << "," << rec.initial_guess_y << ","
             << rec.initial_guess_yaw_deg << ","
             << rec.raw_x << "," << rec.raw_y << "," << rec.raw_z << ","
             << rec.raw_yaw_deg << ","
             << rec.ekf_x << "," << rec.ekf_y << ","
             << rec.output_x << "," << rec.output_y << "," << rec.output_z << ","
             << rec.output_yaw_deg << ","
             << std::setprecision(4)
             << rec.ndt_correction_from_initial_guess_m << ","
             << rec.raw_ndt_step_from_previous_m << ","
             << rec.raw_step_m << ","
             << rec.output_dx << "," << rec.output_dy << ","
             << rec.output_step_m << "," << rec.output_speed_mps << ","
             << rec.output_yaw_step_deg << ","
             << rec.allowed_step_m << "," << rec.innovation_m << ","
             << (rec.prediction_only ? 1 : 0) << ","
             << rec.prediction_reason << ","
             << (rec.map_commit_allowed ? 1 : 0) << ","
             << rec.map_commit_reason << ","
             << (rec.motion_gate_stationary ? 1 : 0) << ","
             << (rec.motion_gate_velocity_modified ? 1 : 0) << ","
             << (rec.motion_gate_map_commit_blocked ? 1 : 0) << ","
             << rec.motion_gate_check_count << ","
             << rec.motion_gate_block_count << ","
             << rec.motion_gate_violation_count << ","
             << (rec.icp_config_enabled ? 1 : 0) << ","
             << rec.icp_job_count << ","
             << rec.icp_stale_drop_count << ","
             << rec.icp_map_use_count << "\n";
  }

  // Update rolling stats
  total_ms_stats_.add(rec.total_ms);
  ndt_ms_stats_.add(rec.ndt_align_ms);
  fitness_stats_.add(rec.fitness);

  if (cfg_.csv_enabled) maybeFlushCsv();
}

void RuntimeDiagnostics::writeCargoFrame(const CargoFrameRecord& rec) {
  if (!cfg_.enabled || !cfg_.csv_enabled) return;

  std::lock_guard<std::mutex> lock(csv_mutex_);
  if (cargo_csv_.is_open()) {
    cargo_csv_ << std::fixed << std::setprecision(6) << rec.stamp << ","
               << rec.track_state << "," << rec.track_id << ","
               << rec.lock_state << "," << (rec.observation_valid ? 1 : 0) << ","
               << rec.cluster_points << "," << rec.support_points << ","
               << std::setprecision(4)
               << rec.center_x << "," << rec.center_y << "," << rec.center_z << ","
               << rec.size_x << "," << rec.size_y << "," << rec.size_z << ","
               << std::setprecision(4)
               << rec.raw_bottom_z << "," << rec.filtered_bottom_z << ","
               << rec.stable_bottom_z << "," << rec.top_z << ","
               << rec.height_m << ","
               << (rec.bottom_valid ? 1 : 0) << "," << (rec.height_valid ? 1 : 0) << ","
               << (rec.filter_accepted ? 1 : 0) << "," << rec.filter_reason << ","
               << rec.odom_x << "," << rec.odom_y << "," << rec.odom_z << "\n";
  }
  maybeFlushCsv();
}

void RuntimeDiagnostics::maybeFlushCsv() {
  auto now = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(now - last_csv_flush_).count();
  if (elapsed >= cfg_.csv_flush_period_sec) {
    if (ndt_csv_.is_open()) ndt_csv_.flush();
    if (cargo_csv_.is_open()) cargo_csv_.flush();
    last_csv_flush_ = now;
  }
}

void RuntimeDiagnostics::flushCsv() {
  std::lock_guard<std::mutex> lock(csv_mutex_);
  if (ndt_csv_.is_open()) ndt_csv_.flush();
  if (cargo_csv_.is_open()) cargo_csv_.flush();
}

void RuntimeDiagnostics::logNdtHealth(int frame, double stamp, double input_hz,
                                       double processed_hz, double sensor_dt_ms,
                                       double total_ms_last, double ndt_ms_last,
                                       const std::string& target_source, int target_points,
                                       double converged_ratio, double fitness_last,
                                       int prediction_only_count, int consecutive_prediction_only,
                                       double raw_step_m, double output_step_m,
                                       double allowed_step_m) {
  if (!cfg_.enabled) return;
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_health_console_).count() < cfg_.console_period_sec)
    return;
  last_health_console_ = now;

  std::cout << "[NDT_HEALTH] frame=" << frame
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " input_hz=" << std::setprecision(1) << input_hz
            << " processed_hz=" << processed_hz
            << " sensor_dt_ms=" << std::setprecision(1) << sensor_dt_ms
            << " total_ms_last=" << std::setprecision(1) << total_ms_last
            << " total_ms_p50=" << total_ms_stats_.median()
            << " total_ms_p95=" << total_ms_stats_.p95()
            << " total_ms_p99=" << total_ms_stats_.p99()
            << " ndt_ms_p50=" << ndt_ms_stats_.median()
            << " ndt_ms_p95=" << ndt_ms_stats_.p95()
            << " ndt_ms_p99=" << ndt_ms_stats_.p99()
            << " target_source=" << target_source
            << " target_points=" << target_points
            << " converged_ratio=" << std::setprecision(2) << converged_ratio
            << " fitness_last=" << std::setprecision(4) << fitness_last
            << " fitness_median=" << fitness_stats_.median()
            << " fitness_p95=" << fitness_stats_.p95()
            << " prediction_only_count=" << prediction_only_count
            << " consecutive_prediction_only=" << consecutive_prediction_only
            << " raw_step_m=" << std::setprecision(4) << raw_step_m
            << " output_step_m=" << output_step_m
            << " allowed_step_m=" << allowed_step_m
            << std::endl;
}

void RuntimeDiagnostics::logMergerHealth(int64_t received_201, int64_t received_203,
                                          int64_t paired, int64_t single_201, int64_t single_203,
                                          double pair_ratio, double pair_dt_ms_p50,
                                          double pair_dt_ms_p95, double pair_dt_ms_max,
                                          double output_hz, int64_t dropped, int64_t reused,
                                          const std::string& last_mode) {
  if (!cfg_.enabled) return;
  std::cout << "[MERGER_HEALTH]"
            << " received_201=" << received_201
            << " received_203=" << received_203
            << " paired=" << paired
            << " single_201=" << single_201
            << " single_203=" << single_203
            << " pair_ratio=" << std::fixed << std::setprecision(2) << pair_ratio
            << " pair_dt_ms_p50=" << std::setprecision(1) << pair_dt_ms_p50
            << " pair_dt_ms_p95=" << pair_dt_ms_p95
            << " pair_dt_ms_max=" << pair_dt_ms_max
            << " output_hz=" << std::setprecision(1) << output_hz
            << " dropped=" << dropped
            << " reused=" << reused
            << " last_mode=" << last_mode
            << std::endl;
}

void RuntimeDiagnostics::logCargoHealth(const CargoFrameRecord& rec) {
  if (!cfg_.enabled) return;
  std::cout << "[CARGO_HEALTH]"
            << " stamp=" << std::fixed << std::setprecision(3) << rec.stamp
            << " track_state=" << rec.track_state
            << " track_id=" << rec.track_id
            << " lock_state=" << rec.lock_state
            << " observation_valid=" << (rec.observation_valid ? 1 : 0)
            << " cluster_points=" << rec.cluster_points
            << " support_points=" << rec.support_points
            << " center_x=" << std::setprecision(3) << rec.center_x
            << " center_y=" << rec.center_y
            << " center_z=" << rec.center_z
            << " size_x=" << rec.size_x
            << " size_y=" << rec.size_y
            << " size_z=" << rec.size_z
            << " raw_bottom_z=" << rec.raw_bottom_z
            << " filtered_bottom_z=" << rec.filtered_bottom_z
            << " stable_bottom_z=" << rec.stable_bottom_z
            << " top_z=" << rec.top_z
            << " height_m=" << rec.height_m
            << " bottom_valid=" << (rec.bottom_valid ? 1 : 0)
            << " height_valid=" << (rec.height_valid ? 1 : 0)
            << " filter_accepted=" << (rec.filter_accepted ? 1 : 0)
            << " filter_reason=" << rec.filter_reason
            << " lost_frames=" << rec.lost_frames
            << std::endl;
}

// ---- Risk outputs ----

void RuntimeDiagnostics::logNdtRiskNotConverged(int frame, double stamp, double fitness,
                                                  int iterations, const std::string& target_source,
                                                  int target_points, int input_points,
                                                  double ndt_ms, double total_ms) {
  if (!cfg_.enabled) return;
  std::cout << "[NDT_RISK] reason=NOT_CONVERGED"
            << " frame=" << frame
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " fitness=" << std::setprecision(4) << fitness
            << " iterations=" << iterations
            << " target_source=" << target_source
            << " target_points=" << target_points
            << " input_points=" << input_points
            << " ndt_ms=" << std::setprecision(1) << ndt_ms
            << " total_ms=" << total_ms
            << std::endl;
}

void RuntimeDiagnostics::logNdtRiskFitnessSpike(int frame, double stamp, double fitness,
                                                  double rolling_median, double rolling_mad,
                                                  double configured_threshold, bool converged,
                                                  double raw_step_m, double innovation_m) {
  if (!cfg_.enabled) return;
  std::cout << "[NDT_RISK] reason=FITNESS_SPIKE"
            << " frame=" << frame
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " fitness=" << std::setprecision(4) << fitness
            << " rolling_median=" << rolling_median
            << " rolling_mad=" << rolling_mad
            << " configured_threshold=" << configured_threshold
            << " converged=" << (converged ? 1 : 0)
            << " raw_step_m=" << std::setprecision(4) << raw_step_m
            << " innovation_m=" << innovation_m
            << std::endl;
}

void RuntimeDiagnostics::logNdtRiskTargetTooSmall(int frame, double stamp,
                                                    const std::string& candidate_source,
                                                    int candidate_points, int required_points,
                                                    const std::string& fallback_source,
                                                    int fallback_points) {
  if (!cfg_.enabled) return;
  std::cout << "[NDT_RISK] reason=TARGET_TOO_SMALL"
            << " frame=" << frame
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " candidate_source=" << candidate_source
            << " candidate_points=" << candidate_points
            << " required_points=" << required_points
            << " fallback_source=" << fallback_source
            << " fallback_points=" << fallback_points
            << std::endl;
}

void RuntimeDiagnostics::logNdtRiskTargetFallbackStreak(int count,
                                                          const std::string& current_source,
                                                          const std::string& fallback_source) {
  if (!cfg_.enabled) return;
  std::cout << "[NDT_RISK] reason=TARGET_FALLBACK_STREAK"
            << " count=" << count
            << " current_source=" << current_source
            << " fallback_source=" << fallback_source
            << std::endl;
}

void RuntimeDiagnostics::logMergerRiskSingleTimeout(const std::string& sensor, double stamp,
                                                      double pair_wait_wall_ms,
                                                      double nearest_sensor_dt_ms,
                                                      int64_t received_201, int64_t received_203,
                                                      int64_t paired, int64_t single_count) {
  if (!cfg_.enabled) return;
  std::cout << "[MERGER_RISK] reason=SINGLE_TIMEOUT"
            << " sensor=" << sensor
            << " stamp=" << std::fixed << std::setprecision(6) << stamp
            << " pair_wait_wall_ms=" << std::setprecision(1) << pair_wait_wall_ms
            << " nearest_sensor_dt_ms=" << nearest_sensor_dt_ms
            << " received_201=" << received_201
            << " received_203=" << received_203
            << " paired=" << paired
            << " single_count=" << single_count
            << std::endl;
}

void RuntimeDiagnostics::logMergerRiskPairDtExceeded(double stamp_201, double stamp_203,
                                                       double pair_dt_ms, double limit_ms) {
  if (!cfg_.enabled) return;
  std::cout << "[MERGER_RISK] reason=PAIR_DT_EXCEEDED"
            << " stamp_201=" << std::fixed << std::setprecision(6) << stamp_201
            << " stamp_203=" << stamp_203
            << " pair_dt_ms=" << std::setprecision(1) << pair_dt_ms
            << " limit_ms=" << limit_ms
            << std::endl;
}

void RuntimeDiagnostics::logMergerRiskCloudReuse(const std::string& sensor, double stamp,
                                                    double previous_stamp) {
  if (!cfg_.enabled) return;
  std::cout << "[MERGER_RISK] reason=CLOUD_REUSE"
            << " sensor=" << sensor
            << " stamp=" << std::fixed << std::setprecision(6) << stamp
            << " previous_stamp=" << previous_stamp
            << std::endl;
}

void RuntimeDiagnostics::logPipelineRiskFrameOverrun(int frame, double stamp,
                                                       double playback_rate,
                                                       double frame_budget_ms,
                                                       double total_ms,
                                                       double preprocess_ms,
                                                       double target_prepare_ms,
                                                       double ndt_align_ms,
                                                       double ekf_ms,
                                                       double map_commit_ms,
                                                       int consecutive_overruns) {
  if (!cfg_.enabled) return;
  std::cout << "[PIPELINE_RISK] reason=FRAME_OVERRUN"
            << " frame=" << frame
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " playback_rate=" << std::setprecision(1) << playback_rate
            << " frame_budget_ms=" << std::setprecision(1) << frame_budget_ms
            << " total_ms=" << total_ms
            << " preprocess_ms=" << preprocess_ms
            << " target_prepare_ms=" << target_prepare_ms
            << " ndt_align_ms=" << ndt_align_ms
            << " ekf_ms=" << ekf_ms
            << " map_commit_ms=" << map_commit_ms
            << " consecutive_overruns=" << consecutive_overruns
            << std::endl;
}

void RuntimeDiagnostics::logPipelineRiskSustainedOverrun(int count,
                                                           double estimated_backlog_frames,
                                                           double processed_hz,
                                                           double input_hz) {
  if (!cfg_.enabled) return;
  std::cout << "[PIPELINE_RISK] reason=SUSTAINED_OVERRUN"
            << " count=" << count
            << " estimated_backlog_frames=" << std::fixed << std::setprecision(1)
            << estimated_backlog_frames
            << " processed_hz=" << processed_hz
            << " input_hz=" << input_hz
            << std::endl;
}

void RuntimeDiagnostics::logOdomRiskRawStepExceeded(int frame, double stamp,
                                                       double sensor_dt_ms,
                                                       double raw_dx, double raw_dy, double raw_dz,
                                                       double raw_step_m, double allowed_step_m,
                                                       double fitness, bool converged) {
  if (!cfg_.enabled) return;
  std::cout << "[ODOM_RISK] reason=RAW_STEP_EXCEEDED"
            << " frame=" << frame
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " sensor_dt_ms=" << std::setprecision(1) << sensor_dt_ms
            << " raw_dx=" << std::setprecision(4) << raw_dx
            << " raw_dy=" << raw_dy
            << " raw_dz=" << raw_dz
            << " raw_step_m=" << raw_step_m
            << " allowed_step_m=" << allowed_step_m
            << " fitness=" << fitness
            << " converged=" << (converged ? 1 : 0)
            << std::endl;
}

void RuntimeDiagnostics::logOdomRiskOutputStepViolation(int frame, double stamp,
                                                           double output_dx, double output_dy,
                                                           double output_dz,
                                                           double output_step_m,
                                                           double allowed_step_m) {
  if (!cfg_.enabled) return;
  std::cout << "[ODOM_RISK] reason=OUTPUT_STEP_VIOLATION"
            << " frame=" << frame
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " output_dx=" << std::setprecision(4) << output_dx
            << " output_dy=" << output_dy
            << " output_dz=" << output_dz
            << " output_step_m=" << output_step_m
            << " allowed_step_m=" << allowed_step_m
            << std::endl;
}

void RuntimeDiagnostics::logEkfRiskPredictionOnly(int frame, double stamp,
                                                    const std::string& cause,
                                                    double fitness, bool converged,
                                                    double innovation_m,
                                                    int consecutive_count) {
  if (!cfg_.enabled) return;
  std::cout << "[EKF_RISK] reason=PREDICTION_ONLY"
            << " frame=" << frame
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " cause=" << cause
            << " fitness=" << std::setprecision(4) << fitness
            << " converged=" << (converged ? 1 : 0)
            << " innovation_m=" << innovation_m
            << " consecutive_count=" << consecutive_count
            << std::endl;
}

void RuntimeDiagnostics::logEkfRiskPredictionStreak(int count, double duration_sec,
                                                      double last_valid_stamp) {
  if (!cfg_.enabled) return;
  std::cout << "[EKF_RISK] reason=PREDICTION_STREAK"
            << " count=" << count
            << " duration_sec=" << std::fixed << std::setprecision(2) << duration_sec
            << " last_valid_measurement_stamp=" << std::setprecision(3) << last_valid_stamp
            << std::endl;
}

void RuntimeDiagnostics::logEkfRiskRecovery(const std::string& recovery_cause,
                                              int high_fitness_frames,
                                              int prediction_only_frames,
                                              double innovation_m, double fitness,
                                              double covariance_before, double covariance_after) {
  if (!cfg_.enabled) return;
  std::cout << "[EKF_RISK] reason=RECOVERY"
            << " recovery_cause=" << recovery_cause
            << " high_fitness_frames=" << high_fitness_frames
            << " prediction_only_frames=" << prediction_only_frames
            << " innovation_m=" << std::fixed << std::setprecision(4) << innovation_m
            << " fitness=" << fitness
            << " covariance_before=" << covariance_before
            << " covariance_after=" << covariance_after
            << std::endl;
}

void RuntimeDiagnostics::logMapCommitBlocked(const std::string& reason, double fitness,
                                               bool converged, bool prediction_only,
                                               bool step_valid,
                                               const std::string& target_source) {
  if (!cfg_.enabled) return;
  std::cout << "[MAP_COMMIT_BLOCKED]"
            << " reason=" << reason
            << " fitness=" << std::fixed << std::setprecision(4) << fitness
            << " converged=" << (converged ? 1 : 0)
            << " prediction_only=" << (prediction_only ? 1 : 0)
            << " step_valid=" << (step_valid ? 1 : 0)
            << " target_source=" << target_source
            << std::endl;
}

// ---- Cargo risk outputs ----

void RuntimeDiagnostics::logCargoRiskDetectionLost(double stamp, int track_id,
                                                     const std::string& previous_state,
                                                     const std::string& current_state,
                                                     int lost_frames,
                                                     double last_valid_bottom_z) {
  if (!cfg_.enabled) return;
  std::cout << "[CARGO_RISK] reason=DETECTION_LOST"
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " track_id=" << track_id
            << " previous_state=" << previous_state
            << " current_state=" << current_state
            << " lost_frames=" << lost_frames
            << " last_valid_bottom_z=" << std::setprecision(4) << last_valid_bottom_z
            << std::endl;
}

void RuntimeDiagnostics::logCargoRiskInsufficientPoints(double stamp, int track_id,
                                                          int cluster_points,
                                                          int required_points) {
  if (!cfg_.enabled) return;
  std::cout << "[CARGO_RISK] reason=INSUFFICIENT_POINTS"
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " track_id=" << track_id
            << " cluster_points=" << cluster_points
            << " required_points=" << required_points
            << std::endl;
}

void RuntimeDiagnostics::logCargoRiskBottomJump(double stamp, int track_id,
                                                  double previous_bottom_z,
                                                  double raw_bottom_z,
                                                  double filtered_bottom_z,
                                                  double delta_m, bool filter_accepted,
                                                  const std::string& filter_reason) {
  if (!cfg_.enabled) return;
  std::cout << "[CARGO_RISK] reason=BOTTOM_JUMP"
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " track_id=" << track_id
            << " previous_bottom_z=" << std::setprecision(4) << previous_bottom_z
            << " raw_bottom_z=" << raw_bottom_z
            << " filtered_bottom_z=" << filtered_bottom_z
            << " delta_m=" << delta_m
            << " filter_accepted=" << (filter_accepted ? 1 : 0)
            << " filter_reason=" << filter_reason
            << std::endl;
}

void RuntimeDiagnostics::logCargoRiskHeightJump(double stamp, int track_id,
                                                   double previous_height_m,
                                                   double height_m, double delta_m,
                                                   double bottom_z, double top_z) {
  if (!cfg_.enabled) return;
  std::cout << "[CARGO_RISK] reason=HEIGHT_JUMP"
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " track_id=" << track_id
            << " previous_height_m=" << std::setprecision(4) << previous_height_m
            << " height_m=" << height_m
            << " delta_m=" << delta_m
            << " bottom_z=" << bottom_z
            << " top_z=" << top_z
            << std::endl;
}

void RuntimeDiagnostics::logCargoRiskHeightInvalid(double stamp, int track_id,
                                                      bool bottom_valid, bool top_valid,
                                                      int cluster_points,
                                                      const std::string& state) {
  if (!cfg_.enabled) return;
  std::cout << "[CARGO_RISK] reason=HEIGHT_INVALID"
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " track_id=" << track_id
            << " bottom_valid=" << (bottom_valid ? 1 : 0)
            << " top_valid=" << (top_valid ? 1 : 0)
            << " cluster_points=" << cluster_points
            << " state=" << state
            << std::endl;
}

void RuntimeDiagnostics::logCargoRiskBoxGeometryJump(double stamp, int track_id,
                                                        double center_delta_m,
                                                        double size_delta_x,
                                                        double size_delta_y,
                                                        double size_delta_z,
                                                        double odom_step_m) {
  if (!cfg_.enabled) return;
  std::cout << "[CARGO_RISK] reason=BOX_GEOMETRY_JUMP"
            << " stamp=" << std::fixed << std::setprecision(3) << stamp
            << " track_id=" << track_id
            << " center_delta_m=" << std::setprecision(4) << center_delta_m
            << " size_delta_x=" << size_delta_x
            << " size_delta_y=" << size_delta_y
            << " size_delta_z=" << size_delta_z
            << " odom_step_m=" << odom_step_m
            << std::endl;
}

}  // namespace ndt_slam
