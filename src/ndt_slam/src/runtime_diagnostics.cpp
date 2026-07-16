#include "ndt_slam/runtime_diagnostics.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace ndt_slam {

RuntimeDiagnostics::RuntimeDiagnostics()
    : last_csv_flush_(std::chrono::steady_clock::now()),
      last_health_console_(std::chrono::steady_clock::now()),
      last_cargo_console_(std::chrono::steady_clock::now()),
      last_pipeline_console_(std::chrono::steady_clock::now()),
      last_pipeline_risk_console_(std::chrono::steady_clock::now()),
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
  cfg_.health_period_sec = std::max(0.1, cfg_.health_period_sec);
  cfg_.risk_repeat_period_sec =
      std::max(0.1, cfg_.risk_repeat_period_sec);
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
             << "ndt_execution_state,ndt_converged,ndt_iterations,fitness,transformation_probability,"
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
               << "cluster_points,support_points,candidate_count,selected_candidate_id,"
               << "identity_score,orientation_confidence,shape_confidence,"
               << "motion_confidence,overall_lock_confidence,"
               << "obstacle_roi_finite_points,obstacle_roi_coverage_ratio,"
               << "self_removed_points,identity_self_removed_points,"
               << "rigging_self_removed_points,"
               << "external_obstacle_points,self_margin_xy_m,self_margin_z_m,"
               << "horizontal_uncertainty_m,vertical_uncertainty_m,"
               << "ground_reference_valid,ground_z,"
               << "dangerous_cluster_points,nearest_obstacle_x,"
               << "nearest_obstacle_y,nearest_obstacle_z,"
               << "nearest_cluster_center_x,"
               << "nearest_cluster_center_y,nearest_cluster_center_z,"
               << "nearest_cluster_distance,obstacle_top_z95_m,"
               << "obstacle_uncertainty_m,conservative_clearance_m,"
               << "requested_alarm_code,raw_warning_code,"
               << "confirmed_warning_code,temporal_candidate_code,"
               << "temporal_candidate_count,temporal_hold_age_sec,"
               << "used_previous_confirmation,obstacle_track_id,"
               << "obstacle_track_age_sec,obstacle_track_confirm_count,"
               << "obstacle_track_static,obstacle_track_velocity_x,"
               << "obstacle_track_velocity_y,obstacle_track_velocity_z,"
               << "safety_spatial_mode,cargo_map_speed_mps,"
               << "corridor_eligible_clusters,corridor_rejected_clusters,"
               << "safety_reason,"
               << "live_center_x,live_center_y,live_center_z,"
               << "measured_center_x,measured_center_y,measured_center_z,"
               << "predicted_center_x,predicted_center_y,predicted_center_z,"
               << "center_residual_x,center_residual_y,center_residual_z,"
               << "pose_sensor_dt_sec,position_source,vertical_position_source,"
               << "observed_top_z,frozen_thickness_m,pose_evidence_age_sec,"
               << "height_evidence_age_sec,"
               << "locked_length_m,locked_width_m,locked_height_m,locked_yaw_deg,"
               << "raw_bottom_z,filtered_bottom_z,conservative_bottom_z,"
               << "top_z,height_m,bottom_valid,height_valid,filter_accepted,filter_reason,"
               << "lost_frames,odom_x,odom_y,odom_z\n";
  }

  last_csv_flush_ = std::chrono::steady_clock::now();
  last_health_console_ = std::chrono::steady_clock::now();
  last_cargo_console_ = std::chrono::steady_clock::now();
  last_pipeline_console_ = std::chrono::steady_clock::now();
  last_pipeline_risk_console_ = std::chrono::steady_clock::now();
  last_pipeline_queue_drop_total_ = 0;
  last_pipeline_processed_total_ = 0;
  last_pipeline_overrun_count_ = 0;
  pipeline_risk_active_ = false;
  pipeline_risk_level_ = 0;
  pipeline_risk_reason_.clear();
  {
    std::lock_guard<std::mutex> lock(console_risk_mutex_);
    console_risk_states_.clear();
  }
}

void RuntimeDiagnostics::resetTimeEpoch() {
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(rate_mutex_);
    last_callback_sensor_stamp_ = 0.0;
    last_processed_sensor_stamp_ = 0.0;
    callback_sensor_dt_last_ms_ = 0.0;
    processed_sensor_dt_last_ms_ = 0.0;
    callback_wall_dt_last_ms_ = 0.0;
    processed_wall_dt_last_ms_ = 0.0;
    last_callback_wall_ = now;
    last_processed_wall_ = now;
  }
  pipeline_risk_active_ = false;
  pipeline_risk_level_ = 0;
  pipeline_risk_reason_.clear();
  consecutive_overruns_ = 0;
  consecutive_prediction_only_ = 0;
  consecutive_target_fallback_ = 0;
  last_health_console_ = now;
  last_cargo_console_ = now;
  last_pipeline_console_ = now;
  last_pipeline_risk_console_ = now;
  {
    std::lock_guard<std::mutex> lock(console_risk_mutex_);
    console_risk_states_.clear();
  }
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
  if (!cfg_.enabled || !cfg_.console_health_enabled) return;
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_pipeline_console_).count() <
      cfg_.health_period_sec) {
    return;
  }
  last_pipeline_console_ = now;
  const bool queue_dropped_since_last_report =
      rate.queue_drop_total > last_pipeline_queue_drop_total_;
  const uint64_t window_frames =
      rate.processed_total >= last_pipeline_processed_total_
          ? rate.processed_total - last_pipeline_processed_total_
          : rate.processed_total;
  const int window_overruns =
      frame_overrun_count_ >= last_pipeline_overrun_count_
          ? frame_overrun_count_ - last_pipeline_overrun_count_
          : frame_overrun_count_;
  const double overrun_ratio = window_frames > 0
      ? static_cast<double>(window_overruns) /
            static_cast<double>(window_frames)
      : 0.0;
  last_pipeline_queue_drop_total_ = rate.queue_drop_total;
  last_pipeline_processed_total_ = rate.processed_total;
  last_pipeline_overrun_count_ = frame_overrun_count_;
  std::cout << "[PIPELINE_HEALTH] window=" << std::fixed
            << std::setprecision(1) << cfg_.health_period_sec << "s"
            << " frames=" << window_frames
            << " overrun_ratio=" << std::setprecision(3) << overrun_ratio
            << " total_p95_ms=" << std::setprecision(1)
            << total_ms_stats_.p95()
            << " callback=" << rate.callback_total
            << " processed=" << rate.processed_total
            << " queue_drop=" << rate.queue_drop_total
            << " callback_hz=" << std::setprecision(2)
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
  if (!cfg_.enabled || !cfg_.console_health_enabled) return;
  std::cout << "[RUNCFG]" << std::endl;
  for (auto& kv : params) {
    std::cout << "  " << kv.first << "=" << kv.second << std::endl;
  }
}

void RuntimeDiagnostics::logMergerCfg(const std::map<std::string, std::string>& params) {
  if (!cfg_.enabled || !cfg_.console_health_enabled) return;
  std::cout << "[MERGER_CFG]" << std::endl;
  for (auto& kv : params) {
    std::cout << "  " << kv.first << "=" << kv.second << std::endl;
  }
}

void RuntimeDiagnostics::logBuildId(const std::map<std::string, std::string>& params) {
  if (!cfg_.enabled || !cfg_.console_health_enabled) return;
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
             << rec.ndt_execution_state << ","
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
  if (rec.ndt_execution_state.find("NDT_ATTEMPTED") == 0U &&
      rec.ndt_converged && std::isfinite(rec.fitness)) {
    fitness_stats_.add(rec.fitness);
  }

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
               << rec.candidate_count << "," << rec.selected_candidate_id << ","
               << rec.identity_score << "," << rec.orientation_confidence << ","
               << rec.shape_confidence << "," << rec.motion_confidence << ","
               << rec.overall_lock_confidence << ","
               << rec.obstacle_roi_finite_points << ","
               << rec.obstacle_roi_coverage_ratio << ","
               << rec.self_removed_points << ","
               << rec.identity_self_removed_points << ","
               << rec.rigging_self_removed_points << ","
               << rec.external_obstacle_points << ","
               << rec.self_margin_xy_m << "," << rec.self_margin_z_m << ","
               << rec.horizontal_uncertainty_m << ","
               << rec.vertical_uncertainty_m << ","
               << (rec.ground_reference_valid ? 1 : 0) << ","
               << rec.ground_z << ","
               << rec.dangerous_cluster_points << ","
               << rec.nearest_obstacle_x << ","
               << rec.nearest_obstacle_y << ","
               << rec.nearest_obstacle_z << ","
               << rec.nearest_cluster_center_x << ","
               << rec.nearest_cluster_center_y << ","
               << rec.nearest_cluster_center_z << ","
               << rec.nearest_cluster_distance << ","
               << rec.obstacle_top_z95_m << ","
               << rec.obstacle_uncertainty_m << ","
               << rec.conservative_clearance_m << ","
               << rec.requested_alarm_code << ","
               << rec.raw_warning_code << ","
               << rec.confirmed_warning_code << ","
               << rec.temporal_candidate_code << ","
               << rec.temporal_candidate_count << ","
               << rec.temporal_hold_age_sec << ","
               << (rec.used_previous_confirmation ? 1 : 0) << ","
               << rec.obstacle_track_id << ","
               << rec.obstacle_track_age_sec << ","
               << rec.obstacle_track_confirm_count << ","
               << (rec.obstacle_track_static ? 1 : 0) << ","
               << rec.obstacle_track_velocity_x << ","
               << rec.obstacle_track_velocity_y << ","
               << rec.obstacle_track_velocity_z << ","
               << rec.safety_spatial_mode << ","
               << rec.cargo_map_speed_mps << ","
               << rec.corridor_eligible_clusters << ","
               << rec.corridor_rejected_clusters << ","
               << rec.safety_reason << ","
               << std::setprecision(4)
               << rec.center_x << "," << rec.center_y << "," << rec.center_z << ","
               << rec.measured_center_x << "," << rec.measured_center_y << ","
               << rec.measured_center_z << ","
               << rec.predicted_center_x << "," << rec.predicted_center_y << ","
               << rec.predicted_center_z << ","
               << rec.center_residual_x << "," << rec.center_residual_y << ","
               << rec.center_residual_z << ","
               << rec.pose_sensor_dt_sec << "," << rec.position_source << ","
               << rec.vertical_position_source << ","
               << rec.observed_top_z << "," << rec.frozen_thickness_m << ","
               << rec.pose_evidence_age_sec << ","
               << rec.height_evidence_age_sec << ","
               << rec.size_x << "," << rec.size_y << "," << rec.size_z << ","
               << rec.footprint_yaw_deg << ","
               << std::setprecision(4)
               << rec.raw_bottom_z << "," << rec.filtered_bottom_z << ","
               << rec.stable_bottom_z << "," << rec.top_z << ","
               << rec.height_m << ","
               << (rec.bottom_valid ? 1 : 0) << "," << (rec.height_valid ? 1 : 0) << ","
               << (rec.filter_accepted ? 1 : 0) << "," << rec.filter_reason << ","
               << rec.lost_frames << "," << rec.odom_x << "," << rec.odom_y
               << "," << rec.odom_z << "\n";
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
  if (!cfg_.enabled || !cfg_.console_health_enabled) return;
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_health_console_).count() < cfg_.health_period_sec)
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
  if (!cfg_.enabled || !cfg_.console_health_enabled) return;
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
  if (!cfg_.enabled || !cfg_.cargo_console_enabled) return;
  const auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now - last_cargo_console_).count() <
      cfg_.health_period_sec) {
    return;
  }
  last_cargo_console_ = now;
  std::cout << "[CARGO_MONITOR]"
            << " stamp=" << std::fixed << std::setprecision(3) << rec.stamp
            << " safety_code=" << rec.requested_alarm_code
            << " safety_reason=" << rec.safety_reason
            << " track_state=" << rec.track_state
            << " track_id=" << rec.track_id
            << " lock_state=" << rec.lock_state
            << " observation_valid=" << (rec.observation_valid ? 1 : 0)
            << " points=" << rec.cluster_points
            << " support=" << rec.support_points
            << " live_center=(" << std::setprecision(3) << rec.center_x
            << "," << rec.center_y << "," << rec.center_z << ")"
            << " locked_shape=(" << rec.size_x << "," << rec.size_y
            << "," << rec.size_z << ")"
            << " vertical_source=" << rec.vertical_position_source
            << " observed_top=" << rec.observed_top_z
            << " frozen_thickness=" << rec.frozen_thickness_m
            << " locked_yaw_deg=" << std::setprecision(1)
            << rec.footprint_yaw_deg
            << " conservative_bottom=" << std::setprecision(3)
            << rec.stable_bottom_z
            << " top=" << rec.top_z
            << " height_valid=" << (rec.height_valid ? 1 : 0)
            << " external_points=" << rec.external_obstacle_points
            << " dangerous_cluster_points="
            << rec.dangerous_cluster_points
            << " nearest_distance=" << rec.nearest_cluster_distance
            << " clearance=" << rec.conservative_clearance_m
            << " reason=" << rec.filter_reason
            << std::endl;
}

// ---- Risk outputs ----

void RuntimeDiagnostics::logNdtRiskNotConverged(int frame, double stamp, double fitness,
                                                  int iterations, const std::string& target_source,
                                                  int target_points, int input_points,
                                                  double ndt_ms, double total_ms) {
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  if (!shouldEmitConsoleRisk("NDT_NOT_CONVERGED", "NOT_CONVERGED")) return;
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
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  if (!shouldEmitConsoleRisk("NDT_FITNESS_SPIKE", "FITNESS_SPIKE")) return;
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
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  if (!shouldEmitConsoleRisk("NDT_TARGET_TOO_SMALL", "TARGET_TOO_SMALL")) return;
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
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  if (!shouldEmitConsoleRisk(
          "NDT_TARGET_FALLBACK_STREAK", "TARGET_FALLBACK_STREAK")) return;
  std::cout << "[NDT_RISK] reason=TARGET_FALLBACK_STREAK"
            << " count=" << count
            << " current_source=" << current_source
            << " fallback_source=" << fallback_source
            << std::endl;
}

void RuntimeDiagnostics::writePipelineRisk(
    const char* event,
    const PipelineRiskRecord& rec,
    const std::string& previous_reason) {
  std::cout << "[PIPELINE_RISK_" << event << "]"
            << " reason=" << rec.reason
            << " level=" << rec.level;
  if (!previous_reason.empty()) {
    std::cout << " previous_reason=" << previous_reason;
  }
  std::cout << " frame=" << rec.frame
            << " stamp=" << std::fixed << std::setprecision(3) << rec.stamp
            << " frame_budget_ms=" << std::setprecision(1)
            << rec.frame_budget_ms
            << " total_ms=" << rec.total_ms
            << " preprocess_ms=" << rec.preprocess_ms
            << " target_prepare_ms=" << rec.target_prepare_ms
            << " ndt_align_ms=" << rec.ndt_align_ms
            << " ekf_ms=" << rec.ekf_ms
            << " map_commit_ms=" << rec.map_commit_ms
            << " consecutive_overruns=" << rec.consecutive_overruns
            << " estimated_backlog_frames="
            << rec.estimated_backlog_frames
            << " processed_hz=" << rec.processed_hz
            << " input_hz=" << rec.input_hz
            << " drop_ratio=" << std::setprecision(3) << rec.drop_ratio
            << std::endl;
}

void RuntimeDiagnostics::updatePipelineRisk(const PipelineRiskRecord& rec) {
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  const auto now = std::chrono::steady_clock::now();
  const bool entering = !pipeline_risk_active_;
  const bool changed = pipeline_risk_active_ &&
      (rec.reason != pipeline_risk_reason_ ||
       rec.level != pipeline_risk_level_);
  const bool repeat_due = pipeline_risk_active_ && !changed &&
      std::chrono::duration<double>(
          now - last_pipeline_risk_console_).count() >=
          cfg_.risk_repeat_period_sec;
  if (!entering && !changed && !repeat_due) return;

  const std::string previous_reason = pipeline_risk_reason_;
  writePipelineRisk(entering ? "ENTER" : (changed ? "CHANGE" : "REPEAT"),
                    rec, changed ? previous_reason : "");
  pipeline_risk_active_ = true;
  pipeline_risk_reason_ = rec.reason;
  pipeline_risk_level_ = rec.level;
  last_pipeline_risk_console_ = now;
}

void RuntimeDiagnostics::clearPipelineRisk(const PipelineRiskRecord& rec) {
  if (!cfg_.enabled || !cfg_.console_risk_enabled || !pipeline_risk_active_) return;
  PipelineRiskRecord cleared = rec;
  cleared.reason = pipeline_risk_reason_;
  cleared.level = pipeline_risk_level_;
  writePipelineRisk("CLEAR", cleared);
  pipeline_risk_active_ = false;
  pipeline_risk_reason_.clear();
  pipeline_risk_level_ = 0;
}

bool RuntimeDiagnostics::shouldEmitConsoleRisk(
    const std::string& key,
    const std::string& reason) {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(console_risk_mutex_);
  auto found = console_risk_states_.find(key);
  if (found == console_risk_states_.end()) {
    console_risk_states_[key] = ConsoleRiskState{reason, now};
    return true;
  }
  const bool changed = found->second.reason != reason;
  const bool repeat_due = std::chrono::duration<double>(
      now - found->second.last_emit).count() >= cfg_.risk_repeat_period_sec;
  if (!changed && !repeat_due) return false;
  found->second.reason = reason;
  found->second.last_emit = now;
  return true;
}

void RuntimeDiagnostics::clearConsoleRisk(
    const std::string& key,
    const std::string& output_tag) {
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  std::string previous_reason;
  {
    std::lock_guard<std::mutex> lock(console_risk_mutex_);
    const auto found = console_risk_states_.find(key);
    if (found == console_risk_states_.end()) return;
    previous_reason = found->second.reason;
    console_risk_states_.erase(found);
  }
  std::cout << "[" << output_tag << "_CLEAR]"
            << " type=" << key
            << " reason=" << previous_reason
            << std::endl;
}

void RuntimeDiagnostics::logOdomRiskRawStepExceeded(int frame, double stamp,
                                                       double sensor_dt_ms,
                                                       double raw_dx, double raw_dy, double raw_dz,
                                                       double raw_step_m, double allowed_step_m,
                                                       double fitness, bool converged) {
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  if (!shouldEmitConsoleRisk("ODOM_RAW_STEP", "RAW_STEP_EXCEEDED")) return;
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
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  if (!shouldEmitConsoleRisk(
          "ODOM_OUTPUT_STEP", "OUTPUT_STEP_VIOLATION")) return;
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
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  if (!shouldEmitConsoleRisk("EKF_PREDICTION_ONLY", cause)) return;
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
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  if (!shouldEmitConsoleRisk(
          "EKF_PREDICTION_STREAK", "PREDICTION_STREAK")) return;
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
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  clearConsoleRisk("EKF_PREDICTION_ONLY", "EKF_RISK");
  clearConsoleRisk("EKF_PREDICTION_STREAK", "EKF_RISK");
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
  if (!cfg_.enabled || !cfg_.console_risk_enabled) return;
  if (!shouldEmitConsoleRisk("MAP_COMMIT_BLOCKED", reason)) return;
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
  if (!cfg_.enabled || !cfg_.cargo_console_enabled) return;
  if (!shouldEmitConsoleRisk("CARGO_DETECTION_LOST", "DETECTION_LOST")) return;
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
  if (!cfg_.enabled || !cfg_.cargo_console_enabled) return;
  if (!shouldEmitConsoleRisk(
          "CARGO_INSUFFICIENT_POINTS", "INSUFFICIENT_POINTS")) return;
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
  if (!cfg_.enabled || !cfg_.cargo_console_enabled) return;
  if (!shouldEmitConsoleRisk("CARGO_BOTTOM_JUMP", filter_reason)) return;
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
  if (!cfg_.enabled || !cfg_.cargo_console_enabled) return;
  if (!shouldEmitConsoleRisk("CARGO_HEIGHT_JUMP", "HEIGHT_JUMP")) return;
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
  if (!cfg_.enabled || !cfg_.cargo_console_enabled) return;
  if (!shouldEmitConsoleRisk("CARGO_HEIGHT_INVALID", state)) return;
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
  if (!cfg_.enabled || !cfg_.cargo_console_enabled) return;
  if (!shouldEmitConsoleRisk(
          "CARGO_BOX_GEOMETRY_JUMP", "BOX_GEOMETRY_JUMP")) return;
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
