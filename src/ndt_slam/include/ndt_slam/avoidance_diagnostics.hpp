#pragma once

#include <cstddef>
#include <mutex>
#include <string>
#include <utility>

namespace ndt_slam {

struct AvoidanceDiagnosticsSnapshot {
  double source_stamp_sec = 0.0;
  std::string perception_phase = "NONE";
  bool perception_executed = false;
  bool query_geometry_valid = false;
  std::string query_geometry_source = "NONE";
  double query_geometry_age_sec = 0.0;
  bool query_horizontal_valid = false;
  bool query_vertical_valid = false;
  bool roi_attempted = false;
  std::size_t roi_point_count = 0U;
  bool external_extraction_executed = false;
  std::size_t external_point_count = 0U;
  bool clustering_executed = false;
  std::size_t cluster_count = 0U;
  bool tracking_attempted = false;
  std::size_t observation_count = 0U;
  std::size_t track_created_count = 0U;
  bool warning_evaluation_attempted = false;
  bool warning_authority_valid = false;
  std::string block_reason = "FORMAL_PERCEPTION_NOT_RUN";
};

// Diagnostics is a post-decision observer. The business pipeline can replace
// a completed-frame snapshot, but it cannot read this store to make a gate.
class AvoidanceDiagnosticsStore {
 public:
  void replace(AvoidanceDiagnosticsSnapshot snapshot) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = std::move(snapshot);
  }

  AvoidanceDiagnosticsSnapshot snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

 private:
  mutable std::mutex mutex_;
  AvoidanceDiagnosticsSnapshot snapshot_;
};

}  // namespace ndt_slam
