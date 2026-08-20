#pragma once

#include <sophus/se3.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>

namespace ndt_slam {

struct RailTranslationRefinerConfig {
  bool enabled = false;
  bool shadow_only = true;
  double configured_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  double initial_step_m = 0.25;
  double minimum_step_m = 0.01;
  double trust_radius_m = 1.0;
  double deadline_ms = 20.0;
  std::size_t maximum_evaluations = 96U;
};

struct RailTranslationRefinerInput {
  Sophus::SE3d free_pose;
  Sophus::SE3d predicted_pose;
  double free_ndt_reported_fitness =
      std::numeric_limits<double>::quiet_NaN();
  std::uint64_t captured_target_version = 0U;
  std::uint64_t current_target_version = 0U;
};

struct RailTranslationRefinerResult {
  bool valid = false;
  Sophus::SE3d free_pose;
  Sophus::SE3d rail_pose;
  double free_ndt_reported_fitness =
      std::numeric_limits<double>::quiet_NaN();
  double free_fitness = std::numeric_limits<double>::quiet_NaN();
  double rail_fitness = std::numeric_limits<double>::quiet_NaN();
  double translation_delta_m =
      std::numeric_limits<double>::quiet_NaN();
  double fitness_delta = std::numeric_limits<double>::quiet_NaN();
  std::size_t evaluations = 0U;
  double elapsed_ms = 0.0;
  std::string reason = "not_evaluated";
};

using RailPoseObjective = std::function<double(const Sophus::SE3d&)>;

class RailConstrainedTranslationRefiner {
 public:
  explicit RailConstrainedTranslationRefiner(
      RailTranslationRefinerConfig config = {});

  void configure(const RailTranslationRefinerConfig& config);
  RailTranslationRefinerResult refine(
      const RailTranslationRefinerInput& input,
      const RailPoseObjective& objective) const;

 private:
  RailTranslationRefinerConfig config_;
};

}  // namespace ndt_slam
