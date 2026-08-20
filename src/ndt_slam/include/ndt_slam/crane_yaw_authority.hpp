#pragma once

#include <cstddef>
#include <limits>
#include <string>

namespace ndt_slam {

enum class CraneYawAuthorityState {
  UNCONFIGURED = 0,
  CONFIG_HOLD = 1,
  FALLBACK_BOOTSTRAP = 2,
  PHYSICAL_CONFLICT_EVIDENCE = 3,
  INVALID = 4,
};

struct CraneYawAuthorityConfig {
  bool enabled = false;
  bool apply_to_runtime_pose = false;
  std::string reference_source = "CONFIG";
  double configured_base_yaw_in_map_rad =
      std::numeric_limits<double>::quiet_NaN();
  std::string map_frame_convention_id;
  std::string map_frame_convention_description;
  bool allow_first_reliable_fallback = false;
  std::size_t fallback_required_reliable_frames = 0U;

  double raw_yaw_threshold_rad =
      std::numeric_limits<double>::quiet_NaN();
  double rail_fitness_delta_threshold =
      std::numeric_limits<double>::quiet_NaN();
  double rail_translation_delta_threshold_m =
      std::numeric_limits<double>::quiet_NaN();
  std::size_t required_consecutive_frames = 0U;
};

struct CraneYawEvidence {
  double raw_ndt_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  bool yaw_observability_strong = false;
  bool rail_registration_valid = false;
  double rail_fitness_delta = std::numeric_limits<double>::quiet_NaN();
  double rail_translation_delta_m =
      std::numeric_limits<double>::quiet_NaN();
};

struct CraneYawAuthorityDecision {
  CraneYawAuthorityState state = CraneYawAuthorityState::UNCONFIGURED;
  bool configured = false;
  bool proposal_valid = false;
  bool raw_innovation_valid = false;
  bool composite_conflict_evidence = false;
  bool product_application_allowed = false;
  double authoritative_yaw_rad =
      std::numeric_limits<double>::quiet_NaN();
  double raw_minus_authoritative_yaw_rad =
      std::numeric_limits<double>::quiet_NaN();
  double raw_yaw_unwrapped_rad =
      std::numeric_limits<double>::quiet_NaN();
  double authoritative_yaw_unwrapped_rad =
      std::numeric_limits<double>::quiet_NaN();
  std::size_t conflict_evidence_frames = 0U;
  std::string reason = "not_evaluated";
};

class CraneYawAuthority {
 public:
  CraneYawAuthority() = default;
  explicit CraneYawAuthority(const CraneYawAuthorityConfig& config);

  void configure(const CraneYawAuthorityConfig& config);
  CraneYawAuthorityDecision observe(const CraneYawEvidence& evidence);
  const CraneYawAuthorityConfig& config() const { return config_; }
  const CraneYawAuthorityDecision& decision() const { return decision_; }

  static double shortestAngle(double angle_rad);
  static const char* stateName(CraneYawAuthorityState state);

 private:
  double unwrap(double wrapped, bool* initialized, double* previous_wrapped,
                double* unwrapped);

  CraneYawAuthorityConfig config_;
  CraneYawAuthorityDecision decision_;
  bool raw_unwrap_initialized_ = false;
  double previous_raw_wrapped_ = 0.0;
  double raw_unwrapped_ = 0.0;
  bool authority_unwrap_initialized_ = false;
  double previous_authority_wrapped_ = 0.0;
  double authority_unwrapped_ = 0.0;
  double fallback_sin_sum_ = 0.0;
  double fallback_cos_sum_ = 0.0;
  std::size_t fallback_reliable_frames_ = 0U;
  bool fallback_established_ = false;
  double fallback_yaw_rad_ = 0.0;
  std::size_t conflict_evidence_frames_ = 0U;
};

}  // namespace ndt_slam
