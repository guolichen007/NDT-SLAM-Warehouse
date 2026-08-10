#pragma once

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class MappingAuthorityState : std::uint8_t {
  ACTIVE = 0,
  PAUSED_QUALITY = 1,
  PAUSED_IO = 2,
  FAIL_CLOSED = 3,
};

const char* mappingAuthorityStateName(MappingAuthorityState state) noexcept;

struct MappingRuntimePolicyConfig {
  int confirmed_hard_failure_frames = 3;
  double confirmed_hard_failure_duration_sec = 0.4;
};

struct MappingRuntimeEvidence {
  double stamp_sec = 0.0;
  bool ndt_converged = true;
  bool high_fitness = false;
  bool observability_invalid = false;
  bool hard_innovation_reject = false;
  bool geometry_invalid = false;
  bool nonfinite = false;
  bool severe_source_time_rollback = false;
  bool physical_impossibility = false;
  bool identity_corruption = false;
  bool io_paused = false;
};

struct MappingRuntimeDecision {
  MappingAuthorityState state = MappingAuthorityState::ACTIVE;
  bool trusted_writes_allowed = true;
  bool formal_warning_authority_allowed = true;
  bool transition = false;
  bool fail_closed_latched = false;
  int consecutive_hard_failure_frames = 0;
  double hard_failure_duration_sec = 0.0;
  std::string reason = "healthy";
};

class MappingRuntimePolicy {
 public:
  explicit MappingRuntimePolicy(
      const MappingRuntimePolicyConfig& config = {});

  void configure(const MappingRuntimePolicyConfig& config);
  MappingRuntimeDecision update(const MappingRuntimeEvidence& evidence);
  MappingRuntimeDecision latchFailClosed(const std::string& reason);
  void resetForNewSegment();
  const MappingRuntimeDecision& decision() const noexcept {
    return decision_;
  }

 private:
  MappingRuntimePolicyConfig config_;
  MappingRuntimeDecision decision_;
  double hard_failure_started_sec_ = 0.0;
  double previous_stamp_sec_ = 0.0;
};

}  // namespace ndt_slam
