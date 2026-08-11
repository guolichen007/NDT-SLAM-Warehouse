#pragma once

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class StartupRecoveryState : std::uint8_t {
  BOOT = 0,
  LOAD_REFERENCE,
  SENSOR_WARMUP,
  LOCAL_RECOVERY,
  GLOBAL_RECOVERY,
  VERIFYING,
  READONLY_STABILIZING,
  ACTIVE,
  REFERENCE_INVALID,
  RECOVERY_RETRY,
};

struct StartupRecoveryInput {
  bool reference_load_finished = false;
  bool reference_present = false;
  bool reference_valid = false;
  bool first_boot = false;
  bool isolated_map_created = false;
  bool sensors_warm = false;
  bool checkpoint_available = false;
  bool local_recovery_finished = false;
  bool local_recovery_succeeded = false;
  bool global_recovery_finished = false;
  bool global_recovery_succeeded = false;
  bool verification_finished = false;
  bool verification_succeeded = false;
  bool map_write_rearmed = false;
  bool retry_ready = false;
};

struct StartupRecoveryDecision {
  StartupRecoveryState state = StartupRecoveryState::BOOT;
  bool startup_recovery_verified = false;
  bool map_write_rearmed = false;
  bool map_write_allowed = false;
  bool reset_runtime_evidence = false;
  bool create_isolated_map = false;
  std::uint64_t process_epoch = 0U;
  std::uint64_t continuity_generation = 0U;
  std::uint64_t lifecycle_epoch = 0U;
  std::string reason = "boot";
};

class StartupRecoveryController {
 public:
  StartupRecoveryController();

  // Starts a new process epoch. Persistent map data is intentionally not part
  // of this controller and cannot be cleared here.
  StartupRecoveryDecision boot();
  StartupRecoveryDecision update(const StartupRecoveryInput& input);
  const StartupRecoveryDecision& decision() const { return decision_; }

 private:
  void transition(StartupRecoveryState state, const std::string& reason);
  StartupRecoveryDecision decision_;
};

const char* startupRecoveryStateName(StartupRecoveryState state);

}  // namespace ndt_slam
