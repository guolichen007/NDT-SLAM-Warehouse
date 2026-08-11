#include "ndt_slam/startup_recovery_controller.hpp"

#include <atomic>
#include <chrono>

namespace ndt_slam {
namespace {

std::atomic<std::uint64_t> g_process_sequence{0U};

std::uint64_t newProcessEpoch() {
  const auto ticks = static_cast<std::uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  return ticks ^ (++g_process_sequence);
}

}  // namespace

StartupRecoveryController::StartupRecoveryController() { boot(); }

StartupRecoveryDecision StartupRecoveryController::boot() {
  decision_ = {};
  decision_.state = StartupRecoveryState::BOOT;
  decision_.process_epoch = newProcessEpoch();
  decision_.continuity_generation = 1U;
  decision_.lifecycle_epoch = decision_.process_epoch;
  decision_.reset_runtime_evidence = true;
  decision_.reason = "new_process_epoch_runtime_evidence_invalidated";
  return decision_;
}

void StartupRecoveryController::transition(
    StartupRecoveryState state, const std::string& reason) {
  decision_.state = state;
  decision_.reason = reason;
  decision_.reset_runtime_evidence = false;
  decision_.create_isolated_map = false;
  decision_.map_write_allowed = state == StartupRecoveryState::ACTIVE &&
      decision_.startup_recovery_verified && decision_.map_write_rearmed;
}

StartupRecoveryDecision StartupRecoveryController::update(
    const StartupRecoveryInput& input) {
  switch (decision_.state) {
    case StartupRecoveryState::BOOT:
      transition(StartupRecoveryState::LOAD_REFERENCE, "load_reference");
      break;
    case StartupRecoveryState::LOAD_REFERENCE:
      if (!input.reference_load_finished) break;
      if (input.first_boot && !input.reference_present) {
        decision_.startup_recovery_verified = true;
        decision_.map_write_rearmed = true;
        transition(StartupRecoveryState::ACTIVE, "first_boot_fresh_mapping");
      } else if (!input.reference_valid) {
        transition(StartupRecoveryState::REFERENCE_INVALID,
                   "persistent_reference_corrupted");
        decision_.create_isolated_map = true;
      } else {
        transition(StartupRecoveryState::SENSOR_WARMUP,
                   "reference_verified_readonly");
      }
      break;
    case StartupRecoveryState::SENSOR_WARMUP:
      if (input.sensors_warm) {
        transition(input.checkpoint_available
                       ? StartupRecoveryState::LOCAL_RECOVERY
                       : StartupRecoveryState::GLOBAL_RECOVERY,
                   input.checkpoint_available ? "checkpoint_local_search"
                                              : "global_place_search");
      }
      break;
    case StartupRecoveryState::LOCAL_RECOVERY:
      if (!input.local_recovery_finished) break;
      transition(input.local_recovery_succeeded
                     ? StartupRecoveryState::VERIFYING
                     : StartupRecoveryState::GLOBAL_RECOVERY,
                 input.local_recovery_succeeded
                     ? "local_candidate_requires_verification"
                     : "local_failed_global_fallback");
      break;
    case StartupRecoveryState::GLOBAL_RECOVERY:
      if (!input.global_recovery_finished) break;
      transition(input.global_recovery_succeeded
                     ? StartupRecoveryState::VERIFYING
                     : StartupRecoveryState::RECOVERY_RETRY,
                 input.global_recovery_succeeded
                     ? "global_candidate_requires_verification"
                     : "global_recovery_retry");
      break;
    case StartupRecoveryState::VERIFYING:
      if (!input.verification_finished) break;
      if (input.verification_succeeded) {
        decision_.startup_recovery_verified = true;
        transition(StartupRecoveryState::READONLY_STABILIZING,
                   "localization_verified_map_readonly");
      } else {
        transition(StartupRecoveryState::RECOVERY_RETRY,
                   "sequential_verification_failed");
      }
      break;
    case StartupRecoveryState::READONLY_STABILIZING:
      if (input.map_write_rearmed) {
        decision_.map_write_rearmed = true;
        transition(StartupRecoveryState::ACTIVE,
                   "map_write_rearmed_active");
      }
      break;
    case StartupRecoveryState::REFERENCE_INVALID:
      if (input.isolated_map_created) {
        ++decision_.continuity_generation;
        ++decision_.lifecycle_epoch;
        decision_.startup_recovery_verified = true;
        decision_.map_write_rearmed = true;
        transition(StartupRecoveryState::ACTIVE,
                   "isolated_fresh_map_active");
      }
      break;
    case StartupRecoveryState::RECOVERY_RETRY:
      if (input.retry_ready) {
        transition(input.checkpoint_available
                       ? StartupRecoveryState::LOCAL_RECOVERY
                       : StartupRecoveryState::GLOBAL_RECOVERY,
                   "automatic_recovery_retry");
      }
      break;
    case StartupRecoveryState::ACTIVE:
      break;
  }
  decision_.map_write_allowed =
      decision_.state == StartupRecoveryState::ACTIVE &&
      decision_.startup_recovery_verified && decision_.map_write_rearmed;
  return decision_;
}

const char* startupRecoveryStateName(StartupRecoveryState state) {
  switch (state) {
    case StartupRecoveryState::BOOT: return "BOOT";
    case StartupRecoveryState::LOAD_REFERENCE: return "LOAD_REFERENCE";
    case StartupRecoveryState::SENSOR_WARMUP: return "SENSOR_WARMUP";
    case StartupRecoveryState::LOCAL_RECOVERY: return "LOCAL_RECOVERY";
    case StartupRecoveryState::GLOBAL_RECOVERY: return "GLOBAL_RECOVERY";
    case StartupRecoveryState::VERIFYING: return "VERIFYING";
    case StartupRecoveryState::READONLY_STABILIZING:
      return "READONLY_STABILIZING";
    case StartupRecoveryState::ACTIVE: return "ACTIVE";
    case StartupRecoveryState::REFERENCE_INVALID: return "REFERENCE_INVALID";
    case StartupRecoveryState::RECOVERY_RETRY: return "RECOVERY_RETRY";
  }
  return "UNKNOWN";
}

}  // namespace ndt_slam
