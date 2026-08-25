#pragma once

#include "ndt_slam/ndt_relocalizer.hpp"

#include <cstdint>
#include <string>

namespace ndt_slam {

enum class RelocalizationConfirmationOutcome {
  DISCARD_IDENTITY = 0,
  DISCARD_DUPLICATE,
  DISCARD_STALE,
  DISCARD_INVALID,
  CONFIRMING,
  CONFIRMED,
};

struct RelocalizationConfirmationConfig {
  int required_confirmations = 2;
  std::uint64_t maximum_age_frames = 15U;
  double maximum_age_sec = 1.50;
  double maximum_translation_delta_m = 0.50;
  double maximum_yaw_delta_deg = 5.0;
};

struct RelocalizationConfirmationInput {
  RelocalizationResult result;
  std::uint64_t current_frame_index = 0U;
  double current_stamp_sec = 0.0;
  std::uint64_t expected_map_generation = 0U;
  std::uint64_t expected_pose_version = 0U;
  std::uint64_t expected_yaw_authority_generation = 0U;
  std::string expected_yaw_reference_hash;
  std::uint64_t last_result_frame = 0U;
  int previous_confirmation_count = 0;
  Sophus::SE3d previous_correction;
};

struct RelocalizationConfirmationDecision {
  RelocalizationConfirmationOutcome outcome =
      RelocalizationConfirmationOutcome::DISCARD_INVALID;
  bool update_last_result_frame = false;
  std::uint64_t last_result_frame = 0U;
  int confirmation_count = 0;
  Sophus::SE3d correction;
  std::string reason = "not_evaluated";
};

RelocalizationConfirmationDecision evaluateRelocalizationConfirmation(
    const RelocalizationConfirmationInput& input,
    const RelocalizationConfirmationConfig& config =
        RelocalizationConfirmationConfig{});

// RAIL confirmation deliberately ignores free-search yaw.  The result pose
// has already identified a basin; only independently repeated XY correction
// may mature.  This API separation prevents legacy yaw gates/writers from
// being reintroduced into the rail authority path.
RelocalizationConfirmationDecision evaluateRailRelocalizationConfirmation(
    const RelocalizationConfirmationInput& input,
    const RelocalizationConfirmationConfig& config =
        RelocalizationConfirmationConfig{});

}  // namespace ndt_slam
