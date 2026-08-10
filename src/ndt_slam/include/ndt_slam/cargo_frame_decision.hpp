#pragma once

#include <cstdint>
#include <string>

namespace ndt_slam {

// One immutable frame record joins cargo identity with positive hazard
// authority. It prevents a warning assembled later in the callback from
// carrying a zero, previous-lifecycle or half-updated cargo identity.
struct CargoFrameDecision {
  double stamp_sec = 0.0;
  bool cargo_identity_confirmed_this_frame = false;
  bool cargo_identity_authorized = false;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_track_id = 0U;
  bool positive_warning_confirmed_this_frame = false;
  std::int32_t warning_code = 0;
  bool authoritative_hazard_valid = false;
  std::int32_t authoritative_warning_code = 0;
  std::uint64_t warning_cargo_lifecycle_id = 0U;
  std::uint64_t warning_cargo_track_id = 0U;
  std::uint64_t obstacle_track_id = 0U;
};

struct CargoFrameCommitDecision {
  bool authorized = false;
  std::int32_t status_code = 35;
  std::string reason = "not_evaluated";
};

CargoFrameCommitDecision commitCargoFrameDecision(
    const CargoFrameDecision& decision);

}  // namespace ndt_slam
