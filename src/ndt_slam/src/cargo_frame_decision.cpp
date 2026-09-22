#include "ndt_slam/cargo_frame_decision.hpp"

#include <cmath>

namespace ndt_slam {

CargoFrameCommitDecision commitCargoFrameDecision(
    const CargoFrameDecision& decision) {
  CargoFrameCommitDecision result;
  // 17/18/29 are all positive safety outcomes and must bind the same-frame
  // authoritative hazard identity. Non-positive statuses (0/14/33/34/35)
  // keep their original path and carry no positive hazard identity.
  const bool positive_code = decision.warning_code == 17 ||
      decision.warning_code == 18 || decision.warning_code == 29;
  if (!positive_code) {
    result.authorized = true;
    result.status_code = decision.warning_code;
    result.reason = "no_positive_warning_to_commit";
    return result;
  }
  // 17/18 additionally require a confirmed periodic warning. 29 is a review
  // episode and may commit with positive_warning_confirmed_this_frame=false,
  // but it still requires the full authoritative hazard identity below.
  if (decision.warning_code != 29 &&
      !decision.positive_warning_confirmed_this_frame) {
    result.reason = "positive_warning_not_confirmed";
    return result;
  }
  if (!std::isfinite(decision.stamp_sec) || decision.stamp_sec <= 0.0) {
    result.reason = "cargo_frame_stamp_invalid";
    return result;
  }
  if (!decision.cargo_identity_authorized ||
      decision.cargo_lifecycle_id == 0U || decision.cargo_track_id == 0U) {
    result.reason = "cargo_identity_not_authorized";
    return result;
  }
  if (!decision.authoritative_hazard_valid ||
      decision.obstacle_track_id == 0U) {
    result.reason = "authoritative_hazard_identity_missing";
    return result;
  }
  if (decision.authoritative_warning_code != decision.warning_code) {
    result.reason = "authoritative_hazard_code_mismatch";
    return result;
  }
  if (decision.warning_cargo_lifecycle_id !=
          decision.cargo_lifecycle_id ||
      decision.warning_cargo_track_id != decision.cargo_track_id) {
    result.reason = "authoritative_hazard_cargo_identity_mismatch";
    return result;
  }
  result.authorized = true;
  result.status_code = decision.warning_code;
  result.reason = decision.cargo_identity_confirmed_this_frame
      ? "identity_and_positive_warning_committed_same_frame"
      : "retained_identity_and_positive_warning_committed";
  return result;
}

}  // namespace ndt_slam
