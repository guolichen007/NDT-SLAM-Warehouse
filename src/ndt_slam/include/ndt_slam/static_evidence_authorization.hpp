#pragma once

#include "ndt_slam/static_obstacle_evidence_index.hpp"

namespace ndt_slam {

// One typed authority decision is shared by origin binding, thickness fusion,
// official hazard publication and clear authorization. Unverified clean-map
// geometry remains available for diagnostics only.
struct StaticEvidenceAuthorization {
  bool diagnostic_height_allowed = true;
  bool formal_origin_authorized = false;
  bool formal_thickness_authorized = false;
  bool official_static_risk_authorized = false;
  bool official_clear_authorized = false;
};

inline StaticEvidenceAuthorization authorizeStaticEvidence(
    StaticEvidenceAuthority authority) noexcept {
  StaticEvidenceAuthorization result;
  const bool formal =
      authority == StaticEvidenceAuthority::RUNTIME_MATURE ||
      authority == StaticEvidenceAuthority::OPERATOR_APPROVED_BASELINE;
  result.formal_origin_authorized = formal;
  result.formal_thickness_authorized = formal;
  result.official_static_risk_authorized = formal;
  result.official_clear_authorized = formal;
  return result;
}

}  // namespace ndt_slam
