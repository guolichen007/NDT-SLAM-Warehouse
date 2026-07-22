#include "ndt_slam/pending_cargo_self_evidence.hpp"

#include <algorithm>
#include <cmath>

namespace ndt_slam {
namespace {

bool validConfig(const PendingCargoSelfEvidenceConfig& config) {
  return std::isfinite(config.minimum_identity_confidence) &&
      config.minimum_identity_confidence >= 0.0F &&
      config.minimum_identity_confidence <= 1.0F &&
      std::isfinite(config.minimum_shape_confidence) &&
      config.minimum_shape_confidence >= 0.0F &&
      config.minimum_shape_confidence <= 1.0F &&
      std::isfinite(config.point_match_radius_m) &&
      config.point_match_radius_m > 0.0F &&
      std::isfinite(config.maximum_candidate_age_sec) &&
      config.maximum_candidate_age_sec >= 0.0 &&
      std::isfinite(config.maximum_retired_age_sec) &&
      config.maximum_retired_age_sec >= 0.0;
}

bool validFootprint(const CargoObbFootprint& footprint) {
  return footprint.valid && footprint.center_base.allFinite() &&
      std::isfinite(footprint.length_m) && footprint.length_m > 0.0F &&
      std::isfinite(footprint.width_m) && footprint.width_m > 0.0F &&
      std::isfinite(footprint.yaw_base_rad) &&
      std::isfinite(footprint.min_z) && std::isfinite(footprint.max_z) &&
      footprint.max_z > footprint.min_z;
}

}  // namespace

PendingCargoSelfEvidence buildPendingCargoSelfEvidence(
    const PendingCargoSelfEvidenceInput& input,
    const PendingCargoSelfEvidenceConfig& requested_config) {
  PendingCargoSelfEvidence result;
  const PendingCargoSelfEvidenceConfig config = validConfig(requested_config)
      ? requested_config : PendingCargoSelfEvidenceConfig{};
  result.source = input.source;
  result.cargo_lifecycle_id = input.cargo_lifecycle_id;
  result.track_segment_id = input.track_segment_id;
  result.point_match_radius_m = config.point_match_radius_m;
  result.evidence_stamp_sec = input.evidence_stamp_sec;
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0 ||
      !std::isfinite(input.evidence_stamp_sec) ||
      input.evidence_stamp_sec <= 0.0 ||
      input.evidence_stamp_sec > input.stamp_sec + 1.0e-4 ||
      input.cargo_lifecycle_id == 0U || !validFootprint(input.tight_identity_obb)) {
    result.reason = "invalid_identity_contract";
    return result;
  }
  const double age_sec = input.stamp_sec - input.evidence_stamp_sec;
  if (input.source == PendingCargoEnvelopeSource::CURRENT_CANDIDATE) {
    if (!input.candidate_current || input.candidate_track_id == 0U ||
        input.candidate_track_id != input.provisional_track_id ||
        input.identity_confidence < config.minimum_identity_confidence ||
        input.shape_confidence < config.minimum_shape_confidence ||
        input.identity_points_base.empty() ||
        age_sec > config.maximum_candidate_age_sec) {
      result.reason = "candidate_identity_not_authorized";
      return result;
    }
  } else if (input.source ==
             PendingCargoEnvelopeSource::RETIRED_FORMAL_SHAPE) {
    if (!input.retired_track_was_locked ||
        input.retired_cargo_lifecycle_id != input.cargo_lifecycle_id ||
        age_sec > config.maximum_retired_age_sec ||
        (input.identity_points_base.empty() &&
         !input.retired_formal_obb_authorized)) {
      result.reason = "retired_identity_not_authorized";
      return result;
    }
    result.formal_obb_only_authorized =
        input.identity_points_base.empty() &&
        input.retired_formal_obb_authorized;
  } else {
    result.reason = "envelope_source_cannot_mask_live_points";
    return result;
  }
  result.tight_identity_obb = input.tight_identity_obb;
  result.identity_points_base.reserve(input.identity_points_base.size());
  for (const Eigen::Vector3f& point : input.identity_points_base) {
    if (point.allFinite()) result.identity_points_base.push_back(point);
  }
  if (result.identity_points_base.empty() &&
      !result.formal_obb_only_authorized) {
    result.reason = "no_finite_identity_points";
    return result;
  }
  result.valid = true;
  result.reason = result.formal_obb_only_authorized
      ? "retired_formal_obb_identity" : "identity_points_authorized";
  return result;
}

PendingPointClassification classifyPendingCargoPoint(
    const Eigen::Vector3f& point_base,
    const PendingCargoEnvelope& envelope,
    const PendingCargoSelfEvidence& self_evidence,
    float query_shell_m) {
  PendingPointClassification result;
  if (!point_base.allFinite() || !envelope.valid ||
      !std::isfinite(query_shell_m) || query_shell_m < 0.0F) {
    return result;
  }
  const CargoObbFootprint envelope_footprint =
      toCargoObbFootprint(envelope);
  result.envelope_distance_m = pointToCargoObbDistance2D(
      point_base.head<2>(), envelope_footprint);
  const bool in_vertical_query =
      point_base.z() >= envelope_footprint.min_z - query_shell_m &&
      point_base.z() <= envelope_footprint.max_z + query_shell_m;
  if (!in_vertical_query || result.envelope_distance_m > query_shell_m) {
    return result;
  }

  if (self_evidence.valid &&
      containsPointInCargoObbBase(
          point_base, self_evidence.tight_identity_obb, 0.0F, 0.0F)) {
    if (self_evidence.formal_obb_only_authorized) {
      result.identity_distance_m = 0.0F;
      // A retired formal box is a motion-envelope prior, not point identity.
      // Anything inside remains unresolved unless retained identity points
      // independently match it.
      result.classification =
          PendingPointClass::UNRESOLVED_INSIDE_PENDING;
      return result;
    }
    float nearest_squared = std::numeric_limits<float>::infinity();
    for (const Eigen::Vector3f& identity :
         self_evidence.identity_points_base) {
      nearest_squared = std::min(
          nearest_squared, (point_base - identity).squaredNorm());
    }
    result.identity_distance_m = std::sqrt(nearest_squared);
    if (result.identity_distance_m <= self_evidence.point_match_radius_m) {
      result.classification = PendingPointClass::IDENTITY_SELF;
      return result;
    }
  }

  if (containsPointInCargoObbBase(
          point_base, envelope_footprint, 0.0F, 0.0F)) {
    result.classification = PendingPointClass::UNRESOLVED_INSIDE_PENDING;
  } else {
    result.classification = PendingPointClass::EXTERNAL_SHELL;
  }
  return result;
}

}  // namespace ndt_slam
