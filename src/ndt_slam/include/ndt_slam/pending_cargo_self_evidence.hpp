#pragma once

#include "ndt_slam/pending_cargo_envelope.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace ndt_slam {

enum class PendingPointClass : std::uint8_t {
  OUTSIDE_QUERY = 0,
  IDENTITY_SELF,
  EXTERNAL_SHELL,
  UNRESOLVED_INSIDE_PENDING,
};

struct PendingCargoSelfEvidenceConfig {
  float minimum_identity_confidence = 0.55F;
  float minimum_shape_confidence = 0.55F;
  float point_match_radius_m = 0.10F;
  double maximum_candidate_age_sec = 0.50;
  double maximum_retired_age_sec = 8.0;
};

struct PendingCargoSelfEvidenceInput {
  double stamp_sec = 0.0;
  PendingCargoEnvelopeSource source = PendingCargoEnvelopeSource::NONE;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t track_segment_id = 0U;
  std::uint64_t provisional_track_id = 0U;
  std::uint64_t candidate_track_id = 0U;
  bool candidate_current = false;
  float identity_confidence = 0.0F;
  float shape_confidence = 0.0F;
  bool retired_track_was_locked = false;
  std::uint64_t retired_cargo_lifecycle_id = 0U;
  bool retired_formal_obb_authorized = false;
  CargoObbFootprint tight_identity_obb;
  std::vector<Eigen::Vector3f> identity_points_base;
  double evidence_stamp_sec = 0.0;
};

struct PendingCargoSelfEvidence {
  bool valid = false;
  PendingCargoEnvelopeSource source = PendingCargoEnvelopeSource::NONE;
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t track_segment_id = 0U;
  CargoObbFootprint tight_identity_obb;
  std::vector<Eigen::Vector3f> identity_points_base;
  float point_match_radius_m = 0.10F;
  double evidence_stamp_sec = 0.0;
  bool formal_obb_only_authorized = false;
  std::string reason = "unavailable";
};

struct PendingPointClassification {
  PendingPointClass classification = PendingPointClass::OUTSIDE_QUERY;
  float envelope_distance_m = std::numeric_limits<float>::infinity();
  float identity_distance_m = std::numeric_limits<float>::infinity();
};

PendingCargoSelfEvidence buildPendingCargoSelfEvidence(
    const PendingCargoSelfEvidenceInput& input,
    const PendingCargoSelfEvidenceConfig& config =
        PendingCargoSelfEvidenceConfig{});

PendingPointClassification classifyPendingCargoPoint(
    const Eigen::Vector3f& point_base,
    const PendingCargoEnvelope& envelope,
    const PendingCargoSelfEvidence& self_evidence,
    float query_shell_m);

}  // namespace ndt_slam
