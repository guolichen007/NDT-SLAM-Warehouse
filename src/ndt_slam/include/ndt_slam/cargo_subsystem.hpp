#pragma once

#include "ndt_slam/cargo_capability.hpp"
#include "ndt_slam/cargo_domain_contracts.hpp"
#include "ndt_slam/pending_cargo_envelope.hpp"

#include <cstdint>
#include <string>

namespace ndt_slam {

struct CargoSubsystemFrameInput {
  double source_stamp_sec = 0.0;
  double evaluation_stamp_sec = 0.0;
  std::string frame_id = "base_link";
  std::uint64_t cargo_lifecycle_id = 0U;
  std::uint64_t cargo_track_id = 0U;
  PendingCargoEnvelope envelope;
  CargoVerticalAuthority vertical_authority =
      CargoVerticalAuthority::INVALID;
  bool config_valid = true;
  bool external_output_authorized = true;
  bool identity_valid = false;
  bool lifecycle_valid = false;
  bool cloud_fresh = false;
  bool vertical_geometry_valid = false;
  bool positive_identity_authorized = false;
  bool formal_geometry_valid = false;
  bool formal_clear_contract_valid = false;
  bool formal_removal_contract_valid = false;
};

CargoContractSource cargoContractSource(
    CargoEnvelopePoseSource source) noexcept;

struct CargoSubsystemSnapshot {
  CargoTrackSnapshot track;
  CargoGeometryEstimate geometry;
  CargoSafetyEnvelope envelope;
  CargoCapability capability;
};

class CargoSubsystem {
 public:
  const CargoSubsystemSnapshot& update(
      const CargoSubsystemFrameInput& input);
  const CargoSubsystemSnapshot& snapshot() const noexcept {
    return snapshot_;
  }
  void reset() noexcept { snapshot_ = CargoSubsystemSnapshot{}; }

 private:
  CargoSubsystemSnapshot snapshot_;
};

}  // namespace ndt_slam
