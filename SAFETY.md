# Safety

This project implements a typed cargo-safety contract for warehouse crane
operations. It is not a safety-certified device.

## Status Codes

| Code | Meaning | Authority |
|---:|---|---|
| 14 | CLEAR — no collision risk | Requires formal geometry + all contracts |
| 17 | NEAR_3M — ≤3 m, clearance < 0.8 m | Formal or degraded geometry |
| 18 | NEAR_5M — 3–5 m, clearance < 0.8 m | Formal or degraded geometry |
| 30 | System not ready | Fault |
| 31 | Localization invalid | Fault |
| 32 | Gravity / load-cell invalid | Fault |
| 33 | Cargo evidence invalid | Fault |
| 34 | Obstacle evidence invalid | Fault |
| 35 | Internal contract error | Fault |

## Key Safety Properties

### 14 is explicit CLEAR

Code 14 means the system has positively determined no collision risk. It
requires:
- Formal (authorized) cargo geometry
- Valid obstacle observation with no detected hazard
- All safety contracts satisfied

Degraded geometry **cannot** produce Code 14.

### 17 / 18 are positive collision warnings

Code 17 and 18 mean a real spatial collision risk has been detected. They
require:
- A valid obstacle track at the reported distance
- Vertical clearance below 0.8 m
- Consecutive validated observations

### 30–35 are not CLEAR

Codes 30–35 represent system faults or evidence problems. They must never
be interpreted as "safe" or "clear." If the system cannot positively
determine safety, it outputs a fault code.

### Degraded geometry: warn but don't clear

Live-only (degraded) geometry that has not achieved formal authorization:
- **CAN** produce positive 17 / 18 warnings
- **CANNOT** produce CLEAR 14
- **CANNOT** remove cargo points from registration
- **CANNOT** exclude from static map
- **CANNOT** commit to MapCommit

### Display markers are not safety evidence

RViz markers (cargo_core_bbox_marker, cargo_tight_box_marker,
cargo_warning_zone_marker) are visualization aids. They do not carry
safety authority.

### Historical / retired evidence

Stale evidence (expired LOST_HOLD, retired static snapshots, old pending
envelopes) cannot produce CLEAR or authorize map mutation.

## Deployment Safety

This software is not a safety-certified device. Deployment must retain:

- External emergency stop circuits
- Physical limit switches
- Site safety policies and procedures
- Independent operational oversight

The typed safety contract (CargoSafetyStatus schema v6) is the authoritative
output. Downstream controllers must:
- Treat codes 30–35 as non-clear
- Not infer CLEAR from absence of 17/18
- Implement their own timeout / watchdog on the status stream
- Maintain independent safety logic

## Changing Safety Behaviour

Any PR that changes:
- Warning codes (14, 17, 18, 30–35)
- Distance bands (3 m, 5 m)
- Clearance threshold (0.8 m)
- Message schema
- Geometry authority rules
- Source-validation fail-safe

Must:
- State the safety-contract impact explicitly
- Include before/after SHA
- Pass static contracts, clean build, gtest, bag, and field verification
- Not rely on Windows-only checks

## Validation Baseline

Field-validated safety baseline:
- **SHA**: `8d7d7eed0548321bf0646232f374fe95a29990dd`
- **Tag**: `validation-obstacle-avoidance-20260728`
- **Evidence**: Code 18 → S3 voice alarm path confirmed in field

Roll back to this SHA if a regression in safety behaviour is suspected.
