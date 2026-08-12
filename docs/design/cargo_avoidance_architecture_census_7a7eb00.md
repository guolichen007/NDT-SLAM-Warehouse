# Cargo / Avoidance Architecture Census at 7a7eb00

This document freezes the engineering baseline before the behavior-preserving
Cargo/avoidance extraction. It describes ownership and execution; it does not
grant safety authority.

## Baseline

- Branch: `codex/334ba3b-local-map-self-healing-v1`
- Commit: `7a7eb007e74c660e0fef584c8bad45b00acc8358`
- `ndt_slam.cpp`: 28,780 lines, 1,410,511 bytes
- `ndt_slam.hpp`: 2,660 lines, 120,925 bytes
- Runtime: one `ndt_slam_node`; Cargo and avoidance are in-process.

## Current dataflow

`processCloudThread` time-aligns cloud and localization state, updates Cargo
identity/geometry, then calls `updateAndPublishCargoSafetyPipeline`.

- A retained Cargo with frozen geometry enters the formal path.
- A present Cargo without frozen geometry enters `runPendingCargoAvoidance`
  when a pending envelope is valid.
- Formal and pending paths publish different diagnostic clouds.
- Pending observations update both `pending_cargo_obstacle_tracker_` and
  `cargo_obstacle_tracker_`; formal observations update the latter.
- `CargoAvoidanceFusion` and `composeCargoSafetyDecision` determine whether
  physical evidence has enough authority to reach the typed status message.
- `/cargo_avoidance/safety_status` is authoritative. The heartbeat node is the
  only publisher of `/cargo_avoidance/status_code`.

## State ownership census

| State | Current writers/readers | Target owner |
| --- | --- | --- |
| Cargo lifecycle and track segment | Hook lock, presence transitions and node reset paths | CargoTracking |
| Measured/trusted/retired Cargo pose | NdtSlamNode Cargo pipeline | CargoTracking snapshot |
| Cargo dimensions, yaw, top, bottom and height | lock, bottom fusion, frozen and rigid geometry | CargoGeometry |
| Pending/effective/formal safety envelope | NdtSlamNode | CargoSafetyEnvelopeBuilder |
| Per-frame permissions | distributed `allowed/ready/valid` booleans | pure CargoCapability |
| Physical obstacle identity and far history | formal and pending trackers | PhysicalObstacleTrackStore |
| Distance/top/clearance | CargoSafetyEvaluator plus node mirrors | HazardEvaluator |
| Final protocol code | evaluator, fusion, temporal filter and status composition | AvoidanceDecision |
| Diagnostics | mutable node mirrors | read-only snapshot |

## Gate census

| Gate | Blocks today | Required invariant |
| --- | --- | --- |
| external output authorization | all Cargo safety output | retain |
| pending envelope valid | pending ROI and tracking | split horizontal and vertical validity |
| recognition allows warning | tracking and warning | replace with precise capabilities |
| Cargo height valid | clustering, tracking and warning | may block warning, never physical perception |
| cloud age/coverage | perception evidence | retain and diagnose separately |
| motion corridor | acquisition | never veto an established same-track hazard |
| far-history/provenance | 17/18 | one physical owner |
| formal CLEAR contract | 14 | retain |

## Effective configuration ownership

- `cargo_safety` owns obstacle geometry, tracking, far-history, Code29 and
  motion-corridor policy.
- `cargo_geometry_fusion`, `pending_cargo_envelope`,
  `pending_cargo_self_evidence`, `cargo_presence`, `cargo_physical_motion`,
  `cargo_recognition` and `cargo_swing` retain their domain-specific inputs.
- `status_codes` is a compatibility assertion, not a runtime override.
- The legacy warning block may be checked for compatibility but must not form
  a second 3 m / 5 m / 0.8 m policy.
- An explicitly invalid safety value must never select a complete C++ default
  policy.

## Topic inventory

| Topic family | Meaning |
| --- | --- |
| `safety_status` | authoritative typed decision |
| `status_code` | heartbeat-owned mirror of the typed decision |
| `recognition_status` | typed Cargo lifecycle and identity |
| raw status/code | diagnostic pre-heartbeat status |
| formal/pending clouds and markers | diagnostic transport; empty messages are valid observations |
| operational/pending/static/geometry status | diagnostic JSON; never an authority input |

Public topic names, message schemas, callback scheduling and process count are
frozen throughout the refactor.
