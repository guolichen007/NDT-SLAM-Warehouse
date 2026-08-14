# Stage 2 Stability/Avoidance Integration Audit

## Provenance and gate

- Stability baseline / `TIME_EPOCH_FIX_SHA`:
  `b5565fe648cfff6a9a1486bcdb96c291c63d99cf`
- Avoidance semantic source:
  `b88653879f918e7466c99ae69bf5327df8fa5308`
- Merge base: `42f921e91574ff0f29b91fc08cbade67c976aa36`
- Integration branch: `codex/integration-stability-avoidance-v1`
- Stage 1 runtime evidence: `TIME_EPOCH_FIX_RUNTIME_CONFIRMED=YES`,
  `EPOCH_RESET_THEN_RESTART=PASS`, `READY_FOR_STAGE2=YES`.

The integration is a semantic port, not a merge or cherry-pick of the source
history. Stability-owned source-time, registration, map persistence, NDT/EKF,
relocalization, PointCloudMerger and operations policies remain on Stage 1.

## TIME_DOMAIN_MATRIX

| State or scheduler | Domain | Rollback action |
|---|---|---|
| Keyframe outer gate | LiDAR source | Stage 1 rebase |
| KeyFrameManager temporal gate | LiDAR source | Stage 1 narrow rebase |
| Cargo evidence and LiveOBB hold | LiDAR source | reset/reacquire |
| PhysicalObstacleTrack age/association/far-history | LiDAR source | reset; far history false |
| PendingStaticHazard temporal maturity | LiDAR source | reset |
| Code29 review episode | LiDAR source | reset |
| CargoSubsystem frame snapshot | LiDAR source | reset |
| ROS simulated time | ROS sim | observation metadata only |
| runtime status / health / dirty-tile interval | steady | Stage 1 unchanged |
| watchdog / process heartbeat | wall/steady | unchanged |

No Stage 2 scheduler adopts `negative delta => due` as a general policy.

## LOCK_ORDER_AUDIT

Stage 2 does not change Stage 1 map-commit locking. The retained order is:

```text
runtime_state_mutex_
  -> map_commit_lifecycle_mutex_
  -> subsystem ownership lock
```

Avoidance reset runs from the process-owner rollback path after the existing
lifecycle fence. It takes only subsystem-local locks. The diagnostics store is
a post-decision mutex-protected observer. Tile flush does not acquire Cargo,
Obstacle or diagnostics locks, and avoidance readiness never owns mapping
authority.

## MAP_COMMIT_EPOCH_CONTRACT

Stage 2 leaves the Stage 1 lifecycle epoch implementation byte/semantically
unchanged. Stale completion cannot restore old temporal authority; valid dirty
map content remains retryable; no Cargo/Obstacle state participates in
keyframe, MapCommit or TileFlush gates.

`AVOIDANCE_CAN_BLOCK_MAPPING_RECOVERY=NO`.

## OWNERSHIP_AND_DEPENDENCY_MATRIX

| File/symbol group | Direct dependencies | Class | Adapter / exception |
|---|---|---|---|
| cargo config validation, capability, domain contracts | Eigen, typed Cargo data | AVOIDANCE | exact semantic port |
| Cargo geometry/OBB/fusion/preload/subsystem | stable pose and static-height read API | AVOIDANCE | manual call-site adapter |
| ObstaclePerception/HazardEvaluator | PCL cloud, Cargo OBB | AVOIDANCE | canonical one-frame result |
| PhysicalObstacleTrackStore | map-frame observations | AVOIDANCE | one store; Pending/Formal policy projections |
| CargoAvoidanceFusion/AuthoritativeCargoHazard | typed live/static risks | AVOIDANCE | single-source code/metrics/identity |
| CargoFrameDecision/AvoidanceDecisionOwner | immutable frame records | AVOIDANCE | manual node adapter |
| CargoSafetyStatus schema 7/heartbeat | ROS message generation | STABLE_API/ABI | rebuild every consumer |
| static height/evidence query | stability static-map API | STABLE_API | read-only; no source module port |
| `ndt_slam.cpp/.hpp` Cargo call sites | stable localization/map state | SHARED_HOTSPOT | hand-edited Cargo-only hunks |
| persistent loader, MapCommit, NDT/EKF, relocalization | system policy | FORBIDDEN_SYSTEM_DEP | not imported |
| PointCloudMerger, watchdog, systemd/service | runtime operations | FORBIDDEN_SYSTEM_DEP | byte-identical/not touched |

`DEPENDENCY_EXCEPTION=NONE`. Static evidence is consumed through the existing
stable read interface; no b886 system policy was pulled to satisfy an
avoidance dependency.

## SHARED HUNK AUDIT

| Hunk owner | Target symbol | System state touched | NDT/EKF/MAP/RELOCALIZATION/COMMIT |
|---|---|---|---|
| CARGO | `initializeParameters` Cargo sections | Cargo config only | NO |
| CARGO | `resetCargoForHookState` | Cargo/Obstacle transient state | TIME narrow reset only |
| CARGO | `resetCargoAfterPoseDiscontinuity` | Cargo review/subsystem state | existing relocalization caller; no policy change |
| CARGO | `runPendingCargoAvoidance` | Cargo/Obstacle authority | NO |
| CARGO | `updateAndPublishCargoSafetyPipeline` | Cargo/Obstacle authority | reads stable pose/map snapshots only |
| DIAGNOSTIC | status composition/operational JSON | observer output | NO |

No Stage 2 hunk modifies NDT, EKF or relocalization thresholds, keyframe or
MapCommit decisions, persistent registration restore, watchdog or services.

## AVOIDANCE_TIME_EPOCH_AUDIT

| Temporal state | Classification | Reset path |
|---|---|---|
| Cargo geometry evidence/hold/lifecycle identity | SOURCE_TIME | `resetCargoForHookState` |
| Physical track age, confirmation and association | SOURCE_TIME | physical store reset |
| live far-history maturity | SOURCE_TIME | physical store reset; default false |
| static hazard history | SOURCE_TIME | pending-static tracker reset |
| Pending/Formal temporal filter | SOURCE_TIME | temporal-filter reset |
| Code29 episode/cooldown | SOURCE_TIME | anomaly tracker reset |
| Cargo capability snapshot | SOURCE_TIME | subsystem reset |
| runtime diagnostics | observer snapshot | replaced with NOT_EXECUTED |

Persistent static map/evidence and persistent registration are retained. A
new source epoch must reacquire Cargo identity and accumulate new obstacle far
history before 17/18. A missing Cargo bottom may keep physical 5-8 m tracking
alive, but cannot grant warning, clearance or CLEAR authority.

## MESSAGE_CONSUMER_AUDIT

`CargoSafetyStatus` is schema 7 with Code29 and evidence state 8.

| Consumer | Role | Required action |
|---|---|---|
| `NdtSlamNode` | publisher (raw and final) | rebuild |
| `cargo_alarm_heartbeat_node` | subscriber/protocol validator | rebuild; schema-7 static assert |
| `check_cargo_safety_e2e.py` | source contract | updated to schema 7 |
| `summarize_safety_status.py` | bag/CSV acceptance | validates observable 17/18/29 contracts |
| API/architecture/monitoring docs | external contract | updated to schema 7 |

Ubuntu must regenerate `lidar_slam2_msgs` and rebuild every publisher and
subscriber in the workspace; an old devel artifact is not acceptable.

## Bag result semantics

Each Code17/18/29 result is one of `PASS`, `FAIL`, `NOT_COVERED`, or
`INVALID_TEST`. Absence of a code without its prerequisite same-track distance,
clearance and history trajectory is `NOT_COVERED`, never an automatic failure.
Operational diagnostics distinguish `NOT_EXECUTED`, invalid perception,
executed zero points, track pending and authority blocked.

## Static hard invariants

```text
PERSISTENT_RESTORE_CHANGED=NO
TIME_EPOCH_FIX_OVERWRITTEN_BY_AVOIDANCE=NO
NDT_THRESHOLD_CHANGED=NO
EKF_THRESHOLD_CHANGED=NO
RELOCALIZATION_THRESHOLD_CHANGED=NO
SOURCE_TIME_POLICY_IMPORTED_FROM_B886=NO
WATCHDOG_REDESIGN_IMPORTED=NO
SYSTEM_SERVICE_REDESIGN_IMPORTED=NO
```
