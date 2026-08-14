# ClaudeCLI handoff: Stage 2 exact-SHA validation

## Checkout and evidence identity

Windows Codex owns all code changes. Ubuntu ClaudeCLI only checks out and
tests the exact pushed SHA; do not edit or auto-format it.

```bash
git fetch origin codex/integration-stability-avoidance-v1
git checkout --detach origin/codex/integration-stability-avoidance-v1
export INTEGRATION_SHA="$(git rev-parse HEAD)"
git status --short
```

Record `INTEGRATION_SHA` in every result. Use the workspace's already verified
build system. If it uses `catkin_make`, do not switch to `catkin build`. Do not
delete `build` or `devel`.

## Message ABI and build

Schema 6 -> 7 requires regeneration of `lidar_slam2_msgs` and rebuild of all
publishers/subscribers, including `ndt_slam` and
`cargo_alarm_heartbeat_node`. Confirm that no MD5/schema mismatch appears.

Required build result:

```text
MESSAGE_GENERATION=PASS
ALL_CONSUMERS_REBUILT=PASS
BUILD=PASS
MD5_SCHEMA_MISMATCH=NO
```

## Targeted tests

Run at minimum:

```text
time_epoch_contract_test
keyframe_manager_time_epoch_test
persistent_registration_loader_test
cargo_geometry_fusion_test
cargo_oriented_footprint_test
cargo_capability_test
cargo_subsystem_test
cargo_frame_decision_test
cargo_safety_evaluator_test
cargo_safety_temporal_filter_test
cargo_obstacle_tracker_test
cargo_avoidance_fusion_test
pending_static_hazard_tracker_test
hazard_evaluator_test
anomaly_review_episode_tracker_test
```

Then run the repository Python contracts and regression scripts. Targeted
Cargo/Obstacle GTest failures must be zero.

## Runtime sequence

1. Production YAML startup and one normal bag.
2. Three-bag source-timestamp rollback stress.
3. Thirty-minute combined mapping/Cargo/Obstacle stress.
4. Clean shutdown/restart and persistent registration restore.
5. Only after these pass, continue hours/days testing.

SLAM evidence must independently show callback, NDT attempt, accepted
localization, keyframe (when normal motion criteria are met), MapCommit, tile
flush and runtime-status sequence progress.

At every TIME_EPOCH_RESET, also prove:

```text
Cargo transient identity reacquired
PhysicalObstacleTrack identity reset
far_history_valid=false before new evidence
17/18 authority re-accumulated in the new epoch
NDT/keyframe/MapCommit/TileFlush recovery independent of Cargo readiness
```

## Coverage-safe avoidance verdict

For each code report:

```text
CODE17_RESULT=PASS|FAIL|NOT_COVERED|INVALID_TEST
CODE18_RESULT=PASS|FAIL|NOT_COVERED|INVALID_TEST
CODE29_RESULT=PASS|FAIL|NOT_COVERED|INVALID_TEST
REASON=<scene prerequisite and evidence>
```

Do not fail a code merely because the bag did not contain the same-track
5-8 m -> 3-5 m -> <=3 m trajectory with clearance below 0.8 m. Use
`scripts/acceptance/summarize_safety_status.py --validate-avoidance-first`
for observable CSV invariants; add `--require-code N` only when the bag is
known to cover that code.

## Required return block

```text
INTEGRATION_SHA=
WORKTREE_CLEAN=
MESSAGE_GENERATION=
ALL_CONSUMERS_REBUILT=
BUILD=
TARGETED_GTEST=
PYTHON_CONTRACTS=
NORMAL_SINGLE_BAG=
THREE_BAG_ROLLBACK=
TIME_EPOCH_MAPPING_RECOVERY=
AVOIDANCE_EPOCH_RESET=
EPOCH_RESET_THEN_RESTART=
COMBINED_30_MIN=
SLAM_GATE=
CODE17_RESULT=
CODE18_RESULT=
CODE29_RESULT=
AVOIDANCE_GATE=
MD5_SCHEMA_MISMATCH=
NEW_STATIC_FAIL=
NOT_COVERED=
```

Known inherited Stage 1 non-blocking coverage gaps remain: deterministic
runtime MapCommit race injection and hours/days memory stability. Report them
as `NOT_COVERED`; do not reinterpret them as Stage 2 failures.
