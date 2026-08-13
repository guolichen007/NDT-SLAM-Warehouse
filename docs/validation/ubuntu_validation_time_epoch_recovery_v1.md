# Ubuntu ClaudeCLI handoff: TIME_EPOCH recovery v1

## Scope and stop rule

Test only the exact `TIME_EPOCH_FIX_SHA` supplied by Windows Codex from branch
`codex/slam-time-epoch-recovery-v1`. Do not merge or test Stage 2 avoidance
integration. Stage 2 remains blocked until every gate below passes and the
result contains:

```text
TIME_EPOCH_FIX_RUNTIME_CONFIRMED=YES
READY_FOR_STAGE2=YES
```

Do not change NDT, EKF, relocalization, Cargo or obstacle thresholds. Use the
workspace's already verified build system. If it uses `catkin_make`, do not
switch to `catkin build`; do not remove `build/` or `devel/`.

## Time-domain matrix

| State / action | Domain | Rollback behavior |
|---|---|---|
| Outer keyframe time gate | LiDAR source | Rebase stamp; preserve pose |
| KeyFrameManager time gate | LiDAR source | Rebase time only |
| NDT/EKF/Cargo input history | LiDAR source | Existing narrow reset |
| ROS bag/status timestamps | ROS sim | Metadata only |
| Runtime status and health log cadence | steady/wall | No source-time authority |
| Dirty-tile periodic flush cadence | steady | No source-time authority |
| Pointcloud watchdog | wall | No source/sim rollback effect |
| `last_flush_time` status value | ROS sim | Metadata only |

## Lock-order and MapCommit contract

```text
LOCK_ORDER_AUDIT=
process owner already holds runtime_state_mutex_
→ handleLidarTimeRollback acquires map_commit_lifecycle_mutex_
→ map/keyframe/cargo ownership locks are acquired in separate sections
→ lifecycle is released
→ queue/completion locks are acquired separately
```

The worker releases its queue lock before waiting for the lifecycle lock. The
rollback handler never owns queue/completion locks while waiting for lifecycle.
Tile flush never acquires lifecycle/runtime locks.

```text
MAP_COMMIT_EPOCH_CONTRACT=
old queued job: counted stale and removed
old dequeued/pre-write job: rejected by lifecycle fence
old physical write: map/dirty content remains valid
old completion: cannot update new temporal gate/reference/authority
dirty/failed batches: never cleared by epoch reset; normal retry is scheduled
```

## Checkout and Windows-independent checks

```bash
git fetch origin codex/slam-time-epoch-recovery-v1
git checkout --detach "$TIME_EPOCH_FIX_SHA"
test "$(git rev-parse HEAD)" = "$TIME_EPOCH_FIX_SHA"
python3 scripts/regression/run_static_contracts.py
```

Record the pre-existing workspace state and build command before building.
With the historically verified `catkin_make` workflow, run from the workspace
root without cleaning:

```bash
catkin_make -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_ndt_slam_gtest_time_epoch_contract_test \
  run_tests_ndt_slam_gtest_keyframe_manager_time_epoch_test
catkin_test_results --verbose
```

Required build result:

```text
BUILD=PASS
TARGETED_GTEST=PASS
```

## Normal bag and real rollback runtime

Use the existing production YAML and launch sequence. First replay one bag
without a source-time rollback and prove no new behavior:

```text
NORMAL_SINGLE_BAG=PASS
time_epoch_reset_count delta=0
cloud_callback_count grows
ndt_attempt_count grows
accepted_localization_count grows when registration is accepted
runtime_status_seq and runtime_status.json mtime grow
```

Then use the real three-bag boundary or another real replay boundary containing
approximately:

```text
1779155914 → 1778217251
```

Capture `/runtime_status.json`, node logs and tile/keyframe artifacts before
and after the boundary. For one reset, require the same epoch value on:

```text
[TimeEpoch] RESET_BEGIN
[TimeEpoch] MAP_STATE_PRESERVED
[TimeEpoch] MAPPING_TEMPORAL_STATE_RESET
[TimeEpoch] FIRST_NDT_AFTER_RESET
[TimeEpoch] FIRST_ACCEPT_AFTER_RESET
[TimeEpoch] MAP_COMMIT_REARM_AFTER_RESET
[TimeEpoch] FIRST_KEYFRAME_AFTER_RESET
[TimeEpoch] FIRST_TILE_FLUSH_AFTER_RESET
```

The reset frame may use fallback `sensor_dt=0.10`; this is expected. A static
device does not have to emit a keyframe immediately. After normal motion/time
eligibility is present, require all independent liveness evidence:

```text
cloud_callback_count grows
ndt_attempt_count grows
accepted_localization_count grows
keyframe_count grows
map_commit_completed_count grows
tile_flush_completed_count grows
runtime_status_seq and mtime keep growing
```

For the reset-storm check, replay/observe:

```text
1779155914
→ 1778217251
→ 1778217251.1
→ 1778217251.2
```

Require `time_epoch_reset_count` to increase by exactly one, not three.

## Epoch reset followed by restart

After the recovered pipeline creates and flushes new data:

1. Stop the node cleanly and record the new persistent tile/manifest state.
2. Restart the exact same SHA and production YAML.
3. Require persistent registration restore to include the new durable data.
4. Require registration target readiness and normal NDT attempts/acceptance.

Report:

```text
REAL_TIME_ROLLBACK=PASS|FAIL
MAPPING_RECOVERY=PASS|FAIL
NO_OLD_EPOCH_TEMPORAL_STATE_REINTRODUCED=YES|NO
NO_DIRTY_TILE_SILENT_LOSS=YES|NO
NO_DEADLOCK=YES|NO
NO_DUPLICATE_KEYFRAME_AUTHORITY=YES|NO
EPOCH_RESET_THEN_RESTART=PASS|FAIL
TIME_EPOCH_FIX_RUNTIME_CONFIRMED=YES|NO
READY_FOR_STAGE2=YES|NO
```

Any `FAIL` or `NO` blocks Stage 2. Return logs, the exact SHA, counter samples,
runtime status snapshots, and the first failed gate to Windows Codex; do not
work around a failure by tuning thresholds.
