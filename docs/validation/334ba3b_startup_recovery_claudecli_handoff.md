# Crane SLAM startup recovery V1 — Ubuntu/Claude CLI handoff

## Immutable scope

- Input SHA: `334ba3b55d2de04e7426c505fcfaec6e05f3f8bf`
- Branch: `feature/334ba3b-crane-slam-startup-recovery-v1`
- Do not push until every required gate below passes.
- Never force-push.
- Do not change Cargo association/geometry/Z, 14/17/18/29, far history,
  `AuthoritativeHazard`, `CraneMotionEKF`, runtime yaw, normal NDT parameters,
  registration source/target, `MotionGate`, or simultaneous X/Y motion.
- This V1 is startup/crash recovery only. Do not add runtime NDT-loss recovery,
  failure-triggered restarts, operator/stationary waits, manual recovery
  services, PLC/Modbus/motor control, or avoidance changes.

Windows-side evidence already collected:

- `python -m unittest discover -s tests -p "test_*.py"`: 128 tests PASS
  before handoff.
- `python scripts/regression/check_repository_integrity.py`: PASS.
- `git diff --check`: PASS.
- Ubuntu build, gtest, ROS bag, systemd and fault injection: **NOT RUN**.

## 1. Preflight and exact-source audit

Run from the repository root:

```bash
set -euo pipefail
test "$(git branch --show-current)" = \
  feature/334ba3b-crane-slam-startup-recovery-v1
git merge-base --is-ancestor 334ba3b55d2de04e7426c505fcfaec6e05f3f8bf HEAD
git status --short --branch
git diff --check
python3 -m unittest discover -s tests -p 'test_*.py'
python3 scripts/regression/check_repository_integrity.py
```

Confirm no remote branch has been created:

```bash
git ls-remote --heads origin \
  feature/334ba3b-crane-slam-startup-recovery-v1
```

Expected output is empty. Do not push.

## 2. Clean Ubuntu/ROS build

Use the deployment's supported Ubuntu/ROS environment (normally Ubuntu 20.04
and ROS Noetic). Cleaning must be restricted to this workspace's `build` and
`devel` directories.

```bash
set -euo pipefail
source /opt/ros/noetic/setup.bash
repo_root="$(pwd -P)"
test -f "$repo_root/src/ndt_slam/package.xml"
rm -rf -- "$repo_root/build" "$repo_root/devel"
catkin_make -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests
catkin_test_results --verbose
```

Required result: clean build PASS, all gtests PASS, zero failed test cases.
Pay particular attention to:

- `durable_map_store_test`
- `recovery_checkpoint_test`
- `accepted_keyframe_journal_test`
- `crane_place_descriptor_test`
- `crane_startup_relocalizer_test`
- `localization_health_policy_test`
- `map_write_rearm_policy_test`
- `startup_recovery_controller_test`
- `map_write_authority_test`

If compilation fails, fix only the recovery modules/integration. Re-run the
whole clean build after targeted fixes.

## 3. Test-data safety boundary

Never run destructive corruption tests against production map data. Create a
dedicated root whose resolved path ends in `startup-recovery-fixture`:

```bash
export NDT_RECOVERY_TEST_ROOT=/var/tmp/ndt-startup-recovery-fixture
test "$(basename "$(realpath -m "$NDT_RECOVERY_TEST_ROOT")")" = \
  ndt-startup-recovery-fixture
mkdir -p "$NDT_RECOVERY_TEST_ROOT"
```

Render/install the service with that data root. Confirm:

```bash
systemctl cat ndt-slam.service
```

The effective unit must contain:

```text
Restart=on-failure
RestartSec=5
use_ndt_recovery_watchdog:=false
/usr/bin/flock --no-fork --exclusive --nonblock
```

## 4. Fixed-bag startup recovery matrix

Set `RECOVERY_BAG` to the approved fixed bag and record its SHA256. Capture
`/localization/health` (or the configured localization health topic), odom,
diagnostics, CPU and RSS for every run. The health JSON must expose:

- `startup_recovery_state`
- `startup_recovery_verified`
- `map_write_rearmed`
- `recovery_map_write_attempt_count`
- `recovery_map_write_authorized_count`

For every recovery case, the authorized counter must remain exactly zero until
`ACTIVE`; `objects_clean` and static-evidence revisions must remain unchanged.

### A. Kill `ndt_slam_node`

1. Run the fixed bag for at least 20 seconds in normal state.
2. `kill -9` only `ndt_slam_node`.
3. Verify systemd restarts within the configured delay.
4. Required sequence:
   `BOOT -> LOAD_REFERENCE -> SENSOR_WARMUP -> LOCAL_RECOVERY` (when checkpoint
   is valid) `-> VERIFYING -> READONLY_STABILIZING -> ACTIVE`.
5. Verify no trusted map write during recovery and normal processing resumes.

### B. Kill `roslaunch`

Repeat A while killing the service's `roslaunch` parent. Verify the same
crash-only restart and recovery invariants.

### C. Kill during snapshot publication

Trigger a new persistent snapshot and wait until `maps/staging` contains the
new transaction, then kill the node. After restart, either the previous
`CURRENT` loads or verified fallback selects `PREVIOUS`; a partial generation
must never become current.

### D. Partial staging directory

With the service stopped, create an incomplete directory under `maps/staging`.
Restart. It must be ignored and must not affect the selected generation.

### E. Corrupt one CURRENT PCD

Stop the service, resolve `maps/CURRENT`, back up one PCD inside that generation,
append corruption, and start the service. It must reject CURRENT by hash and
load PREVIOUS. Restore the fixture from its clean copy after recording evidence.

### F. Corrupt CURRENT manifest

Repeat E for `manifest.yaml`; automatic PREVIOUS fallback is required.

Also run an all-generations-broken case. It must publish a relative
`ACTIVE_ROOT` pointer, preserve the damaged tree in place, create a fresh
`isolated/<new-map-uuid>` storage root, and use that same isolated root after a
second process restart. The runtime monitor must follow the bounded pointer;
it must not read stale status or map evidence from the quarantined root.

### G. Valid checkpoint

With matching map UUID/generation, verify local deterministic checkpoint search
is attempted before global retrieval and succeeds without direct checkpoint-to-
AcceptedPose promotion.

Also force the local search to exceed its global threshold once. Verify the
controller reports local failure and transitions `LOCAL_RECOVERY ->
GLOBAL_RECOVERY`; the global job must not be silently mixed into the local
stage.

### H. Wrong checkpoint identity

Change checkpoint map UUID or generation in the isolated fixture. It must be
discarded, local recovery must not accept it, and global place retrieval must
recover against the verified map.

For G/H, create a durable snapshot, then append one authorized journal
keyframe outside the snapshot's spatial coverage. After restart, verify the
identity-matched/checksum-verified record extends only the read-only recovery
reference (`durable_plus_verified_journal`). It must not mutate the durable
generation, `objects_clean`, or static evidence. A wrong-identity or partial
journal tail must be ignored/truncated and must not enter the reference.

### I. Ambiguous global candidates

Use the prepared repeated-structure fixture/bag with two near-equal candidates.
Verify `AMBIGUOUS`, insufficient Top1/Top2 margin, no pose acceptance, and
continued frame collection until ambiguity clears.

### J/K. Pollution and rearm

Across A-I verify:

```text
recovery_map_write_authorized_count = 0
objects_clean mutations during recovery = 0
static evidence mutations during recovery = 0
```

After localization verification, require at least 20 accepted frames and at
least 2.5 seconds in `READONLY_STABILIZING`, with no active relocalization job
and unchanged map UUID/generation/continuity, before `ACTIVE`.

### L. Volatile-state reset

Seed Cargo tracks, obstacle tracks, far history and a 29 episode before the
kill. After restart, confirm they are empty and rebuild only from new evidence.

## 5. Normal-runtime non-regression

Run the same no-restart control bag twice: once at exact `334ba3b`, once on the
candidate branch. Prefer a separate temporary git worktree for the base build.
Compare synchronized outputs and resource traces for:

- NDT fitness
- odom/pose
- processing lag
- CPU and RSS
- Cargo output
- 14/17/18/29

Normal healthy-path behavior must match `334ba3b` within the repository's
existing tolerances. Recovery-inactive CPU/RSS overhead must be small and
bounded. Do not waive a Cargo or safety-code mismatch.

## 6. Required result artifact

Create `docs/validation/startup_recovery_v1_ubuntu_results.md` containing:

- INPUT_SHA, candidate OUTPUT_SHA, branch and `PUSHED: false`
- exact local commit list
- toolchain/Ubuntu/ROS versions
- clean build and every gtest result
- fixed bag path and SHA256
- normal regression metrics
- CURRENT/PREVIOUS/PREVIOUS_2 fallback results
- crash-during-save and partial-staging results
- checkpoint local/global fallback results
- ambiguity result and score margin
- verification frame counts and rearm duration
- recovery write attempt/authorized counters
- every A-L fault-injection result
- explicit `NOT_RUN` entries (never convert missing evidence to PASS)

## 7. Final gate and single push

Only after build, all tests, A-L, normal regression and pollution invariants
pass:

```bash
git diff --check
git status --short
```

The working tree must be clean after committing the Ubuntu evidence/fixes. Then
perform exactly one normal push:

```bash
git push -u origin \
  feature/334ba3b-crane-slam-startup-recovery-v1
```

No intermediate push and no force push.
