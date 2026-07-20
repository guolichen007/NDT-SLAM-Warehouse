# P0: Cropped localization target causes odom loss and persistent drift during diagonal motion

## 1. Problem Summary

When the localization target switches from `fallback_local_map` to
`cropped_localization_target`, fitness jumps from ~0.05–0.17 to ~4–9.
Raw NDT correction exceeds the step limit, EKF enters prediction-only
mode repeatedly, and the odom trajectory departs from the real diagonal
path with persistent drift.

## 2. Reproducible Environment

- OS: Ubuntu 20.04 (Linux 5.15.0-139-generic)
- ROS: Noetic
- Repository: `git@github.com:guolichen007/NDT-SLAM-Warehouse.git`
- Branch: `fix/588-production-localization-cargo-fusion-v4`

## 3. Pre-Fix Code Identity

```
Codex base SHA: 3f7283d2cb0de8612adff31ac51fdf127effac11
Diagnostic SHA: 9d15c3f
```

## 4. Input Bag Identity

```
Path: /home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag
SHA256: a6805f48ca0cccf231370045808c60ca1c623ac2c6bf2c7b9ec05b804d7df33c
Duration: 135 seconds
/rs_201 messages: 1352
/rs_203 messages: 1353
```

## 5. Run Command

```bash
roscore &
rosparam set /use_sim_time true
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_rviz:=false use_cargo_visualizer:=false &
rosbag play /home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag --clock -r 1.0
```

## 6. RViz Observation

When observing with RViz at 1.0× playback:
- Frames 1–162: trajectory follows the physical diagonal path
- Frame 163: sudden trajectory jump; odom leaves the diagonal
- Frames 163–188: persistent drift; trajectory no longer tracks the crane
- Frames 189+: intermittent recovery but trajectory remains offset

## 7. Failure Timeline

```
Frame 140-162: fallback_local_map, fitness 0.04-0.17, no prediction-only
Frame 162:    fitness=0.169, raw_step=0.008, target=fallback_local_map
Frame 163:    fitness=7.913, raw_step=0.425, target=cropped_localization_target
              → SUDDEN catastrophic target switch
              → raw_step 0.425 >> allowed 0.25
              → prediction_only=1, EKF rejects measurement
Frame 164-170: fitness 7.0-7.9, raw_step 0.27-0.73, alternating prediction-only
Frame 171-188: fitness 6.6-7.6, raw_step 0.05-0.59, persistent drift
Frame 189:    back to fallback_local_map, fitness=0.054
              → intermittent recovery begins
Frame 192+:   cropped target returns, fitness 5.5-6.0, drift continues
```

## 8. Preliminary Root Cause

1. **Target readiness is determined primarily by point count.** When
   `cropped_localization_target` reaches 3000 points, it is marked "ready"
   and bound to NDT — without checking spatial coverage, degeneracy, or
   source/target compatibility.

2. **No shadow/probation period.** The new target is immediately used for
   production NDT alignment, with no gradual validation.

3. **No same-frame local-map fallback.** When the new target produces
   catastrophic results (fitness > 1.0, step > limit), there is no
   mechanism to fall back to the known-good local map within the same frame.

4. **Bad target results can enter EKF.** Even when the target produces
   clearly wrong corrections, the EKF may accept or partially accept them.

5. **Prediction-only continues from affected state.** When EKF enters
   prediction-only, it continues predicting from the already-corrupted
   state, compounding the drift.

6. **Localization thread processes at reduced rate.** The "keep latest only"
   queue policy drops frames when processing is slow, reducing the effective
   localization frequency below the input rate.

## 9. Secondary Performance Issue

Processing time P50 = 376ms exceeds the ~300ms frame budget at 10Hz input.
This is not the primary cause of the diagonal drift (the target switch
causes it), but it contributes to frame drops and reduced localization
quality.

## 10. Regression Prohibition

The following must NEVER happen again in production:

1. `cropped_localization_target` bound to NDT without geometric validation
2. Target switch without same-frame fallback to local map
3. Fitness > 1.0 results entering EKF without quarantine
4. Raw step > allowed step accepted as valid measurement
5. Prediction-only frames continuing from corrupted state without recovery

## 11. Current Mitigation

Production configuration temporarily disables unvalidated localization
target from participating in formal NDT:

```yaml
localization_target:
  build_enabled: true      # continue building target for diagnostics
  use_for_ndt: false       # DO NOT bind to NDT setInputTarget
  shadow_diagnostics_enabled: true
```

Target may continue to be built, cropped, voxel-filtered, and quality-
monitored in shadow mode, but must NOT be bound to `ndt_->setInputTarget()`.

## 12. Full Closure Conditions

All of the following must be satisfied before this issue can be closed:

1. Target has geometric quality checks (spatial coverage, degeneracy)
2. Target passes continuous probation period
3. Candidate target is compared against local map in same frame
4. Catastrophic anomaly triggers immediate same-frame fallback
5. Bad target is quarantined (version blacklisted)
6. Bad target results never enter EKF
7. Bad target results never enter MapCommit
8. Full 1.0× bag test shows no diagonal drift
9. Processing rate meets acceptance standard
10. Regression test automatically prevents recurrence

## 13. Related Commits

```
3f7283d feat(cargo-safety): add bottom fusion header, safety evaluator, and alarm heartbeat
69977f4 test(runtime): add 1x and 1.5x localization cargo diagnostics
f0c718d fix(diagnostics): move diag code after EKF variables, fix format string
9d15c3f fix(diagnostics): add destructor for CSV flush, safe EKF access
```

## 14. Test Report Location

```
tests/regression/588_v4/baseline_metrics.json
tests/regression/588_v4/failure_window.csv
tests/regression/588_v4/cargo_failure_window.csv
tests/regression/588_v4/baseline_acceptance_corrected.md
```
