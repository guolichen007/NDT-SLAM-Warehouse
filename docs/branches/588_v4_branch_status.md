# 588 V4 Branch Status

## Branch Identity

```
Branch:
fix/588-production-localization-cargo-fusion-v4

Codex baseline:
3f7283d2cb0de8612adff31ac51fdf127effac11

Runtime diagnostic commits:
69977f4 test(runtime): add 1x and 1.5x localization cargo diagnostics
f0c718d fix(diagnostics): move diag code after EKF variables, fix format string
9d15c3f fix(diagnostics): add destructor for CSV flush, safe EKF access

Test date:
2026-07-10

Launch:
warehouse_live_longterm_mapping.launch

Bag:
调运大件.bag

Bag SHA256:
a6805f48ca0cccf231370045808c60ca1c623ac2c6bf2c7b9ec05b804d7df33c

Baseline verdict:
FAIL
```

## Baseline Metrics (Pre-Fix)

```
processed frames: 363
NDT converged: 362/363 (99.7%)
total time P50/P95/P99: 376/422/493 ms
NDT time P50/P95: 11/36 ms
fitness P50/P95: 0.108/7.415
prediction-only: 63 frames (17.4%)
```

## Critical Finding

**NDT converged does not mean localization correct.**

At frame 162→163, the localization target switches from `fallback_local_map`
(fitness ~0.17) to `cropped_localization_target` (fitness ~7.91). The NDT
"converges" to the wrong target, producing a raw step of 0.4254m — far
exceeding the allowed 0.25m maximum. The EKF enters prediction-only mode
repeatedly, and the odom trajectory leaves the physical diagonal path,
drifting persistently.

The RViz trajectory visibly departs from the real diagonal path during this
period and never fully recovers.

The old CONDITIONAL_PASS verdict is superseded by **FAIL**.

## Branch Status Table

| Item | Status |
|------|--------|
| Dual lidar pairing | Verified |
| NDT compute time | Not the primary bottleneck |
| Diagonal localization | **FAIL** |
| cropped_localization_target | **P0 unsafe** |
| prediction-only recovery | **FAIL** |
| Real-time processing ratio | **FAIL** (P50=376ms > 300ms budget) |
| Cargo LOCKED state | State-only continuity, not observation validity |
| 14/17/18 alarm chain | Not activated |
| Absolute localization accuracy | Not verified (no ground truth) |

## Issue Tracking

GitHub Issue: **PENDING**

See: `docs/known_issues/588_cropped_target_diagonal_drift.md`

## Resolution Path

1. Record failure baseline (this commit)
2. Add reproducible test harness
3. Minimum stop-loss: disable unsafe target in production NDT
4. Performance optimization
5. Full 1.0× regression verification
6. Guarded target activation with probation/fallback/quarantine

## GitHub Issue

Status: **PENDING MANUAL CREATION**

Title: `[P0][588] Cropped localization target causes odom loss and persistent drift during diagonal motion`

Body: See `docs/known_issues/588_cropped_target_diagonal_drift.md`

Labels: `bug`, `P0`, `regression`, `localization`, `NDT`

Note: `gh` CLI not available in this environment. Issue must be created
manually via GitHub web interface or from a machine with `gh` installed.
