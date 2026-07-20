# 588 V4 Regression Test Baseline

This directory contains the pre-fix failure evidence for issue 588.

## Files

| File | Description |
|------|-------------|
| `baseline_metrics.json` | Numerical summary of the failure |
| `failure_window.csv` | Runtime frames 140-210 showing the catastrophic target switch |
| `cargo_failure_window.csv` | Cargo detection data for the same time window |
| `failure_window_metadata.json` | Timing metadata for reproducing the failure window |
| `baseline_acceptance_corrected.md` | Explanation of why the original verdict was wrong |
| `README.md` | This file |

## Purpose

These files are the **permanent failure baseline** for issue 588. They must:

1. Never be overwritten by subsequent test results
2. Always be used as the "before" comparison in regression tests
3. Cause the analysis script to return FAIL when processed

## Failure Pattern

```
Frame 162: fitness=0.169, target=fallback_local_map
Frame 163: fitness=7.913, target=cropped_localization_target
           → raw_step=0.425 >> allowed=0.25
           → prediction_only=1
           → trajectory leaves diagonal path
```

## Key Metrics

```
processed_ratio: 0.256 (only 25.6% of input frames processed)
fitness P95: 7.415 (catastrophic)
prediction_only: 17.4%
manual_diagonal_drift: true
```

## Related Issue

GitHub Issue: [PENDING]
See: `docs/known_issues/588_cropped_target_diagonal_drift.md`
