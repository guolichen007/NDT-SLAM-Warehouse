# Obstacle Avoidance End-to-End Field Verification

2026-07-27 / 2026-07-28

## Scope

Field verification of cargo obstacle avoidance pipeline: SLAM-side detection
and code output, controller-side code reception, and S3 voice alarm triggering.

## Validated SHA

`8d7d7eed0548321bf0646232f374fe95a29990dd`

Tag: `validation-obstacle-avoidance-20260728`

## SLAM-side Evidence (2026-07-27)

Independent SLAM field run:

| Metric | Value |
|---|---|
| Code 18 (NEAR_5M) observations | 235 |
| Code 17 (NEAR_3M) observations | 53 |
| Independent avoidance episodes | 24 |

Cargo detection, degraded geometry, obstacle tracking, and avoidance
fusion pipeline produced expected positive warning codes.

## Controller-side Evidence (2026-07-28)

Independent main-controller integration run:

| Metric | Value |
|---|---|
| Code 18 received | 243 |
| S3 voice sends | 224 |
| S3 rate limit (inter-alarm gap) | 2.2 s |
| First observed callback-to-send latency | < 1 s |

The controller correctly mapped Code 18 reception to S3 voice alarm
output, with throttle protection.

## Caveats

1. **Separate runs.** SLAM logs (2026-07-27) and controller logs
   (2026-07-28) are from different sessions. They are not a single
   synchronized frame-by-frame end-to-end run. The counts (235 SLAM 18s,
   243 controller 18s) should not be read as a 1:1 correspondence.

2. **Code 17 → S3 not verified.** Controller-side Code 17 behaviour was
   not independently confirmed in this batch. Only Code 18 → S3 was
   observed.

3. **S3 independent gate path not covered.** The total-gate closed
   scenario (where the original main gate is closed and S3 must trigger
   independently) was not exercised in these runs. The code path exists
   but lacks field evidence.

## Not a Production Release

This verification confirms that the obstacle avoidance pipeline produces
expected outputs under field conditions. It does not constitute a formal
production release. Known gaps remain — see project README and CHANGELOG
for the full list.
