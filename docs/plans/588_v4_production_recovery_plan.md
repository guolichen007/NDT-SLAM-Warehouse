# 588 V4 production recovery plan

## Decision and immutable baseline

The result at `3f6492fd7930e42150566c25cbc4a4768aaac9c0` is
`FAIL / PARTIAL_MITIGATION`, not a conditional pass.

- The cropped localization target incident is mitigated by keeping the
  unvalidated target in shadow mode (`use_for_ndt: false`).
- The 1.0x runtime baseline is still a P0 failure: 1070 callbacks, 259 dequeued
  frames, 810 overwritten frames, a 24.2% processed ratio and a 75.7% drop
  ratio.
- NDT itself is not the dominant cost. NDT P50/P95/P99 is approximately
  37.28/43.67/44.55 ms while end-to-end P50/P95/P99 is approximately
  407.93/486.05/550.77 ms.
- No later change may be declared PASS without a new full-bag 1.0x run and the
  evidence gates in this document.

The recovery branch is based on the remote `3f6492f` history. It does not
rewrite the experimental V3 tags or the failed V4 evidence.

The implementation branch is
`feature/588-production-cargo-safety-e2e`. Windows review can establish only
source/API/static-contract consistency; the runtime verdict remains `FAIL /
AWAITING_UBUNTU_EVIDENCE` until the mandatory 1.0x replay passes.

Cargo source semantics are explicit in message schema v2:

- `POINTS` is strongly supported physical point evidence.
- `ORIGIN_HEIGHT` is height learned from the first supported POINTS estimate
  of the current track; it is never reported as map evidence.
- `MAP_DIFF` and `MAP_STATIC` are reserved for real map-derived height priors
  and are not synthesized from a historical current-footprint crop.
- `RECENT_STABLE` is a short, age-bounded hold and cannot cross track, stale or
  timestamp-rollback resets.

## Architecture boundaries

The runtime is split into four ownership domains.

1. **Localization thread** owns NDT, EKF, `current_pose_`, output constraints,
   odom/TF/path publication and `local_map_`. No other worker may modify these
   objects.
2. **Map worker** owns expensive keyframe semantic filtering, clean/display
   map maintenance, long-term map writes and shadow-target construction. Map
   admission is decided by the localization thread before enqueueing.
3. **Cargo pipeline** consumes a cloud and the pose carrying the same sensor
   timestamp. Cargo-bottom temporal state is reset on track change, stale time
   or time rollback. Its typed output is the only height evidence accepted by
   the formal safety evaluator.
4. **Alarm heartbeat** is independent of SLAM scheduling. It starts at and
   returns to fail-safe code 18 when evidence is invalid or stale, and emits
   only the PLC contract 14/17/18.

MotionGate is map-admission evidence only. It must never modify EKF velocity,
the localization map, the published pose or Cargo state. Optional ICP is a
map-only aid and performs zero copies/jobs/threads when disabled.

## Ordered implementation gates

| Gate | Required change | Exit evidence |
| --- | --- | --- |
| P0-A | Keep cropped/clean target in shadow mode | target source remains `local_map`; no 3218-point target incident |
| P0-B | Wire truthful callback/processed rates and stage timers | callback dt, processed dt, queue age and non-zero stage columns in CSV |
| P0-C | Remove MotionGate and disabled-ICP side effects | invariant violations = 0; disabled ICP copy/job/thread/use = 0 |
| P0-D | Remove non-localization work from the critical path | 1.0x processed ratio >= 0.90 and queue drop ratio <= 0.10 |
| P0-E | Bound localization and worker queues | finite capacity; explicit overflow/stale counters; no unbounded backlog |
| P1-A | Compensate the two lidar timestamps conservatively | paired-cloud diagnostics identify applied/skipped rigid compensation |
| P1-B | Validate local-map and path geometry | no diagonal drift or discontinuity; published step is measured from actual odom |
| P1-C | Publish fused cargo bottom and map geometry | typed, same-stamp base/map result; uncertainty and source populated |
| P1-D | Evaluate every obstacle cluster independently | 2.9/3.1/4.9/5.1 m and vertical-clearance boundary tests pass |
| P1-E | Continuous fail-safe alarm | startup/stale/invalid = 18; healthy clear = 14; inner/outer = 17/18 |

## Runtime acceptance contract

A full `调运大件.bag` run at 1.0x must satisfy all hard gates:

- `processed / callback >= 0.90` after the configured warm-up window.
- queue drops are `<= 10%`, with no sustained queue-age violation.
- total processing P95 `<= 100 ms`; NDT P95 is reported separately.
- actual published planar step max `<= 0.15 m` and steps above `0.10 m`
  occur no more than eight times in the agreed evaluation window.
- no non-finite odom, no timestamp rollback and no TF discontinuity.
- formal NDT target remains local-map-only until the shadow target separately
  passes point-count, geometry and A/B trajectory tests.
- Cargo messages never reuse height across track IDs or across a stale/time
  rollback boundary.
- missing, stale or invalid Cargo evidence continuously produces alarm 18.

If a required CSV field is absent, the corresponding gate is
`UNVERIFIABLE` and the overall result is FAIL. Processed sensor dt must never
be substituted for callback sensor dt, and EKF-internal step must never be
substituted for the actual published odom step.

## Ubuntu execution order

1. Build the exact pushed commit in a clean catkin workspace.
2. Run unit and static regression tests before launching ROS.
3. Run an isolation profile with long-term commits, Cargo visualization and
   debug serialization disabled; establish the localization-only ceiling.
4. Run the production profile at 1.0x and archive console, diagnostic CSVs,
   parameters, `/odom`, Cargo typed topics and alarm output.
5. Enable each downstream domain in order: map worker, Cargo fusion, safety
   evaluator, alarm heartbeat. Re-run 1.0x after every enablement.
6. Only after 1.0x passes, run the stress profile at 1.5x. A 1.5x result never
   replaces the mandatory 1.0x acceptance result.

Windows can validate source, history, bag metadata and pure regression logic,
but it cannot provide a ROS1/PCL/NDT runtime PASS. Final acceptance therefore
remains pending until the Ubuntu replay artifacts are returned.
