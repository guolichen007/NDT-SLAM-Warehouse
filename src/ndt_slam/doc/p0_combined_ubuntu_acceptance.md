# P0 Cargo V6 + Rail Localization V2 — Ubuntu Combined Handoff

This handoff starts only after the desktop branch is pushed and its remote SHA
matches the local `HEAD`. It performs the sole combined four-bag attempt. It
does not activate Cargo product takeover, production maps, or systemd units.

## Frozen inputs

- Parent: `99bfcb049fdc0d0d5592e93fefd75ed0a4d43688`
- Branch: `codex/p0-cargo-v6-rail-localization-v2`
- Candidate: externally substitute the exact `COMBINED_CANDIDATE_SHA`.
- Runtime: Cargo V6 Shadow enabled; yaw mode `RAIL_AUTHORITY`.
- Map: verified schema-v2 snapshot or a fresh Rail-authority sandbox.
- Yaw reference: approved immutable YAML, frozen before replay.
- Bags: exactly one each of `无 / 有 / 长件 / 大件`.

The yaw reference must contain the canonical semantic fields and hash. An old
map without a verified reference is deliberately Rail read-only and is not a
valid writable acceptance map. Do not create a sidecar to “certify” an old
skewed map.

## SHA and worktree gate

```bash
git fetch origin codex/p0-cargo-v6-rail-localization-v2
git switch codex/p0-cargo-v6-rail-localization-v2
git rev-parse HEAD
git rev-parse origin/codex/p0-cargo-v6-rail-localization-v2
git status --porcelain --untracked-files=all
```

Both SHAs must equal the externally supplied candidate and the status output
must be empty.

## One-shot command

```bash
source /opt/ros/noetic/setup.bash

src/ndt_slam/scripts/validation/run_p0_combined_ubuntu_acceptance.sh \
  "$PWD" \
  <COMBINED_CANDIDATE_SHA> \
  /absolute/path/to/verified_or_fresh_map \
  /absolute/path/to/frozen_yaw_reference.yaml \
  /absolute/path/to/output \
  /absolute/path/to/oracles \
  /absolute/path/to/v5_baseline_traces \
  1200 \
  /absolute/path/to/无.bag \
  /absolute/path/to/有.bag \
  /absolute/path/to/长件.bag \
  /absolute/path/to/大件.bag
```

The outer gate runs, in order, before any bag can start:

1. SHA + worktree + runtime-isolation gates (a running production
   `ndt_slam_node` aborts the gate; it is never killed).
2. Input preflight: every bag, oracle, V5 baseline trace, map tree and the yaw
   reference are SHA-256 frozen into `frozen_acceptance_inputs.json` (candidate
   SHA, input hashes, reference hash, map_frame_uuid, ROS/PCL/build env).
3. The deterministic 72-hour authority soak (contract-level, no bag).
4. The nested matrix: `catkin_make clean` (own `CLEAN_RC`), build, full package
   gtests and `catkin_test_results` — each RC gated independently.

Only after every build gate passes does the matrix atomically claim a SHA-level
one-shot ledger (`server_runs/p0_combined_attempts/<sha>.attempt`) with `mkdir`
and then replay the first bag. The same candidate SHA can never be attempted
twice regardless of output directory. `combined_four_bag_attempt.marker` in the
output directory is only a result copy, not the anti-replay authority. A failing
bag does not stop the other three. Development, tuning, or a Yaw OFF comparison
is forbidden after the attempt begins.

The harness copies the immutable reference and records its SHA-256 beside the
frozen input manifest. Service supervision code is present for review only:
exit 75 is restartable, exit 78 is restart-prevented, and this gate never
enables or starts the systemd unit.

## Separate result gates

The final report must keep these decisions independent:

```text
BUILD_GATE=
CARGO_V6_SHADOW_IMPLEMENTATION_GATE=
CARGO_V6_SHADOW_FUNCTION_GATE=
CARGO_V6_SHADOW_RESULT=
REFERENCE_SLID_AFTER_FREEZE=
WRONG_HISTORY_REFERENCE_BORROW=
REACQUISITION_REFERENCE_AUTHORITY_LEAK=

YAW_AUTHORITY_SINGLE_WRITER=
FIXED_YAW_TRANSLATION_VALIDATED=
RAIL_POSE_FITNESS_VALIDATED=
YAW_LOCALIZATION_GATE=

SAFETY_LOCALIZATION_AUTHORITY_GATE=
MIXED_POSE_GENERATION_SAFETY_FRAME_COUNT=

MAP_REFERENCE_GATE=
SEVERE_OBSERVABILITY_MAP_COMMIT_COUNT=
WRONG_REFERENCE_MAP_COMMIT_COUNT=
LEGACY_SKEW_MAP_WRITE_ALLOWED=NO
MAP_OPS_GATE=

RUNTIME_REGRESSION=
PRODUCT_AVOIDANCE_RESULT=
AVOIDANCE_PRODUCT_GATE=
EARLIEST_REMAINING_BLOCKER=
FIELD_READY=NO
```

`YAW_AUTHORITY_SINGLE_WRITER` is a combination of the static writer-contract
GTest pass, the runtime `RAIL_AUTHORITY` mode and a non-empty reference
identity — never the runtime mode string alone. `SEVERE_OBSERVABILITY_MAP_COMMIT_COUNT`
and `WRONG_REFERENCE_MAP_COMMIT_COUNT` are conservative derivations from the
runtime-status snapshot (the per-frame veto counter is not a snapshot field);
their observed value must stay `0`. `NONRECOVERABLE_RESTART_LOOP_COUNT` is a
service-supervision metric and is intentionally out of scope for bag acceptance.

Required performance summaries are P50/P95/MAX for fixed-yaw solver, final
rail-pose fitness and whole-frame time, plus target normal cache build, Rail
graph worker, slow warn/emergency, large gaps, and queue overwrites. Existing
runtime budgets decide regression; this phase defines no new threshold.

## Read-only map skew audit

```bash
python3 src/ndt_slam/scripts/analysis/analyze_map_yaw_consistency.py \
  /absolute/path/to/map_objects_clean.pcd \
  --reference-yaw-deg <approved-map-yaw-deg> \
  --output /absolute/path/to/output/map_yaw_consistency.json
```

`SEGMENTED_SKEW_OR_WARP` requires map rebuild. The audit never edits map data
or reference metadata.

## Hard stop

After the report and remote SHA receipt: no Cargo product takeover, no
Obstacle/G11.2 work, no old-map forced certification, no parameter patch, and
no `systemctl enable/start`. `FIELD_READY` remains `NO`.
