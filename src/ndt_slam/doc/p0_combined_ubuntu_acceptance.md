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

The harness creates `combined_four_bag_attempt.marker` before any replay and
refuses a second attempt in the same output directory. A failing bag does not
stop the other three. Development, tuning, or a Yaw OFF comparison is forbidden
after the attempt begins.

The harness copies the immutable reference and records its SHA-256 beside the
attempt marker. Service supervision code is present for review only: exit 75 is
restartable, exit 78 is restart-prevented, and this gate never enables or starts
the systemd unit.

The nested matrix performs a clean package build, full package gtests and
`catkin_test_results`, then runs all four bags. The outer gate additionally
runs the deterministic 72-hour authority soak and records the frozen reference
configuration used by every sandbox run.

## Separate result gates

The final report must keep these decisions independent:

```text
CARGO_V6_SHADOW_IMPLEMENTATION_GATE=
CARGO_V6_SHADOW_FUNCTION_GATE=
CARGO_V6_SHADOW_RESULT=
PRODUCT_AVOIDANCE_RESULT=

YAW_AUTHORITY_SINGLE_WRITER=
FIXED_YAW_TRANSLATION_VALIDATED=
RAIL_POSE_FITNESS_VALIDATED=
YAW_LOCALIZATION_GATE=

SAFETY_LOCALIZATION_AUTHORITY_GATE=
SAFETY_AVOIDANCE_GATE=
MIXED_POSE_GENERATION_SAFETY_FRAME_COUNT=

MAP_REFERENCE_GATE=
MAP_OPS_GATE=
LEGACY_SKEW_MAP_WRITE_ALLOWED=NO
NONRECOVERABLE_RESTART_LOOP_COUNT=0

RUNTIME_REGRESSION=
COMBINED_FOUR_BAG_ATTEMPT_COUNT=1
AVOIDANCE_PRODUCT_GATE=
EARLIEST_REMAINING_BLOCKER=
FIELD_READY=NO
```

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
