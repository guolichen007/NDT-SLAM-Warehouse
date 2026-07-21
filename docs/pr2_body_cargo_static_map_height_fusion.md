# Cargo static-map height fusion and failure-atomic session loading

## Scope

- Base SHA: `10eba695faee0775ecdbfd806a7813fb38f69fd4`
- Rollback SHA before this follow-up: `dc129a921cdfcbd53b461cbc58da74b653199490`
- Branch: `feature/cargo-static-map-height-fusion-v1`
- Submodule `src/ndt_omp`: `5495fd9214945afcb4b35d5a1da385e405c52bf9`

Latest head SHA should be copied from PR #2 after the evidence commit is pushed.

## Follow-up fixes

- Align GitHub's ROS-container static contracts and preserve the original
  `dubious ownership` failure evidence.
- Add one cross-platform `run_static_contracts.py` entry point.
- Define the Pending cargo center relative to the hook and expand uncertainty
  exactly once in all axes.
- Stage every fallible map-session input before mutation, including static
  install state, height field, keyframe scan contexts, NDT and cloud buffers.
- Make persistent-static suspension a blocking preflight.
- Reuse one staged candidate for `/load_map` and `/load_map_session`.
- Enforce a typed UNVERIFIED authority gate for origin, thickness, official
  static risk and CLEAR.

## Safety invariants

- Pending never authorizes code 14.
- Pending never authorizes formal cargo-point deletion or MapCommit.
- Default Pending positive risk remains provisional with official 30/33.
- Explicit opt-in may only escalate a positive risk to 17/18.
- `cargo_valid` remains false until formal frozen geometry and bottom evidence
  are valid.
- Static danger survives a one-frame live-clear conflict.

## Validation

- `PASS_WINDOWS`: unified static runner.
- `PASS_WINDOWS`: YAML duplicate-key check.
- `PASS_WINDOWS`: repository integrity check (897 tracked files).
- `PASS_WINDOWS`: cargo safety end-to-end static contracts.
- `PASS_WINDOWS`: Python compileall.
- `PASS_WINDOWS`: Python unittest 25/25.
- `PASS_WINDOWS`: `git diff --check`.
- `NOT_RUN_REQUIRES_UBUNTU`: ROS Noetic build and C++ gtest.
- `NOT_RUN_REQUIRES_REAL_BAG`: Bag replay.
- `NOT_RUN_REQUIRES_SERVER`: server and real lifting validation.

## GitHub CI

- Previous failing run: `29806291286`, job `88557420965`.
- Root cause: Git safe-directory ownership rejection inside the ROS container.
- Latest run: pending push; update this section from PR #2 Checks.
