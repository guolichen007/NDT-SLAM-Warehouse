# Rail-crane yaw authority: Phase 1 SHADOW gate

This revision is deliberately incapable of applying CONFIG yaw or the rail
translation proposal to the runtime pose. `apply_to_runtime_pose: true` is
rejected with `PRODUCT_MODE_NOT_IMPLEMENTED_IN_SHADOW_BUILD`.

## Site inputs required before replay

Create an operator-reviewed record with
`scripts/analysis/record_site_yaw_reference.py`. Copy the approved yaw,
convention ID and description into a run-specific copy of
`config/crane_yaw_authority.yaml`; do not edit a production config from the
tool. With either value absent, the node reports `UNCONFIGURED` and the rail
refiner does not run.

## Ubuntu validation

On the exact `SHADOW_SHA`, perform a clean/appropriate ROS Noetic catkin
build and full gtest run. Replay normal SLAM bags, long/large/steel cargo
bags, `有.bag`, and both `incident_bad_state` bags. Preserve:

- `yaw_authority_shadow.csv`;
- Monitor `runtime_samples.csv`, `mapping_samples.csv` and final summary;
- PointCloudMerger diagnostics including rs201, rs203 and merged stamps;
- raw obstacle and base-frame cargo geometry evidence.

Use `scripts/analysis/analyze_yaw_authority_shadow.py` for ALL_TIME,
CRANE_STATIONARY and CRANE_MOVING distributions. The script intentionally
reports `RAIL_CONSTRAINED_XY_VALID=UNVERIFIED`; an Ubuntu reviewer must decide
YES or NO from approved criteria. Use
`scripts/analysis/analyze_dual_lidar_incident.py` for pair-dt, residual and
ghosting evidence; without approved thresholds it reports
`DUAL_LIDAR_INPUT_HEALTH=UNVERIFIED`.

The gate report must contain:

```text
RAIL_CONSTRAINED_XY_VALID=YES|NO
DUAL_LIDAR_INPUT_HEALTH=...
SITE_YAW_CONFIG_STATUS=...
MAP_FRAME_CONVENTION_STATUS=...
```

Phase 2 remains blocked unless rail XY is valid, dual-LiDAR input is
acceptable, and the site yaw/map convention are approved. A failure does not
authorize tuning `sanitizeDt()`, Cargo identity/lifecycle, obstacle
fragmentation, OBB logic, or existing safety thresholds.
