# NDT-SLAM Warehouse

ROS1 / Noetic warehouse crane localization, persistent mapping,
suspended-cargo tracking and collision avoidance.

## Project Status

**Field-validated obstacle-avoidance RC.**

Validated:
- NDT localization and long-term map runtime
- cargo detection and rigid cargo tracking
- degraded positive-warning geometry
- external obstacle tracking
- Code 18 (3–5 m collision risk)
- Code 17 (≤3 m collision risk with authorized history)
- Code 18 → external main controller → S3 voice alarm path

Not yet production-release complete:
- independent S3 gate-off field case (total-gate closed, S3 trigger)
- NDT relocalization final acceptance
- persistent map report layer
- NDT fitness auto-fuse
- remaining P3 diagnostics (torsion HOIST_MISSING, tracker boundary conditions)

This is not a production release. The software is not a safety-certified device;
deployment must retain external emergency stops, limit switches, and site
safety policies.

## Architecture

```
Dual LiDAR
    │
    ▼
pointcloud_merger
    │
    ▼
NDT Localization ───────────────────► odom / map / TF
    │
    ├── Cargo Detection
    │      │
    │      ▼
    │   Cargo Lifecycle (EMPTY → CANDIDATE → LOCKED → LOST_HOLD)
    │      │
    │      ▼
    │   Cargo Geometry
    │      ├── Formal geometry (frozen shape + live pose)
    │      └── Degraded positive-warning geometry (live-only)
    │
    ├── External Obstacle Tracker
    │
    ▼
Cargo Avoidance Fusion
    │
    ├── 14 CLEAR
    ├── 17 NEAR_3M   (≤3 m, vertical clearance < 0.8 m)
    ├── 18 NEAR_5M   (3–5 m, vertical clearance < 0.8 m)
    └── 30–35 FAULT / INVALID
    │
    ▼
CargoSafetyStatus (typed, schema v6)
    │
    ▼
cargo_alarm_heartbeat_node (contract validation + 5 Hz status re-publish)
    │
    ▼
/cargo_avoidance/status_code (std_msgs/Int32)
    │
    ▼
External Main Controller (maintained outside this repository)
    │
    ▼
S3 Voice Alarm
```

## Safety Contract

### Status Codes

| Code | Meaning |
|---:|---|
| 14 | CLEAR — no collision risk; obstacle geometry may be NaN when no obstacle present |
| 17 | NEAR_3M — obstacle ≤3 m from cargo OBB, vertical clearance < 0.8 m |
| 18 | NEAR_5M — obstacle 3–5 m from cargo OBB, vertical clearance < 0.8 m |
| 30 | System not ready, state timeout, or source timeline rollback |
| 31 | Localization invalid |
| 32 | Gravity / load-cell signal invalid |
| 33 | Cargo evidence invalid (geometry, bottom, height, or formal-hold timeout) |
| 34 | Obstacle evidence insufficient or invalid |
| 35 | Configuration, schema, non-finite value, or internal contract error |

### Geometry Authority

The system distinguishes between formal and degraded geometry. This
distinction is a core safety property of the `8d7d7ee` baseline:

**Formal geometry** (frozen shape, authorised by static+live convergence):

| Operation | Allowed |
|---|---|
| Positive 17 / 18 | YES |
| CLEAR 14 (all contracts satisfied) | YES |
| Cargo point removal from registration | YES |
| Static map exclusion | YES |
| MapCommit exclusion | YES |

**Degraded live-only geometry** (positive-warning only, no clear authority):

| Operation | Allowed |
|---|---|
| Positive 17 / 18 | YES |
| CLEAR 14 | **NO** |
| Cargo point removal | **NO** |
| Static map exclusion | **NO** |
| MapCommit exclusion | **NO** |

A stable live-only geometry may freeze after formal track lock, but it
retains `formal_authorized=false`. It only upgrades to formal when static
and live physical sources converge continuously.

### Heartbeat Node

`cargo_alarm_heartbeat_node`:
- Validates `CargoSafetyStatus` typed contract
- Accepts fresh progressing source stamps (new code takes effect immediately)
- Re-publishes current status code at 5 Hz
- Duplicate source stamps do not produce new evidence

Spatial and temporal evidence confirmation occurs upstream in the safety /
obstacle tracking pipeline, not in the heartbeat node.

## Build

Ubuntu / ROS Noetic:

```bash
cd ~/NDT-slam-ws
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin clean -y
catkin build --no-status
source devel/setup.bash
```

Windows is for source editing and static contract checks only; it cannot
replace ROS / PCL / Sophus compilation or bag acceptance.

## Run

Production server (real sensor timestamps, persistent map):

```bash
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=false use_rviz:=false persistent_map:=true
```

Server validation with unified entrypoint:

```bash
rosrun ndt_slam run_server_validation.sh prepare \
  --workspace ~/NDT-slam-ws --expected-sha <SHA> --run-id rc1-live-001
rosrun ndt_slam run_server_validation.sh start \
  --workspace ~/NDT-slam-ws --expected-sha <SHA> --run-id rc1-live-001
```

Bag acceptance (simulation time, non-persistent test map):

```bash
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=true use_rviz:=true persistent_map:=false
rosbag play /path/to/warehouse.bag --clock
```

## Main-controller Integration

The controller application is maintained outside this repository.

NDT-SLAM publishes:

| Topic | Type |
|---|---|
| `/cargo_avoidance/safety_status` | `lidar_slam2_msgs/CargoSafetyStatus` |
| `/cargo_avoidance/status_code` | `std_msgs/Int32` |

The verified external integration maps **Code 18 to S3 voice alarm**.
Controller-side source code and release lifecycle are managed independently.

Code 17 → controller S3 mapping: SLAM-side Code 17 output is field-observed;
controller-side Code 17 behaviour requires separate verification if needed.

## Field Validation

2026-07-27 / 2026-07-28 field verification:

| Item | Count / Value |
|---|---|
| SLAM Code 18 observations | 235 |
| SLAM Code 17 observations | 53 |
| Independent SLAM avoidance episodes | 24 |
| Controller Code 18 receptions | 243 |
| S3 voice sends | 224 |
| S3 rate limit | 2.2 s |
| First observed controller alarm latency | < 1 s |

These are two independent field runs (SLAM 2026-07-27, controller 2026-07-28),
not a single synchronized frame-by-frame log. The S3 independent gate path
(total-gate closed scenario) was not covered by these runs.

Detailed evidence: [docs/validation/obstacle_avoidance_e2e_20260727_20260728.md](docs/validation/obstacle_avoidance_e2e_20260727_20260728.md)

## Documentation

Technical documentation (current master reference):

- [Architecture](src/ndt_slam/doc/architecture.md)
- [Localization Runtime](src/ndt_slam/doc/localization_runtime.md)
- [Cargo Tracking & Safety](src/ndt_slam/doc/cargo_tracking_and_safety.md)
- [Map Lifecycle](src/ndt_slam/doc/map_lifecycle.md)
- [Configuration](src/ndt_slam/doc/configuration.md)
- [Deployment](src/ndt_slam/doc/deployment.md)
- [Operations](src/ndt_slam/doc/operations.md)
- [Server Monitoring](src/ndt_slam/doc/server_monitoring.md)
- [Server Validation Runbook](src/ndt_slam/doc/server_validation_runbook.md)
- [Testing & Acceptance](src/ndt_slam/doc/testing_and_acceptance.md)
- [Troubleshooting](src/ndt_slam/doc/troubleshooting.md)

Historical evidence, design decisions, and incident reports:
[docs/](docs/)

## Static Checks

```bash
git diff --check
python3 scripts/regression/run_static_contracts.py
python3 scripts/regression/check_repository_integrity.py
python3 scripts/regression/check_cargo_safety_e2e.py
python3 -m compileall scripts tests tools
python3 -m unittest discover
```

Ubuntu must also complete: clean build, gtest, stationary-drift bag, real
moving catch-up, cargo lift/translation, 17/18/14 spatial contracts, and
second bag time-epoch rollback test.

## Known Limitations

- Independent S3 gate-off field case pending (total-gate closed scenario)
- Persistent map report layer not yet implemented
- NDT fitness auto-fuse pending
- Relocalization final acceptance pending
- Torsion HOIST_MISSING diagnostics (2 gtest failures in cargo_swing_monitor)
- Obstacle tracker boundary conditions (10 gtest failures, known as of 8d7d7ee)
- Cargo component fusion edge cases (2 gtest failures, known as of 8d7d7ee)
- Controller-side Code 17 → S3 path not yet field-verified
