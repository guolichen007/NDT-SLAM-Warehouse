# 588 NDT Relocalization Ubuntu Acceptance

## Scope

This stage adds an asynchronous recovery path without changing the normal
NDT/EKF output contract. It must be tested after the production Cargo safety
branch is built successfully. The Windows implementation does not claim ROS,
PCL or bag runtime acceptance.

## Pull and build

```bash
cd ~/NDT-slam-ws
git fetch origin
git checkout feature/588-production-cargo-safety-e2e
git pull --ff-only origin feature/588-production-cargo-safety-e2e
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_ndt_slam
catkin_test_results --verbose
```

The build must include `ndt_relocalizer.cpp`, `cargo_bottom_fusion.cpp`,
`cargo_safety_evaluator.cpp` and `cargo_alarm_heartbeat_node`. Any missing
symbol or truncated-source error is an immediate failure.

## Runtime observation

In separate terminals:

```bash
rostopic echo /ndt_slam/relocalization_status
rostopic echo /cargo_avoidance/safety_status
rostopic hz /odom
```

Run the warehouse launch and play the standard bag at 0.8x, 1.0x and 1.5x.
During an NDT failure burst the expected state sequence is:

```text
DEGRADED -> SEARCHING_LOCAL -> CONFIRMING -> COOLDOWN -> IDLE
```

If local search keeps failing it must upgrade to `SEARCHING_GLOBAL`. While the
pose is unreliable, Cargo safety must immediately publish `valid=false` and
request alarm code 18; map commit must report `relocalization_guard`. `/odom`
must continue instead of the processing thread waiting for a service call.

## Forced global recovery

```bash
rosservice call /relocalize
```

The service must only queue a global search. It must never publish an identity
pose. A recovered pose is committed only after two independent results agree.

## Acceptance gates

- No identity-pose jump after `/relocalize`.
- No unbounded point-cloud queue and no `/odom` stall longer than two sensor
  periods during search.
- Recovery result age is at most 0.50 s and two correction estimates agree
  within 0.35 m and 5 degrees.
- No persistent-map commit in DEGRADED, SEARCHING, CONFIRMING or cooldown.
- Cargo state is cleared on pose discontinuity and reacquired after recovery;
  alarm output stays fail-safe (18) until fresh valid evidence returns.
- Existing 1.0x NDT timing, step and jump acceptance must be reported together
  with recovery latency and success count. A recovery event does not erase an
  earlier odometry jump from the report.
