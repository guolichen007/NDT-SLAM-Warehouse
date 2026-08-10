# Offline static-map certification

These tools are intentionally outside `ndt_slam_node` and never run from a
LiDAR callback.

1. `static_map_rebuilder` verifies critical archive SHA-256 sidecars, performs
   ScanContext + ICP loop discovery, g2o pose-graph optimization, and reprojects
   immutable raw/registration keyframes. Per-cell robust Z histograms reject
   isolated height spikes. Only cells supported by at least three independent
   episodes and two survey passes enter either candidate map.
2. `static_map_certifier.py` verifies coverage, cell support, independent-route
   evidence, source/config/extrinsic identities, and explicit operator approval.
3. `baseline_installer.py` installs the split localization and avoidance maps
   into an immutable UUID directory and atomically updates `CURRENT.json`.

Runtime evidence is always `CANDIDATE_NOT_CERTIFIED`; it cannot bypass step 2.
