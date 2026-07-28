# CHANGELOG

## [Unreleased] - Field-validated obstacle avoidance RC

### Added

- Cargo geometry fusion: formal frozen-shape vs. degraded live-only geometry
  with distinct safety authority (formal can clear; degraded can only warn).
- Pending cargo envelope: positive-warning temporary geometry for newly
  appearing obstacles before formal confirmation.
- External obstacle tracker with near-field / far-field history,
  independent static provenance, and embedded/separated track distinction.
- `CargoSwingMonitor`: skew-pull detection, sway oscillation tracking,
  hoist anchor authority, rope-length measurement integration.
- `CargoLiftOriginBinder`: lift origin tracking and frozen-thickness
  formal cargo height.
- `StaticHeightField` and `StaticHeightComponentExtractor`: uncertainty-aware
  static height from map evidence.
- `MapSessionSnapshot`: coherent map session save/load with manifest.
- `StaticEvidenceAuthorization`: immutable bounded query snapshots,
  clean-map provenance, and versioned sidecar manifests.
- `RevealedSupportObserver`: cargo bottom fusion from direct top measurement
  and frozen thickness.
- `CargoPhysicalMotionEstimator`: physical-speed-bounded cargo center update.
- `CargoPresenceStateMachine`: EMPTY→CANDIDATE→LOCKED→LOST_HOLD lifecycle
  with short formal-hold timeout and fault-closed 33.
- `CargoSafetyEvaluator`: 14/17/18/30–35 typed safety contract with
  formal-clear authority gating.
- `cargo_alarm_heartbeat_node`: typed `CargoSafetyStatus` contract validation,
  5 Hz status re-publish, duplicate-stamp rejection.
- Server monitor v2 with cargo telemetry integration and 60/600 s safety
  windows.
- Comprehensive gtest suites: cargo avoidance, geometry fusion, swing monitor,
  pending envelope, obstacle tracker, static evidence authorization,
  static height field, lift origin binder, physical motion estimator,
  presence state machine, map session snapshot, revealed support observer.
- Python test contracts: map load transaction, server monitor, safety
  aggregator, cargo geometry classification, typed callback time validation.

### Changed

- Cargo safety is now an explicit typed contract with formal authority
  separation (formal vs. degraded geometry).
- Obstacle tracking requires independent static provenance before issuing
  Level 2 warnings; near-field track missing far history is not sufficient.
- Pending envelope requires consecutive stable evidence before warning.
- Static map authorization uses current-query coverage, cell count, height,
  source validation, and observation history.
- Lifecycle changes make previous manifest inactive and retain it as
  last-good archive until mature current-epoch snapshot is atomically
  committed.
- Old monitor/deploy scripts are compatibility wrappers around
  `server_monitorctl.sh`.
- Server validation is SHA-gated with explicit preflight, manifest, and
  PASS/FAIL/NOT_RUN preservation.
- CI enforces static contracts (yaml, repository integrity, cargo safety
  e2e, compileall, unittest, git diff) on every push.

### Fixed

- Degraded geometry can now safely produce positive 17/18 warnings without
  being blocked by a previous formal-authority requirement (the core fix
  of `8d7d7ee`).
- Motion safety hold gaps closed: remaining edge cases where conservative
  evidence could be dropped.
- Envelope authority separated from motion safety; persistent envelope
  and skew authority covered by tests.
- Loaded cargo envelope and motion safety hardened.
- Pending cargo envelope growth bounded.
- Cargo monitoring gated under gravity evidence conflict.
- Normal lifecycle no longer produces spurious Code 35.
- False-positive Code 17 from pending cargo growth addressed.

### Safety

- Code 17 / 18 only represent real spatial collision risk.
- Localization, gravity, cargo height, obstacle evidence, and internal
  faults only produce 30–35.
- Degraded geometry **cannot** produce CLEAR 14, remove cargo points,
  exclude from static map, or commit to MapCommit.
- Formal-clear requires all authorities (formal geometry, valid observation,
  no obstacle, all contracts satisfied).
- LOST_HOLD expiry outputs 33 and stops formal map removal.
- Duplicate timestamps, heartbeat ticks, and single-frame CLEAR do not
  advance confirmation.
- Stale pending envelope cannot keep session ready.
- Static evidence from map alone cannot bypass `current_source_unvalidated`
  or `cargo_residual_source_unresolved` fail-safe 34.

### Field Validation

2026-07-27 / 2026-07-28 field verification of `8d7d7ee`:

- SLAM: 235 × Code 18, 53 × Code 17, 24 independent avoidance episodes
- Controller: 243 × Code 18 received, 224 × S3 sends, 2.2 s rate limit,
  < 1 s first callback-to-send latency
- SLAM and controller logs from different dates; not a single synchronized
  frame-by-frame run
- S3 independent gate path (total-gate closed) not covered

Tag: `validation-obstacle-avoidance-20260728` → `8d7d7ee`

### Known Limitations

- Independent S3 gate-off field case pending
- Persistent map report layer not yet implemented
- NDT fitness auto-fuse pending
- Relocalization final acceptance pending
- Torsion HOIST_MISSING diagnostics (2 cargo_swing_monitor test failures)
- Obstacle tracker boundary conditions (10 test failures, known at 8d7d7ee)
- Cargo component fusion edge cases (2 test failures, known at 8d7d7ee)
- Controller-side Code 17 → S3 path requires separate field verification

## Cargo safety and localization production hardening (2026-07)

### Added

- 稳健二维 OBB、冻结 `LockedCargoShape` 与实时 `LiveCargoPose`；
- 旋转几何统一服务 marker、Cargo Bottom、避障距离、Registration 和 MapCommit 剔除；
- `pose_evidence_stamp / height_evidence_stamp / evaluation_stamp` 分离；
- LOST_HOLD 显示/正式安全双生命周期、短时运动预测和不确定度扩张；
- `STATIONARY_HOLD / MOVING_CONFIRM / CATCH_UP` 状态机；
- 结构保持 Registration Source、可观测性代理和 EKF 各向异性协方差；
- 不可变 `MapLayerBundle` 与后台 clean 同代发布；
- 14/17/18 正式空间合同、heartbeat 新证据/时间 epoch 状态机；
- 项目级静态与 ROS Noetic catkin CI、完整工程文档。

### Changed

- 吊物中心限制改为 `physical speed * sensor dt + margin`；
- Gravity AUXILIARY 以 LiDAR 为主，空载使用三态观测和独立确认；
- 结构不足改为 prediction-only，删除 full-ground fallback；
- 风险控制台改为 ENTER/CHANGE/REPEAT/CLEAR 事件模式，逐帧数据保留 CSV；
- 安装规则补齐 rviz、scripts、docs 和 systemd 模板。

### Safety

- 17/18 只表示真实空间碰撞风险；定位、Gravity、吊物高度、障碍证据和内部故障只能输出 30-35；
- LOST_HOLD 过期后 marker 可继续显示，但输出 33 且停止正式地图剔除；
- 重复时间戳、heartbeat tick 和单帧 CLEAR 不再推进确认。

### Validation status

- Windows 静态合同可执行；
- Ubuntu clean build、gtest 与顺序 bag 验收仍是发布准入项；
- 根目录 LICENSE 与 package.xml 的 MIT 声明现已一致。

## master cargo v1 - OdomAnchorBox clean pipeline

### Added

- OdomAnchorBox：绿色货物框中心固定在 `base_link` / odom anchor；
- 货物框 size / height 自适应；
- `OdomAnchorSummary` 每 2 秒输出；
- cargo debug 点云配置开关；
- legacy cargo 配置 gate。

### Changed

- 高频 cargo 调试日志降级到 DEBUG；
- cargo visualizer 简化为 `base_link` marker visualizer；
- HookCargoRemoval 默认关闭；
- debug 点云默认关闭；
- 主线默认保持 A7 风格平滑轨迹链路。

### Removed

- 旧 hook ROI 检测主路径；
- 旧 locked hook box 发布路径；
- precise box / map corners 主显示路径；
- precise box callback。

### Validation

- Baseline：cargo 全关，`CraneMotionEKF recovery=0`；
- Display-only：只开 OdomAnchorBox，`CraneMotionEKF recovery=0`；
- NDT fitness 正常；
- 无 `[ERROR]`；
- 绿色框中心锚定 `base_link` / odom anchor；
- legacy cargo 默认关闭。

## v2.1-cleanup (2026-06-24)

### 项目结构清理
- 删除死代码：odometry、visualizer、MappingNode、KISS-ICP、ImGui
- 拆分 mapping.hpp 为 keyframe_manager.hpp
- 清理 CMakeLists.txt（去掉 OpenGL/GLFW/ImGui 依赖）
- 整理 scripts/ 为 postprocess/mapping/deploy/tools
- 重构 README 为项目入口文档
- 统一 doc/ 目录

## v2.0-longterm (2026-06-24)

### 长期在线建图
- MotionGate：静止不建图
- 关键帧 active window：最近 80 帧保留 raw cloud
- 磁盘 tile：20m × 20m 增量落盘，tmp + rename
- MemoryGuard 四级分级（OK/SOFT/HARD/EMERGENCY）
- DiskGuard 磁盘保护
- NDT fitness 健康监控
- observe_only 观察模式
- runtime_status.json 运行状态
- systemd 自动重启

### 动态物体过滤
- BasePayloadChannelFilter：吊货通道过滤
- PayloadTrackManager：吊货轨迹跟踪
- HumanObjectDynamicFilter：人体动态过滤
- DynamicEventManager：统一事件管理

## v1.0 (初始版本)

- NDT_OMP 配准
- 网格局部地面分割
- ScanContext + g2o 闭环检测
- 多层地图输出
