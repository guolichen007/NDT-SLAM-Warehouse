# CHANGELOG

## [Unreleased] - Server Validation RC

### Added

- Persistent static-obstacle evidence with immutable bounded query snapshots,
  clean-map provenance, versioned sidecar manifests, and point-coordinate tile
  partitioning.
- Per-cell clean/invalidation versions, consecutive observation streaks, and
  detailed `static_evidence.csv`, cargo CSV, topic, runtime-status, and
  periodic summary diagnostics.
- Top-derived frozen-thickness formal cargo height and an independent periodic
  MemoryGuard timer.

### Changed

- Stale clean workers may confirm cells that were not invalidated by a newer
  build; newer invalidation tombstones always win.
- Lifecycle changes make the previous manifest inactive and retain it as a
  last-good archive until a mature current-epoch snapshot is atomically
  committed.
- Static-map authorization uses current-query coverage, cell count, height,
  source validation, and observation history. IoU remains diagnostic so
  partial visibility does not reject a valid dense static map.

### Known limitations

- Main-controller integration is not accepted and no production tag is issued.
- Ubuntu clean build/tests and sequential/long-duration Bag results remain
  release gates to be executed on the ROS Noetic validation host.
- Source, geometry, association, formal-height, tile, and memory thresholds
  remain field-validation parameters; this RC does not lower safety gates.

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
