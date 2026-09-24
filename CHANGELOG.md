# 变更日志

## [Unreleased]

无未发布生产行为变更。

## baseline-field-legacy-20260923

当前现场 LEGACY 运行基线（`18619e7`）。

- Cargo V6 functional baseline（吊物身份/几何/安全）
- Avoidance V4 functional baseline（障碍追踪 + 17/18/29 避障，双 Bag 验收 PASS）
- semantic map identity tooling（schema2 `map_frame_uuid` path-independent）
- production yaw authority = LEGACY，`verified=false`
- Full GTest 1000/0，Python 107/0
- 历史取证文档与生成数据从当前工作树移除，由 Git 历史承担

## Cargo V6 / Avoidance V4 Freeze（2026-08 ~ 2026-09）

- 吊物几何融合、Pending Cargo Envelope、外部障碍追踪器、CargoSwingMonitor、
  CargoLiftOriginBinder、StaticHeightField、MapSessionSnapshot、
  StaticEvidenceAuthorization、RevealedSupportObserver、CargoPhysicalMotionEstimator、
  CargoPresenceStateMachine、CargoSafetyEvaluator、cargo_alarm_heartbeat_node
- Code 14/17/18/30-35 类型化安全合同，正式 CLEAR 授权门控
- Cargo V6 canonical authority（identity → group → formal → vertical → static）
- 假 29 权威链闭环（CargoFrameDecision atomic authority + distance<=0 + geometry 兜底）
- semantic map identity（legacy→schema2 migration）

## 货物安全与定位生产加固（2026-07）

- 稳健二维 OBB、冻结 LockedCargoShape 与实时 LiveCargoPose
- 结构保持 Registration Source、可观测性代理和 EKF 各向异性协方差
- 14/17/18 正式空间合同、heartbeat 状态机

## master cargo v1 — OdomAnchorBox 干净管线

- OdomAnchorBox 货物框、OdomAnchorSummary
- 移除旧 hook ROI 检测主路径

## v2.1-cleanup (2026-06-24)

- 删除死代码（odometry/visualizer/MappingNode/KISS-ICP/ImGui）
- 拆分 mapping.hpp、清理 CMakeLists、重构 README 与 doc/ 目录

## v2.0-longterm (2026-06-24)

- 长期在线建图（MotionGate、关键帧 active window、磁盘 tile、MemoryGuard/DiskGuard）
- 动态物体过滤（PayloadChannelFilter、PayloadTrackManager、HumanObjectDynamicFilter）

## v1.0（初始版本）

- NDT_OMP 配准、网格局部地面分割、ScanContext + g2o 闭环检测、多层地图输出
