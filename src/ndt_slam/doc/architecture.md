# 系统架构

## 运行链路

### 定位链路

```
双雷达 → pointcloud_merger → /merged_points
  → 近场/人体/吊物候选过滤
  → RegistrationCloudBuilder（结构保持）
  → NDT + NdtObservability
  → 各向异性 EKF（CraneMotionEKF）
  → StationaryMotionPolicy
  → odom / TF / runtime_path
  → MapCommit → raw layer snapshot → Clean Worker → 五层 MapLayerBundle
```

### 吊物安全链路

```
同一帧 LiDAR
  → Cargo Observation（检测/聚类/方向）
  → 货物生命周期（EMPTY → CANDIDATE → LOCKED → LOST_HOLD）
  → Track Lock（冻结长/宽/高/yaw）
  → CargoGeometryFusion
      ├── Formal Geometry（静态+实时来源连续一致授权）
      │     可 CLEAR 14、可 removal、可 map exclusion、可 MapCommit
      └── Degraded Geometry（仅实时来源）
            仅正向 17/18 告警，不授权 CLEAR/removal/exclusion
  → Pending Envelope（新出现障碍临时告警几何）
  → Cargo Bottom（支撑点/跨度/网格覆盖/底部高度）
  → CargoObstacleTracker（近场/远场历史、静态 provenance、独立/嵌入 track）
  → CargoAvoidanceFusion（安全决策融合）
  → CargoSafetyStatus（类型化输出，schema v6）
  → cargo_alarm_heartbeat_node（合同校验 + 5Hz 状态重发）
  → /cargo_avoidance/status_code
  → 外部主控程序
  → S3 语音告警
```

## 定位职责边界

- `RegistrationCloudBuilder`：优先保留竖直静态结构、未授权候选和限额地面，不允许 full-ground fallback。
- `NdtObservability`：使用结构法向构造二维信息代理（非 NDT Hessian）。强弱方向旋转到 map/EKF 坐标系后生成各向异性测量协方差。
- `CraneMotionEKF`：负责预测、NDT 更新和静止伪测量。
- `StationaryMotionPolicy`：独立决定运行位姿保持（STATIONARY_HOLD）、真实移动确认（MOVING_CONFIRM）、追赶（CATCH_UP），以及 local map 与持久地图写入权限。

## 吊物职责边界

- **检测与锁定**：稳健二维 OBB（中心化协方差主轴、P08/P92 投影范围、长宽比、特征值比、多帧轴向集中度）。确认后冻结 `length/width/height/yaw`。
- **CargoGeometryFusion**：管理 Formal 和 Degraded 两种几何授权级别。稳定 live-only 几何可在 Track Lock 后冻结，但保持 `formal_authorized=false`；仅静态+实时物理来源连续一致后才升级为 Formal。
- **LiveCargoPose**：保存中心、真实证据时间、计算时间、来源和位置不确定度。作业期间只更新中心，起升只改变 Z。
- **CargoMarkerLifecycle**：显示生命周期。显示保持不等于正式安全证据有效。
- **CargoSafetyEvaluator**：只根据空间碰撞关系产生 14/17/18，证据故障产生 30-35。
- **cargo_alarm_heartbeat_node**：校验 `CargoSafetyStatus` 合同，接受前进时间戳的新状态，5Hz 重发。重复时间戳不产生新证据。

## 地图职责边界

运行地图是可变工作区；正式发布地图是不可变 `MapLayerBundle`。

五层地图：
- `registration` — 配准层
- `display` — 全量显示层
- `ground` — 地面层
- `objects` — 原始静态物体层
- `objects_clean` — 清理后静态物体层

Clean Worker 从 raw bundle N 构建 clean N，完成后一次性发布同代五层。工作地图已前进到 N+1 时，完整 N 仍可发布，但其 clean 层不会反向覆盖当前工作地图。

## 障碍物追踪职责边界

- `CargoObstacleTracker`：维护近场/远场历史、独立静态 provenance、嵌入/分离 track 区分。
- 正式 provenance：`PRE_CARGO_OCCUPANCY`、`STATIC_MAP_MATCH`、`CARGO_MOVED_AWAY_PERSISTENCE`、`DUAL_LIDAR_CONSENSUS`。
- `OUTSIDE_CARGO_SHELL_ONLY` 仅说明点在吊物壳层之外，不能独立证明外部静态障碍。
- 长期地图只提供独立静态 provenance，不能绕过 `current_source_unvalidated` 或 `cargo_residual_source_unresolved` 的 fail-safe 34。

## 时间合同

LiDAR、Gravity、Cargo Bottom 和 Safety 都以源时间戳判定证据是否前进。时间回退帧立即故障闭锁（Code 30）并建立新 epoch；后续新时间轴可恢复。显示时间、证据时间和评估时间不得互相冒充。
