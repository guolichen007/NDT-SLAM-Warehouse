# 吊物跟踪与安全

## 检测、锁定与移动

稳健二维 OBB 使用中心化协方差主轴、P08/P92 投影范围、长宽比、特征值比和多帧轴向集中度。`yaw` 与 `yaw + pi` 等价。确认后冻结 `length/width/height/yaw`，作业期间只更新中心。

实时中心保留 `measured/predicted/filtered/residual/sensor_dt/source`。单帧允许修正量由物理速度决定：

```text
max_xy_step = max_xy_speed * sensor_dt + margin
max_z_step  = max_z_speed  * sensor_dt + margin
```

重复时间戳不更新中心。当前 LiDAR 是首选来源；丢点时只在短窗内使用运动预测，最后才保持最近位置。

## 生命周期

`EMPTY → CANDIDATE → LOCKED → LOST_HOLD → EMPTY`

- **EMPTY**：等待货物检测
- **CANDIDATE**：检测到候选货物，积累确认证据（方向集中度、几何稳定性）
- **LOCKED**：冻结长/宽/高/yaw，开始正式安全评估
- **LOST_HOLD**：丢失货物跟踪

## 几何融合（CargoGeometryFusion）

`CargoGeometryFusion` 是 8d7d7ee 基线核心设计，管理两种几何授权级别：

### Formal Geometry（正式几何）

由静态地图证据和实时 LiDAR 观测连续一致后授权：
- 冻结形状（长/宽/高/yaw）经多源确认
- 可以产生 CLEAR 14（当所有安全合同同时满足）
- 可以正式剔除货物点（registration、MapCommit）
- 可以排除静态地图中的货物区域

### Degraded Geometry（降级几何）

仅由实时 LiDAR 观测支持，未获得静态+实时双源一致授权：
- 只能产生正向 17/18 告警
- **不能**产生 CLEAR 14
- **不能**正式剔除货物点
- **不能**排除静态地图区域
- **不能**授权 MapCommit 排除

稳定 live-only 几何可在 Track Lock 后冻结，但保持 `formal_authorized=false`。仅当静态+实时物理来源连续一致后才升级为 Formal。

### Pending Envelope（待确认包围盒）

新出现但尚未达到正式确认条件的障碍几何：
- 可持续积累观测证据
- 连续确认后可升级
- 旧 Pending Envelope 不能保持 session ready

## LOST_HOLD 双生命周期

- **显示**：保留到 `lost_clear_sec`，用于操作员识别，降级样式不表示安全有效。
- **正式安全/剔除**：只在 `formal_hold_sec` 内允许使用预测或保持证据；过期后输出 33，并立即停止正式地图剔除。

系统分别保存 `pose_evidence_stamp`、`height_evidence_stamp`、`evaluation_stamp`。评估 tick 不得刷新证据时间。短窗保持期间安全 OBB 的长宽按横向位置不确定度双侧扩张。

## 空载证据

检测结果是 `CARGO_DETECTED / EMPTY_CONFIRMED / UNKNOWN` 三态。点云稀疏、HAG 残余、体素点不足、无有效聚类、地面无效或覆盖不足都属于 UNKNOWN，不推进空载确认。只有连续独立、质量有效的明确空观测才能确认无货。

## Gravity 角色

- `REQUIRED`：候选和最终锁定都要求有效 LOADED。
- `AUXILIARY`：LiDAR 为主；LOADED 可增强确认，EMPTY 产生冲突/延迟，但不能永久禁止可靠 LiDAR 小件检测。
- `DISABLED`：完全使用 LiDAR。

EMPTY 阶段可采集起吊前高度，但无抬升/运动证据不得把地面货物锁成悬吊货物。

## 正式安全码

| Code | 合同 |
|---:|---|
| 14 | 无空间碰撞风险。必须 Formal Geometry + 有效观测无检测障碍 + 全部安全合同满足 |
| 17 | OBB 距离 ≤ 3m 且保守垂直净空 < 0.8m。Formal 或 Degraded Geometry 均可 |
| 18 | 3m < OBB 距离 ≤ 5m 且保守垂直净空 < 0.8m。Formal 或 Degraded Geometry 均可 |
| 30-35 | 系统、定位、Gravity、吊物、障碍或内部证据故障 |

17/18 只表达真实空间碰撞风险。新鲜正式状态立即生效；重复 stamp、heartbeat tick 和单帧 CLEAR 都不能伪造恢复证据。

## 障碍物追踪（CargoObstacleTracker）

每条障碍 track 独立维护：
- 关联（中心距离、top step、cell overlap、IoU）
- 连续帧计数
- 持续时间
- 大簇几何（点数、XY 面积、长边、高度跨度、占用网格）
- Provenance（来源证明）

### Provenance 级别

正式 provenance：
- `PRE_CARGO_OCCUPANCY` — 吊物到达前已存在
- `STATIC_MAP_MATCH` — 与静态地图匹配
- `CARGO_MOVED_AWAY_PERSISTENCE` — 吊物移开后持续存在
- `DUAL_LIDAR_CONSENSUS` — 双雷达一致确认

非正式：
- `OUTSIDE_CARGO_SHELL_ONLY` — 仅位于吊物壳层之外，不能独立证明是外部静态障碍，不能单独授权 17/18

### 近场/远场历史

障碍在进入告警范围之前需要建立远场历史。near_field_track_missing_far_history 的 track 不能直接产生告警。

### 34 决策

任一来源未验证、静态来源不足、几何不足、连续帧不足或持续时间不足均输出 34，并在 `static_evidence.csv` 中记录实际值、门限和 reset reason。长期地图不能覆盖 `current_source_unvalidated` 或 `cargo_residual_source_unresolved`。

## 摆动监控（CargoSwingMonitor）

检测货物摆动和偏拉：
- Sway：摆动振荡
- Skew-pull：持续性方向偏移
- Torsion：扭转（需要非正方形货物）
- 吊钩锚点授权（真实 hoist 信号 vs 配置值）
