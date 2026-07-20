# 吊物跟踪与安全

## 检测、锁定与移动

稳健二维 OBB 使用中心化协方差主轴、P08/P92 投影范围、长宽比、特征值比和多帧轴向集中度。`yaw` 与 `yaw + pi` 等价。确认后冻结 `length/width/height/yaw`，作业期间只更新中心。

实时中心保留 `measured/predicted/filtered/residual/sensor_dt/source`。单帧允许修正量由物理速度决定：

```text
max_xy_step = max_xy_speed * sensor_dt + margin
max_z_step  = max_z_speed  * sensor_dt + margin
```

重复时间戳不更新中心。当前 LiDAR 是首选来源；丢点时只在短窗内使用运动预测，最后才保持最近位置。

## LOST_HOLD 双生命周期

- 显示：保留到 `lost_clear_sec`，用于操作员识别，降级样式不表示安全有效。
- 正式安全/剔除：只在 `formal_hold_sec` 内允许使用预测或保持证据；过期后输出 33，并立即停止正式地图剔除。

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
| 14 | 无空间碰撞风险 |
| 17 | OBB 距离 <= 3 m 且保守垂直净空 < 0.8 m |
| 18 | 3 m < OBB 距离 <= 5 m 且保守垂直净空 < 0.8 m |
| 30-35 | 系统、定位、Gravity、吊物、障碍或内部证据故障 |

17/18 只表达真实空间碰撞风险。新鲜正式状态立即生效；重复 stamp、heartbeat tick 和单帧 CLEAR 都不能伪造恢复证据。

## 静态障碍 provenance 与 34 决策

正式 provenance 包括 `PRE_CARGO_OCCUPANCY`、`STATIC_MAP_MATCH`、
`CARGO_MOVED_AWAY_PERSISTENCE` 和 `DUAL_LIDAR_CONSENSUS`。
`OUTSIDE_CARGO_SHELL_ONLY` 仅说明点位于当前吊物壳层之外，不能独立证明它是
外部静态障碍，因此不能单独授权 17/18。

每条障碍 track 独立维护关联、连续帧、持续时间、大簇几何和 provenance。
大簇门同时检查点数、XY 面积、长边、高度跨度与占用网格；关联检查中心距离、
top step、cell overlap 和 IoU。任一来源未验证、静态来源不足、几何不足、连续
帧不足或持续时间不足均输出 34，并在 `static_evidence.csv` 中记录实际值、门限和
reset reason。长期地图不能覆盖 `current_source_unvalidated` 或
`cargo_residual_source_unresolved`。
