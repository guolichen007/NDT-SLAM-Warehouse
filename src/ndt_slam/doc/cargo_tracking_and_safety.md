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

### 起吊前基线

独立的 `CargoPreloadBaselineTracker` 在 `EMPTY` 且定位可靠时直接读取静态
高度场，不依赖被 EMPTY 门控关闭的实时货物检测。最近 8 帧中至少 5 帧稳定后保存
顶面、局部支撑面、OBB、成员网格、地图代次和 MAD，并在 `EMPTY→LOADED` 生命周期中
复用。普通组件要求车辆静止；已经独立成熟、早于本次生命周期且身份/代次/几何连续一致的静态组件允许在移动中复核。它不读取 odom Z；NDT 只提供地图 XY 位姿和运动状态。锚点与静态组件距离超过
0.5m、少于 6 个授权网格、组件不确定度超过 0.2m、地图代次/组件身份变化、时间回退
或观测间隔超限都会清空该候选。

### 厚度约束

- `FULL_MEASUREMENT`：起吊前 top-support、露底复核或确实可见底边的完整测量；
- `LOWER_BOUND`：普通实时 LiDAR 可见高度，只证明货物至少这么厚；
- `PRIOR_ONLY`：退役形状或配置先验，不计作独立物理证据。

同一批实时点只计一次。静态完整厚度与实时下界冲突时不再平均，也不再用
`thickness_source_disagreement` 永久清零；系统采用两者上界，并按下式形成保守底面：

```text
conservative_bottom = top_reference
                    - thickness_upper_bound
                    - tracking_allowance
                    - max(0.15m, 3 * top_MAD)
                    - safety_margin
```

### 三级授权

| 级别 | 形成条件 | 允许行为 |
|---|---|---|
| `PENDING` | 身份、形状或厚度约束不足 | 仅积累证据，正式状态保持 33 |
| `POSITIVE_ONLY` | 当前货物身份有效，最近 8 帧内至少 5 帧实测尺寸、LiDAR 厚度约束与保守底面稳定；不要求先取得双源正式锁 | 有真实危险可输出 17/18；无危险输出 33 |
| `FORMAL` | 静态基线+露底复核，或静态基线+明确可见底边连续一致 | 可输出 14，并可剔除货物点和授权 MapCommit |

没有静态地图时，锁定形状以不少于 50% 高度不确定度形成实时下界，满足多帧条件后
可进入 `POSITIVE_ONLY`；没有当前身份、成员点或稳定实测形状则保持 `PENDING`。地图代次变化会撤销旧的正式
授权，必须用当前代次证据重建。

### Pending Envelope（待确认包围盒）

新出现但尚未达到正式确认条件的障碍几何：
- 可持续积累观测证据
- 连续确认后可升级
- 旧 Pending Envelope 不能保持 session ready

Pending 阶段仍评估实时障碍与静态地图风险并积累 track、来源和区域连续性证据。生产
默认 `fusion_pending_warning_promotion_policy: evidence_backed_only`，只有身份、外部障碍
Track、来源、几何和置信度门禁全部通过时，原始 Pending 正向风险才可升级为 17/18。
该策略不影响已经通过稳健几何确认的 `POSITIVE_ONLY` 正向告警。首次名义长宽取最近
8 帧合格观测的中值；长度或宽度 MAD 超过 0.30 m 时继续等待，不使用最后一帧直接
冻结，从而避免高置信度合并簇造成框瞬时放大。

Pending 路径永不授权 CLEAR，也不借用另一条路径的 track 身份。运行诊断分别输出
`pending_live_warning_authorized` 和 `pending_static_warning_authorized`，不能再把旧的
`pending_positive_warning_authorized` 原因串当作两条路径的共同证据。

Pending 静态查询不再循环依赖正式起吊原点。若当前候选已通过同生命周期身份、成员点、
形状和时效门禁，查询会按当前名义 OBB 与垂直区间排除货物自身地图层；外部成熟静态格
仍需独立 3 帧确认。该几何自排除只授权正向风险，不能授权 CLEAR 或地图删除。诊断字段
`pending_static_geometry_exclusion_authorized` 与
`pending_static_self_exclusion_authorized` 用于区分几何排除和正式原点排除。

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
| 17 | OBB 距离 ≤ 3m 且保守垂直净空 < 0.8m。FORMAL 或 POSITIVE_ONLY 均可 |
| 18 | 3m < OBB 距离 ≤ 5m 且保守垂直净空 < 0.8m。FORMAL 或 POSITIVE_ONLY 均可 |
| 29 | 0.30m 全方向接触候选，或没有 18 远场接近历史的突发 3m 内候选；仅供人工复核 |
| 30-35 | 系统、定位、Gravity、吊物、障碍或内部证据故障 |

### 运行方向门禁

3m/5m 距离只在运行方向前方 90° 扇区内判定，即速度方向左右各 45°。该门禁同时作用于
正式几何、Pending 实时点簇、预跟踪候选和静态高度场查询，不能因包络来源切换而绕过。
扇区边界计入前方，侧后方普通障碍不会授权 17/18；距离货物不超过 0.30m 的接触级候选仍保留全方向检测，但只输出 Code 29。速度无效或低于运动门限时不使用旧方向授权正向预警，保持故障状态。

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

已由 `STATIC_MAP_MATCH` 或 `PRE_CARGO_OCCUPANCY` 证明的静态障碍连续 3 帧即可建立 Track；
它的成熟地图证据本身就是独立的跨帧物理历史，因此首次在 3m 内出现时可以直接输出
Code 17。实时新障碍仍必须先有 Code 18 远场接近历史；缺少该历史却突然进入 3m 时
输出 Code 29，不直接进入主流避障链。未入静态地图的大型固定物体要求至少 5 帧且
持续不少于 0.8 秒。人员状小簇、稀疏点和快速变化点簇不能仅凭距离授权告警。

障碍 Track 还记录稳健底面、顶面和垂直连续性。整体位于货物上方的悬空簇、孤立高点
或垂直连续性不足的点簇不会触发；从地面连续延伸到货物高度以上的墙体仍会触发。
envelope 的 pose/shape/source 枚举切换不清空同一障碍 Track，只有生命周期改变、身份
断裂，或伴随实际位置/尺寸跳变的 Track 段变化才重置。

### 34 决策

任一来源未验证、静态来源不足、几何不足、连续帧不足或持续时间不足均输出 34，并在 `static_evidence.csv` 中记录实际值、门限和 reset reason。长期地图不能覆盖 `current_source_unvalidated` 或 `cargo_residual_source_unresolved`。

## 摆动监控（CargoSwingMonitor）

检测货物摆动和偏拉：
- Sway：摆动振荡
- Skew-pull：持续性方向偏移
- Torsion：扭转（需要非正方形货物）
- 吊钩锚点授权（真实 hoist 信号 vs 配置值）

对应版本：`f57d68a`。
