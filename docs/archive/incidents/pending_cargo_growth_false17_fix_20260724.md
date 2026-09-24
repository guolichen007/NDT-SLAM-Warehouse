# Pending 吊物框膨胀与假 17 根因修复（2026-07-24）

## 输入与边界

- 输入提交：`b992378217b4d8757f293cad033a09798182616d`
- 上一修复提交：`26cfef8e0c4a6aba20aa481017874cd4e88f0157`
- 分支：`fix/56f-false17-monitor-map-persistence-v1`
- 本轮只修改吊物 Pending 几何与避障安全链。
- 未修改消息 schema、MotionGate、定位、地图持久化、`.github` 或
  `src/ndt_omp`。

## 服务器根因

1. `HOOK_DEFAULT_OFFSET` 的水平不确定度使用
   `loaded_duration_sec`，只要重力保持 LOADED，框体就按固定速率持续
   膨胀。该量不是失联时长，也不包含重量信息。
2. Pending 构造把不确定度直接写进 `length/width/height`，导致 RViz
   名义框和安全查询区域混成同一个不断扩大的实体。
3. 测得形状又被 `max(measured, configured)` 强制以 4×3×3 m 为下限，
   配置 fallback 错误地覆盖了真实 LiDAR 几何。
4. 正式框退役时先线性外推一次，后续每帧又按旧速度重复外推，产生
   Episode 2 中整体落到地下的 OBB。
5. `CLEAR_WAIT_REARM` 仍可携带同 lifecycle 的退役身份点，因此旧的
   identity gate 允许其继续积累障碍确认并提升 17。
6. Pending 发布链只带障碍点数和距离，没有把 code、cargo track、
   obstacle track、确认次数、来源和置信度绑定到同一个确认 cluster。
   因此服务器日志出现 `code=17` 同时 `obstacle_track=0`、
   `confidence=0`。

## 实施修改

### 名义几何和安全不确定度分离

- Pending 的 `length/width/height` 只保存名义物理尺寸，供 RViz 和诊断
  使用。
- 水平、垂直不确定度单独保存，并只在实时点云/静态高度查询时显式
  扩张一次。
- 不确定区域内的点保持 `UNRESOLVED_INSIDE_PENDING`，不会把位置误差
  自己制造成“外部障碍物”。
- 4×3×3 m 配置仅在完全没有物理形状时作为显示 fallback；不再是
  LiDAR、退役正式框或静态来源形状的尺寸下限。

### 候选框连续性

- 新增按 `cargo_lifecycle_id + provisional_track_id` 隔离的
  `CURRENT_TRACKED_BOUNDED_SHAPE`。
- 弱但已关联的 LiDAR 形状可以持续驱动 Pending 框，不再必须等待全部
  high-quality 条件同时成立。
- 尺寸扩大和缩小分别限速，短时丢帧只在有限 hold 窗口内保留。
- lifecycle 或 provisional track 改变时立即清空，禁止跨货物继承。

### 正确的不确定度时间基准

- 完全移除 `loaded_duration_sec` 对框体的驱动。
- 只在 `LOST_HOLD` 中，按“当前时刻 - 最后可靠货物位姿时间”增长失联
  不确定度，并受最大值约束。
- CANDIDATE/GEOMETRY_CONFIRMING 使用当前跟踪残差，不因带载时间增长。

### 退役框物理约束

- 退役中心使用现有带速度衰减、速度上限和预测时长上限的传播函数
  一次性确定；保存后速度清零，禁止二次外推。
- 退役 Pending 在参与任何正告警查询前检查高度范围、局部地面穿透和
  权威吊点 Z 一致性。
- 缺少地面/权威吊点参考时不授权退役框正告警。

### 17/18 授权和发布证据绑定

- `CLEAR_WAIT_REARM`、`LOADED_REACQUIRE` 和 `EMPTY`
  路径明确撤销 Pending 正告警查询权。
- 未授权状态会清空 Pending 障碍 tracker，不能继承历史确认计数。
- 配置 fallback 和静态来源框只能显示与诊断，不能自行查询并提升
  17/18。
- Pending 官方 17/18 必须同时具有：
  - 当前允许告警的识别状态；
  - 物理可解释的位姿；
  - 同 lifecycle、同 track 的货物身份点；
  - 已分离且连续确认的外部障碍 track；
  - 非零 obstacle track、有效来源、大几何和最低置信度。
- 即使启用 legacy policy，也不能绕过 cargo identity 与 external track。
- 静态高度危险只做 corroboration；Pending 官方 code、距离和净空必须
  来自同一个已确认实时 cluster。
- 发布的 `CargoSafetyStatus` 同时写入 cargo track、obstacle track、
  confirmation、provenance、confidence 和 cargo bottom。任何一个合同
  字段缺失时生产节点会降为 33；heartbeat 还会把绕过生产节点的畸形
  17/18 判为合同错误 35，而不是继续转发证据为零的告警。

### 最新服务器日志增量修复

- 当识别状态已经 `LOCKED`，但双物理来源厚度尚未完成冻结时，不再退回
  `CONFIGURED_CONSERVATIVE_DEFAULT`。锁定阶段已经通过多帧身份与形状合同的
  `ACTIVE_LOCKED_TRACK_SHAPE` 会继续驱动 Pending 显示和正向避障。
- `ACTIVE_LOCKED_TRACK_SHAPE` 仍是 Pending：只允许在当前 track 身份点、
  物理可解释位姿和连续确认的外部障碍 track 同时存在时产生 17/18；
  不授权 14、正式货物点删除或 MapCommit。
- `LOST_HOLD` 的单帧关联成功不再立即切回 `LOCKED`。现在必须满足
  `loaded_reacquire_confirm_frames` 个连续恢复帧；中间任何拒绝或无检测都会
  清零计数，抑制日志中 `LOCKED ↔ LOST_HOLD` 的单帧抖动。
- 最新日志中的 `locked_shape=(2.837,1.121,0.319)` 高度仍高于当前
  `minimum_height_m=0.30`，所以“仅因货物薄而无法冻结”不是已证实根因。
  `geometry_reason=insufficient_independent_thickness_sources` 才是直接原因。
- 没有采用“只凭 locked shape 且当前无障碍点就输出 14”的建议。日志同时显示
  `observation_valid=0`、`dangerous_cluster_points=0`、`clearance=nan`；
  这代表没有完成可验证的清空观测，而不是已证明安全。正式 14 继续要求冻结几何
  以及实时/静态清空合同，避免把传感器盲区误报为安全。

## 配置决策

服务器 10 个 episode 显示实际货物 footprint 约 4.15×3.15 m，因此本轮
没有盲目把配置 fallback 缩到 2.5×2.0 m。保留 4×3×3 m 只作为无物理
形状时的可视保守框；真正的小/大货物均优先采用同 track 的 LiDAR
名义尺寸。这样既消除时间膨胀，又不会因缩小 fallback 漏掉当前现场
货物。

## Windows 验证

- `python scripts/regression/check_cargo_safety_e2e.py`：PASS
- `python -m compileall -q scripts tests`：PASS
- `python scripts/regression/check_yaml_duplicate_keys.py`：PASS
- `git diff --check`：PASS
- ROS/C++/Catkin：`NOT_RUN_WINDOWS`
- Bag：`NOT_RUN_WINDOWS`
- Ubuntu：`NOT_RUN_REQUIRES_UBUNTU`
- 服务器 episode 验收：`NOT_RUN_REQUIRES_SERVER`

## 服务器验收重点

1. CANDIDATE/GEOMETRY_CONFIRMING 的 RViz 框在 10 秒内不再从
   4×3 级别膨胀到 8×7 m。
2. `loaded_duration_sec` 增长时，名义框尺寸保持由 LiDAR/retired/static
   来源决定。
3. `CLEAR_WAIT_REARM` 全程不得输出 17/18，诊断原因为
   `clear_wait_rearm_warning_revoked`。
4. 任一 17/18 必须同时满足 `cargo_track_id>0`、
   `obstacle_track_id>0`、确认次数达标、provenance 有效、
   confidence 达标，且距离/净空来自该 obstacle track。
5. 退役框落到地面以下或缺少物理参考时，诊断显示明确 reject reason，
   不产生官方正告警。
6. 正式厚度尚未冻结的 `LOCKED` 阶段，Pending 来源应为
   `ACTIVE_LOCKED_TRACK`，框尺寸应保持锁定形状，不再回退到 4×3×3 m 默认框。
7. `LOST_HOLD` 只有连续达到配置恢复帧数后才能返回 `LOCKED`；间歇性
   `center_too_far` 不得造成逐帧往返切换。

该提交完成代码侧 P0 修复，但仍需 Ubuntu 编译与 1.0× 真实服务器 episode
验证；不得据此声明 `READY_FOR_SERVER_PRODUCTION` 或
`READY_FOR_MAIN_MERGE`。
