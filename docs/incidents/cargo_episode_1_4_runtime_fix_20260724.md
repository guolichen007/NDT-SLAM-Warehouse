# Episode 1–4 吊物避障运行根因修复说明

## 基线与边界

- 输入提交：`7ba1c40ebcad3b41cc857088ec2bad8cfc82507f`
- 分支：`fix/56f-false17-monitor-map-persistence-v1`
- 本次只修改吊物识别、持续跟踪、Pending 避障、OBB 与摆动监控。
- 未修改定位 MotionGate、消息 schema、`.github`、`src/ndt_omp` 和地图持久化。

## 对运行报告的因果校正

运行日志中的 `POSE_REJECTED` 并不是
`CargoVerticalPoseSource` 枚举没有设置。代码已经能把顶面观测标记为
`DIRECT_TOP`。真正的拒绝条件是：即使当前帧已经通过正式货物关联，
Pending 垂直合理性检查仍要求额外存在局部地面或权威吊点 Z。

`LOCKED -> LOST_HOLD` 也不由 Pending 位姿合理性检查触发。它由货物检测
与冻结 OBB 的 XY 中心、点数、覆盖率和尺寸关联结果触发。旧逻辑在关联丢失
8 秒后，不区分重力仍为 `LOADED`，直接删除正式轨迹并进入
`CLEAR_WAIT_REARM`，这是 7–21 秒后长期失去避障能力的主因。

## 已实施修改

### 1. 重力带载时持续保留正式货物框

- `LOST_HOLD` 超过原 `lost_clear_sec` 后：
  - 重力为 `LOADED`：保留冻结形状、最后可信位姿和增长后的不确定度；
  - REQUIRED 重力变 stale/UNKNOWN/INHIBIT：同样不得把货物清空；
  - 只有新鲜、权威的 `EMPTY` 才允许退役该轨迹。
- 未恢复当前点云时不会刷新正式 XY/Z 证据时间，因此仍禁止用陈旧轨迹授权
  CLEAR 14。
- Pending 17/18 仍要求真实外部障碍轨迹、连续确认、有效来源和同一危险
  cluster；仅有保留货物框不会制造正式告警。

### 2. 有界重捕获

- `LOST_HOLD` 的 XY 重捕获门限从 `0.55 m` 起步，按丢失时长和位置不确定度
  增长，最大限制为 `1.05 m`。
- 扩大 XY 门限不绕过以下条件：
  - 冻结 OBB 内点比例；
  - 长短轴覆盖率；
  - 冻结尺寸一致性；
  - 最小点数；
  - 未发生 OBB 尺寸裁剪。
- 对旧版本遗留的 `CLEAR_WAIT_REARM`，允许与退役冻结形状一致但位置已经移动
  的候选进入五帧连续重确认。第一帧匹配退役身份，后续帧匹配新候选自身的
  连续运动，不再强迫它回到过期的旧中心。

### 3. 当前 LiDAR 位姿与可信位姿快照

- 当前帧同时满足正式关联和物理垂直观测
  (`DIRECT_TOP` / `DIRECT_BOTTOM` / `LOCKED_OBB_POINT_SUPPORT`) 时，
  该 LiDAR 位姿本身可作为垂直物理参考，不再强制要求同帧额外出现地面或
  权威吊点 Z。
- 保存同一 lifecycle、同一 track segment 的最后物理测量位姿。
- 短时预测或退役框缺少地面参考时，只能在有限时间内、且 Z 与该可信快照
  连续时通过检查。
- 大幅 Z 跳变（例如 Episode 1 的地下框）仍会被拒绝。
- 退役时优先保存最后可信物理位姿，并继承其真实证据时间；不会把超时退役
  的时刻冒充为新鲜测量时间。

### 4. OBB 上限污染

- OBB 估计同时输出裁剪前尺寸和裁剪标志。
- 超过物理搜索上限的融合组件在候选排序前被排除，使较小的独立货物组件有
  机会被选择。
- 任何仍发生尺寸裁剪的 OBB 只能用于诊断，禁止建立正式锁，也禁止更新已锁
  定身份。
- 运行诊断新增裁剪前尺寸和 `footprint_clamped`，可直接确认是否再次出现
  Episode 4 的 `4.0 m` 上限饱和值。

### 5. 非权威吊点不得产生摆动/斜拽正式状态

- 角度权威现在同时要求吊点 XY 和 Z 都权威。
- 所有基于水平偏移、速度、振幅、DC/AC 的摆动与斜拽阈值都要求
  `offset_authoritative=true`。
- 配置吊点仍发布原始偏移和诊断角度，但 sway/skew 保持
  `NOT_EVALUATED`，不再产生假 `sway=4`。
- 已由权威测量锁存的真实报警不会因后续权威丢失而被自动清除。

### 6. 默认 Pending 框

- 默认框由 `4.0 × 3.0 × 3.0 m` 调整为 `3.0 × 2.0 × 1.5 m`。
- 该尺寸覆盖当前三组可信实测货物并保留余量。
- 该默认源仍仅用于 LOADED 且无可靠形状时的显示和保守查询；永远不能授权
  CLEAR 14、货物点删除、MapCommit 或单独提升为正式 17/18。

## 运行验收重点

下一轮 Ubuntu/服务器运行应重点确认：

1. 重力持续 `LOADED` 时，状态可在 `LOCKED` 与 `LOST_HOLD` 间恢复，不再因
   8 秒超时进入长期 `CLEAR_WAIT_REARM`。
2. `CargoAssociation` 日志给出具体 reason、动态 gate、点数、OBB 覆盖率和
   裁剪状态。
3. 当前正式关联且顶面有效时，Pending reason 为
   `current_lidar_pose_physically_plausible`。
4. 退役/短时保持若使用可信快照，reason 为
   `held_pose_matches_trusted_center`；地下跳变必须继续 `POSE_REJECTED`。
5. 配置吊点下 `offset_authoritative=false`，sway/skew 为
   `NOT_EVALUATED`。
6. 不应再形成裁剪到 `4.0 m` 的正式冻结框。

Ubuntu ROS/C++、Bag 和服务器验证均未在 Windows 执行。
