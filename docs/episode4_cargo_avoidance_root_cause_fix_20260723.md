# Episode 4 吊物避障根因修复说明（2026-07-23）

## 基线与边界

- 输入提交：`b992378217b4d8757f293cad033a09798182616d`
- 分支：`fix/56f-false17-monitor-map-persistence-v1`
- 本次仅修改桌面 Windows 工作区；不推送。
- 不修改定位 MotionGate、消息 schema、`.github`、`src/ndt_omp`。
- 不伪造 Ubuntu/ROS/Catkin/Bag 结果。

## Episode 4 根因复核

### 1. Pending 障碍轨迹未绑定吊物身份上下文

`pending_cargo_obstacle_tracker_` 原来只按空间位置关联障碍，没有绑定：

- `cargo_lifecycle_id`
- `cargo_track_segment_id`
- Pending envelope source
- pose source
- shape source

包络在 `CURRENT_CANDIDATE`、`RETIRED_FORMAL_SHAPE`、
`CONFIGURED_CONSERVATIVE` 之间切换时，旧确认次数可能被新上下文中的点簇续接。

### 2. 被确认的障碍和发布的障碍不是同一个 cluster

跟踪器通过 `selected_source_index` 确认一个具体点簇，但原实现发布距离和净空时使用
`most_dangerous_cluster`。多点簇帧中，两者可能不是同一物理对象。

### 3. 外部障碍“几何足够大”只检查当前帧

障碍身份需要连续帧确认，但 `large_cluster_geometry_valid` 只看当前帧。因此一个已确认
track 在几何阈值边缘波动时会产生 `17 -> 33 -> 17`。

### 4. 退役 OBB 被同时当作分类依据和告警身份依据

退役正式 OBB 可以作为保守运动范围，但没有保留身份点时不能证明框内哪些点属于吊物。
原结构没有区分“可用于 unresolved 分类”和“可授权正告警”。

### 5. 主显示链只接受正式几何

Pending envelope 已存在，但 `/cargo_avoidance/fused_box_marker`、
`CargoBottomEstimate` 诊断字段和 `cargo_geometry_debug` 在正式厚度冻结前仍输出零尺寸。
因此服务器监控看到 `0x0x0`，RViz 主框也消失。

### 6. 正式几何初始化存在回调顺序缺口

正式锁刚建立但 `live_pose` 尚未更新时，`CargoGeometryFrame` 没有中心和 footprint。
即使锁内已有多帧稳健 shape summary，也会返回
`footprint_or_center_invalid`。

### 7. GEOMETRY_CONFIRMING 把短遮挡当作身份重置

超过两个弱帧就清空整个多帧窗口；同时绝对 8 秒超时会清掉仍在持续获得有效物理证据
的候选，造成反复重新累计。

### 8. CLEAR_WAIT_REARM 混淆内部重置和安全 CLEAR

重力权威为 EMPTY、LiDAR 仍看到吊具/结构时，安全输出必须保持冲突、不能给 14；但内部
识别锁也因此无法 rearm，下一次 LOADED 无法开始干净的新生命周期。

## 已实施修复

### Pending 17/18 授权链

- Pending tracker 上下文变化时立即 reset，确认次数禁止跨生命周期、track segment 或
  包络来源继承。
- envelope、自身证据和当前节点生命周期必须完全一致。
- 退役 OBB 无身份点时仍可把框内点标为 `UNRESOLVED_INSIDE_PENDING`，但
  `positive_warning_identity_authorized=false`，不得升级 17/18。
- Pending 模式新增连续大几何计数；障碍身份确认和大几何确认都达到配置帧数后才可告警。
- 授权、warning code、距离、净空和点数全部绑定同一个
  `selected_source_index`。
- 增加 Pending 诊断字段：identity context、warning identity、selected source、
  geometry confirmation streak 和 tracker reason。

### 持续吊物框与正式授权隔离

- LOADED 且正式厚度未冻结时，主 fused marker 使用
  `EffectiveCargoEnvelope` 持续显示，颜色保持降级橙色。
- `CargoBottomEstimate.valid` 仍为 false、source 仍为 INVALID，但携带 Pending
  bottom/top/height/corners 供 RViz 和监控诊断。
- `cargo_geometry_debug` 在正式几何不可用时输出
  `geometry_source=PENDING_*` 和非零包络尺寸。
- Pending 几何仍禁止：
  - 授权 14；
  - 正式吊物点删除；
  - MapCommit；
  - 冒充正式物理厚度。

### 正式几何融合

- 锁已建立但 `live_pose` 暂缺时，使用同一锁内的 `last_accepted_center` 和
  `locked_shape` 构造过渡 geometry frame。
- 已通过正式多帧锁定的 provisional summary 可作为 shape 完整性证据，不再要求同一帧
  再次同时通过全部 high-quality 条件。
- `minimum_independent_sources` 保持为 2；配置高度不计独立物理来源。
- 正式 transition floor 改为物理最小尺寸 `0.30 x 0.20 m`，正式框从多帧实测 shape
  开始；`4 x 3 x 3 m` 只保留为 Pending 保守包络。

### 小型吊物与识别连续性

- OBB 前置尺寸门限与 compact cargo 合同统一为 `0.30 x 0.20 m`。
- 点数、面积、身份置信度、shape 置信度、覆盖率和物理起吊证据未降低。
- `GEOMETRY_CONFIRMING` 遇到短时遮挡只停止进度，不清空已确认窗口。
- 只有连续无进度达到 `candidate_progress_timeout_sec` 才回到 CANDIDATE。
- 正在持续获得有效物理证据时不再被 absolute timeout 周期性清空。

### 重力空载重新武装

- 稳定权威 EMPTY 可重置内部识别锁，即使 LiDAR 仍看到冲突结构。
- 该操作不改变 outward safety contract：有视觉冲突时仍不能输出 CLEAR 14。

## 明确保留的安全合同

- 正式厚度至少需要两个独立物理来源。
- 配置高度只用于 Pending 保守 fallback。
- Pending 永远不能授权 14。
- 无身份点的 retired OBB 永远不能授权 17/18。
- 非权威吊点/绳长不能产生正式斜拽角度报警。
- `CargoSwingStatus` schema 不变。

## Windows 验证

- `scripts/regression/run_static_contracts.py`: PASS
- YAML duplicate key: PASS
- repository integrity: PASS（919 个 tracked source/config 文件）
- Cargo safety E2E 静态链: PASS
- Python compileall: PASS
- Python unittest: PASS（43/43）
- `git diff --check`: PASS
- C++/ROS/Catkin: `NOT_RUN_REQUIRES_UBUNTU`
- 真实 Bag/服务器: `NOT_RUN_REQUIRES_UBUNTU_AND_SERVER`

## Ubuntu/服务器验收重点

1. LOADED 后立即观察主 fused box：正式厚度未冻结时应为橙色 Pending 框且尺寸非零。
2. `CargoBottomEstimate.valid=false` 时可有 `source_detail=PENDING_*` 和非零角点；不得据此
   授权 14 或删除点。
3. 包络来源或 lifecycle 切换后，Pending obstacle confirmation 必须从 1 重新累计。
4. `RETIRED_FORMAL_SHAPE` 且无身份点时，
   `pending_warning_identity_authorized=false`，最终不得出现 17/18。
5. 正式冻结后尺寸应来自多帧实测 shape，不应被 4x3 Pending 默认框放大。
6. EMPTY + LiDAR conflict 时仍保持非 CLEAR；稳定 EMPTY 后 lock state 可退出
   `CLEAR_WAIT_REARM`，下一次 LOADED 创建新 lifecycle。
