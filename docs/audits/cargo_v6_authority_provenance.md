# Cargo V6 Physical Authority Provenance Audit

> 阶段 2+3 交付物。只读代码审计，不改产品代码。BASE_SHA=`b9f66ae`。
> 目标：为 FINAL AUTHORITY CLOSURE 建立「哪些字段是独立机器证据、哪些是 candidate-derived」的 source graph。

## 结论速览

```
HOOK_LIVE_POSE_INDEPENDENT       = NO   (candidate-derived)
HOOK_VELOCITY_INDEPENDENT        = NO   (candidate-derived)
PROVISIONAL_VELOCITY_INDEPENDENT = NO   (candidate-derived)
TROLLEY_MOTION_INDEPENDENT       = NO_TOPIC (bag 无 /crane/base_motion_state)
HOIST_MOTION_INDEPENDENT         = NO_TOPIC (bag 无 /crane/hoist_motion_state)
HOOK_ANCHOR_INDEPENDENT          = NO_TOPIC (config 常数 hook_anchor_source="config")
LOAD_STATE_INDEPENDENT           = YES  (/gravity 荷重计 → HookLoadState)
STARTED_LOADED_INDEPENDENT       = YES  (load edge, 来自 HookLoadState 跳变)
CRANE_BRIDGE_MOTION_INDEPENDENT  = YES  (CraneMotionEKF / raw_physical_pose, 来自 NDT 定位)
LIFT_CONFIRMED_INDEPENDENT       = NO   (candidate z95/top_z delta vs frozen baseline)
FORMAL_LIFT_CONFIRMED_INDEPENDENT= NO   (派生自 lift_confirmed)
MULTI_CANDIDATE_PHYSICAL_EVIDENCE= YES  (authority 层 per-group histories_) / NO (provisional 单 winner)
```

## 一、Identity ranker 数据源（cargo_track_policy.cpp）

`scoreCargoCandidateIdentity`（行 342）的评分项 provenance：

| 分项 | 数据源 | 判定 |
|---|---|---|
| `hook_distance_score` (0.60) | `context.hook_center` = `anchor` = `getCargoAnchorXY()` = **config 常数** `(anchor_x, anchor_y)`（ndt_slam.hpp:1813） | 非 self-ref，但弱（固定点，非真实 hook 位置） |
| `point_support_confidence` | `point_count / strong_point_count`，candidate 点云 | candidate-derived；b9f66ae 已移出 prelock ranking |
| `suspension_confidence` (0.27) | `candidate.suspension_evidence`（candidate 上的 flag） | candidate-derived |
| `shape_confidence` (0.13) | orientation_confidence + 长宽比 | candidate-derived |
| predicted 分支 `predicted_center_score/overlap/motion` | `predicted_center` = `hook_lock_.live_pose`（candidate-derived） | candidate-derived self-ref（LOCKED/RETIRED 分支） |

**关键判定：整个 identity ranker 除 hook_distance（固定 anchor 距离）外，无任何独立物理证据。**
prelock 排名本质是「距固定 anchor 最近 + 形状好 + suspension flag」，无法区分 true Cargo 与 dense static。
`nearest hook`（固定 anchor 距离）不得单独拥有 owner authority（与用户阶段 4 原则一致）。

## 二、live_pose / velocity（ndt_slam.cpp）

`updateLiveCargoPose`（行 17090）：
- `measured = det.center_base`（行 17096），`det` = `HookCargoDetection`（当前 selected candidate）。
- `measured.z()` 来自 `evaluateCargoTopSurfaceHeight`（candidate z95 / bottom / frozen height）。
- `hook_lock_.live_pose` = 对 `measured` 的滤波（`updateCargoLivePoseStep`）。
- `hook_lock_.live_pose_velocity_base` = 对 candidate center 的速度滤波。

**判定：live_pose / live_pose_velocity_base / live_pose_predicted_base 全部 candidate-derived。**
拿它们做 co-motion 验证 candidate 本身 = 第二种 self-reference（用户风险 1 坐实）。

## 三、独立机器运动源（唯一可用的独立证据）

### 3.1 荷重计 /gravity（INDEPENDENT）
- `hook_load_state_node.cpp`：`/gravity`（std_msgs/Float32 电压）→ `HookLoadState`（EMPTY/LOADED/UNKNOWN，阈值 low=1.90V high=2.10V）。
- bag 四包均含 `/gravity` topic（无/有/长件/大件）。
- 只给 load existence（LOADED/EMPTY + 跳变 edge），不给 identity 或 position。

### 3.2 天车桥架定位 CraneMotionEKF / raw_physical_pose（INDEPENDENT）
- `crane_motion_ekf.hpp:83`：`status.velocity`（Eigen::Vector2d）暴露桥架速度。
- `ndt_slam.cpp:24770`：`physical_input.raw_position = raw_physical_pose.translation().head<2>()`（SLAM/NDT 定位，非 cargo candidate）。
- `cargo_physical_motion_estimator_` 据此判 STATIONARY/MOVING。
- **但注意**：这是桥架（大车）在 map frame 的运动，不是小车（trolley）运动。

### 3.3 trolley / hoist / hook anchor position（NO_TOPIC）
- config 引用 `/crane/base_motion_state`、`/crane/hoist_motion_state`、`/crane/hook_anchor_base`，但 **bag 四包只有 /gravity /rs_201 /rs_203 /hikcamera**，这些 topic 都不存在。
- `hook_anchor_source: "config"`（常数，非 topic）。
- 结论：**bag 内无独立 trolley/hoist/hook position 源**。co-motion 只能用「桥架运动（SLAM 定位）」而非「trolley 运动」。

## 四、lift / formal lift（candidate-derived，风险 2 坐实）

`cargo_physical_identity_authority.cpp`（行 3135-3180）：
- `delta = current_surface_vertical.top_z_base - history->baseline_z95`。
- `baseline_z95` = frozen candidate surface z95；`current_surface_vertical.top_z_base` = current candidate top z。
- `lift_confirmed = (delta >= threshold)` 连续 `requiredFrames` 帧。

**判定：lift_confirmed / formal_lift_confirmed / precluster_lift_confirmed 全部 candidate-derived（z95/top_z 比较）。**
不能反过来用它证明「该 candidate 是 Cargo」——那就是 z95 自证。

## 五、temporal candidate evidence 架构（阶段 3）

### 5.1 provisional 窗口（ndt_slam.cpp）= 单 winner
- `hook_lock_.provisional_observations.push_back(observation)`（行 18097）：每帧只 push 一个 selected observation。
- `provisional_scores / lift_confirm_count / suspension_confirm_count / lift_from_origin_m / provisional_velocity_base` 都只跟踪 top-1 winner。
- `MULTI_CANDIDATE_PROVISIONAL_EVIDENCE = NO`。

### 5.2 authority 层 = per-group 多候选
- `cargo_physical_identity_authority.hpp:720`：`std::vector<History> histories_`。
- `struct History`（行 567）每 group 独立持有：`baseline_z95 / lift_confirm_count / lift_confirmed / validation_stamp_sec / frozen_preload_footprint / association_ambiguous`。
- `groupCargoPhysicalCandidates()`（行 246）构建 frame-local physical groups（多候选）。
- `validated_history_id_`（行 726）= 最终 validated owner 的 history id。

**判定：physical identity confirmation 已有 per-group 多候选基础设施（histories_）。**
MULTI_CANDIDATE_PHYSICAL_EVIDENCE = YES（authority 层），NO（provisional 单 winner 层）。
Closure 应在 authority 层对多个 group 建立/比较 physical evidence，而非依赖 provisional winner。

## 六、给 FINAL AUTHORITY CLOSURE 的可用独立证据清单

真正允许作为 PRELOCK physical identity confirmation 的独立证据（经 provenance 审查）：

1. **荷重计 load edge**（`/gravity` EMPTY→LOADED 跳变，`node_started_loaded`）：独立 load existence。产生「load 开始时刻」这个物理时间锚点。
2. **桥架运动兼容性**（CraneMotionEKF velocity / raw_physical_pose）：独立机器运动。当桥架移动时，true Cargo 在 base_link 内近似静止（悬吊共移），static 障碍在 base_link 内扫过（需要 frame/timestamp/dt 对齐）。
3. **per-group temporal physical consistency**（authority 层 histories_）：每个 group 独立积累 baseline/lift/footprint，跨帧一致性。
4. **geometry validity**：candidate 几何合法性（shape/aspect/footprint）。

禁止作为 PRELOCK authority 的（candidate-derived / 循环）：
- hook_distance（固定 anchor 距离）单独 owner
- point_count（observability，b9f66ae 已移除）
- live_pose / velocity / provisional_velocity（candidate center 滤波）
- lift_confirmed / formal_lift_confirmed（z95 自证）
- predicted_center / overlap（self-ref）

## 七、关键风险复核（用户四问）

1. **hook_lock_.live_pose/velocity 是否独立机器运动源？** 否。直接/间接来自 selected candidate center（det.center_base）→ 拿它做 co-motion = 第二种 self-reference。
2. **formal_lift_confirmed / started_loaded 是否循环 authority？** started_loaded 独立（load edge）；formal_lift_confirmed 循环（z95 自证），不得反过来证明 candidate。
3. **temporal history 是否只维护 winner？** provisional 层只维护 winner（MULTI=NO），authority 层 per-group（MULTI=YES）。Closure 用 authority 层。
4. **Bottom 是否消费 prelock rank top-1 几何？** 待阶段 6 专项审查（hook_fixed_cargo_.z95 → Bottom 的 lifecycle 绑定）。

## 八、架构含义（供阶段 4 离线设计）

- 独立机器证据**只有两个**：荷重计（load edge）+ 桥架定位（motion）。没有 trolley/hoist/hook 编码器。
- 因此「co-motion with trolley」必须改为「co-motion with bridge (SLAM 定位)」+「load edge 时间锚点」。
- 桥架运动在 base_link 内对「悬吊货物（静止）vs 地面静态（扫过）」有判别力，但**不能区分「小车移动导致的货物 base_link 位移」vs「静态」**（缺 trolley 编码器）。
- lift 证据（z95 delta）虽然 candidate-derived，但作为 per-group 的「起升过程单调性」物理特征仍可用于区分，前提是**不与 z95 自证身份**，而是配合 load edge 的「load 开始时刻」做时间对齐确认。

## 九、Bottom authority 专项审查（阶段 6）

Bottom 消费链：`hook_fixed_cargo_` → `observation.current_top_z_base` → `cargo_bottom_fusion_.update` → `legacy_formal_height` → `legacy_self_removal_authorized`。

关键事实（BASE_SHA=`b9f66ae`）：
1. `hook_fixed_cargo_ = detectCargoAroundOdomAnchor(...)`（ndt_slam.cpp:5783）= **selected candidate（winner）**。
2. `observation.current_top_z_base = hook_fixed_cargo_.z95`（24995）仅在 `detection_is_current` 时设置。
3. **LOCKED 分支覆盖**：`if (active_track)`（24997）内用 `extractCargoVerticalEvidence`（25029）从 raw range cloud 在 locked footprint 内重测 top，覆盖 winner z95（25035）。footprint 来自 `hook_lock_.locked_shape` + `hook_lock_.live_pose`（25002）。
4. Bottom 融合无条件调用 `cargo_bottom_fusion_.update(observation)`（25229），但 `observation.track_valid = active_track`（24966）。
5. Bottom→safety 授权链 `legacy_self_removal_authorized = active_track && formal_authorized && ...`（25249）被 `active_track`（LOCKED）门控。

**判定：**
- `BOTTOM_PRELOCK_GEOMETRY_ALLOWED = NO`（safety 授权被 active_track 门控；prelock 时 track_valid=false，Bottom 不产 formal height）。
- 但 **Bottom 继承 LOCKED 的 winner 几何**：`locked_shape` + `live_pose` 是「当时被 lock 的 winner（可能为 wrong static）」的 frozen shape/center。
- 因此 Bottom authority 的正确性**完全取决于 LOCKED 是否只发生在 VALIDATED physical owner**。这正是 FINAL AUTHORITY CLOSURE 要修的上游：VALIDATED → FORMAL LOCK 才允许 Bottom 消费 fresh geometry。

**Closure 要求（阶段 6 落地）：**
- PRELOCK/PENDING：Bottom 不得 CLEAR（fail-closed，现状已由 active_track 门控满足）。
- VALIDATED/LOCKED：Bottom 消费真正 physical owner 的 fresh geometry（current_top 已用 raw cloud 重测，满足；footprint 用 locked_shape，需保证 locked 只在 VALIDATED 后发生）。

报告结束。数据源行号均为 BASE_SHA=`b9f66ae`。
