# B3B — Cargo Wrong-Lock Authority 审计

日期：2026-08-21
性质：纯取证。未改产品算法。只回答「wrong low candidate 为什么有资格进入 LOCKED」。

---

## 0. 固定字段

```
BASE_PRODUCT_SHA=df45c397cf02dbd3dfbf4d7f7549929f58482366
B3A_FORENSIC_SHA=b07d2ca5f88892f656b12f09f95a117f032351df
BRANCH=investigate/cargo-ranker-score-lineage-v1

PRODUCT_BEHAVIOR_CHANGED=NO  CARGO_PRODUCT_FIX_IMPLEMENTED=NO
OBSTACLE_FRAGMENTATION_FIXED=NO  YAW_PRODUCT_WORK=PAUSED  FIELD_READY=NO
```

---

## 1. LOCKED promotion 完整链（逐项）

### 1.1 selected winner 的产生（`ndt_slam.cpp` detectCargoAroundOdomAnchor）

```
14088  scoreCargoCandidateIdentity(descriptor, identity_context)   // 每 component 打分
14112  rankCargoCandidateIdentityScores(component_scores)          // rank = identity + 0.25*overall
14118  selected_identity = candidate_ranking.top1
14126  result.selected_candidate_id = selected_hypothesis_index      // ← winner
14133  result.candidate_score_margin = candidate_ranking.margin     // top1_rank - top2_rank
```

- **selection winner** = ranker top1，由 B3A 已证明的 `scoreCargoCandidateIdentity` 产生。
- **score margin** = `candidate_ranking.margin`（top1_rank − top2_rank），在无.bag wrong 帧 P50=0.428（B2）——明确选择非噪声。

### 1.2 predicted reference 来源（`ndt_slam.cpp:13762-13845`）

`identity_context.predicted_track_valid` / `predicted_center` 三分支：

| b3a_predicted_source | 条件 | predicted_center |
|---|---|---|
| 1 LOCKED | `cargoTrackRetained() && live_pose.valid && locked_shape.valid` | `live_pose.center_base + live_pose_velocity_base * decayed_dt` |
| 2 PROVISIONAL | `!provisional_observations.empty()` | `provisional_observations.back().center` |
| 3 RETIRED | retired signature fresh | `retired_cargo_center_base_` |

→ 这是 B3A 证明的 self-reference 源头：LOCKED 后 predicted 直接来自 locked live_pose。

### 1.3 状态机路径（`UpdateHookCargoLock`，`ndt_slam.cpp:15398` 起）

```
EMPTY
  → CANDIDATE            (15400-15402)  candidate_policy.allow_candidate && isStrongDetection
  → GEOMETRY_CONFIRMING  (15457-15459)  candidate_spatially_consistent && candidate_progress_count>=2
      15470  observation.component_id = det.selected_candidate_id   // ← winner 冻结进 provisional window
      15495  provisional_observations.push_back(observation)
  → LOCKED               (15926-15980)  三重门槛全部满足
      15979  state = LOCKED
      15980  lock_authority_source = physical_authority.source
      16020  updateLiveCargoPose(...)   // ← live_pose 成为 LOCKED reference
```

### 1.4 GEOMETRY_CONFIRMING→LOCKED 三重门槛（`ndt_slam.cpp:15926-15928`）

```cpp
if (!hook_lock_.provisional_summary.formal_lock_allowed ||   // ① 几何窗口收敛
    !candidate_policy.allow_lock ||                          // ② 候选策略放行
    !physical_authority.allowed) {                           // ③ 物理锁权限放行
    break;   // 三缺一都不 LOCK
}
```

## 2. 关键耦合点：gravity_loaded 双路径直接放行

### 2.1 `evaluateCargoPhysicalLockAuthority`（`cargo_track_policy.cpp:181-228`）

```cpp
188  if (gravity_loaded && input.signal_role != HookLoadSignalRole::DISABLED) {
189      decision.allowed = true;
190      decision.source = CargoLockAuthoritySource::GRAVITY_LOADED;
191      decision.reason = "gravity_loaded";
192      return decision;   // ← 早返回，无任何 candidate-specific identity 检查
193  }
```

`gravity_loaded = gravity_valid && gravity_state == LOADED`。**只要「有货」就 `allowed=true`**，不校验「当前 ranker 赢的 candidate 是否就是那件货」。

### 2.2 `evaluateSuspendedCargoLock`（`hook_load_evidence_policy.cpp:102-141`）

```cpp
112  if (input.role == REQUIRED) { allow_candidate = allow_lock = gravity_loaded; return; }
126  if (gravity_loaded) { decision.allow_lock = true; reason="auxiliary_gravity_support"; return; }
```

两条路径（REQUIRED / AUXILIARY）下 `gravity_loaded` 都直接 `allow_lock=true`。

### 2.3 结论

```
WRONG_LOCK_AUTHORITY_SOURCE=GRAVITY_LOADED
GRAVITY_EXISTENCE_AND_IDENTITY_COUPLED=YES
WRONG_LOCK_PROMOTION_REASON=gravity_loaded（双路径：allow_lock=true 且 allowed=true，均不校验 candidate 身份）
```

gravity LOADED 同时满足门槛②（allow_lock）和门槛③（physical_authority.allowed），且门槛①（formal_lock_allowed）只检查几何窗口收敛（size/orientation 稳定），**三层门槛无一检查「selected candidate 的物理身份是否独立可证」**。

## 3. 非 gravity 的 authority 路径（candidate-specific 但被选中后才生效）

`evaluateCargoPhysicalLockAuthority` 里 gravity 之后的两条：

| source | 条件 | 输入 |
|---|---|---|
| LIFT_FROM_ORIGIN | `lift_from_origin_m >= 0.25 && lift_confirm_frames >= required` | `lift_from_origin_m` / `lift_confirm_count`（来自 LiftOriginBinder） |
| LIDAR_SUSPENDED | `ground_clearance_m >= 0.30 && suspension_confirm_frames >= required` | `ground_clearance_m` / `suspension_confirm_count` |

**关键**：这两条虽然名义上是「candidate 的物理证据」，但它们的输入（`lift_from_origin_m`、`ground_clearance_m`）都是**对已 selected top/component 计算**的：

- `ground_clearance_m`（`ndt_slam.cpp:15500-15504`）= `bottom.bottom_z_base - det.ground_z`，bottom 来自 selected component。
- `lift_from_origin_m`（`cargo_lift_origin_binder.cpp:309-310`）= `input.current_top_z_map - origin.top_z95_map`，`current_top` = 已 selected 的顶面。

→ 即「selected 错 → bottom/current_top 错 → suspension/lift 证据错」的循环，与用户 Section 7 顾虑一致。**LiftOrigin 的 origin 绑定本身是 per-candidate 的**（`candidateScore` 在 candidates 里选，L249-262），但 `lift_delta` 测量用单值 `current_top_z_map`（selected top），非 per-candidate。

## 4. 何时 live_pose 成为 LOCKED reference

- `updateLiveCargoPose`（`ndt_slam.cpp:14845`）：LOCKED 转换时调用（16020），用 `summary.median_center` 初始化 `measured`，`hook_lock_.live_pose.valid = true`（15045）。
- 之后 `cargoTrackRetained()`（14840）= `state==LOCKED || state==LOST_HOLD`，配合 `live_pose.valid && locked_shape.valid` → `b3a_predicted_source=1 (LOCKED)`。
- 即：**LOCKED 一旦发生，live_pose 就锁定 winner 的 center（含错误的 z），并成为后续 predicted_center 的自激 reference**（B3A 已证）。

## 5. 审计结论汇总

```
WRONG_LOCK_AUTHORITY_SOURCE=GRAVITY_LOADED
WRONG_LOCK_PROMOTION_REASON=gravity_loaded（allow_lock + allowed 双放行）
WRONG_LOCK_SCORE_MARGIN=0.428（无.bag wrong 帧 P50，B2）
WRONG_LOCK_GRAVITY_STATE=LOADED（gravity_valid && LOADED）
WRONG_LOCK_INDEPENDENT_IDENTITY_EVIDENCE=NONE（三层门槛无一要求 candidate-specific 独立物理证据）
GRAVITY_EXISTENCE_AND_IDENTITY_COUPLED=YES
```

**核心答案（回答「wrong low candidate 为什么有资格进入 LOCKED」）**：
不是 ranker 权重、不是几何门槛——而是**锁权限把「重力存在（有货）」等价为「当前 ranker winner 就是货物身份」**。`gravity_loaded` 在 role≠DISABLED 时，同时通过 `evaluateSuspendedCargoLock.allow_lock` 和 `evaluateCargoPhysicalLockAuthority.allowed` 两道门槛，二者都不校验 selected candidate 的物理身份。rank 一旦选错（B3A 的 point_support 起源），错误的 winner 就带着「有货」的 gravity 授权合法进入 LOCKED，随后 live_pose 锁定错误位置，predicted_center 自激。
