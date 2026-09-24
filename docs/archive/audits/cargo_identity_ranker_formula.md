# Cargo Identity Ranker — Formula & Reference-Lineage Audit

Audit SHA: `c68606e2d9baa1298eb77c13dc9fa41428ab74c2` (B2 forensic tree)
Product parent: `df45c397cf02dbd3dfbf4d7f7549929f58482366`
Source: `src/ndt_slam/src/cargo_track_policy.cpp` (`scoreCargoCandidateIdentity`, `rankCargoCandidateIdentityScores`) + `ndt_slam.cpp` (identity_context construction).

---

## 0. 排序目标

`rankCargoCandidateIdentityScores` 对每个 valid score 计算：

```
rank = identity_confidence + 0.25 * overall_lock_confidence
```

取 rank 最大者为 top1。

---

## 1. 硬门（reject，不计分）

| 门 | 条件 | reject_reason |
|---|---|---|
| hook containment | `require_hook_containment && !containsHookAnchor(candidate, hook, margin)` | `hook_anchor_outside_candidate_obb` |
| hook distance gate | `gated_distance > dynamic_gate` | `candidate_center_too_far_from_hook_anchor` |
| hook normalized offset | `hook_normalized_offset > maximum_hook_normalized_offset` | `hook_anchor_outside_candidate_central_region` |

- `gated_distance` = `hook_association_residual_m`（若 learned offset valid）否则 `hook_center_distance`。
- `dynamic_gate`（size_aware_hook_gate 时）= `min(configured_base_gate + 0.35*max(L,W) + hook_xy_uncertainty, maximum_dynamic_gate)`；否则 `configured_base_gate`。
- `hook_normalized_offset` = 最大(|hook 在 candidate 局部坐标的归一化 long/short|)。

---

## 2. Score term 逐项

`unitScore(v, limit) = clamp(1 - v/limit, 0, 1)`（v 越小分越高）。
`relativeError(v, ref) = |v - ref| / ref`。

| term | 公式 | REFERENCE_SOURCE | SELECTED_DERIVED | POSE_DEPENDENT |
|---|---|---|---|---|
| `hook_distance_score` | `unitScore(gated_distance, dynamic_gate)` | hook anchor (config 0,0) | NO | NO |
| `point_support_confidence` | `clamp(point_count / strong_point_count, 0, 1)` | candidate point_count | NO | NO |
| `suspension_confidence` | `suspension_evidence ? 1.0 : 0.35` | ground_reference + candidate z_low | NO | PARTIAL (z) |
| `shape_confidence` | `clamp(0.5*orientation_confidence + 0.5*unitScore(aspect=W/L, 1.0), 0, 1)` | candidate footprint (pose-dependent L/W) | NO | **YES** (L/W) |
| `predicted_center_score` | `unitScore(\|center - predicted_center\|, association_radius)` | predicted_center | **YES** (provisional branch) | NO |
| `overlap_score` | `cargoOrientedOverlapRatio(predicted, candidate)` | predicted size/yaw/center | **YES** (provisional branch) | **YES** |
| `motion_confidence` | `= predicted_center_score`（predicted branch） | predicted_center | YES | NO |

---

## 3. identity_confidence 合成（关键分叉）

### 3a. predicted branch（`predicted_track_valid == true`）

```
identity_confidence =
    0.30 * predicted_center_score      # 基于 predicted reference
  + 0.25 * overlap_score               # 基于 predicted size/yaw/center
  + 0.25 * shape_confidence            # 含 predicted L/W/H 误差修正
  + 0.20 * unitScore(|yaw - predicted_yaw|, 0.70)
```

**注意**：此分支中 `point_support_confidence` 和 `hook_distance_score` **完全不参与 identity_confidence**（hook 仅做硬门）。

### 3b. 非 predicted branch（`predicted_track_valid == false`）

```
identity_confidence =
    0.45 * hook_distance_score
  + 0.25 * point_support_confidence
  + 0.20 * suspension_confidence
  + 0.10 * shape_confidence
```

---

## 4. overall_lock_confidence

```
overall_lock_confidence =
    0.35 * identity_confidence
  + 0.20 * shape_confidence
  + 0.15 * candidate.orientation_confidence
  + 0.15 * motion_confidence
  + 0.15 * suspension_confidence
```

---

## 5. Reference lineage（predicted_center 的 producer）

构造点：`ndt_slam.cpp` `detectCargoAroundOdomAnchor` 的 `identity_context`（优先级从高到低）：

| 分支 | 条件 | predicted_center | predicted_size | SELECTED_DERIVED |
|---|---|---|---|---|
| LOCKED | `cargoTrackRetained() && live_pose.valid && locked_shape.valid` | `live_pose.center_base + velocity * decayed_dt` | `locked_shape` (L/W/H) | NO（track 状态） |
| PROVISIONAL | `!provisional_observations.empty()` | `provisional_observations.back().center` | `provisional.size` | **YES**（上一帧 provisional selected candidate） |
| RETIRED | `retired_cargo_signature_valid_` | `retired_cargo_center_base_` | `retired_cargo_shape_` | NO（历史 frozen） |
| NONE | 以上都不满足 | — | — | —（非 predicted branch） |

---

## 6. 静态结论：PRELOCK_SELECTED_DERIVED_EDGES

以下 term 在 PROVISIONAL 分支下全部由"上一帧 selected candidate"派生：

```
PRELOCK_SELECTED_DERIVED_EDGES =
    predicted_center_score  (predicted_center = previous selected.center)
    overlap_score           (predicted size/yaw/center = previous selected)
    shape_confidence        (predicted L/W/H 误差修正 = previous selected size)
    (yaw score)             (predicted_yaw = previous selected yaw)
```

且 predicted branch 的 identity_confidence 中，这四项合计权重 = 0.30+0.25+0.25+0.20 = 1.00。

**假设的自激机制**：
1. 第一帧（无 provisional、无 locked）→ 非 predicted branch → `identity = 0.45*hook + 0.25*point_support + 0.20*suspension + 0.10*shape`。此时更近 hook 且点更多的**错误低 component** 可能赢。
2. 错误低 component 进入 `provisional_observations`。
3. 后续帧 → predicted branch → `predicted_center = previous selected(错误低)` → 错误低 component 因更接近 predicted_center/overlap 持续赢 → **自激锁定**。

（注：这仍是静态推断，需 B3A runtime 数据确认。）
