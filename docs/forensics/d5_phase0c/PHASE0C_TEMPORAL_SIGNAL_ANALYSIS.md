# Phase 0C Temporal + Static Signal Analysis

> 数据源：无.bag forensic replay（NDT_D5_FRAGMENT_FORENSIC=1 + NDT_PHASE0C_FORENSIC=1，empty_map 冷启动）。
> oracle window（safe-over 起升段）= 1787034958.42 ~ 1787035076.80。

## 1. Mature Static 状态（决定性负面）

```text
static cells 总数       = 2622
clean_map_confirmed     = 2622
temporally_mature       = 0
```

**empty_map 冷启动 + 无.bag 的 200s 采集里，static evidence 没有达到 temporally_mature 状态。**

原因（`isTemporallyMatureLocked` 判定）：
```text
temporally_mature 需要 clean_map_confirmed
  && map_generation == working_generation_
  && consecutive_observation_count >= 4
  && consecutive_stable_duration_sec >= 1.0
```

`clean_map_confirmed` 在 empty_map 冷启动里通过 clean-map build 建立（2622 cell），但 `consecutive_observation_count >= 4` 的连续 streak 在 200s 内未累计到（cargo safe-over 运动 + 观测被 ROI/遮挡打断）。

**结论：`mature Static provenance` 这条 discriminant 在当前数据集上无法评估（数据不可用，非 PROVEN 非 UNSAFE）。**

## 2. base/map temporal signal（决定性正面）

### cargo 顶面随 pose 运动（14m 位移）

`z>=1.3` 且 `cluster_label>=0` 的 primary point（cargo 顶面）转 map 后，map 中心随时间：

| t (sec) | map_center | 
|---|---|
| 1787034972 | (-0.72, 0.80) |
| 1787035006 | (0.28, -0.53) |
| 1787035040 | (6.45, -1.14) |
| 1787035074 | (13.17, -2.26) |

**map_center 位移 ≈ 14m，精确匹配 pose 位移（tx 0→13.76, ty 0.07→-2.54）。**

这证明：**cargo 顶面点是真实存在且随天车运动的**，且大部分被 clustering 正确归入 primary component（64% 帧有 high component）。

### static 高点 map 稳定

weak fragment 里 `oracle_high_surface=1` 的 fragment（330 个 F1），map 空间聚类显示 `map_spread_p50=0.25m`（稳定）、`base_spread_p50=0.65m`（随 pose 反向运动）→ **static_like**。

这说明 weak fragment 里的「高处 fragment」是**货架/结构高点（static）**，不是 cargo 顶面。

### 区分度总结

```text
cargo 顶面（primary point, z>=1.3）：map 随 pose 移动 14m（cargo_like）
static 高点（weak fragment, z>=1.3）：map 稳定（static_like）
```

**base/map temporal signal 在离线层面能清楚区分 cargo 与 static（14m vs 0m 的 map 位移）。**

## 3. 关键限制

1. **temporal signal 是跨帧信号**：它需要「同一 physical object 的多个 frame 观察」才能判断「移动（cargo）vs 静止（static）」。而「跨帧关联」正是 identity association 在做的事（base frame representative center，0.30m 阈值），当前因 representative center 抖动（D5 碎片化下游）而失败。

2. **weak fragment 里的高处点是 static，不是 cargo 顶面**：这意味着「恢复 weak fragment」并不会恢复 cargo 顶面（cargo 顶面在 primary component 里）。D5 的「cargo 顶面碎片化」在大件更严重（47% FULL），在无.bag 较轻（64% 有 high component）。

3. **mature static 不可用**：无法验证「mature Static discriminant」能否排除错误 support（这条 signal 在当前数据集上未触发）。

## 4. 候选 signal 评估（任务书第 20 节）

```text
A. Mature Static provenance     = NOT_AVAILABLE（temporally_mature=0）
B. Base-frame consistency       = 部分支持（cargo base 稳定，static base 随 pose 反向移动）
C. Map-frame world-static       = 支持（static map 稳定 0m，cargo map 移动 14m）
D. RAW owner-column vertical    = 未单独评估（需 extractCargoVerticalEvidence 结果）
E/F/G 组合                     = 无法形成（A 不可用）
```

## 5. 结论

```text
BASE_ATTACHED_SIGNAL   = PROVEN（cargo 顶面 base 稳定，随天车）
WORLD_STATIC_SIGNAL    = PROVEN（static map 稳定，cargo map 移动 14m）
TEMPORAL_COMPOSITE_SIGNAL = PROVEN（离线层面 map 位移能清楚区分 cargo/static）

但实时使用受限于：
  - mature static 不可用（无法作 provenance 排除）
  - temporal 需跨帧关联（identity association 是当前 blocker）
  - weak fragment 高处点是 static 非 cargo（恢复 weak fragment ≠ 恢复 cargo 顶面）
```

**`PURE_LIDAR_SAFE_IDENTITY_SUPPORT = INCONCLUSIVE`**：temporal signal 有物理区分度，但「实时安全使用」需要「稳定的跨帧关联」+「可用的 mature static provenance」，两者在当前数据集上都未满足。
