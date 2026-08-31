# D5 Current-Frame Recovery Signal 分析

> 数据源：大件 forensic replay（6d1b2d4 + diagnostic-only trace），4528 个 weak fragment。
> oracle window（起升段）= 1778303489.58 ~ 1778303538.98（大件 oracle 正样本）。

## 1. D5 机制的精确数据画像

被 clustering 丢弃的 weak fragment 分两类（按 `oracle_high_surface`，即 fragment 内是否有 z>=1.3 点，纯 bag 标记）：

| 类 | n | center_z p50 | xy_overlap p50 | vertical_sep p50 | 3d_dist p50 |
|---|---|---|---|---|---|
| HIGH（顶面/高处） | 1298 | 2.04 | **1.00** | 0.61 | 0.24 |
| LOW（贴地/侧面） | 3230 | 0.73 | 0.26 | 0.35 | 0.26 |

**核心机制（精确）**：cargo 顶面碎片与 primary（主体）在 **XY 上完全重叠（overlap=1.0），XY 距离仅 0.035m，但 z 差距 0.6m**。3D 欧氏距离 `sqrt(0.035² + 0.6²) ≈ 0.6m`，远超 `component_cluster_tolerance_m=0.20m`，因此被 EuclideanClusterExtraction 切断。**这是纯 3D 欧氏 clustering 对「XY 重叠但 Z 大空洞」的点云的结构性失配**，不是 tolerance 太小。

## 2. 2.5D 假设的反事实验证（任务书第 9 节）

候选 current-frame 信号：**「XY 投影完全重叠 + 相对 z 差距大」**（不用绝对 z）。

### 正样本（cargo 顶面）满足该关系吗？

**满足**：起升段内 HIGH fragment `xy_overlap p50=1.00`、`vertical_sep p50=0.52`。

### 负样本（静态结构 / 障碍）也满足吗？—— 分两段

| 阶段 | LOW 的 xy_overlap p50 | 结论 |
|---|---|---|
| 起升段内（cargo 吊起，主体离地） | **0.00**（XY 与主体分离） | 2.5D 假设**区分有效** |
| 地面段（cargo 在地面，主体贴地） | 高（与地面 cargo XY 重叠） | 2.5D 假设**失效** |

全局联合信号「XY overlap>0.9 且 vs>=0.45」的命中：
- HIGH（真 cargo 顶面）命中 841/1298 = **65%**
- LOW（贴地静态/侧面）误命中 612/3230 = **19%**

**19% 误判率不可接受**（任务书第 15 节：宁愿继续 Code30 也不能把障碍物吃进 cargo）。

## 3. 绝对 z 也不能救

`oracle_high_surface`（z>=1.3）把 fragment 分成 HIGH/LOW，但 HIGH 内部混有：
- **F1 TRUE_CARGO_SURFACE**（起升段内高点）488 个
- **F2 INDEPENDENT_STATIC_HIGH**（起升段外高点，货架/结构）810 个

两者 `center_z p50` 都是 2.0~2.1（**z 高无法区分 cargo 顶面 vs 货架高点**）。这直接印证任务书第 11 节「禁止用 z>1.3 判断 Cargo」。

## 4. 无法分类的 F4

F1-F4 分类结果：

```
F1_TRUE_CARGO_SURFACE        488   起升段内 + 高点
F2_INDEPENDENT_STATIC_HIGH   810   起升段外 + 高点（货架/结构）
F3_EXTERNAL_GROUND_OR_SIDE  1713   贴地 + XY 无重叠
F4_UNKNOWN                  1517   贴地 + XY 重叠（无法区分 cargo 侧面 vs 地面静态）
```

**F4（1517 个）是核心难题**：这些 fragment 贴地（z<1.3）且 XY 与 primary 重叠，current-frame 无法区分「cargo 侧面点」和「地面静态结构」。它们本质上是「cargo 主体被 clustering 从主体切下的侧面/底面点」与「独立地面物体」的混合。

## 5. 最终判定

```
ISOLATED_CURRENT_FRAME_RECOVERY_SIGNAL = NOT_FOUND
  没有单独的 current-frame 物理特征能在不使用历史/绝对 z 的情况下
  安全区分 F1(cargo 表面) 与 F2/F3(独立静态/障碍)。

2P5D_CURRENT_FRAME_ASSOCIATION = UNSAFE
  「XY 投影重叠 + 相对 z 差距」在起升段有效（XY overlap 1.0 vs 0.0），
  但在地面段失效（贴地静态也与地面 cargo XY 重叠），
  全局误判率 19%，不可作为无条件恢复信号。

结论类型 = B + C
  B：没有单独安全的 current-frame signal，不能直接恢复。
  C：需要双通道 evidence —— DETECT_CONTINUITY_SUPPORT ≠ EXACT_CARGO_OWNED_POINTS。
```

## 6. 给 Codex 的方向（只描述合同，不落产品 patch）

恢复 cargo 顶面点**只能**用于 continuity（identity 的 extent/z 统计），**绝不能**用于 exact ownership（self-removal / map mutation）。这是本轮 consumer graph 审计（D5_CONSUMER_GRAPH.md）的唯一安全出口：

- `DETECT_CONTINUITY` 通道：允许 recovered 点流入（identity 才能 VALIDATED）
- `EXACT_OWNERSHIP` / `STATIC_CONFLICT` / `GEOMETRY` 通道：recovered 点**绝不**流入

任何「把 recovered 点直接塞回 authoritative component」的实现都会被 consumer graph 证明不安全，直接否决。
