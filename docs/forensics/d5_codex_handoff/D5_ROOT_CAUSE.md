# D5 Root Cause — 冻结根因

## 冻结结论

```text
BASE_SHA = 6d1b2d4c8f4a1f129c4c23b8f02c05e503589704
FIRST_BAD_STAGE = D5_COMPONENT_CLUSTERING
CONSUMER_GRAPH_EXACT_OWNERSHIP_LEAK = YES
```

## 完整故障链

```text
detectCargoAroundOdomAnchor 的 EuclideanClusterExtraction(0.20m, min=10)
→ cargo 顶面点与主体在 XY 完全重叠、但 Z 差 0.6m+（3D 距离 0.6m > 0.20m）
→ 顶面点被 3D 欧氏 clustering 切成 <10 点碎片丢弃
→ high component z95 = 0，component extent/vz 时整时碎
→ group descriptor extent/vz 抖动（union_points_base 从 component 派生）
→ IdentityAuthority 正确 fail-closed（EXTENT_GATE/Z_GATE 拒绝，lift 无法连续 4 帧）
→ identity 无法 VALIDATED
→ safety 输出 v6_authority_fail_closed → code 30 全程
```

## 精确机制（本 forensic 决定性证据）

| 证据 | 值 |
|---|---|
| 顶面碎片 XY 与 primary 的 overlap | p50 = **1.00**（完全重叠） |
| 顶面碎片 XY 与 primary 的距离 | p50 = **0.035m**（极近） |
| 顶面碎片与 primary 的 z 差距 | p50 = **0.61m** |
| 顶面碎片与 primary 的 3D 距离 | p50 = **0.24m**（略超 0.20m tolerance） |

**结论**：不是 tolerance 太小，也不是聚类算法 bug，而是「cargo 顶/侧面在 XY 上属于同一 footprint，但 Z 上存在约 0.6m 的空洞」这一物理事实，与「3D 欧氏 clustering」的结构性失配。

## 配置矛盾已澄清（重要）

```text
RUNTIME_VOXEL_LEAF_M = 0.05（源码硬编码 ndt_slam.cpp:15454，非 0.20）
RUNTIME_COMPONENT_CLUSTER_TOLERANCE_M = 0.20（yaml）
RUNTIME_MIN_CLUSTER_SIZE = 10（yaml）
```

历史报告「voxel leaf = 0.20m」是笔误。见 D5_RUNTIME_CONFIG.json 五处交叉确认。

## 为什么不能简单修复（给 Codex）

1. **one-gap（一个 voxel 缺口 ≈0.10m）几何上无法连接**：顶面与主体的 z 差 0.6m，XY 虽然重叠但 3D 距离 0.6m，远超任何「一个 voxel 缺口」的语义。
2. **绝对 z 无法区分**：cargo 顶面（z≈2.0）与货架高点（z≈2.0）的 z 分布重叠，`z>1.3` 无法区分 F1 与 F2。
3. **XY overlap + z gap 无法无条件恢复**：地面段贴地静态也与地面 cargo XY 重叠，全局误判 19%。
4. **recovered points 会泄漏进 exact ownership**：consumer graph 证明（D5_CONSUMER_GRAPH.md），直接塞回 component 会把独立障碍标记成 cargo-owned 并从地图删除。

## 安全出口（唯一）

恢复点只能用于 **DETECT_CONTINUITY**（identity extent/z 统计），绝不能用于 **EXACT_OWNERSHIP / STATIC_CONFLICT / GEOMETRY**（self-removal / map mutation）。这是「continuity support ≠ exact ownership」双通道合同的根源。

## 相关下游状态

```text
D1_STATUS = POSSIBLE_D5_DOWNSTREAM（ROI clipping 28% 是无.bag 特有，D5 循环下游，本轮不修）
G11_2_STATUS = OUT_OF_SCOPE_FOR_D5（obstacle fragmentation 是 later risk，本轮不碰）
```
