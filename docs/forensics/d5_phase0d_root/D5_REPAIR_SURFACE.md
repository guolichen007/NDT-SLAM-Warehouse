# D5 Repair Surface（只描述层，不写算法）

## FIRST_REPAIRABLE_ROOT

```text
C2_COMPONENT_SPLIT_NOT_GROUPED
```

## Repair Surface 描述

```text
REPAIR_SURFACE = component-to-candidate membership（S4 clustering 的 connectivity 语义）

具体：
  稀疏 cargo 顶面点(z>=1.3) 与 主体 在 S4 Euclidean clustering 下
  的 3D connectivity 在 0.20m tolerance 附近不稳定，
  导致 component 物理内容跨帧切换。
```

## 修复方向（交给 Codex，不在 Ubuntu 实现）

修的不是「Identity 层」也不是「descriptor 层」，而是 **S4 之后的「component 物理身份连续性」**：

- 目标：让「同一真实 Cargo 的顶面点 + 主体」在跨帧时组织成「物理身份稳定」的 support，不因 0.20m tolerance 的临界波动而切换 component 内容。
- 关键约束：**不调 clustering tolerance / min_cluster_size / voxel**（Phase 0B 已锁），而是维护「稀疏顶面点与主体的拓扑连续性」（用当前帧的几何关系，非历史、非绝对 z）。

## 明确的非修复面（本轮证据排除）

```text
- 不修 CargoPhysicalIdentityAuthority（它正确 fail-closed）
- 不修 descriptor 算法（C5 排除）
- 不修 identity association gate 语义（C6 排除）
- 不调 clustering 参数（0.20m / min=10 / voxel 0.05m）
- 不引入 temporal classifier / static gate / 绝对 z 判断
```

## 消费者安全约束（沿用 Phase 0B consumer graph）

任何「component 物理身份连续性」的修复，recovered/重组的点只能进入 **DETECT_CONTINUITY**（identity 统计），**绝不**进入 EXACT_OWNERSHIP / STATIC_CONFLICT / GEOMETRY（self-removal 安全）。见 D5_CONSUMER_GRAPH.md。
