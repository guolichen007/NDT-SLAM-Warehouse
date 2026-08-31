# D5 Support Lineage Root Cause（Phase 0D）

> BASE_SHA = 6d1b2d4；forensic 分支 investigate/cargo-v6-d5-support-lineage-forensic-v1。
> 数据源：大件一次 replay（NDT_PHASE0D_FORENSIC=1），S3 voxel → component → candidate → group → descriptor → identity 全链 lineage。

## 最终结论

```text
FIRST_BAD_SUBSTAGE = S4 component clustering（connectivity 不稳定）
FIRST_REPAIRABLE_ROOT = C2_COMPONENT_SPLIT_NOT_GROUPED
SECONDARY_ROOT = C1_COMPONENT_DROP（5.2%，非主因）
```

## 决定性证据

### 1. S3 cargo 顶面点没有被大量丢弃（否定「C1 顶面点全丢」）

```text
S3 high-oracle voxel (z>=1.3) 总数 = 108585
  PRIMARY_ACCEPTED = 94.8%  (102924)
  WEAK_REJECTED   =  3.5%   (3777)
  UNASSIGNED      =  1.7%   (1884)
```

**94.8% 的 cargo 顶面点被 clustering 正确归入 primary component，只有 5.2% 被丢弃。** 之前 Phase 0B「10~16 个顶面体素被切碎丢弃」的描述只覆盖少数 FRAGMENT 帧，不是主要机制。

### 2. S4 component 的物理内容跨帧不稳定（核心）

```text
component_id=0 的 center 跨帧 step > 0.5m 的帧 = 122/373 (33%)
component_id=0 的 high_oracle_count 跨帧 0 ↔ 11 剧烈切换
```

component_id 是帧局部的，但「component 0 的物理内容」跨帧剧烈变化：
- 有时是「cargo 主体 + 顶面点」（high_oracle=11，center 在 cargo 位置）
- 有时是「低处结构」（high_oracle=0，center 在另一个位置，跳 1.7m）

**这就是 S4 clustering 的 connectivity 不稳定：顶面点（稀疏，z>=1.3）与主体（密集，低处）的 3D 距离在 0.20m tolerance 附近波动，导致它们有时连成一个 component，有时切成多个 component，component 物理内容跨帧切换。**

### 3. R1 vs R2：member id 不变但 center 跳，是「物理内容切换」的假象

```text
相邻帧 member_component_ids 不变: 287 帧，其中 76 帧 (26.5%) center 跳 >0.5m（p50=1.30m）
相邻帧 member_component_ids 变化: 32 帧（10%）
```

R1（member id 不变但 center 跳 1.3m）的真相是：**component_id 是帧局部的，同一 id=0 跨帧对应不同物理内容**，所以「member set 字符串不变」≠「physical support 不变」。

### 4. XY / EXTENT / Z 三类 gate 同根因

```text
stable_anchor_xy_step:  MATCHED p50=0.059m；XY_GATE p50=1.393m；EXTENT_GATE p50=0.871m；Z_GATE p50=0.782m
大件 reject: XY_GATE 97 / EXTENT_GATE 61 / Z_GATE 51 / GAP 60
无.bag reject: EXTENT_GATE 86 / XY_GATE 84 / Z_GATE 62
```

三类 gate 都来自「component 物理内容切换 → union_points 变化 → descriptor XY center/extent/Z 同时跳变」。**SAME_ROOT_CAUSES_XY_EXTENT_Z = YES**。无.bag 与大件同根因（NONE_BAG_ROOT_RELATION = SAME_ROOT）。

## 根因链（最终版）

```text
S3 voxel：cargo 顶面点(z>=1.3) 仍存在（94.8% 被 accepted）
  ↓
S4 EuclideanClusterExtraction(0.20m)：
  顶面点(稀疏,高) 与 主体(密集,低) 的 3D 距离 ≈ 0.6m > 0.20m
  → 两者 connectivity 在 tolerance 附近波动
  → component 物理内容跨帧切换（顶面点有时归 component A，有时归 component B，有时切碎）
  ↓
candidate：selected hypothesis 的 member component 物理内容变化
  ↓
group.union_points_base 变化
  ↓
descriptor：robust XY center 跳 1.3m / extent 跳 / vertical 跳
  ↓
XY_GATE / EXTENT_GATE / Z_GATE
  ↓
NEW_HISTORY → lift continuity 断裂 → identity 无法 VALIDATED
```

## 关键排除

- **C5_DESCRIPTOR_CONSTRUCTION_INSTABILITY 排除**：component 物理内容本身在变（center 跳 1.7m、high_oracle 0↔11），不是 descriptor 算法在稳定输入上不稳定。
- **C6_IDENTITY_ASSOCIATION_SEMANTICS 排除**：identity association 在正确拒绝「物理内容已变化的 component」，不是 gate 语义写错。
- **C1_COMPONENT_DROP 非主因**：只有 5.2% 顶面点被丢弃，不是 descriptor 抖动的主因。
