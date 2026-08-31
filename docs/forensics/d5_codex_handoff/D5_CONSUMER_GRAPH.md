# D5 Consumer Graph — recovered point evidence channel 依赖图

> BASE_SHA = 6d1b2d4c8f4a1f129c4c23b8f02c05e503589704
> 目的：让 Codex 明确「如果把被 D5 clustering 丢弃的弱碎片点恢复回 component，会同时影响哪些 evidence channel」，从而决定必须拆哪些 channel。

## 核心结论（硬门，已冻结）

`recovered points → components[].indices → union_points_base` 会**同时**流入 5 个 consumer channel。任何一个「把 recovered points 直接塞回 authoritative component」的实现，都会把 5 个 channel 一起污染。必须拆。

## 完整数据流

```
voxel_cloud (VoxelGrid 0.05m, 硬编码)
  → EuclideanClusterExtraction(0.20m, min_cluster_size=10)   # D5 切断处
  → cluster_indices (primary, >=10)
  → components[].indices (detectCargoAroundOdomAnchor 15628)
  → component_observations[].points_base (15916-15931)
  → groupCargoPhysicalCandidates → group.union_points_base (473-484)
  → union_points_base
       ├─ [1] DETECT_CONTINUITY   (identity extent/z)         ← 恢复目标
       ├─ [2] GEOMETRY            (resolved footprint)
       ├─ [3] VERTICAL            (supported_top + bottom)
       ├─ [4] EXACT_OWNERSHIP     (self-removal / map mutation) ← 危险
       └─ [5] STATIC_CONFLICT     (mature static provenance)
```

## 五个 channel 的精确依赖

### [1] DETECT_CONTINUITY — 恢复目标（必须让 recovered 流入）
- 位置：`cargo_physical_identity_authority.cpp:484-699`（group descriptor 构造）
- `aggregate_extent` / `robust_x05..y95` / `stable_anchor` 全部从 `union_points` 算（484-532）。
- `physical_vertical_z` = median(supported_tops)，其中 component evidence 的 `selected_points_base = union_points`（556 行）。
- **这是 identity lift-confirm 的输入**。D5 修复的唯一目的就是让这里 extent/z 稳定。

### [2] GEOMETRY — resolved Cargo footprint（被污染）
- 位置：`integrated_cargo_identity_shadow.cpp:79-97`（bindCargoPhysicalGroupEvidence）
- `resolved_geometry.footprint_center_base / size / yaw` 来自 `selected->representative`（= hypothesis descriptor）。
- hypothesis descriptor 的 center/size/yaw 在 detect 层从 `footprint_points`（= components indices 的 xy）算。
- **recovered points 进 indices 会改变 footprint**（可能扩大 footprint 到误归并点）。

### [3] VERTICAL — supported_top + bottom（被污染）
- `supported_top_z` ← `descriptor.physical_vertical_z` ← union_points（bind 70-72）。
- `bottom.points_base` = `integrated_group_evidence_.union_points_base`（ndt_slam.cpp:23416）。
- **recovered points 进 union_points_base 会改变 supported_top 和 bottom 的输入点云**。

### [4] EXACT_OWNERSHIP — self-removal / map mutation（危险，必须隔离）
- 位置：`cargo_v6_authority_adapter.cpp:140-180`（buildCanonicalCargoAuthoritySnapshot）
- `mutation.owner_points.exact_points` 遍历 `input.group.union_points_base`（145-157），用 SourcePointKey 精确匹配。
- 下游：`avoidance_map_mutation.cargo_points`（ndt_slam.cpp:7609）→ `commitKeyFrameWithDynamicFiltering` 里 `cargo_points.owns(point)` 从地图剔除（ndt_slam.cpp:28839）。
- **语义 = 「这些点是 cargo，写图时剔除」**。recovered 点若混入 exact_points，会把邻近独立障碍物标记成 cargo-owned 并从地图删除 → 后续避障「看不见」障碍物。

### [5] STATIC_CONFLICT — mature static provenance（被污染）
- 位置：`cargo_v6_authority_adapter.cpp:233-265`（cargoGroupOverlapsMatureStaticEvidence）
- 遍历 `group.union_points_base`，若命中 mature static cell 则返回 conflict。
- conflict → `would_authorize_map_mutation = tight_geometry_valid && !conflict`（169-171）。
- **recovered points 混入会改变 static conflict 判定**。

## 必须拆的 channel（给 Codex 的硬约束）

```
DETECT_CONTINUITY  ← recovered points 必须流入（否则 identity 不恢复）
EXACT_OWNERSHIP    ← recovered points 绝不能流入（否则误删障碍物）
STATIC_CONFLICT    ← recovered points 绝不能流入（否则误判 provenance）
GEOMETRY           ← 需评估：footprint 是否允许被 recovered 扩大（倾向禁止，用原始 primary 点）
VERTICAL           ← 需评估：supported_top 是 identity 的核心，bottom 是 downstream geometry
```

## 最小拆分建议（不落实现，只给 Codex 方向）

把 `union_points_base` 拆成两个点集：
- `continuity_points`（含 recovered，只喂 DETECT_CONTINUITY / VERTICAL 的 supported_top）
- `exact_owned_points`（不含 recovered，只喂 EXACT_OWNERSHIP / STATIC_CONFLICT / GEOMETRY）

拆分点候选（非 frozen）：
- `CargoPhysicalGroupEvidenceSnapshot`（integrated_cargo_identity_shadow.hpp:45）可加字段
- `CanonicalCargoAuthorityInput`（cargo_v6_authority_adapter.hpp:32）可加字段
- `buildCanonicalCargoAuthoritySnapshot` 里 exact_points 构造（cargo_v6_authority_adapter.cpp:145）改为消费 exact_owned_points

frozen（本 handoff 禁止改动，除非 Codex 单独论证）：
- `cargo_physical_identity_authority.cpp/.hpp`（groupCargoPhysicalCandidates 产出 union_points_base）
- `CargoPhysicalGroupObservation`（frozen 结构）
