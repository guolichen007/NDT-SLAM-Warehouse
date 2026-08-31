# Phase 0C Artifact Audit — 现有产物字段盘点

## 结论

```text
BIG_STATIC_ALIGNMENT_AVAILABLE  = NO
NONE_STATIC_ALIGNMENT_AVAILABLE = NO
```

现有数据**无法**从保存的 snapshot 精确恢复 `source_stamp → pose → static generation` 的逐点对齐，因此必须 replay `无.bag` 补齐。

## 逐 CSV 字段盘点

### 1. integrated_identity_groups.csv（大件/无/有/长件 都有）
- **有**：stamp、frame_group_id、stable_anchor、robust_x05/x95/y05/y95、aggregate_extent、physical_vertical_z、vertical_source、association（XY_GATE/EXTENT/Z/step）、prelift 全字段、lift 全字段、hook_role、physical_cargo_epoch_id、identity_state。
- **缺**：pose_map_base、SourceFrameIdentity 全字段、PoseAuthorityIdentity、mature Static cell 逐点归属、point map xyz。

### 2. detection_pipeline_trace.csv（大件/无）
- **有**：stamp、merged/near/range/roi/hag/voxel/component/hypothesis/selected/core 逐级统计、rejected_xy、ground。
- **缺**：pose、static、map xyz、per-point lineage。

### 3. d5_voxel_lineage.csv（仅大件，Phase 0B 产物）
- **有**：stamp、base xyz、voxel_ix/iy/iz、cluster_label、cluster_size、assignment、fragment_id（point-level lineage，base frame）。
- **缺**：map xyz、static cell 归属、pose、source identity。

### 4. d5_weak_fragments.csv（仅大件）
- **有**：stamp、fragment_id、point_count、base center、bbox、z 分位、nearest_primary_*、xy overlap/gap、oracle_high_surface。
- **缺**：map frame 几何、static 归属、base/map 跨帧轨迹。

### 5. runtime_samples.csv（大件/无/有/长件）
- **有**：yaw_deg、pose_step_*、runtime_static_evidence_epoch/revision/cells/mature_cells（static **全局统计**）。
- **缺**：static cell 的**逐点/逐 cell** 归属（哪个 point 落在哪个 mature static cell）、source-frame 对齐的 pose 全字段。

## 缺口汇总

| 需求字段 | 现有 | 处理 |
|---|---|---|
| pose_map_base（source-frame 对齐） | ✗ | 新增 phase0c_pose_source.csv |
| SourceFrameIdentity 全字段 | ✗ | 新增 phase0c_pose_source.csv |
| PoseAuthorityIdentity 全字段 | ✗ | 新增 phase0c_pose_source.csv |
| point → map xyz | ✗ | 离线用 d5 base xyz × pose 转 |
| point → mature static cell | ✗ | 新增 phase0c_static_cells.csv |
| map_generation / static revision | ✗ | 新增 phase0c_pose_source.csv + phase0c_static_cells.csv |

## 本轮动作

新增 diagnostic-only 输出（`NDT_PHASE0C_FORENSIC=1` 门控）：
- `phase0c_pose_source.csv`：每 source stamp 一行（pose + SourceFrameIdentity + PoseAuthorityIdentity + static generation + lifecycle/hook）。
- `phase0c_static_cells.csv`：static snapshot 的 mature/clean cells（revision 变化时重写）。

配合 Phase 0B 已有的 `d5_voxel_lineage.csv`（point base xyz）+ `d5_weak_fragments.csv`（fragment 几何），离线即可完成 `point_base × pose → point_map → static cell` 的完整对齐，无需在线 point→static 查询（避免 detector 时序早于 pose 的问题）。
