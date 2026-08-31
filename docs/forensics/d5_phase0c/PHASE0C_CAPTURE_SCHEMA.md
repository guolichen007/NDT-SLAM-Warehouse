# Phase 0C Capture Schema

> 本轮 diagnostic-only（`NDT_D5_FRAGMENT_FORENSIC=1` + `NDT_PHASE0C_FORENSIC=1`），产品逻辑零改动。

## 采集输出（/tmp/cargo_forensic/）

### 复用 Phase 0B（point-level lineage，base frame）

| 文件 | 内容 |
|---|---|
| d5_voxel_lineage.csv | 每 voxel 一行：stamp, base xyz, voxel_ix/iy/iz, cluster_label, cluster_size, assignment(0=PRIMARY_ACCEPTED/1=WEAK_REJECTED/2=UNASSIGNED), fragment_id |
| d5_weak_fragments.csv | 每 weak fragment 一行：stamp, fragment_id, point_count, base center, bbox, z p05/p50/p95, nearest_primary_*, xy overlap/gap, candidate_primary_neighbor_count, oracle_high_surface |

### 新增 Phase 0C

**phase0c_pose_source.csv**（每 source stamp 一行）：
```
stamp,source_stamp,processing_frame_index,source_cloud_size,source_cloud_signature_hash,
pose_tx,pose_ty,pose_tz,pose_qx,pose_qy,pose_qz,pose_qw,pose_yaw,
map_rebuild_generation,keyframe_pose_version,yaw_authority_generation,target_snapshot_id,
static_map_generation,static_revision,static_cell_size,static_mature_cells,
lifecycle_id,hook_state,hook_role
```

**phase0c_static_cells.csv**（static snapshot 的 clean/mature cells，revision 变化时重写）：
```
map_generation,revision,cell_key,cell_x,cell_y,min_z,max_z,clean_map_confirmed,temporally_mature
```

## 对齐方式

- 所有数据用 `sensor source stamp` 对齐（d5 base xyz 的 stamp == phase0c_pose_source 的 stamp）。
- `point_map = pose_map_base * point_base`（离线用 pose quaternion/translation 转）。
- `point → static cell`：离线用 point_map 的 xy 落 cell（`packStaticEvidenceCell(floor(x/cell_size), floor(y/cell_size))`）+ z 落 `[min_z, max_z]`。

## 关键约束

- 禁止用 latest pose / latest static snapshot 替代 source-time 对齐数据。
- 不 dump 完整 static map（只 dump clean/mature cells，控制数据量）。
- 不用 oracle 决定是否保存（全部保存）。
