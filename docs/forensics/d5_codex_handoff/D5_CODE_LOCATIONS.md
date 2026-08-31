# D5 Code Locations — 关键代码定位（BASE_SHA 6d1b2d4）

所有行号以 6d1b2d4 为准（未经 diagnostic 改动偏移）。

## detectCargoAroundOdomAnchor（D5 所在函数）

文件：`src/ndt_slam/src/ndt_slam.cpp`

| 行号 | 内容 |
|---|---|
| 15190 | 函数入口 `detectCargoAroundOdomAnchor` |
| 15223-15293 | ROI 裁剪（fixed + adaptive），`rejected_xy` 计数在 15290 |
| 15388-15406 | HAG 预过滤（`hag_filter_enabled`，ground invalid 则 bypass） |
| 15450-15464 | VoxelGrid 0.05m 降采样（硬编码 `setLeafSize(0.05f,...)`） |
| 15470 | `too_few_points` 检查（`voxel_cloud->size() < weak_min_points`） |
| **15477-15498** | **EuclideanClusterExtraction（D5 切断处）**：tolerance=0.20m, min=10, max=8000 |
| 15500 | `no_clusters` 检查 |
| 15628-15725 | 构造 `components`（CandidateComponentData），每 component 做 footprint + aspect 检查 |
| 15649 | `!footprint.valid || component_z.size()<3` continue |
| 15657 | `upper_extent_clipped` continue（超 max_size_x/y 拒绝） |
| 15684-15705 | B2 diagnostic：high_component（有 z>=1.3 的 component） |
| 15732-15769 | 构造 `fragments` → `buildCargoComponentHypotheses`（轴向融合） |
| 15773-15908 | 构造 hypothesis 的 footprint/z + score → shadow_candidates |
| **15916-15931** | **构造 component_observations[].points_base（→ union_points_base 的唯一入口）** |
| 15942-15951 | `groupCargoPhysicalCandidates` → shadow_physical_groups |
| 16031-16042 | `best_cluster` → `core_points_base`（selected hypothesis 的点） |

## Consumer 链（recovered points 的流向）

| 位置 | 内容 |
|---|---|
| `cargo_physical_identity_authority.cpp:473-484` | group.union_points_base = union(member component points) |
| `cargo_physical_identity_authority.cpp:484-699` | group descriptor extent/z 从 union_points 算（identity 统计） |
| `integrated_cargo_identity_shadow.cpp:63` | snapshot.union_points_base = selected->union_points_base |
| `integrated_cargo_identity_shadow.cpp:79-97` | resolved_geometry 从 representative（hypothesis descriptor） |
| `ndt_slam.cpp:23416` | bottom.points_base = integrated_group_evidence_.union_points_base |
| `cargo_v6_authority_adapter.cpp:140-180` | buildCanonicalCargoAuthoritySnapshot → map_mutation.owner_points.exact_points |
| `cargo_v6_authority_adapter.cpp:145-157` | exact_points 遍历 union_points_base（SourcePointKey 精确匹配） |
| `cargo_v6_authority_adapter.cpp:233-265` | cargoGroupOverlapsMatureStaticEvidence（static conflict） |
| `ndt_slam.cpp:7609` | avoidance_map_mutation.cargo_points = canonical snapshot map_mutation |
| `ndt_slam.cpp:28839` | commitKeyFrameWithDynamicFiltering 里 cargo_points.owns(point) 剔除 |

## 相关 frozen 文件（本 handoff 禁止改动）

```
cargo_physical_identity_authority.cpp / .hpp
cargo_bottom_fusion.cpp
cargo_safety_temporal_filter.cpp
cargo_avoidance_fusion.cpp
cargo_obstacle_tracker.cpp
fixed_yaw_translation_solver.cpp
rail_translation_pose_graph.cpp
rail_localization_authority.cpp
ndt_relocalizer.cpp
crane_motion_ekf.cpp
pose_authority_identity.hpp
frame_authority_context.hpp
```

## 本 forensic 新增 diagnostic 文件（diagnostic-only，不产生产品行为变化）

```
src/ndt_slam/include/ndt_slam/cargo_d5_fragment_forensic.hpp   （新增，纯函数模块）
src/ndt_slam/src/cargo_d5_fragment_forensic.cpp                （新增）
src/ndt_slam/src/ndt_slam.cpp                                  （+接线 +dump 方法）
src/ndt_slam/include/ndt_slam/ndt_slam.hpp                     （+成员 +声明 +include）
src/ndt_slam/CMakeLists.txt                                    （+源文件）
```

环境开关：`NDT_D5_FRAGMENT_FORENSIC=1`（默认关闭，产品路径零影响）。
输出：`/tmp/cargo_forensic/d5_voxel_lineage.csv` + `d5_weak_fragments.csv`。
