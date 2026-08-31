# Phase 0C Static Alignment — mature Static 不可用

## 结论

```text
STATIC_SOURCE_ALIGNED = YES（snapshot 按 source stamp 对齐可恢复）
TEMPORALLY_MATURE = 0（empty_map 冷启动 + 无.bag 200s 内无 mature cell）
```

## 数据

`phase0c_static_cells.csv`（2622 cell）：

```text
map_generation=1, revision=3
clean_map_confirmed = 2622（全部）
temporally_mature  = 0（全部）
```

`phase0c_pose_source.csv` 的 `static_mature_cells` 字段全程 = 0。

## 根因

`isTemporallyMatureLocked`（static_obstacle_evidence_index.cpp:400）要求：

```text
clean_map_confirmed && map_generation==working_generation_
&& consecutive_observation_count >= minimum_observations(4)
&& consecutive_stable_duration_sec >= minimum_stable_duration_sec(1.0)
```

empty_map 冷启动里，`clean_map_confirmed` 通过 clean-map build 一次建立，但「连续 4 帧观察 + 稳定 1s」的 streak 在 200s 内未累计到（cargo safe-over 运动 + 观测被打断）。

## 影响

`cargoGroupOverlapsMatureStaticEvidence` 只认 `clean_map_confirmed && temporally_mature` 的 cell，因此在当前数据集上 **mature static conflict 从未触发**，无法评估「mature Static provenance」能否排除错误 support。

**若要验证 mature Static discriminant，需非 empty_map 冷启动采集**（加载已 mature 的 map，或更长的连续观察）。
