# 地图生命周期

## 工作层与正式层

运行线程维护 `global/display/ground/objects/objects_clean` 工作指针。MapCommit 先更新 raw 层并推进 objects version，然后请求后台 clean 重建。

Clean Worker 在一个锁内深拷贝 raw bundle N，锁外构建 clean N，完成后组成：

```text
MapLayerBundle {
  generation, objects_version, lifecycle_epoch, source_stamp,
  registration, display, ground, objects, objects_clean
}
```

五个指针在 bundle 完成后均为只读。ROS 发布与正式多层保存只读取同一个 completed bundle，所以同一 `header.seq` 对应同一内容代次，而不是同一复制时刻的混合层。

## 并发与活性

若工作地图在 build N 期间前进到 N+1：

- 完整 N 仍发布，避免持续提交造成 clean 饥饿；
- clean N 不安装到当前工作地图；
- 调度 N+1 clean 重建。

reset/load 会增加 lifecycle epoch。旧 epoch 的后台结果直接丢弃，不能在新地图之后重新发布。

## 写入门控

`STATIONARY_HOLD`、`MOVING_CONFIRM`、`CATCH_UP`、prediction-only 和严重退化均禁止 local map 更新与持久 MapCommit。两类权限分别统计，不能用一个布尔量隐式代替。

## 保存

正式五层 PCD 来自 latest completed bundle。诊断层可使用当前工作快照，并在文件名与文档中保持其非正式性质。

## 长期静态证据生命周期

静态证据分为三种状态：当前 epoch 的 working index、供安全线程无锁读取的
immutable snapshot，以及 Manifest 指向的 persistent committed snapshot。
MapCommit 只累积连续观测；clean worker 才能确认授权。人体、吊物历史和三维
动态 deny 生成带版本 tombstone，同轮或更旧 clean 结果不得重新插入该单元。

`reset`、`load_map`、重定位/闭环和全图重建会推进 lifecycle epoch。运行时立即
切换到空的 fail-safe snapshot；磁盘上的当前 Manifest 被移动为 inactive
last-good 归档。只有当前 epoch、clean-confirmed 且达到最小持久化单元数的新
snapshot 才能通过临时文件和 rename 成为新的 Manifest commit point。旧 worker
epoch 始终丢弃；同 epoch 的 stale worker 只可确认未被更高版本失效的单元。


## 静态权威级别

地图里「存在点」与「该点可作为正式安全证据」分开。静态权威只允许：

- `RUNTIME_MATURE`：在线连续观测形成的成熟证据；
- `OPERATOR_APPROVED_BASELINE`：离线审计后操作员批准的基线；
- `UNVERIFIED_LOADED_CLEAN`：仅用于显示和再验证，不进入正式避障（不能发布 14/17/18）。

历史 clean PCD 若无审核结论，一律标 `UNVERIFIED_LOADED_CLEAN`。


## 长期在线建图

### 核心目标

```
雷达常开 → 天车静止时不重复建图 → 天车移动时增量落盘 → 内存和硬盘都有上限保护
```

### MotionGate（地图提交门控）

静止时不添加关键帧，移动时才提交：

```yaml
motion_gate:
  enabled: true
  min_translation_m: 0.30
  min_rotation_deg: 3.0
  min_time_between_keyframes_sec: 2.0
```

### 关键帧 Active Window

最近 80 个关键帧保留完整 `cloud_`，超出的释放 `cloud_`：

```yaml
online_cache:
  max_active_keyframes: 80
  release_check_interval: 10
```

### 磁盘 Tile 增量落盘

20m × 20m tile，五层地图（registration / display / ground / objects / objects_clean）：

```yaml
persistent_map:
  enabled: true
  tile_size_m: 20.0
  flush_interval_sec: 60
  write_tmp_then_rename: true
```

### Active Map 定期重建

每 10 个关键帧从最近 80 帧重建 active map：

```yaml
active_map:
  rebuild_every_keyframes: 10
  max_active_keyframes: 80
```

### observe_only 模式

首次上服务器时使用观察模式：

```yaml
longterm_mapping:
  enabled: true
  commit_enabled: false  # 观察模式
```

确认稳定后改为 `commit_enabled: true`。


## 地图后处理

### 后处理流程

```bash
bash src/ndt_slam/scripts/postprocess/run_map_postprocess.sh <输入目录> <输出目录>
```

流程：
1. 分析原始地图质量
2. 生成 ground_map_clean
3. 生成 objects_clean
4. 生成 registration_map_fixed
5. 生成 navigation_grid
6. 生成地图清单

### 输出结构

```
release/
├── registration_map_fixed.pcd
├── localization_map_fine.pcd
├── localization_map_report.json
├── objects_clean.pcd
├── ground_map_clean.pcd
├── navigation_grid_0.05m.pgm
├── navigation_grid_0.05m.yaml
├── map_manifest.yaml
└── quality_after.json

debug_layers/
├── display_map.pcd
├── objects_raw.pcd
└── ...
```

### 定位地图报告合同

`build_localization_map.py` 生成 `localization_map_report.json`，至少包含输入地图、地面模型、
输入点数、输出点数和实际配置。报告在目标目录内写入临时文件，显式使用 UTF-8/LF，
执行文件 flush 和 `fsync`，回读校验 JSON 与必填字段后再用 `os.replace` 原子替换。

序列化、校验或替换前失败时会删除临时文件并保留上一份正式报告。该原子性只覆盖
JSON 报告的可见性与文件数据落盘，不扩大为整个输出目录或 PCD 文件的事务承诺。

对应版本：`f57d68a`。
