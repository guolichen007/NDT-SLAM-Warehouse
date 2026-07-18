# 长期在线建图

## 核心目标

```
雷达常开 → 天车静止时不重复建图 → 天车移动时增量落盘 → 内存和硬盘都有上限保护
```

## MotionGate（地图提交门控）

静止时不添加关键帧，移动时才提交：

```yaml
motion_gate:
  enabled: true
  min_translation_m: 0.30
  min_rotation_deg: 3.0
  min_time_between_keyframes_sec: 2.0
```

## 关键帧 Active Window

最近 80 个关键帧保留完整 cloud_，超出的释放 cloud_：

```yaml
online_cache:
  max_active_keyframes: 80
  release_check_interval: 10
```

## 磁盘 Tile 增量落盘

20m × 20m tile，4 层（registration/display/ground/objects）：

```yaml
persistent_map:
  enabled: true
  tile_size_m: 20.0
  flush_interval_sec: 60
  write_tmp_then_rename: true
```

## Active Map 定期重建

每 10 个关键帧从最近 80 帧重建 active map：

```yaml
active_map:
  rebuild_every_keyframes: 10
  max_active_keyframes: 80
```

## observe_only 模式

首次上服务器时使用观察模式：

```yaml
longterm_mapping:
  enabled: true
  commit_enabled: false  # 观察模式
```

确认稳定后改为 `commit_enabled: true`。
