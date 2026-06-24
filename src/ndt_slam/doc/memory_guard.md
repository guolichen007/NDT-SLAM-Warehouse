# MemoryGuard / DiskGuard / Watchdog

## MemoryGuard 分级策略

| 级别 | 阈值 | 行为 |
|------|------|------|
| OK | < 6GB | 正常运行 |
| SOFT | 6-7GB | 释放缓存 + flush dirty tiles + malloc_trim |
| HARD | 7-8GB | 暂停地图 commit（NDT/TF 继续） |
| EMERGENCY | > 8GB | 降采样 active map |

配置：
```yaml
memory_guard:
  enabled: true
  soft_threshold_mb: 6000
  hard_threshold_mb: 7000
  emergency_threshold_mb: 8000
  check_interval_sec: 30
```

## DiskGuard 磁盘保护

磁盘空间不足时暂停 tile 写入：

```yaml
disk_guard:
  enabled: true
  min_free_disk_gb: 30
  pause_mapping_when_low: true
```

## Pointcloud Watchdog

雷达断流检测：

```yaml
pointcloud_watchdog:
  stale_timeout_sec: 10.0
```

## NDT Health Monitor

NDT fitness 健康监控：

```yaml
ndt_health:
  fitness_warning_threshold: 2.0
  fitness_warning_count: 50
```

## runtime_status.json 字段

```json
{
  "memory_mb": 162,
  "memory_guard_triggered": false,
  "disk_free_gb": 70.45,
  "disk_guard_triggered": false,
  "pointcloud_stale": false,
  "ndt_fitness_warning": false
}
```
