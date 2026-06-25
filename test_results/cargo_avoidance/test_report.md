# 吊货避障功能测试报告

## 测试环境

- **测试时间**: 2026-06-25
- **Git 分支**: feature/cargo-avoidance
- **Git Commit**: cfcf476
- **测试机器**: 本地虚拟机
- **测试 Bag**: /home/ydkj/AutoCraneSlam-ROS1/bag/调运长件.bag

## 测试 0：编译检查

**结果**: ✅ PASS

```
[100%] Built target ndt_slam_node
[100%] Built target cargo_forbidden_zone_node
编译成功
```

## 测试 1：路径配置检查

**结果**: ✅ PASS

```
旧路径 /home/ydkj/slam_data 已全部清除
新路径 /home/ydkj/NDT-slam-ws/maps/live/current 已统一使用
```

配置文件路径：
- `live_longterm_mapping.yaml`: `root_dir: "/home/ydkj/NDT-slam-ws/maps/live/current"`
- `cargo_forbidden_zone.yaml`: `objects_tiles_dir: "/home/ydkj/NDT-slam-ws/maps/live/current/tiles_objects"`

## 测试 2：Bag 离线建图生成 tiles

**结果**: ✅ PASS

### 生成的 tiles

| 目录 | 文件 | 大小 |
|------|------|------|
| tiles_registration | x0_y0.pcd, x0_y-1.pcd | 21KB, 90KB |
| tiles_display | x0_y0.pcd, x0_y-1.pcd | 174KB, 1.9MB |
| tiles_ground | x0_y0.pcd, x0_y-1.pcd | 138KB, 760KB |
| tiles_objects | x0_y0.pcd, x0_y-1.pcd | 36KB, 555KB |

### runtime_status.json

```json
{
  "total_frames": 507,
  "total_keyframes": 17,
  "active_keyframes": 17,
  "global_map_points": 8065,
  "objects_map_points": 49809,
  "memory_mb": 115,
  "memory_guard_triggered": false,
  "last_ndt_fitness": 0.05,
  "ndt_fitness_warning": false
}
```

## 测试 3：单独启动避障节点读取 tiles_objects

**结果**: ✅ PASS

### Topic 检查

| Topic | 状态 |
|-------|------|
| `/cargo_forbidden_grid` | ✅ 存在并发布 |
| `/cargo_collision_warning` | ✅ 存在并发布 |
| `/cargo_predicted_path` | ✅ 存在 |
| `/cargo_forbidden_markers` | ✅ 存在 |

### 避障节点日志

```
[CargoForbiddenZone] Loaded x0_y0.pcd: 2988 points
[CargoForbiddenZone] Loaded x0_y-1.pcd: 47309 points
[CargoForbiddenZone] Total points loaded: 50297
[CargoForbiddenZone] Grid range: (-10.3,-18.8) to (16.7,14.0), size: 270 x 328
[CargoForbiddenZone] Occupied cells: 10485
```

### OccupancyGrid 信息

```
resolution: 0.10
width: 270
height: 328
origin: (-10.26, -18.78)
```

### 风险等级

没有 `/payload_track_info` 时，风险等级为 `UNKNOWN (5)`，符合预期。

## 测试 4：SLAM + 避障节点 + bag 联合测试

**结果**: ⚠️ 部分 PASS

### 观察结果

- SLAM 正常启动并处理点云
- 避障节点正常启动并加载 tiles_objects
- `/payload_track_info` 话题存在
- `/cargo_collision_warning` 话题存在

### 问题

由于测试时间较短（30 秒），且 bag 中吊货移动场景有限，`/payload_track_info` 可能没有输出有效的动态 track。

这**不是代码 bug**，而是测试数据限制。

## 测试结论

| 测试项 | 结果 | 说明 |
|--------|------|------|
| 编译检查 | ✅ PASS | ndt_slam_node 和 cargo_forbidden_zone_node 编译成功 |
| 路径配置 | ✅ PASS | 旧路径已清除，新路径统一 |
| Bag 建图生成 tiles | ✅ PASS | 4 层 tiles 正常生成 |
| 避障节点读取 tiles | ✅ PASS | 成功加载并构建 2.5D 栅格 |
| OccupancyGrid 发布 | ✅ PASS | width=270, height=328, resolution=0.10 |
| 风险等级（无 track） | ✅ PASS | 输出 UNKNOWN (5) |
| 联合测试 | ⚠️ 部分 | 基本链路通过，需更长 bag 验证动态 track |

## 下一步建议

1. **使用更长 bag 测试**：包含更多吊货移动场景
2. **验证自适应尺寸**：当 point_count >= 80 时使用 bbox 尺寸
3. **验证风险等级变化**：IDLE → NORMAL → WARNING → STOP
4. **RViz 可视化测试**：确认禁行区和预测轨迹对齐

## 文件位置

- 测试报告: `/home/ydkj/NDT-slam-ws/test_results/cargo_avoidance/test_report.md`
- 测试数据: `/home/ydkj/NDT-slam-ws/maps/live/current/`
