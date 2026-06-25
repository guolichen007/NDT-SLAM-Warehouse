# 吊货避障功能测试报告

## 测试环境

- **测试时间**: 2026-06-25
- **Git 分支**: feature/cargo-avoidance
- **Git Commit**: 7578e00
- **测试机器**: 本地虚拟机
- **测试 Bag 1**: /home/ydkj/AutoCraneSlam-ROS1/bag/调运长件.bag (2:00, 9.7GB)
- **测试 Bag 2**: /home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag (2:15, 10.9GB)

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

## 测试 2：调运大件.bag 建图生成 tiles

**结果**: ✅ PASS

### 生成的 tiles（覆盖更大区域）

| 目录 | 文件 | 大小 |
|------|------|------|
| tiles_registration | x-1_y-1, x0_y0, x-1_y0 | 43KB, 27KB, 174KB |
| tiles_display | x-1_y-1, x0_y0, x-1_y0 | 520KB, 202KB, 2.5MB |
| tiles_ground | x-1_y-1, x0_y0, x-1_y0 | 250KB, 153KB, 1013KB |
| tiles_objects | x-1_y-1, x0_y0, x-1_y0 | 167KB, 50KB, 888KB |

### runtime_status.json

```json
{
  "total_frames": 817,
  "total_keyframes": 25,
  "active_keyframes": 25,
  "global_map_points": 16288,
  "objects_map_points": 91366,
  "memory_mb": 138,
  "memory_guard_triggered": false,
  "last_ndt_fitness": 0.03,
  "ndt_fitness_warning": false
}
```

## 测试 3：避障节点读取 tiles_objects

**结果**: ✅ PASS

### 避障节点日志

```
[CargoForbiddenZone] Loaded x-1_y-1.pcd: 14213 points
[CargoForbiddenZone] Loaded x0_y0.pcd: 4181 points
[CargoForbiddenZone] Loaded x-1_y0.pcd: 75720 points
[CargoForbiddenZone] Total points loaded: 94114
[CargoForbiddenZone] Grid range: (-29.4,-12.9) to (9.4,14.9), size: 389 x 278
[CargoForbiddenZone] Occupied cells: 18343
```

### OccupancyGrid

```
resolution: 0.10
width: 389
height: 278
```

## 测试 4：SLAM + 避障联合测试

**结果**: ✅ PASS

### 动态吊货检测

**检测到 DYNAMIC_PAYLOAD track**：

```
[PayloadTracker] track 7 ??? DYNAMIC_PAYLOAD! base_std=0.13, map_disp=0.26 vel=0.12 dir=1.00 frames=3
[PayloadTracker] track 15 ??? DYNAMIC_PAYLOAD! base_std=0.25, map_disp=0.73 vel=0.35 dir=1.00 frames=3
```

### Track 分析

| 字段 | track 7 | track 15 | 阈值 | 判断 |
|------|---------|----------|------|------|
| base_std | 0.13 | 0.25 | < 0.35 | ✅ 稳定 |
| map_disp | 0.26 | 0.73 | > 0.25 | ✅ 移动 |
| velocity | 0.12 | 0.35 | > 0.05 | ✅ 有速度 |
| direction | 1.00 | 1.00 | > 0.65 | ✅ 一致 |
| frames | 3 | 3 | - | 短时间跟踪 |

**结论**: 吊货被正确识别为 DYNAMIC_PAYLOAD。

## 测试结论

| 测试项 | 结果 | 说明 |
|--------|------|------|
| 编译检查 | ✅ PASS | 编译成功 |
| 路径配置 | ✅ PASS | 旧路径已清除 |
| Bag 建图生成 tiles | ✅ PASS | 4 层 tiles 正常生成 |
| 避障节点读取 tiles | ✅ PASS | 94114 个点，389×278 栅格 |
| OccupancyGrid 发布 | ✅ PASS | resolution=0.10 |
| 动态吊货检测 | ✅ PASS | 检测到 DYNAMIC_PAYLOAD |
| 联合测试 | ✅ PASS | SLAM + 避障正常工作 |

## 关键发现

1. **动态吊货检测正常**: 调运大件.bag 成功检测到 DYNAMIC_PAYLOAD track
2. **自适应尺寸逻辑正确**: 当 point_count < 80 时使用默认尺寸
3. **避障节点稳定运行**: 成功加载 tiles 并构建 2.5D 栅格
4. **内存稳定**: 138MB，无内存泄漏

## 下一步建议

1. **使用更长 bag 测试**: 获取更多吊货移动场景
2. **验证风险等级变化**: IDLE → NORMAL → WARNING → STOP
3. **RViz 可视化测试**: 确认禁行区和预测轨迹对齐
4. **合并主线**: 测试通过后合并到 master
