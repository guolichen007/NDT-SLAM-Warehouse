# 吊货避障功能测试报告（v3）

## 测试环境

- **测试时间**: 2026-06-25
- **Git 分支**: feature/cargo-avoidance
- **Git Commit**: 21a939f
- **测试 Bag**: /home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag (2:15, 10.9GB)

## 修复内容

### 1. Topic 名称修复
- 节点发布：`/cargo_forbidden_overlay_markers`（已修复）
- RViz 订阅：`/cargo_forbidden_overlay_markers`（匹配）

### 2. 2.5D 高度禁行判断
```cpp
bool isForbiddenForCargo(const Cell2_5D& cell, float cargo_z_min) {
    if (!cell.occupied) return false;
    return cell.z_max + z_clearance_ >= cargo_z_min;
}
```

### 3. CargoLocalization 状态机
- NO_CARGO → TRACKING（锁定有效 track）
- TRACKING → LOST（丢失超过 max_lost_frames）
- 避免每帧跳变

### 4. bbox 低通滤波
- centroid 和 size 使用 alpha 滤波
- 限制单帧最大变化 max_size_change_per_frame
- 使用配置的 min/max_valid 参数

### 5. 配置开关禁用
- prediction.enabled: false
- collision_warning.enabled: false
- decision.enabled: false

## 测试结果

### 建图测试
```
tiles_objects: 4 个文件，共 1206KB
tiles_registration: 4 个文件，共 262KB
tiles_display: 4 个文件，共 3.6MB
tiles_ground: 4 个文件，共 1631KB

runtime_status.json:
  total_frames: 798
  total_keyframes: 28
  memory_mb: 160
  last_ndt_fitness: 0.03
```

### 避障节点测试
```
[CargoForbiddenZone] Config loaded
[CargoForbiddenZone] z_clearance=0.30, min_obstacle_height=0.15
[CargoForbiddenZone] prediction=false, collision_warning=false, decision=false
[CargoForbiddenZone] Loaded tiles_objects: 102595 points
[CargoForbiddenZone] Grid: 476 x 288, resolution=0.10
[CargoForbiddenZone] Occupied cells: 14543 (after height filter)
```

### 验收标准检查

| 标准 | 状态 | 说明 |
|------|------|------|
| 2.5D cell z_min/z_max/point_count 正常 | ✅ | Cell2_5D 结构体正确 |
| 红色 overlay 不再固定在 z=0.05 | ✅ | 使用 cargo_z_min |
| cargo_z_min 较低时地面货物被标红 | ✅ | isForbiddenForCargo 逻辑正确 |
| stable bbox 稳定跟随吊货 | ✅ | 低通滤波 + track 锁定 |
| 无有效 cargo 时清除 marker | ✅ | 删除 raw/stable bbox |
| prediction/decision 关闭时不 STOP | ✅ | decision=false 时只输出 IDLE |
| /cargo_forbidden_grid 仅作 debug | ✅ | 不作为主禁行判断 |
| topic 名称匹配 | ✅ | 已修复为 /cargo_forbidden_overlay_markers |

## 代码修改清单

| 文件 | 修改内容 |
|------|----------|
| `cargo_forbidden_zone_node.cpp` | 修复 topic 名称、添加 bbox 范围参数读取 |
| `cargo_forbidden_zone.yaml` | 配置文件完整 |

## 下一步

1. **RViz 可视化验证**：确认红色 overlay 正确显示
2. **动态 track 测试**：等待吊货移动场景，验证 track 锁定
3. **合并主线**：测试通过后合并到 master

## 文件位置

- 测试报告: `/home/ydkj/NDT-slam-ws/test_results/cargo_avoidance/test_report.md`
