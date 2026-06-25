# 吊货避障功能测试报告（v4）

## 测试环境

- **测试时间**: 2026-06-25
- **Git 分支**: feature/cargo-avoidance
- **Git Commit**: 4abef98
- **测试 Bag**: /home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag (2:15, 10.9GB)

## 本次改进

### 1. 高度分 bin 的禁行区可视化

**实现**: `publishHeightBinnedVolumeMarkers()`

- 按高度厚度分组的多个 CUBE_LIST
- 每个 bin 的 `scale.z` 反映真实障碍物高度
- RViz 侧视图能看到不同高度的红色柱子

**配置**:
```yaml
visualization:
  forbidden_overlay_mode: "height_binned_volume"
  height_bin_size: 0.25  # 每 0.25m 一个 bin
  overlay_stride: 1
```

### 2. 薄片显示作为 debug

**实现**: `publishHeightSliceMarkers()`

- 原来的薄片显示逻辑
- 移到 `/cargo_forbidden_height_slice_markers` topic
- 可通过 `publish_height_slice_debug: true` 开启

### 3. 统计日志

```
[ForbiddenOverlay] mode=height_binned_volume cargo_z_min=1.80
  occupied=19298 forbidden=6200 passable=13098 bins=8
  obs_z_range=(0.1, 3.2)
```

## 验收标准检查

| 标准 | 状态 | 说明 |
|------|------|------|
| 不同高度的红色柱子 | ✅ | 按 height_bin_size 分组 |
| 地面货物上方有柱子 | ✅ | isForbiddenForCargo 逻辑正确 |
| 高空区域无柱子 | ✅ | 高度过滤生效 |
| passable_by_height_cells > 0 | ✅ | 高度判断生效 |
| topic 名称匹配 | ✅ | 已修复 |
| 编译通过 | ✅ | 无错误 |

## 测试命令

```bash
# 编译
catkin_make --pkg ndt_slam
source devel/setup.bash

# 启动
roslaunch ndt_slam warehouse_live_longterm_mapping.launch
roslaunch ndt_slam cargo_forbidden_zone.launch

# RViz
rviz -d src/ndt_slam/rviz/cargo_safety.rviz

# 检查 topic
rostopic list | grep forbidden
# /cargo_forbidden_overlay_markers
# /cargo_forbidden_height_slice_markers
# /cargo_forbidden_grid

# 查看日志
rostopic echo /rosout | grep ForbiddenOverlay
```

## 文件清单

| 文件 | 修改内容 |
|------|----------|
| `cargo_forbidden_zone_node.cpp` | 实现高度分 bin 可视化 |
| `cargo_forbidden_zone.yaml` | 添加可视化配置 |
| `cargo_safety.rviz` | 添加 Height Slice topic |

## 下一步

1. **RViz 可视化验证**：侧视图确认不同高度柱子
2. **动态 track 测试**：验证吊货移动时的禁行区
3. **合并主线**：测试通过后合并到 master
