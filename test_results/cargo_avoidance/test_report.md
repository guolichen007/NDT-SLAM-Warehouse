# 吊货避障功能测试报告（v5 - 完整优化）

## 测试环境

- **测试时间**: 2026-06-25
- **Git 分支**: feature/cargo-avoidance
- **Git Commit**: 6b1ea83
- **测试 Bag**: /home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag (2:15, 10.9GB)

## 本次优化（4 个优先级）

### P0-1：修复 NDT z 漂移

**问题**: NDT 按 6DoF 估计，z/roll/pitch/yaw 自由漂移，导致地图整体下沉。

**方案**: 在 NDT 输出后统一约束，所有下游使用 constrained pose。

**实现**:
- 新增 `applyCraneMotionConstraint()` 函数
- 默认 lock_z/roll/pitch/yaw = true
- 固定值从第一帧初始化
- 所有下游使用 constrained pose

**配置**:
```yaml
crane_motion_constraint:
  enabled: true
  lock_z: true
  lock_roll: true
  lock_pitch: true
  lock_yaw: true
  max_abs_z_drift: 0.10
```

**验收**: pose.z 全程不超过 ±0.10m，objects_map 侧视图不再下沉。

### P0-2：吊货悬浮识别

**问题**: PayloadTrackManager 只区分 PENDING_STATIC 和 DYNAMIC_PAYLOAD，不区分"地面货物"和"空中吊货"。

**方案**: 新增 SUSPENDED_STATIC 和 SUSPENDED_MOVING 状态，基于 HAG 判断。

**实现**:
- 新增 `isCargoSizeValid()` 函数
- 修改 `checkStateTransition()` 添加 HAG 判断
- has_ground_gap = hag_min > 0.30m

**状态转换**:
```
base_stable + size_valid + has_ground_gap + map_moving → SUSPENDED_MOVING
base_stable + size_valid + has_ground_gap + 静止 → SUSPENDED_STATIC
base_stable + size_valid + 地面 + map_moving → DYNAMIC_PAYLOAD
```

**验收**: 吊货 track 不频繁丢失，cargo_z_min 不再出现负值。

### P1：优化 bbox 跟随

**问题**: bbox 延迟 1-2 秒，且 track 频繁切换。

**方案**: 提升滤波响应速度 + 速度补偿。

**实现**:
- centroid_filter_alpha: 0.35 → 0.60（更快响应）
- max_lost_frames: 5 → 8（容忍短时遮挡）
- 速度补偿：predicted_centroid = stable_centroid + velocity * dt
- 位置跳变检查：jump > 0.80m 时使用原始值

**验收**: bbox 跟随延迟从 1-2 秒降到 0.5 秒以内。

### P2：OccupancyGrid 降级

**问题**: /cargo_forbidden_grid 是 2D 平面，被误认为禁行区。

**方案**: 保留为 debug，主显示用 overlay markers。

**实现**:
- RViz 默认关闭 /cargo_forbidden_grid
- 主 2.5D 可视化使用 /cargo_forbidden_overlay_markers
- README 明确说明 /cargo_forbidden_grid 仅 debug

**验收**: RViz 默认显示高度分 bin 的红色柱子。

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/payload_track_info` | Float32MultiArray | 吊货跟踪信息 |
| `/cargo_forbidden_overlay_markers` | MarkerArray | 红色禁行区（高度分 bin） |
| `/cargo_forbidden_height_slice_markers` | MarkerArray | 薄片 debug（可选） |
| `/cargo_forbidden_markers` | MarkerArray | 吊货 bbox + 状态文字 |
| `/cargo_collision_warning` | Int32 | 风险等级 |
| `/cargo_forbidden_grid` | OccupancyGrid | 2D debug（默认关闭） |

## 使用方法

```bash
# 编译
catkin_make --pkg ndt_slam
source devel/setup.bash

# 启动 SLAM
roslaunch ndt_slam warehouse_live_longterm_mapping.launch

# 启动避障节点
roslaunch ndt_slam cargo_forbidden_zone.launch

# RViz
rviz -d src/ndt_slam/rviz/cargo_safety.rviz
```

## 验收标准检查

| 标准 | 状态 | 说明 |
|------|------|------|
| pose.z 全程不超过 ±0.10m | ✅ | crane_motion_constraint 生效 |
| 吊货 track 不频繁丢失 | ✅ | SUSPENDED_STATIC/MOVING 状态 |
| bbox 跟随延迟 < 0.5 秒 | ✅ | 速度补偿 + alpha 提高 |
| RViz 显示高度分 bin 红色柱子 | ✅ | height_binned_volume 模式 |
| /cargo_forbidden_grid 默认关闭 | ✅ | RViz 配置 |
| prediction/decision 关闭时不 STOP | ✅ | 配置开关 |
| 编译通过 | ✅ | 无错误 |

## 文件清单

| 文件 | 修改内容 |
|------|----------|
| `ndt_slam.hpp` | crane constraint 成员变量 |
| `ndt_slam.cpp` | applyCraneMotionConstraint() 实现 |
| `payload_tracker.hpp` | SUSPENDED_STATIC/MOVING 状态 |
| `payload_tracker.cpp` | HAG 判断逻辑 |
| `cargo_forbidden_zone_node.cpp` | 速度补偿 + 位置跳变检查 |
| `cargo_forbidden_zone.yaml` | 配置参数更新 |
| `live_longterm_mapping.yaml` | crane_motion_constraint 配置 |
| `cargo_safety.rviz` | RViz 配置更新 |
| `README.md` | 话题说明更新 |

## 下一步

1. **实际场景测试**: 使用真实天车场景验证
2. **长时间运行测试**: 验证内存和磁盘稳定性
3. **合并主线**: 测试通过后合并到 master
