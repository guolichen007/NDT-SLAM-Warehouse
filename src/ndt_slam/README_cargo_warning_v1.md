# Cargo Warning V1

## 目标

实现货物预警系统 V1，包括：
1. 绿色框 = 真实货物 tight box（紧贴货物边界）
2. 黄色框 = 5m 二级预警范围
3. 红色框 = 3m 一级预警范围
4. 输出 `/cargo_warning` 报警消息
5. 只从 NDT registration input 去除锁定吊物，不擦地图

## 预警规则

- 预警距离从**货物 footprint 边界**计算，不从中心计算
- 障碍物高度使用 z95（95%分位数），不用最高点
- 货物底部使用保守值：`cargo_bottom_safe = bottom - uncertainty - margin`
- 垂直净空：`clearance = cargo_bottom_safe - obstacle_top_z`

### 报警等级

| 等级 | 条件 | alarm_code |
|------|------|------------|
| LEVEL_1 | 距离 ≤ 3m 且 clearance < 0.80m | 17 |
| LEVEL_2 | 距离 ≤ 5m 且 clearance < 0.80m | 18 |
| LEVEL_NONE | 其他情况 | 0 |

### 防误报规则

1. 地面点不参与报警：HAG < 0.20m 直接过滤
2. 货物自身不参与报警：tight_box 外扩 margin 后，内部点全部排除

## 输出 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/cargo_warning` | `ndt_slam/CargoWarning` | 预警消息 |
| `/cargo_tight_box_marker` | `visualization_msgs/Marker` | 绿色货物紧框 |
| `/cargo_warning_zone_marker` | `visualization_msgs/MarkerArray` | 黄色/红色预警范围 |
| `/cargo_warning_obstacle_marker` | `visualization_msgs/Marker` | 白色最近危险障碍物 |

## 运行方式

```bash
# 编译
cd ~/NDT-slam-ws
catkin_make
source devel/setup.bash

# 启动
rosparam set /use_sim_time true

roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=true \
  persistent_map:=false \
  odom_anchored_cargo_box_enabled:=true \
  hook_cargo_removal_enabled:=true \
  cargo_warning_enabled:=true \
  publish_rslidar_tf:=true
```

## RViz 显示

推荐显示以下 topic：
- `/cargo_tight_box_marker` - 绿色货物紧框
- `/cargo_warning_zone_marker` - 黄色/红色预警范围
- `/cargo_warning_obstacle_marker` - 白色危险障碍物
- `/cargo_warning` - 预警消息数据
- `/merged_points` - 点云
- `/display_map` - 地图

## 验收标准

### 1. 框验收

- [ ] 绿色 tight box 与货物对齐，误差 10~20cm
- [ ] 绿色框不明显超过货物边界
- [ ] locked size 不再长期固定不变
- [ ] 绿色框不因 anchor 对称被撑大

### 2. 预警验收

- [ ] 黄色 5m 框、红色 3m 框显示正常
- [ ] `/cargo_warning` 稳定输出 valid、level、distance、clearance、alarm_code
- [ ] 3m 内净空 < 0.80m 输出 LEVEL_1
- [ ] 5m 内净空 < 0.80m 输出 LEVEL_2
- [ ] 无危险障碍物时输出 LEVEL_NONE / clear

### 3. 轨迹验收

- [ ] HookCargoRemoval enabled=1
- [ ] registration_cargo_removed > 0
- [ ] 无重复 TF 节点名
- [ ] 无 ERROR

## 验收命令

```bash
# 检查 tight box 日志
grep "TightBox\|OdomAnchorDetect\|CargoBoxLock" /tmp/cargo_warning_v1.log | tail -80

# 检查预警输出
grep "CargoWarning" /tmp/cargo_warning_v1.log | tail -80

# 检查点云去除
grep "HookCargoRemoval\|RegistrationCargoRemoval\|cargo_removed" /tmp/cargo_warning_v1.log | tail -80

# 检查错误
grep "\[ERROR\]" /tmp/cargo_warning_v1.log | head -20

# 检查重复 TF
grep "base_link_to_rslidar_tf\|duplicate\|same name" /tmp/cargo_warning_v1.log | head -20
```

## 当前不做什么（V1 限制）

- 不启用 CargoBoxEstimator V2
- 不启用 dynamic_history_eraser
- 不启用 global_payload_tracker
- 不擦除地图中的货物点
- 不移植 A7 runtime_path（后续版本）
- 不接 PLC
- 不支持多个吊物

## 配置参数

关键配置项（在 `config/live_longterm_mapping.yaml`）：

```yaml
odom_anchored_cargo_box:
  enabled: true
  tight_box:
    enabled: true
    anchor_symmetry_mode: "soft"
    hag_filter_enabled: true
    percentile_low: 0.08
    percentile_high: 0.92
    size_update_mode: "adaptive"
  cargo_warning:
    enabled: true
    level1_distance_m: 3.0
    level2_distance_m: 5.0
    min_vertical_clearance_m: 0.80

hook_cargo_lock:
  enable_hook_cargo_removal: true
```
