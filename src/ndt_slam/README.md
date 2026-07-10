# ndt_slam ROS Package

这是 NDT-SLAM-Warehouse 的核心 ROS 包，包含双雷达合并、NDT_OMP 建图、动态物体过滤、长期在线建图和定位运行节点。

完整项目说明见仓库根目录 [README.md](../../README.md)。

详细文档见 [doc/](doc/) 目录。

## 常用入口

```bash
# 主建图
roslaunch ndt_slam mapping.launch

# 长期在线建图
roslaunch ndt_slam warehouse_live_longterm_mapping.launch

# 定位运行
roslaunch ndt_slam warehouse_runtime.launch
```

## Cargo Warning V1

货物预警系统，详见 [README_cargo_warning_v1.md](README_cargo_warning_v1.md)。

### 关键 Topic

| Topic | 类型 | 说明 |
|-------|------|------|
| `/cargo_warning` | `std_msgs/String` | 预警消息（JSON 格式） |
| `/cargo_tight_box_marker` | `visualization_msgs/Marker` | 绿色货物紧框 |
| `/cargo_warning_zone_marker` | `visualization_msgs/MarkerArray` | 黄色/红色预警范围 |
| `/cargo_warning_obstacle_marker` | `visualization_msgs/Marker` | 白色最近危险障碍物 |

### 启动参数

```bash
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  odom_anchored_cargo_box_enabled:=true \
  hook_cargo_removal_enabled:=true \
  cargo_warning_enabled:=true
```

## 配置文件

| 文件 | 用途 |
|------|------|
| `config/live_longterm_mapping.yaml` | 长期建图配置（含 Cargo Warning） |
| `config/slam_params.yaml` | 主配置 |
| `config/cargo_forbidden_zone.yaml` | Cargo 可视化节点配置 |
