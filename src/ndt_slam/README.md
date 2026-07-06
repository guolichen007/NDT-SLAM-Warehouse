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

## Cargo State Tightbox Baseline

当前版本为 **cargo_state_tightbox_baseline**，已完成：

- ✅ CargoState 统一货物状态源
- ✅ TightBox 绿色紧框显示（HAG 过滤、soft symmetry、子簇重聚类）
- ✅ CargoHeightFilter 底部高度稳定保护
- ✅ EKF 高 fitness 拒绝
- ✅ 轨迹稳定化
- ✅ 3m/5m 预警接口预留

**未完成（后续分支开发）：**

- ⏳ HookCargoRemoval / RegistrationCargoRemoval 真正生效
- ⏳ 正式避障报警消息
- ⏳ 静态结构过滤
- ⏳ 底部高度最终可信估计

详见 [README_cargo_warning_v1.md](README_cargo_warning_v1.md)。

### 关键 Topic

| Topic | 类型 | 说明 | 状态 |
|-------|------|------|------|
| `/cargo_tight_box_marker` | `visualization_msgs/Marker` | 绿色货物紧框 | ✅ 可用 |
| `/cargo_warning_zone_marker` | `visualization_msgs/MarkerArray` | 黄色/红色预警范围 | ✅ 可用 |
| `/cargo_warning` | `std_msgs/String` | 预警消息（JSON 格式） | ⚠️ 默认关闭 |

### 启动参数

```bash
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  odom_anchored_cargo_box_enabled:=true
```

## 配置文件

| 文件 | 用途 |
|------|------|
| `config/live_longterm_mapping.yaml` | 长期建图配置（含 Cargo State） |
| `config/slam_params.yaml` | 主配置 |
| `config/cargo_forbidden_zone.yaml` | Cargo 可视化节点配置 |

## 分支说明

| 分支 | 用途 |
|------|------|
| `master` | 主线（当前为 tightbox baseline） |
| `feature/cargo-obstacle-warning-v2` | 避障预警开发分支 |
| `fix/display-map-publish-v8` | A7 参考分支（保留） |
