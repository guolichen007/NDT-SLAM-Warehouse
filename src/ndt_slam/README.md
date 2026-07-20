# ndt_slam ROS Package

NDT-SLAM-Warehouse 的核心 ROS1 包，负责双雷达合并、结构保持 NDT 定位、各向异性 EKF、长期多层地图、吊物刚体跟踪和正式 Cargo Safety 协议。

项目入口与构建命令见仓库根目录 [README](../../README.md)。架构、部署、运维和验收说明见 [doc](doc/)。

## 常用入口

```bash
# 长期在线建图与吊物安全
roslaunch ndt_slam warehouse_live_longterm_mapping.launch

# 运行时定位
roslaunch ndt_slam warehouse_runtime.launch

# 基础建图
roslaunch ndt_slam mapping.launch
```

## 生产合同

- Registration Source 不允许 full-ground fallback；结构不足进入 prediction-only。
- 静止、移动确认和 CATCH_UP 分别控制运行位姿、local map 和持久 MapCommit。
- 吊物确认后冻结 OBB 长宽高与轴向 yaw，只更新实时中心。
- LOST_HOLD 的 marker 显示和正式安全证据使用不同时间窗；证据过期输出 33并停止正式剔除。
- 14/17/18 只表示空间碰撞关系，30-35 表示系统或证据故障。
- 五层正式地图只从同一不可变 `MapLayerBundle` 发布和保存。

## 关键 Topic

| Topic | 说明 |
|---|---|
| `/merged_points` | 双雷达合并当前帧 |
| `/odom` | EKF/定位运行位姿 |
| `/ndt_slam/runtime_path` | 实时轨迹 |
| `/map`、`/display_map*` | 同代五层地图 |
| `/cargo_core_bbox_marker` | 冻结形状、实时中心的正式吊物框 |
| `/cargo_avoidance/bottom_estimate` | 正式底部高度与 OBB 几何 |
| `/cargo_avoidance/safety_status` | 正式安全证据 |
| `/cargo_avoidance/status_code` | 14/17/18/30-35 输出 |

## 配置

| 文件 | 用途 |
|---|---|
| `config/live_longterm_mapping.yaml` | 定位、地图、吊物和安全生产参数 |
| `config/merger_params.yaml` | 双雷达合并 |
| `launch/warehouse_live_longterm_mapping.launch` | 生产启动入口 |
| `rviz/cargo_safety.rviz` | 吊物安全 RViz 布局 |

旧 OdomAnchor/TightBox 接口只作为兼容输出存在，不是正式安全或地图剔除的几何所有者。
