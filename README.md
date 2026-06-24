# NDT-SLAM 起重机仓库建图系统

基于 NDT_OMP 的激光雷达 SLAM 系统，面向起重机仓库高位俯视场景。
支持双雷达点云合并、动态物体过滤、长期在线增量建图、多层地图输出和定位运行。

## 项目简介

起重机仓库场景，传感器安装在约 8m 高度俯视金属物料堆场。该场景下 KISS-ICP 完全退化（地面点占比 80-88%，导致平面退化），因此采用 NDT_OMP 配准策略，配合工程化的多层地图和动态物体过滤。

## 核心能力

- **NDT_OMP 配准**：1.0m 分辨率，处理仓库大空间退化场景
- **网格局部地面分割**：1.5m 网格、z 值第 20 百分位，处理倾斜地面
- **动态物体过滤**：吊货通道过滤 + 人体动态过滤 + 统一事件管理
- **长期在线建图**：MotionGate 静止不建图、关键帧 active window、20m 磁盘 tile 增量落盘、四级内存保护
- **多层地图输出**：registration / display / ground / objects / navigation grid
- **闭环检测**：ScanContext + g2o 位姿图优化
- **自动重定位**：ScanContext Top-K 粗定位 + NDT 精配准，天车约束（只取 x/y）

## 系统架构

```
rs_201 + rs_203 (双雷达)
    ↓
pointcloud_merger (点云合并 + 体素去重)
    ↓
/merged_points
    ↓
┌─ 近场过滤（去除起重机抓臂/吊具）
├─ 范围过滤 + 体素降采样
├─ 网格局部地面分割
├─ 动态物体过滤（吊货 + 工人）
├─ MotionGate（静止不建图）
├─ NDT_OMP 配准 → 位姿估计
├─ 关键帧管理（active window + 磁盘 tile）
└─ 多层地图生成 + 闭环检测
```

详细架构说明见 [doc/architecture.md](src/ndt_slam/doc/architecture.md)。

## 快速开始

### 依赖

- Ubuntu 20.04 + ROS Noetic
- PCL, Eigen3, Sophus, yaml-cpp, g2o, ndt_omp, TBB

### 编译

```bash
cd ~/NDT-slam-ws
catkin_make --pkg ndt_slam
source devel/setup.bash
```

### 运行

```bash
# 主建图
roslaunch ndt_slam mapping.launch

# 长期在线建图（雷达常开、磁盘 tile 增量保存）
roslaunch ndt_slam warehouse_live_longterm_mapping.launch

# 定位运行（加载已有地图）
roslaunch ndt_slam warehouse_runtime.launch

# 离线建图（回放 bag）
roslaunch ndt_slam offline_mapping.launch
```

## 运行模式

| 模式 | launch | 说明 |
|------|--------|------|
| 日常建图 | `mapping.launch` | 双雷达合并 + NDT_OMP + 动态过滤 |
| 长期建图 | `warehouse_live_longterm_mapping.launch` | MotionGate + tile 持久化 + 内存保护 |
| 定位运行 | `warehouse_runtime.launch` | 加载预建地图，纯定位 |
| 离线建图 | `offline_mapping.launch` | 回放 bag 文件建图 |
| 继续建图 | `continue_mapping.launch` | 在已有地图基础上继续建图 |

## 地图输出

| 地图 | 体素 | 用途 |
|------|------|------|
| registration_map | 0.30m | NDT 配准用 |
| display_map | 0.10m | 全量显示 |
| ground_map | 0.15m | 地面层 |
| objects_map | 0.08m | 物体/货物层 |
| objects_clean | 0.06m | 过滤后物体层（后处理） |
| navigation_grid | 0.05m | 2D 导航栅格 |

详细说明见 [doc/map_postprocess.md](src/ndt_slam/doc/map_postprocess.md)。

## 配置文件

| 文件 | 用途 |
|------|------|
| `slam_params.yaml` | 主 SLAM 配置（NDT 参数、话题、坐标系） |
| `live_longterm_mapping.yaml` | 长期建图配置（MotionGate、tile、内存保护） |
| `merger_params.yaml` | 双雷达合并配置 |
| `engineering_mapping.yaml` | 工程化建图配置 |
| `map_postprocess.yaml` | 地图后处理配置 |

详细说明见 [doc/configuration.md](src/ndt_slam/doc/configuration.md)。

## 代码模块

| 模块 | 说明 |
|------|------|
| `ndt_slam.cpp` | 主 SLAM 节点：NDT 配准、地图管理、长期建图调度 |
| `keyframe_manager.*` | 关键帧管理、active window、metadata |
| `loop_closure.*` | ScanContext 闭环检测、g2o 位姿图优化 |
| `point_cloud_processing.*` | 地面分割、滤波、近场过滤 |
| `base_payload_channel_filter.*` | 吊货通道过滤 |
| `payload_tracker.*` | 吊货轨迹跟踪 |
| `human_object_filter.*` | 人体动态过滤 |
| `dynamic_event_manager.*` | 统一动态事件管理 |
| `PointCloudMerger.cpp` | 双雷达点云合并 |
| `CloudDiagnostics.cpp` | 点云诊断工具 |

## 文档索引

| 文档 | 内容 |
|------|------|
| [doc/architecture.md](src/ndt_slam/doc/architecture.md) | 系统架构与数据流 |
| [doc/configuration.md](src/ndt_slam/doc/configuration.md) | 配置文件说明 |
| [doc/dynamic_filtering.md](src/ndt_slam/doc/dynamic_filtering.md) | 吊货/人体动态过滤设计 |
| [doc/longterm_mapping.md](src/ndt_slam/doc/longterm_mapping.md) | 长期在线增量建图 |
| [doc/memory_guard.md](src/ndt_slam/doc/memory_guard.md) | MemoryGuard / DiskGuard / Watchdog |
| [doc/deployment.md](src/ndt_slam/doc/deployment.md) | 部署指南（systemd、监控、备份） |
| [doc/map_postprocess.md](src/ndt_slam/doc/map_postprocess.md) | 地图后处理与 release 地图生成 |
| [doc/localization_runtime.md](src/ndt_slam/doc/localization_runtime.md) | 定位模式说明 |
| [doc/roadmap.md](src/ndt_slam/doc/roadmap.md) | 后续开发路线 |

## 当前限制

- 长期在线建图仍需要真实双雷达长时间现场验证
- 动态吊货过滤主要依赖点云轨迹，PLC 信号接入仍在规划中
- 导航栅格需要进一步降低 unknown 区域比例
- objects 层边缘锐度仍需通过后处理和局部精配准优化

## 许可证

MIT License
