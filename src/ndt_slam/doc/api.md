# 对外接口

本文档描述 NDT-SLAM 对外的 ROS 话题、服务、消息类型和主控集成合同。

内部实现类不在本文档范围。

## 输入话题

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/merged_points` | `sensor_msgs/PointCloud2` | 合并后点云（由外部 pointcloud_merger 提供），YAML 参数 `pointcloud_topic` 配置 |
| `/gravity` | `std_msgs/Float32` 或同类 | 称重/Gravity 信号，YAML 参数 `gravity_topic` 配置 |

## 输出话题

### 定位与地图

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | 运行位姿 |
| `/ndt_slam/runtime_path` | `nav_msgs/Path` | 实时轨迹 |
| `/current_pose` | `geometry_msgs/PoseStamped` | 当前位姿 |
| `/map` | `sensor_msgs/PointCloud2` | registration 层 |
| `/display_map` | `sensor_msgs/PointCloud2` | 全量显示层 |
| `/display_map_ground` | `sensor_msgs/PointCloud2` | 地面层 |
| `/display_map_objects` | `sensor_msgs/PointCloud2` | 原始静态物体层 |
| `/display_map_objects_clean` | `sensor_msgs/PointCloud2` | 清理后静态物体层 |
| `/mapping_current_cloud` | `sensor_msgs/PointCloud2` | 当前帧输入点云（经近场过滤后） |

### 吊物可视化

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/cargo_core_bbox_marker` | `visualization_msgs/Marker` | 正式冻结形状、实时移动的吊物框 |
| `/cargo_tight_box_marker` | `visualization_msgs/Marker` | 兼容框（使用相同刚体几何） |
| `/cargo_warning_zone_marker` | `visualization_msgs/Marker` | 3m/5m 方向一致告警区域 |
| `/cargo_warning_obstacle_marker` | `visualization_msgs/Marker` | 障碍物标记 |
| `/cargo_warning_text` | `visualization_msgs/Marker` | 告警文字 |

### 安全输出（主控必须订阅）

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/cargo_avoidance/safety_status` | `lidar_slam2_msgs/CargoSafetyStatus` | 正式安全状态（schema v6），含安全码、距离、净空、几何来源、时间戳 |
| `/cargo_avoidance/status_code` | `std_msgs/Int32` | Heartbeat 简码输出（14/17/18/30-35） |

### 吊物诊断

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/cargo_recognition_status` | `lidar_slam2_msgs/CargoRecognitionStatus` | 货物识别状态 |
| `/cargo_swing_status` | `lidar_slam2_msgs/CargoSwingStatus` | 货物摆动状态 |
| `/cargo_bottom_estimate` | `lidar_slam2_msgs/CargoBottomEstimate` | 货物底部估计 |

## 服务

| 服务名 | 服务类型 | 功能 |
|---|---|---|
| `~/reset` | `std_srvs/Empty` | 重置 SLAM 系统 |
| `~/set_pose` | `std_srvs/Empty` | 设置当前位姿 |
| `~/relocalize` | `std_srvs/Empty` | 触发重定位 |
| `~/save_map` | `lidar_slam2_msgs/SaveMap` | 保存地图 |
| `~/load_map` | `lidar_slam2_msgs/LoadMap` | 加载地图 |
| `~/load_map_session` | `lidar_slam2_msgs/LoadMapSession` | 加载地图会话 |
| `~/rebuild_map` | `std_srvs/Empty` | 重建全局地图 |

## 消息类型

### CargoSafetyStatus（schema v6）

正式安全输出消息，定义于 `src/lidar_slam2_msgs/msg/CargoSafetyStatus.msg`。

| 字段 | 类型 | 说明 |
|---|---|---|
| `header` | `std_msgs/Header` | 时间戳为证据评估时间 |
| `status_code` | `int32` | 14/17/18/30-35 |
| `min_distance_m` | `float64` | 最近障碍距离 |
| `vertical_clearance_m` | `float64` | 垂直净空 |
| `geometry_source` | `int32` | 几何来源（Formal/Degraded） |
| `reason` | `string` | 决策原因 |

### 其他消息

完整消息定义见 `src/lidar_slam2_msgs/msg/` 目录。

## TF / 坐标系

| 坐标系 | 说明 |
|---|---|
| `map` | 全局固定坐标系，RViz Fixed Frame |
| `odom` | 里程计坐标系 |
| `base_link` | 天车本体坐标系 |

## 主控集成合同

### 安全输出

类型化输出 `/cargo_avoidance/safety_status`（`CargoSafetyStatus` schema v6）是主控程序的安全权威输入。

Heartbeat 简码 `/cargo_avoidance/status_code`（`std_msgs/Int32`）供兼容显示和冗余心跳。

### 主控侧要求

下游控制器必须：
- 将 Code 30-35 视为非 CLEAR（不安全的未知状态）
- 不能因缺少 17/18 而推断为 CLEAR
- 对安全状态流实现独立超时/Watchdog
- 保持独立的安全逻辑

### 已验证集成

Code 18 → 外部主控程序 → S3 语音告警链路已通过现场验证。

主控程序不在本仓库中维护。

## 关键参数入口

| 文件 | 说明 |
|---|---|
| `src/ndt_slam/config/live_longterm_mapping.yaml` | 生产配置（传感器、建图、吊物、安全） |
| `src/ndt_slam/config/merger_params.yaml` | 点云合并参数 |
| `src/ndt_slam/config/slam_params.yaml` | SLAM 算法参数 |
| `src/ndt_slam/config/server_monitor.yaml` | 服务器监控配置 |
