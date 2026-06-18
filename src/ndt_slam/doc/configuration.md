# 配置参数详解

> 配置文件 slam_params.yaml 中所有参数的详细说明

---

## 1. 话题配置

### 输入话题

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `pointcloud_topic` | string | `"/rs_150"` | 激光点云输入话题，支持 remap |

### 输出话题

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `odom_topic` | string | `"/odom"` | 里程计输出话题 |
| `map_topic` | string | `"/map"` | 全局地图点云话题 |
| `current_cloud_topic` | string | `"/mapping_current_cloud"` | 变换后的当前帧点云 |
| `pose_topic` | string | `"/current_pose"` | 当前位姿（PoseStamped 格式） |
| `mapping_pointcloud_topic` | string | `"/current_cloud"` | 对应里程计的原始点云 |

---

## 2. 坐标系配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `base_frame` | string | `"base_link"` | 机器人基座坐标系 |
| `odom_frame` | string | `"odom"` | SLAM 里程计输出坐标系 |
| `map_frame` | string | `"map"` | 全局地图坐标系 |
| `lidar_odom_frame` | string | `"odom_lidar"` | KISS-ICP 相对位姿参考坐标系 |

**坐标系说明**：
- `base_frame`：机器人本体坐标系，所有传感器数据转换到此坐标系
- `odom_frame`：SLAM 里程计输出的参考坐标系
- `map_frame`：全局点云地图发布的 frame_id
- `lidar_odom_frame`：KISS-ICP 计算的相对位姿的参考坐标系

---

## 3. 里程计参数

### 里程计模式

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `odom_mode` | int | `0` | 里程计模式：0=纯激光，1=纯视觉，2=松耦合 |
| `lidar_weight` | double | `0.3` | 激光里程计权重 |
| `visual_weight` | double | `0.7` | 视觉里程计权重 |

### KISS-ICP 核心参数

#### data（数据预处理）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `deskew` | bool | `False` | 是否去畸变（点云无时间戳时设为 False） |
| `max_range` | double | `16.0` | 最大有效距离，超出距离的点被过滤 |
| `min_range` | double | `0.5` | 最小有效距离 |

#### mapping（地图构建）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `voxel_size` | double | `0.3` | 体素降采样大小（米） |
| `max_points_per_voxel` | int | `20` | 每个体素最多保留的点数 |

#### adaptive_threshold（自适应阈值）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `initial_threshold` | double | `2.0` | 初始配准阈值 |
| `min_motion_th` | double | `0.1` | 最小运动阈值 |

#### registration（配准）

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `max_num_iterations` | int | `500` | ICP 最大迭代次数 |
| `convergence_criterion` | double | `0.0001` | 收敛阈值 |
| `max_num_threads` | int | `0` | 最大线程数，0 表示使用默认值 |

---

## 4. 建图参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `map_voxel_size` | double | `0.3` | 地图体素降采样大小 |
| `max_map_size` | double | `200.0` | 地图最大范围（米） |
| `use_voxel_filter` | bool | `true` | 是否使用体素滤波 |
| `map_update_interval` | int | `1` | 地图更新间隔（帧数） |
| `num_worker_threads` | int | `0` | 异步工作线程数，0 为同步模式 |

---

## 5. 跟踪质量检测

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `inlier_ratio_threshold` | double | `0.5` | 内点率阈值，低于此值判定为跟踪丢失 |
| `mean_distance_threshold` | double | `0.1` | 平均距离阈值，高于此值判定为跟踪丢失 |
| `model_deviation_threshold` | double | `0.2` | 模型偏差阈值，高于此值判定为跟踪丢失 |

**判断逻辑**：
- 当 `inlier_ratio < inlier_ratio_threshold` 时，跟踪丢失
- 或当 `mean_distance > mean_distance_threshold` 时，跟踪丢失
- 或当 `model_deviation > model_deviation_threshold` 时，跟踪丢失

---

## 6. 传感器参数

### 外参矩阵

| 参数名 | 类型 | 说明 |
|--------|------|------|
| `lidar2base_extrinsic` | 4x4 矩阵 | 激光雷达到机器人基座的外参变换 |
| `lidar2cam_extrinsic` | 4x4 矩阵 | 激光雷达到相机的外参变换 |

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `use_lidar2base_transform` | bool | `true` | 是否将点云从雷达坐标系转换到 base_link 坐标系 |

### 相机内参

| 参数名 | 类型 | 说明 |
|--------|------|------|
| `cam_intrinsic` | 9 元素数组 | 相机内参矩阵 [fx, 0, cx, 0, fy, cy, 0, 0, 1] |
| `cam_distort` | 5 元素数组 | 相机畸变系数 [k1, k2, p1, p2, k3] |
| `fx`, `fy` | double | 焦距 |
| `cx`, `cy` | double | 主点坐标 |
| `baseline` | double | 双目基线（米） |

---

## 7. 关键帧管理

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `translation_threshold` | double | `3.0` | 关键帧平移阈值（米），累积移动超过此值添加关键帧 |
| `rotation_threshold` | double | `20.0` | 关键帧旋转阈值（度），累积旋转超过此值添加关键帧 |
| `time_threshold` | double | `5.0` | 关键帧时间阈值（秒），超过此时间添加关键帧 |
| `max_keyframes` | int | `500` | 最大关键帧数量 |

**建议**：
- 增大阈值可减少关键帧数量，提高闭环检测速度
- 减小阈值可获得更密集的关键帧，提高闭环精度

---

## 8. 闭环检测参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `loop_detection_interval` | int | `20` | 闭环检测间隔（每 N 个关键帧检测一次） |

### Scan Context 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `num_rings` | int | `20` | 环的数量 |
| `num_sectors` | int | `60` | 扇区的数量 |
| `max_range` | double | `80.0` | 最大距离（米） |
| `spatial_search_radius` | double | `5.0` | 空间搜索半径（米） |
| `similarity_threshold` | double | `0.85` | 相似度阈值 |
| `translation_threshold` | double | `0.5` | 一致性检查平移阈值（米） |
| `rotation_threshold` | double | `15.0` | 一致性检查旋转阈值（度） |

**Scan Context 说明**：
Scan Context 是一种基于激光点云的全局位置描述子，将 360 度环境分成多个环和扇区，提取每个区域的点云高度统计信息作为描述子。

**参数调优建议**：
- `similarity_threshold`：增大可减少误检测，但可能漏掉正确的闭环
- `spatial_search_radius`：减小可避免频繁检测，但可能漏掉近距离的闭环
- `translation_threshold`、`rotation_threshold`：收紧可提高闭环精度

---

## 9. 发布配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `publish_odom_tf` | bool | `true` | 是否发布 odom 到 base_link 的 TF |
| `publish_debug_clouds` | bool | `false` | 是否发布调试点云 |

---

## 10. 其他参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `use_sim_time` | bool | `true` | 是否使用 ROS 仿真时间 |

---

## 参数调优指南

### 室外大场景

```yaml
data:
  max_range: 50.0          # 增大感知范围
  min_range: 1.0

keyframe:
  translation_threshold: 5.0   # 增大阈值减少关键帧
  rotation_threshold: 30.0

scan_context:
  max_range: 100.0           # 增大检测范围
  similarity_threshold: 0.9   # 提高阈值减少误检
```

### 室内小场景

```yaml
data:
  max_range: 20.0
  min_range: 0.3

keyframe:
  translation_threshold: 1.0   # 减小阈值增加关键帧
  rotation_threshold: 10.0

scan_context:
  max_range: 30.0
  similarity_threshold: 0.7
```

### 低特征环境（如走廊）

```yaml
data:
  max_range: 30.0

keyframe:
  translation_threshold: 2.0
  rotation_threshold: 15.0

scan_context:
  similarity_threshold: 0.6   # 降低阈值增加检测机会
  spatial_search_radius: 10.0  # 增大搜索范围
```

---

**下一步建议**: 阅读 [故障排查.md](troubleshooting.md) 了解常见问题解决方案
