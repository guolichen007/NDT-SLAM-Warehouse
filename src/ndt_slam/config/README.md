# LiDAR SLAM2 参数配置说明

## 概述

本 SLAM 系统通过 YAML 配置文件统一管理所有参数，无需修改代码即可调整系统行为。

## 配置文件位置

- **源文件**: `lidar_slam2/config/slam_params.yaml`
- **安装位置**: `/tmp/lidar_slam2_install/share/lidar_slam2/config/slam_params.yaml`

## 使用方法

### 方法 1：使用 launch 文件（推荐）

```bash
source /tmp/lidar_slam2_install/setup.bash
ros2 launch lidar_slam2 slam_launch.py
```

### 方法 2：直接运行节点

```bash
source /tmp/lidar_slam2_install/setup.bash
ros2 run lidar_slam2 lidar_slam2_node --ros-args --params-file /tmp/lidar_slam2_install/share/lidar_slam2/config/slam_params.yaml
```

### 方法 3：命令行覆盖参数

```bash
source /tmp/lidar_slam2_install/setup.bash
ros2 run lidar_slam2 lidar_slam2_node --ros-args \
  --params-file /tmp/lidar_slam2_install/share/lidar_slam2/config/slam_params.yaml \
  -p odom_voxel_size:=0.2 \
  -p odom_max_iterations:=100
```

## 参数说明

### 基本配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `use_sim_time` | bool | false | 使用仿真时间 |
| `pointcloud_topic` | string | "/globalmap" | 输入点云话题 |
| `odom_topic` | string | "/odom" | 输出里程计话题 |
| `map_topic` | string | "/map" | 输出地图话题 |
| `current_cloud_topic` | string | "/current_cloud" | 当前帧点云话题 |

### 坐标系配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `base_frame` | string | "base_link" | 机器人基坐标系 |
| `odom_frame` | string | "odom" | 里程计坐标系 |
| `map_frame` | string | "map" | 地图坐标系 |

### 里程计参数 (OdometryNode)

**适用于室内大场景（20 米范围）的推荐配置已标注⭐**

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `odom_voxel_size`⭐ | float | 0.3 | 体素滤波大小（米） |
| `odom_max_iterations`⭐ | int | 80 | ICP 最大迭代次数 |
| `odom_max_correspondence_distance`⭐ | float | 0.8 | ICP 最大对应距离（米） |
| `odom_convergence_threshold`⭐ | float | 0.0005 | ICP 收敛阈值 |
| `odom_max_valid_distance`⭐ | float | 25.0 | 最大有效距离（米） |
| `odom_min_valid_distance` | float | 0.5 | 最小有效距离（米） |
| `odom_min_neighbors`⭐ | int | 3 | 离群点过滤最少邻居数 |
| `odom_neighbor_search_radius`⭐ | float | 0.3 | 邻居搜索半径（米） |
| `odom_use_corner_extraction`⭐ | bool | true | 启用角点特征提取 |
| `odom_corner_curvature_threshold`⭐ | float | 0.12 | 角点曲率阈值 |

### 建图参数 (MappingNode)

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `map_voxel_size` | float | 0.2 | 地图体素滤波大小（米） |
| `max_map_size` | float | 200.0 | 最大地图范围（米） |
| `use_voxel_filter` | bool | true | 使用体素滤波 |
| `map_update_interval` | int | 10 | 地图更新间隔（帧数） |

### 里程计协方差参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `position_covariance` | float | 0.1 | 位置协方差（发布到里程计消息） |
| `orientation_covariance` | float | 0.1 | 姿态协方差（发布到里程计消息） |

### 发布配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `publish_odom_tf` | bool | true | 发布 TF 变换 |
| `publish_debug_clouds` | bool | false | 发布调试点云 |

## 场景配置示例

### 室内大场景（20 米范围）⭐推荐

```yaml
/**:
  ros__parameters:
    # 里程计参数
    odom_voxel_size: 0.3
    odom_max_iterations: 80
    odom_max_correspondence_distance: 0.8
    odom_convergence_threshold: 0.0005
    odom_max_valid_distance: 25.0
    odom_min_valid_distance: 0.5
    odom_min_neighbors: 3
    odom_neighbor_search_radius: 0.3
    odom_use_corner_extraction: true
    odom_corner_curvature_threshold: 0.12
    
    # 建图参数
    map_voxel_size: 0.2
    max_map_size: 200.0
```

### 高精度模式（计算量大）

```yaml
/**:
  ros__parameters:
    odom_voxel_size: 0.2
    odom_max_iterations: 100
    odom_max_correspondence_distance: 0.6
    odom_convergence_threshold: 0.0001
    odom_use_corner_extraction: true
    odom_corner_curvature_threshold: 0.1
```

### 高速模式（精度略低）

```yaml
/**:
  ros__parameters:
    odom_voxel_size: 0.4
    odom_max_iterations: 60
    odom_max_correspondence_distance: 1.0
    odom_use_corner_extraction: false
```

### 室外大场景

```yaml
/**:
  ros__parameters:
    odom_voxel_size: 0.5
    odom_max_valid_distance: 150.0
    odom_min_neighbors: 5
    odom_neighbor_search_radius: 0.5
    odom_use_corner_extraction: false
```

## 调试技巧

### 1. 查看参数

```bash
# 列出所有参数
ros2 param dump /lidar_slam2_node

# 查看特定参数
ros2 param get /lidar_slam2_node odom_voxel_size
```

### 2. 动态调整参数

```bash
# 运行时调整参数
ros2 param set /lidar_slam2_node odom_voxel_size 0.2
ros2 param set /lidar_slam2_node odom_max_iterations 100
```

### 3. 监控日志

```bash
# 查看详细日志
tail -f /tmp/ros_log/*.log

# 或实时查看节点输出
ros2 launch lidar_slam2 slam_launch.py 2>&1 | grep -E "ICP|预处理|角点"
```

### 4. 性能分析

关注日志中的关键信息：
- `[预处理] 原始点数：XXXXX` - 原始点云数量
- `[远点过滤] 距离过滤后点数：XXXXX` - 过滤后点数
- `[角点提取] 提取角点数量：XXXX` - 特征点数量
- `[ICP] 匹配完成：收敛=1, 迭代=XX/80, 适配分数=X.XXXX, 耗时=XXms` - ICP 性能

## 常见问题

### Q1: ICP 经常不收敛怎么办？

**A**: 尝试以下调整：
1. 增大 `odom_max_correspondence_distance` 到 1.0
2. 增加 `odom_max_iterations` 到 100
3. 减小 `odom_voxel_size` 到 0.2
4. 检查点云质量，是否有足够的特征

### Q2: 位姿估计漂移大？

**A**: 
1. 启用角点提取：`odom_use_corner_extraction:=true`
2. 降低 `odom_corner_curvature_threshold` 到 0.1
3. 减小 `odom_convergence_threshold` 到 0.0001

### Q3: 处理速度太慢？

**A**:
1. 禁用角点提取：`odom_use_corner_extraction:=false`
2. 增大 `odom_voxel_size` 到 0.4-0.5
3. 减少 `odom_max_iterations` 到 50-60

## 修改配置文件后的操作

修改 `config/slam_params.yaml` 后，需要重新编译安装：

```bash
cd /tmp/lidar_slam2_src/build
make install
```

或者直接修改已安装的文件：
```bash
sudo nano /tmp/lidar_slam2_install/share/lidar_slam2/config/slam_params.yaml
```
