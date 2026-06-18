# 故障排查指南

> 常见问题、错误信息和解决方案

---

## 常见问题快速索引

| 问题类型 | 页码 |
|---------|------|
| 点云回调不触发 | [#1](#1-点云回调不触发) |
| 里程计漂移严重 | [#2](#2-里程计漂移严重) |
| 跟踪丢失后无法重定位 | [#3](#3-跟踪丢失后无法重定位) |
| 系统崩溃/段错误 | [#4](#4-系统崩溃段错误) |
| TF 坐标错误 | [#5](#5-tf-坐标错误) |
| RViz 无法显示地图 | [#6](#6-rviz-无法显示地图) |
| 闭环检测失败 | [#7](#7-闭环检测失败) |
| 图优化后地图不更新 | [#8](#8-图优化后地图不更新) |

---

## 1. 点云回调不触发

### 症状
- 系统启动后没有点云数据处理
- 没有里程计输出

### 可能原因

1. **点云话题名称不匹配**
   ```bash
   # 检查点云话题
   rostopic list | grep -E "point|cloud|scan"
   ```

2. **点云数据格式错误**
   - 需要 `sensor_msgs/PointCloud2` 格式

3. **数据源没有发布**

### 解决方案

```bash
# 1. 查看可用的点云话题
rostopic list | grep -E "point|cloud|scan"

# 2. 确认点云话题正在发布数据
rostopic hz /your_pointcloud_topic

# 3. 在启动文件中 remap 点云话题
roslaunch lidar_slam2 slam_launch.launch pointcloud_topic:=/your_topic
```

---

## 2. 里程计漂移严重

### 症状
- 机器人移动后，里程计位置与实际位置偏差大
- 轨迹有明显漂移

### 可能原因

1. **激光雷达质量问题**
   - 噪点过多
   - 分辨率低

2. **参数配置不当**
   - `max_range` 太小
   - `voxel_size` 太大

3. **环境特征少**
   - 走廊、隧道等低特征环境
   - 开阔场地

### 解决方案

```yaml
# 调整参数
data:
  max_range: 50.0          # 增大感知范围
  min_range: 0.5

mapping:
  voxel_size: 0.2           # 减小体素大小提高精度
```

或者：

```yaml
# 室内小场景
data:
  max_range: 20.0
  min_range: 0.3
  voxel_size: 0.15
```

---

## 3. 跟踪丢失后无法重定位

### 症状
- 跟踪丢失后触发重定位
- 重定位返回的位姿不正确
- 系统崩溃或位置飞掉

### 可能原因

1. **重定位算法问题**
   - 没有足够的关键帧
   - Scan Context 匹配失败
   - ICP 配准失败

2. **关键帧数据问题**
   - 关键帧点云损坏
   - 关键帧位姿错误

3. **Sophus 库报错**
   - 旋转矩阵不是正交的

### 解决方案

```bash
# 查看日志中的详细信息
rostopic echo /rosout | grep -i "relocal"
```

**检查关键帧数量**：
```bash
# 确认关键帧正在添加
rostopic echo /rosout | grep -i "keyframe"
```

**如果遇到 Sophus 错误**：
```
[SOPHUS_ENSURE] R is not orthogonal
```
这是外参矩阵的旋转部分不满足正交性，需要在配置文件中修正外参。

---

## 4. 系统崩溃/段错误

### 症状
- 程序突然退出
- 显示 `Segmentation fault` 或 `process has died`

### 可能原因

1. **点云数据问题**
   - 包含 NaN 或 Inf 值
   - 空点云
   - 点云格式错误

2. **内存问题**
   - 内存不足
   - 内存泄漏

3. **TF 配置错误**
   - 坐标系循环
   - 无效的坐标系

### 解决方案

```bash
# 1. 检查点云数据
rostopic echo /your_pointcloud_topic | head -n 20

# 2. 查看 dmesg 获取崩溃信息
dmesg | tail -n 50

# 3. 使用 debug 模式运行
roslaunch lidar_slam2 slam_launch.launch
# 然后在另一个终端查看详细日志
rostopic echo /rosout
```

**添加数据验证代码**（临时）：
```cpp
// 在回调中添加
if (cloud->empty()) {
    ROS_WARN("Empty point cloud received!");
    return;
}
```

---

## 5. TF 坐标错误

### 症状
- RViz 中显示的机器人位置不正确
- TF 树报错

### 可能原因

1. **外参配置错误**
   - `lidar2base_extrinsic` 参数不正确
   - 静态 TF 没有正确发布

2. **坐标系配置错误**
   - `base_frame`、`odom_frame` 等参数错误

### 解决方案

**检查当前 TF 树**：
```bash
# 方法1：使用 tf2_tools
rosrun tf2_tools view_frames.py

# 方法2：实时查看
rostopic echo /tf_static
```

**修正外参**：
```yaml
# 配置文件中修正外参矩阵
lidar2base_extrinsic: [r11, r12, r13, tx,
                       r21, r22, r23, ty,
                       r31, r32, r33, tz,
                       0,   0,   0,   1]
```

**添加静态 TF**（如果需要）：
```xml
<!-- 在 launch 文件中添加 -->
<node pkg="tf" type="static_transform_publisher" 
      name="rslidar_to_base_link"
      args="x y z roll pitch yaw rslidar base_link 100"/>
```

---

## 6. RViz 无法显示地图

### 症状
- `/map` 话题没有数据
- RViz 中无法看到地图点云

### 可能原因

1. **话题名称不匹配**
   - RViz 订阅的话题与发布的不一致

2. **地图没有重建**
   - 关键帧数量为 0
   - `global_map_` 为空

3. **发布频率低**
   - 地图更新间隔太大

### 解决方案

```bash
# 1. 检查地图话题
rostopic hz /map

# 2. 在 RViz 中：
#    - 添加 PointCloud2 显示
#    - 将 Topic 设置为 /map
#    - 设置 Frame 为 map 或 odom
```

**调整地图更新参数**：
```yaml
map_update_interval: 1  # 每帧更新
```

---

## 7. 闭环检测失败

### 症状
- 走了一圈回到原点没有检测到闭环
- 日志显示 "No loop closure found"

### 可能原因

1. **Scan Context 参数不当**
   - `similarity_threshold` 太高
   - `spatial_search_radius` 太小

2. **关键帧太少**
   - 移动距离不够
   - 关键帧阈值太大

3. **环境特征不明显**
   - 重复场景
   - 低纹理环境

### 解决方案

```yaml
# 调整 Scan Context 参数
scan_context:
  similarity_threshold: 0.7   # 降低阈值
  spatial_search_radius: 10.0 # 增大搜索范围

# 调整关键帧参数
keyframe:
  translation_threshold: 1.0   # 减小阈值
  rotation_threshold: 10.0
```

---

## 8. 图优化后地图不更新

### 症状
- 闭环检测成功
- 但全局点云没有重新拼接
- 地图还是原来的位置

### 可能原因

1. **rebuildGlobalMap 函数没有调用**
   - 代码逻辑问题

2. **关键帧位姿没有更新**
   - `updateKeyFramePoses` 没有正确执行

3. **发布频率问题**
   - 地图发布间隔太大

### 解决方案

```bash
# 查看日志
rostopic echo /rosout | grep -i "rebuild\|optim"
```

**检查代码**：
确保以下调用链正确执行：
1. `pose_graph_optimizer_.optimize()` 返回 true
2. `loop_closure_detector_.updateKeyFramePoses()` 被调用
3. `rebuildGlobalMap()` 被调用

---

## 调试技巧

### 查看实时日志

```bash
# 查看所有日志
rostopic echo /rosout

# 只查看 WARNING 及以上级别
rostopic echo /rosout | grep -E "WARN|ERROR|FATAL"

# 查看特定模块的日志
rostopic echo /rosout | grep -i "slam\|icp\|loop"
```

### 可视化调试

```bash
# 查看 TF 树
rosrun rqt_tf_tree rqt_tf_tree

# 查看点云
rviz 或 cloud_viewer

# 监控话题
rostopic hz /odom
rostopic hz /map
```

### 运行时参数调整

```bash
# 使用 rqt_reconfigure 动态调整参数（如果启用了动态配置）
rosrun rqt_reconfigure rqt_reconfigure

# 或者使用 rosparam 调整
rosparam set /lidar_slam2_node/scan_context/similarity_threshold 0.7
```

---

## 性能优化

### 内存使用

```yaml
# 限制关键帧数量
keyframe:
  max_keyframes: 500

# 增大降采样
mapping:
  voxel_size: 0.5
```

### CPU 占用

```yaml
# 减少闭环检测频率
loop_detection_interval: 50

# 减少线程数
registration:
  max_num_threads: 2

num_worker_threads: 0  # 同步模式
```

---

## 报告问题

当报告问题时，请提供以下信息：

1. **系统环境**
   - ROS 版本
   - Ubuntu 版本
   - 硬件配置

2. **问题描述**
   - 复现步骤
   - 期望行为 vs 实际行为

3. **日志输出**
   - 相关 ROS 日志
   - dmesg 输出（如果崩溃）

4. **配置参数**
   - 使用的 YAML 配置文件
   - 修改过的参数

---

**文档最后更新**: 2026-04-24
