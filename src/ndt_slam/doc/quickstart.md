# 快速启动指南

> 推荐首先阅读本指南了解系统启动、基本操作和常见问题解决方法

---

## 启动流程

### 前置环境检查

在启动系统前，请确保以下条件已满足：

- ROS Noetic 已安装
- 所有依赖项已安装
- 项目已成功编译

```bash
# 检查 ROS 是否安装
rosversion -d

# 检查依赖项是否安装
dpkg -l | grep -E "libpcl|libeigen|libglfw"

# 检查项目是否编译成功
cd ~/lidarslam_ws
catkin build lidar_slam2
```

---

### 步骤 1: 激活工作空间

```bash
cd ~/lidarslam_ws
source devel/setup.bash
```

### 步骤 2: 启动 SLAM 系统

```bash
# 基本启动
roslaunch lidar_slam2 slam_launch.launch

# 使用自定义点云话题
roslaunch lidar_slam2 slam_launch.launch pointcloud_topic:=/velodyne_points

# 关闭 RViz（如果只需要命令行运行）
roslaunch lidar_slam2 slam_launch.launch
# 修改 launch 文件，注释掉 rviz 节点
```

### 步骤 3: 验证运行

1. **检查节点状态**
   ```bash
   rosnode list
   # 应该看到 /lidar_slam2_node 节点
   ```

2. **检查发布的话题**
   ```bash
   rostopic list
   # 应该看到 /odom, /current_pose, /map, /current_cloud 等话题
   ```

3. **检查可视化窗口**
   - 系统启动后会自动弹出 OpenGL 可视化窗口
   - 窗口中应该显示点云数据和轨迹

4. **查看里程计数据**
   ```bash
   rostopic echo /odom | head -20
   ```

5. **查看当前位姿**
   ```bash
   rostopic echo /current_pose
   ```

---

## 基本使用

### 1. 启动系统

系统启动后会：
- 订阅点云话题 `/points_raw`
- 发布里程计 `/odom`
- 发布位姿 `/current_pose`
- 发布全局地图 `/map`
- 发布当前帧点云 `/current_cloud`
- 发布 TF 变换

### 2. 查看数据

```bash
# 查看所有相关话题
rostopic list | grep -E "odom|pose|map|cloud"

# 查看里程计频率
rostopic hz /odom

# 查看地图大小
rostopic echo /map | grep height
```

### 3. 使用 RViz 可视化

如果需要使用 RViz：
```bash
# 启动 RViz
rviz

# 添加显示项：
# 1. PointCloud2 - 订阅 /map 话题
# 2. PointCloud2 - 订阅 /current_cloud 话题  
# 3. Odometry - 订阅 /odom 话题
# 4. TF - 显示坐标系
```

---

## 可视化控制

### 相机控制

- **鼠标左键拖动**：旋转相机
- **鼠标右键拖动**：平移相机
- **滚轮**：缩放
- **WASD 键**：移动视角中心
- **ESC 键**：关闭窗口

### 可视化内容

- **点云显示**：实时显示处理后的点云
- **轨迹跟踪**：显示机器人运动轨迹
- **坐标系**：显示关键坐标系

---

## 常用命令

### 重置 SLAM 系统

```bash
rosservice call /lidar_slam2_node/reset
```

### 触发重定位

```bash
rosservice call /lidar_slam2_node/relocalize
```

### 保存地图

```bash
# 需要先创建保存地图的服务请求
rosservice call /lidar_slam2_node/save_map "{filename: '/tmp/map.pcd'}"
```

### 查看节点信息

```bash
rosnode info /lidar_slam2_node
```

### 查看参数列表

```bash
rosparam list /lidar_slam2_node
```

### 动态调整参数

```bash
# 调整 KISS-ICP 体素大小
rosparam set /lidar_slam2_node/mapping/voxel_size 0.3

# 调整最大有效距离
rosparam set /lidar_slam2_node/data/max_range 80.0

# 调整闭环检测相似度阈值
rosparam set /lidar_slam2_node/scan_context/similarity_threshold 0.7
```

---

## 坐标系说明

系统使用以下坐标系：

| 坐标系 | 说明 |
|-------|------|
| `rslidar` | 激光雷达坐标系 |
| `base_link` | 机器人基座坐标系 |
| `odom` | 里程计坐标系 |

**TF 树**：
- `rslidar → base_link`：静态外参（雷达固定在车上）
- `base_link → odom`：动态里程计（机器人移动时变化）

---

## 地图构建模式

系统现在使用**关键帧建图**模式：

1. **关键帧添加**：只有当累积运动超过阈值时才添加关键帧
2. **地图更新**：关键帧被添加到全局地图
3. **图优化**：闭环检测后触发位姿图优化
4. **地图重建**：图优化后使用优化后的位姿重新拼接全局点云

---

## 常见问题排查

### 问题 1: 点云回调不触发

**解决方案**:
1. 检查点云话题是否正确配置
   ```bash
   rostopic list | grep -E "point|cloud|scan"
   ```
2. 确认点云数据是否正常发布
   ```bash
   rostopic hz /points_raw
   ```
3. 检查 launch 文件中的 remapping 配置

### 问题 2: 里程计漂移严重

**解决方案**:
1. 减小 `mapping.voxel_size` 参数，提高点云精度
   ```bash
   rosparam set /lidar_slam2_node/mapping/voxel_size 0.2
   ```
2. 调整 `data.max_range`，过滤掉远处的噪声点
   ```bash
   rosparam set /lidar_slam2_node/data/max_range 80.0
   ```
3. 检查激光雷达的安装位置和校准

### 问题 3: 系统崩溃

**解决方案**:
1. 检查是否有 NaN 值导致的错误
2. 确认点云数据格式正确
3. 检查硬件资源是否充足

### 问题 4: 跟踪丢失频繁

**解决方案**:
1. 检查环境特征是否充足
2. 调整跟踪阈值
   ```bash
   rosparam set /lidar_slam2_node/tracking/inlier_ratio_threshold 0.3
   ```
3. 检查激光雷达数据质量
4. 降低机器人运动速度

### 问题 5: 闭环检测失败

**解决方案**:
1. 降低相似度阈值
   ```bash
   rosparam set /lidar_slam2_node/scan_context/similarity_threshold 0.7
   ```
2. 调整关键帧阈值
   ```bash
   rosparam set /lidar_slam2_node/keyframe/translation_threshold 1.0
   ```

### 问题 6: 图优化后地图不更新

**解决方案**:
1. 检查日志中是否有 "Global map rebuilt" 信息
   ```bash
   rostopic echo /rosout | grep -i rebuild
   ```
2. 确认闭环检测和优化是否成功执行

---

## 性能优化

### 实时性能提升

1. **减少点云密度**
   ```bash
   rosparam set /lidar_slam2_node/mapping/voxel_size 0.8
   ```

2. **降低地图更新频率**
   ```bash
   rosparam set /lidar_slam2_node/map_update_interval 20
   ```

3. **关闭调试点云**
   ```bash
   rosparam set /lidar_slam2_node/publish_debug_clouds false
   ```

4. **减少闭环检测频率**
   ```bash
   rosparam set /lidar_slam2_node/loop_detection_interval 50
   ```

### 精度提升

1. **减小体素大小**
   ```bash
   rosparam set /lidar_slam2_node/mapping/voxel_size 0.15
   ```

2. **降低相似度阈值**（增加闭环检测机会）
   ```bash
   rosparam set /lidar_slam2_node/scan_context/similarity_threshold 0.6
   ```

---

## 下一步

- 阅读 [系统架构.md](architecture.md) 了解系统设计原理
- 阅读 [配置参数详解.md](configuration.md) 了解参数调优
- 阅读 [故障排查.md](troubleshooting.md) 了解常见问题解决方案
- 阅读 [API文档.md](api.md) 了解接口详情
