# API 文档

> 类、函数、接口定义

---

## 核心类

### 1. SlamNode

主 SLAM 节点，集成里程计、建图和闭环检测。

**头文件**: `include/lidar_slam2/SlamNode.hpp`

#### 公共方法

| 方法名 | 返回类型 | 说明 |
|--------|---------|------|
| `SlamNode` | constructor | 构造函数，初始化节点 |
| `~SlamNode` | destructor | 析构函数 |
| `run` | void | 启动节点主循环 |
| `reset` | void | 重置 SLAM 系统 |

#### 消息回调

| 方法名 | 参数 | 说明 |
|--------|------|------|
| `pointCloudCallback` | `const sensor_msgs::PointCloud2::ConstPtr&` | 点云数据回调 |

#### 服务

| 方法名 | 参数 | 说明 |
|--------|------|------|
| `resetService` | `std_srvs::Empty::Request&, std_srvs::Empty::Response&` | 重置服务 |
| `setPoseService` | `std_srvs::Empty::Request&, std_srvs::Empty::Response&` | 设置位姿服务 |
| `relocalizeService` | `std_srvs::Empty::Request&, std_srvs::Empty::Response&` | 重定位服务 |
| `saveMapService` | `lidar_slam2_msgs::SaveMap::Request&, lidar_slam2_msgs::SaveMap::Response&` | 保存地图服务 |

#### 私有方法

| 方法名 | 返回类型 | 说明 |
|--------|---------|------|
| `loadParameters` | void | 加载参数 |
| `initialize` | void | 初始化发布者、订阅者、服务 |
| `publishOdometry` | void | 发布里程计 |
| `publishMap` | void | 发布地图 |
| `publishCurrentCloud` | void | 发布当前帧点云 |
| `addFrameToMap` | void | 添加帧到地图（已修改为只用关键帧） |
| `addKeyFrameToLoopClosure` | void | 添加关键帧到闭环检测 |
| `rebuildGlobalMap` | void | 重建全局地图（新增） |
| `updatePoseFromLoopClosure` | void | 更新位姿（重定位后） |
| `processLoopClosure` | void | 处理闭环检测 |
| `performRelocalization` | void | 执行重定位 |
| `processingWorker` | void | 处理线程工作函数 |

---

### 2. OdometryNode

里程计节点，基于 KISS-ICP。

**头文件**: `include/lidar_slam2/OdometryNode.hpp`

#### 公共方法

| 方法名 | 返回类型 | 说明 |
|--------|---------|------|
| `OdometryNode` | constructor | 构造函数 |
| `~OdometryNode` | destructor | 析构函数 |
| `run` | void | 启动节点 |

#### 消息回调

| 方法名 | 参数 | 说明 |
|--------|------|------|
| `pointCloudCallback` | `const sensor_msgs::PointCloud2::ConstPtr&` | 点云回调 |

#### 服务

| 方法名 | 参数 | 说明 |
|--------|------|------|
| `resetService` | `std_srvs::Empty::Request&, std_srvs::Empty::Response&` | 重置里程计 |

---

### 3. MappingNode

建图节点。

**头文件**: `include/lidar_slam2/MappingNode.hpp`

#### 公共方法

| 方法名 | 返回类型 | 说明 |
|--------|---------|------|
| `MappingNode` | constructor | 构造函数 |
| `~MappingNode` | destructor | 析构函数 |
| `run` | void | 启动节点 |

---

### 4. LoopClosureNode

闭环检测节点。

**头文件**: `include/lidar_slam2/LoopClosureNode.hpp`

#### 公共方法

| 方法名 | 返回类型 | 说明 |
|--------|---------|------|
| `LoopClosureNode` | constructor | 构造函数 |
| `~LoopClosureNode` | destructor | 析构函数 |
| `run` | void | 启动节点 |

#### 服务

| 方法名 | 参数 | 说明 |
|--------|------|------|
| `relocalizeService` | `std_srvs::Empty::Request&, std_srvs::Empty::Response&` | 重定位服务 |

---

### 5. LoopClosureDetector

闭环检测核心算法。

**头文件**: `include/lidar_slam2/LoopClosureDetector.hpp`

#### 公共方法

| 方法名 | 参数 | 说明 |
|--------|------|------|
| `addKeyFrame` | `const Sophus::SE3d&, pcl::PointCloud<pcl::PointXYZ>::Ptr, const ros::Time&` | 添加关键帧 |
| `globalRelocalization` | `pcl::PointCloud<pcl::PointXYZ>::Ptr` | 全局重定位 |
| `getKeyFrames` | - | 获取所有关键帧 |
| `updateKeyFramePoses` | `const std::vector<KeyFrame>&` | 更新关键帧位姿（新增） |

#### 私有方法

| 方法名 | 返回类型 | 说明 |
|--------|---------|------|
| `isKeyFrame` | bool | 判断是否关键帧 |
| `detectLoopClosure` | int | 检测闭环 |
| `refinePose` | Sophus::SE3d | ICP 精配准 |

---

### 6. PoseGraphOptimizer

位姿图优化器。

**头文件**: `include/lidar_slam2/PoseGraphOptimizer.hpp`

#### 公共方法

| 方法名 | 参数 | 说明 |
|--------|------|------|
| `addOdometryEdge` | `int, int, const Sophus::SE3d&, const Eigen::MatrixXd&` | 添加里程计边 |
| `addLoopEdge` | `int, int, const Sophus::SE3d&, const Eigen::MatrixXd&` | 添加闭环边 |
| `optimize` | `int` | 执行优化，返回是否成功 |
| `updateKeyFramePoses` | `std::vector<KeyFrame>&` | 更新关键帧位姿 |

---

### 7. KeyFrame

关键帧数据结构。

**头文件**: `include/lidar_slam2/KeyFrame.hpp`

#### 成员变量

| 变量名 | 类型 | 说明 |
|--------|------|------|
| `id_` | `uint64_t` | 关键帧 ID |
| `stamp_` | `ros::Time` | 时间戳 |
| `pose_` | `Sophus::SE3d` | 位姿 |
| `cloud_` | `pcl::PointCloud<pcl::PointXYZ>::Ptr` | 点云 |
| `scan_context_` | `Eigen::MatrixXd` | Scan Context 描述子 |

---

### 8. ScanContext

Scan Context 描述子生成与匹配。

**头文件**: `include/lidar_slam2/ScanContext.hpp`

#### 公共方法

| 方法名 | 参数 | 说明 |
|--------|------|------|
| `computeScanContext` | `pcl::PointCloud<pcl::PointXYZ>::Ptr` | 计算 Scan Context |
| `getRingKey` | - | 获取环键 |
| `getSectorKey` | - | 获取扇区键 |
| `computeSimilarity` | `const Eigen::MatrixXd&, const Eigen::MatrixXd&` | 计算相似度 |

---

### 9. Visualizer

3D 可视化工具。

**头文件**: `include/lidar_slam2/Visualizer.hpp`

#### 公共方法

| 方法名 | 参数 | 说明 |
|--------|------|------|
| `Visualizer` | constructor | 构造函数 |
| `~Visualizer` | destructor | 析构函数 |
| `run` | void | 启动可视化 |
| `addPointCloud` | `const std::string&, pcl::PointCloud<pcl::PointXYZ>::Ptr` | 添加点云 |
| `addTrajectory` | `const std::vector<Eigen::Vector3d>&` | 添加轨迹 |
| `updatePose` | `const Sophus::SE3d&` | 更新位姿 |

---

## 消息类型

### 输入

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/points_raw` | `sensor_msgs/PointCloud2` | 原始点云 |

### 输出

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/odom` | `nav_msgs::Odometry` | 里程计 |
| `/current_pose` | `geometry_msgs::PoseStamped` | 当前位姿 |
| `/map` | `sensor_msgs::PointCloud2` | 全局地图 |
| `/current_cloud` | `sensor_msgs::PointCloud2` | 当前帧点云 |
| `/tf` | `tf2_msgs::TFMessage` | 坐标变换 |

---

## 服务类型

| 服务名 | 服务类型 | 功能 |
|--------|---------|------|
| `~/reset` | `std_srvs::Empty` | 重置系统 |
| `~/set_pose` | `std_srvs::Empty` | 设置位姿 |
| `~/relocalize` | `std_srvs::Empty` | 触发重定位 |
| `~/save_map` | `lidar_slam2_msgs::SaveMap` | 保存地图 |

---

## 数据结构

### KeyFrame

```cpp
struct KeyFrame {
    uint64_t id_;
    ros::Time stamp_;
    Sophus::SE3d pose_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    Eigen::MatrixXd scan_context_;
};
```

### OdometryInfo

```cpp
struct OdometryInfo {
    Sophus::SE3d pose_;
    double inlier_ratio_;
    double mean_distance_;
    double model_deviation_;
    bool is_tracking_lost_;
};
```

---

## 常量定义

### 默认话题名称

| 常量 | 值 |
|------|-----|
| `kDefaultPointCloudTopic` | `"/points_raw"` |
| `kDefaultOdomTopic` | `"/odom"` |
| `kDefaultMapTopic` | `"/map"` |
| `kDefaultPoseTopic` | `"/current_pose"` |

### 默认坐标系

| 常量 | 值 |
|------|-----|
| `kDefaultBaseFrame` | `"base_link"` |
| `kDefaultOdomFrame` | `"odom"` |
| `kDefaultMapFrame` | `"map"` |

---

**下一步建议**: 关注系统更新日志，了解最新功能
