## LiDAR SLAM2 系统功能分析报告

---

### 📦 已实现的功能

#### 1. **OdometryNode（里程计节点）**
| 功能 | 说明 |
|------|------|
| KISS-ICP 点云配准 | 基于自适应阈值的实时点云配准算法 |
| 点云预处理 | VoxelGrid滤波、去畸变（deskew） |
| 离群点过滤 | 基于半径搜索的过滤方法 |
| 跟踪质量监控 | 内点率、平均距离、模型偏差检测 |
| 自动重定位 | 跟踪丢失时触发全局重定位 |
| TF/里程计发布 | 支持坐标变换和里程计话题输出 |
| 服务接口 | `/reset` 重置, `/set_pose` 设置位姿 |

#### 2. **MappingNode（建图节点）**
| 功能 | 说明 |
|------|------|
| 增量式建图 | 实时累积点云构建全局地图 |
| 体素降采样 | 保持地图大小合理 |
| 点云坐标变换 | 统一到地图坐标系 |
| **异步处理（可选）** | 支持多线程异步点云处理，通过 `num_worker_threads` 配置 |
| **YAML 参数配置** | 所有参数可通过 `slam_params.yaml` 动态配置 |
| **地图保存/加载** | 通过 ROS2 服务保存和加载 PCD 文件 |

#### 3. **LoopClosureNode（回环检测节点）**
| 功能 | 说明 |
|------|------|
| ScanContext 回环检测 | 基于激光雷达描述子的回环识别 |
| ICP 精配准 | 回环候选帧的精细配准 |
| 关键帧管理 | 距离/角度/时间阈值判断，使用 deque 高效管理 |
| PoseGraph 优化 | 基于 g2o 的图优化 |
| 全局重定位 | 丢失后重新定位 |
| **YAML 参数配置** | Scan Context 和关键帧参数可通过 YAML 动态配置 |

#### 4. **Visualizer（可视化）**
| 功能 | 说明 |
|------|------|
| OpenGL 渲染 | 高性能3D点云显示 |
| 轨迹跟踪 | 实时显示运动轨迹 |
| 交互控制 | 鼠标/键盘操作 |

---

### ❌ 缺失的功能

#### 中优先级缺失：

**动态物体检测/移除**
   - 当前系统未区分静态和动态物体

**多会话SLAM**
   - 不支持地图拼接和长期定位

---

### ⚡ 已完成的优化

#### 1. **回环检测参数优化** ✅
- Scan Context 参数可通过 YAML 配置（[slam_params.yaml](file:///home/ydkj/lidarslam_ws/src/lidar_slam2/config/slam_params.yaml#L73-L88)）
- 可配置参数：环数、扇区数、最大距离、空间搜索半径、相似度阈值等
- 关键帧参数可配置：平移/旋转/时间阈值、最大关键帧数量

#### 2. **MappingNode 异步处理** ✅
- 支持通过 `num_worker_threads` 配置异步工作线程
- `num_worker_threads: 0` → 同步模式（默认，实时发布地图）
- `num_worker_threads: N (N>0)` → 异步模式，提高处理吞吐量

#### 3. **关键帧管理优化** ✅
- 使用 `std::deque` 替代 `std::vector`，支持高效的两端操作
- 添加空间哈希索引加速邻近关键帧搜索
- 支持最大关键帧数量限制（`max_keyframes`），自动清理旧关键帧

#### 4. **组件节点名称修复** ✅
- 修复 launch 文件中节点名覆盖问题
- 各组件使用独立节点名：`odometry_node`、`mapping_node`、`loop_closure_node`
- 服务注册路径正确：`/loop_closure_node/relocalize`

#### 5. **地图保存/加载功能** ✅
- 新增 ROS2 服务接口（定义在 `lidar_slam2_msgs` 包）：
  - `/mapping_node/save_map` - 保存地图为 PCD 文件
  - `/mapping_node/load_map` - 从 PCD 文件加载地图

---

### 📊 系统架构总结

```
┌─────────────────────────────────────────────────────────┐
│                    LiDAR SLAM2 系统                      │
├─────────────────────────────────────────────────────────┤
│  输入: /rs_150 (PointCloud2)                            │
│    │                                                     │
│    ▼                                                     │
│  ┌──────────────────┐                                   │
│  │  OdometryNode    │ ◄── 已实现: KISS-ICP             │
│  │  (里程计)         │ ◄── 已实现: 跟踪监控              │
│  │  (odometry_node) │ ◄── 已实现: 自动重定位触发        │
│  └────────┬─────────┘                                   │
│           │ /odom                                        │
│           ▼                                              │
│  ┌──────────────────┐      ┌──────────────────┐         │
│  │  MappingNode     │      │  LoopClosureNode  │         │
│  │  (建图)          │      │  (回环检测)        │         │
│  │  (mapping_node)  │      │  (loop_closure)   │         │
│  │  ✓ 同步/异步     │      │  ✓ YAML 配置      │         │
│  └────────┬─────────┘      │  ✓ 高效关键帧管理 │         │
│           │ /map            └────────┬─────────┘         │
│           ▼                          │                    │
│  ┌──────────────────┐                │                    │
│  │  Visualizer      │◄───────────────┘                    │
│  │  (可视化)         │                                   │
│  │  ✓ 已实现        │                                   │
│  └──────────────────┘                                   │
├─────────────────────────────────────────────────────────┤
│  ⚠ 未实现: 地图保存/加载    │
└─────────────────────────────────────────────────────────┘
```

---

### ⚙️ YAML 配置参数说明

#### 建图参数 (MappingNode)
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_voxel_size` | 0.3 | 体素大小 (米) |
| `max_map_size` | 200.0 | 最大地图范围 (米) |
| `use_voxel_filter` | true | 是否使用体素滤波 |
| `map_update_interval` | 1 | 地图发布间隔 (帧) |
| `num_worker_threads` | 0 | 异步工作线程数，0为同步模式 |

#### Scan Context 参数 (LoopClosureNode)
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `scan_context.num_rings` | 20 | 环的数量 |
| `scan_context.num_sectors` | 60 | 扇区的数量 |
| `scan_context.max_range` | 80.0 | 最大距离 (米) |
| `scan_context.spatial_search_radius` | 8.0 | 空间搜索半径 (米) |
| `scan_context.similarity_threshold` | 0.8 | 相似度阈值 |
| `scan_context.translation_threshold` | 1.0 | 一致性检查平移阈值 (米) |
| `scan_context.rotation_threshold` | 10.0 | 一致性检查旋转阈值 (度) |

#### 关键帧参数 (KeyFrameManager)
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `keyframe.translation_threshold` | 0.8 | 关键帧平移阈值 (米) |
| `keyframe.rotation_threshold` | 8.0 | 关键帧旋转阈值 (度) |
| `keyframe.time_threshold` | 1.5 | 关键帧时间阈值 (秒) |
| `keyframe.max_keyframes` | 1000 | 最大关键帧数量，0表示不限制 |

---

### 🔧 ROS2 服务接口

#### MappingNode 服务
| 服务名称 | 类型 | 说明 |
|----------|------|------|
| `/mapping_node/save_map` | `lidar_slam2_msgs/srv/SaveMap` | 保存当前地图为 PCD 文件 |
| `/mapping_node/load_map` | `lidar_slam2_msgs/srv/LoadMap` | 从 PCD 文件加载地图 |

#### 使用示例

**保存地图（指定路径）：**
```bash
ros2 service call /mapping_node/save_map lidar_slam2_msgs/srv/SaveMap "{file_path: '/home/user/map.pcd'}"

# 例如
ros2 service call /mapping_node/save_map lidar_slam2_msgs/srv/SaveMap "{file_path: '/home/ydkj/lidarslam_ws/map.pcd'}"
```

**保存地图（自动生成文件名）：**
```bash
ros2 service call /mapping_node/save_map lidar_slam2_msgs/srv/SaveMap "{file_path: ''}"
```

**加载地图（替换当前地图）：**
```bash
ros2 service call /mapping_node/load_map lidar_slam2_msgs/srv/LoadMap "{file_path: '/home/user/map.pcd', load_as_current: true}"
```

**加载地图（仅查询点数，不替换）：**
```bash
ros2 service call /mapping_node/load_map lidar_slam2_msgs/srv/LoadMap "{file_path: '/home/user/map.pcd', load_as_current: false}"
```
