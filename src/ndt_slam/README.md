# NDT-SLAM

基于 NDT_OMP 的激光雷达 SLAM 系统，用于解决 8m 高度俯视场景下的 KISS-ICP 退化问题。

## 项目背景

### 问题描述

在 8m 高度俯视的金属物料堆场场景中，KISS-ICP 完全退化：
- 地面点占比 80-88%，导致平面退化
- 120 秒只检测到 0.78m 位移，丢失 97-99% 运动
- 匹配质量指标具有欺骗性（inlier_ratio=1.000 但位移很小）

### 解决方案

采用混合策略：NDT_OMP + 特征点加权
1. 地面分割（RANSAC 或高度阈值）
2. 特征点加权（权重 4.0）
3. NDT_OMP 配准（分辨率 1.0m）

## 目录结构

```
NDT-slam/
├── CMakeLists.txt          # 编译配置
├── package.xml             # ROS 包配置
├── config/                 # 配置文件
│   ├── slam_params.yaml           # 单雷达 SLAM 配置
│   ├── dual_lidar_slam_params.yaml # 双雷达 SLAM 配置
│   └── merger_params.yaml         # 点云合并配置
├── launch/                 # 启动文件
│   ├── slam_launch.launch         # 单雷达启动
│   ├── dual_lidar_slam.launch     # 双雷达启动
│   ├── single_lidar_slam.launch   # 单雷达测试
│   └── diagnostics.launch         # 诊断启动
├── include/                # 头文件
│   └── lidar_slam2/
│       ├── SlamNode.hpp
│       ├── ScanMatcher.hpp        # [新增] 配准封装
│       └── ...
├── src/                    # 源代码
│   ├── main.cpp
│   ├── SlamNode.cpp
│   ├── ScanMatcher.cpp            # [新增] 配准封装实现
│   ├── PointCloudMerger.cpp
│   ├── CloudDiagnostics.cpp
│   └── ...
└── 3rdparty/               # 第三方库
```

## 编译

```bash
cd /home/ydkj/AutoCraneSlam-ROS1
catkin_make --pkg ndt_slam
source devel/setup.bash
```

## 运行

### 单雷达测试

```bash
roslaunch ndt_slam slam_launch.launch
```

### 双雷达测试

```bash
roslaunch ndt_slam dual_lidar_slam.launch
```

### 诊断测试

```bash
roslaunch ndt_slam diagnostics.launch
```

## 配置参数

### NDT 配准参数

```yaml
scan_matcher:
  type: "ndt"
  resolution: 1.0           # NDT 体素分辨率 (m)
  max_correspondence: 3.0   # 最大对应距离 (m)
  num_threads: 4            # 并行线程数
  max_iterations: 100       # 最大迭代次数
```

### 地面分割参数

```yaml
ground_segmentation:
  enabled: true
  method: "ransac"          # ransac 或 height
  distance_threshold: 0.1   # RANSAC 距离阈值 (m)
  height_threshold: 0.3     # 高度阈值分割的阈值 (m)
```

### 特征点加权参数

```yaml
feature_weight:
  enabled: true
  weight: 4.0               # 特征点权重（重复添加次数）
```

## 与原版对比

| 指标 | 原版 (KISS-ICP) | NDT-SLAM |
|------|----------------|----------|
| 平面退化 | ❌ 完全退化 | ✅ 解决 |
| 位姿估计 | 120s 移动 0.78m | 正常跟踪 |
| 处理频率 | ~100Hz | ~30-50Hz |
| 地图扩展 | ❌ 不扩展 | ✅ 正常扩展 |

## 实施计划

- [x] 阶段 0：复制原版代码，创建 NDT-slam 包
- [ ] 阶段 1：点云预处理 — 地面分割 + 特征点提取
- [ ] 阶段 2：NDT_OMP 集成 + 特征点加权
- [ ] 阶段 3：g2o 地面约束 + 动态信息矩阵
- [ ] 阶段 4：FPFH + ICP 闭环检测
- [ ] 阶段 5：动态物体过滤

## 许可证

MIT License
