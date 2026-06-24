# 系统架构

## 整体数据流

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

## 代码结构

```
src/ndt_slam/
├── src/                           # 源文件
│   ├── main.cpp                   # 主入口
│   ├── ndt_slam.cpp               # 主 SLAM 节点
│   ├── keyframe_manager.cpp       # 关键帧管理
│   ├── loop_closure.cpp           # 闭环检测
│   ├── point_cloud_processing.cpp # 点云处理
│   ├── base_payload_channel_filter.cpp # 吊货通道过滤
│   ├── payload_tracker.cpp        # 吊货跟踪
│   ├── human_object_filter.cpp    # 人体过滤
│   ├── dynamic_event_manager.cpp  # 动态事件管理
│   ├── PointCloudMerger.cpp       # 双雷达合并
│   └── CloudDiagnostics.cpp       # 点云诊断
│
├── include/ndt_slam/              # 头文件
│   ├── ndt_slam.hpp
│   ├── keyframe_manager.hpp
│   ├── loop_closure.hpp
│   ├── point_cloud_processing.hpp
│   ├── base_payload_channel_filter.hpp
│   ├── payload_tracker.hpp
│   ├── human_object_filter.hpp
│   └── dynamic_event_manager.hpp
```

## 配准流程

```
实时定位链路：
  merged_points → 近场过滤 → 预处理 → 地面/物体分割
  → NDT_OMP 配准（1.0m 分辨率）→ 位姿估计 → 发布 odom/TF

后台精配准：
  ICP 精配准（异步）→ 修正关键帧位姿 → 用于地图插入
```

## 地图分层

| 地图类型 | 文件名 | 体素 | 用途 |
|---------|--------|------|------|
| 配准地图 | registration_map.pcd | 0.3m | NDT 配准用粗地图 |
| 显示地图 | display_map.pcd | 0.1m | 全量显示，保留货物轮廓 |
| 地面地图 | ground_map.pcd | 0.15m | 地面分割结果 |
| 物体地图 | objects_raw.pcd | 0.06m | 非地面/货物原始层 |
| 物体干净版 | objects_clean.pcd | 0.06m | 经 BEV+时间一致性过滤 |
| 地面干净版 | ground_map_clean.pcd | 0.08m | 后处理生成的干净地面 |
| 精定位图 | localization_map_fine.pcd | 0.10m | 后处理生成的精定位地图 |
| 导航栅格 | navigation_grid_0.05m.pgm | 0.05m | 2D 导航占用栅格 |
