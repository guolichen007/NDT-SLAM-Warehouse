# 工程化建图指南

## 概述

本系统提供了一套完整的工程化建图流程，支持多次运行 bag 文件进行迭代优化，最终生成高质量的多层地图。

## 核心原则

**不要叠加旧 PCD，而是从关键帧重新生成地图。**

每次运行的流程：
1. 保存关键帧原始点云和位姿
2. 用上一轮结果作为先验，重新优化关键帧位姿
3. 从原始关键帧重新生成地图

## 四张地图

系统生成四张不同用途的地图：

| 地图类型 | 用途 | 体素大小 | 说明 |
|---------|------|---------|------|
| `localization_map` | 重定位和定位匹配 | 0.2-0.4m | 保留稳定结构：墙体、库房边界、固定设备 |
| `detail_map` | 查看高质量点云细节 | 0.05-0.10m | 保留更多货物、设备、地面结构 |
| `objects_clean_map` | 货物结构观察和识别 | 0.06m | 只保留货物、设备、墙体等非地面结构 |
| `navigation_map` | 自动导航 | 0.1m | 2D occupancy grid，用于路径规划 |

## 多次运行 bag 的正确流程

### 第一轮：粗建图

目标：得到一个基本正确的轨迹和初始地图。

```bash
./scripts/offline_mapping.sh /path/to/bag /path/to/session coarse
```

流程：
```
bag 输入
→ 点云预处理
→ NDT_OMP 粗定位
→ 关键帧提取
→ 回环检测
→ g2o 图优化
→ 保存 keyframes + poses + 初始 map
```

输出：
```
session_001/
  keyframes/
    000001.pcd
    000002.pcd
    ...
  poses_raw.txt
  poses_optimized.txt
  map_registration.pcd
  map_display.pcd
  map_objects_raw.pcd
  map_objects_clean.pcd
```

### 第二轮：精配准

目标：用上一轮地图作为先验，重新优化关键帧位姿。

```bash
./scripts/offline_mapping.sh /path/to/bag /path/to/session refine
```

流程：
```
加载上一轮 poses_optimized.txt
加载上一轮 localization_map
每一帧用上一轮位姿作为初值
对关键帧做 scan-to-map 精配准
更新关键帧位姿
重新做图优化
重新生成地图
```

**重要：不要把第二轮点云直接叠加到第一轮 PCD 上。要从 keyframes 原始点云重新生成地图。**

### 第三轮：地图清洁与分层

目标：清理地图，生成多层地图。

```bash
./scripts/offline_mapping.sh /path/to/bag /path/to/session rebuild
```

流程：
```
读取最终 keyframes + optimized poses
→ 重建全量 detail_map
→ 重建 ground_map
→ 重建 objects_raw
→ 重建 objects_candidate
→ 重建 objects_clean
→ 输出 navigation occupancy map
```

### 导出导航地图

```bash
./scripts/offline_mapping.sh /path/to/bag /path/to/session export
```

输出：
```
warehouse_2d_occupancy.yaml
warehouse_2d_occupancy.pgm
```

## 关键帧数据库

每个关键帧保存以下信息：

```json
{
  "id": 1,
  "timestamp": 1234567890.123456789,
  "fitness_score": 0.1234,
  "transformation_probability": 0.9876,
  "inlier_ratio": 0.85,
  "ground_thickness": 0.05,
  "obj_ratio": 0.35,
  "ground_points": 5000,
  "object_points": 2000,
  "registration_time_ms": 15.5,
  "accepted_for_localization": true,
  "accepted_for_detail_map": true,
  "accepted_for_clean_map": true
}
```

## 自适应过滤

系统根据点到传感器的距离动态调整过滤参数：

### SOR 过滤
| 距离 | mean_k | stddev_mul_thresh | 说明 |
|------|--------|-------------------|------|
| <10m | 10 | 2.0 | 近处：严格过滤 |
| 10-20m | 8 | 2.5 | 中距离：适中 |
| >20m | 5 | 3.0 | 远处：宽松过滤 |

### BEV min_height
| 距离 | min_height | 说明 |
|------|------------|------|
| <10m | 0.35m | 近处：保持原值 |
| 10-20m | 0.25m | 中距离：降低 30% |
| >20m | 0.15m | 远处：降低 57% |

### 时间一致性
| 距离 | min_observations | 说明 |
|------|------------------|------|
| <10m | 2次 | 近处：保持原值 |
| 10-20m | 1次 | 中距离：首次即可保留 |
| >20m | 1次 | 远处：首次即可保留 |

## 证据累积

每个 voxel / BEV cell 记录：

```
observed_count      被观测次数
occupied_count      被判定为货物/障碍次数
ground_count        被判定为地面次数
max_height          最大高度
mean_height         平均高度
last_seen_time      最后一次看到
confidence          稳定置信度
```

最终判断：

- **稳定货物**：occupied_count >= 2 或 3，confidence 高，height > 0.45m / 0.55m
- **稳定地面**：ground_count 高，高度方差小
- **可疑点**：只出现 1 次，或者某次出现某次消失，或者高度变化大
- **动态/噪声**：出现次数少，位置不稳定，不进入最终地图

## 重定位

重定位使用 `localization_map`，不要直接用最细的 `objects_clean_map`。

定位流程建议：
```
当前帧点云
→ 近场过滤
→ ground/object 分离
→ 提取稳定结构点
→ Scan Context / FPFH 找初始位置
→ NDT scan-to-map 粗匹配
→ GICP / ICP 精匹配
→ 输出 map->base_link
```

## 自动导航

从最终 detail map / objects map 中生成导航图：

- 地面层 → free space
- objects_clean → obstacle
- 墙体/设备 → obstacle
- 未观测区域 → unknown
- 货物间通道 → free / narrow passage

建议分辨率：0.05m 或 0.10m

## 工程化架构

系统包含 5 个模块：

1. **bag_mapping_node**：离线跑 bag，生成关键帧和初始轨迹
2. **map_refine_node**：读取 keyframes + poses，做多轮精配准和图优化
3. **map_builder_node**：根据最终 keyframe poses 重建多层地图
4. **map_server_node**：加载 localization_map / navigation_map，提供定位使用
5. **localization_node**：在线接收实时点云，在已有地图中重定位

## 目录结构

```
warehouse_map_project/
  bags/
    warehouse_full_01.bag
    warehouse_full_02.bag

  sessions/
    session_001/
      keyframes/
      poses_raw.txt
      poses_optimized.txt
      metrics.json

  maps/
    v001/
      localization_map.pcd
      detail_map.pcd
      ground_map.pcd
      objects_raw.pcd
      objects_candidate.pcd
      objects_clean.pcd
      cargo_bev_edges.pcd
      navigation_map.yaml
      navigation_map.pgm
      metadata.yaml

  configs/
    mapping.yaml
    refine.yaml
    localization.yaml
    navigation.yaml
```

## 使用示例

### 完整流程

```bash
# 第一轮：粗建图
./scripts/offline_mapping.sh /data/warehouse.bag /data/sessions/session_001 coarse

# 第二轮：精配准
./scripts/offline_mapping.sh /data/warehouse.bag /data/sessions/session_001 refine

# 第三轮：重建地图
./scripts/offline_mapping.sh /data/warehouse.bag /data/sessions/session_001 rebuild

# 导出导航地图
./scripts/offline_mapping.sh /data/warehouse.bag /data/sessions/session_001 export
```

### 一键执行

```bash
./scripts/offline_mapping.sh /data/warehouse.bag /data/sessions/session_001 full
```

## 质量报告

系统会自动生成质量报告 `quality_report.txt`：

```
========== Map Quality Report ==========

Keyframes:
  Total: 150
  Accepted: 142
  Rejected: 8

Registration Quality:
  Avg Fitness Score: 0.1234
  Avg Inlier Ratio: 0.8567

Trajectory:
  Length: 245.67 m

Map Points:
  Localization Map: 1234567
  Detail Map: 2345678
  Ground Map: 987654
  Objects Raw: 345678
  Objects Clean: 234567
```

## 注意事项

1. **不要叠加旧 PCD**：每次都要从关键帧重新生成地图
2. **保存原始关键帧**：这是后续优化的基础
3. **多轮优化**：轨迹优化越来越准，关键帧筛选越来越严格
4. **地图分层**：定位、细节、导航地图分开
5. **实时定位只用轻量稳定地图**：不要用最细的地图定位
