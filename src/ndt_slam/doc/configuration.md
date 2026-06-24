# 配置文件说明

## 主要配置文件

| 文件 | 用途 |
|------|------|
| `slam_params.yaml` | 主 SLAM 配置（NDT 参数、话题、坐标系） |
| `live_longterm_mapping.yaml` | 长期建图配置（MotionGate、tile、内存保护） |
| `merger_params.yaml` | 双雷达合并配置 |
| `engineering_mapping.yaml` | 工程化建图配置 |
| `map_postprocess.yaml` | 地图后处理配置 |

## slam_params.yaml 主要参数

### NDT 配准
```yaml
ndt_omp:
  resolution: 1.0        # NDT 分辨率（米）
  step_size: 0.1         # 步长
  max_iterations: 50     # 最大迭代次数
```

### 地图参数
```yaml
map_voxel_size: 0.30     # 配准地图体素
display_voxel_size: 0.10 # 显示地图体素
ground_voxel_size: 0.15  # 地面地图体素
objects_voxel_size: 0.06 # 物体地图体素
```

### 话题配置
```yaml
pointcloud_topic: "/merged_points"
odom_topic: "/odom"
map_topic: "/map"
```

## live_longterm_mapping.yaml 主要参数

### MotionGate
```yaml
motion_gate:
  enabled: true
  min_translation_m: 0.30      # 最小位移（米）
  min_rotation_deg: 3.0        # 最小旋转（度）
  min_time_between_keyframes_sec: 2.0
```

### 内存保护
```yaml
memory_guard:
  enabled: true
  soft_threshold_mb: 6000      # 6GB: 释放缓存
  hard_threshold_mb: 7000      # 7GB: 暂停 commit
  emergency_threshold_mb: 8000 # 8GB: 降采样
```

### 磁盘 tile
```yaml
persistent_map:
  enabled: true
  tile_size_m: 20.0
  flush_interval_sec: 60
  tile_voxel_registration: 0.30
  tile_voxel_display: 0.10
  tile_voxel_ground: 0.15
  tile_voxel_objects: 0.08
```
