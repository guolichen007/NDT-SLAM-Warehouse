# 定位运行模式

## 启动

```bash
roslaunch ndt_slam warehouse_runtime.launch
```

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| map_dir | maps/current | 地图目录 |
| localization_only | true | 纯定位模式 |
| use_sim_time | false | 使用仿真时间 |

## 定位流程

1. 加载 registration_map_fixed.pcd
2. 等待 /merged_points
3. NDT scan-to-map 配准
4. 发布 odom/TF/current_pose

## 注意事项

- localization_only=true 时不更新地图
- 不保存新 keyframe
- 不触发 rebuild_map
