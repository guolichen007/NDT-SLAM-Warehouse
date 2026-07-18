# 地图后处理

## 后处理流程

```bash
bash src/ndt_slam/scripts/postprocess/run_map_postprocess.sh <输入目录> <输出目录>
```

流程：
1. 分析原始地图质量
2. 生成 ground_map_clean
3. 生成 objects_clean
4. 生成 registration_map_fixed
5. 生成 navigation_grid
6. 生成地图清单

## 输出结构

```
release/
├── registration_map_fixed.pcd
├── objects_clean.pcd
├── ground_map_clean.pcd
├── navigation_grid_0.05m.pgm
├── navigation_grid_0.05m.yaml
├── map_manifest.yaml
└── quality_after.json

debug_layers/
├── display_map.pcd
├── objects_raw.pcd
└── ...
```
