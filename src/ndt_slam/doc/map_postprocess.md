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
├── localization_map_fine.pcd
├── localization_map_report.json
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

## 定位地图报告合同

`build_localization_map.py` 生成 `localization_map_report.json`，至少包含输入地图、地面模型、
输入点数、输出点数和实际配置。报告在目标目录内写入临时文件，显式使用 UTF-8/LF，
执行文件 flush 和 `fsync`，回读校验 JSON 与必填字段后再用 `os.replace` 原子替换。

序列化、校验或替换前失败时会删除临时文件并保留上一份正式报告。该原子性只覆盖
JSON 报告的可见性与文件数据落盘，不扩大为整个输出目录或 PCD 文件的事务承诺。

对应版本：`f57d68a`。
