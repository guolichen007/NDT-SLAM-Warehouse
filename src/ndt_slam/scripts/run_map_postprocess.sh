#!/bin/bash
# 一键地图后处理脚本
# 用法: bash run_map_postprocess.sh [输入目录] [输出目录]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/../config/map_postprocess.yaml"

INPUT_DIR="${1:-maps/current}"
OUTPUT_DIR="${2:-maps/warehouse_v003}"

echo "============================================================"
echo "  地图后处理"
echo "============================================================"
echo "  输入目录: ${INPUT_DIR}"
echo "  输出目录: ${OUTPUT_DIR}"
echo "  配置文件: ${CONFIG_FILE}"
echo "============================================================"

# 创建输出目录
mkdir -p "${OUTPUT_DIR}"

# 第一步：分析原始地图质量
echo ""
echo ">>> 第一步：分析原始地图质量"
python3 "${SCRIPT_DIR}/analyze_map_quality.py" \
    --map_dir "${INPUT_DIR}" \
    --output "${OUTPUT_DIR}/quality_before.json"

# 第二步：生成 ground_map_clean
echo ""
echo ">>> 第二步：生成 ground_map_clean"
python3 "${SCRIPT_DIR}/build_ground_clean.py" \
    --config "${CONFIG_FILE}" \
    --input "${INPUT_DIR}/display_map.pcd" \
    --output "${OUTPUT_DIR}"

# 第三步：生成 objects_clean
echo ""
echo ">>> 第三步：生成 objects_clean"
python3 "${SCRIPT_DIR}/build_objects_clean.py" \
    --config "${CONFIG_FILE}" \
    --input "${INPUT_DIR}/display_map.pcd" \
    --output "${OUTPUT_DIR}"

# 第四步：生成 localization_map_fine
echo ""
echo ">>> 第四步：生成 localization_map_fine"
python3 "${SCRIPT_DIR}/build_localization_map.py" \
    --config "${CONFIG_FILE}" \
    --input "${INPUT_DIR}/display_map.pcd" \
    --output "${OUTPUT_DIR}"

# 第五步：生成 navigation_grid
echo ""
echo ">>> 第五步：生成 navigation_grid"
python3 "${SCRIPT_DIR}/build_navigation_grid.py" \
    --config "${CONFIG_FILE}" \
    --output "${OUTPUT_DIR}"

# 第六步：复制原始地图
echo ""
echo ">>> 第六步：复制原始地图"
cp "${INPUT_DIR}/display_map.pcd" "${OUTPUT_DIR}/"
cp "${INPUT_DIR}/objects_raw.pcd" "${OUTPUT_DIR}/"
cp "${INPUT_DIR}/registration_map.pcd" "${OUTPUT_DIR}/registration_map_coarse.pcd"

# 第七步：分析新地图质量
echo ""
echo ">>> 第七步：分析新地图质量"
python3 "${SCRIPT_DIR}/analyze_map_quality.py" \
    --map_dir "${OUTPUT_DIR}" \
    --output "${OUTPUT_DIR}/quality_after.json"

echo ""
echo "============================================================"
echo "  地图后处理完成!"
echo "============================================================"
echo "  输出目录: ${OUTPUT_DIR}"
echo ""
echo "  生成的文件:"
ls -la "${OUTPUT_DIR}"
echo ""
echo "  请查看质量报告:"
echo "    quality_before.json  (原始地图质量)"
echo "    quality_after.json   (新地图质量)"
echo "    ground_clean_report.json"
echo "    objects_clean_report.json"
echo "    localization_map_report.json"
echo "    navigation_report.json"
echo "============================================================"
