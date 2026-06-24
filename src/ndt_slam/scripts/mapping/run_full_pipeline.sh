#!/bin/bash

# 完整工程化建图流程脚本
# 用法: ./run_full_pipeline.sh <bag_file> [session_name]

set -e

BAG_FILE=$1
SESSION_NAME=${2:-"session_$(date +%Y%m%d_%H%M%S)"}

if [ -z "$BAG_FILE" ]; then
    echo "用法: $0 <bag_file> [session_name]"
    exit 1
fi

# 检查 bag 文件
if [ ! -f "$BAG_FILE" ]; then
    echo "错误: bag 文件不存在: $BAG_FILE"
    exit 1
fi

# 设置目录
BASE_DIR="/data/sessions"
SESSION_DIR="$BASE_DIR/$SESSION_NAME"
MAPS_DIR="$BASE_DIR/maps"

# 创建目录
mkdir -p "$SESSION_DIR"
mkdir -p "$MAPS_DIR"

# 设置环境
source /opt/ros/noetic/setup.bash
source /home/ydkj/NDT-slam-ws/devel/setup.bash

echo "=========================================="
echo "工程化建图流程"
echo "=========================================="
echo "Bag 文件: $BAG_FILE"
echo "会话目录: $SESSION_DIR"
echo "=========================================="

# 第一轮：粗建图
echo ""
echo "========== 第一轮：粗建图 =========="
echo "目标：得到基本正确的轨迹和初始地图"
echo ""

# 启动 SLAM 节点
roslaunch ndt_slam dual_lidar_slam.launch &
SLAM_PID=$!

# 等待节点启动
sleep 5

# 播放 bag
rosbag play --clock "$BAG_FILE" &
BAG_PID=$!

# 等待 bag 播放完成
wait $BAG_PID

# 保存关键帧数据库
rosservice call /save_map "file_path: '$SESSION_DIR/map_coarse.pcd'"

# 停止 SLAM 节点
kill $SLAM_PID 2>/dev/null || true
sleep 2

echo "第一轮完成"

# 第二轮：精配准
echo ""
echo "========== 第二轮：精配准 =========="
echo "目标：用上一轮地图作为先验，重新优化关键帧位姿"
echo ""

# 这里需要实现精配准逻辑
# 目前需要手动触发
echo "精配准功能需要通过服务调用触发"
echo "请使用以下命令："
echo "rosservice call /refine_poses \"session_dir: '$SESSION_DIR'\""

# 第三轮：重建地图
echo ""
echo "========== 第三轮：重建地图 =========="
echo "目标：从关键帧重新生成多层地图"
echo ""

# 从关键帧重建地图
rosservice call /rebuild_map "session_dir: '$SESSION_DIR'"

echo "地图重建完成"

# 导出导航地图
echo ""
echo "========== 导出导航地图 =========="
echo ""

# 导出导航地图
rosservice call /export_navigation "session_dir: '$SESSION_DIR', resolution: 0.1"

echo "导航地图导出完成"

# 生成质量报告
echo ""
echo "========== 生成质量报告 =========="
echo ""

rosservice call /generate_report "session_dir: '$SESSION_DIR'"

echo "质量报告生成完成"

# 复制最终地图到 maps 目录
echo ""
echo "========== 复制最终地图 =========="
echo ""

FINAL_MAPS_DIR="$MAPS_DIR/$SESSION_NAME"
mkdir -p "$FINAL_MAPS_DIR"

# 复制地图文件
cp "$SESSION_DIR"/map_*.pcd "$FINAL_MAPS_DIR/" 2>/dev/null || true
cp "$SESSION_DIR"/navigation_map.* "$FINAL_MAPS_DIR/" 2>/dev/null || true
cp "$SESSION_DIR"/quality_report.txt "$FINAL_MAPS_DIR/" 2>/dev/null || true

echo "最终地图已复制到: $FINAL_MAPS_DIR"

echo ""
echo "=========================================="
echo "建图完成！"
echo "=========================================="
echo "会话目录: $SESSION_DIR"
echo "最终地图: $FINAL_MAPS_DIR"
echo ""
echo "生成的文件："
echo "  - keyframes/          : 关键帧点云"
echo "  - poses_raw.txt       : 原始位姿"
echo "  - poses_optimized.txt : 优化位姿"
echo "  - metrics.json        : 质量指标"
echo "  - map_registration.pcd: 配准地图"
echo "  - map_display.pcd     : 显示地图"
echo "  - map_ground.pcd      : 地面地图"
echo "  - map_objects_raw.pcd : 物体地图（原始）"
echo "  - map_objects_clean.pcd: 物体地图（干净）"
echo "  - navigation_map.pgm  : 导航地图"
echo "  - navigation_map.yaml : 导航地图配置"
echo "  - quality_report.txt  : 质量报告"
echo "=========================================="
