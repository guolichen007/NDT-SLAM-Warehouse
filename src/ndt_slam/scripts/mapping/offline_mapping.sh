#!/bin/bash

# 离线建图工具脚本
# 用法: ./offline_mapping.sh <bag_file> <session_dir> [mode]

set -e

BAG_FILE=$1
SESSION_DIR=$2
MODE=${3:-"full"}  # coarse, refine, rebuild, export, full

if [ -z "$BAG_FILE" ] || [ -z "$SESSION_DIR" ]; then
    echo "用法: $0 <bag_file> <session_dir> [mode]"
    echo "  mode: coarse|refine|rebuild|export|full (默认: full)"
    exit 1
fi

# 检查 bag 文件是否存在
if [ ! -f "$BAG_FILE" ]; then
    echo "错误: bag 文件不存在: $BAG_FILE"
    exit 1
fi

# 创建会话目录
mkdir -p "$SESSION_DIR"

# 设置 ROS 环境
source /opt/ros/noetic/setup.bash
source /home/ydkj/NDT-slam-ws/devel/setup.bash

echo "=========================================="
echo "离线建图工具"
echo "=========================================="
echo "Bag 文件: $BAG_FILE"
echo "会话目录: $SESSION_DIR"
echo "运行模式: $MODE"
echo "=========================================="

# 第一轮：粗建图
run_coarse_mapping() {
    echo ""
    echo "========== 第一轮：粗建图 =========="
    echo "目标：得到基本正确的轨迹和初始地图"
    echo ""

    # 启动 SLAM 节点
    roslaunch ndt_slam offline_mapping.launch \
        bag_file:="$BAG_FILE" \
        session_dir:="$SESSION_DIR" \
        mode:=coarse &
    SLAM_PID=$!

    # 等待节点启动
    sleep 3

    # 开始播放 bag（按空格键开始）
    echo "按空格键开始播放 bag..."
    read -n 1 -s

    # 等待 bag 播放完成
    wait $SLAM_PID

    # 保存关键帧数据库
    rosservice call /save_map "file_path: '$SESSION_DIR/map_coarse.pcd'"

    echo "第一轮完成"
}

# 第二轮：精配准
run_refine_poses() {
    echo ""
    echo "========== 第二轮：精配准 =========="
    echo "目标：用上一轮地图作为先验，重新优化关键帧位姿"
    echo ""

    # 加载上一轮的关键帧数据库和定位地图
    rosservice call /load_map "file_path: '$SESSION_DIR/session_*/map_registration.pcd'"

    # 执行离线精配准
    # 这个需要通过服务或参数触发
    echo "精配准功能需要通过服务调用触发"
    echo "请使用 rosservice call /refine_poses 触发"
}

# 第三轮：重建地图
run_rebuild_maps() {
    echo ""
    echo "========== 第三轮：重建地图 =========="
    echo "目标：从关键帧重新生成多层地图"
    echo ""

    # 从关键帧重建地图
    rosservice call /rebuild_map "session_dir: '$SESSION_DIR'"

    echo "地图重建完成"
}

# 导出导航地图
run_export_navigation() {
    echo ""
    echo "========== 导出导航地图 =========="
    echo ""

    # 导出导航地图
    rosservice call /export_navigation "session_dir: '$SESSION_DIR', resolution: 0.1"

    echo "导航地图导出完成"
}

# 根据模式执行
case $MODE in
    coarse)
        run_coarse_mapping
        ;;
    refine)
        run_refine_poses
        ;;
    rebuild)
        run_rebuild_maps
        ;;
    export)
        run_export_navigation
        ;;
    full)
        run_coarse_mapping
        run_refine_poses
        run_rebuild_maps
        run_export_navigation
        ;;
    *)
        echo "未知模式: $MODE"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "建图完成！"
echo "=========================================="
echo "输出目录: $SESSION_DIR"
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
