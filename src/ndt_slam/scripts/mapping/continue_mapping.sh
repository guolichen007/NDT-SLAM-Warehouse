#!/bin/bash

# 继续建图脚本
# 加载已有地图 -> 播放bag -> 保存关键帧 -> 重建 -> 自动升级版本

set -e

BAG_FILE=$1
MAPS_DIR="/home/ydkj/NDT-slam-ws/maps"
OUTPUT_DIR="/home/ydkj/NDT-slam-ws/output"
NDT_WS="/home/ydkj/NDT-slam-ws"

if [ -z "$BAG_FILE" ]; then
    echo "用法: $0 <bag_file>"
    echo "示例: $0 /home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag"
    exit 1
fi

if [ ! -f "$BAG_FILE" ]; then
    echo "错误: bag文件不存在: $BAG_FILE"
    exit 1
fi

source /opt/ros/noetic/setup.bash
source $NDT_WS/devel/setup.bash

# 检查当前地图
if [ ! -L "$MAPS_DIR/current" ]; then
    echo "错误: 没有当前地图 ($MAPS_DIR/current)"
    echo "请先运行: python3 $NDT_WS/src/ndt_slam/scripts/postprocess/build_map_pipeline.py --bag <bag>"
    exit 1
fi

CURRENT_MAP=$(readlink -f "$MAPS_DIR/current")
CURRENT_VERSION=$(basename "$CURRENT_MAP" | grep -o '[0-9]*' | head -1)
NEW_VERSION=$((CURRENT_VERSION + 1))

echo "=========================================="
echo "继续建图 - 版本升级"
echo "=========================================="
echo "Bag: $BAG_FILE"
echo "当前地图: $CURRENT_MAP (v$CURRENT_VERSION)"
echo "目标版本: warehouse_v$(printf '%03d' $NEW_VERSION)"
echo "=========================================="

# 清理旧数据
rm -rf $OUTPUT_DIR/session_* $OUTPUT_DIR/rebuild_* $OUTPUT_DIR/mapping_* $OUTPUT_DIR/continue_* 2>/dev/null

# 步骤1: 启动SLAM
echo ""
echo "[1/6] 启动SLAM（加载已有地图）..."
cd $NDT_WS
roslaunch ndt_slam dual_lidar_slam.launch &
SLAM_PID=$!
sleep 8

# 加载已有地图
echo "[2/6] 加载先验地图..."
rosservice call /load_map "file_path: '$CURRENT_MAP/registration_map.pcd'"
sleep 2

# 步骤3: 播放bag
echo "[3/6] 播放bag..."
rosbag play --clock "$BAG_FILE" &
BAG_PID=$!

# 等待bag播放完成
wait $BAG_PID 2>/dev/null || true
sleep 5

# 步骤4: 保存关键帧
echo "[4/6] 保存关键帧..."
rosservice call /save_map "file_path: '$OUTPUT_DIR/continue_mapping.pcd'"
sleep 3

LATEST_SESSION=$(ls -td $OUTPUT_DIR/session_* 2>/dev/null | head -1)
echo "  Session: $LATEST_SESSION"

# 步骤5: 重建地图
echo "[5/6] 重建地图..."
rosservice call /rebuild_map
sleep 15

LATEST_REBUILD=$(ls -td $OUTPUT_DIR/rebuild_* 2>/dev/null | head -1)
echo "  Rebuild: $LATEST_REBUILD"

# 步骤6: 停止SLAM
echo "[6/6] 停止SLAM..."
kill $SLAM_PID 2>/dev/null
sleep 2

# 创建新版本
echo ""
echo "创建新版本: warehouse_v$(printf '%03d' $NEW_VERSION)..."
cd $NDT_WS
python3 src/ndt_slam/scripts/postprocess/map_version_manager.py create \
    --version $NEW_VERSION \
    --source "$LATEST_REBUILD" \
    --session "$LATEST_SESSION"

# 更新current软链接
python3 src/ndt_slam/scripts/postprocess/map_version_manager.py promote $NEW_VERSION

# 删除旧版本备份（只保留最新）
OLD_BACKUP="$MAPS_DIR/warehouse_v$(printf '%03d' $CURRENT_VERSION).bak"
if [ -d "$OLD_BACKUP" ]; then
    rm -rf "$OLD_BACKUP"
fi

# 备份旧版本（不删除，只改名）
OLD_MAP_DIR="$MAPS_DIR/warehouse_v$(printf '%03d' $CURRENT_VERSION)"
if [ -d "$OLD_MAP_DIR" ]; then
    mv "$OLD_MAP_DIR" "$OLD_BACKUP"
    echo "旧版本已备份: $OLD_BACKUP"
fi

# 清理临时文件
rm -rf $OUTPUT_DIR/session_* $OUTPUT_DIR/rebuild_* $OUTPUT_DIR/mapping_* $OUTPUT_DIR/continue_* 2>/dev/null

echo ""
echo "=========================================="
echo "建图更新完成！"
echo "=========================================="
echo "新版本: warehouse_v$(printf '%03d' $NEW_VERSION)"
echo "路径: $MAPS_DIR/warehouse_v$(printf '%03d' $NEW_VERSION)"
echo "当前地图: maps/current -> warehouse_v$(printf '%03d' $NEW_VERSION)"
echo ""
echo "地图统计:"
for f in $MAPS_DIR/warehouse_v$(printf '%03d' $NEW_VERSION)/*.pcd; do
    if [ -f "$f" ]; then
        POINTS=$(head -10 "$f" | grep "POINTS" | awk '{print $2}')
        SIZE=$(du -h "$f" | cut -f1)
        echo "  $(basename $f): $POINTS 点 ($SIZE)"
    fi
done
echo ""
echo "下一步:"
echo "  继续优化: $0 <bag_file>"
echo "  运行系统: roslaunch ndt_slam warehouse_runtime.launch"
echo "=========================================="
