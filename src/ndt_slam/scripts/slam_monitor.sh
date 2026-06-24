#!/bin/bash
# SLAM 长期运行监控脚本
# 用法: bash slam_monitor.sh [监控间隔秒数]

INTERVAL=${1:-60}
DATA_DIR="/home/ydkj/NDT-slam-ws/maps/live/current"
STATUS_FILE="$DATA_DIR/runtime_status.json"
TREND_FILE="$DATA_DIR/memory_trend.csv"
ALERTS_FILE="$DATA_DIR/alerts.log"

# 初始化 CSV 头
echo "timestamp,memory_mb,global_pts,display_pts,obj_pts,ground_pts,active_kf,is_stationary,dirty_tiles,disk_free_gb,ndt_fitness,pc_stale,mem_guard,disk_guard" > $TREND_FILE

echo "$(date): SLAM monitor started (interval=${INTERVAL}s)" >> "$DATA_DIR/monitor.log"

while true; do
    TS=$(date +%Y-%m-%d_%H:%M:%S)

    if [ -f "$STATUS_FILE" ]; then
        DATA=$(python3 -c "
import json
try:
    d = json.load(open('$STATUS_FILE'))
    print(','.join([
        str(d.get('memory_mb', 0)),
        str(d.get('global_map_points', 0)),
        str(d.get('display_map_points', 0)),
        str(d.get('objects_map_points', 0)),
        str(d.get('ground_map_points', 0)),
        str(d.get('active_keyframes', 0)),
        str(d.get('is_stationary', False)),
        str(d.get('dirty_tile_count', 0)),
        str(d.get('disk_free_gb', 0)),
        str(d.get('last_ndt_fitness', 0)),
        str(d.get('pointcloud_stale', False)),
        str(d.get('memory_guard_triggered', False)),
        str(d.get('disk_guard_triggered', False))
    ]))
except:
    '0,0,0,0,0,0,False,0,0,0,False,False,False'
" 2>/dev/null)
        echo "$TS,$DATA" >> $TREND_FILE

        # 检查告警条件
        MEM=$(echo "$DATA" | cut -d',' -f1)
        PC_STALE=$(echo "$DATA" | cut -d',' -f11)
        MEM_GUARD=$(echo "$DATA" | cut -d',' -f12)
        DISK_GUARD=$(echo "$DATA" | cut -d',' -f13)

        if [ "$PC_STALE" = "True" ]; then
            echo "$(date): [ALARM] Point cloud stale!" >> "$ALERTS_FILE"
        fi
        if [ "$MEM_GUARD" = "True" ]; then
            echo "$(date): [ALARM] Memory guard triggered! memory=${MEM}MB" >> "$ALERTS_FILE"
        fi
        if [ "$DISK_GUARD" = "True" ]; then
            echo "$(date): [ALARM] Disk guard triggered!" >> "$ALERTS_FILE"
        fi
    fi

    sleep $INTERVAL
done
