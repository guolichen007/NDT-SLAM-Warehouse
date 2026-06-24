#!/bin/bash
# 长期建图监控脚本
# 用法: bash monitor_longterm.sh [监控间隔秒数]

INTERVAL=${1:-60}
LOG_FILE="/home/ydkj/NDT-slam-ws/maps/live/current/monitor_log.csv"
STATUS_FILE="/home/ydkj/NDT-slam-ws/maps/live/current/runtime_status.json"

# 创建日志文件
echo "timestamp,total_frames,keyframes,active_kf,map_points,dirty_tiles,mem_mb,disk_free_gb,is_stationary,process_ms" > $LOG_FILE

echo "=========================================="
echo "  长期建图监控"
echo "=========================================="
echo "  监控间隔: ${INTERVAL}秒"
echo "  日志文件: $LOG_FILE"
echo "  状态文件: $STATUS_FILE"
echo "=========================================="
echo ""

while true; do
    TS=$(date +%Y-%m-%d_%H:%M:%S)

    if [ -f "$STATUS_FILE" ]; then
        # 读取 runtime_status.json
        TF=$(python3 -c "import json; d=json.load(open('$STATUS_FILE')); print(d.get('total_frames',0))" 2>/dev/null || echo "0")
        KF=$(python3 -c "import json; d=json.load(open('$STATUS_FILE')); print(d.get('total_keyframes',0))" 2>/dev/null || echo "0")
        AK=$(python3 -c "import json; d=json.load(open('$STATUS_FILE')); print(d.get('active_keyframes',0))" 2>/dev/null || echo "0")
        MP=$(python3 -c "import json; d=json.load(open('$STATUS_FILE')); print(d.get('active_map_points',0))" 2>/dev/null || echo "0")
        DT=$(python3 -c "import json; d=json.load(open('$STATUS_FILE')); print(d.get('dirty_tile_count',0))" 2>/dev/null || echo "0")
        DF=$(python3 -c "import json; d=json.load(open('$STATUS_FILE')); print(d.get('disk_free_gb',0))" 2>/dev/null || echo "0")
        IS=$(python3 -c "import json; d=json.load(open('$STATUS_FILE')); print(d.get('is_stationary',False))" 2>/dev/null || echo "False")
        PT=$(python3 -c "import json; d=json.load(open('$STATUS_FILE')); print(d.get('average_process_time_ms',0))" 2>/dev/null || echo "0")
    else
        TF=0; KF=0; AK=0; MP=0; DT=0; DF=0; IS="N/A"; PT=0
    fi

    # 获取内存使用
    PID=$(pgrep ndt_slam_node 2>/dev/null)
    if [ -n "$PID" ]; then
        MEM=$(ps -o rss= -p $PID 2>/dev/null | awk '{printf "%.0f", $1/1024}')
    else
        MEM=0
    fi

    # 写入日志
    echo "$TS,$TF,$KF,$AK,$MP,$DT,$MEM,$DF,$IS,$PT" >> $LOG_FILE

    # 输出到终端
    echo "[$TS]"
    echo "  帧数: $TF | 关键帧: $KF | 活跃: $AK"
    echo "  地图点数: $MP | 脏块: $DT"
    echo "  内存: ${MEM}MB | 磁盘: ${DF}GB"
    echo "  静止: $IS | 处理时间: ${PT}ms"
    echo ""

    sleep $INTERVAL
done
