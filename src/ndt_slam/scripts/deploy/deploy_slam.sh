#!/bin/bash
set -e

echo "========== NDT-SLAM 长期建图部署 =========="

# 1. 创建数据目录
echo "[1/7] 创建数据目录..."
mkdir -p /home/ydkj/NDT-slam-ws/maps/live/current
mkdir -p /home/ydkj/NDT-slam-ws/maps/live/archive
mkdir -p /home/ydkj/NDT-slam-ws/logs

# 2. 检查磁盘空间
echo "[2/7] 检查磁盘空间..."
FREE_GB=$(df --output=avail /home/ydkj | tail -1 | awk '{printf "%.0f", $1/1024/1024}')
if [ "$FREE_GB" -lt 50 ]; then
    echo "ERROR: 只有 ${FREE_GB}GB 可用，需要至少 50GB"
    exit 1
fi
echo "  磁盘可用: ${FREE_GB}GB ✓"

# 3. 检查内存
echo "[3/7] 检查内存..."
FREE_MEM=$(free -m | awk '/Mem:/{print $7}')
echo "  内存可用: ${FREE_MEM}MB ✓"

# 4. 检查时钟同步
echo "[4/7] 检查时钟同步..."
if timedatectl status | grep -q "NTP synchronized: yes"; then
    echo "  NTP 同步: ✓"
else
    echo "  WARNING: NTP 未同步，建议执行 sudo timedatectl set-ntp true"
fi

# 5. 编译
echo "[5/7] 编译..."
cd /home/ydkj/NDT-slam-ws
catkin_make --pkg ndt_slam 2>&1 | tail -5
echo "  编译完成 ✓"

# 6. 安装 systemd 服务
echo "[6/7] 安装 systemd 服务..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -f /etc/systemd/system/ndt-slam.service ]; then
    sudo cp "$SCRIPT_DIR/ndt-slam.service" /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable ndt-slam
    echo "  systemd 服务已安装 ✓"
else
    sudo cp "$SCRIPT_DIR/ndt-slam.service" /etc/systemd/system/
    sudo systemctl daemon-reload
    echo "  systemd 服务已更新 ✓"
fi

# 7. 启动监控
echo "[7/7] 启动监控..."
if ! pgrep -f slam_monitor.sh > /dev/null; then
    nohup "$SCRIPT_DIR/slam_monitor.sh" > /dev/null 2>&1 &
    echo "  监控已启动 (PID: $!) ✓"
else
    echo "  监控已在运行 ✓"
fi

echo ""
echo "========== 部署完成 =========="
echo "启动 SLAM:  sudo systemctl start ndt-slam"
echo "查看状态:    sudo systemctl status ndt-slam"
echo "查看日志:    journalctl -u ndt-slam -f"
echo "查看运行:    cat /home/ydkj/NDT-slam-ws/maps/live/current/runtime_status.json"
echo "查看趋势:    tail -f /home/ydkj/NDT-slam-ws/maps/live/current/memory_trend.csv"
echo "查看告警:    tail -f /home/ydkj/NDT-slam-ws/maps/live/current/alerts.log"
