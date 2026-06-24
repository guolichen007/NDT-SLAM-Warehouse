# 部署指南

## systemd 服务

```bash
# 安装服务
sudo cp src/ndt_slam/scripts/deploy/ndt-slam.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable ndt-slam

# 启动/停止
sudo systemctl start ndt-slam
sudo systemctl stop ndt-slam

# 查看日志
journalctl -u ndt-slam -f
```

## 监控脚本

```bash
# 启动监控（每 60 秒记录一次）
bash src/ndt_slam/scripts/deploy/slam_monitor.sh 60

# 查看运行状态
cat /home/ydkj/slam_data/maps/live/current/runtime_status.json | python3 -m json.tool

# 查看趋势
tail -f /home/ydkj/slam_data/maps/live/current/memory_trend.csv

# 查看告警
tail -f /home/ydkj/slam_data/maps/live/current/alerts.log
```

## 一键部署

```bash
bash src/ndt_slam/scripts/deploy/deploy_slam.sh
```

## 部署前检查清单

```bash
# 1. 雷达 topic
rostopic hz /rs_201
rostopic hz /rs_203

# 2. 磁盘空间
df -h /home/ydkj/

# 3. 内存
free -m

# 4. 时钟同步
timedatectl status | grep NTP
```
