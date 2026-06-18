#!/usr/bin/env python3
"""
一键建图脚本
自动完成：播放bag -> 保存关键帧 -> 重建地图 -> 生成版本
"""

import os
import sys
import time
import subprocess
import signal
import argparse
from pathlib import Path
from datetime import datetime

MAPS_BASE = "/home/ydkj/NDT-slam-ws/maps"
SESSION_BASE = "/home/ydkj/NDT-slam-ws/output"

def run_cmd(cmd, timeout=None):
    """运行命令"""
    print(f"  执行: {cmd}")
    proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        stdout, _ = proc.communicate(timeout=timeout)
        return proc.returncode, stdout.decode()
    except subprocess.TimeoutExpired:
        proc.kill()
        return -1, "timeout"

def wait_for_ros_topic(topic, timeout=30):
    """等待 ROS topic 出现"""
    print(f"  等待 topic: {topic}")
    start = time.time()
    while time.time() - start < timeout:
        ret, out = run_cmd(f"rostopic list 2>/dev/null | grep -q '^{topic}$'")
        if ret == 0:
            return True
        time.sleep(1)
    return False

def get_latest_session():
    """获取最新的 session 目录"""
    sessions = sorted(Path(SESSION_BASE).glob("session_*"), key=lambda p: p.stat().st_mtime)
    return sessions[-1] if sessions else None

def main():
    parser = argparse.ArgumentParser(description="一键建图脚本")
    parser.add_argument("--bag", required=True, help="bag 文件路径")
    parser.add_argument("--map-name", default="warehouse", help="地图名称前缀")
    parser.add_argument("--output", default=MAPS_BASE, help="输出目录")
    parser.add_argument("--skip-play", action="store_true", help="跳过播放bag（手动播放）")

    args = parser.parse_args()

    bag_path = Path(args.bag)
    if not bag_path.exists():
        print(f"错误: bag 文件不存在: {bag_path}")
        return 1

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    print(f"=" * 60)
    print(f"一键建图开始")
    print(f"=" * 60)
    print(f"Bag: {bag_path}")
    print(f"地图名: {args.map_name}")
    print(f"时间: {timestamp}")
    print()

    # 步骤 1: 启动 SLAM
    print("[1/5] 启动 SLAM...")
    slam_proc = subprocess.Popen(
        "source /opt/ros/noetic/setup.bash && "
        "source /home/ydkj/NDT-slam-ws/devel/setup.bash && "
        "roslaunch ndt_slam dual_lidar_slam.launch",
        shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(8)
    print(f"  SLAM PID: {slam_proc.pid}")

    # 步骤 2: 播放 bag
    if not args.skip_play:
        print("[2/5] 播放 bag...")
        bag_proc = subprocess.Popen(
            f"source /opt/ros/noetic/setup.bash && rosbag play --clock '{bag_path}'",
            shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        print(f"  Bag PID: {bag_proc.pid}")

        # 等待 bag 播放完成
        print("  等待 bag 播放完成...")
        bag_proc.wait()
        time.sleep(5)
    else:
        print("[2/5] 跳过播放 bag（请手动播放）")
        input("  播放完成后按回车继续...")

    # 步骤 3: 保存地图
    print("[3/5] 保存地图...")
    save_path = f"{SESSION_BASE}/mapping_{timestamp}.pcd"
    run_cmd(
        f"source /home/ydkj/NDT-slam-ws/devel/setup.bash && "
        f"rosservice call /save_map \"file_path: '{save_path}'\""
    )
    time.sleep(3)

    # 获取最新 session
    session_dir = get_latest_session()
    if session_dir:
        print(f"  Session: {session_dir}")
    else:
        print("  警告: 未找到 session 目录")

    # 步骤 4: 重建地图
    print("[4/5] 重建地图...")
    run_cmd(
        "source /home/ydkj/NDT-slam-ws/devel/setup.bash && "
        "rosservice call /rebuild_map"
    )
    time.sleep(10)

    # 获取最新 rebuild 目录
    rebuild_dirs = sorted(Path(SESSION_BASE).glob("rebuild_*"), key=lambda p: p.stat().st_mtime)
    rebuild_dir = rebuild_dirs[-1] if rebuild_dirs else None

    if rebuild_dir:
        print(f"  Rebuild: {rebuild_dir}")
    else:
        print("  错误: 未找到 rebuild 目录")
        os.killpg(os.getpgid(slam_proc.pid), signal.SIGTERM)
        return 1

    # 步骤 5: 创建地图版本
    print("[5/5] 创建地图版本...")

    # 导入版本管理器
    sys.path.insert(0, os.path.dirname(__file__))
    from map_version_manager import MapVersionManager

    mgr = MapVersionManager(args.output)
    version_num = mgr.get_next_version()
    version_dir = mgr.create_version(version_num)

    # 复制地图文件
    mgr.copy_maps_to_version(version_dir, rebuild_dir)

    # 复制关键帧
    if session_dir:
        kf_count = mgr.copy_keyframes_to_version(version_dir, session_dir)
    else:
        kf_count = 0

    # 统计点数
    total_points = 0
    for f in version_dir.glob("*.pcd"):
        try:
            with open(f) as fh:
                for line in fh:
                    if line.startswith("POINTS"):
                        total_points += int(line.split()[1])
                        break
        except:
            pass

    # 生成元数据
    mgr.generate_metadata(version_dir, keyframe_count=kf_count, total_points=total_points)

    # 发布版本
    mgr.promote_version(version_num)

    # 停止 SLAM
    print("\n停止 SLAM...")
    os.killpg(os.getpgid(slam_proc.pid), signal.SIGTERM)

    # 完成
    print()
    print(f"=" * 60)
    print(f"建图完成！")
    print(f"=" * 60)
    print(f"版本: {version_dir.name}")
    print(f"路径: {version_dir}")
    print(f"关键帧: {kf_count}")
    print(f"总点数: {total_points}")
    print(f"当前版本: maps/current -> {version_dir.name}")
    print()
    print(f"下一步:")
    print(f"  查看地图: ls -lh {version_dir}/")
    print(f"  运行系统: roslaunch ndt_slam warehouse_runtime.launch")
    print()

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
