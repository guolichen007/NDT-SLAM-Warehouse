#!/usr/bin/env python3
"""
地图版本管理器
管理地图版本：创建、发布、回滚、列表
"""

import os
import sys
import json
import shutil
import yaml
from datetime import datetime
from pathlib import Path

MAPS_BASE = "/home/ydkj/NDT-slam-ws/maps"

class MapVersionManager:
    def __init__(self, base_dir=MAPS_BASE):
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.current_link = self.base_dir / "current"

    def list_versions(self):
        """列出所有地图版本"""
        versions = []
        for d in sorted(self.base_dir.iterdir()):
            if d.is_dir() and d.name.startswith("warehouse_v"):
                meta_file = d / "map_metadata.yaml"
                if meta_file.exists():
                    with open(meta_file) as f:
                        meta = yaml.safe_load(f)
                    versions.append({
                        "version": d.name,
                        "created": meta.get("created", "unknown"),
                        "keyframes": meta.get("keyframes", 0),
                        "points": meta.get("total_points", 0),
                    })
                else:
                    versions.append({"version": d.name, "created": "unknown"})
        return versions

    def get_latest_version(self):
        """获取最新版本号"""
        versions = self.list_versions()
        if not versions:
            return 0
        nums = []
        for v in versions:
            try:
                num = int(v["version"].split("_v")[1])
                nums.append(num)
            except (ValueError, IndexError):
                pass
        return max(nums) if nums else 0

    def get_next_version(self):
        """获取下一个版本号"""
        return self.get_latest_version() + 1

    def create_version(self, version_num=None, source_dir=None):
        """创建新版本目录"""
        if version_num is None:
            version_num = self.get_next_version()

        version_name = f"warehouse_v{version_num:03d}"
        version_dir = self.base_dir / version_name

        if version_dir.exists():
            print(f"版本 {version_name} 已存在")
            return version_dir

        version_dir.mkdir(parents=True, exist_ok=True)
        (version_dir / "keyframes").mkdir(exist_ok=True)

        print(f"创建版本目录: {version_dir}")
        return version_dir

    def copy_maps_to_version(self, version_dir, source_dir):
        """从 rebuild 目录复制地图文件到版本目录"""
        source = Path(source_dir)
        version_dir = Path(version_dir)

        map_files = [
            "map_registration.pcd",
            "map_display.pcd",
            "map_ground.pcd",
            "map_objects_raw.pcd",
            "map_objects_clean.pcd",
        ]

        copied = 0
        for f in map_files:
            src = source / f
            if src.exists():
                dst = version_dir / f.replace("map_", "").replace("_raw", "_raw")
                # 重命名为标准名称
                if f == "map_registration.pcd":
                    dst = version_dir / "registration_map.pcd"
                elif f == "map_display.pcd":
                    dst = version_dir / "display_map.pcd"
                elif f == "map_ground.pcd":
                    dst = version_dir / "ground_map.pcd"
                elif f == "map_objects_raw.pcd":
                    dst = version_dir / "objects_raw.pcd"
                elif f == "map_objects_clean.pcd":
                    dst = version_dir / "objects_clean.pcd"
                else:
                    dst = version_dir / f

                shutil.copy2(src, dst)
                copied += 1
                print(f"  复制: {f} -> {dst.name}")

        return copied

    def copy_keyframes_to_version(self, version_dir, session_dir):
        """从 session 目录复制关键帧到版本目录"""
        session = Path(session_dir)
        version_dir = Path(version_dir)
        kf_dir = version_dir / "keyframes"

        src_kf = session / "keyframes"
        if not src_kf.exists():
            print(f"关键帧目录不存在: {src_kf}")
            return 0

        count = 0
        for f in src_kf.iterdir():
            if f.suffix == ".pcd":
                shutil.copy2(f, kf_dir / f.name)
                count += 1

        print(f"  复制 {count} 个关键帧")

        # 复制位姿文件
        for pose_file in ["poses_raw.txt", "poses_optimized.txt"]:
            src = session / pose_file
            if src.exists():
                shutil.copy2(src, version_dir / pose_file)
                print(f"  复制: {pose_file}")

        return count

    def generate_metadata(self, version_dir, keyframe_count=0, total_points=0, params=None):
        """生成地图元数据"""
        version_dir = Path(version_dir)
        version_name = version_dir.name

        metadata = {
            "version": version_name,
            "created": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "coordinate_frame": "map",
            "keyframes": keyframe_count,
            "total_points": total_points,
            "maps": {
                "localization": "registration_map.pcd",
                "display": "display_map.pcd",
                "ground": "ground_map.pcd",
                "objects_raw": "objects_raw.pcd",
                "objects_clean": "objects_clean.pcd",
            },
            "voxel_sizes": {
                "registration": 0.3,
                "display": 0.1,
                "ground": 0.15,
                "objects": 0.06,
            },
        }

        if params:
            metadata["params"] = params

        meta_file = version_dir / "map_metadata.yaml"
        with open(meta_file, "w") as f:
            yaml.dump(metadata, f, default_flow_style=False, allow_unicode=True)

        print(f"  生成元数据: {meta_file}")

    def promote_version(self, version_num):
        """将指定版本设为当前版本（更新软链接）"""
        version_name = f"warehouse_v{version_num:03d}"
        version_dir = self.base_dir / version_name

        if not version_dir.exists():
            print(f"版本不存在: {version_name}")
            return False

        # 删除旧软链接
        if self.current_link.exists() or self.current_link.is_symlink():
            self.current_link.unlink()

        # 创建新软链接
        self.current_link.symlink_to(version_dir)
        print(f"当前版本已切换到: {version_name}")
        return True

    def get_current_version(self):
        """获取当前使用的版本"""
        if self.current_link.exists():
            target = self.current_link.resolve()
            return target.name
        return None

    def rollback(self):
        """回滚到上一个版本"""
        versions = self.list_versions()
        current = self.get_current_version()

        if not current or len(versions) < 2:
            print("无法回滚：没有其他版本")
            return False

        # 找到当前版本的前一个
        current_idx = None
        for i, v in enumerate(versions):
            if v["version"] == current:
                current_idx = i
                break

        if current_idx is None or current_idx == 0:
            print("无法回滚：已是最早版本")
            return False

        prev_version = versions[current_idx - 1]["version"]
        prev_num = int(prev_version.split("_v")[1])
        return self.promote_version(prev_num)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="地图版本管理器")
    subparsers = parser.add_subparsers(dest="command")

    # list
    subparsers.add_parser("list", help="列出所有版本")

    # current
    subparsers.add_parser("current", help="显示当前版本")

    # promote
    promote_parser = subparsers.add_parser("promote", help="发布版本")
    promote_parser.add_argument("version", type=int, help="版本号")

    # rollback
    subparsers.add_parser("rollback", help="回滚到上一版本")

    # create
    create_parser = subparsers.add_parser("create", help="创建新版本")
    create_parser.add_argument("--source", help="rebuild 目录路径")
    create_parser.add_argument("--session", help="session 目录路径（关键帧）")
    create_parser.add_argument("--version", type=int, help="版本号（默认自动递增）")

    args = parser.parse_args()
    mgr = MapVersionManager()

    if args.command == "list":
        versions = mgr.list_versions()
        if not versions:
            print("没有地图版本")
        else:
            current = mgr.get_current_version()
            for v in versions:
                marker = " <-- current" if v["version"] == current else ""
                print(f"  {v['version']}  created={v.get('created', '?')}  keyframes={v.get('keyframes', '?')}{marker}")

    elif args.command == "current":
        current = mgr.get_current_version()
        if current:
            print(f"当前版本: {current}")
            print(f"路径: {mgr.current_link.resolve()}")
        else:
            print("未设置当前版本")

    elif args.command == "promote":
        mgr.promote_version(args.version)

    elif args.command == "rollback":
        mgr.rollback()

    elif args.command == "create":
        version_dir = mgr.create_version(args.version)

        if args.source:
            mgr.copy_maps_to_version(version_dir, args.source)

        if args.session:
            kf_count = mgr.copy_keyframes_to_version(version_dir, args.session)

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

        mgr.generate_metadata(version_dir, keyframe_count=kf_count if args.session else 0, total_points=total_points)
        print(f"\n版本创建完成: {version_dir}")

    else:
        parser.print_help()


if __name__ == "__main__":
    main()
