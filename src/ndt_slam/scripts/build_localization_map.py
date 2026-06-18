#!/usr/bin/env python3
"""
精定位地图生成脚本
生成适合 10cm 精定位的稳定结构地图
"""

import argparse
import json
import os
import sys
import numpy as np
from pathlib import Path
from collections import defaultdict

def read_pcd(filename):
    """读取 PCD 文件"""
    with open(filename, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('DATA'):
                data_type = line.split()[1]
                break
            parts = line.split()
            if len(parts) >= 2:
                header[parts[0]] = parts[1:]

        fields = header.get('FIELDS', ['x', 'y', 'z'])
        num_points = int(header.get('POINTS', [0])[0])

        if data_type == 'ascii':
            points = []
            for _ in range(num_points):
                line = f.readline().decode('utf-8').strip()
                if line:
                    vals = [float(x) for x in line.split()]
                    points.append(vals[:3])
            return np.array(points) if points else np.empty((0, 3))
        else:
            data = f.read()
            dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
            points = np.frombuffer(data, dtype=dtype)
            return np.column_stack([points['x'], points['y'], points['z']])

def save_pcd(filename, points):
    """保存 PCD 文件"""
    os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

def load_ground_model(filename):
    """加载地面模型"""
    print(f"  加载地面模型: {filename}")

    grid = {}
    with open(filename, 'r') as f:
        header = f.readline()
        for line in f:
            parts = line.strip().split(',')
            if len(parts) >= 7:
                cx = int(parts[0])
                cy = int(parts[1])
                ground_z = float(parts[4])
                grid[(cx, cy)] = ground_z

    print(f"    加载了 {len(grid)} 个网格")
    return grid

def filter_for_localization(points, ground_model, xy_min, grid_size, config):
    """过滤定位地图点"""
    min_height = config['min_height_above_ground']
    max_height = config['max_height_above_ground']

    print(f"  过滤定位点 (height=[{min_height}, {max_height}]m)")

    kept_points = []
    removed_low = 0
    removed_high = 0

    for p in points:
        ci = (int(np.floor((p[0] - xy_min[0]) / grid_size)),
              int(np.floor((p[1] - xy_min[1]) / grid_size)))

        if ci not in ground_model:
            continue

        ground_z = ground_model[ci]
        local_height = p[2] - ground_z

        if local_height < min_height:
            removed_low += 1
        elif local_height > max_height:
            removed_high += 1
        else:
            kept_points.append(p)

    print(f"    保留: {len(kept_points):,} 点")
    print(f"    移除 (too_low): {removed_low:,} 点")
    print(f"    移除 (too_high): {removed_high:,} 点")

    return np.array(kept_points) if kept_points else np.empty((0, 3))

def sor_filter(points, mean_k=15, stddev_mul=1.5):
    """统计离群过滤 (SOR)"""
    print(f"  SOR 过滤 (mean_k={mean_k}, stddev_mul={stddev_mul})")

    if len(points) < 100:
        return points

    from scipy.spatial import cKDTree
    tree = cKDTree(points)
    distances, _ = tree.query(points, k=mean_k)
    mean_distances = distances.mean(axis=1)

    global_mean = mean_distances.mean()
    global_std = mean_distances.std()
    threshold = global_mean + stddev_mul * global_std

    mask = mean_distances < threshold
    filtered = points[mask]
    print(f"    保留: {len(filtered):,} / {len(points):,} 点")
    return filtered

def radius_filter(points, radius=0.30, min_neighbors=4):
    """半径离群过滤"""
    print(f"  半径离群过滤 (radius={radius}m, min_neighbors={min_neighbors})")

    if len(points) < 100:
        return points

    from scipy.spatial import cKDTree
    tree = cKDTree(points)
    neighbors = tree.query_ball_point(points, radius)
    neighbor_counts = np.array([len(n) for n in neighbors])

    mask = neighbor_counts >= min_neighbors
    filtered = points[mask]
    print(f"    保留: {len(filtered):,} / {len(points):,} 点")
    return filtered

def voxel_downsample(points, voxel_size):
    """体素下采样"""
    print(f"  体素下采样 (size={voxel_size}m)")

    if len(points) == 0:
        return points

    voxel_indices = np.floor(points / voxel_size).astype(int)
    voxel_dict = defaultdict(list)
    for i, vi in enumerate(map(tuple, voxel_indices)):
        voxel_dict[vi].append(i)

    downsampled = []
    for indices in voxel_dict.values():
        downsampled.append(points[indices].mean(axis=0))

    result = np.array(downsampled)
    print(f"    {len(points):,} -> {len(result):,} 点")
    return result

def main():
    parser = argparse.ArgumentParser(description='精定位地图生成')
    parser.add_argument('--config', required=True, help='配置文件路径')
    parser.add_argument('--input', help='输入 PCD 文件（覆盖配置）')
    parser.add_argument('--ground_model', help='地面模型文件（覆盖配置）')
    parser.add_argument('--output', help='输出目录（覆盖配置）')
    args = parser.parse_args()

    # 读取配置
    import yaml
    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    lm_config = config.get('localization_map', {})
    input_config = config.get('input', {})
    output_config = config.get('output', {})

    # 输入文件
    input_file = args.input or os.path.join(
        input_config.get('map_dir', 'maps/current'),
        lm_config.get('source', 'display_map.pcd')
    )

    # 地面模型文件
    ground_model_file = args.ground_model or os.path.join(
        output_config.get('map_dir', 'maps/warehouse_v003'),
        lm_config.get('ground_model', 'ground_model.csv')
    )

    # 输出目录
    output_dir = args.output or output_config.get('map_dir', 'maps/warehouse_v003')

    print(f"="*60)
    print(f"精定位地图生成")
    print(f"="*60)
    print(f"  输入: {input_file}")
    print(f"  地面模型: {ground_model_file}")
    print(f"  输出目录: {output_dir}")

    # 读取点云
    print(f"\n读取点云...")
    points = read_pcd(input_file)
    print(f"  原始点数: {len(points):,}")

    # 加载地面模型
    ground_model = load_ground_model(ground_model_file)

    # 计算 xy_min
    xy_min = points[:, :2].min(axis=0)

    # 过滤定位点
    localization_points = filter_for_localization(
        points, ground_model, xy_min,
        config.get('ground_clean', {}).get('grid_size', 0.5),
        lm_config
    )

    # SOR 过滤
    localization_points = sor_filter(
        localization_points,
        mean_k=lm_config.get('sor_mean_k', 15),
        stddev_mul=lm_config.get('sor_stddev_mul', 1.5)
    )

    # 半径离群过滤
    localization_points = radius_filter(
        localization_points,
        radius=lm_config.get('radius_filter_radius', 0.30),
        min_neighbors=lm_config.get('radius_filter_min_neighbors', 4)
    )

    # 体素下采样
    localization_points = voxel_downsample(localization_points, lm_config.get('voxel_size', 0.10))

    # 保存结果
    os.makedirs(output_dir, exist_ok=True)

    localization_file = os.path.join(output_dir, 'localization_map_fine.pcd')
    save_pcd(localization_file, localization_points)
    print(f"\n  localization_map_fine.pcd 已保存: {len(localization_points):,} 点")

    # 生成报告
    report = {
        'input_file': input_file,
        'ground_model_file': ground_model_file,
        'original_points': int(len(points)),
        'output_points': int(len(localization_points)),
        'config': lm_config,
    }

    report_file = os.path.join(output_dir, 'localization_map_report.json')
    with open(report_file, 'w') as f:
        json.dump(report, f, indent=2)
    print(f"  报告已保存: {report_file}")

    print(f"\n{'='*60}")
    print(f"精定位地图生成完成!")
    print(f"{'='*60}")

if __name__ == '__main__':
    main()
