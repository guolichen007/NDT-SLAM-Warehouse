#!/usr/bin/env python3
"""
地面清理脚本
使用局部地面模型生成干净的地面地图
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
    """保存 PCD 文件（ASCII 格式）"""
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

def build_ground_model(points, grid_size=0.5, quantile=0.10):
    """构建局部地面模型"""
    print(f"  构建局部地面模型 (grid={grid_size}m, quantile={quantile})")

    xy = points[:, :2]
    z = points[:, 2]
    xy_min = xy.min(axis=0)

    # 计算网格索引
    cell_indices = np.floor((xy - xy_min) / grid_size).astype(int)

    # 每个网格的 Z 值
    cell_z = defaultdict(list)
    for ci, z_val in zip(map(tuple, cell_indices), z):
        cell_z[ci].append(z_val)

    # 计算每个网格的地面高度（低分位数）
    ground_model = {}
    for key, z_list in cell_z.items():
        z_arr = np.array(z_list)
        if len(z_arr) >= 3:
            ground_z = float(np.percentile(z_arr, quantile * 100))
            vertical_span = float(z_arr.max() - z_arr.min())
            ground_model[key] = {
                'ground_z': ground_z,
                'vertical_span': vertical_span,
                'point_count': len(z_arr)
            }
        else:
            ground_model[key] = {
                'ground_z': float(z_arr.min()),
                'vertical_span': 0.0,
                'point_count': len(z_arr)
            }

    return ground_model, xy_min

def get_local_ground(ground_model, xy_min, grid_size, x, y):
    """获取局部地面高度"""
    ci = (int(np.floor((x - xy_min[0]) / grid_size)),
          int(np.floor((y - xy_min[1]) / grid_size)))
    if ci in ground_model:
        return ground_model[ci]
    return None

def filter_ground_points(points, ground_model, xy_min, config):
    """过滤地面点"""
    grid_size = config['grid_size']
    keep_min = config['keep_height_min']
    keep_max = config['keep_height_max']
    max_span = config['max_cell_vertical_span']

    print(f"  过滤地面点 (height=[{keep_min}, {keep_max}]m, max_span={max_span}m)")

    kept_points = []
    removed_reasons = defaultdict(int)

    for p in points:
        info = get_local_ground(ground_model, xy_min, grid_size, p[0], p[1])
        if info is None:
            removed_reasons['no_ground_model'] += 1
            continue

        # 检查垂向跨度
        if info['vertical_span'] > max_span:
            removed_reasons['vertical_span_too_large'] += 1
            continue

        # 计算相对高度
        local_height = p[2] - info['ground_z']
        if keep_min <= local_height <= keep_max:
            kept_points.append(p)
        else:
            removed_reasons['height_out_of_range'] += 1

    print(f"    保留: {len(kept_points):,} 点")
    for reason, count in removed_reasons.items():
        print(f"    移除 ({reason}): {count:,} 点")

    return np.array(kept_points) if kept_points else np.empty((0, 3))

def radius_filter(points, radius=0.25, min_neighbors=4):
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

def save_ground_model_csv(ground_model, xy_min, grid_size, filename):
    """保存地面模型为 CSV"""
    os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
    with open(filename, 'w') as f:
        f.write("cell_x,cell_y,world_x,world_y,ground_z,vertical_span,point_count\n")
        for (cx, cy), info in ground_model.items():
            wx = xy_min[0] + cx * grid_size + grid_size / 2
            wy = xy_min[1] + cy * grid_size + grid_size / 2
            f.write(f"{cx},{cy},{wx:.3f},{wy:.3f},{info['ground_z']:.4f},{info['vertical_span']:.4f},{info['point_count']}\n")
    print(f"  地面模型已保存: {filename}")

def main():
    parser = argparse.ArgumentParser(description='地面清理')
    parser.add_argument('--config', required=True, help='配置文件路径')
    parser.add_argument('--input', help='输入 PCD 文件（覆盖配置）')
    parser.add_argument('--output', help='输出目录（覆盖配置）')
    args = parser.parse_args()

    # 读取配置
    import yaml
    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    gc_config = config.get('ground_clean', {})
    input_config = config.get('input', {})
    output_config = config.get('output', {})

    # 输入文件
    input_file = args.input or os.path.join(
        input_config.get('map_dir', 'maps/current'),
        gc_config.get('source', 'display_map.pcd')
    )

    # 输出目录
    output_dir = args.output or output_config.get('map_dir', 'maps/warehouse_v003')

    print(f"="*60)
    print(f"地面清理")
    print(f"="*60)
    print(f"  输入: {input_file}")
    print(f"  输出目录: {output_dir}")

    # 读取点云
    print(f"\n读取点云...")
    points = read_pcd(input_file)
    print(f"  原始点数: {len(points):,}")

    # 构建地面模型
    ground_model, xy_min = build_ground_model(
        points,
        grid_size=gc_config.get('grid_size', 0.5),
        quantile=gc_config.get('ground_quantile', 0.10)
    )

    # 过滤地面点
    ground_points = filter_ground_points(points, ground_model, xy_min, gc_config)

    # 半径离群过滤
    if gc_config.get('remove_isolated_points', True):
        ground_points = radius_filter(
            ground_points,
            radius=gc_config.get('radius_filter_radius', 0.25),
            min_neighbors=gc_config.get('radius_filter_min_neighbors', 4)
        )

    # 体素下采样
    ground_points = voxel_downsample(ground_points, gc_config.get('voxel_size', 0.08))

    # 保存结果
    os.makedirs(output_dir, exist_ok=True)

    ground_clean_file = os.path.join(output_dir, 'ground_map_clean.pcd')
    save_pcd(ground_clean_file, ground_points)
    print(f"\n  ground_map_clean.pcd 已保存: {len(ground_points):,} 点")

    # 保存地面模型
    model_file = os.path.join(output_dir, 'ground_model.csv')
    save_ground_model_csv(ground_model, xy_min, gc_config.get('grid_size', 0.5), model_file)

    # 生成报告
    report = {
        'input_file': input_file,
        'original_points': int(len(points)),
        'output_points': int(len(ground_points)),
        'config': gc_config,
        'ground_model_cells': len(ground_model),
    }

    report_file = os.path.join(output_dir, 'ground_clean_report.json')
    with open(report_file, 'w') as f:
        json.dump(report, f, indent=2)
    print(f"  报告已保存: {report_file}")

    print(f"\n{'='*60}")
    print(f"地面清理完成!")
    print(f"{'='*60}")

if __name__ == '__main__':
    main()
