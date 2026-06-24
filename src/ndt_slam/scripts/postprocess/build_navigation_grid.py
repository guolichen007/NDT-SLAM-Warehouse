#!/usr/bin/env python3
"""
导航栅格生成脚本
从 ground_map_clean 和 objects_clean 生成 2D 导航栅格图
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

def save_pgm(filename, grid):
    """保存 PGM 文件"""
    os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
    height, width = grid.shape
    with open(filename, 'wb') as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        grid.tofile(f)

def save_png(filename, grid):
    """保存 PNG 文件（可视化用）"""
    try:
        from PIL import Image
        os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
        img = Image.fromarray(grid.astype(np.uint8), mode='L')
        img.save(filename)
        print(f"  PNG 已保存: {filename}")
    except ImportError:
        print(f"  跳过 PNG 保存（需要 PIL）")

def save_yaml(filename, resolution, origin, image_file):
    """保存 YAML 配置文件"""
    os.makedirs(os.path.dirname(filename) or '.', exist_ok=True)
    with open(filename, 'w') as f:
        f.write(f"image: {os.path.basename(image_file)}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{origin[0]}, {origin[1]}, 0.0]\n")
        f.write(f"negate: 0\n")
        f.write(f"occupied_thresh: 0.65\n")
        f.write(f"free_thresh: 0.196\n")

def inflate_obstacles(grid, inflation_radius, resolution):
    """障碍物膨胀"""
    print(f"  障碍物膨胀 (radius={inflation_radius}m)")

    inflation_cells = int(np.ceil(inflation_radius / resolution))
    obstacle_mask = (grid == 255)

    # 创建膨胀核
    y, x = np.ogrid[-inflation_cells:inflation_cells+1, -inflation_cells:inflation_cells+1]
    kernel = (x*x + y*y) <= inflation_cells*inflation_cells

    # 膨胀
    from scipy.ndimage import binary_dilation
    inflated = binary_dilation(obstacle_mask, structure=kernel)

    # 更新栅格
    grid[inflated & (grid != 255)] = 200  # 膨胀区域标记为 200

    return grid

def remove_small_obstacles(grid, min_area, resolution):
    """删除小面积障碍物"""
    print(f"  删除小障碍 (min_area={min_area}m²)")

    obstacle_mask = (grid == 255)

    # 连通域分析
    from scipy.ndimage import label
    labeled, num_features = label(obstacle_mask)

    min_pixels = int(min_area / (resolution * resolution))
    removed_count = 0

    for i in range(1, num_features + 1):
        region_size = (labeled == i).sum()
        if region_size < min_pixels:
            grid[labeled == i] = 128  # 标记为 free
            removed_count += 1

    print(f"    删除了 {removed_count} 个小障碍")
    return grid

def main():
    parser = argparse.ArgumentParser(description='导航栅格生成')
    parser.add_argument('--config', required=True, help='配置文件路径')
    parser.add_argument('--ground', help='地面 PCD 文件（覆盖配置）')
    parser.add_argument('--objects', help='物体 PCD 文件（覆盖配置）')
    parser.add_argument('--output', help='输出目录（覆盖配置）')
    args = parser.parse_args()

    # 读取配置
    import yaml
    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    ng_config = config.get('navigation_grid', {})
    output_config = config.get('output', {})

    # 输出目录
    output_dir = args.output or output_config.get('map_dir', 'maps/warehouse_v003')

    # 输入文件
    ground_file = args.ground or os.path.join(output_dir, 'ground_map_clean.pcd')
    objects_file = args.objects or os.path.join(output_dir, 'objects_clean.pcd')

    resolution = ng_config.get('resolution', 0.05)
    inflation_radius = ng_config.get('obstacle_inflation_radius', 0.30)
    min_obstacle_area = ng_config.get('min_obstacle_area', 0.05)
    unknown_as_obstacle = ng_config.get('unknown_as_obstacle', True)

    print(f"="*60)
    print(f"导航栅格生成")
    print(f"="*60)
    print(f"  地面: {ground_file}")
    print(f"  物体: {objects_file}")
    print(f"  输出目录: {output_dir}")
    print(f"  分辨率: {resolution}m")

    # 读取点云
    print(f"\n读取点云...")
    ground_points = read_pcd(ground_file)
    object_points = read_pcd(objects_file)
    print(f"  地面点: {len(ground_points):,}")
    print(f"  物体点: {len(object_points):,}")

    # 计算地图边界
    all_points = np.vstack([ground_points, object_points])
    xy_min = all_points[:, :2].min(axis=0)
    xy_max = all_points[:, :2].max(axis=0)

    # 添加边距
    margin = 2.0
    xy_min -= margin
    xy_max += margin

    # 计算栅格大小
    width = int(np.ceil((xy_max[0] - xy_min[0]) / resolution))
    height = int(np.ceil((xy_max[1] - xy_min[1]) / resolution))

    print(f"  栅格大小: {width} x {height}")

    # 初始化栅格 (0=unknown, 128=free, 255=obstacle)
    grid = np.zeros((height, width), dtype=np.uint8)  # 默认为 unknown

    # 标记物体为 obstacle
    print(f"  标记物体...")
    for p in object_points:
        ix = int((p[0] - xy_min[0]) / resolution)
        iy = int((p[1] - xy_min[1]) / resolution)
        if 0 <= ix < width and 0 <= iy < height:
            grid[iy, ix] = 255

    # 标记地面为 free（只有在没有被标记为 obstacle 的位置）
    print(f"  标记地面...")
    for p in ground_points:
        ix = int((p[0] - xy_min[0]) / resolution)
        iy = int((p[1] - xy_min[1]) / resolution)
        if 0 <= ix < width and 0 <= iy < height:
            if grid[iy, ix] != 255:  # 不覆盖 obstacle
                grid[iy, ix] = 128

    # 障碍物膨胀
    grid = inflate_obstacles(grid, inflation_radius, resolution)

    # 删除小障碍
    if ng_config.get('remove_small_obstacles', True):
        grid = remove_small_obstacles(grid, min_obstacle_area, resolution)

    # 保存结果
    os.makedirs(output_dir, exist_ok=True)

    pgm_file = os.path.join(output_dir, f'navigation_grid_{resolution}m.pgm')
    save_pgm(pgm_file, grid)
    print(f"\n  PGM 已保存: {pgm_file}")

    png_file = os.path.join(output_dir, f'navigation_grid_{resolution}m.png')
    save_png(png_file, grid)

    yaml_file = os.path.join(output_dir, f'navigation_grid_{resolution}m.yaml')
    save_yaml(yaml_file, resolution, xy_min, pgm_file)
    print(f"  YAML 已保存: {yaml_file}")

    # 统计
    free_pixels = (grid == 128).sum()
    obstacle_pixels = (grid == 255).sum()
    inflated_pixels = (grid == 200).sum()
    unknown_pixels = (grid == 0).sum()
    total_pixels = width * height

    print(f"\n  统计:")
    print(f"    Free: {free_pixels:,} ({free_pixels/total_pixels:.2%})")
    print(f"    Obstacle: {obstacle_pixels:,} ({obstacle_pixels/total_pixels:.2%})")
    print(f"    Inflated: {inflated_pixels:,} ({inflated_pixels/total_pixels:.2%})")
    print(f"    Unknown: {unknown_pixels:,} ({unknown_pixels/total_pixels:.2%})")

    # 生成报告
    report = {
        'ground_file': ground_file,
        'objects_file': objects_file,
        'resolution': resolution,
        'width': width,
        'height': height,
        'origin': xy_min.tolist(),
        'inflation_radius': inflation_radius,
        'statistics': {
            'free_pixels': int(free_pixels),
            'obstacle_pixels': int(obstacle_pixels),
            'inflated_pixels': int(inflated_pixels),
            'unknown_pixels': int(unknown_pixels),
            'total_pixels': int(total_pixels),
            'free_ratio': round(free_pixels/total_pixels, 4),
            'obstacle_ratio': round(obstacle_pixels/total_pixels, 4),
        }
    }

    report_file = os.path.join(output_dir, 'navigation_report.json')
    with open(report_file, 'w') as f:
        json.dump(report, f, indent=2)
    print(f"  报告已保存: {report_file}")

    print(f"\n{'='*60}")
    print(f"导航栅格生成完成!")
    print(f"{'='*60}")

if __name__ == '__main__':
    main()
