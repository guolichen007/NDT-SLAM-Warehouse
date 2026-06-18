#!/usr/bin/env python3
"""
地图质量分析脚本
分析 PCD 点云地图的质量指标，输出 JSON 报告
"""

import argparse
import json
import os
import sys
import numpy as np
from pathlib import Path

def read_pcd(filename):
    """读取 PCD 文件（支持 ASCII 和 Binary）"""
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

        # 解析字段
        fields = header.get('FIELDS', ['x', 'y', 'z'])
        num_points = int(header.get('POINTS', [0])[0])

        if data_type == 'ascii':
            points = []
            for _ in range(num_points):
                line = f.readline().decode('utf-8').strip()
                if line:
                    vals = [float(x) for x in line.split()]
                    points.append(vals[:3])  # 只取 xyz
            return np.array(points) if points else np.empty((0, 3))
        else:
            # Binary
            data = f.read()
            if data_type == 'binary_compressed':
                import struct
                # 简化处理，实际需要解压缩
                print(f"Warning: binary_compressed not fully supported")
                return np.empty((0, 3))
            else:
                # Binary
                dtype = np.float32
                if len(fields) == 3:
                    dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
                else:
                    dtype = np.dtype([(f, np.float32) for f in fields])
                points = np.frombuffer(data, dtype=dtype)
                return np.column_stack([points['x'], points['y'], points['z']])

def analyze_map(pcd_file, cell_size=0.10):
    """分析单个地图文件"""
    print(f"  读取: {pcd_file}")
    points = read_pcd(pcd_file)

    if len(points) == 0:
        return {"error": "Empty or failed to read"}

    # 基本统计
    xyz_min = points.min(axis=0).tolist()
    xyz_max = points.max(axis=0).tolist()
    xyz_range = (points.max(axis=0) - points.min(axis=0)).tolist()

    z_values = points[:, 2]
    z_min = float(z_values.min())
    z_max = float(z_values.max())
    z_mean = float(z_values.mean())
    z_std = float(z_values.std())

    # 低高度比例
    low_height_ratio = float((z_values < 0.3).sum() / len(z_values))
    negative_z_ratio = float((z_values < 0).sum() / len(z_values))

    # XY 网格分析
    xy = points[:, :2]
    xy_min = xy.min(axis=0)
    xy_max = xy.max(axis=0)

    # 计算网格索引
    cell_indices = np.floor((xy - xy_min) / cell_size).astype(int)
    cell_indices = np.clip(cell_indices, 0, None)

    # 统计每个网格的点数和垂向跨度
    from collections import defaultdict
    cell_stats = defaultdict(list)
    for i, (ci, z) in enumerate(zip(map(tuple, cell_indices), z_values)):
        cell_stats[ci].append(z)

    cell_vertical_spans = []
    cell_point_counts = []
    for key, z_list in cell_stats.items():
        z_arr = np.array(z_list)
        span = z_arr.max() - z_arr.min()
        cell_vertical_spans.append(span)
        cell_point_counts.append(len(z_list))

    cell_vertical_spans = np.array(cell_vertical_spans)
    cell_point_counts = np.array(cell_point_counts)

    # 输出指标
    result = {
        "point_count": int(len(points)),
        "xyz_min": xyz_min,
        "xyz_max": xyz_max,
        "xyz_range": xyz_range,
        "z_min": z_min,
        "z_max": z_max,
        "z_mean": z_mean,
        "z_std": z_std,
        "low_height_ratio": round(low_height_ratio, 4),
        "negative_z_ratio": round(negative_z_ratio, 4),
        "xy_cell_size": cell_size,
        "xy_occupied_cell_count": int(len(cell_stats)),
        "xy_mean_points_per_cell": round(float(cell_point_counts.mean()), 2) if len(cell_point_counts) > 0 else 0,
        "xy_vertical_span_median": round(float(np.median(cell_vertical_spans)), 4) if len(cell_vertical_spans) > 0 else 0,
        "xy_vertical_span_p90": round(float(np.percentile(cell_vertical_spans, 90)), 4) if len(cell_vertical_spans) > 0 else 0,
        "xy_vertical_span_gt_0_3_ratio": round(float((cell_vertical_spans > 0.3).sum() / len(cell_vertical_spans)), 4) if len(cell_vertical_spans) > 0 else 0,
    }

    # 问题诊断
    problems = []
    if low_height_ratio > 0.3:
        problems.append(f"Low height points ratio too high: {low_height_ratio:.2%}")
    if negative_z_ratio > 0.1:
        problems.append(f"Negative Z points ratio: {negative_z_ratio:.2%}")
    if result["xy_vertical_span_gt_0_3_ratio"] > 0.3:
        problems.append(f"Vertical span > 0.3m cells ratio: {result['xy_vertical_span_gt_0_3_ratio']:.2%}")
    if result["xy_vertical_span_p90"] > 0.5:
        problems.append(f"Vertical span P90 too high: {result['xy_vertical_span_p90']:.2f}m")

    result["estimated_problem"] = problems

    return result

def print_summary(map_name, report):
    """打印简明统计表"""
    print(f"\n{'='*60}")
    print(f"地图: {map_name}")
    print(f"{'='*60}")
    print(f"  点数: {report['point_count']:,}")
    print(f"  XYZ 范围: [{report['xyz_range'][0]:.2f}, {report['xyz_range'][1]:.2f}, {report['xyz_range'][2]:.2f}] m")
    print(f"  Z 范围: [{report['z_min']:.2f}, {report['z_max']:.2f}] m")
    print(f"  Z 均值/标准差: {report['z_mean']:.3f} / {report['z_std']:.3f} m")
    print(f"  Z < 0 比例: {report['negative_z_ratio']:.2%}")
    print(f"  Z < 0.3m 比例: {report['low_height_ratio']:.2%}")
    print(f"  XY 网格数: {report['xy_occupied_cell_count']:,}")
    print(f"  每网格平均点数: {report['xy_mean_points_per_cell']:.1f}")
    print(f"  垂向厚度中位数: {report['xy_vertical_span_median']:.3f} m")
    print(f"  垂向厚度 P90: {report['xy_vertical_span_p90']:.3f} m")
    print(f"  垂向厚度 > 0.3m 网格比例: {report['xy_vertical_span_gt_0_3_ratio']:.2%}")
    if report['estimated_problem']:
        print(f"  问题诊断:")
        for p in report['estimated_problem']:
            print(f"    ⚠️  {p}")
    else:
        print(f"  问题诊断: ✅ 无明显问题")

def main():
    parser = argparse.ArgumentParser(description='地图质量分析')
    parser.add_argument('--map_dir', required=True, help='地图目录')
    parser.add_argument('--output', required=True, help='输出 JSON 文件')
    parser.add_argument('--cell_size', type=float, default=0.10, help='XY 网格大小 (m)')
    args = parser.parse_args()

    map_dir = Path(args.map_dir)
    if not map_dir.exists():
        print(f"错误: 地图目录不存在 {map_dir}")
        sys.exit(1)

    # 需要分析的地图文件
    map_files = {
        'registration_map': 'registration_map.pcd',
        'display_map': 'display_map.pcd',
        'ground_map': 'ground_map.pcd',
        'objects_raw': 'objects_raw.pcd',
    }

    report = {}
    for map_name, filename in map_files.items():
        pcd_path = map_dir / filename
        if pcd_path.exists():
            print(f"\n分析 {map_name}...")
            report[map_name] = analyze_map(str(pcd_path), args.cell_size)
            report[map_name]['map_name'] = map_name
            report[map_name]['file'] = str(pcd_path)
            print_summary(map_name, report[map_name])
        else:
            print(f"\n跳过 {map_name}: 文件不存在 {pcd_path}")
            report[map_name] = {"error": "File not found"}

    # 保存报告
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(f"\n{'='*60}")
    print(f"报告已保存: {output_path}")
    print(f"{'='*60}")

if __name__ == '__main__':
    main()
