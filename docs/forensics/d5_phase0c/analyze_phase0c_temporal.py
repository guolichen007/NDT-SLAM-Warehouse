#!/usr/bin/env python3
"""Phase 0C temporal signal：map-cell 聚类后，F1 vs F2/F3 的 base/map 运动差异。"""
import csv
import math
import bisect
from collections import defaultdict, Counter

FRAG = "/tmp/cargo_forensic/d5_weak_fragments.csv"
POSE = "/tmp/cargo_forensic/phase0c_pose_source.csv"
WIN_START, WIN_END = 1787034958.42, 1787035076.80


def fnum(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return float("nan")


def classify(r):
    t = fnum(r, "stamp")
    hs = r["oracle_high_surface"] == "1"
    in_win = WIN_START <= t <= WIN_END
    ov = fnum(r, "xy_projected_overlap")
    if hs and in_win:
        return "F1"
    if hs and not in_win:
        return "F2"
    if not hs and ov < 0.5:
        return "F3"
    return "F4"


def to_map(tx, ty, yaw, bx, by):
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + bx * c - by * s, ty + bx * s + by * c


def main():
    frags = list(csv.DictReader(open(FRAG)))
    pose_by_stamp = {}
    for r in csv.DictReader(open(POSE)):
        pose_by_stamp[float(r["stamp"])] = r
    pose_stamps = sorted(pose_by_stamp)

    def nearest_pose(t):
        i = bisect.bisect_left(pose_stamps, t)
        if i == 0:
            return pose_by_stamp[pose_stamps[0]]
        if i >= len(pose_stamps):
            return pose_by_stamp[pose_stamps[-1]]
        a, b = pose_stamps[i - 1], pose_stamps[i]
        return pose_by_stamp[a] if abs(a - t) < abs(b - t) else pose_by_stamp[b]

    # 每个 fragment 转 map，按 0.25m grid 聚类
    grid = defaultdict(list)
    for r in frags:
        t = fnum(r, "stamp")
        p = nearest_pose(t)
        tx, ty, yaw = fnum(p, "pose_tx"), fnum(p, "pose_ty"), fnum(p, "pose_yaw")
        bx, by = fnum(r, "center_x"), fnum(r, "center_y")
        mx, my = to_map(tx, ty, yaw, bx, by)
        key = (int(mx / 0.25), int(my / 0.25))
        grid[key].append((t, bx, by, mx, my, classify(r)))

    # 对每个 >=4 点的 cell，计算 base/map spread + 主导分类
    def spread(vals):
        vals = sorted(vals)
        return vals[-1] - vals[0]

    rows = []
    for key, pts in grid.items():
        if len(pts) < 4:
            continue
        bx = spread([p[1] for p in pts])
        by = spread([p[2] for p in pts])
        mx = spread([p[3] for p in pts])
        my = spread([p[4] for p in pts])
        cls = Counter(p[5] for p in pts).most_common(1)[0][0]
        rows.append((cls, len(pts), math.hypot(bx, by), math.hypot(mx, my)))

    # 按主导分类统计 base/map spread
    by_cls = defaultdict(list)
    for cls, n, base_disp, map_disp in rows:
        by_cls[cls].append((base_disp, map_disp))

    print("=== 每个 map-cell（>=4 点）的 base spread vs map spread（按主导分类）===")
    for cls in sorted(by_cls):
        v = by_cls[cls]
        n = len(v)
        base_med = sorted(x[0] for x in v)[n // 2]
        map_med = sorted(x[1] for x in v)[n // 2]
        static_like = sum(1 for b, m in v if b > 0.5 and m < 0.3)
        cargo_like = sum(1 for b, m in v if b < 0.3 and m > 0.5)
        print(f"  {cls}: cells={n} base_spread_p50={base_med:.2f} map_spread_p50={map_med:.2f} "
              f"static_like={static_like} cargo_like={cargo_like}")

    print()
    print("判据：static_like = base 随 pose 反向移动(map固定)；cargo_like = base 固定(map 随 pose 移动)")
    print("F1 应 cargo_like；F2/F3 应 static_like")


if __name__ == "__main__":
    main()
