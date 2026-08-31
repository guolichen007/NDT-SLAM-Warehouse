#!/usr/bin/env python3
"""Phase 0C 分析：mature static + base/map temporal signal（离线，只读 CSV）。"""
import csv
import math
from collections import defaultdict

FRAG = "/tmp/cargo_forensic/d5_weak_fragments.csv"
POSE = "/tmp/cargo_forensic/phase0c_pose_source.csv"

WIN_START, WIN_END = 1787034958.42, 1787035076.80  # 无.bag oracle safe-over window


def load_frag():
    rows = []
    with open(FRAG) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def load_pose():
    pose_by_stamp = {}
    with open(POSE) as f:
        for r in csv.DictReader(f):
            t = float(r["stamp"])
            pose_by_stamp[t] = r
    return pose_by_stamp


def fnum(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return float("nan")


def to_map(tx, ty, yaw, bx, by):
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + bx * c - by * s, ty + bx * s + by * c


def main():
    frags = load_frag()
    poses = load_pose()
    print(f"weak fragments: {len(frags)}, pose stamps: {len(poses)}")

    # 1. mature static 状态
    static_cells = []
    with open("/tmp/cargo_forensic/phase0c_static_cells.csv") as f:
        for r in csv.DictReader(f):
            static_cells.append(r)
    mature = [c for c in static_cells if c["temporally_mature"] == "1"]
    clean = [c for c in static_cells if c["clean_map_confirmed"] == "1"]
    print(f"static cells: {len(static_cells)}, clean_confirmed={len(clean)}, temporally_mature={len(mature)}")
    print()

    # 2. F1-F4 分类（oracle window + oracle_high_surface）
    def classify(r):
        t = fnum(r, "stamp")
        hs = r["oracle_high_surface"] == "1"
        in_win = WIN_START <= t <= WIN_END
        ov = fnum(r, "xy_projected_overlap")
        if hs and in_win:
            return "F1_TRUE_CARGO_SURFACE"
        if hs and not in_win:
            return "F2_STATIC_HIGH"
        if not hs and ov < 0.5:
            return "F3_EXTERNAL_GROUND_OR_SIDE"
        return "F4_UNKNOWN"

    from collections import Counter
    cls = Counter(classify(r) for r in frags)
    print("=== F1-F4 分类（无.bag）===")
    for k, v in cls.most_common():
        print(f"  {k}: {v}")
    print()

    # 3. base/map temporal signal：fragment base center → map center（用 pose）
    #    对每个 fragment，找最近时间戳的 pose，转 map
    pose_stamps = sorted(poses.keys())
    def nearest_pose(t):
        # 二分最近
        import bisect
        i = bisect.bisect_left(pose_stamps, t)
        if i == 0:
            return poses[pose_stamps[0]]
        if i == len(pose_stamps):
            return poses[pose_stamps[-1]]
        a, b = pose_stamps[i - 1], pose_stamps[i]
        return poses[a] if abs(a - t) < abs(b - t) else poses[b]

    # 每个 fragment 转 map center
    map_points = []
    for r in frags:
        t = fnum(r, "stamp")
        p = nearest_pose(t)
        tx, ty, yaw = fnum(p, "pose_tx"), fnum(p, "pose_ty"), fnum(p, "pose_yaw")
        bx, by = fnum(r, "center_x"), fnum(r, "center_y")
        mx, my = to_map(tx, ty, yaw, bx, by)
        map_points.append((t, bx, by, mx, my, classify(r), r))

    # map 空间聚类（1m grid），分析每个簇的 base 坐标 vs pose 运动
    grid = defaultdict(list)
    for t, bx, by, mx, my, c, r in map_points:
        key = (int(mx / 1.0), int(my / 1.0))
        grid[key].append((t, bx, by, mx, my, c))

    # 对每个簇（>=5 个点），分析 base 坐标变化 vs pose 变化
    print("=== map 空间簇（>=5 点）的 base 运动 vs pose 运动 ===")
    static_like = cargo_like = 0
    for key, pts in grid.items():
        if len(pts) < 5:
            continue
        pts.sort()
        t0, bx0, by0, mx0, my0, c0 = pts[0]
        t1, bx1, by1, mx1, my1, c1 = pts[-1]
        # base 位移 vs map 位移
        base_disp = math.hypot(bx1 - bx0, by1 - by0)
        map_disp = math.hypot(mx1 - mx0, my1 - my0)
        # static: map 稳定（map_disp 小）但 base 运动（base_disp 大）
        # cargo: base 稳定（base_disp 小）但 map 运动（map_disp 大）
        if base_disp > 0.5 and map_disp < 0.3:
            static_like += 1
        elif base_disp < 0.3 and map_disp > 0.5:
            cargo_like += 1
    print(f"  static_like(map稳定+base运动): {static_like}")
    print(f"  cargo_like(base稳定+map运动): {cargo_like}")
    print(f"  （注意：fragment_id 是帧局部，map 簇是空间近似，不是精确 physical object）")


if __name__ == "__main__":
    main()
