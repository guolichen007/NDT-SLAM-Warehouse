#!/usr/bin/env python3
"""D5 fragment 分类 + current-frame recovery signal 分析（离线，只读 CSV）。"""
import csv
import json
import sys
from collections import Counter, defaultdict

FRAG = "/tmp/cargo_forensic/d5_weak_fragments.csv"


def load_fragments(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def fnum(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return float("nan")


def stats(vals):
    vals = sorted(v for v in vals if v == v)  # drop nan
    if not vals:
        return {"n": 0}
    n = len(vals)
    return {
        "n": n,
        "p05": round(vals[int(n * 0.05)], 3),
        "p50": round(vals[n // 2], 3),
        "p95": round(vals[int(n * 0.95)], 3),
    }


def main():
    rows = load_fragments(FRAG)
    print(f"total weak fragments: {len(rows)}")

    high = [r for r in rows if r.get("oracle_high_surface") == "1"]
    low = [r for r in rows if r.get("oracle_high_surface") == "0"]
    print(f"oracle_high_surface=1 (cargo top candidate): {len(high)}")
    print(f"oracle_high_surface=0 (ground/static): {len(low)}")
    print()

    def summarize(name, subset):
        print(f"=== {name} (n={len(subset)}) ===")
        for k in [
            "point_count",
            "nearest_primary_3d_distance",
            "nearest_primary_xy_distance",
            "vertical_separation",
            "xy_projected_overlap",
            "xy_projected_gap",
            "candidate_primary_neighbor_count",
        ]:
            print(f"  {k}: {stats([fnum(r, k) for r in subset])}")
        print()

    summarize("HIGH (oracle_high_surface=1)", high)
    summarize("LOW (oracle_high_surface=0)", low)

    # 关键交叉：oracle_high_surface=1 但 xy_overlap 是否 > 0
    high_overlap = [r for r in high if fnum(r, "xy_projected_overlap") > 0.5]
    high_no_overlap = [r for r in high if fnum(r, "xy_projected_overlap") <= 0.5]
    print(f"HIGH 里 xy_overlap>0.5: {len(high_overlap)} / {len(high)}")
    print(f"HIGH 里 xy_overlap<=0.5: {len(high_no_overlap)} / {len(high)}")
    print()

    # 2.5D 信号：XY overlap 大 + vertical separation 大
    # 反证：LOW（贴地静态）里是否有 XY overlap 也大的（会误判）
    low_overlap = [r for r in low if fnum(r, "xy_projected_overlap") > 0.5]
    print(f"LOW 里 xy_overlap>0.5（潜在误判）: {len(low_overlap)} / {len(low)}")
    if low_overlap:
        print("  LOW 高 overlap 样本的 vertical_separation 分布:",
              stats([fnum(r, "vertical_separation") for r in low_overlap]))
    print()

    # 按 stamp 分桶统计 high fragment 的时间连续性（配合起升轨迹）
    high_stamps = sorted(set(r["stamp"][:10] for r in high))
    print(f"HIGH fragment 出现的时间段（前20）: {high_stamps[:20]}")

    # 输出 F1-F4 初判
    print()
    print("=== F1-F4 初判依据 ===")
    print("F1 TRUE_CARGO_SURFACE: oracle_high_surface=1 且 xy_overlap 大、vertical_sep 大")
    print("F2/F3 独立静态/障碍: oracle_high_surface=0 且贴地、xy_overlap 小")
    print()


if __name__ == "__main__":
    main()
