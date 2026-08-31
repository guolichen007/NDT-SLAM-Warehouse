#!/usr/bin/env python3
"""D5 2.5D current-frame signal 精确验证 + F1-F4 分类。"""
import csv
import json
from collections import defaultdict

FRAG = "/tmp/cargo_forensic/d5_weak_fragments.csv"
GROUPS = "/tmp/cargo_forensic/integrated_identity_groups.csv"


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def fnum(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return float("nan")


def main():
    frags = load(FRAG)
    high = [r for r in frags if r["oracle_high_surface"] == "1"]
    low = [r for r in frags if r["oracle_high_surface"] == "0"]

    # 候选恢复信号：完全 XY 重叠 + vertical separation 大于 cargo 厚度阈值
    # 不用绝对 z，只测试「XY overlap 大 + z 相对差」这个 2.5D 特征
    def sig(r):
        return fnum(r, "xy_projected_overlap") > 0.9

    for name, subset in (("HIGH", high), ("LOW", low)):
        s = [r for r in subset if sig(r)]
        vs = [fnum(r, "vertical_separation") for r in s]
        vs = sorted(v for v in vs if v == v)
        n = len(vs)
        print(f"{name}: XY完全重叠(>0.9) = {len(s)}/{len(subset)}")
        if n:
            print(f"  vertical_separation p10={vs[n//10]:.3f} p50={vs[n//2]:.3f} p90={vs[int(n*0.9)]:.3f}")
        print()

    # 联合信号：XY 完全重叠 且 vertical_separation >= 0.45（约 cargo 厚度下限）
    def strong_sig(r):
        return fnum(r, "xy_projected_overlap") > 0.9 and \
            fnum(r, "vertical_separation") >= 0.45

    for name, subset in (("HIGH", high), ("LOW", low)):
        s = [r for r in subset if strong_sig(r)]
        print(f"强信号(XY重叠>0.9 且 vs>=0.45): {name} = {len(s)}/{len(subset)} = {len(s)/max(1,len(subset)):.3f}")
    print()

    # 反证：LOW 里有多少会命中强信号（误判率）
    low_strong = [r for r in low if strong_sig(r)]
    high_strong = [r for r in high if strong_sig(r)]
    print(f"强信号命中：HIGH={len(high_strong)} LOW={len(low_strong)}")
    print(f"  误判率(LOW命中/全部LOW) = {len(low_strong)/max(1,len(low)):.3f}")
    print(f"  召回率(HIGH命中/全部HIGH) = {len(high_strong)/max(1,len(high)):.3f}")
    print()

    # 时间维度：HIGH fragment 的 vertical_separation 随起升时间的变化
    # oracle window: 1778303489.58 ~ 1778303538.98
    by_time = defaultdict(list)
    for r in high:
        t = fnum(r, "stamp")
        by_time[int(t)].append(fnum(r, "vertical_separation"))
    print("=== HIGH fragment vertical_separation 随 stamp 变化（每10s中位数）===")
    for t in sorted(by_time):
        vs = sorted(v for v in by_time[t] if v == v)
        if not vs:
            continue
        print(f"  t={t}: n={len(vs)} vs_p50={vs[len(vs)//2]:.3f}")
    print()

    # LOW 高 overlap 样本的 z（贴地确认）
    low_ov = [r for r in low if fnum(r, "xy_projected_overlap") > 0.9]
    zs = sorted(fnum(r, "center_z") for r in low_ov if fnum(r, "center_z") == fnum(r, "center_z"))
    if zs:
        print(f"LOW 高overlap 样本 center_z: p50={zs[len(zs)//2]:.3f} p90={zs[int(len(zs)*0.9)]:.3f}")
    zs2 = sorted(fnum(r, "center_z") for r in high if fnum(r, "center_z") == fnum(r, "center_z"))
    if zs2:
        print(f"HIGH 样本 center_z: p50={zs2[len(zs2)//2]:.3f} p90={zs2[int(len(zs2)*0.9)]:.3f}")


if __name__ == "__main__":
    main()
