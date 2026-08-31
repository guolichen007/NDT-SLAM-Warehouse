#!/usr/bin/env python3
"""Phase 0D support lineage 分析：S3->component conservation + membership churn + gate root。"""
import csv
from collections import Counter, defaultdict

VOX = "/tmp/cargo_forensic/d5_voxel_lineage.csv"
COMP = "/tmp/cargo_forensic/phase0d_component_lineage.csv"
CAND = "/tmp/cargo_forensic/phase0d_candidate_lineage.csv"
GROUPS = "/tmp/cargo_forensic/integrated_identity_groups.csv"


def fnum(r, k):
    try:
        return float(r[k])
    except (KeyError, ValueError):
        return float("nan")


def main():
    # 1. S3 high-oracle (z>=1.3) voxel conservation
    print("=== 1. S3 high-oracle voxel (z>=1.3) conservation ===")
    assign = Counter()
    for r in csv.DictReader(open(VOX)):
        if fnum(r, "z") >= 1.3:
            assign[int(r["assignment"])] += 1
    total = sum(assign.values())
    print(f"  total high-oracle voxels: {total}")
    print(f"  PRIMARY_ACCEPTED(0): {assign[0]} ({assign[0]/max(1,total):.1%})")
    print(f"  WEAK_REJECTED(1):    {assign[1]} ({assign[1]/max(1,total):.1%})")
    print(f"  UNASSIGNED(2):       {assign[2]} ({assign[2]/max(1,total):.1%})")
    print()

    # 2. component high-oracle support 与 selected 的关系
    print("=== 2. component high-oracle support ===")
    comp_by_stamp = defaultdict(list)
    for r in csv.DictReader(open(COMP)):
        comp_by_stamp[float(r["stamp"])].append(r)
    # selected candidate 的 member_component_ids
    cand_selected = {}
    for r in csv.DictReader(open(CAND)):
        if r["selected"] == "1":
            cand_selected[float(r["stamp"])] = r["member_component_ids"]
    # 每帧：high_oracle 落在 selected vs other
    sel_high = other_high = 0
    n_frames = 0
    for t, comps in comp_by_stamp.items():
        if t not in cand_selected:
            continue
        n_frames += 1
        sel_members = set(cand_selected[t].split("|"))
        for c in comps:
            h = int(c["high_oracle_count"])
            if h == 0:
                continue
            if c["component_id"] in sel_members:
                sel_high += h
            else:
                other_high += h
    print(f"  frames: {n_frames}")
    print(f"  high-oracle voxels in SELECTED component: {sel_high}")
    print(f"  high-oracle voxels in OTHER component:    {other_high}")
    if sel_high + other_high > 0:
        print(f"  selected 占比: {sel_high/(sel_high+other_high):.1%}")
    print()

    # 3. selected candidate member_component_ids 跨帧 churn（R1 vs R2）
    print("=== 3. selected candidate member set churn (R1 vs R2) ===")
    ts = sorted(cand_selected)
    changes = 0
    member_sets = {}
    for i in range(len(ts)):
        t = ts[i]
        member_sets[t] = frozenset(cand_selected[t].split("|"))
    for i in range(1, len(ts)):
        if member_sets[ts[i]] != member_sets[ts[i-1]]:
            changes += 1
    print(f"  selected frames: {len(ts)}, member-set changes: {changes} "
          f"({changes/max(1,len(ts)-1):.1%} of transitions)")
    print()

    # 4. gate reject reason 分布（integrated_identity_groups）
    print("=== 4. association reject reason 分布 ===")
    reject = Counter()
    for r in csv.DictReader(open(GROUPS)):
        reject[r.get("association_reject_reason", "?")] += 1
    for k, v in reject.most_common(15):
        print(f"  {k}: {v}")
    print()


if __name__ == "__main__":
    main()
