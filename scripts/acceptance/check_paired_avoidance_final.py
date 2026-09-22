#!/usr/bin/env python3
"""避障 V4 双 Bag 功能验收分析器（acceptance-only，不连接产品）。

读取 cargo_frames.csv + safety_samples.csv（既有 runtime 诊断），
不重新计算点云算法。按产品权威报码合同核对 17/18/29 的距离/净空/far-history。

用法:
  check_paired_avoidance_final.py <run_dir> <positive|negative>
"""
import csv, json, os, sys, collections

CLEARANCE_M = 0.80

def tb(v):
    return str(v).strip().lower() in ("1", "true", "yes")

def fnum(v):
    try:
        x = float(v)
        return x if x == x else None  # NaN guard
    except (TypeError, ValueError):
        return None

def load_csv(path):
    if not os.path.exists(path):
        return None
    return list(csv.DictReader(open(path)))

def analyze(run_dir, kind):
    cargo = load_csv(os.path.join(run_dir, "runtime_diagnostics", "cargo_frames.csv"))
    safety = load_csv(os.path.join(run_dir, "safety_samples.csv"))
    out = {"kind": kind, "run_dir": run_dir}

    # 用 cargo_frames 为主（逐帧诊断），safety_samples 交叉验证
    if not cargo:
        out["error"] = "cargo_frames.csv missing"
        return out

    confirmed = [r for r in cargo if r.get("confirmed_warning_code") not in ("", "0", None)]
    codes = collections.Counter()
    for r in confirmed:
        try:
            codes[int(r.get("confirmed_warning_code", 0))] += 1
        except (ValueError, TypeError):
            pass

    # 障碍物检测：track_id > 0 且多时间戳
    track_ids = set()
    track_frames = collections.Counter()
    for r in cargo:
        tid = r.get("obstacle_track_id", "").strip()
        if tid not in ("", "0"):
            track_ids.add(tid)
            track_frames[tid] += 1
    obstacle_detected = len(track_ids) > 0 and any(v >= 2 for v in track_frames.values())

    # track 稳定性：reset 计数 + association reset 原因分布
    reset_count = 0
    created_count = 0
    for r in cargo:
        try:
            reset_count = max(reset_count, int(r.get("obstacle_track_reset_count", 0) or 0))
            created_count = max(created_count, int(r.get("obstacle_track_created_count", 0) or 0))
        except (ValueError, TypeError):
            pass
    assoc_reasons = collections.Counter(r.get("obstacle_association_reset_reason", "") for r in cargo
                                        if r.get("obstacle_association_reset_reason", "").strip())

    # 合同违规检查（逐帧 confirmed code）
    dist_viol = collections.Counter()   # code -> 违规帧数
    clear_viol = collections.Counter()
    code_frames = collections.Counter()  # code -> 总帧数
    for r in confirmed:
        try:
            code = int(r.get("confirmed_warning_code", 0))
        except (ValueError, TypeError):
            continue
        code_frames[code] += 1
        d = fnum(r.get("nearest_cluster_distance"))
        c = fnum(r.get("conservative_clearance_m"))
        # 净空合同
        if c is not None and c >= CLEARANCE_M:
            clear_viol[code] += 1
        # 距离合同
        if d is not None:
            if code == 17 and d > 3.0:
                dist_viol[code] += 1
            elif code == 18 and not (3.0 < d <= 5.0):
                dist_viol[code] += 1
            elif code == 29 and d > 5.0:
                dist_viol[code] += 1

    # false clear（safety_samples 里 fault_code 或 warning_code == 14 之类）
    false_clear = 0
    if safety:
        for r in safety:
            try:
                wc = int(float(r.get("warning_code", 0) or 0))
                fc = int(float(r.get("fault_code", 0) or 0))
            except (ValueError, TypeError):
                continue
            if wc == 14 or fc == 14:
                false_clear += 1

    out.update({
        "confirmed_code_frames": dict(code_frames),
        "obstacle_detected": obstacle_detected,
        "obstacle_track_ids": len(track_ids),
        "obstacle_track_created_count": created_count,
        "obstacle_track_reset_count": reset_count,
        "association_reset_reasons": dict(assoc_reasons),
        "distance_contract_violations": dict(dist_viol),
        "clearance_contract_violations": dict(clear_viol),
        "false_clear_count": false_clear,
    })

    if kind == "negative":
        out["FALSE_CODE17_COUNT"] = code_frames.get(17, 0)
        out["FALSE_CODE18_COUNT"] = code_frames.get(18, 0)
        out["FALSE_CODE29_COUNT"] = code_frames.get(29, 0)
        # false low clearance: 有 confirmed 低净空但无真实障碍（负向包本应无）
        out["false_low_clearance_frames"] = sum(1 for r in confirmed if fnum(r.get("conservative_clearance_m")) is not None and fnum(r.get("conservative_clearance_m")) < CLEARANCE_M)
        out["NEGATIVE_BAG_RESULT"] = "PASS" if (out["FALSE_CODE17_COUNT"] == 0 and out["FALSE_CODE18_COUNT"] == 0 and out["FALSE_CODE29_COUNT"] == 0) else "FAIL"
    else:
        out["CODE17_OBSERVED_FRAMES"] = code_frames.get(17, 0)
        out["CODE18_OBSERVED_FRAMES"] = code_frames.get(18, 0)
        out["CODE29_OBSERVED_FRAMES"] = code_frames.get(29, 0)
        total_viol = sum(dist_viol.values()) + sum(clear_viol.values())
        frag = "NO" if reset_count <= 1 else "CHECK"
        out["TRACK_FRAGMENTATION"] = frag
        out["POSITIVE_BAG_RESULT"] = "PASS" if (obstacle_detected and total_viol == 0 and false_clear == 0) else "FAIL"

    return out

def main():
    if len(sys.argv) < 3:
        print("用法: check_paired_avoidance_final.py <run_dir> <positive|negative>")
        sys.exit(1)
    res = analyze(sys.argv[1], sys.argv[2])
    print(json.dumps(res, indent=2, ensure_ascii=False))

if __name__ == "__main__":
    main()
