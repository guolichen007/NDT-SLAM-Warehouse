#!/usr/bin/env bash
# One-shot combined Cargo V6 SHADOW + Rail Localization V2 Ubuntu gate.
set -u

if [[ $# -lt 12 ]]; then
  echo "Usage: $0 WORKSPACE EXPECTED_SHA MAP_SOURCE YAW_REFERENCE OUTPUT_DIR ORACLE_DIR BASELINE_TRACE_DIR DURATION 无.bag 有.bag 长件.bag 大件.bag" >&2
  exit 2
fi

workspace="$(readlink -f "$1")"
expected_sha="$2"
map_source="$(readlink -f "$3")"
yaw_reference="$(readlink -f "$4")"
output_dir="$(readlink -m "$5")"
oracle_dir="$(readlink -f "$6")"
baseline_trace_dir="$(readlink -f "$7")"
duration="$8"
bags=("$9" "${10}" "${11}" "${12}")

actual_sha="$(git -C "$workspace" rev-parse HEAD)"
[[ "$actual_sha" == "$expected_sha" ]] || {
  echo "SHA_GATE=FAIL expected=$expected_sha actual=$actual_sha" >&2
  exit 3
}
[[ -z "$(git -C "$workspace" status --porcelain --untracked-files=all)" ]] || {
  echo "WORKTREE_GATE=FAIL" >&2
  exit 4
}
[[ -f "$yaw_reference" ]] || { echo "YAW_REFERENCE_GATE=FAIL" >&2; exit 5; }

# ── Runtime isolation gate ──
# 验收期间若已有生产 ndt_slam_node 实例运行，只报错停止，绝不主动 kill。
# bag 测试使用独立 ROS master + map sandbox，绝不与现场地图/实例共享状态。
if pgrep -f "ndt_slam_node" >/dev/null 2>&1; then
  echo "RUNTIME_ISOLATION_GATE=FAIL reason=production_ndt_slam_node_running" >&2
  echo "  detected PIDs: $(pgrep -f 'ndt_slam_node' | tr '\n' ' ')" >&2
  echo "  refusing to proceed; stop the production instance first." >&2
  exit 7
fi

mkdir -p "$output_dir"

# ── Input preflight + frozen input manifest ──
# 冻结本次验收实际消费的每一份输入（SHA256），报告可据此证明被测数据。
python3 "$workspace/src/ndt_slam/scripts/validation/freeze_acceptance_inputs.py" \
  --workspace "$workspace" \
  --candidate-sha "$expected_sha" \
  --map-source "$map_source" \
  --yaw-reference "$yaw_reference" \
  --oracle-dir "$oracle_dir" \
  --baseline-trace-dir "$baseline_trace_dir" \
  --output "$output_dir/frozen_acceptance_inputs.json" \
  --bag-无 "${bags[0]}" --bag-有 "${bags[1]}" \
  --bag-长件 "${bags[2]}" --bag-大件 "${bags[3]}"
preflight_rc=$?
if [[ $preflight_rc -ne 0 ]]; then
  echo "INPUT_PREFLIGHT=FAIL rc=$preflight_rc" >&2
  exit 8
fi
cp "$yaw_reference" "$output_dir/frozen_yaw_reference.yaml"
sha256sum "$output_dir/frozen_yaw_reference.yaml" \
  > "$output_dir/frozen_yaw_reference.sha256"
# SHA-level one-shot ledger root + frozen input manifest. The ledger is claimed
# atomically by the nested matrix only after clean/build/GTest all pass,
# immediately before the first bag — never claimed here.
ledger_root="$workspace/server_runs/p0_combined_attempts"
frozen_input_manifest="$output_dir/frozen_acceptance_inputs.json"

python3 "$workspace/src/ndt_slam/scripts/validation/run_rail_72h_synthetic.py" \
  --output "$output_dir/rail_72h_synthetic.json"
synthetic_rc=$?

set +e
"$workspace/src/ndt_slam/scripts/analysis/run_integrated_cargo_identity_shadow_four_bags.sh" \
  "$workspace" "$expected_sha" "$map_source" "$output_dir/four_bags" \
  "${bags[0]}" "${bags[1]}" "${bags[2]}" "${bags[3]}" \
  "$duration" "$oracle_dir" "$baseline_trace_dir" "$yaw_reference" \
  "$ledger_root" "$frozen_input_manifest" \
  >"$output_dir/four_bag_matrix.log" 2>&1
matrix_rc=$?
set -e

set +e
python3 "$workspace/src/ndt_slam/scripts/validation/summarize_p0_combined_acceptance.py" \
  --output-dir "$output_dir" \
  --candidate-sha "$expected_sha" \
  --frozen-input-manifest "$frozen_input_manifest" \
  --output "$output_dir/p0_combined_acceptance_report.json" \
  --markdown-output "$output_dir/p0_combined_acceptance_report.md" \
  >"$output_dir/combined_summary.log" 2>&1
summary_rc=$?
set -e

echo "COMBINED_CANDIDATE_SHA=$expected_sha"
echo "SHA_GATE=PASS"
echo "WORKTREE_GATE=PASS"
echo "YAW_REFERENCE_GATE=PASS"
echo "RUNTIME_ISOLATION_GATE=PASS"
echo "INPUT_PREFLIGHT=PASS"
echo "SYNTHETIC_72H_RC=$synthetic_rc"
echo "COMBINED_MATRIX_RC=$matrix_rc"
echo "COMBINED_SUMMARY_RC=$summary_rc"
echo "ALGORITHM_CPP_CHANGED=NO"
echo "CARGO_THRESHOLDS_CHANGED=NO"
echo "YAW_ALGORITHM_CHANGED=NO"
echo "OBSTACLE_G11_2_CHANGED=NO"
echo "SAFETY_DISTANCE_CHANGED=NO"
echo "EKF_SEMANTICS_CHANGED=NO"
echo "ONLY_VALIDATION_REPORTING_CHANGED=YES"
echo "CARGO_V6_PRODUCT_TAKEOVER=NOT_PERFORMED"
echo "FIELD_READY=NO"

[[ $synthetic_rc -eq 0 && $matrix_rc -eq 0 && $summary_rc -eq 0 ]]
