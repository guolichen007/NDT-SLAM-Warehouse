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

mkdir -p "$output_dir"
cp "$yaw_reference" "$output_dir/frozen_yaw_reference.yaml"
sha256sum "$output_dir/frozen_yaw_reference.yaml" \
  > "$output_dir/frozen_yaw_reference.sha256"
attempt_file="$output_dir/combined_four_bag_attempt.marker"
if [[ -e "$attempt_file" ]]; then
  echo "COMBINED_FOUR_BAG_ATTEMPT_GATE=FAIL reason=already_attempted" >&2
  exit 6
fi
printf '%s\n' "$expected_sha" > "$attempt_file"

python3 "$workspace/src/ndt_slam/scripts/validation/run_rail_72h_synthetic.py" \
  --output "$output_dir/rail_72h_synthetic.json"
synthetic_rc=$?

set +e
"$workspace/src/ndt_slam/scripts/analysis/run_integrated_cargo_identity_shadow_four_bags.sh" \
  "$workspace" "$expected_sha" "$map_source" "$output_dir/four_bags" \
  "${bags[0]}" "${bags[1]}" "${bags[2]}" "${bags[3]}" \
  "$duration" "$oracle_dir" "$baseline_trace_dir" "$yaw_reference" \
  >"$output_dir/four_bag_matrix.log" 2>&1
matrix_rc=$?
set -e

echo "COMBINED_CANDIDATE_SHA=$expected_sha"
echo "SHA_GATE=PASS"
echo "YAW_REFERENCE_GATE=PASS"
echo "SYNTHETIC_72H_RC=$synthetic_rc"
echo "COMBINED_MATRIX_RC=$matrix_rc"
echo "COMBINED_FOUR_BAG_ATTEMPT_COUNT=1"
echo "CARGO_V6_PRODUCT_TAKEOVER=NOT_PERFORMED"
echo "OBSTACLE_G11_2_CHANGED=NO"
echo "FIELD_READY=NO"

[[ $synthetic_rc -eq 0 && $matrix_rc -eq 0 ]]
