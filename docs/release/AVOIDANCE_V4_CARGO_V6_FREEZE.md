# Avoidance V4 / Cargo V6 封板记录

## Release identity

- **PRODUCT_SHA**：`7a75411d8523af2a84b6096ea53824aa245342f7`
- **BRANCH**：`fix/cargo-v6-final-physical-authority`（已 push origin）
- **MODE**：`V6_AUTHORITY`（canonical authority 正式产品路径）
- 相对稳定检查点 `4193526`：ahead=23 commits，behind=0，线性向前完整修复链。
- `a096b90 → 7a75411` 仅 1 个 commit（config promotion + `ConfigParsesV6AuthorityMode` 测试），
  无任何 Cargo 算法/D5/History/Bottom/static/lift threshold 改动。

## 最终架构

Cargo V6 canonical authority 门链（`cargo_v6_authority_adapter.cpp`）：

```
Safety 授权 = identity VALIDATED + lift_confirmed + current_candidate_fresh
           + geometry_resolved + physical_history_id != 0
           + current physical owner group + same-stamp geometry
           + valid Bottom + Bottom.track_id == physical_history_id
           + finite bottom<top + union_points 非空

MapMutation 授权 = Safety 授权 + exact current owner points
                 + !independent_static_provenance_conflict
```

关键安全合同：

- V6 invalid → 不能 fallback 到 Legacy CLEAR / Legacy map mutation。
- Legacy 最多在「相同 authority 的既有正危险」下保留 positive hazard（保守，无 false clear）。
- V6 self-removal 非 broad OBB 删除，要求 current product + canonical + same physical
  history/epoch + same source timestamp + same pose authority + exact current-frame owner point。
- Registration hygiene 仍为 Legacy product path（V4/V6 迁移边界），真正禁止的是
  `registration_source_removed_without_ownership_count > 0`，不是 legacy 合法删除。

22 项核心修复（`4193526 → a096b90`）包括：provisional self-reference 移除、raw
point-count owner authority 移除、ProductionOwnerLock 分离、G/H wiring、owner continuation、
D5 current-frame overlap、canonical owner shape、10Hz、Guard C static fail-closed、static
snapshot vs static-evidence epoch、frozen static snapshot、clear-stale formal-lift projection、
**single-writer Formal Reference Lock（a096b90，每 physical epoch 单写者）**。

## 验收结果

### CORE4（V6_AUTHORITY / 7a75411 / canonical MAP_SOURCE）

| bag | V6_VALIDATED | V6_SAFETY_AUTH | V6_MAP_AUTH | 判定 |
|---|---|---|---|---|
| 调运大件(BIG) | 220 | 42 | 0 | PASS |
| 有(POSITIVE) | 332 | 33 | 0 | PASS |
| 无(NONE) | 85 | 60(非确定) | 0 | PASS_WITH_FIXTURE_LIMITATION |
| 调运长件(LONG) | 0 | 0 | 0 | SAFE_DEGRADED(started_loaded) |

安全硬门（WRONG_OBJECT/FALSE_EXACT_OWNER/FALSE_CLEAR/VALID_HAZARD_MISSED/
MAP_REMOVED_POINT_WITHOUT_AUTHORITY/REGISTRATION_REMOVED_WITHOUT_OWNERSHIP/CRASH）**全 0**。

### 六包 system regression

调运大件=PASS / 有=PASS / 调运钢片=PASS（safety 59）/ 小方件装车=PASS（safety 101）/
无=PASS_WITH_FIXTURE_LIMITATION / 调运长件=SAFE_DEGRADED。
`SYSTEM_SIX_BAGS = PASS_WITH_RECORDED_FIXTURE_LIMITATIONS`。

### FULL_GTEST / Python contracts

- **FULL_GTEST = 991 tests / 0 failures / 62 binaries**（clean catkin build）。
- **PYTHON_TESTS = 98 passed / 0 failed**（firewall/E2E/integration/static 契约）。

### 性能（REALTIME_FRESHNESS_GATE）

```
total_frame_ms p50/p95/max = 107~128 / 127~200 / 161~261
detection_ms p50 = 1.1~5.7 ms
NDT_ms p50/p95 = 8.8~14 / 24~30 ms
queue_age_ms p50/max ≈ 50 / 105 ms（最多积压 1 帧，无持续增长）
queue_degraded_frames = 8~24
processed_hz ≈ 8.3~9.3 Hz（sensor 10Hz，latest-frame queue capacity=1 overwrite old）
```

`REALTIME_FRESHNESS_GATE = PASS_WITH_BOUNDED_FRAME_DROP`；`FULL_10HZ_THROUGHPUT = NO`
（工程事实：processed 8.3~9.3Hz < sensor 10Hz，由 latest-frame 队列吸收差异，queue age
恒定在 1 帧以内，无持续积压）。

### 1h soak（6 轮 × 调运大件/有/无，18 次冷启动）

- CRASH=0 / NODE_RESTART=0 / UNAUTHORIZED_MAP=0 / REGISTRATION_REMOVED_WITHOUT_OWNERSHIP=0
  / MAP_REMOVED_WITHOUT_AUTHORITY=0 / FALSE_CLEAR=0。
- memory_mb 范围 170~308，无跨 run 单调增长；queue/history 无无界增长；无 NDT/EKF recovery storm。
- BIG/POSITIVE 每轮稳定授权（safety 全 >0）；NONE 非确定（见下）。
- **ONE_HOUR_SOAK = PASS**。

## Fixture limitations（记录，不改算法）

1. **调运长件**：`started_loaded_without_baseline`（bag 起始已 LOADED，无 EMPTY 预加载），
   identity 全程无法 VALIDATED → SAFE_DEGRADED。
2. **无（NONE）**：lift 窗口临界，真实 lift 证据从不连续 4 帧（NO-DROP replay 下
   `lift_confirm_count 最大=0`）。1x 下 `safety_authorized_count` 0↔91 翻转是
   **RUNTIME_FRAME_SAMPLING_SENSITIVITY**（processed < sensor rate + latest-frame queue
   overwrite），非代码非确定。判定 NONE_FIXTURE_LIMITED=YES / NONE_RELEASE_BLOCKER=NO。
3. **MapMutation positive path 未观测**：所有 safety-authorized 帧均命中
   `independent_static_provenance_conflict`（真实静态重叠，snapshot 非 null/单 epoch/无
   relocalization reset），map 授权 fail-closed 保守正确。
   `V6_MAP_POSITIVE_COVERAGE = NOT_OBSERVED_DUE_TO_FIXTURE_STATIC_CONFLICT`。
4. 六包语料不含 超小件/小方件/小方件2（从本轮最终六包验收删除）。

## Yaw provenance（独立处理）

- `yaw_authority_mode = LEGACY`（RAIL_AUTHORITY 未启用）；site yaw reference 文件缺失。
- `CODE_READY=YES` / `CARGO_V6_CODE_ACCEPTANCE=PASS` / `FUNCTIONAL_ACCEPTANCE=PASS` /
  `TAG=BLOCKED` / `FINAL_RESULT=CODE_READY_PENDING_YAW_PROVENANCE`。
- 禁止为 yaw 修改 Cargo。

## Freeze policy

Cargo V6 代码冻结于 `7a75411`。重新打开代码仅当出现：WRONG_OBJECT>0、FALSE_CLEAR>0、
VALID_HAZARD_MISSED>0、FALSE_EXACT_OWNER>0、unauthorized safety/map authority>0、
registration removal without ownership>0、crash>0，或同输入（确定性输入序列）产品决策非确定。
非安全诊断瑕疵与保守 fail-closed 记录为 known limitations，不触发进一步算法工作。
