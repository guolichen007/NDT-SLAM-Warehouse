# Cargo V6 核心四包 baseline + 最终 authority 设计

> 阶段 1 + 4 交付物。BASE_SHA=`b9f66ae`，四包连续重放，无代码改动。
> 数据目录 `/home/ydkj/avoidance_v4_bag_acceptance/b9f66ae_core4_baseline/`。

## 一、B9F66AE_CORE4_BASELINE_MATRIX

| 指标 | none(无) | positive(有) | long(长件) | big(大件) |
|---|---:|---:|---:|---:|
| HIGH_PRESENT | 282 | 0 | 146 | 123 |
| HIGH_SELECTED | 139 | 0 | 76 | 58 |
| LOW_SELECTED_DESPITE_HIGH | 143 | 0 | 70 | 65 |
| SELECTION_SWITCH_COUNT | 144 | 164 | 134 | 123 |
| IDENTITY_UNKNOWN | 1801 | 1449 | 1652 | 1401 |
| IDENTITY_AMBIGUOUS | 16 | 46 | 67 | 23 |
| IDENTITY_VALIDATED | 224 | 147 | **0** | 96 |
| GROUP_LIFT_CONFIRMED | 277 | 180 | **0** | 173 |
| WORLD_STATIC_VETO_COUNT | 0 | 0 | 0 | 0 |
| FORMAL_LOCK_ALLOWED | 0 | 245 | 0 | 0 |
| LOCKED_FRAMES | 0 | 212 | 0 | 0 |
| MAX_LIFT(m) | 0.15 | 0.03 | 0.23 | 1.37 |
| MAX_LIFT_CONFIRM_COUNT | 0 | 0 | 0 | 15 |
| CARGO_VALID | 0 | 0 | 0 | 0 |
| CLEAR14 | 0 | 0 | 0 | 0 |
| WARNING29 | 0 | 44 | 0 | 0 |
| fault30/33(/34) | 202/1258 | 537/228/329 | 64/1076 | 112/992 |

### 决定性观察

1. **CLEAR14=0 全程**（四包 fail-closed），CARGO_VALID=0 全程。
2. **fault33(cargo invalid) 主导**：none 1258 / long 1076 / big 992。
3. **FORMAL_LOCK_ALLOWED=0 在 3/4 包**（none/long/big），仅 positive 有 245；LOCKED 仅 positive 212 帧。
4. **big_1 有 lift（MAX_LIFT=1.37, confirm_count=15）但 STILL formal_lock_allowed=0**——lift 不是唯一阻塞。
5. **long 包 IDENTITY_VALIDATED=0 全程**（长件方向歧义从未验证）。

## 二、churn 根因（每层都在抖动）

### 2.1 身份 ranker 层
- `scoreCargoCandidateIdentity` 全部 candidate-derived（除固定 anchor hook_distance）。
- none 仍有 143/282（51%）high-present 帧选了 low；SELECTION_SWITCH 123~164。
- big_1 `selected_center_x` min=-1.68 / p50=0 / max=1.65（跨度 3.3m）——**winner 在物体间跳变**。

### 2.2 provisional 窗口层（单 winner）
- 单 winner 的 center step 巨大 → `summarizeCargoProvisionalLock` 的 `motion_discontinuous` / `shape_spread_high` → `formal_lock_allowed=false`。
- 这就是 3/4 包 formal_lock_allowed=0 的直接原因。

### 2.3 authority history 层（per-group，但也碎片化）
- big_1 `association_state`: NEW_HISTORY=1014(66%) / MATCHED=483(32%) / AMBIGUOUS=23。
- `new_history_reason`: NO_HISTORY=768(50%) / XY_GATE=89 / EXTENT_GATE=57 / GAP=57 / Z_GATE=43。
- VALIDATED 分布在 **44 个不同 history id**，每个仅 1~7 帧——history 本身也碎片化（churn + lifecycle reset 双重驱动）。

### 2.4 桥架运动层（关键约束）
- `lineage_motion_observability_state`（big_1）: UNKNOWN_FAIL_CLOSED=819 / LOAD_PRESENT_UNOBSERVABLE=614 / IDLE_ZERO_LOAD=87，**EGO_MOTION_OBSERVABLE=0 帧**。
- `lineage_ego_xy_step` p50=0.000 / max=0.289m——**桥架基本静止**。
- **四包均无 trolley/hoist/hook 编码器 topic**（provenance 报告已证）。

## 三、结论：独立机器证据只有荷重计

```
co-motion（桥架运动）判别器不可用：四包桥架基本静止，无 EGO_MOTION_OBSERVABLE 帧。
独立证据 = /gravity 荷重计（EMPTY→LOADED edge），仅 load existence，无 position/motion。
lift 证据 = candidate-derived（z95 delta），但 per-group + 可与 load edge 时间对齐。
```

这与用户阶段 4 原则一致：`MOTION_OBSERVABLE=NO` 时不得因无法证明 co-motion 而误判，保持 PENDING；此时唯一可用的独立锚点 = **load edge**，配合 **per-group lift 证据（起升过程单调性）+ geometry validity**。

## 四、FINAL AUTHORITY CLOSURE 设计（待实现）

目标：把「每帧先猜 Cargo」改成「per-group 稳定 physical owner + 独立 load edge 时间锚点 + fail-closed」。

### 4.1 核心改动：让 authority 的 per-group VALIDATED 稳定，并接入 production lock

1. **稳定 group→history 关联**：修 history 碎片化（NO_HISTORY=50% 因 lifecycle reset；XY_GATE/Z_GATE/EXTENT_GATE 因 churn）。让「起升中 z 单调的 group」跨帧保持同一 history id。
2. **VALIDATED → production lock**：authority 的 VALIDATED physical owner（唯一、非 ambiguous）驱动 `hook_lock_` 进入 LOCKED（或等价的 active_track），替代 churn-prone 的 `summarizeCargoProvisionalLock` formal_lock_allowed 单 winner gate。
3. **Bottom 消费 validated owner fresh geometry**：LOCKED 后 Bottom 用 validated owner 的 raw-cloud top + frozen shape（现状已具备）。
4. **fail-closed**：无 load edge / 多候选 ambiguous / 无 lift excitation → PENDING，不 CLEAR（现状 fault33 已 fail-closed，需保留而非绕过）。

### 4.2 边界（禁止方向，与用户冻结一致）
- 不调 identity ranker 权重（0.60/0.27/0.13 冻结）。
- 不加 co-motion 权重（桥架静止，无 excitation）。
- 不建第二套 tracker / parallel state machine。
- 不用 point count / previous winner / frozen footprint / nearest hook / absolute high Z 做 owner authority。
- 不改 CargoBoxEstimator / obstacle tracker / NDT / EKF / 阈值。

### 4.3 关键风险
- **long 包**（长件方向歧义，lift 不区分 X/Y）仍是硬骨头：VALIDATED=0、GROUP_LIFT_CONFIRMED=0。需确认「同一套 rule 在 long 包至少 fail-closed（不误判 wrong static owner），不要求强制 VALIDATED」。
- **history 碎片化**是本 closure 的实际工作量主体，不是「加一个 wiring」。

## 五、下一步

按阶段 5~8 执行：实现 4.1 的 authority 稳定化 + VALIDATED→lock wiring + Bottom 审查 + 测试，然后 final 四包回归。此设计文档为决策锚点，实现时同步更新。
