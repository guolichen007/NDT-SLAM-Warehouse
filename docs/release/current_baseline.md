# 当前版本基线

## 身份

```text
FIELD_BASELINE_TAG = baseline-field-legacy-20260923
FIELD_BASELINE_SHA = 18619e7c160eb462ac42c97ad9a03a922a6b8b86
```

## 生产模式

```text
Yaw Authority       = LEGACY
Rail Authority      = DISABLED
Yaw Reference       = verified=false
```

## 子系统状态

| 子系统 | 状态 |
|---|---|
| 定位（NDT/EKF） | PASS（LEGACY） |
| Cargo V6 | FROZEN |
| Avoidance V4 | FROZEN |
| Semantic Map Identity | READY |

## 质量

```text
Clean build  = PASS
Full GTest   = 1000 / 0
Python       = 107 / 0
```

## 冻结边界（不可改）

- Cargo V6 算法与安全阈值
- Avoidance V4 算法与安全阈值（0.8m clearance、3m/5m、far-history）
- 生产地图未替换

## 剩余 Release Gate

```text
- REAL_17_18_POSITIVE_COVERAGE（真实 Bag Code 17/18 正向覆盖尚未获得）
- FIELD_LONG_TERM_ACCEPTANCE（现场长稳验收尚未执行）
- PRODUCTION_RAIL_YAW_COMMISSIONING（仅后续决定启用 RAIL 时）
```

## 回滚

历史现场正向 17/18 证据见 Tag `validation-obstacle-avoidance-20260728`
（`8d7d7ee`），但该历史证据不得作为当前 SHA 的 17/18 覆盖证明。
