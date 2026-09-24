# 项目状态

当前主线状态以本文件所在提交为准。

```text
FIELD_BASELINE_TAG = baseline-field-legacy-20260923
FIELD_BASELINE_SHA = 18619e7c160eb462ac42c97ad9a03a922a6b8b86
```

## 质量状态

| 范围 | 状态 |
|---|---|
| Clean build | PASS |
| Full GTest | 1000 / 0 |
| Python | 107 / 0 |

## 功能状态

| 能力 | 状态 |
|---|---|
| Cargo V6 | FUNCTIONALLY_FROZEN |
| Avoidance V4 | FUNCTIONALLY_FROZEN |
| Semantic Map Identity | READY |
| Yaw Production Mode | LEGACY |
| Rail Authority | NOT_ENABLED |
| Production Map Changed | NO |
| Safety Thresholds Changed | NO |

## 生产 Yaw 模式

```text
Production Yaw Mode       = LEGACY
Production Yaw Reference  = verified=false
Production Rail Authority = DISABLED
```

Rail Yaw authority 已具备软件身份基础（path-independent `map_frame_uuid`、
`sensor_rig_calibration_id`、yaw-reference semantic hash、map-frame lifecycle
contract），但当前不作为生产运行模式。现场 rail yaw 正式测量与 commissioning
完成前，不得设置 `verified=true`，不得切换 `SHADOW` / `RAIL_AUTHORITY`。

## 剩余 Release Blocker

```text
- REAL_17_18_POSITIVE_COVERAGE（真实 Bag Code 17/18 正向覆盖尚未获得）
- FIELD_LONG_TERM_ACCEPTANCE（现场长稳验收尚未执行）
- PRODUCTION_RAIL_YAW_COMMISSIONING（仅在后续决定启用 RAIL 时）
```

当前 baseline 尚未完成真实 17/18 正向覆盖，不得将历史现场证据（`8d7d7ee`）
当作当前 SHA 的 17/18 覆盖证明。
