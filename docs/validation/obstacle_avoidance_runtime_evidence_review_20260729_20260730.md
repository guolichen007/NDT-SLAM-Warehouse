# 避障运行证据审查

审查日期：2026-07-30

证据状态：`CANDIDATE_EVIDENCE_UNBOUND`

## 审查范围

本记录提炼两份仓库外资料，不提交原始 CSV、运行日志或自动生成报告：

- SLAM 长时间运行：`server_runs/manual-20260729-175035/`，约 15.9 小时。
- 主控 S3 Case 资料：2026-07-28 的独立主控运行记录。

资料撰写时引用当前分支 `f57d68a`，但 SLAM 事件仍出现旧原因串
`pending_positive_warning_authorized`。`f57d68a` 已将其拆为
`pending_live_warning_authorized`、`pending_static_warning_authorized` 和双源组合原因。
在缺少可复核运行清单完整 SHA 的情况下，本批数据不能绑定 `f57d68a`，因此标为
`UNBOUND`。

## 数据直接支持的结论

| 指标 | 观测值 | 结论边界 |
|---|---:|---|
| `safety_events.csv` 记录 | 4,110 | 证明监控链路持续产生日志 |
| ROS 事件记录 | 1,101 | 证明 ROS 侧事件采集持续工作 |
| 管线 CSV 记录 | 57,588 | 证明约 15.9 小时运行期间有持续诊断 |
| `SAFETY_WARN` Code 18 | 254 | 历史运行链路产生 3-5m 正向告警 |
| `SAFETY_WARN` Code 17 | 38 | 历史运行链路产生 3m 内正向告警 |
| Code 18 距离范围 | 3.00-4.96m | 与 3-5m 协议区间一致 |
| Code 17 距离范围 | 1.67-2.98m | 与 3m 内协议区间一致 |
| CLEAR 14 | 未观测 | 只能说明样本中没有记录，不能推断 CLEAR 路径失效 |

254 次 Code 18 与 38 次 Code 17 合计 292 条正向告警记录。它们证明历史链路能持续
输出 17/18，但没有独立真值标注，因此不能据此宣称零误报或零漏报。

## Provenance 纠偏

原报告把数值 1-4 重命名为 `LIVE/STATIC/BOTH/DUAL_CONFIRMED`，这与仓库
`CargoSafetyStatus.msg` 的正式枚举不一致。按当前 Schema，数字含义为：

| 数值 | 正式枚举 | 原报告计数 |
|---:|---|---:|
| 1 | `OUTSIDE_CARGO_SHELL_ONLY` | 468 |
| 2 | `PRE_CARGO_OCCUPANCY` | 60 |
| 3 | `STATIC_MAP_MATCH` | 41 |
| 4 | `CARGO_MOVED_AWAY_PERSISTENCE` | 85 |
| 5 | `DUAL_LIDAR_CONSENSUS` | 未给出 |

这些计数可作为来源分布统计，但不能用于证明 `f57d68a` 新增的 Pending 静态独立告警
路径。尤其是数值 2 表示“吊物到达前已存在”，不等同于静态独立告警授权。

## S3 链路边界

SLAM 数据和主控 S3 数据来自不同场次。现有材料支持“SLAM 侧出现 Code 18”和
“主控侧收到 Code 18 后可触发 S3”两个分段结论，但不支持：

- 将两个场次的事件计数做 1:1 对应；
- 计算同一事件从 SLAM 到 S3 的端到端延迟；
- 验证 Code 17→S3；
- 验证总闸关闭时的 S3 独立闸门路径。

## `f57d68a` 补充验收条件

服务器验收应固定 expected SHA，并至少覆盖：

1. `run_manifest.json` 记录完整 SHA，运行中无工作区修改。
2. 实时障碍授权分别出现 `pending_live_warning_authorized`。
3. 在实时障碍不可用、静态来源授权有效时出现
   `pending_static_warning_authorized`，并记录稳定 region confirmations。
4. 生命周期、map generation、观测间隔或匹配区域变化会重置静态身份。
5. Pending 静态路径只能输出 17/18，不能输出 14。
6. NDT fitness 熔断、恢复及 MapCommit 禁止行为通过 Bag 验收。
7. 重定位结果的重复、过期、身份变化和不一致修正被拒绝。
8. SLAM 与主控在同一场次使用同步时间源采集 Code 17/18→S3 证据。

## 审查结论

本批资料是有价值的历史长时间运行证据，补充了 17/18 分布和链路持续性；它不是
`f57d68a` 的现场验收证据，也不替换 `validation-obstacle-avoidance-20260728` 正式
验证 Tag。当前实现只有在完成上述同 SHA 验收后，才能前移现场验证基线。
