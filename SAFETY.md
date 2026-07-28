# 运行安全

本项目实现了面向仓库天车作业的类型化吊物安全合同。本软件不是安全认证设备。

## 安全码

| Code | 含义 | 授权来源 |
|---:|---|---|
| 14 | CLEAR — 无碰撞风险 | 必须 Formal Geometry + 全部合同满足 |
| 17 | NEAR_3M — ≤3m，净空<0.8m | Formal 或 Degraded Geometry |
| 18 | NEAR_5M — 3-5m，净空<0.8m | Formal 或 Degraded Geometry |
| 30 | 系统未就绪 / 时间轴回退 | 故障 |
| 31 | 定位无效 | 故障 |
| 32 | Gravity / 称重无效 | 故障 |
| 33 | 吊物证据无效 | 故障 |
| 34 | 障碍证据不足 | 故障 |
| 35 | 内部合同错误 | 故障 |

## 关键安全属性

### 14 是明确的 CLEAR

Code 14 表示系统已正向确定无碰撞风险。要求：
- Formal（已授权）吊物几何
- 有效障碍观测且未检测到危险
- 全部安全合同满足

Degraded Geometry 不能产生 Code 14。

### 17 / 18 是正向碰撞风险

Code 17 和 18 表示已检测到真实空间碰撞风险。要求：
- 有效障碍追踪，位于报告距离
- 垂直净空低于 0.8m
- 连续验证观测

### 30-35 不是 CLEAR

Code 30-35 代表系统故障或证据问题。不得将其解释为"安全"或"无风险"。如果系统不能正向确定安全，则输出故障码。

### 降级几何：可告警不可 CLEAR

未获得 Formal 授权的 live-only（Degraded）几何：
- 可以产生正向 17 / 18 告警
- 不能产生 CLEAR 14
- 不能从 registration 剔除货物点
- 不能排除静态地图中的货物区域
- 不能授权 MapCommit 排除

### Display Marker 不携带安全权威

RViz marker（cargo_core_bbox_marker、cargo_tight_box_marker、cargo_warning_zone_marker）是可视化辅助工具，不携带安全权威。

### 历史 / 退役证据

过期证据（LOST_HOLD 超时、退役静态快照、旧 Pending Envelope）不能产生 CLEAR 或授权地图变更。

## 部署安全

部署时必须保留：
- 外部急停回路
- 物理限位开关
- 现场安全策略和流程
- 独立运行监督

类型化安全合同（CargoSafetyStatus schema v6）是正式安全输出。下游控制器必须：
- 将 Code 30-35 视为非 CLEAR
- 不能因缺少 17/18 推断为 CLEAR
- 对安全状态流实现独立超时/Watchdog
- 保持独立安全逻辑

## 安全行为变更要求

任何修改以下内容的 PR 必须：
- 安全码（14、17、18、30-35）
- 距离阈值（3m、5m）
- 净空阈值（0.8m）
- 消息 Schema
- 几何授权规则
- 来源验证 fail-safe

必须：
- 显式声明安全合同影响
- 提供变更前后 SHA
- 通过静态合同、clean build、gtest、bag 和现场验证
- 不能仅依赖 Windows 检查

## 现场验证基线

| 证据 | 日期 | 内容 |
|---|---|---|
| SLAM 侧 | 2026-07-27 | Code 17/18 正向输出，24 独立避障片段 |
| 主控侧 | 2026-07-28 | Code 18 接收→S3 发送 |

两次现场运行为不同日期独立运行，不是同步逐帧一一对应。S3 独立闸门路径（总闸关闭场景）未被此批 case 覆盖。

验证基线 SHA：`8d7d7eed0548321bf0646232f374fe95a29990dd`
验证 Tag：`validation-obstacle-avoidance-20260728`

如怀疑安全行为退化，回滚到该 SHA。
