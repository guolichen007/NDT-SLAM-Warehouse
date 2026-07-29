# 变更日志

## [Unreleased] — 现场验证避障基线 RC

### 新增

- 吊物几何融合（CargoGeometryFusion）：正式冻结形状（Formal Geometry）与降级仅实时几何（Degraded Geometry），各自拥有不同的安全授权（正式可 CLEAR，降级仅可告警）。
- Pending Cargo Envelope：新出现障碍在正式确认前的正向告警临时几何。
- 外部障碍追踪器（CargoObstacleTracker）：近场/远场历史、独立静态 provenance、嵌入/分离 track 区分。
- `CargoSwingMonitor`：偏拉检测、摆动振荡追踪、吊钩锚点授权、绳长测量集成。
- `CargoLiftOriginBinder`：起吊原点追踪与冻结厚度正式货物高度。
- `StaticHeightField` 和 `StaticHeightComponentExtractor`：基于地图证据的不确定性感知静态高度。
- `MapSessionSnapshot`：带 Manifest 的一致性地图会话保存/加载。
- `StaticEvidenceAuthorization`：不可变有界查询快照、clean-map provenance 和版本化侧车 Manifest。
- `RevealedSupportObserver`：从直接顶部测量和冻结厚度融合货物底部。
- `CargoPhysicalMotionEstimator`：基于物理速度限制的货物中心更新。
- `CargoPresenceStateMachine`：EMPTY→CANDIDATE→LOCKED→LOST_HOLD 生命周期，含短正式保持超时和故障闭锁 33。
- `CargoSafetyEvaluator`：14/17/18/30-35 类型化安全合同，含正式 CLEAR 授权门控。
- `cargo_alarm_heartbeat_node`：类型化 `CargoSafetyStatus` 合同校验、5Hz 状态重发、重复时间戳拒绝。
- 服务器监控 v2：集成货物遥测和 60/600 秒安全窗口。
- 完整 gtest 套件：货物避障、几何融合、摆动监控、Pending Envelope、障碍追踪、静态证据授权、静态高度场、起吊原点绑定、物理运动估计、存在状态机、地图会话快照、揭示支撑观测。
- Python 测试合同：地图加载事务、服务器监控、安全聚合器、货物几何分类、类型化回调时间验证。

### 变更

- 货物安全现为显式类型化合同，正式授权分离（Formal vs. Degraded Geometry）。
- 障碍追踪在发出 Level 2 告警前需要独立静态 provenance；仅有 near-field track 且 missing far history 不满足条件。
- Pending Envelope 在告警前需要连续稳定证据。
- 静态地图授权使用当前查询覆盖、格网数、高度、来源验证和观测历史。
- 生命周期变更使上一个 Manifest 变为非活跃，保留为最近良好归档，直到成熟当前 epoch 快照被原子提交。
- 旧 monitor/deploy 脚本为 `server_monitorctl.sh` 的兼容包装。
- 服务器验证为 SHA 门控，显式 preflight、Manifest 和 PASS/FAIL/NOT_RUN 保留。
- CI 在每次 push 时执行静态合同。

### 修复

- 降级几何现可安全产生正向 17/18 告警，不再被之前的 formal-authority 要求阻止（`8d7d7ee` 核心修复）。
- 运动安全保持缺口关闭：保守证据可能被丢弃的剩余边界情况。
- Envelope 授权与运动安全分离；持久 Envelope 和偏拉授权被测试覆盖。
- 已加载货物 Envelope 和运动安全加固。
- Pending Cargo Envelope 增长有界。
- 货物监控在 Gravity 证据冲突下被门控。
- 正常生命周期不再产生误报 Code 35。
- Pending Cargo Growth 导致的误报 Code 17 已修复。

### 安全

- Code 17/18 只表示真实空间碰撞风险。
- 定位、Gravity、货物高度、障碍证据和内部故障只能产生 30-35。
- 降级几何**不能**产生 CLEAR 14、剔除货物点、排除静态地图或提交 MapCommit。
- 正式 CLEAR 要求所有授权（正式几何、有效观测无障碍物、全部合同满足）。
- LOST_HOLD 超时输出 33 并停止正式地图剔除。
- 重复时间戳、heartbeat tick 和单帧 CLEAR 不推进确认。
- 过期 Pending Envelope 不能保持 session ready。
- 仅靠地图静态证据不能绕过 `current_source_unvalidated` 或 `cargo_residual_source_unresolved` 的 fail-safe 34。

### 现场验证

2026-07-27 / 2026-07-28 `8d7d7ee` 现场验证：

- SLAM 侧：235 次 Code 18，53 次 Code 17，24 个独立避障片段
- 主控侧：243 次 Code 18 接收，224 次 S3 发送，2.2 秒限速，首次回调到发送延迟 < 1 秒
- SLAM 与主控日志来自不同日期，不是单次同步逐帧运行
- S3 独立闸门路径（总闸关闭场景）未被覆盖

Tag：`validation-obstacle-avoidance-20260728` → `8d7d7ee`

### 已知限制

- S3 独立闸门现场 case 待完成
- 持久地图报告层尚未实现
- NDT fitness 自动熔断待完成
- 重定位最终验收待完成
- Torsion HOIST_MISSING 诊断（2 个 cargo_swing_monitor 测试失败）
- 障碍追踪器边界条件（10 个测试失败，8d7d7ee 基线已知）
- 货物组件融合边界情况（2 个测试失败，8d7d7ee 基线已知）
- 主控侧 Code 17 → S3 路径需独立现场验证

## 货物安全与定位生产加固（2026-07）

### 新增

- 稳健二维 OBB、冻结 `LockedCargoShape` 与实时 `LiveCargoPose`
- 旋转几何统一服务 marker、Cargo Bottom、避障距离、Registration 和 MapCommit 剔除
- `pose_evidence_stamp / height_evidence_stamp / evaluation_stamp` 分离
- LOST_HOLD 显示/正式安全双生命周期、短时运动预测和不确定度扩张
- `STATIONARY_HOLD / MOVING_CONFIRM / CATCH_UP` 状态机
- 结构保持 Registration Source、可观测性代理和 EKF 各向异性协方差
- 不可变 `MapLayerBundle` 与后台 clean 同代发布
- 14/17/18 正式空间合同、heartbeat 新证据/时间 epoch 状态机
- 项目级静态与 ROS Noetic catkin CI、完整工程文档

### 变更

- 吊物中心限制改为 `physical speed * sensor dt + margin`
- Gravity AUXILIARY 以 LiDAR 为主，空载使用三态观测和独立确认
- 结构不足改为 prediction-only，删除 full-ground fallback
- 风险控制台改为 ENTER/CHANGE/REPEAT/CLEAR 事件模式，逐帧数据保留 CSV
- 安装规则补齐 rviz、scripts、docs 和 systemd 模板

### 安全

- 17/18 只表示真实空间碰撞风险；定位、Gravity、吊物高度、障碍证据和内部故障只能输出 30-35
- LOST_HOLD 过期后 marker 可继续显示，但输出 33 且停止正式地图剔除
- 重复时间戳、heartbeat tick 和单帧 CLEAR 不再推进确认

### 验证状态

- Windows 静态合同可执行
- Ubuntu clean build、gtest 与顺序 bag 验收仍是发布准入项
- 根目录 LICENSE 与 package.xml 的 MIT 声明现已一致

## master cargo v1 — OdomAnchorBox 干净管线

### 新增

- OdomAnchorBox：绿色货物框中心固定在 `base_link` / odom anchor
- 货物框 size / height 自适应
- `OdomAnchorSummary` 每 2 秒输出
- cargo debug 点云配置开关
- legacy cargo 配置 gate

### 变更

- 高频 cargo 调试日志降级到 DEBUG
- cargo visualizer 简化为 `base_link` marker visualizer
- HookCargoRemoval 默认关闭
- debug 点云默认关闭
- 主线默认保持 A7 风格平滑轨迹链路

### 移除

- 旧 hook ROI 检测主路径
- 旧 locked hook box 发布路径
- precise box / map corners 主显示路径
- precise box callback

### 验证

- Baseline：cargo 全关，`CraneMotionEKF recovery=0`
- Display-only：只开 OdomAnchorBox，`CraneMotionEKF recovery=0`
- NDT fitness 正常
- 无 `[ERROR]`
- 绿色框中心锚定 `base_link` / odom anchor
- legacy cargo 默认关闭

## v2.1-cleanup (2026-06-24)

### 项目结构清理
- 删除死代码：odometry、visualizer、MappingNode、KISS-ICP、ImGui
- 拆分 mapping.hpp 为 keyframe_manager.hpp
- 清理 CMakeLists.txt（去掉 OpenGL/GLFW/ImGui 依赖）
- 整理 scripts/ 为 postprocess/mapping/deploy/tools
- 重构 README 为项目入口文档
- 统一 doc/ 目录

## v2.0-longterm (2026-06-24)

### 长期在线建图
- MotionGate：静止不建图
- 关键帧 active window：最近 80 帧保留 raw cloud
- 磁盘 tile：20m × 20m 增量落盘，tmp + rename
- MemoryGuard 四级分级（OK/SOFT/HARD/EMERGENCY）
- DiskGuard 磁盘保护
- NDT fitness 健康监控
- observe_only 观察模式
- runtime_status.json 运行状态
- systemd 自动重启

### 动态物体过滤
- BasePayloadChannelFilter：吊货通道过滤
- PayloadTrackManager：吊货轨迹跟踪
- HumanObjectDynamicFilter：人体动态过滤
- DynamicEventManager：统一事件管理

## v1.0（初始版本）

- NDT_OMP 配准
- 网格局部地面分割
- ScanContext + g2o 闭环检测
- 多层地图输出
