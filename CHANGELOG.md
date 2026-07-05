# CHANGELOG

## master cargo v1 - OdomAnchorBox clean pipeline

### Added

- OdomAnchorBox：绿色货物框中心固定在 `base_link` / odom anchor；
- 货物框 size / height 自适应；
- `OdomAnchorSummary` 每 2 秒输出；
- cargo debug 点云配置开关；
- legacy cargo 配置 gate。

### Changed

- 高频 cargo 调试日志降级到 DEBUG；
- cargo visualizer 简化为 `base_link` marker visualizer；
- HookCargoRemoval 默认关闭；
- debug 点云默认关闭；
- 主线默认保持 A7 风格平滑轨迹链路。

### Removed

- 旧 hook ROI 检测主路径；
- 旧 locked hook box 发布路径；
- precise box / map corners 主显示路径；
- precise box callback。

### Validation

- Baseline：cargo 全关，`CraneMotionEKF recovery=0`；
- Display-only：只开 OdomAnchorBox，`CraneMotionEKF recovery=0`；
- NDT fitness 正常；
- 无 `[ERROR]`；
- 绿色框中心锚定 `base_link` / odom anchor；
- legacy cargo 默认关闭。

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

## v1.0 (初始版本)

- NDT_OMP 配准
- 网格局部地面分割
- ScanContext + g2o 闭环检测
- 多层地图输出
