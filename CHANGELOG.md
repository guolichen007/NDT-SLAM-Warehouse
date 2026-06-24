# CHANGELOG

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
