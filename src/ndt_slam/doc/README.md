# NDT-SLAM 技术文档

当前 master 参考文档。历史证据和设计决策见 [docs/](../../../docs/)。

## 系统

- [系统架构](architecture.md) — 组件图、数据流、线程模型
- [对外接口](api.md) — 话题、服务、参数

## 子系统

- [定位运行时](localization_runtime.md) — NDT 扫描匹配、EKF、MotionGate
- [吊物跟踪与安全](cargo_tracking_and_safety.md) — 检测、生命周期、避障、安全合同
- [地图生命周期](map_lifecycle.md) — 五层地图、会话、静态证据、MapCommit
- [长期在线建图](longterm_mapping.md) — 关键帧、Tile 持久化、会话
- [动态过滤](dynamic_filtering.md) — 人体、吊物通道过滤
- [内存保护](memory_guard.md) — 内存压力分级与处理
- [地图后处理](map_postprocess.md) — clean-map 管线

## 运维

- [配置说明](configuration.md) — YAML 参考
- [部署](deployment.md) — 安装、systemd、launch
- [运行与运维](operations.md) — 运行时命令、监控
- [服务器监控](server_monitoring.md) — monitorctl、诊断、CSV 报告
- [服务器验收 Runbook](server_validation_runbook.md) — SHA 门禁验收
- [工程建图指南](engineering_mapping_guide.md) — 现场建图流程

## 质量

- [测试与验收](testing_and_acceptance.md) — gtest、bag、静态合同
- [故障排查](troubleshooting.md) — 常见故障模式与恢复
