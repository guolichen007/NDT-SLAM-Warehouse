# 项目文档

本目录存放历史证据、设计决策、事故报告和项目管理文档。

当前技术参考文档见 [src/ndt_slam/doc/](../src/ndt_slam/doc/)。

## 项目管理

- [项目状态](project/status.md) — 能力状态矩阵、编译测试状态
- [开发路线](project/roadmap.md) — P0/P1/P2 优先级规划
- [已知问题](project/known_issues.md) — KI 编号、现象、安全影响、修复计划
- [发布流程](project/release_process.md) — Tag 规范、RC 条件、回滚
- [文档保留策略](project/documentation_policy.md) — 哪些文档进 Git、哪些不保留
- [GitHub 治理](project/github_governance.md) — 分支保护、CI 策略、Code Owner

## 验证证据

- [避障运行证据审查 2026-07-29/30](validation/obstacle_avoidance_runtime_evidence_review_20260729_20260730.md) — 长时间运行统计、版本边界与补充验收项
- [避障端到端现场验证 2026-07-27/28](validation/obstacle_avoidance_e2e_20260727_20260728.md)
- [实图审计 2026-07-21](validation/real_map_audit_20260721.md)
- [Windows 静态合同结果 2026-07-21](validation/windows_static_contract_result_20260721.md)
- [Ubuntu 吊物静态高度融合验证](validation/ubuntu_validation_cargo_static_map_height_fusion_v1.md)

## 设计

- [吊物静态地图高度融合设计](design/cargo_static_map_height_fusion_design.md)

## 事故分析

- [Episode 4：吊物避障根因修复 2026-07-23](incidents/episode4_cargo_avoidance_root_cause_fix_20260723.md)
- [吊物运行时 Episode 1-4 修复 2026-07-24](incidents/cargo_episode_1_4_runtime_fix_20260724.md)
- [Pending Cargo Growth 误报 17 修复 2026-07-24](incidents/pending_cargo_growth_false17_fix_20260724.md)
