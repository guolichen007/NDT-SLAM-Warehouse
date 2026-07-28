# 开发路线

## P0 质量门禁

- [ ] 清零 14 个 gtest 已知失败（cargo_obstacle_tracker, cargo_swing_monitor, cargo_component_fusion）
- [ ] 完成 Bag 验收套件（静止漂移、移动 catch-up、吊物起升/平移、17/18/14 空间合同、epoch 回退）
- [ ] 启用 master branch protection（required CI 全绿后）

## P1 现场验收

- [ ] S3 独立闸门路径现场验证（总闸关闭场景）
- [ ] Controller 侧 Code 17→S3 行为确认
- [ ] NDT 重定位现场验收
- [ ] 同一次运行 SLAM+主控端到端同步日志采集

## P1 定位

- [ ] NDT fitness 自动熔断（fitness 恶化时自动触发重定位或降级）
- [ ] 重定位成功率与耗时统计

## P1 地图

- [ ] 持久地图报告层
- [ ] 跨 bag 地图复用验证
- [ ] 分区地图支持

## P2 吊物/摆动

- [ ] Torsion HOIST_MISSING 诊断完善
- [ ] 货物变化检测（货物被替换后的检测与重新锁定）
- [ ] 多会话 lifelong mapping

## P2 工程增强

- [ ] PLC 信号接入
- [ ] 天车自动控制接口
- [ ] 实时货物状态监控面板
