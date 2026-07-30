# 开发路线

## P0 质量门禁

- [ ] 清零 14 个 gtest 已知失败（cargo_obstacle_tracker, cargo_swing_monitor, cargo_component_fusion）
- [ ] 完成 Bag 验收套件（静止漂移、移动 catch-up、吊物起升/平移、17/18/14 空间合同、epoch 回退）
- [ ] 启用 master branch protection（required CI 全绿后）

## P1 现场验收

- [ ] S3 独立闸门路径现场验证（总闸关闭场景）
- [ ] Controller 侧 Code 17→S3 行为确认
- [ ] NDT 重定位现场验收
- [ ] `f57d68a` Pending 静态风险路径现场验收（静态独立授权、重置和禁止 CLEAR）
- [ ] `f57d68a` NDT fitness 熔断与恢复 Bag 验收
- [ ] `f57d68a` 重定位多帧确认、过期丢弃和身份切换 Bag 验收
- [ ] 同一次运行 SLAM+主控端到端同步日志采集

## P1 定位

- [x] NDT fitness 自适应熔断（`f57d68a`，持续恶化隔离测量和地图提交）
- [x] 重定位结果身份、时效和多帧一致性确认（`f57d68a`）
- [x] `objects_clean` 静态地图优先的有界全图重定位（`60884c0`）
- [x] 软重定位、证据落盘和 systemd 全栈恢复看门狗（`2759128`）
- [ ] 重定位成功率与耗时统计

## P1 地图

- [x] 定位地图后处理报告原子写入与回读校验（`f57d68a`）
- [ ] 持久地图会话级汇总报告
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
