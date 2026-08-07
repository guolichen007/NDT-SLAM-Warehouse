# `244ba06` 后续稳定性加固说明

## 结论

本轮保留 `244ba06` 已完成的三级 NDT/EKF 门控、前台 supervisor、看门狗预算、
实时货物 OBB、`POSITIVE_ONLY` 正向预警和主线式 `objects_clean` 快照；修复定位
连续性、静态高度尖峰和邻近货物误锁问题。systemd 未启用或修改。

报告中的建议按现场物理约束重新审查：雷达与计算机刚性安装在轨道行车小车上，
车体正常运行没有转向自由度，吊物摆动只能改变吊物 OBB。因此没有把原始 NDT yaw
持续灌入车体发布姿态，也没有允许 UNKNOWN 重力创建吊物，更没有取消隔离期的持久
地图写入保护。这三种做法都会扩大误定位或误识别的后果。

## 定位、Yaw 与恢复

- 每个定位连续性 episode 使用 6 个一致 NDT 帧取得车体 yaw；之后小幅变化只作为
  噪声诊断，不拒绝 XY 测量，不重置 EKF。吊物 OBB yaw 独立更新。
- 手动/自动重定位、回环位姿跳变、全局一致性异常、严格健康验收失败和 LiDAR 时间
  回退显式释放旧 yaw 先验；确认新候选后重新取得 6 帧先验。
- 已拒绝候选的 prediction 若超过动态物理边界，保持最后可信位置、清零速度，并清除
  位置/速度协方差交叉项，避免“位置冻结但速度继续积累”。
- 重定位 reseed 不再永久消耗一次性协方差恢复；5 个名义帧或 15 个软接受帧可重新
  武装下一退化 episode，协方差 trace 上限仍为 25。
- 存在持久地图时，LiDAR 时间回退进入 `STARTUP_QUARANTINE`、强制全局搜索并持续
  Code 31，不能直接恢复 `IDLE + pose_reliable`。

运行时 local-map 与持久地图使用分层质量门限。已接受、有限且结构/可观测性合格的
软修正可以维持 local-map 连续性；`correction_soft`、`output_step_soft`、
prediction-only、隔离或身份代次不一致仍禁止持久地图提交。全局恢复继续使用不可变
持久 registration/`objects_clean` 快照，隔离期不使用不可信位姿改写定位目标。

## 吊物身份、OBB 与避障

- 新吊物身份在启用重力输入时必须具有有效 `LOADED`。
- 候选实测 OBB 必须覆盖 hook，候选中心距 hook 不超过 0.35m，且 hook 在 OBB 长短
  轴两个方向均位于半轴中央 65% 范围内。只擦到 anchor 边缘的相邻托盘不会成为吊物，
  但仍保留为外部障碍。
- 身份通过后，显示和安全几何的 XY 中心严格等于 odom/hook anchor；原始点云中心只
  用于候选关联和摆动诊断。厚度融合、`FORMAL` 条件和 Code 14 逻辑未修改。
- `POSITIVE_ONLY` 继续允许可靠危险输出 18（3–5m）和 17（≤3m），无危险保持 33；
  0.30m 全向接触和缺少 18 接近历史的突然近场候选继续输出 29。

## 静态地图与 `objects_clean`

- 静态障碍 cell 的上下边界改为最近 9 次完整 clean-map 观测的中值/MAD；成熟 cell
  会拒绝孤立 Z 尖峰，不再使用跨会话只增不减的极值。
- clean-map 构建对每个 XY cell 的孤立垂直点做稳健过滤；至少 7 点后才启用，过滤后
  少于 3 点则回退原数据，避免删除稀疏真实结构。
- 连续墙体和固定设施的多点垂直分布被保留；新增接受/拒绝计数用于现场诊断。
- `/display_map_objects_clean` 仍是 map-frame、latched、同代不可变快照，随异步 clean
  重建更新，不改为实时 base-link 点云。

## 验证边界

Windows 侧只执行静态合约、Python 单元测试、YAML 重复键、`git diff --check` 和 LF
检查，不执行 ROS/C++ 编译。新增 C++ gtest 源码覆盖硬 prediction 保持、reseed 恢复
额度、邻近货物中央区拒绝、高度尖峰拒绝和高墙保留；这些测试需在 Linux/ROS 环境
编译执行。最终生产准入仍以 Linux 编译、bag 回放和现场快速运行验收为准。
