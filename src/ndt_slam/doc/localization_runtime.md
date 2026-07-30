# 定位运行时

## Registration Source

构建模式为 `STRUCTURE_RICH`、`STRUCTURE_RECOVERY`、`GROUND_AUGMENTED` 或 `INSUFFICIENT_STRUCTURE`。地面比例受配置上限约束；结构不足时跳过 NDT，进入 prediction-only，不使用整片地面凑点数。

## 可观测性与 EKF

静态物体局部 XY 法向形成信息代理：

```text
H = sum(weight * normal_xy * normal_xy^T)
```

特征值比描述平移弱方向。测量协方差为：

```text
R = V * diag(r_strong, r_weak) * V^T
r_weak = r_base * inflation
```

中度退化只降低弱方向权重；严重退化移除弱方向创新，但保留强方向合法运动。方向不写死为 X 或 Y。

## 静止与恢复

状态机：

```text
MOVING -> STATIONARY_HOLD -> MOVING_CONFIRM -> CATCH_UP -> MOVING
```

进入静止需要连续、独立时间戳的低速度、低原始增量、有效配准和非严重退化证据。静止时 EKF 对锚点位置和零速度施加约束。

退出静止不能只依赖累计 raw drift。必须满足连续有效帧、物理步长、方向一致、累计运动和可观测性。随机漂移即使累计超过阈值也保持 `DRIFT_ONLY_REJECTED`。

CATCH_UP 每帧按上限追赶可靠结果。直到残差连续收敛，`allow_local_map_update` 和 `allow_persistent_map_commit` 都为 false；完成确认的释放帧也继续禁止写图，下一帧才恢复。

## Bootstrap 与重定位

Bootstrap 是一次性生命周期。完成后地图暂时缩小不会重新进入启动旁路。加载已有地图直接标记 Bootstrap 完成。

异步重定位结果不能单次直接写入运行位姿。确认策略依次检查：

1. map generation 和 pose version 与发起请求时一致；
2. 结果不是重复帧，且没有超过最大帧龄和时间龄；
3. 变换有限、结果有效；
4. 连续结果的平移和 yaw 修正在阈值内一致；
5. 达到所需确认次数。

身份变化、过期、非法或不一致结果会被丢弃并重置确认链。保留中的吊物刚体锁不因
map-frame 重定位被清除。

## NDT fitness 自适应熔断

固定 fitness 阈值容易受目标云密度影响。运行时以最近窗口的中位数和 MAD 建立目标云
相关基线，并保留绝对硬上限：

- 预热完成前只积累可用基线；
- 连续多帧超过自适应阈值后打开熔断；
- 单帧超过硬上限可立即拒绝；
- 熔断打开时不向 EKF 注入 NDT 测量，也不允许该帧 MapCommit；
- 连续多帧低于恢复阈值后闭合；
- NDT 目标源切换时重置基线，避免跨目标云污染统计。

该机制隔离错误测量，不把 prediction-only 位姿升级为地图证据。它不等同于现场重定位
成功率保证，仍需 Ubuntu 编译、Bag 和现场数据验收。

对应版本：`f57d68a`。
