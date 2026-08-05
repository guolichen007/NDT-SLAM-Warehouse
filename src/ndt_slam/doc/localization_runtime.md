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

Bootstrap 是一次性生命周期。完成后地图暂时缩小不会重新进入启动旁路。没有旧瓦片
时才允许从当前坐标系 bootstrap；检测到持久 registration 瓦片后，启动必须先校验
manifest、地图 UUID、瓦片大小、字节数和 SHA-256，再恢复 registration 与
`objects_clean` 定位目标。校验失败保持 `MAP_INVALID` 与 Code 31，禁止在旧目录中
静默建立原点坐标的新地图。

已有地图启动状态为：

```text
STARTUP_QUARANTINE -> GLOBAL_SEARCH -> VERIFYING -> HEALTHY
                                         |
                                         +-> WAITING_STATIONARY
```

全局候选只修改搜索 seed，不能直接解除隔离。必须在 20 帧滑动窗口内至少 18 帧满足
NDT converged/accepted、fitness ≤0.35、非 prediction-only、有效可观测性、有限位姿和
增量、无步长限制或 EKF 协方差恢复，且不能连续失败超过 2 帧，才进入
`HEALTHY/IDLE`。单帧或双帧瞬态退化不会清空全部收敛证据，连续第三帧失败会撤销授权。
检查点绑定地图 UUID，只提升 seed 优先级。
隔离期间持续 Code 31。已接受且质量有效的 NDT 帧可以更新临时局部定位目标，并允许
货物身份与实时几何预热；预测帧冻结对外 odom。所有 14/17/18/29、货物删除、静态
证据更新和持久地图提交仍被禁止，直到严格定位验收完成。
60 秒未通过但健康流仍正常时等待新的静止周期；运动中仍可搜索和验收，不提前开放避障。

异步重定位结果不能单次直接写入运行位姿。确认策略依次检查：

1. map generation 和 pose version 与发起请求时一致；
2. 结果不是重复帧，且没有超过最大帧龄和时间龄；
3. 变换有限、结果有效；
4. 连续结果的平移和 yaw 修正在阈值内一致；
5. 达到所需确认次数。

身份变化、过期、非法或不一致结果会被丢弃并重置确认链。保留中的吊物刚体锁不因
map-frame 重定位被清除。

### 全局恢复目标与搜索预算

局部恢复仍使用当前 `local_map_`。进入全局恢复后，目标云按以下顺序选择：

1. 点数满足下限的 `objects_clean_map_` 静态快照；
2. `global_map_` 配准粗图；
3. `local_map_` 最后降级目标。

每个异步任务都复制目标云快照并记录 `map_source`，不会在 worker 中读取正在变化的
运行地图。ScanContext 提示优先执行，剩余候选预算由最远点网格覆盖整幅地图，避免
候选数量上限只截取地图一角。局部任务最多 12 个候选；全局任务最多 48 个候选。
粗网格每个坐标轴最多划分 64 段，因此异常边界或离群点只能改变网格间距，不能造成
无上限的中间数组和瞬时内存峰值。

全局搜索保留 map generation、pose version、重复帧和两次一致确认门禁，但使用独立的
`global_result_max_age_frames` 与 `global_result_max_age_sec`。这是为了允许有界全图
搜索完成，不是放宽实时 NDT fitness 或接受过期地图结果。恢复确认后，局部目标优先
从同一 clean 静态图重新裁剪，并同时重置 EKF、fitness 熔断器和依赖 map-frame 的
吊物证据。

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
