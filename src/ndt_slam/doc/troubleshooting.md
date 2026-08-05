# 故障排查

## RViz 只有当前帧，没有地图层

检查五个地图 topic 是否收到消息、`header.frame_id=map`、同次 `header.seq` 是否一致。完整 bundle 只在 clean 完成后发布；若 worker 持续失败，检查 clean reason 和 observation history，而不是先改 RViz 配置。

## 吊物框方向错误

查看冻结 yaw、几何长宽比、特征值比和方向集中度。近正方形货物方向天然不可观，不应强制旋转。确认 marker 使用 map-frame 八角点而非把 base yaw 直接当 map yaw。

## 框跟不上货物

检查 `pose_sensor_dt_sec`、measured/predicted/filtered center、residual 和 position source。核对最大 XY/Z 物理速度，而不是恢复固定逐帧步长。过大的关联门限只能掩盖问题。

## LOST_HOLD 仍显示框但输出 33

这是预期：显示保持和正式证据是不同生命周期。若设备可提供可靠小车/葫芦控制位置，应接入 `HOOK_OR_CONTROLLER`，并单独验证其时间同步与不确定度后再延长正式窗口。

## 频繁 prediction-only

检查 registration mode、静态物体点数、ground fraction、占用覆盖和 observability ratio。禁止重新开启 full-ground fallback；应改善传感器覆盖、外参或结构采样。

## NDT 持续退化或反复重启

先检查 `/ndt_slam/relocalization_status` 和
`$NDT_SLAM_DATA_ROOT/recovery_watchdog/events.jsonl`。`soft_relocalize` 表示已请求
进程内恢复，`hard_restart` 表示已请求外部监督器重启全栈，`restart_suppressed`
表示 15 分钟重启预算已耗尽，此时应检查雷达输入、静态地图和磁盘证据，不要继续
手工循环重启。

若日志记录了 `hard_restart` 但进程没有重新拉起，确认是否使用
`run_ndt_slam_supervised.sh`，并核对 `restart_request.json` 的
`supervisor_run_id`。旧代次请求会被明确忽略；systemd 当前停用。

若持续 Code 31，查看 `/ndt_slam/localization_health`：`MAP_INVALID` 表示 manifest、
地图 UUID、瓦片大小或 SHA-256 校验失败；`VERIFYING` 必须在最近 8 帧内至少 6 帧
满足严格 fitness/EKF/可观测性门限，且不能连续失败超过 2 帧；`WAITING_STATIONARY`
表示节点和健康流仍正常，系统在等待
下一次运动到静止周期，不应人工循环重启。

若看门狗提示 `localization_health` 从未出现、但兼容重定位状态或 `/odom` 仍在更新，说明拉取源码后
尚未重新编译当前工作空间。先停止 supervisor，在 Linux/ROS 环境完成编译并重新加载
`devel/setup.bash`；看门狗会抑制对此类版本不一致的无效硬重启。

## 静止时错误移动或地图增长

检查 Motion 状态变化 reason、raw 增量方向一致性、EKF 速度、CATCH_UP residual，以及 local/persistent 两个写入许可。累计 raw drift 本身不构成移动证据。

## 安全码不符合预期

- 17/18：核对旋转 OBB 距离、保守垂直净空和运行方向夹角；夹角绝对值超过 45° 时不应输出正式告警；
- 29：检查 `reason`。`review_immediate_contact_guard` 表示 0.30m 全方向接触候选，`review_level1_without_approach_history` 表示实时新障碍未经历 18 就突然进入 3m；保存现场图片并核对是否为货物自身点云或 Track 错乱。已绑定地图身份且成熟的静态障碍不使用这一实时历史门禁，可直接输出 17；
- 33：核对 pose/height evidence age 与 `geometry_authorization`；
- 34：检查障碍 ROI 覆盖、有限点和聚类；
- 30：检查超时或时间回退；下一条新 epoch 前进时间戳应恢复。

### 几何长期不授权

先区分三个状态，不要把旧日志中的 `pending_positive_warning_authorized` 当作几何冻结：

- `PENDING`：检查定位是否可靠、锚点到组件是否 ≤0.5m、普通组件在 EMPTY 阶段是否静止；移动中组件还必须已独立成熟且早于当前生命周期，以及
  `cargo_preload_baseline_reason`；
- `POSITIVE_ONLY`：说明可靠正向避障已经可用，但完整厚度尚未复核；无危险输出 33
  是预期行为；
- `FORMAL`：检查露底覆盖或明确可见底边是否提供第二个完整物理测量。

`source_time_invalid_or_rollback`、地图代次变化和生命周期变化会主动撤销旧授权。
`anchor_component_spatially_uncertain` 表示 NDT XY 漂移或候选绑定距离过大，不应降低
0.5m 门限来强行复用另一处静态组件。

### NDT 平均耗时恒为 0、静止帧恒为 0 或地图出现异常 Z

首次有效 NDT 配准后，`average_ndt_time_ms` 使用指数滑动平均更新；如果仍为 0，先确认
是否一直处于 prediction-only。进入静止状态的第一帧 `stationary_frame_count=1`，随后应
逐帧递增，移动或时间回退时归零。

输入点云默认丢弃非有限点以及传感器坐标 Z 不在 `[-4m, 10m]` 的点，并在
`runtime_status.json` 记录累计计数。这能阻止新的极端回波进入 NDT、跟踪和地图，但不会
静默删除旧持久化地图中的历史点；旧会话应先用离线审计工具确认后再在 Linux 端单独治理。
`OUT_OF_APPROVED_MAP_BOUNDS` 在主动探索批准区域之外时是审计告警，不应自动删除 tile。
RSS 随活动地图和静态证据增长但仍低于既有内存门限时先观察是否平台化，不以缩短证据
寿命的方式掩盖增长。

### clearance 与日志字段看似不一致

预警使用的是保守净空，除了货物底面和障碍顶面，还会扣除跟踪误差、顶面 MAD、
障碍不确定度和安全余量。应读取 `geometry_conservative_*` 与 obstacle interval 字段逐项
核对。整体位于货物上方的簇应显示 `entirely_above=true` 并被拒绝；孤立高点应因稳健
分位数或垂直连续性门限被拒绝。若底面仍出现单帧大跳，检查刚体 pose source、sensor dt
和 `live_pose_max_z_speed_mps`，不要改回未冻结 Pending 框。

### 34 原因决策表

| reason | 含义 | 首查内容 |
|---|---|---|
| `cargo_residual_source_unresolved` | 吊物残余来源未解析 | residual classifier ratios/score |
| `current_source_unvalidated` | 当前簇未证明独立于吊物 | source reason、shell distance |
| `static_geometry_below_threshold` | 大簇几何门失败 | 点数/面积/长边/高度/网格实际值与门限 |
| `static_provenance_unavailable` | 没有独立长期来源 | index epoch/revision、clean counters、query reason |
| `static_frames_pending` | provenance 已有但帧数不足 | track ID 与连续计数 |
| `static_duration_pending` | 静态持续时间不足 | track age 与 source stamp |
| `static_track_association_reset` | 障碍身份被重建 | centroid/top step/cell overlap/IoU/reset reason |

优先读取 `/cargo_avoidance/static_evidence_debug`、`static_evidence.csv` 和
`runtime_status.json`，不要在拿到实际分布前降低几何或来源安全门。

## 终端日志过多或过少

生产保持 health=false、risk=true、cargo=true、CSV=true。risk 应只有 ENTER/CHANGE/REPEAT/CLEAR；如果仍逐帧输出，搜索旧 `[PIPELINE_RISK] reason=FRAME_OVERRUN` 路径。

## 服务器监控自身异常

- `monitor already running`：检查 `server_runs/<run>/monitor.pid` 和 lock；不要启动第二实例。
- `runtime_status_stale`：先检查主节点和 persistent root，监控不会尝试重启或改参数。
- typed/simple code mismatch：以 `/cargo_avoidance/safety_status` 为准，检查 heartbeat；相同 mismatch 已节流。
- CSV 没有新行：检查 writer dropped、目录权限和磁盘；重启会 append，不会覆盖历史。
- service 启动失败：用 `systemctl cat` 确认由安装器生成，旧的反向 `! flock` unit 必须删除。
- `.suspended` 长期存在：当前 epoch 尚未成熟或文件系统激活失败，保持 fail-safe 34，禁止手工把 last-good 改名为 active。
- 静态 mature cell 长期过少：确认 `static_map_immature_max_observation_gap_sec=300`
  和 `static_map_immature_gap_retention_ratio=0.50` 已加载；对照
  `decayed_by_time_gap` 与 `observed_free`。前者表示跨长回访的安全衰减，后者才会
  删除证据。不要通过取消 clean-confirmed 门槛来“加速”，否则动态残留会进入正式避障。
