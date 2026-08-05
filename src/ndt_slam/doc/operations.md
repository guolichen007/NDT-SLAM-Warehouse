# 运行与运维

## 运行入口

真实传感器标准入口：

```bash
cd /home/ydkj/NDT-slam-ws
./src/ndt_slam/scripts/ops/run_ndt_slam_supervised.sh \
  --workspace /home/ydkj/NDT-slam-ws \
  --use-rviz true
```

supervisor 使用 `flock` 防止重复启动，直接保留 roslaunch 输出；只响应当前运行代次
的原子重启请求，消费成功后删除请求文件，避免同代旧标记被重复使用。Ctrl-C、正常
退出和旧代次请求均不会重启。现阶段暂不启用 systemd；
unit 与安装器只保留为未来模板，不要执行 `install_server_services.sh`、
`systemctl enable` 或 `systemctl start ndt-slam*`。

## 启动后检查

1. `/merged_points`、`/odom`、TF 和 `/ndt_slam/runtime_path` 持续前进。
2. 五个正式地图 topic 的同次消息具有相同 `header.seq`。
3. `/cargo_avoidance/status_code` 在无风险时为 14；故障码必须有明确 reason。
4. CSV 持续写入，而终端只出现吊物、安全和事件型风险日志。

统一入口：

```bash
rosrun ndt_slam server_monitorctl.sh start --follow
rosrun ndt_slam server_monitorctl.sh status
rosrun ndt_slam server_monitorctl.sh follow
rosrun ndt_slam server_monitorctl.sh snapshot
```

`start` 单独使用时后台启动，成功后返回终端并打印持续查看命令；需要启动后直接看到
吊物与避障输出时使用 `start --follow`。按 Ctrl-C 只退出查看，监控仍在后台运行；
结束监控使用 `server_monitorctl.sh stop`。

监控是只读进程，typed safety 是权威源，simple status code 只做一致性校验。
定位恢复的现场观察命令：

```bash
rostopic echo /ndt_slam/relocalization_status
rostopic echo /ndt_slam/localization_health
rosservice call /ndt_slam/relocalize
tail -f "$NDT_SLAM_DATA_ROOT/recovery_watchdog/events.jsonl"
```

## 关键事件日志

必须立即输出：SO3Guard、非有限 NDT、首次 prediction-only、定位/重定位切换、Motion 状态切换、安全 code/reason 变化。运行风险使用 `ENTER/CHANGE/REPEAT/CLEAR`；相同风险 10 秒内不重复刷屏。

## NDT 恢复看门狗

生产 launch 默认启动 `ndt_recovery_watchdog.py`，同时订阅兼容状态和
`/ndt_slam/localization_health`：

- 启动后 30 秒为宽限期；
- 连续退化 8 秒时调用 `/ndt_slam/relocalize`，优先使用进程内静态地图恢复；
- 定位进程仍响应时，即使长时间 Code 31 也不盲目硬重启；60 秒未验证会进入
  `WAITING_STATIONARY`，每个新静止周期只搜索一次，运动状态未知时每 30 秒低频搜索；
- 只有健康消息超过 3 秒中断或重定位服务无响应，才写入当前 supervisor 代次的
  原子请求并以 75 退出；
- 若兼容的 `/ndt_slam/relocalization_status` 或 `/odom` 处理流仍在更新、但新
  `/ndt_slam/localization_health` 从未出现，看门狗判定为源码与二进制不一致并提示重新
  编译，不会反复重启同一个旧二进制；
- 15 分钟最多允许 3 次完整重启，达到预算后保持运行并记录
  `restart_suppressed`，防止传感器或地图故障引起重启风暴。

看门狗是 `required` roslaunch 节点。required 会终止当前 launch；前台 supervisor
以原子请求中的运行代次为准，在 5 秒后重启整套建图定位链。
硬恢复不再从 ROS 定时器线程调用 `os._exit`：定时器只设置重启事件，主线程关闭
timer、调用 ROS shutdown，再返回 75。这样看门狗自身完成正常清理，roslaunch 仍能
识别非零退出并按 required 语义优雅停止其余节点。

`required` 是有意的 fail-safe 边界：看门狗异常退出时不允许 SLAM 在失去长期恢复
保护后静默继续运行。证据目录不可写、软重定位服务暂时不可用等外部错误均在节点内
捕获并记录，不会直接造成看门狗崩溃；真正的进程异常才触发整栈重启。

证据默认写入：

```text
$NDT_SLAM_DATA_ROOT/recovery_watchdog/events.jsonl
$NDT_SLAM_DATA_ROOT/recovery_watchdog/state.json
$NDT_SLAM_DATA_ROOT/recovery_watchdog/restart_request.json
```

`events.jsonl` 记录软恢复、硬重启和重启抑制，达到 5 MiB 后保留 5 份轮转；
`state.json` 原子保存重启窗口历史，`restart_request.json` 绑定当前运行代次。
当前手动运行模式的现场验收应同时保留 roslaunch 终端日志、上述两份证据、监控运行
目录和对应提交 SHA。
外部告警必须解析 `events.jsonl` 的稳定 JSON `action` 字段，并关注
`soft_relocalize`、`hard_restart`、`restart_suppressed`；不得依赖
`logfatal` 的完整文本，日志措辞不属于机器接口。

## 吊物观察

LOCKED 后关注冻结尺寸/yaw、实时中心、中心 residual、position source、pose/height evidence age。LOST_HOLD 超过 `formal_hold_sec` 后 marker 可存在，但安全应为 33且正式剔除关闭。

本版还应同时观察 `runtime_status.json` 中的：

- `cargo_preload_baseline_*`：EMPTY 基线是否达到 5/8 帧、地图代次及空间一致性；
- `geometry_authorization`：应按 `PENDING → POSITIVE_ONLY → FORMAL` 单向升级；
- `geometry_thickness_lower_bound_m/upper_bound_m`：实时可见高度只作为下界；
- `geometry_conservative_*`：顶面参考、跟踪余量、基线/MAD 余量和安全余量；
- pending 障碍的 `bottom_z05/top_z95/vertical_continuity/entirely_above`；
- 障碍 Track 的保留或重置 reason。

`POSITIVE_ONLY` 下检测到可靠危险可以输出 17/18；没有危险时必须保持 33，而不是 14。

运行时同时核对 `motion_corridor_forward_half_angle_deg=45`、
`motion_corridor_angle_rejected_clusters` 和 Pending JSON 中的
`pending_angle_rejected_clusters`。实时点簇与静态高度查询必须使用同一个前方 ±45° 门禁；
侧方计数增长但 17/18 不增长是预期行为。0.30m 内的接触保护不受方向门禁限制，但只输出 Code 29；没有先出现 18 而突然出现的近场候选也输出 29。看到 29 时应保存 RViz/相机画面，并结合类型化消息中的 `reason`、距离、净空、Track 和来源进行人工复核。

`runtime_status.json` 中的 `average_ndt_time_ms` 应在首次有效配准后变为正数，
`stationary_frame_count` 应在持续静止时递增。输入点云的非有限点和超出配置 Z 范围的点
分别累计到 `pointcloud_nonfinite_rejected` 与 `pointcloud_z_outlier_rejected`；后者持续快速
增长时应检查雷达外参和原始数据，而不是放宽地图高度范围。
原始 Pending 风险默认使用 `evidence_backed_only`：只有身份、独立外部障碍 Track、来源、
几何和置信度证据全部通过时才可输出 17/18；否则保持 33 并给出具体门禁原因。禁止使用
`legacy_any_pending` 恢复旧版无差别告警。若需紧急关闭该路径，可将兼容熔断开关
`fusion_provisional_warning_to_official_code` 设为 `false`。

## 地图观察

raw 提交快于 clean 时允许发布较旧但完整的 bundle。不得看到同一 seq 下 raw/clean 内容代次混合。reset/load 后旧 worker 结果不得重新出现。

## 关机与保存

先调用保存服务并确认 PCD 写盘成功，再停止节点。正式五层来自 completed bundle；调试层不应用作生产定位地图。

## 持久化静态证据检查

现场启动使用 `use_sim_time:=false persistent_map:=true`。检查 persistent root
可写、`tiles_objects` 持续增长、`static_evidence_manifest.json` 指向存在的 v3
index，且目录中没有长期残留 `.tmp`。生命周期重建期间允许出现
`static_evidence_manifest.last_good.json` 和 `.suspended` 标记，但二者都不参与
运行时授权；标记存在时旧 active Manifest 也必须拒绝加载。

每班次记录 RSS、磁盘余量、dirty/flushed tile 数、Manifest revision、index
cell 数、clean build applied/snapshot-only/discarded、33/34 reason 分布和 obstacle
track churn。`cargo_frames.csv` 与 `static_evidence.csv` 必须持续写入；终端仅保留
安全事件、吊物状态变化和十秒静态证据摘要。

静态证据摘要中同时观察 `decayed_by_time_gap`、`reset_by_time_gap`、
`observed_free` 与 `temporally_mature`。仓库大范围巡航时前者缓慢增长是正常衰减；
若 `reset_by_time_gap` 快速增长且 mature 长期不增，检查实际配置是否仍使用旧的
30 秒窗口。`observed_free` 增长必须对应真实可观测空闲区域，不能把视野外缺测当空闲。

每次运行的目录、字段和事件窗口见 [服务器监控](server_monitoring.md)。
