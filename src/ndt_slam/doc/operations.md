# 运行与运维

## 运行入口

手动调试/验收：

```bash
cd ~/NDT-slam-ws
source devel/setup.bash
export NDT_SLAM_DATA_ROOT="$PWD/maps/live/current"
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=false use_rviz:=true persistent_map:=true \
  use_ndt_recovery_watchdog:=true
```

生产服务：

```bash
sudo systemctl start ndt-slam.service ndt-slam-monitor.service
sudo systemctl status ndt-slam.service ndt-slam-monitor.service
journalctl -u ndt-slam.service -u ndt-slam-monitor.service -f
```

停止服务：

```bash
sudo systemctl stop ndt-slam-monitor.service ndt-slam.service
```

安装或更新 unit 必须使用
[`install_server_services.sh`](deployment.md)，不要手工复制 `.service.in` 模板。

## 启动后检查

1. `/merged_points`、`/odom`、TF 和 `/ndt_slam/runtime_path` 持续前进。
2. 五个正式地图 topic 的同次消息具有相同 `header.seq`。
3. `/cargo_avoidance/status_code` 在无风险时为 14；故障码必须有明确 reason。
4. CSV 持续写入，而终端只出现吊物、安全和事件型风险日志。

统一入口：

```bash
rosrun ndt_slam server_monitorctl.sh status
rosrun ndt_slam server_monitorctl.sh follow
rosrun ndt_slam server_monitorctl.sh snapshot
```

监控是只读进程，typed safety 是权威源，simple status code 只做一致性校验。
定位恢复的现场观察命令：

```bash
rostopic echo /ndt_slam/relocalization_status
rosservice call /ndt_slam/relocalize
tail -f "$NDT_SLAM_DATA_ROOT/recovery_watchdog/events.jsonl"
```

## 关键事件日志

必须立即输出：SO3Guard、非有限 NDT、首次 prediction-only、定位/重定位切换、Motion 状态切换、安全 code/reason 变化。运行风险使用 `ENTER/CHANGE/REPEAT/CLEAR`；相同风险 10 秒内不重复刷屏。

## NDT 恢复看门狗

生产 launch 默认启动 `ndt_recovery_watchdog.py`，订阅
`/ndt_slam/relocalization_status`。它只在连续退化状态上采取动作：

- 启动后 30 秒为宽限期；
- 连续退化 8 秒时调用 `/ndt_slam/relocalize`，优先使用进程内静态地图恢复；
- 连续退化 45 秒，或退化至少 15 秒且 `bad_frames >= 300` 时，先落盘证据，再以
  75 退出；
- 15 分钟最多允许 3 次完整重启，达到预算后保持运行并记录
  `restart_suppressed`，防止传感器或地图故障引起重启风暴。

看门狗是 `required` roslaunch 节点。required 会终止当前 launch，但 ROS Noetic 的
roslaunch 可能在完成清理后返回 0；生产 `ndt-slam.service` 因此使用
`Restart=always`，并在 5 秒后重启整套建图定位链。
直接手工执行 `roslaunch` 时没有外部 supervisor，只会安全退出，不会自行拉起。
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
```

`events.jsonl` 记录软恢复、硬重启和重启抑制；`state.json` 原子保存重启窗口历史。
现场验收应同时保留 systemd journal、上述两份证据和对应提交 SHA。
外部告警必须解析 `events.jsonl` 的稳定 JSON `action` 字段，并关注
`soft_relocalize`、`hard_restart`、`restart_suppressed`；不得依赖
`logfatal` 的完整文本，日志措辞不属于机器接口。

## 吊物观察

LOCKED 后关注冻结尺寸/yaw、实时中心、中心 residual、position source、pose/height evidence age。LOST_HOLD 超过 `formal_hold_sec` 后 marker 可存在，但安全应为 33且正式剔除关闭。

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

每次运行的目录、字段和事件窗口见本文档「服务器监控」章节。


## MemoryGuard / DiskGuard / Watchdog

### MemoryGuard 分级策略

| 级别 | 阈值 | 行为 |
|------|------|------|
| OK | < 6GB | 正常运行 |
| SOFT | 6-7GB | 释放缓存 + flush dirty tiles + malloc_trim |
| HARD | 7-8GB | 暂停地图 commit（NDT/TF 继续） |
| EMERGENCY | > 8GB | 降采样 active map |

配置：
```yaml
memory_guard:
  enabled: true
  soft_threshold_mb: 6000
  hard_threshold_mb: 7000
  emergency_threshold_mb: 8000
  check_interval_sec: 30
```

### DiskGuard 磁盘保护

磁盘空间不足时暂停 tile 写入：

```yaml
disk_guard:
  enabled: true
  min_free_disk_gb: 30
  pause_mapping_when_low: true
```

### Pointcloud Watchdog

雷达断流检测：

```yaml
pointcloud_watchdog:
  stale_timeout_sec: 10.0
```

### NDT Health Monitor

NDT fitness 健康监控：

```yaml
ndt_health:
  fitness_warning_threshold: 2.0
  fitness_warning_count: 50
```

### runtime_status.json 字段

```json
{
  "memory_mb": 162,
  "memory_guard_triggered": false,
  "disk_free_gb": 70.45,
  "disk_guard_triggered": false,
  "pointcloud_stale": false,
  "ndt_fitness_warning": false
}
```


## 服务器监控体系

`server_runtime_monitor.py` 是正式的只读监控入口。它不创建 Publisher、
不修改 ROS 参数、不写地图与 Manifest，也不参与 14/17/18/30–35 的判定。
监控故障不会改变主系统输出；`CargoSafetyStatus schema v7` 始终是安全权威源。

### 数据流

监控低开销订阅：

- `/odom`；
- `/cargo_avoidance/safety_status`（权威）；
- `/cargo_avoidance/status_code`（只做一致性校验）；
- `/cargo_avoidance/static_evidence_debug`；
- `/rosout_agg` 中 WARN/ERROR 和指定关键标签。

它每秒读取 `runtime_status.json`、Manifest 状态、`/proc/<pid>` 和磁盘容量。
默认不订阅 `/map`、`/display_map*` 等大型 PointCloud2。回调只更新有界内存
和有界队列；独立 writer thread 负责 CSV/JSONL。队列满时记录 dropped 数，
不阻塞 SLAM 回调。

### 事件与窗口

以下变化立即打印并写入 `safety_events.jsonl`：code/reason 变化、17/18
进入和解除、30–35 进入和解除、Obstacle Track 变化、静态授权变化、源时间
回退、typed/simple code 不一致、SLAM 进程重启。相同 mismatch 不重复刷屏。

统计同时维护 60 秒、600 秒和整次运行窗口：

- 各安全码样本、时间占比和 reason 分布；
- 最长/当前连续 33、34；
- 34→14 恢复和 34→17/18 确认时间；
- 17/18 事件、unique track、track churn/min；
- 静态授权、source-unvalidated、geometry-rejected 比率；
- odom Hz/age、位姿步长 P50/P95/max；
- RSS、线程、FD、磁盘、runtime status 新鲜度与重启次数；
- 地图点数、dirty/flushed tile、clean worker、静态 epoch/revision/cells。

配置在 `config/server_monitor.yaml`。这里仅允许监控阈值；3 m、5 m、0.8 m、
安全码和算法参数不得放进此文件。

### 每次运行目录

```text
server_runs/<run_id>/
├── run_manifest.json
├── logs/{preflight,build,tests,monitor,slam_journal}.log
├── logs/ros_events.jsonl
├── samples/{runtime,localization,mapping,safety}_samples.csv
├── samples/{safety_events,static_evidence_samples}.jsonl
├── snapshots/{runtime_status.jsonl,manifest_start.json,manifest_end.json,rosparams.yaml}
├── reports/{live_summary.json,final_summary.json,final_report.md}
└── bags/safety_runtime.bag              # 可选
```

CSV 只在空文件写一次表头，监控重启继续 append；`live_summary.json` 使用同目录
tmp + rename 原子替换。默认 Bag 仅含 odom、typed safety、simple code 和静态
证据调试消息，不录制大型点云。

### 日常命令

```bash
rosrun ndt_slam server_monitorctl.sh start \
  --workspace ~/NDT-slam-ws --run-id rc1-live-001 \
  --expected-sha <EXPECTED_SHA>
rosrun ndt_slam server_monitorctl.sh status
rosrun ndt_slam server_monitorctl.sh follow
rosrun ndt_slam server_monitorctl.sh snapshot
rosrun ndt_slam server_monitorctl.sh stop
rosrun ndt_slam server_monitorctl.sh report
```

报告的 `NOT_RUN` 永远不会自动解释为 PASS。监控只生成证据，不能代替 Ubuntu
clean build、gtest、Bag 或长期 soak 的人工验收。

### 服务与通知扩展

`install_server_services.sh` 根据显式 workspace/user/data-root 生成两个 unit。
SLAM 的 flock 覆盖整个 ExecStart 生命周期；monitor 在 SLAM 后启动并可自动
重连。通知渠道应消费 `safety_events.jsonl` 或 `live_summary.json`，不得让邮件、
Slack 等网络调用进入 ROS 回调或安全判定线程。
