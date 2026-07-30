# 运行与运维

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

看门狗是 `required` roslaunch 节点。生产 `ndt-slam.service` 使用
`Restart=on-failure`，因此退出会终止当前 launch 并在 5 秒后重启整套建图定位链。
直接手工执行 `roslaunch` 时没有外部 supervisor，只会安全退出，不会自行拉起。

证据默认写入：

```text
$NDT_SLAM_DATA_ROOT/recovery_watchdog/events.jsonl
$NDT_SLAM_DATA_ROOT/recovery_watchdog/state.json
```

`events.jsonl` 记录软恢复、硬重启和重启抑制；`state.json` 原子保存重启窗口历史。
现场验收应同时保留 systemd journal、上述两份证据和对应提交 SHA。

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

每次运行的目录、字段和事件窗口见 [服务器监控](server_monitoring.md)。
