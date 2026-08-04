# 服务器监控体系

`server_runtime_monitor.py` 是正式的只读监控入口。它不创建 Publisher、
不修改 ROS 参数、不写地图与 Manifest，也不参与 14/17/18/29/30–35 的判定。
监控故障不会改变主系统输出；`CargoSafetyStatus schema v7` 始终是安全权威源。

## 数据流

监控低开销订阅：

- `/odom`；
- `/cargo_avoidance/safety_status`（权威）；
- `/cargo_avoidance/status_code`（只做一致性校验）；
- `/cargo_avoidance/static_evidence_debug`；
- `/rosout_agg` 中 WARN/ERROR 和指定关键标签。

它每秒读取 `runtime_status.json`、Manifest 状态、`/proc/<pid>` 和磁盘容量。
默认不订阅 `/map`、`/display_map*` 等大型 PointCloud2。回调只更新有界内存
和有界队列；独立 writer thread 负责 CSV/JSONL。队列满时记录 dropped 数，
不阻塞 SLAM 回调。每个 track 统计表和事件节流表也有独立容量上限，超限时淘汰
最早条目并在摘要中记录淘汰计数。

## 事件与窗口

以下变化立即打印并写入 `safety_events.jsonl`：code/reason 变化、17/18、29 待复核事件
进入和解除、30–35 进入和解除、Obstacle Track 变化、静态授权变化、源时间
回退、typed/simple code 不一致、SLAM 进程重启。相同 mismatch 不重复刷屏。

统计同时维护 60 秒、600 秒和整次运行窗口：

- 各安全码样本、时间占比和 reason 分布；
- 最长/当前连续 33、34；
- 34→14 恢复和 34→17/18 确认时间；
- 17/18 正式告警事件、29 待复核事件、unique track、track churn/min；
- 静态授权、source-unvalidated、geometry-rejected 比率；
- odom Hz/age、位姿步长 P50/P95/max；
- RSS、线程、FD、磁盘、runtime status 新鲜度与重启次数；
- 地图点数、dirty/flushed tile、clean worker、静态 epoch/revision/cells。

配置在 `config/server_monitor.yaml`。这里仅允许监控阈值；3 m、5 m、0.8 m、
安全码和算法参数不得放进此文件。

## 每次运行目录

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
证据调试消息，不录制大型点云。由监控 writer 管理的追加型 CSV、JSONL 和文本文件默认在 20 MB
轮转并保留 2 个历史段；每次监控启动会按 `output_retention_runs` 删除最旧的已结束
运行目录（默认保留 20 次）。当前运行、仍可能被监控进程占用的目录、隐藏目录、
符号链接和没有 `run_manifest.json` 的人工目录均不会删除。

## 日常命令

```bash
rosrun ndt_slam server_monitorctl.sh start \
  --workspace ~/NDT-slam-ws --run-id rc1-live-001 \
  --expected-sha <EXPECTED_SHA>
# 后台启动后立即留在当前终端持续查看吊物/避障信息：
rosrun ndt_slam server_monitorctl.sh start --follow
rosrun ndt_slam server_monitorctl.sh status
rosrun ndt_slam server_monitorctl.sh follow
rosrun ndt_slam server_monitorctl.sh snapshot
rosrun ndt_slam server_monitorctl.sh stop
rosrun ndt_slam server_monitorctl.sh report
```

`start` 默认后台运行并在成功后打印 `follow` 命令；这是脚本和验收流程依赖的兼容
语义。`start --follow` 会在启动就绪后执行同一个跟随器，按 Ctrl-C 只退出查看，
不会停止后台监控。需要结束采集时必须显式执行 `stop`。

报告的 `NOT_RUN` 永远不会自动解释为 PASS。监控只生成证据，不能代替 Ubuntu
clean build、gtest、Bag 或长期 soak 的人工验收。

## 服务与通知扩展（当前停用）

现阶段运行不启用 systemd，使用手动 `roslaunch` 与上述 `monitorctl` 命令。
`install_server_services.sh` 和 unit 模板仅保留给未来部署，不应在当前服务器执行。
模板可根据显式 workspace/user/data-root 生成两个 unit。
SLAM 的 flock 覆盖整个 ExecStart 生命周期；monitor 在 SLAM 后启动并可自动
重连。通知渠道应消费 `safety_events.jsonl` 或 `live_summary.json`，不得让邮件、
Slack 等网络调用进入 ROS 回调或安全判定线程。
