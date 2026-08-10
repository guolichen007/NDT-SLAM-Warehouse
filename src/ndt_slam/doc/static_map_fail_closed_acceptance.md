# STATIC_MAP_BUILD_FAIL_CLOSED 实施与验收

## 运行边界

`STATIC_MAP_BUILD_FAIL_CLOSED` 只关闭 SLAM 数据链权限。系统不发布、调用或假设任何天车停车、PLC、Modbus、电机或急停接口。Code31 表示定位与正式避障不再可信，ROS 节点继续运行；现场操作员负责人工停车，确认停稳后调用 `/start_new_mapping_segment`。

Supervisor 只处理进程异常退出。算法质量退化、Code31、`PAUSED_QUALITY` 和 `PAUSED_IO` 均不退出进程，也不触发 roslaunch 自动重启。

## 权限状态

- `ACTIVE`：segment、SelfMask、source time 和 archive 均健康，可提交 Trusted Map 与关键归档。
- `PAUSED_QUALITY`：high fitness 或普通质量退化，只暂停正式地图写入，不发布31。
- `PAUSED_IO`：关键 archive 的 job/byte/disk/age/latency 门限失败，发布30；当前段标记 `ARCHIVE_INCOMPLETE`，恢复后仍须人工新建段。
- `FAIL_CLOSED`：连续 hard failure 达到已有健康窗口，或 non-finite、严重时间回退、物理不可能、identity corruption 等立即失败；发布31，冻结 AcceptedPose 和14/17/18，所有正式地图、objects_clean、static evidence 和 baseline 写入关闭。

一个普通 NDT non-convergence 帧和单独 high fitness 不会进入 `FAIL_CLOSED`。本 profile 强制关闭 relocalization、global consistency reseed、VERIFYING 和 live loop-correction；loop candidate 只留作离线重建线索。

## 开段前置条件

部署时必须显式填写 `mapping_campaign.campaign_uuid` 与 `mapping_campaign.survey_pass_id`，并用现场点云完成 `sensor_body_self_mask` 标定。`commissioned=false` 时允许 raw、mask-removed、registration preview、RViz、rosbag 和 diagnostics，但 Code30 且正式地图零写入。

Trigger 是操作员“设备已人工停稳”的主授权。软件只硬检查 source-time 连续、SelfMask commissioned、archive/disk 健康、archive idle 和旧段封存；NDT/raw motion 静止只作 advisory。

```bash
rosservice call /start_new_mapping_segment
```

## Cargo 与避障合同

- Cargo 由 association anchor、measured pose、safety geometry 三层表示；hook 是身份锚，不强制 safety center 回零。
- identity 与 positive danger 同步累计3个不同有效帧，并通过一个 `CargoFrameDecision` 原子提交；Formal shape 仍为 latest-8 中5帧。
- Z 权威依次为 `DIRECT_BOTTOM`、`SUPPORTED_TOP_MINUS_FROZEN_HEIGHT`、`FRESH_HELD_FORMAL`、`INVALID(Code33)`；partial/prediction/display 不可覆盖正式 bottom 或 frozen height。
- `far_history_confirm_frames` 与 `far_history_confirm_duration_sec` 是配置参数。业务不变量是同一 track 在 `measured_distance - combined_xy_uncertainty > 5m` 且不超过8m时形成远到近历史。
- 5–8m只建轨；3–5m有 authority 且 `clearance_safe <0.8m` 为18；不超过3m为17；缺远场 authority 为29。`clearance_safe >=0.8m` 禁止17/18/29。0.30m contact shell 全方向29兜底。

`6 frames + 0.5s` 只作为严格测试和候选调参组合，不是不可变生产常量。生产初值为3帧和0.2秒，未经固定 bag/实机证据不得主动收紧。

## 归档与认证

archive queue 同时受 `max_jobs` 和 `max_queue_mib` 限制，内存统计包含正在写的 job。关键数据拒绝或写失败会报废当前段；best-effort diagnostics 只计数。PCD、SHA256、manifest 与磁盘写入均在 archive worker，不在 LiDAR/NDT callback。

离线链为：

```text
closed immutable segments
  -> static_map_rebuilder (ScanContext + NDT/ICP evidence + g2o + raw reprojection)
  -> CANDIDATE_NOT_CERTIFIED split maps
  -> independent route validation
  -> static_map_certifier (>=3 episodes, >=2 survey passes, hashes)
  -> human approval
  -> baseline_installer (immutable baseline UUID + atomic CURRENT.json)
```

`static_map_rebuilder` 只接受 checksum 完整且段状态为 `CLOSED` 或 `FAILED_CLOSED` 的数据；`RUNNING`、`ABORTED_CRASH` 和 `ARCHIVE_INCOMPLETE` 不能认证。同一 cell 在一个 segment 中超过 `offline_certification.episode_gap_sec` 后再次出现，才计为新的 episode。

重投影后的 Z 使用 `robust_z_bin_size_m` 与上下分位配置按 cell 去除孤立高度尖峰，再加 `robust_z_margin_m` 保守边界；这一步只在离线工具执行。

## 固定回归输入

每个阶段都必须生成具名 candidate manifest。control 与 candidate 允许不同的完整 commit SHA，其余 bag、playback rate、config、canonical ROS parameters、topics、TF/extrinsic、SelfMask、registration target、persistent initial state、profile 和 feature flags 必须相同。Ephemeral A/B 只允许该一个 flag 不同。

固定 bag：`/home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag`

预期 SHA256：`a6805f48ca0cccf231370045808c60ca1c623ac2c6bf2c7b9ec05b804d7df33c`

```bash
python3 scripts/regression/control_manifest.py create \
  --experiment-name ec64a9f-control \
  --commit ec64a9fddb1c9c4d828f448c27c4f7399457eac4 \
  --playback-rate 1.0 \
  --config src/ndt_slam/config/live_longterm_mapping.yaml \
  --ros-parameters-json acceptance/ros_parameters.json \
  --sensor-topics-json acceptance/sensor_topics.json \
  --tf-extrinsic acceptance/extrinsic.yaml \
  --registration-target acceptance/registration_target.pcd \
  --runtime-profile STATIC_MAP_BUILD_FAIL_CLOSED \
  --feature-flags-json acceptance/feature_flags.json \
  --output acceptance/control_manifest.json

scripts/acceptance/run_phase_a_gate.sh \
  acceptance/control_manifest.json acceptance/phase_a_manifest.json \
  acceptance/phase_a
```

Phase B/C/D 分别使用对应 wrapper。每个 wrapper 在启动 bag 前核对所有固定输入；运行结束用 `compare_runtime_results.py` 检查 fitness、convergence、prediction-only、odom step、NDT/clean/callback latency、CPU、RSS、worker discard/static growth、Cargo 指标，以及 archive jobs/bytes/age/latency 硬上限与关键拒绝为零。

## Ubuntu/现场验收命令

以下命令必须在 Ubuntu ROS1 环境执行；Windows 开发机不把它们记为通过：

```bash
catkin build ndt_slam
catkin run_tests ndt_slam
catkin_test_results --verbose
sha256sum /home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag
python3 scripts/regression/control_manifest.py validate \
  acceptance/control_manifest.json --verify-files
```

Phase A 运行同一 bag 验证 Cargo/17/18/29 且 NDT/Odom 不回退；Phase B 增加 mapping smoke、worker discard/static growth、objects_clean 高峰与 RSS；Phase C 注入 transient/hard/immediate failure、SelfMask/IO 和 crash latch；Phase D 只运行离线重建、独立路线验证、认证与原子安装。实机使用前还必须完成 SelfMask removed-ratio、磁盘吞吐和人工停稳流程验收。

本文件所列 Ubuntu catkin/gtest、固定 bag、mapping smoke 与实机验证在 Windows Codex 环境中的结果统一记为 `NOT_RUN_LOCAL_ENV`，不得写成 PASS。
