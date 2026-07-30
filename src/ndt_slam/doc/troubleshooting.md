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
进程内恢复，`hard_restart` 表示已请求 systemd 重启全栈，`restart_suppressed`
表示 15 分钟重启预算已耗尽，此时应检查雷达输入、静态地图和磁盘证据，不要继续
手工循环重启。

若日志记录了 `hard_restart` 但进程没有重新拉起，先确认当前入口：手动
`roslaunch` 没有外部 supervisor，只会安全退出。生产服务检查：

```bash
sudo systemctl status ndt-slam.service ndt-slam-monitor.service
sudo systemctl show ndt-slam.service \
  -p Restart -p RestartUSec -p StartLimitIntervalUSec \
  -p StartLimitBurst -p ExecStart
journalctl -u ndt-slam.service -u ndt-slam-monitor.service -f
```

如安装器报告有效配置失败，使用 `systemctl cat` 检查 drop-in；不要绕过检查直接
启动旧 unit。

## 静止时错误移动或地图增长

检查 Motion 状态变化 reason、raw 增量方向一致性、EKF 速度、CATCH_UP residual，以及 local/persistent 两个写入许可。累计 raw drift 本身不构成移动证据。

## 安全码不符合预期

- 17/18：核对旋转 OBB 距离和保守垂直净空；
- 33：核对 pose/height evidence age，不能用 evaluation stamp 刷新；
- 34：检查障碍 ROI 覆盖、有限点和聚类；
- 30：检查超时或时间回退；下一条新 epoch 前进时间戳应恢复。

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
