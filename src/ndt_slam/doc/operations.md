# 运行与运维

## 启动后检查

1. `/merged_points`、`/odom`、TF 和 `/ndt_slam/runtime_path` 持续前进。
2. 五个正式地图 topic 的同次消息具有相同 `header.seq`。
3. `/cargo_avoidance/status_code` 在无风险时为 14；故障码必须有明确 reason。
4. CSV 持续写入，而终端只出现吊物、安全和事件型风险日志。

## 关键事件日志

必须立即输出：SO3Guard、非有限 NDT、首次 prediction-only、定位/重定位切换、Motion 状态切换、安全 code/reason 变化。运行风险使用 `ENTER/CHANGE/REPEAT/CLEAR`；相同风险 10 秒内不重复刷屏。

## 吊物观察

LOCKED 后关注冻结尺寸/yaw、实时中心、中心 residual、position source、pose/height evidence age。LOST_HOLD 超过 `formal_hold_sec` 后 marker 可存在，但安全应为 33且正式剔除关闭。

## 地图观察

raw 提交快于 clean 时允许发布较旧但完整的 bundle。不得看到同一 seq 下 raw/clean 内容代次混合。reset/load 后旧 worker 结果不得重新出现。

## 关机与保存

先调用保存服务并确认 PCD 写盘成功，再停止节点。正式五层来自 completed bundle；调试层不应用作生产定位地图。
