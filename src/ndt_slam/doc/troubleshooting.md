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

## 静止时错误移动或地图增长

检查 Motion 状态变化 reason、raw 增量方向一致性、EKF 速度、CATCH_UP residual，以及 local/persistent 两个写入许可。累计 raw drift 本身不构成移动证据。

## 安全码不符合预期

- 17/18：核对旋转 OBB 距离和保守垂直净空；
- 33：核对 pose/height evidence age，不能用 evaluation stamp 刷新；
- 34：检查障碍 ROI 覆盖、有限点和聚类；
- 30：检查超时或时间回退；下一条新 epoch 前进时间戳应恢复。

## 终端日志过多或过少

生产保持 health=false、risk=true、cargo=true、CSV=true。risk 应只有 ENTER/CHANGE/REPEAT/CLEAR；如果仍逐帧输出，搜索旧 `[PIPELINE_RISK] reason=FRAME_OVERRUN` 路径。
