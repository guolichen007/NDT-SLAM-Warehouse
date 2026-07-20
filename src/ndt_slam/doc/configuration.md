# 配置说明

生产入口是 `config/live_longterm_mapping.yaml`，双雷达合并使用 `config/merger_params.yaml`。参数修改必须同步静态合同、单元测试和 Ubuntu bag 验收。

## 定位

- `registration_input`：静态物体、候选和地面的体素/重复率、地面比例和最小结构点数。`allow_full_ground_fallback` 必须为 false。
- `ndt_observability`：中度/严重特征值比和弱方向膨胀系数。
- `stationary_policy`：进入/退出确认、方向余弦、CATCH_UP 步长与完成条件。
- `motion_gate`：持久关键帧门限；不能代替 stationary policy。

## 吊物

- `hook_cargo_lock.orientation_*`：OBB 几何和多帧方向确认。
- `live_pose_max_xy_speed_mps`、`live_pose_max_z_speed_mps`、`live_pose_step_margin_m`：按 sensor dt 的中心约束。
- `formal_hold_sec`：LOST_HOLD 的正式安全/剔除短窗，不是 marker 显示时长。
- `lost_hold_sec`、`lost_clear_sec`：锁状态与显示生命周期。
- `cargo_bottom_fusion`：点支撑、先验、时间窗、跳变确认和最大证据年龄。
- `cargo_safety`：3 m、5 m、0.8 m 几何阈值与观测质量门限。

## Gravity

输入话题固定为 `/gravity`。`hook_load_signal.role` 只能是 `REQUIRED/AUXILIARY/DISABLED`。生产修改角色前必须重新验证空载、紧凑货物、断流和冲突场景。

## 日志

生产默认：`debug_perf: false`、health console 关闭、event-mode risk console 开启、cargo console 开启、CSV 开启。risk 只在 ENTER/CHANGE/REPEAT/CLEAR 输出，重复周期为 10 秒。

## 配置准入

```bash
python3 scripts/regression/check_yaml_duplicate_keys.py
python3 scripts/regression/check_repository_integrity.py
python3 scripts/regression/check_cargo_safety_e2e.py
```
