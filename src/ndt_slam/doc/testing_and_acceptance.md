# 测试与验收

## Windows 静态项

```powershell
git diff --check
python scripts/regression/check_yaml_duplicate_keys.py
python scripts/regression/check_repository_integrity.py
python scripts/regression/check_cargo_safety_e2e.py
python -m unittest tests.test_server_monitor
```

这些检查覆盖源码完整性、UTF-8、YAML 重复键、关键安全链和静态架构合同，不替代 C++ 编译。

## Ubuntu 编译项

执行 clean catkin build、全部 gtest 和 `catkin_test_results --verbose`。重点单测包括 stationary policy、registration builder、observability、cargo OBB、bottom fusion、clean worker 活性、heartbeat 状态机和时间回退。

## 顺序 bag 场景

- 静止 8 秒且 raw 漂移累计 0.7 m：不得退出保持、不得写 local/persistent map。
- 三帧同方向真实运动：进入 MOVING_CONFIRM、有限 CATCH_UP、下一帧恢复写图。
- 横向吊物平移/起升：冻结尺寸/yaw，中心连续跟随。
- LOST_HOLD：短窗扩张 OBB；超时 marker 保持但 code 33、剔除关闭。
- 障碍距离/净空边界：严格验证 14/17/18。
- 小件、弱反射、HAG 残余、聚类不足：UNKNOWN 不得变成空载 CLEAR。
- 连续提交快于 clean：仍发布完整 bundle，最终收敛到最新工作图。
- 不重启 heartbeat 第二次播放 bag：回退帧 30，新 epoch 后恢复。

## 通过标准

无崩溃、无非有限位姿、无 full-ground fallback、无单帧 CLEAR、安全码与几何一致、五层同代、CSV 字段完整、终端无逐帧洪泛。

服务器运行必须保留 `run_manifest.json`、`final_summary.json` 和
`final_report.md`。报告中的 Ubuntu build、gtest、Bag、soak 若未实际执行，
必须为 `NOT_RUN`，不得用监控采样自动替代。操作顺序见
[Server Validation Runbook](server_validation_runbook.md)。
