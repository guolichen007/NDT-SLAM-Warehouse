# 测试与验收

## 质量状态定义

| 状态 | 含义 |
|---|---|
| **PASS** | 0 个活跃失败，全部通过 |
| **FAIL** | ≥1 个活跃失败 |
| **SKIPPED** | 跳过执行 |
| **DISABLED** | 测试被禁用 |
| **NOT_RUN** | 未执行（不能标为 PASS） |
| **KNOWN_BASELINE_FAILURE** | 已知基线失败，非本次变更引入。仍然为 FAIL 状态，不满足发布准入条件 |
| **REGRESSION_STATUS=NO_NEW_FAILURES** | 与基线比较无新增失败。不改变测试总状态 |

### 重要

- "KNOWN_BASELINE_FAILURE" 不能写成 PASS
- NOT_RUN 不能写成 PASS
- 测试结果以 `catkin_test_results --verbose` 输出为唯一来源

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

- 静止 8 秒且 raw 漂移累计 0.7m：不得退出保持、不得写 local/persistent map。
- 三帧同方向真实运动：进入 MOVING_CONFIRM、有限 CATCH_UP、下一帧恢复写图。
- 横向吊物平移/起升：冻结尺寸/yaw，中心连续跟随。
- LOST_HOLD：短窗扩张 OBB；超时 marker 保持但 code 33、剔除关闭。
- 障碍距离/净空边界：严格验证 14/17/18。
- 小件、弱反射、HAG 残余、聚类不足：UNKNOWN 不得变成空载 CLEAR。
- 连续提交快于 clean：仍发布完整 bundle，最终收敛到最新工作图。
- 不重启 heartbeat 第二次播放 bag：回退帧 30，新 epoch 后恢复。

## 通过标准

无崩溃、无非有限位姿、无 full-ground fallback、无单帧 CLEAR、安全码与几何一致、五层同代、CSV 字段完整、终端无逐帧洪泛。

服务器运行必须保留 `run_manifest.json`、`final_summary.json` 和 `final_report.md`。报告中的 Ubuntu build、gtest、Bag、soak 若未实际执行，必须为 `NOT_RUN`，不得用监控采样自动替代。操作顺序见 [Server Validation Runbook](server_validation_runbook.md)。
