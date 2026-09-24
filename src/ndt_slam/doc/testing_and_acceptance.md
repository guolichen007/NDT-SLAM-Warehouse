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
python scripts/regression/check_docs_contract.py
python -m unittest discover -s tests -p "test_*.py"
```

这些检查覆盖源码完整性、UTF-8、YAML 重复键、关键安全链和静态架构合同，不替代 C++ 编译。

## 运行证据归属

- 现场运行必须从 `run_manifest.json` 或等价机器记录获取完整 SHA，不能用报告撰写时
  所在分支代替运行时版本。
- 自由文本 `reason` 只能辅助定位。消息 Schema、类型化字段和采集 SHA 才是安全语义
  的依据；旧原因串不能验证后续提交新增的代码路径。
- 不同场次的 SLAM 与主控日志只能证明链路各段可工作，不能计算同一事件的端到端
  延迟或宣称 1:1 对应。
- 没有人工标注、独立传感器真值或可复核场景清单时，不报告零误报、零漏报。
- `obstacle_provenance_type` 必须按采集版本的 `CargoSafetyStatus.msg` 枚举解释。

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
- 注入连续高 fitness：5 帧后进入 DEGRADED，15 帧后进入全局搜索；不得把失败匹配
  写入 EKF 或地图。
- 让全局 worker 耗时超过 0.5 秒但小于 12 秒：结果不得仅因局部时效门限被丢弃，
  且仍需相同 map generation、pose version 和两次一致确认。
- 清空 ScanContext 提示并把初始位姿放在地图一角：全局候选必须覆盖地图不同区域，
  不能按嵌套循环只搜索一个角落。
- 连续退化超过软恢复门限：看门狗只调用一次 `/ndt_slam/relocalize`；超过硬门限后
  写入 JSONL/state 并由 systemd 完整重启。15 分钟第 4 次必须被抑制。
- 启动 RViz：`display_map` 默认关闭，`objects_clean` 等必要轻量显示保持原配置；
  操作者可以手工开启全量显示层。

## 通过标准

无崩溃、无非有限位姿、无 full-ground fallback、无单帧 CLEAR、安全码与几何一致、五层同代、CSV 字段完整、终端无逐帧洪泛。

服务器运行必须保留 `run_manifest.json`、`final_summary.json` 和 `final_report.md`。报告中的 Ubuntu build、gtest、Bag、soak 若未实际执行，必须为 `NOT_RUN`，不得用监控采样自动替代。操作顺序见本文档「服务器验收」章节。

对应版本：`f57d68a`。


## Server Validation Runbook

下面是一条从 exact SHA 到归档的唯一服务器流程。示例变量：

```bash
export WS=~/NDT-slam-ws
export SHA=<EXPECTED_SHA>
export RUN_ID=rc1-live-001
```

### 1. Checkout exact SHA / RC tag

```bash
cd "$WS"
git fetch origin --prune
git switch fix/588-localization-drift-observability
git pull --ff-only origin fix/588-localization-drift-observability
test "$(git rev-parse HEAD)" = "$SHA"
test -z "$(git status --porcelain)"
```

预期：两个 `test` 均返回 0。不得 force push、不得在脏工作区验收。

### 2. Submodule

```bash
git submodule sync --recursive
git submodule update --init --recursive
git submodule status --recursive
```

预期：行首没有 `-`、`+` 或 `U`。

### 3. Clean build

```bash
source /opt/ros/noetic/setup.bash
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin clean -y
catkin build --no-status
source devel/setup.bash
```

### 4. GTest

```bash
catkin run_tests --no-status
catkin_test_results --verbose
```

预期：0 failures。失败时停止，不启动现场服务。

### 5. Prepare/preflight

```bash
rosrun ndt_slam run_server_validation.sh prepare \
  --workspace "$WS" --expected-sha "$SHA" --run-id "$RUN_ID"
```

若希望入口同时执行第 3/4 步，追加 `--build-and-test`。首次没有 Manifest 时
输出 `FIRST_RUN`；这不是“已有恢复证据通过”。

### 6. 安装/检查 service

```bash
sudo rosrun ndt_slam install_server_services.sh \
  --workspace "$WS" --user "$(id -un)" \
  --data-root "$WS/maps/live/current"
sudo systemctl cat ndt-slam.service ndt-slam-monitor.service
sudo systemctl show ndt-slam.service \
  -p Restart -p RestartUSec -p StartLimitIntervalUSec \
  -p StartLimitBurst -p User -p WorkingDirectory \
  -p EnvironmentFiles -p ExecStart
sudo systemctl show ndt-slam-monitor.service \
  -p Restart -p RestartUSec -p TimeoutStopUSec \
  -p User -p WorkingDirectory -p EnvironmentFiles -p ExecStart
```

确认 launch 参数显式为 `use_sim_time:=false use_rviz:=false
persistent_map:=true`。安装器会在 `daemon-reload` 后检查最终生效值，因此危险的
drop-in 覆盖会直接导致安装失败；仅查看主 unit 文件不足以排除此风险。确认路径后
可用 `--yes --enable` 非交互安装/启用。

### 7–8. 启动 SLAM 和 monitor

```bash
sudo systemctl start ndt-slam.service
rosrun ndt_slam run_server_validation.sh start \
  --workspace "$WS" --expected-sha "$SHA" --run-id "$RUN_ID"
```

可追加 `--record-bag` 录制轻量证据 Bag。预期 preflight 显示单 SLAM 实例、
真实时间、持久化地图和必要 Topic 均 PASS。

### 9. 实时观察

```bash
rosrun ndt_slam server_monitorctl.sh status --workspace "$WS" --run-id "$RUN_ID"
rosrun ndt_slam server_monitorctl.sh follow --workspace "$WS" --run-id "$RUN_ID"
journalctl -u ndt-slam.service -f
```

17/18、故障和 reason 变化立即输出；周期摘要不应逐帧刷屏。

### 10. 中途 snapshot

```bash
rosrun ndt_slam run_server_validation.sh snapshot \
  --workspace "$WS" --run-id "$RUN_ID"
```

### 11–12. 停止并生成报告

```bash
rosrun ndt_slam run_server_validation.sh stop \
  --workspace "$WS" --run-id "$RUN_ID"
rosrun ndt_slam run_server_validation.sh report \
  --workspace "$WS" --run-id "$RUN_ID"
```

停止监控会 flush 队列；不会删除地图、Tile、active/last-good Manifest。

### 13. 打包

```bash
rosrun ndt_slam run_server_validation.sh pack \
  --workspace "$WS" --run-id "$RUN_ID"
sha256sum -c "$WS/server_runs/$RUN_ID.tar.zst.sha256"
```

默认不打包大型 Bag/Tile/PCD。确需 Bag 时直接调用
`collect_server_artifacts.sh RUN_DIR --include-bag`。

### 14. 上传/归档

上传 `.tar.zst`、`.sha256`，并在 Server Validation Issue 附 exact SHA、
`final_report.md` 和未运行项目。禁止把地图数据提交进 Git。

### 15. 回滚

```bash
sudo systemctl stop ndt-slam-monitor.service ndt-slam.service
cd "$WS"
git switch --detach <LAST_GOOD_SHA>
catkin clean -y
catkin build --no-status
sudo systemctl start ndt-slam.service
```

回滚不得删除当前地图或改写 Manifest。若 suspension marker 存在，系统保持
fail-safe 34，直到当前 epoch 再次满足成熟激活条件。
