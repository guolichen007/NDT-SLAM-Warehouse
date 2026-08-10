# Server Validation Runbook

下面是一条从 exact SHA 到归档的唯一服务器流程。示例变量：

```bash
export WS=~/NDT-slam-ws
export SHA=<EXPECTED_SHA>
export RUN_ID=rc1-live-001
```

## 1. Checkout exact SHA / RC tag

```bash
cd "$WS"
git fetch origin --prune
git switch fix/588-localization-drift-observability
git pull --ff-only origin fix/588-localization-drift-observability
test "$(git rev-parse HEAD)" = "$SHA"
test -z "$(git status --porcelain)"
```

预期：两个 `test` 均返回 0。不得 force push、不得在脏工作区验收。

## 2. Submodule

```bash
git submodule sync --recursive
git submodule update --init --recursive
git submodule status --recursive
```

预期：行首没有 `-`、`+` 或 `U`。

## 3. Clean build

```bash
source /opt/ros/noetic/setup.bash
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin clean -y
catkin build --no-status
source devel/setup.bash
```

## 4. GTest

```bash
catkin run_tests --no-status
catkin_test_results --verbose
```

预期：0 failures。失败时停止，不启动现场服务。

## 5. Prepare/preflight

```bash
rosrun ndt_slam run_server_validation.sh prepare \
  --workspace "$WS" --expected-sha "$SHA" --run-id "$RUN_ID"
```

若希望入口同时执行第 3/4 步，追加 `--build-and-test`。首次没有 Manifest 时
输出 `FIRST_RUN`；这不是“已有恢复证据通过”。

## 6. 手动启动 SLAM（当前模式）

```bash
cd "$WS"
source devel/setup.bash
export NDT_SLAM_DATA_ROOT="$WS/maps/live/current"
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=false use_rviz:=false persistent_map:=true \
  use_ndt_recovery_watchdog:=false
```

Avoidance-first stable 模式禁用 recovery watchdog。定位拒绝帧不写可信地图，
ROS 节点保持运行，并在后续点云上继续执行正常 local NDT。

## 7–8. 启动 SLAM 和 monitor

```bash
rosrun ndt_slam run_server_validation.sh start \
  --workspace "$WS" --expected-sha "$SHA" --run-id "$RUN_ID"
```

可追加 `--record-bag` 录制轻量证据 Bag。预期 preflight 显示单 SLAM 实例、
真实时间、持久化地图和必要 Topic 均 PASS。

## 9. 实时观察

```bash
rosrun ndt_slam server_monitorctl.sh status --workspace "$WS" --run-id "$RUN_ID"
rosrun ndt_slam server_monitorctl.sh follow --workspace "$WS" --run-id "$RUN_ID"
```

17/18、故障和 reason 变化立即输出；周期摘要不应逐帧刷屏。

## 10. 中途 snapshot

```bash
rosrun ndt_slam run_server_validation.sh snapshot \
  --workspace "$WS" --run-id "$RUN_ID"
```

## 11–12. 停止并生成报告

```bash
rosrun ndt_slam run_server_validation.sh stop \
  --workspace "$WS" --run-id "$RUN_ID"
rosrun ndt_slam run_server_validation.sh report \
  --workspace "$WS" --run-id "$RUN_ID"
```

停止监控会 flush 队列；不会删除地图、Tile、active/last-good Manifest。

## 13. 打包

```bash
rosrun ndt_slam run_server_validation.sh pack \
  --workspace "$WS" --run-id "$RUN_ID"
sha256sum -c "$WS/server_runs/$RUN_ID.tar.zst.sha256"
```

默认不打包大型 Bag/Tile/PCD。确需 Bag 时直接调用
`collect_server_artifacts.sh RUN_DIR --include-bag`。

## 14. 上传/归档

上传 `.tar.zst`、`.sha256`，并在 Server Validation Issue 附 exact SHA、
`final_report.md` 和未运行项目。禁止把地图数据提交进 Git。

## 15. 回滚

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
