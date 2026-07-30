# 部署

## 平台

目标平台是 Ubuntu 20.04 / ROS Noetic。Windows 只承担源码和静态检查，不构成运行准入。

## 构建与测试

克隆时必须初始化固定版本的 `ndt_omp` 子模块：

```bash
git clone --recurse-submodules <repository-url> NDT-slam-ws
# 已有工作区：git submodule update --init --recursive
```

`package.xml` 声明 ROS/rosdep 可解析的 Eigen、PCL、yaml-cpp、OpenCV、
Boost、g2o 与 `ndt_omp` 依赖。ROS Noetic 没有与本工程接口固定一致的
Sophus rosdep 包；干净环境必须按 `.github/workflows/ci.yml` 所列提交安装
Sophus。g2o 也按工作流中的 `2023_02_14` 固定版本安装，避免系统版本的
CMake target 名称与本工程不一致。

```bash
cd ~/NDT-slam-ws
source /opt/ros/noetic/setup.bash
catkin clean -y
catkin build
catkin run_tests
catkin_test_results --verbose
```

安装规则会安装节点/库、头文件、launch、config、rviz、doc、ops 脚本和
systemd service 模板。服务文件不再硬编码用户和路径，必须通过显式安装入口：

```bash
sudo rosrun ndt_slam install_server_services.sh \
  --workspace ~/NDT-slam-ws --user "$(id -un)" \
  --data-root ~/NDT-slam-ws/maps/live/current
```

该入口修正了旧服务反向 `! flock` 的错误；flock 现在覆盖 SLAM 整个
ExecStart 生命周期。

## 启动

```bash
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=false persistent_map:=true
```

bag 验收使用 `use_sim_time:=true` 且 `rosbag play --clock`。第二次播放较小时间戳时不重启 heartbeat，用于验证 epoch 恢复。

生产 launch 默认包含 `ndt_recovery_watchdog.py`。连续重定位失败超过硬门限时，
该 required 节点会记录 `$NDT_SLAM_DATA_ROOT/recovery_watchdog/` 证据并以非零码
结束 launch。ROS Noetic 的 required 语义负责停止整套 launch，但 roslaunch 本身可能
正常返回，因此生产 unit 使用 `Restart=always`，在 5 秒后重启全栈；显式执行
`systemctl stop` 时 systemd 不会重启该服务。
部署后必须确认 unit 使用仓库当前模板，旧 unit 的较长 `RestartSec` 不会自动更新。
不要手工复制未展开的 `.service.in`。每次更新该分支后重新执行
`install_server_services.sh`。安装器会同时验证 SLAM 和 monitor 的渲染结果，
包括用户、路径、重启策略与核心 `ExecStart`，随后执行 `systemctl daemon-reload`，
再通过 `systemctl show` 校验最终生效的配置。drop-in 可以增加资源限制、日志等
配置，但若覆盖 `Restart`、`RestartSec`、运行用户、工作目录、环境文件或核心
启动命令，安装会失败并打印合并后的 unit，必须先处理冲突。
`StartLimitIntervalSec=300` 与 `StartLimitBurst=5` 位于 `[Unit]` 段，在看门狗
自身预算之外提供 systemd 级重启风暴保护。

`Restart=always` 是长期生产服务的预期语义。有限时长 bag、单次调试或其他应在
正常结束后保持停止的任务，应直接使用隔离的 `roslaunch`/bag 验收入口，不要复用
该生产 unit。

## 发布前门禁

- 本地/远端 SHA 一致且工作区干净；
- CI 静态、catkin build、gtest 全绿；
- 顺序 bag 验收通过；
- 正式配置无重复键；
- 第三方目录无换行噪声；
- 根目录 `LICENSE` 与 `package.xml` 的 MIT 声明一致；发布仍需 exact SHA、
  clean build、gtest 和服务器验收报告。

完整部署到归档流程见 [Server Validation Runbook](server_validation_runbook.md)。
