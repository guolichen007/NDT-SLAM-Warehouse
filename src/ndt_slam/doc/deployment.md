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

安装规则仍保留 systemd 模板，但当前阶段禁用该模式，不安装、不 enable、不 start。

## 启动

真实传感器运行：

```bash
cd /home/ydkj/NDT-slam-ws
./src/ndt_slam/scripts/ops/run_ndt_slam_supervised.sh \
  --workspace /home/ydkj/NDT-slam-ws \
  --use-rviz true
```

bag 验收使用 `use_sim_time:=true` 且 `rosbag play --clock`。第二次播放较小时间戳时不重启 heartbeat，用于验证 epoch 恢复。

前台 supervisor 直接显示整栈输出，以运行代次过滤旧的重启请求，并将 Ctrl-C 识别
为人工停止。看门狗仅在健康流中断或服务无响应时请求硬重启；响应正常但定位未验证
时持续 Code 31，并等待严格 20 帧或新的静止周期，不形成重启风暴。

## 发布前门禁

- 本地/远端 SHA 一致且工作区干净；
- CI 静态、catkin build、gtest 全绿；
- 顺序 bag 验收通过；
- 正式配置无重复键；
- 第三方目录无换行噪声；
- 根目录 `LICENSE` 与 `package.xml` 的 MIT 声明一致；发布仍需 exact SHA、
  clean build、gtest 和服务器验收报告。

完整部署到归档流程见 [Server Validation Runbook](server_validation_runbook.md)。
