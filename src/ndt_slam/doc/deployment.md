# 部署

## 平台

目标平台是 Ubuntu 20.04 / ROS Noetic。Windows 只承担源码和静态检查，不构成运行准入。

## 构建与测试

```bash
cd ~/NDT-slam-ws
source /opt/ros/noetic/setup.bash
catkin clean -y
catkin build
catkin run_tests
catkin_test_results --verbose
```

安装规则会安装节点/库、头文件、launch、config、rviz、doc、scripts 和 systemd service 模板。服务文件中的工作区、用户和日志路径必须在部署机上复核后再启用。

## 启动

```bash
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=false persistent_map:=true
```

bag 验收使用 `use_sim_time:=true` 且 `rosbag play --clock`。第二次播放较小时间戳时不重启 heartbeat，用于验证 epoch 恢复。

## 发布前门禁

- 本地/远端 SHA 一致且工作区干净；
- CI 静态、catkin build、gtest 全绿；
- 顺序 bag 验收通过；
- 正式配置无重复键；
- 第三方目录无换行噪声；
- 未决许可证由仓库所有者明确。当前 `package.xml` 声明 MIT，但根目录缺少 LICENSE，不能由维护脚本自行补写授权文本。
