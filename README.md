# NDT-SLAM Warehouse

双雷达室内天车定位、长期建图、吊物跟踪与碰撞避障系统（ROS1 Noetic）。

## 当前正式基线

```text
FIELD_BASELINE_TAG = baseline-field-legacy-20260923
FIELD_BASELINE_SHA = 18619e7c160eb462ac42c97ad9a03a922a6b8b86
```

当前生产模式：

```text
Production Yaw Mode       = LEGACY
Production Rail Authority = DISABLED
Production Yaw Reference  = verified=false
```

> 本软件不是安全认证设备。部署时必须保留外部急停、限位开关和现场安全策略。

## 支持环境

- ROS1 Noetic
- Ubuntu 20.04
- PCL / Eigen / Sophus

## 核心架构

```text
双 3D LiDAR
    ↓ 时间同步与外参变换
base_link 合并点云
    ↓ 结构保持 Registration Cloud
NDT_OMP + CraneMotionEKF
    ↓
地图/定位状态
    ├── Cargo V6（吊物身份/几何/安全）
    ├── Avoidance V4（障碍追踪 + 17/18/29 避障）
    ├── Safety Authority（安全授权）
    └── Persistent Map（长期多层地图）
```

## 快速构建

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 快速启动

```bash
# 长期在线建图与吊物安全
roslaunch ndt_slam warehouse_live_longterm_mapping.launch

# 运行时定位
roslaunch ndt_slam warehouse_runtime.launch
```

## 关键接口

- 对外接口：[api.md](src/ndt_slam/doc/api.md)
- 安全合同：[SAFETY.md](SAFETY.md) + [cargo_tracking_and_safety.md](src/ndt_slam/doc/cargo_tracking_and_safety.md)
- 定位运行时：[localization_runtime.md](src/ndt_slam/doc/localization_runtime.md)
- 配置说明：[configuration.md](src/ndt_slam/doc/configuration.md)
- 部署：[deployment.md](src/ndt_slam/doc/deployment.md)

## 文档导航

- 文档中心：[docs/README.md](docs/README.md)
- 当前项目状态：[docs/project/status.md](docs/project/status.md)
- 技术文档：[src/ndt_slam/doc/](src/ndt_slam/doc/)

## 许可证与安全

MIT License，详见 [LICENSE](LICENSE)。运行安全声明见 [SAFETY.md](SAFETY.md)，
软件安全策略见 [SECURITY.md](SECURITY.md)。
