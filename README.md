# NDT-SLAM Warehouse

双雷达室内天车定位、长期建图、吊物跟踪与碰撞避障系统。

## 项目简介

NDT-SLAM Warehouse 是面向室内仓库和天车作业的 ROS1 Noetic 工程。系统同时维护
NDT 定位、长期在线建图、五层地图输出和一条完整的吊物安全协议。

当前现场验证基线：[`validation-obstacle-avoidance-20260728`](https://github.com/guolichen007/NDT-SLAM-Warehouse/tree/validation-obstacle-avoidance-20260728)（`8d7d7ee`）。

> 本软件不是安全认证设备。部署时必须保留外部急停、限位开关和现场安全策略。

当前主线已包含 NDT fitness 自适应熔断、重定位多帧确认、全图静态地图恢复、
长期失败看门狗、Pending 风险诊断和定位地图报告原子写入。当前实现的
Windows 静态检查与历史现场证据分开管理；现场验证 Tag 不会随代码提交自动前移。

## 核心能力

- **双雷达融合定位**：结构优先 NDT 配准 + 各向异性 EKF + 静止保持策略。结构不足时进入 prediction-only，不回退到整片地面。
- **定位故障隔离**：按目标云分布建立自适应 fitness 基线；持续恶化时隔离 NDT 测量和地图提交，恢复后再闭合。异步重定位结果需满足身份、时效和多帧一致性。
- **定位自动恢复**：局部恢复失败后优先使用 `objects_clean` 静态地图做有界全图搜索；长期无法恢复时由看门狗落盘证据并安全结束当前 launch，等待当前人工运行策略重新启动。
- **长期在线建图**：MotionGate 静止不建图，关键帧 active window，20m×20m tile 增量落盘，MemoryGuard/DiskGuard 保护。
- **五层地图输出**：registration / display / ground / objects / objects_clean，同代发布，`header.seq` 一致。
- **吊物刚体跟踪**：稳健二维 OBB 检测，锁定后冻结长/宽/高/yaw，作业期间只更新中心。起升不会导致错误丢失。
- **多源几何融合**：起吊前静态基线与起吊后约束融合形成 `PENDING / POSITIVE_ONLY / FORMAL` 三级授权；可见高度按下界处理，不再因厚度源差异永久卡死。
- **正向风险门禁**：`POSITIVE_ONLY` 只在身份、稳健实测尺寸和保守底面连续确认后允许 17/18；无危险时保持 33，只有 `FORMAL` 可输出 14、剔除货物点和授权 MapCommit。
- **障碍物追踪与安全码**：Code 14（CLEAR）/ 17（≤3m）/ 18（3-5m）/ 30-35（故障），只由真实空间碰撞风险产生正向告警。
- **主控集成**：Code 18 → 外部主控程序 → S3 语音告警，已取得现场证据。
- **服务器监控**：统一运维入口、SHA 门禁验证、CSV 诊断输出、只读安全窗口。

## 系统架构

```
双雷达 → pointcloud_merger → /merged_points
  │
  ├── 定位链路
  │   ├── RegistrationCloudBuilder（结构保持）
  │   ├── NDT + NdtObservability
  │   ├── NdtFitnessCircuitBreaker
  │   ├── 局部/全局重定位（objects_clean 静态地图优先）
  │   ├── 各向异性 EKF（CraneMotionEKF）
  │   ├── StationaryMotionPolicy
  │   ├── MapCommit → Clean Worker → 五层 MapLayerBundle
  │   └── NDT Recovery Watchdog → 软重定位 / 证据落盘 / 安全退出
  │
  └── 吊物安全链路
      ├── Cargo Observation → 生命周期（EMPTY→CANDIDATE→LOCKED→LOST_HOLD）
      ├── CargoGeometryFusion
      │   ├── PENDING（仅严格证据门控的正向风险可输出 17/18）
      │   ├── POSITIVE_ONLY（只允许可靠的 17/18）
      │   └── FORMAL（可 CLEAR、可 map exclusion、可 MapCommit）
      ├── Cargo Bottom
      ├── CargoObstacleTracker
      ├── CargoAvoidanceFusion → CargoSafetyStatus
      └── cargo_alarm_heartbeat_node → /cargo_avoidance/status_code
```

## 吊物避障安全协议

### 安全码

| Code | 含义 | 授权来源 |
|---:|---|---|
| 14 | CLEAR — 无碰撞风险 | 仅 Formal Geometry + 全部合同满足 |
| 17 | NEAR_3M — ≤3m，垂直净空<0.8m | FORMAL 或 POSITIVE_ONLY |
| 18 | NEAR_5M — 3-5m，垂直净空<0.8m | FORMAL 或 POSITIVE_ONLY |
| 30 | 系统未就绪 / 时间轴回退 | 故障 |
| 31 | 定位无效 | 故障 |
| 32 | Gravity/称重信号无效 | 故障 |
| 33 | 吊物证据无效 | 故障 |
| 34 | 障碍证据不足 | 故障 |
| 35 | 内部合同错误 | 故障 |

### 几何权限

| 操作 | PENDING | POSITIVE_ONLY | FORMAL |
|---|---|---|---|
| 正向 17/18 告警 | 仅 `evidence_backed_only` 全门禁通过时允许 | 允许（障碍证据也须通过） | 允许 |
| 无危险时输出 | 33 | 33 | 14（全部合同满足） |
| 货物点从 registration 剔除 | 禁止 | 禁止 | 允许 |
| 静态地图/MapCommit 排除 | 禁止 | 禁止 | 允许 |

起吊前基线只在 `EMPTY + 静止 + 定位可靠` 时从同代静态高度场建立，记录顶面、支撑面、
OBB、成员网格和 MAD；不使用 odom Z 推导厚度。地图代次变化、时间回退或锚点与组件
距离超过 0.5m 会使该证据失效，避免 NDT 漂移把另一处静态物体绑定为本次货物。

### Heartbeat 节点

`cargo_alarm_heartbeat_node` 对类型化 `CargoSafetyStatus` 做协议校验，接受前进时间戳的新状态，以 5Hz 重发当前安全码。重复时间戳不产生新证据。时序确认由上游安全/障碍追踪管线负责。

## 构建

Ubuntu / ROS Noetic：

```bash
cd ~/NDT-slam-ws
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin clean -y
catkin build --no-status
source devel/setup.bash
```

Windows 仅用于源码编辑和静态合同检查，不能替代 ROS/PCL/Sophus 编译与 bag 验收。

## 启动

手动调试/验收（真实传感器时间、带 RViz 和看门狗）：

```bash
cd ~/NDT-slam-ws
source devel/setup.bash
export NDT_SLAM_DATA_ROOT="$PWD/maps/live/current"
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=false use_rviz:=true persistent_map:=true \
  use_ndt_recovery_watchdog:=true
```

手动运行没有外部进程监督；看门狗请求硬恢复时会安全结束 launch，但不会自行重拉。
现阶段 systemd 模式暂时停用，不执行 unit 安装、enable 或 start。监控使用：

```bash
rosrun ndt_slam server_monitorctl.sh start --follow \
  --workspace ~/NDT-slam-ws
# 另一个终端停止后台监控：
rosrun ndt_slam server_monitorctl.sh stop \
  --workspace ~/NDT-slam-ws
```

服务器验收统一入口：

```bash
rosrun ndt_slam run_server_validation.sh prepare \
  --workspace ~/NDT-slam-ws --expected-sha <SHA> --run-id rc1-live-001
rosrun ndt_slam run_server_validation.sh start \
  --workspace ~/NDT-slam-ws --expected-sha <SHA> --run-id rc1-live-001
```

Bag 验收（仿真时间）：

```bash
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=true use_rviz:=true persistent_map:=false
rosbag play /path/to/warehouse.bag --clock
```

该 launch 默认启动 SLAM、NDT 恢复看门狗和 RViz。RViz 中全量
`display_map` 默认关闭以避免大点云拖慢界面，需要时可手工开启；这不会关闭
`objects_clean` 等运行显示。无 RViz 运行时显式传入 `use_rviz:=false`。
完整安装、有效配置检查和故障恢复命令见[部署](src/ndt_slam/doc/deployment.md)与
[运行与运维](src/ndt_slam/doc/operations.md)。

## 主要话题与接口

| Topic | 类型 | 说明 |
|---|---|---|
| `/odom` | `nav_msgs::Odometry` | 运行位姿 |
| `/ndt_slam/runtime_path` | `nav_msgs::Path` | 实时轨迹 |
| `/merged_points` | `sensor_msgs::PointCloud2` | 合并后当前帧点云 |
| `/map` | `sensor_msgs::PointCloud2` | registration 层 |
| `/display_map` | `sensor_msgs::PointCloud2` | 全量显示层 |
| `/display_map_ground` | `sensor_msgs::PointCloud2` | 地面层 |
| `/display_map_objects` | `sensor_msgs::PointCloud2` | 原始静态物体层 |
| `/display_map_objects_clean` | `sensor_msgs::PointCloud2` | 清理后静态物体层 |
| `/cargo_core_bbox_marker` | `visualization_msgs::Marker` | 正式冻结形状吊物框 |
| `/cargo_tight_box_marker` | `visualization_msgs::Marker` | 兼容框（相同刚体几何） |
| `/cargo_warning_zone_marker` | `visualization_msgs::Marker` | 3m/5m 方向一致告警区域 |
| `/cargo_avoidance/safety_status` | `lidar_slam2_msgs/CargoSafetyStatus` | 正式安全输出（主控必须订阅） |
| `/cargo_avoidance/status_code` | `std_msgs/Int32` | Heartbeat 简码输出 |

完整接口文档见 [对外接口](src/ndt_slam/doc/api.md)。

## 现场验证基线

验证 Tag：[`validation-obstacle-avoidance-20260728`](https://github.com/guolichen007/NDT-SLAM-Warehouse/tree/validation-obstacle-avoidance-20260728) → `8d7d7ee`

已取得现场证据：
- SLAM 侧：已观测 Code 17/18 正向安全告警（235×18, 53×17, 24 独立避障片段）
- 外部主控侧：已观测 Code 18 接收并触发 S3 语音告警（243×Code18, 224×S3, <1s 延迟）

详细证据：[避障端到端现场验证](docs/validation/obstacle_avoidance_e2e_20260727_20260728.md)

2026-07-29/30 的长时间运行数据已完成证据边界审查。它补充证明历史链路持续输出
17/18，但因版本原因串与 `f57d68a` 不一致、SLAM 与主控数据不同步，不能作为
`f57d68a` 新增路径的现场验收结论。详见
[避障运行证据审查](docs/validation/obstacle_avoidance_runtime_evidence_review_20260729_20260730.md)。

## 文档导航

**技术文档（当前 master 参考）：**
- [系统架构](src/ndt_slam/doc/architecture.md)
- [对外接口](src/ndt_slam/doc/api.md)
- [定位运行时](src/ndt_slam/doc/localization_runtime.md)
- [吊物跟踪与安全](src/ndt_slam/doc/cargo_tracking_and_safety.md)
- [地图生命周期](src/ndt_slam/doc/map_lifecycle.md)
- [长期在线建图](src/ndt_slam/doc/longterm_mapping.md)
- [配置说明](src/ndt_slam/doc/configuration.md)
- [部署](src/ndt_slam/doc/deployment.md)
- [运行与运维](src/ndt_slam/doc/operations.md)
- [服务器监控](src/ndt_slam/doc/server_monitoring.md)
- [服务器验收 Runbook](src/ndt_slam/doc/server_validation_runbook.md)
- [测试与验收](src/ndt_slam/doc/testing_and_acceptance.md)
- [故障排查](src/ndt_slam/doc/troubleshooting.md)

**项目管理：**
- [项目状态](docs/project/status.md)
- [开发路线](docs/project/roadmap.md)
- [已知问题](docs/project/known_issues.md)
- [发布流程](docs/project/release_process.md)

## 许可证与安全说明

MIT License。详见 [LICENSE](LICENSE)。

运行安全声明见 [SAFETY.md](SAFETY.md)，软件安全策略见 [SECURITY.md](SECURITY.md)。
