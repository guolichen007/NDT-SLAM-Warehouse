# 系统架构

## 运行链路

```text
双雷达 -> pointcloud_merger -> /merged_points
  -> 近场/人体/吊物候选过滤
  -> 结构保持 Registration Source
  -> NDT + Observability -> 各向异性 EKF
  -> StationaryMotionPolicy -> odom / TF / runtime_path
  -> MapCommit -> raw layer snapshot -> Clean Worker -> MapLayerBundle

同一帧 LiDAR -> Cargo observation -> Hook Cargo Lock
  -> LockedCargoShape + LiveCargoPose
  -> Cargo Bottom -> Cargo Safety 14/17/18 或 30-35
  -> Heartbeat（状态所有者）
```

## 定位职责边界

- `RegistrationCloudBuilder` 优先保留竖直静态结构、未授权候选和限额地面，不允许 full-ground fallback。
- `NdtObservability` 使用结构法向构造二维信息代理；它不是 NDT Hessian。强弱方向先旋转到 map/EKF 坐标系，再生成各向异性测量协方差。
- `CraneMotionEKF` 负责预测、NDT 更新和静止伪测量。
- `StationaryMotionPolicy` 独立决定运行位姿保持、真实移动确认、CATCH_UP，以及 local map 与持久地图写入权限。

## 吊物职责边界

- `LockedCargoShape`：确认后冻结长、宽、高、轴向 yaw。
- `LiveCargoPose`：保存中心、真实证据时间、计算时间、来源和位置不确定度。
- `RigidCargoGeometry`：生成 base/map 八角点，并作为 marker、Cargo Bottom、安全距离、自体点剔除和 MapCommit 的唯一正式几何。
- `CargoMarkerLifecycle`：显示生命周期；显示保持不等于正式安全证据有效。
- `CargoSafetyEvaluator`：只根据空间碰撞关系产生 14/17/18，证据故障产生 30-35。
- `cargo_alarm_heartbeat_node`：维护输出状态；重复时间戳和 heartbeat tick 不产生新证据。

## 地图职责边界

运行地图是可变工作区；正式发布地图是不可变 `MapLayerBundle`。Clean Worker 从 raw bundle N 构建 clean N，完成后一次性发布五层 N。工作地图已前进到 N+1 时，完整 N 仍可发布，但其 clean 层不会反向覆盖当前工作地图。

## 时间合同

LiDAR、Gravity、Cargo Bottom 和 Safety 都以源时间戳判定证据是否前进。时间回退帧立即故障闭锁并建立新 epoch；后续新时间轴可恢复。显示时间、证据时间和评估时间不得互相冒充。
