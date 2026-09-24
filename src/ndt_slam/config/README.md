# ndt_slam 配置说明（ROS1 Noetic）

本目录存放 `ndt_slam` ROS1 包的全部 YAML 配置。运行命令使用 `roslaunch` /
`rosparam` / `rosrun`，不使用 ROS2 命令。

## 配置文件索引

| 文件 | 用途 | 是否生产使用 |
|---|---|---|
| `live_longterm_mapping.yaml` | 长期在线建图 + 吊物安全 + 定位主配置 | 是（生产） |
| `merger_params.yaml` | 双雷达同步、外参、合并 | 是（生产） |
| `server_monitor.yaml` | 服务器监控/自愈 | 是（生产） |
| `engineering_mapping.yaml` | 工程建图（离线/后处理） | 是（工程） |
| `map_postprocess.yaml` | 地图后处理 | 是（工程） |
| `cargo_forbidden_zone.yaml` | 货物禁入区 | 专项 |
| `cargo_vertical_evidence_v2.yaml` | 货物垂直证据（shadow） | 专项/诊断 |
| `integrated_cargo_identity_shadow_v1.yaml` | 集成货物身份 shadow | 专项/诊断 |
| `slam_params.yaml` | 兼容/基础 SLAM 参数 | 兼容 |

## 运行方式

```bash
# 长期在线建图与吊物安全
roslaunch ndt_slam warehouse_live_longterm_mapping.launch

# 运行时定位
roslaunch ndt_slam warehouse_runtime.launch
```

## 参数查看与覆盖

```bash
# 查看全部参数
rosparam list

# 读取单个参数
rosparam get /ndt_slam_node/param_name

# 运行时覆盖（临时）
rosparam set /ndt_slam_node/param_name value
```

## 生产 Yaw 模式

`live_longterm_mapping.yaml` 中：

```yaml
runtime_yaw_authority:
  mode: "LEGACY"      # LEGACY | SHADOW | RAIL_AUTHORITY
  reference:
    verified: false
```

生产保持 `LEGACY`，`verified=false`。现场 rail yaw 正式测量与 commissioning
完成前，不得设置 `verified=true`，不得切换 `SHADOW` / `RAIL_AUTHORITY`。

## 双雷达外参

`merger_params.yaml` 中的 `Lidar2BaseExtrinsic` 是双雷达外参（标定身份）。
`voxel_size` / 同步窗口 / 队列等运行参数不属于标定身份，调整运行参数不会改变
`sensor_rig_calibration_id`。

## 详细说明

- 配置项完整说明见 [configuration.md](../doc/configuration.md)。
- 部署说明见 [deployment.md](../doc/deployment.md)。
- 服务器运维见 [operations.md](../doc/operations.md)。
