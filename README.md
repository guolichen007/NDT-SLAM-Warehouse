# NDT-SLAM Warehouse

面向室内仓库和天车作业的 ROS1 NDT 定位、长期建图、吊物跟踪与避障工程。

当前生产链路同时维护定位、五层地图和一条正式吊物安全协议。吊物一旦确认，系统冻结稳健二维定向包围框的长、宽、方向和高度；作业期间只更新刚体中心，因此框会随平移与起升实时移动，不会因单帧稀疏点云改变形状。

## 核心行为

- 定位：结构优先的 NDT 输入，结构不足时进入 EKF prediction-only，不回退到整片地面。
- 静止保持：`STATIONARY_HOLD -> MOVING_CONFIRM -> CATCH_UP -> MOVING`，随机累计漂移不能直接解除静止状态。
- 吊物几何：同一份 `LockedCargoShape + LiveCargoPose` 同时服务于 RViz、Cargo Bottom、避障、自体点剔除、NDT 输入和 MapCommit。
- 生命周期：`EMPTY -> CANDIDATE -> LOCKED -> LOST_HOLD -> EMPTY`。`LOST_HOLD` 保留最近可信框并增加不确定度，不生成新 track id。
- 正式安全码：`14` 为 CLEAR；`17` 为 3 m 内且垂直净空小于 0.8 m；`18` 为 3–5 m 且垂直净空小于 0.8 m；`30–35` 为系统或证据故障。
- Gravity：输入话题统一为 `/gravity`。`AUXILIARY` 模式下 LiDAR 是主信号，Gravity 不可用不能永久阻断紧凑货物检测。
- 地图：同次发布的 registration/display/ground/objects/objects_clean 使用同一内容代次；空层也会发布同代空消息，避免 RViz 保留旧层。
- 控制台：生产默认只显示吊物状态、安全码变化和不可忽略的运行时错误；逐帧定位、地图和性能数据继续写入 CSV。

## 构建

Ubuntu / ROS Noetic：

```bash
cd ~/NDT-slam-ws
catkin_make --pkg ndt_slam
source devel/setup.bash
```

Windows 只用于源码修改和静态合同检查，不能替代 ROS/PCL/Sophus 编译与 bag 验收。

## 启动

```bash
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=true \
  persistent_map:=false
```

随后播放带 `/clock` 的 bag：

```bash
rosbag play /path/to/warehouse.bag --clock
```

正式配置位于：

```text
src/ndt_slam/config/live_longterm_mapping.yaml
src/ndt_slam/config/merger_params.yaml
```

## 吊物框合同

检测阶段从货物点计算稳健二维 OBB：

1. 过滤非有限点并计算中心化协方差；
2. 以主特征向量得到轴向 yaw（`yaw` 与 `yaw + pi` 等价）；
3. 使用 P08/P92 投影范围抑制离群点；
4. 同时检查几何长宽比、特征值比和多帧方向集中度；
5. 达到确认帧数后冻结 `length/width/height/yaw`。

锁定后：

- `LiveCargoPose.center_base` 由当前 LiDAR 观测做有界滤波更新；
- 起升只改变中心 Z，不因绝对 bottom/top 变化而错误进入 LOST；
- 正式 marker 和旧兼容 marker 都使用同一 map-frame yaw；
- Cargo Bottom 在 OBB 局部坐标中统计支撑点、跨度和网格覆盖；
- 避障距离按点到旋转矩形的真实二维距离计算；
- 正式货物点从 registration/MapCommit 中按同一 OBB 剔除。

若货物近似正方形或方向证据不稳定，系统不会伪造一个方向；它会继续保持候选或已有冻结方向。

## 安全协议

输入：

```text
/cargo_avoidance/safety_status
```

输出：

```text
/cargo_avoidance/status_code
```

状态定义：

| Code | 含义 |
|---:|---|
| 14 | 无碰撞风险；无障碍时障碍几何允许为 NaN |
| 17 | 障碍距吊物 OBB 不超过 3 m，且垂直净空小于 0.8 m |
| 18 | 障碍距吊物 OBB 大于 3 m、不超过 5 m，且垂直净空小于 0.8 m |
| 30 | 系统未就绪、状态超时或源时间轴回退帧 |
| 31 | 定位无效 |
| 32 | 吊物几何无效 |
| 33 | 吊物底部证据无效 |
| 34 | 障碍证据不足或无效 |
| 35 | 内部错误 |

新鲜且时间戳前进的正式状态立即生效；重复时间戳不能推进任何状态，heartbeat 只重发当前码。时间戳回退帧输出 30 并建立新 epoch，下一条前进时间戳可恢复。

## 主要 Topic

| Topic | 内容 |
|---|---|
| `/odom` | 运行位姿 |
| `/ndt_slam/runtime_path` | 实时轨迹 |
| `/merged_points` | 合并后的当前帧点云 |
| `/map` | registration 层 |
| `/display_map` | 全量显示层 |
| `/display_map_ground` | 地面层 |
| `/display_map_objects` | 原始静态物体层 |
| `/display_map_objects_clean` | 清理后的静态物体层 |
| `/cargo_core_bbox_marker` | 正式冻结形状、实时移动的吊物框 |
| `/cargo_tight_box_marker` | 使用相同刚体几何的兼容框 |
| `/cargo_warning_zone_marker` | 与吊物方向一致的 3 m / 5 m 区域 |
| `/cargo_avoidance/status_code` | 14/17/18/30–35 安全码 |

RViz 的 Fixed Frame 使用 `map`。若只看到当前帧点云，应先检查五个地图 topic 是否均在发布以及同一时刻的 `header.seq` 是否一致，而不是修改 RViz 左侧显示配置。

## 诊断

生产配置默认：

```yaml
logging:
  debug_perf: false
  summary_interval_sec: 10.0

debug:
  runtime_diagnostics:
    enabled: true
    console_health_enabled: false
    console_risk_enabled: false
    cargo_console_enabled: true
    csv_enabled: true
```

终端保留：

- `CargoLock` / `CARGO_HEALTH`；
- `SAFETY_WARN` / `SAFETY_FAULT` 以及安全码或 reason 变化；
- `SO3Guard` 失败、非有限 NDT、时间 epoch 重置和节点级错误。

逐帧性能、可观测性、registration 模式、地图门控和 pipeline 风险写入配置的 diagnostics 目录，不靠高频控制台输出做验收。

## 静态检查

```bash
git diff --check
python scripts/regression/check_repository_integrity.py
python scripts/regression/check_cargo_safety_e2e.py
```

Ubuntu 还必须按顺序执行 clean build、gtest、静止漂移 bag、真实移动 catch-up、吊物起升/平移、17/18/14 空间合同和第二次 bag 时间 epoch 回退测试。

## 验收重点

- 横向实际货物的框长轴应与点云长轴一致，而不是固定沿 map/base 轴。
- LOCKED 后长宽、高度、yaw 保持不变，中心随吊物连续移动。
- 起升过程中不能因 bottom Z 变化进入 LOST。
- 短时点云破碎不能直接解释为明确无货，也不能产生错误 CLEAR。
- 只有真实空间碰撞风险输出 17/18；定位、Gravity、证据质量问题只能输出 30–35。
- 五层地图在 RViz 中持续可见，且同次发布 `header.seq` 一致。
