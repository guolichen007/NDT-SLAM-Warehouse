# NDT-SLAM-Warehouse

室内仓库 / 天车场景的 NDT-SLAM 定位、建图与吊物可视化工程。

当前主线版本目标：

- 保持 A7 风格平滑轨迹链路；
- 输出稳定 `odom` / `TF` / `runtime_path`；
- 使用 `OdomAnchorBox` 在 `base_link` / odom 锚点显示绿色吊物框；
- 货物框中心固定在 `base_link` 坐标系下的机械锚点；
- 点云检测只更新货物框尺寸和高度；
- 默认不启用货物移除、动态擦除和避障报警。

---

## 当前主线功能状态

| 功能 | 状态 | 说明 |
|---|---|---|
| NDT 定位 | ✅ 默认启用 | 输出 `odom` / `TF` |
| CraneMotionEKF | ✅ 默认启用 | 合并验收 `recovery=0` |
| runtime_path | ✅ 默认启用 | A7 风格轨迹显示 |
| display_map | ✅ 默认启用 | 建图与显示层 |
| OdomAnchorBox | ✅ 默认启用 | 绿色框锁定 `base_link` / odom anchor |
| size / height 自适应 | ✅ 默认启用 | 由 anchor 附近点云估计 |
| cargo debug 点云 | ❌ 默认关闭 | 调试时手动打开 |
| HookCargoRemoval | ❌ 默认关闭 | 后续单独验证 |
| cargo 避障报警 | 🚧 未启用 | 后续接入 14 / 17 / 18 预警 |
| 重定位 | 🚧 独立开发 | 不属于当前 cargo 主线内容 |

---

## 快速启动

### 1. 编译

```bash
cd ~/NDT-slam-ws
catkin_make --pkg ndt_slam
source devel/setup.bash
```

### 2. 启动定位 / 建图 / OdomAnchorBox 显示

```bash
rosnode kill -a || true
pkill -f ndt_slam_node || true
pkill -f cargo_forbidden_zone_node || true
pkill -f pointcloud_merger || true

rosparam set /use_sim_time true

roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=true \
  persistent_map:=false \
  odom_anchored_cargo_box_enabled:=true \
  hook_cargo_removal_enabled:=false \
  use_cargo_visualizer:=true \
  ndt_publish_cargo_markers:=false \
  publish_cargo_debug_points:=false
```

### 3. 播放 bag

```bash
rosbag play /home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag --clock
```

---

## 关键配置

配置文件：

```bash
src/ndt_slam/config/live_longterm_mapping.yaml
```

核心配置：

```yaml
odom_anchored_cargo_box:
  enabled: true
  anchor_x: 0.0
  anchor_y: 0.0
  detect_rate_hz: 5.0
  marker_rate_hz: 5.0

  publish_debug_points: false
  publish_selected_core_points: false
  publish_raw_candidate_points: false
  publish_default_box_marker: false

  verbose_debug: false

  use_global_payload_tracker: false
  use_cargobox_v2: false
  use_dynamic_history_eraser: false
  enable_hook_cargo_removal: false
```

说明：

- `anchor_x / anchor_y` 是绿色框在 `base_link` 坐标系下的固定机械锚点；
- 默认 `(0.0, 0.0)` 表示框中心锁在 `base_link` 原点；
- 如果现场确认吊钩相对 `base_link` 有固定机械偏移，只修改 `anchor_x / anchor_y`；
- 点云检测不能修改框中心，只能更新尺寸和高度；
- `HookCargoRemoval` 默认关闭，避免影响 NDT 定位链路；
- `publish_debug_points` 默认关闭，避免 RViz 和终端卡顿。

---

## 主要 Topic

| Topic | 说明 |
|---|---|
| `/odom` | 定位结果 |
| `/tf` | `map` / `odom` / `base_link` 变换 |
| `/ndt_slam/runtime_path` | 运行轨迹 |
| `/payload_track_info` | OdomAnchorBox bbox 数据 |
| `/cargo_core_bbox_marker` | 绿色货物框 |
| `/display_map` | 显示地图 |
| `/merged_points` | 输入点云或当前点云 |

---

## RViz 推荐显示

默认打开：

- `/ndt_slam/runtime_path`
- `/cargo_core_bbox_marker`
- `/merged_points` 或当前点云
- `/odom`

调试时才打开：

- `/cargo_selected_core_points`
- `/hook_raw_candidate_points`
- `/hook_default_box_marker`

默认不要打开过多 display/debug 点云，否则 RViz 容易卡顿。

---

## 主线验收

### Baseline：cargo 全关

```bash
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=true \
  persistent_map:=false \
  odom_anchored_cargo_box_enabled:=false \
  hook_cargo_removal_enabled:=false \
  use_cargo_visualizer:=false \
  ndt_publish_cargo_markers:=false \
  2>&1 | tee /tmp/a7_baseline_final.log
```

验收：

```bash
grep "CraneMotionEKF.*recovery" /tmp/a7_baseline_final.log | wc -l
grep "NDTHealth" /tmp/a7_baseline_final.log | tail -30
grep "\[ERROR\]" /tmp/a7_baseline_final.log | head -20
```

通过标准：

- `recovery = 0`
- 无 `[ERROR]`
- NDT fitness 不持续升高
- `runtime_path` 平滑

---

### Display-only：只开 OdomAnchorBox

```bash
roslaunch ndt_slam warehouse_live_longterm_mapping.launch \
  use_sim_time:=true \
  persistent_map:=false \
  odom_anchored_cargo_box_enabled:=true \
  hook_cargo_removal_enabled:=false \
  use_cargo_visualizer:=true \
  ndt_publish_cargo_markers:=false \
  publish_cargo_debug_points:=false \
  2>&1 | tee /tmp/a7_odom_anchor_final.log
```

验收：

```bash
LOG=/tmp/a7_odom_anchor_final.log

grep "OdomAnchorBoxConfig\|OdomAnchorSummary\|CargoLock" "$LOG" | head -120
grep "CraneMotionEKF.*recovery" "$LOG" | wc -l
grep "\[ERROR\]" "$LOG" | head -20
```

通过标准：

- `recovery = 0`
- 无 `[ERROR]`
- `OdomAnchorSummary` 正常输出
- `CargoLock` 能进入 `LOCKED`
- 绿色框跟随 `odom` / `base_link`

---

## 当前版本不包含

当前主线版本暂不默认启用：

- HookCargoRemoval；
- 动态货物移除；
- cargo volume 动态擦除；
- cargo 避障报警 14 / 17 / 18；
- 全局重定位流程。

这些功能需要后续独立验证后再进入主线。

---

## 常见问题

### 1. 绿色框为什么不跟检测簇中心走？

当前主线采用 `OdomAnchorBox`。绿色框中心固定在 `base_link` 坐标系下的机械锚点，点云检测只负责更新尺寸和高度。这样可以避免货物框被旁边点云簇、吊具点、局部噪声带偏。

### 2. 现场吊钩不在 base_link 原点怎么办？

只修改：

```yaml
odom_anchored_cargo_box:
  anchor_x: <机械固定偏移 x>
  anchor_y: <机械固定偏移 y>
```

不要用检测点中心修改 anchor。

### 3. 为什么默认关闭 HookCargoRemoval？

当前阶段优先保证定位轨迹稳定。HookCargoRemoval 会影响 NDT 输入点云，必须单独分支验证，不在当前主线默认开启。

### 4. 终端为什么只看到少量 OdomAnchorSummary？

高频 cargo 调试日志已降级为 DEBUG。默认只输出状态变化和 summary，避免 rosconsole 和 RViz 卡顿。

---

## 依赖

- Ubuntu 20.04 + ROS Noetic
- PCL, Eigen3, Sophus, yaml-cpp, g2o, ndt_omp, TBB

---

## 许可证

MIT License
