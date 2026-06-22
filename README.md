# NDT-SLAM 工程化建图系统

基于 NDT_OMP 的激光雷达 SLAM 系统，面向起重机仓库场景，支持多次运行 bag 文件进行迭代优化，最终生成高质量的多层地图。

## 当前开发状态（feature/placed-cargo-suppressor 分支）

### 已完成功能

#### 1. 人体动态过滤 (HumanObjectDynamicFilter)
- 从 objects_cloud 中检测小型地面动态人体点云
- 先 BEV 聚类，再判断聚类是否符合人体特征
- 支持 map 坐标系下动态确认
- 从 NDT 输入、地图插入、关键帧保存、rebuild 中统一剔除人体点

#### 2. 吊货动态过滤 (BasePayloadChannelFilter + PayloadTrackManager)
- BasePayloadChannelFilter：base_link 下中间通道吊货候选筛选
- PayloadTrackManager：双坐标系吊货跟踪，确认 DYNAMIC_PAYLOAD
- 支持 swept volume 生成

#### 3. 动态事件管理器 (DynamicEventManager)
- 统一管理吊货会话和人体轨迹事件
- PayloadSession 状态机：CANDIDATE → CARRIED_MOVING → CARRIED_STOPPING → PLACED_STATIC → CLOSED
- PayloadSession 去重合并：同 track_id 或 bbox IoU > 0.2 不重复创建
- FinalPlacementProtector：连续稳定 5 帧、velocity < 0.04、map_disp < 0.15 判定为停放
- CleanMap protect mask 优先于 deny mask：停放货物不被删除

#### 4. 地图重建 (rebuildGlobalMapFiltered)
- 从 filtered keyframes + optimized poses 重建所有地图层
- dirty_dynamic 的 keyframe 会重新过滤
- 应用 dynamic event mask
- 输出详细的重建日志

#### 5. CleanMap Dynamic Deny Gate
- CleanMap 重建时跳过动态事件覆盖的 BEV cell
- 支持 static_protect_mask 优先级
- 输出拒绝统计日志

#### 6. 扩展 rebuild_map 输出
- rebuild_map 服务输出 11 个 PCD 文件
- 包括：registration、display、ground、objects_raw、objects_filtered、payload_candidate、human_candidate、human_dynamic、human_pending、ground_raw、display_full

#### 7. PlacedCargoSuppressor（停放货物抑制器）
- PLACED_STATIC 后，protect bbox 不仅保护 CleanMap，还反向约束 PayloadTrackManager 和 PayloadSessionManager
- 禁止在停放区域重新创建动态会话
- 增强调试日志：输出 centroid、bbox 和 placed_sessions 数量

#### 8. 分层地图策略
- moving payload 不进任何地图
- placed cargo 可进 objects_clean 和 display_map
- rebuildCleanMap：通过 protect_cells 保护停放货物点
- rebuildDisplayMap：将 placed cargo 的 payload_candidate 点添加到 display_map

#### 9. save_map 确认日志
- 保存地图时输出 placed sessions 信息
- 检查 objects_clean_map 和 display_map 中的 placed cargo 点数
- 输出 mask 使用确认日志

### 验证数据（使用调运长件.bag 测试）

#### 测试环境
- Bag 文件：`/home/ydkj/AutoCraneSlam-ROS1/bag/调运长件.bag`
- Bag 时长：119.85 秒
- 关键帧数量：80

#### 测试结果
```
[SaveMapMaskConfirm] dynamic_events enabled: placed=0, active=3
[SaveMapMaskConfirm] display_map contains 0 placed cargo points out of 270680 total

地图输出：
- map_registration.pcd: 27135 points
- map_display.pcd: 270680 points
- map_ground.pcd: 94746 points
- map_objects_raw.pcd: 146066 points
- map_objects_filtered.pcd: 255184 points
- map_payload_candidate.pcd: 55428 points
- map_human_candidate.pcd: 13901 points
- map_human_dynamic.pcd: 241 points
- map_human_pending.pcd: 12705 points
- map_ground_raw.pcd: 1086316 points
- map_display_full.pcd: 1341500 points
```

#### PayloadSession 状态转换
```
[PayloadSession] create id=0, track=0
[PayloadSession] id=0 -> CARRIED_MOVING
[PayloadSession] id=0 -> CARRIED_STOPPING

PlacementDebug:
- vel=0.053, disp=0.080, stable=3 (未满足停放条件)
- vel=0.143, disp=0.219, stable=2
- vel=0.214, disp=0.333, stable=0
```

#### 测试结论
- ✅ 代码编译成功
- ✅ 系统正常运行
- ✅ PayloadSession 正确创建和更新
- ✅ 分层地图策略已实现（代码逻辑正确）
- ✅ save_map 确认日志已添加
- ⚠️ 测试 bag 中没有停放事件（货物一直在移动），无法完全验证 PlacedCargoSuppressor

### 待验证

1. **使用包含停放事件的 bag 验证**：需要货物被放下并静止的场景
2. **验证 PlacedCargoSuppressor**：确保停放货物不会被重新识别为新的动态吊货
3. **验证分层地图**：确认 placed cargo 点正确进入 objects_clean 和 display_map

---

## 长期建图功能（feature/longterm-mapping 分支）

### 功能概述

支持雷达常开、天车静止时不重复建图、天车移动时增量落盘、内存和硬盘都有上限保护。

### 已完成功能

#### 1. MotionGate（地图提交门控）
- 静止检测：`min_translation_m: 0.30`，`min_rotation_deg: 3.0`
- 时间间隔：`min_time_between_keyframes_sec: 2.0`
- 静止时不添加关键帧，移动时才提交

#### 2. 关键帧 active window
- 最大活跃关键帧数：`max_active_keyframes: 80`
- 定期释放旧关键帧的点云，保留 metadata

#### 3. 磁盘增量保存（tile）
- tile 大小：`tile_size_m: 20.0`
- flush 间隔：`flush_interval_sec: 60`
- 先写临时文件再重命名（防断电损坏）

#### 4. runtime status
- 每 5 秒写入 `runtime_status.json`
- 包含帧数、关键帧、内存、磁盘等信息

### 新增文件

| 文件 | 说明 |
|------|------|
| `launch/warehouse_live_longterm_mapping.launch` | 长期运行 launch |
| `config/live_longterm_mapping.yaml` | 长期运行全部参数 |
| `scripts/monitor_longterm.sh` | 过夜监控脚本 |

### 验证数据（使用调运长件.bag 测试）

#### 测试环境
- Bag 文件：`/home/ydkj/AutoCraneSlam-ROS1/bag/调运长件.bag`
- Bag 时长：119.85 秒

#### 测试结果
```
runtime_status.json:
{
  "total_frames": 759,
  "total_keyframes": 27,
  "active_keyframes": 27,
  "is_stationary": false,
  "delta_translation_m": 0.01,
  "delta_yaw_deg": 0.04,
  "active_map_points": 9147,
  "dirty_tile_count": 1,
  "flushed_tile_count": 2,
  "disk_free_gb": 70.48,
  "memory_mb": 123.93,
  "average_process_time_ms": 150.77
}

tiles_registration 目录：
- x0_y0.pcd (34KB)
- x0_y-1.pcd (716KB)
```

#### 测试结论
- ✅ MotionGate 生效（27 个关键帧，而非 80+）
- ✅ tiles 写入正常（2 个 tile 文件）
- ✅ runtime_status.json 正常更新
- ✅ 内存稳定（124MB）
- ✅ 磁盘增量保存正常

### 使用方法

```bash
# 启动长期建图模式
roslaunch ndt_slam warehouse_live_longterm_mapping.launch

# 使用 bag 测试（use_sim_time=true）
roslaunch ndt_slam warehouse_live_longterm_mapping.launch use_sim_time:=true

# 监控运行状态
bash src/ndt_slam/scripts/monitor_longterm.sh 60

# 查看 runtime_status.json
cat maps/live/current/runtime_status.json | python3 -m json.tool
```

---

## 项目背景

起重机仓库场景，传感器安装在约 8m 高度俯视金属物料堆场。该场景下 KISS-ICP 完全退化（地面点占比 80-88%，导致平面退化），因此采用 NDT_OMP + 特征点加权的混合策略，配合工程化的多轮迭代建图流程。

---

## 当前架构

### 代码结构

```
src/ndt_slam/
├── src/                           # 源文件（11 个 cpp）
│   ├── main.cpp                   # 主入口
│   ├── ndt_slam.cpp               # 主 SLAM 节点（NDT 配准、位姿估计、地图管理）
│   ├── odometry.cpp               # 里程计（KISS-ICP + ScanMatcher）
│   ├── mapping.cpp                # 建图（关键帧管理、地图生成）
│   ├── loop_closure.cpp           # 闭环检测（ScanContext + PoseGraph）
│   ├── point_cloud_processing.cpp # 点云处理（地面分割、滤波）
│   ├── base_payload_channel_filter.cpp # 通道吊货候选筛选（base_link 下）
│   ├── payload_tracker.cpp        # 吊货双坐标系跟踪管理
│   ├── visualizer.cpp             # OpenGL 可视化
│   ├── PointCloudMerger.cpp       # 双雷达点云合并（独立可执行）
│   └── CloudDiagnostics.cpp       # 点云诊断工具（独立可执行）
│
├── include/ndt_slam/              # 头文件（8 个 hpp）
│   ├── ndt_slam.hpp
│   ├── odometry.hpp
│   ├── mapping.hpp
│   ├── loop_closure.hpp
│   ├── point_cloud_processing.hpp
│   ├── base_payload_channel_filter.hpp
│   ├── payload_tracker.hpp
│   └── visualizer.hpp
│
├── config/                        # 配置文件
│   ├── dual_lidar_slam_params.yaml  # SLAM 主配置
│   ├── engineering_mapping.yaml     # 工程化建图配置
│   ├── map_postprocess.yaml         # 地图后处理配置
│   └── merger_params.yaml           # 双雷达合并配置
│
├── scripts/                       # 工具脚本
│   ├── analyze_map_quality.py       # 地图质量分析
│   ├── build_ground_clean.py        # 干净地面生成
│   ├── build_objects_clean.py       # 干净物体层生成
│   ├── build_localization_map.py    # 精定位地图生成
│   ├── build_navigation_grid.py     # 导航栅格生成
│   ├── run_map_postprocess.sh       # 一键后处理
│   ├── map_version_manager.py       # 地图版本管理
│   └── continue_mapping.sh          # 继续建图脚本
│
└── launch/                        # Launch 文件
    ├── warehouse_runtime.launch     # 仓库运行模式（定位）
    ├── dual_lidar_slam.launch       # 双雷达建图模式
    └── offline_mapping.launch       # 离线建图模式
```

### 配准流程

```
实时定位链路：
  merged_points → 近场过滤 → 预处理 → 地面/物体分割
  → NDT_OMP 配准（1.0m 分辨率）→ 位姿估计 → 发布 odom/TF

后台精配准：
  ICP 精配准（异步）→ 修正关键帧位姿 → 用于地图插入
```

### 地图分层

| 地图类型 | 文件名 | 体素 | 用途 |
|---------|--------|------|------|
| 配准地图 | registration_map.pcd | 0.3m | NDT 配准用粗地图 |
| 显示地图 | display_map.pcd | 0.1m | 全量显示，保留货物轮廓 |
| 地面地图 | ground_map.pcd | 0.15m | 地面分割结果 |
| 物体地图 | objects_raw.pcd | 0.06m | 非地面/货物原始层 |
| 物体干净版 | objects_clean.pcd | 0.06m | 经 BEV+时间一致性过滤 |
| 地面干净版 | ground_map_clean.pcd | 0.08m | 后处理生成的干净地面 |
| 精定位图 | localization_map_fine.pcd | 0.10m | 后处理生成的精定位地图 |
| 导航栅格 | navigation_grid_0.05m.pgm | 0.05m | 2D 导航占用栅格 |

### 地面分割算法

使用 XY 网格局部地面模型，处理倾斜地面和局部高度变化：

```
1. 将点云按 XY 坐标分入 1.5m×1.5m 网格
2. 每个格子内，取 z 值第 20 百分位作为局部地面参考
3. 点的 z 高于局部地面 + 0.45m → objects
4. 点的 z 低于局部地面 + 0.45m → ground
5. 对 objects 层做自适应 SOR 过滤
6. 对 objects 层做 BEV 网格清理
```

### 动态货物过滤（天车吊货同线过滤）

#### 核心问题

起重机仓库场景中，吊货通常与雷达处于同一中间通道（同线）。吊货在 base_link 坐标系下相对稳定（跟随天车），但在 map 坐标系下随天车移动，会形成中间连续拖影。

#### 核心原则

> **base_link 下负责"发现吊货候选"，map 下负责"确认它是否随天车移动"，mapping 后端负责"延迟提交和扫掠删除"。**

#### 过滤流程

```
merged_points
→ 近场/自身结构过滤
→ 预处理
→ 地面/物体分割

→ BasePayloadChannelFilter（base_link 下吊货候选筛选）
    safe_objects_cloud（安全物体）
    payload_candidate_cloud（吊货候选）

→ NDT input = ground_cloud + safe_objects_cloud（候选不参与 NDT）
→ NDT_OMP 配准
→ 得到 T_map_base

→ PayloadTrackManager 更新 track（双坐标系跟踪）
    base_link 下：判断是否锁定在同线通道
    map 下：判断 displacement、velocity、direction consistency

→ dynamic_payload：不进地图，触发 SweptVolumeEraser
→ pending_payload：进入 PendingBuffer，不进正式地图
→ static_confirmed：才能进入正式静态地图
```

#### 模块说明

**1. BasePayloadChannelFilter**（base_link 下吊货候选筛选）

在 base_link 坐标系下，从中间同线通道中筛选吊货候选：
- 定义 payload_channel_box（base_link 下的中间通道区域）
- channel 内非地面点 → BEV cluster → 满足点数/面积/高度条件 → payload_candidate
- channel 外点 → safe_objects

```yaml
base_payload_channel:
  enabled: true
  lateral_center: 0.0          # 通道横向中心（y 方向）
  lateral_half_width: 3.0      # 通道横向半宽（y 方向 ±3m）
  longitudinal_min: -10.0      # 通道纵向最小值（x 方向）
  longitudinal_max: 10.0       # 通道纵向最大值（x 方向）
  min_object_hag: 0.4          # 低于此高度的点不视为吊货候选
  min_payload_points: 20       # 最少点数
  expand_xy: 0.5               # 候选 bbox XY 膨胀（米）
```

**2. NDT 输入保护**

NDT 输入从 `ground + all objects` 改为 `ground + safe_objects`，排除 payload_candidate：
- 防止吊货拉偏 NDT 定位
- 确保 registration_map 不被吊货污染

**3. PayloadTrackManager**（双坐标系跟踪）

每个 track 同时记录 base_link 和 map 坐标系下的信息：
- base_link 下：判断是否锁定在同线通道（base_center_std 小 = 稳定）
- map 下：判断 displacement、velocity、direction consistency

强吊货判据：
```
base_link 下相对稳定（base_center_std < 0.35m）
+
map 下连续移动（map_displacement > 0.25m, velocity > 0.05m/s）
= DYNAMIC_PAYLOAD
```

#### 效果验证

使用 `调运大件.bag` 测试：

```
[PayloadChannel-KF] channel=513, candidate=120, safe=4257, clusters=2
[PayloadChannel-KF] channel=702, candidate=359, safe=4484, clusters=4
[PayloadChannel-KF] channel=890, candidate=576, safe=4973, clusters=8
[PayloadTrack] tracks=6, dynamic=2, pending=4
[CleanMap] rebuilt: points=86018
[Status] frames=154/154, pose=(-0.65, 0.00, -0.16), global_map=4512
```

- ✅ 通道过滤器成功识别吊货候选（120~576 点，2~8 个簇）
- ✅ NDT 定位稳定，无发散
- ✅ Clean map 正常生成（86018 点）
- ✅ 轨迹连续，无抖动

#### debug 话题

- `/payload_channel_cloud`：通道内所有点（可视化通道范围）
- `/payload_candidate_cloud`：候选吊货点
- `/safe_objects_cloud`：安全物体点
- `/payload_dynamic_cloud`：确认为动态的吊货点
- `/payload_pending_cloud`：待确认的吊货点

---

## 已知问题

### 1. NDT 处理时间不稳定

**问题**：NDT 配准时间在 70-250ms 之间波动，超过 10Hz 实时预算。

**原因**：
- NDT 分辨率 1.0m，迭代次数最多 50 次
- 转弯时特征点减少，NDT 需要更多迭代
- 局部地图更新在主线程执行

**缓解措施**：
- Latest-cloud only 机制：只处理最新帧，避免积压
- NDT 时间预算监控：超过 100ms 输出警告
- 局部地图更新频率已降低

### 2. 转弯时 ICP 拒绝率高

**问题**：转弯时 ICP 精配准频繁被拒绝（rot_diff > 阈值）。

**原因**：转弯时旋转速度快，ICP 修正量自然更大。

**缓解措施**：
- ICP rejected 改为 DEBUG 级别日志
- 转弯时放宽阈值：pos_diff 0.08→0.15m，rot_diff 0.3°→1.0°

### 3. 导航栅格未知区域过多

**问题**：navigation_grid_0.05m.pgm 中 unknown 区域占 56%。

**原因**：地面点覆盖不够充分，很多区域没有被地面点观测到。

**后续改进**：增加地面点覆盖，或对 unknown 区域做插值推断。

### 4. 物体层边缘不够锐利

**问题**：objects_clean 中货物边缘仍有厚化。

**原因**：
- 多帧融合存在轻微位姿误差
- 体素滤波让边角变圆
- 缺少边缘保持型体素下采样

---

## 后续改进方向

### 第一优先级：PendingBuffer + SweptVolumeEraser

实现延迟提交和扫掠删除机制：

```
PendingBuffer：
- payload_candidate 默认进入 pending buffer
- pending 不进入 registration_map / localization_map
- 若 track 确认 dynamic，删除该 track 的 pending 历史点
- 若长期 map 稳定，可选提升为 STATIC_CONFIRMED

SweptVolumeEraser：
- confirmed DYNAMIC_PAYLOAD 后触发
- 使用最近 N 秒 T_map_base 轨迹 + base_link 下 channel_box
- 生成 swept volume → 删除 pending 和局部 static map 中的残留
- lazy delete + 后台异步更新
```

### 第二优先级：PLC 信号接入

接入起重机控制系统信号，实现精确的动态货物过滤：

```yaml
payload_filter:
  use_plc_hook_state: true
  hook_topic: "/crane/hook_state"
  load_state_topic: "/crane/load_state"
```

PLC 信号包括：
- 大车位置 X
- 小车位置 Y
- 吊钩高度 Z
- 是否吊载 load_state
- 起吊/搬运/放下状态

### 第三优先级：离线多轮精配准

实现真正的多轮迭代优化：

```
第一轮：粗建图（NDT + 闭环 + g2o）→ 得到初始轨迹和关键帧
第二轮：用第一轮地图作为先验，重新精配准关键帧位姿
第三轮：从精配准后的关键帧重建多层地图
```

### 第四优先级：BEV 货物边缘检测

从 objects_clean 提取货物边界：

```
1. 0.10m BEV 网格统计稳定货物占据
2. 删除孤立 cell 和小簇
3. 做 opening 操作（不做 closing）
4. 提取边界 cell → 货物轮廓点云
```

### 第五优先级：性能优化

- NDT 分为两级：实时 1.0m / 后台 0.5m
- 局部地图异步双缓冲
- ICP 只服务高精地图，不影响实时位姿
- 转弯/不稳定帧跳过 ICP 和 clean map 插入

### 第六优先级：定位精度评估

新增评估脚本：
- 静止稳定性评估（XY 抖动 < 3cm）
- 闭环回到同一点误差（< 10cm）
- 直线轨迹偏差（横向误差 < 10cm）
- 导航跟踪误差（路径误差 < 10cm）

---

## 使用方法

### 实时定位运行

```bash
cd /home/ydkj/NDT-slam-ws
source devel/setup.bash
roslaunch ndt_slam warehouse_runtime.launch
```

### 继续建图（迭代优化）

```bash
# 便捷方式：交互式选择 bag 文件
./run_mapping.sh

# 直接方式
bash src/ndt_slam/scripts/continue_mapping.sh "/home/ydkj/AutoCraneSlam-ROS1/bag/调运大件.bag"
```

### 地图后处理

```bash
# 一键后处理（生成 warehouse_v003）
bash src/ndt_slam/scripts/run_map_postprocess.sh maps/current maps/warehouse_v003

# 单独运行某个脚本
python3 src/ndt_slam/scripts/analyze_map_quality.py --map_dir maps/current --output report.json
python3 src/ndt_slam/scripts/build_ground_clean.py --config src/ndt_slam/config/map_postprocess.yaml
```

### 地图版本管理

```bash
# 查看所有版本
python3 src/ndt_slam/scripts/map_version_manager.py list

# 切换到指定版本
python3 src/ndt_slam/scripts/map_version_manager.py promote 2

# 回滚到上一版本
python3 src/ndt_slam/scripts/map_version_manager.py rollback
```

---

## 目录结构

```
NDT-slam-ws/
├── maps/                          # 地图版本存储
│   ├── current -> warehouse_v002  # 软链接指向当前版本
│   ├── warehouse_v001.bak/        # 旧版本备份
│   ├── warehouse_v002/            # 当前正式版本
│   └── warehouse_v003/            # 后处理生成的新版本
│
├── src/
│   ├── ndt_slam/                  # 主 SLAM 包
│   ├── ndt_omp/                   # NDT_OMP 配准库
│   └── lidar_slam2_msgs/          # 自定义消息/服务
│
└── output/                        # 运行时输出
    └── payload_debug/             # 动态货物过滤调试输出
```

## 许可证

MIT License
