# 静态地图厚度融合与避障 V1

## 安全边界

本版本把“地图里存在点”与“该点可作为正式安全证据”分开。运行时格子只有在同一 clean-build 序列中满足连续观测次数、稳定时长和 clean 确认后，才具有 `RUNTIME_MATURE` 权威；离开当前视野是 `NOT_IN_VIEW`，暂停 streak，不再被误当成消失。明确看见空闲格子才会生成 tombstone 并失效。历史 clean PCD 若没有审核结论，一律标为 `UNVERIFIED_LOADED_CLEAN`，不能发布 14/17/18。

允许的静态权威只有：

- `RUNTIME_MATURE`：在线连续观测形成的成熟证据；
- `OPERATOR_APPROVED_BASELINE`：离线审计后由操作员批准的基线；
- `UNVERIFIED_LOADED_CLEAN`：仅用于显示和再验证，不进入正式避障。

## 会话事务

`/save_map` 现在先创建同文件系统的临时目录，写入正式五层地图、兼容的 `map_display_full.pcd`、静态证据、关键帧、位姿、诊断文件和 `runtime_status_at_save.json`。每个文件计算 SHA-256，最后写 `manifest.yaml`，再用目录重命名发布。任一写入失败都会删除临时目录，不会留下看似成功的半会话。

`/load_map_session` 和向 `/load_map` 传目录都会先验证 schema、complete、UUID、相对路径、点数和全部 SHA-256。正式层同时装载；单 PCD 加载仍保留为兼容入口，但其静态权威会被清空，不会被当成正式安全会话。

正式层为：

- `map_registration.pcd`
- `map_display.pcd`
- `map_ground.pcd`
- `map_objects_raw.pcd`
- `map_objects_clean.pcd`

`map_display_full.pcd` 是同一代 `map_display.pcd` 的字节级兼容别名，不再由 `ground_raw + objects_filtered` 跨代拼接。调试点云放入 `diagnostics/`，不参与正式层契约。

## 静态高度场

`StaticHeightField` 使用 0.25 m 稀疏 XY 格子，每格最多三个垂直层。每层保存 z05/z50/z95、粗糙度、不确定度、点数、观测数和权威来源。孤立的高 Z 尾点不会抬高 z95。地面支持面采用低分位格子高度和鲁棒平面拟合；缺失格可在有限邻域内插值，并显式增加不确定度。查询由 OBB 外壳和最大格子数双重限制，正式安全线程不会扫描整张 PCD。

运行时高度场只收录已经成熟且 clean-confirmed 的对象格；已批准基线可收录审核过的 clean 层；未验证层即使建立高度场，也会在融合入口被拒绝。

## 起吊原点、厚度和生命周期

`CargoLiftOriginBinder` 从吊钩附近的候选中按“退役正式形状、批准基线、运行时成熟静态、配置包络”排序。没有当前覆盖、没有揭露支撑面或变化未超过 `max(0.15 m, 3σ)` 时，不能把缺点误判为货物消失。原点和揭露厚度都需要多帧确认。

`CargoGeometryFusion` 至少要求两个非配置兜底的独立厚度源，使用加权中位数和 Huber 权重，检查源间差异及融合不确定度。确认后冻结 length/width/height/yaw；后续 track segment 只能更新中心、顶面和保守底面。保守底面同时扣除顶面、厚度、跟踪不确定度和配置裕量。

新增 `LOADED_REACQUIRE`：如果进程或跟踪恢复时重力信号始终为 LOADED，不再等待不存在的 EMPTY 边沿。它只能复用退役正式签名，并通过独立的多帧身份、中心和尺寸门控后回到 LOCKED；期间按候选状态处理，不能授权清空或地图剔除。

## 实时与静态避障融合

正式规则如下：

- 可靠实时危险或可靠静态危险均可发布 17/18；
- 实时为空但静态仍危险时保留静态告警，原因是 `MAP_LIVE_CONFLICT_static_hazard_retained`；
- 14 必须同时具备可靠实时 ROI clear、可信静态会话/运行时成熟证据和正式货物几何/底面；
- 待确认包络可用于正向临时危险提示，但绝不能授权 14；
- 定位、货物、障碍证据故障继续走 31/33/34，不映射成 14。

## 观测与回归

新增话题：

- `/static_evidence/status`
- `/static_evidence/cell_state_counts`
- `/static_evidence/streak_histogram`

新增离线工具 `tools/analyze_map_session.py`，可直接读取会话目录、上层目录或 ZIP；它不依赖 ROS/PCL，输出层哈希、点数、非有限点、包围盒、0.25 m XY 格子/XYZ 体素、层间字节相等和点集包含关系。

Windows 环境没有 ROS Noetic/catkin，因此这里只能运行 Python 回归、真实数据审计和静态差异检查；C++/ROS 单元测试需在 Ubuntu 20.04/ROS Noetic 构建机执行。
