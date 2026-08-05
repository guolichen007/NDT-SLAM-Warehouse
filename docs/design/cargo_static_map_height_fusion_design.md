# 静态地图厚度融合与避障 V1

> 当前实现已升级为约束融合 V2。本节以下的 V1 历史说明仅用于理解原始数据；涉及
> “至少两个来源取平均”“厚度源差异直接拒绝”或 Pending 正向放行的描述，均由下面的
> V2 合同取代。

## V2：预载基线与约束融合

V2 将“货物身份已经可靠”和“厚度已经获得正式复核”拆开。EMPTY 阶段由独立跟踪器
在静止、定位可靠时读取静态高度组件，最近 8 帧至少 5 帧稳定才建立预载基线。
基线绑定地图代次和成员网格；锚点与组件相距超过 0.5m、代次变化、时间回退或观测
断档都会使候选失效。odom Z 不参与厚度计算。

厚度证据按语义分为完整测量、下界和先验。普通实时点云只提供下界；静态厚度 1.0m
与实时可见下界 1.6m 不再互相否决，而是用 1.6m 以上的保守上界形成
`POSITIVE_ONLY`。这一级只允许在障碍证据可靠时输出 17/18，无危险仍输出 33。
起吊前 top-support 与露底复核一致，或静态完整厚度与明确可见底边一致，连续确认后
升级为 `FORMAL`；只有它能输出 14、剔除货物点和授权 MapCommit。

保守底面诊断明确拆分为：顶面参考、厚度上界、跟踪余量、基线/MAD 余量和安全余量。
因此日志里的 clearance 是扣除双方不确定度后的安全净空，不应简单等同于
`cargo_bottom - obstacle_top`；每一项均在 JSON 中独立记录。

### 配置迁移

| V1 字段 | V2 处理 |
|---|---|
| `minimum_independent_sources` | 删除；由约束类型与明确的 FORMAL 证据对决定 |
| `require_authoritative_static_and_live_thickness` | 删除；FORMAL 路径内建此合同 |
| `allow_degraded_live_only_freeze` | 改为 `allow_positive_only_without_static_baseline` |
| `degraded_live_only_uncertainty_floor_m` | 改为 `positive_only_uncertainty_floor_m` |

旧字段不会被运行时读取。部署时必须使用仓库内的新 YAML，不要把旧服务器配置片段覆盖
回来。生产默认使用 `fusion_pending_warning_promotion_policy: evidence_backed_only`：
原始 Pending 只有在身份、外部障碍 Track、来源、几何和置信度门禁全部通过时才可输出
正向 17/18，仍不能输出 CLEAR。已经通过 V2 几何确认的 `POSITIVE_ONLY` 正向告警不依赖
该兼容路径。

## 安全边界

本版本把“地图里存在点”与“该点可作为正式安全证据”分开。运行时格子只有在受控回访窗口内满足观测次数、有效稳定时长和 clean 确认后，才具有 `RUNTIME_MATURE` 权威；离开当前视野是 `NOT_IN_VIEW`，暂停 streak，不再被误当成消失。明确看见空闲格子才会生成 tombstone 并失效。历史 clean PCD 若没有审核结论，一律标为 `UNVERIFIED_LOADED_CLEAN`，不能发布 14/17/18。

允许的静态权威只有：

- `RUNTIME_MATURE`：在线连续观测形成的成熟证据；
- `OPERATOR_APPROVED_BASELINE`：离线审计后由操作员批准的基线；
- `UNVERIFIED_LOADED_CLEAN`：仅用于显示和再验证，不进入正式避障。

## 会话事务

`/save_map` 现在先创建同文件系统的临时目录，写入正式五层地图、兼容的 `map_display_full.pcd`、静态证据、关键帧、位姿、诊断文件和 `runtime_status_at_save.json`。每个文件计算 SHA-256，最后写 `manifest.yaml`，再用目录重命名发布。任一写入失败都会删除临时目录，不会留下看似成功的半会话。这是对并发读者的 **visibility-atomic（可见性原子）** 发布；当前实现只有流 `flush` 和目录 `rename`，没有对每个文件、临时目录和父目录执行 Linux `fsync`，因此不宣称断电后的 crash durability。

`/load_map_session` 和向 `/load_map` 传目录都会先验证 schema、complete、UUID、相对路径、点数和全部 SHA-256，并在临时对象中解析静态索引、构建按授权格过滤的高度场、加载全部关键帧。只有阶段一全部成功后，才在生命周期锁内纯替换运行对象；提交阶段不再读取文件。单 PCD 加载仍保留为兼容入口，但其静态权威会被清空，不会被当成正式安全会话。

正式层为：

- `map_registration.pcd`
- `map_display.pcd`
- `map_ground.pcd`
- `map_objects_raw.pcd`
- `map_objects_clean.pcd`

`map_display_full.pcd` 是同一代 `map_display.pcd` 的字节级兼容别名，不再由 `ground_raw + objects_filtered` 跨代拼接。调试点云放入 `diagnostics/`，不参与正式层契约。

## 静态高度场

`StaticHeightField` 使用 0.25 m 稀疏 XY 格子，每格最多三个垂直层。每层保存 z05/z50/z95、粗糙度、不确定度、点数、观测数和权威来源。孤立的高 Z 尾点不会抬高 z95。地面支持面采用低分位格子高度和鲁棒平面拟合；缺失格可在有限邻域内插值，并显式增加不确定度。查询由 OBB 外壳和最大格子数双重限制，正式安全线程不会扫描整张 PCD。

正向危险查询还必须通过与实时点簇相同的运行方向扇区：速度方向左右各 45°。扇区过滤
发生在静态格子匹配之前，因此侧后方成熟 cell 不能通过 Pending 静态路径绕过实时门禁。
0.30m 接触级近场保持全方向检测，但只输出异常复核 Code 29；速度方向无效时静态正向授权关闭而不是复用旧航向。没有 18 远场接近历史而突然进入 3m 的实时候选或尚未形成独立地图身份的临时静态候选输出 29，不得直接升级为正式 Code 17。

已经由 `STATIC_MAP_MATCH`/`PRE_CARGO_OCCUPANCY` 绑定地图身份、达到成熟门限并通过当前方向与
垂直区间检查的静态障碍，其长期地图观测等价于独立接近历史；首次在 3m 内命中即可
输出 Code 17。实时突发近场和未成熟静态候选仍输出 Code 29。

运行时高度场只收录已经成熟且 clean-confirmed 的对象格；已批准基线可收录审核过的 clean 层；未验证层即使建立高度场，也会在融合入口被拒绝。

静态成熟采用适配仓库回访周期的衰减窗口：300 秒内的独立 clean build 可继续累计，
超窗后保留 50% 的观测数和有效稳定时长，且绝不累计视野外的墙钟时间。明确可观测
空闲或 deny 仍立即撤销；成熟格不会因未回访而自行死亡。跨重启的已验证快照只通过
Manifest/事务加载重绑定当前代次，运行时任意代次不一致仍拒绝授权。

## 起吊原点、厚度和生命周期

`CargoLiftOriginBinder` 已进入 `NdtSlamNode` 的逐帧 LiDAR owner-thread 链路，从吊钩附近的候选中按“退役正式形状、批准基线、运行时成熟静态、配置包络”排序。没有当前覆盖、没有揭露支撑面或变化未超过 `max(0.15 m, 3σ)` 时，不能把缺点误判为货物消失。原点和揭露厚度都要求新鲜、严格连续的多帧确认；重复/回退时间戳、观测间隔超限、覆盖不足、无效 top/support 或 origin 身份变化都会中断相应计数。

`CargoGeometryFusion` 接收静态 origin、map-diff 露底复核、实时可见下界和历史先验。
它不再按“来源数量”平均：完整测量决定 FORMAL，下界决定保守厚度范围，先验不计独立
物理证据。只有 `valid && frozen` 才能覆盖锁定形状；其中 POSITIVE_ONLY 只取得正向
告警权，FORMAL 才取得 CLEAR 和删除权限。确认后冻结 length/width/height/yaw；后续
高置信度多帧证据才能扩缩尺寸，单帧合并点簇不会改写正式形状。

新增 `LOADED_REACQUIRE`：如果进程或跟踪恢复时重力信号始终为 LOADED，不再等待不存在的 EMPTY 边沿。它只能复用退役正式签名，并通过独立的多帧身份、中心和尺寸门控后回到 LOCKED；期间按候选状态处理，不能授权清空或地图剔除。

## 实时与静态避障融合

正式规则如下：

- 可靠实时危险或可靠静态危险均可发布 17/18；
- 实时为空但静态仍危险时保留静态告警，原因是 `MAP_LIVE_CONFLICT_static_hazard_retained`；
- 14 必须同时具备可靠实时 ROI clear、可信静态会话/运行时成熟证据和正式货物几何/底面；
- 原始待确认包络只积累诊断；POSITIVE_ONLY 可用于正向危险告警，但绝不能授权 14；
- 定位、货物、障碍证据故障继续走 31/33/34，不映射成 14。

## 观测与回归

新增话题：

- `/static_evidence/status`
- `/static_evidence/cell_state_counts`
- `/static_evidence/streak_histogram`
- `/cargo_avoidance/pending_status`
- `/cargo_avoidance/pending_envelope_marker`

当重力已经确认 `LOADED` 但还没有 authoritative track 时，主节点按“当前连续候选、退役正式形状、已绑定起吊原点、配置最大包络”建立 `PendingCargoEnvelope`，执行 live 外壳和静态高度场的正向危险查询并发布 provisional 状态。该包络始终保持 `cargo_valid=false`，不会删除正式货物点、不会写入成熟静态证据、不会授权 MapCommit；默认 `fusion_provisional_warning_to_official_code=true` 仅允许执行 `evidence_backed_only` 的严格门控，门禁通过后只能把正向危险升级为 17/18，永远不能产生 14。设为 `false` 会作为兼容熔断开关关闭该路径，但设为 `true` 不能绕过类型化策略。

新增离线工具 `tools/analyze_map_session.py`，可直接读取会话目录、上层目录或 ZIP；它不依赖 ROS/PCL，输出层哈希、点数、非有限点、包围盒、0.25 m XY 格子/XYZ 体素、层间字节相等和点集包含关系。

Windows 环境没有 ROS Noetic/catkin，因此这里只能运行 Python 回归、真实数据审计和静态差异检查；C++/ROS 单元测试需在 Ubuntu 20.04/ROS Noetic 构建机执行。
