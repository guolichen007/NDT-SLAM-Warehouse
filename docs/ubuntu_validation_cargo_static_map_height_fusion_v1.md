# Ubuntu 20.04 + ROS Noetic 验证：Cargo Static Map Height Fusion V1

本文件交给 Claude CLI 在 Ubuntu 20.04/ROS Noetic 工作空间执行。Windows 侧没有声明 C++、ROS、Bag 或服务器验证通过。

## 1. 环境与 Git 预检

```bash
source /opt/ros/noetic/setup.bash
cd ~/NDT-slam-ws

git fetch origin --prune
git switch feature/cargo-static-map-height-fusion-v1
git pull --ff-only origin feature/cargo-static-map-height-fusion-v1

git status --short
git rev-parse HEAD
git merge-base HEAD origin/master
git submodule status --recursive
git log --oneline --decorate -12
```

要求：merge-base 为 `10eba695faee0775ecdbfd806a7813fb38f69fd4`，`src/ndt_omp` 为 `5495fd9214945afcb4b35d5a1da385e405c52bf9`，工作区干净。最终 HEAD 以 Codex 最终报告和远端分支为准。

## 2. 静态合同

逐条执行，保留每条退出码：

```bash
python3 scripts/regression/check_yaml_duplicate_keys.py
python3 scripts/regression/check_repository_integrity.py
python3 scripts/regression/check_cargo_safety_e2e.py
python3 -m compileall -q scripts src/ndt_slam/scripts tests
python3 -m unittest discover -v
git diff --check
```

任何非零退出码都先停止，不进入运行验收。

## 3. ROS 编译

```bash
source /opt/ros/noetic/setup.bash
cd ~/NDT-slam-ws

catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin clean -y
catkin build lidar_slam2_msgs ndt_slam --no-status --summarize
```

不得用 Windows 静态检查替代此结果。

## 4. C++ 测试

```bash
source /opt/ros/noetic/setup.bash
source ~/NDT-slam-ws/devel/setup.bash
cd ~/NDT-slam-ws

catkin run_tests lidar_slam2_msgs ndt_slam --no-status
catkin_test_results --verbose
```

重点确认这些新增/扩展目标实际被构建并执行：

```text
pending_cargo_envelope_test
cargo_lift_origin_binder_test
cargo_geometry_fusion_test
cargo_avoidance_fusion_test
static_obstacle_evidence_index_test
map_session_snapshot_test
```

## 5. 五个生产节点

```bash
find devel -type f -executable | grep -E \
'ndt_slam_node|pointcloud_merger|hook_load_state_node|cargo_alarm_heartbeat_node|cargo_forbidden_zone_node'
```

必须同时找到五个目标；`cargo_alarm_heartbeat_node` 仍应是 `/cargo_avoidance/status_code` 唯一发布者。

## 6. 服务与话题

启动实际生产 launch 后：

```bash
rosservice list | grep -E 'load_map_session|save_map|load_map'
rostopic list | grep -E \
'cargo_avoidance|static_evidence|hook/load_state'

rostopic info /cargo_avoidance/status_code
rostopic info /cargo_avoidance/pending_status
rostopic info /cargo_avoidance/pending_envelope_marker
```

## 7. `/load_map_session` 两阶段事务故障注入

先记录当前状态：

```bash
rostopic echo -n 1 /static_evidence/status
rostopic echo -n 1 /map | sed -n '1,20p'
rosservice call /load_map_session "session_directory: '/absolute/path/to/valid/session'"
```

复制合法 session 到临时目录，分别构造以下独立案例；每次失败前后记录五层点数、当前 generation、session UUID 和 static revision：

1. 合法 session：成功，五层同时切换。
2. 修改一个 PCD 字节但不更新 manifest：hash 拒绝。
3. manifest UUID 非法：拒绝。
4. manifest generation 与静态索引 source generation 不同：拒绝。
5. 删除任一正式文件：拒绝。
6. 删除/损坏一个 `keyframes/*.pcd` 或 `poses_raw.txt`：拒绝。
7. 所有失败案例：原地图点数、generation、UUID、static revision 完全不变。
8. 成功案例：五层 generation 一致，内存 static revision 与 manifest 完全相等（不得 N+1）。
9. `RUNTIME_MATURE`：高度场仅含 `clean_map_confirmed && temporally_mature` 格。
10. `OPERATOR_APPROVED_BASELINE`：高度场仅含 snapshot 明确列出的格。
11. `UNVERIFIED_LOADED_CLEAN`：可诊断，但不能形成正式 static 17/18/14 或正式厚度源。

## 8. Bag 回放矩阵

对每个场景保存 `/cargo_avoidance/safety_status`、`/cargo_avoidance/pending_status`、`/cargo_avoidance/bottom_estimate`、`/static_evidence/status`、pending/fused marker 和 runtime CSV。

| 场景 | 预期 requested code | provisional | cargo_valid / frozen | 风险与 14 约束 |
|---|---|---|---|---|
| A EMPTY→LOADED→CANDIDATE→LOCKED | EMPTY 可 14；pending 默认 30/33；LOCKED 按证据 | pending 可 UNKNOWN/NEAR | 锁定前 false/false，双源连续确认后 frozen | pending 永不 14 |
| B 节点启动时已 LOADED | 默认 30/33，危险显式开关才 17/18 | 能输出 NEAR | reacquire 前 cargo_valid=false | 不等待 EMPTY 边沿；不能清空 |
| C LOADED 未正式识别 | 默认 30/33 | live/static 危险产生 NEAR_3M/NEAR_5M | false/false | 必须执行两路正向查询；永不 14 |
| D LOCKED→LOST_HOLD→LOADED_REACQUIRE | 保守保持或故障码 | 记录 retained/pending | lifecycle id 不变，segment id 增加 | 复用 frozen 只在 `valid && frozen` |
| E 起吊后暴露支撑面 | 证据不足仍非正式 | 记录 lift/thickness count | 连续帧后 map-diff 有效并冻结 | 无效帧必须打断计数 |
| F static 危险、live clear | 17/18 | static NEAR | 正式 track 为 true | `MAP_LIVE_CONFLICT`，不能 14 |
| G live 危险、static clear | 17/18 | live NEAR | 正式 track 为 true | 危险不能被 static clear 覆盖 |
| H live/static 同时可靠 clear | 14 | CLEAR_NOT_AUTHORIZED 仅限 pending | 必须正式 track + 正式 bottom | pending 仍不能 14 |
| I 静态 session 未验证 | 34/33 或 live 正向危险 | static 不具正式 authority | 不得用其冻结正式厚度 | 不得 static 14/17/18 |
| J Pending 生命周期 | 默认 30/33 | 来源按 current→retired→origin→configured | cargo_valid 始终 false | 不删货物点、不授权 MapCommit、不写成熟证据 |

## 9. 关键断言

```text
Pending envelope 永远不输出 14。
Pending envelope 永远不授权 MapCommit 吊物删除。
正式 14 必须 live 与可信 static 同时 clear。
静态危险不能被单帧 live clear 覆盖。
LOADED 未识别时仍产生 provisional 危险信息。
至少两个独立非配置厚度来源才能冻结几何。
无效、重复、回退或超间隔帧会中断连续确认。
失败的 load_map_session 不改变当前地图。
加载后 revision 与 manifest 完全一致。
runtime mature 只授权成熟 cell。
```

## 10. 服务器与真实设备

完成编译/单测/Bag 后，仍需在真实双雷达和服务器上验证长时间运行、地图切换、断流、时间回退和重定位跳变。保存所有日志及 session 副本；在此之前不要把结果标成生产 PASS。
