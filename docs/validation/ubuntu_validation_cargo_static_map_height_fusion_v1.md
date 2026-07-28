# Ubuntu 20.04 + ROS Noetic 验证移交

Windows 侧只完成代码修改和静态验证。以下项目必须由 Ubuntu 20.04、ROS
Noetic 环境真实执行。

## 1. Git 与统一静态入口

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

python3 scripts/regression/run_static_contracts.py
```

要求：工作区干净，merge-base 为 `10eba695...`，`src/ndt_omp` 为
`5495fd9214945afcb4b35d5a1da385e405c52bf9`。静态入口非零时停止。

## 2. 编译与 C++ gtest

```bash
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin clean -y
catkin build lidar_slam2_msgs ndt_slam --no-status --summarize

source devel/setup.bash
catkin run_tests lidar_slam2_msgs ndt_slam --no-status
catkin_test_results --verbose
```

重点确认实际构建执行：

```text
pending_cargo_envelope_test
cargo_lift_origin_binder_test
cargo_geometry_fusion_test
cargo_avoidance_fusion_test
static_evidence_authorization_test
static_obstacle_evidence_index_test
map_session_snapshot_test
relocalization_scan_context_test
```

## 3. `/load_map_session` 故障注入

每个失败案例前后保存五层点数、UUID、map generation、static revision、
keyframe count 和 height-field revision：

1. 修改 PCD 但不更新 hash。
2. manifest UUID 非法。
3. static source generation/revision 不匹配。
4. 删除或损坏关键帧 PCD、`poses_raw.txt`。
5. 注入 suspension marker 写入/rename 失败。
6. 注入静态 candidate prepare 失败。
7. 注入关键帧 scan-context prepare 失败。

所有失败必须返回 `success=false`，旧地图、UUID、generation、revision、
keyframes 和 height field 全部不变。成功案例必须一次性切换全部身份字段，
内存 revision 必须严格等于 manifest，而不是 N+1。

## 4. 权限与 Pending 场景

- `RUNTIME_MATURE` 高度层只含 `clean_map_confirmed && temporally_mature`
  cell。
- `OPERATOR_APPROVED_BASELINE` 只含 snapshot 明确批准 cell。
- `UNVERIFIED_LOADED_CLEAN` 可诊断，但不得绑定正式 origin、计入正式厚度、
  冻结正式几何、产生官方 static 17/18 或参与 14。
- LOADED 且未 LOCKED 时必须同时执行 live/static 正向危险查询。
- Pending 默认只发布 provisional `NEAR_3M/NEAR_5M`，正式 code 保持
  30/33；显式开关只允许 17/18，永不允许 14。
- Pending 全程 `cargo_valid=false`、正式货物删除=false、MapCommit 授权=false。
- configured fallback 中心必须位于 hook 下方 1.50 m；marker 高度必须等于
  `top-bottom`。

## 5. 状态标记

在真实执行前保持：

```text
ROS Noetic build: NOT_RUN_REQUIRES_UBUNTU
C++ gtest: NOT_RUN_REQUIRES_UBUNTU
Bag replay: NOT_RUN_REQUIRES_REAL_BAG
Server/real crane: NOT_RUN_REQUIRES_SERVER
```
