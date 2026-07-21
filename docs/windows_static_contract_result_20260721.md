# Windows 静态合同验证结果（2026-07-21）

## 验证边界

执行环境为 Windows。本报告只确认 Python、YAML、Git 和源码结构合同。
ROS Noetic 编译、C++ gtest、Bag 回放、服务器运行和真实吊装均没有在
Windows 上伪造为通过。

代码提交基线：

- base/master：`10eba695faee0775ecdbfd806a7813fb38f69fd4`
- 修复前 HEAD：`dc129a921cdfcbd53b461cbc58da74b653199490`
- 本轮代码 HEAD（证据文档提交前）：`d0c2f6b`
- `src/ndt_omp`：`5495fd9214945afcb4b35d5a1da385e405c52bf9`

## GitHub 原始失败

Run `29806291286`、job `88557420965` 的 `Static repository contracts`
失败。原始失败行为：

```text
fatal: detected dubious ownership in repository at '/__w/NDT-SLAM-Warehouse/NDT-SLAM-Warehouse'
FAIL: cannot enumerate tracked files: Command '['git', 'ls-files', '-z']' returned non-zero exit status 128.
Process completed with exit code 2.
```

完整时间戳证据见 `docs/github_ci_failure_dc129a9.md`。根因是 ROS job
container 内 Git 没有把 bind-mounted `$GITHUB_WORKSPACE` 视为安全目录；
失败发生在 `git ls-files`，货物安全合同和 catkin 尚未执行。

修复后 CI 在容器内登记该精确目录，并统一调用：

```text
python3 scripts/regression/run_static_contracts.py
```

## Windows 结果

| 命令 | 退出码 | 结果 |
|---|---:|---|
| `python scripts/regression/run_static_contracts.py` | 0 | PASS_WINDOWS |
| `python scripts/regression/check_yaml_duplicate_keys.py` | 0 | PASS_WINDOWS |
| `python scripts/regression/check_repository_integrity.py` | 0 | PASS_WINDOWS；897 个已跟踪文件 |
| `python scripts/regression/check_cargo_safety_e2e.py` | 0 | PASS_WINDOWS |
| `python -m compileall -q scripts src/ndt_slam/scripts tests` | 0 | PASS_WINDOWS |
| `python -m unittest discover -v` | 0 | PASS_WINDOWS；25/25 |
| `git diff --check` | 0 | PASS_WINDOWS |

## 本轮合同

- Pending 配置兜底中心满足
  `cargo_center_z = hook_anchor_z + configured_center_offset_z_m`，默认偏移
  `-1.50 m`。
- Pending 长宽高均把候选不确定度和 margin 各展开一次；
  `height_m == top_z_base - bottom_z_base`。
- Pending evaluator 使用已展开的 bottom，额外 bottom uncertainty 为零，避免
  同一不确定度再次扣减。
- `/load_map` 与 `/load_map_session` 共用单次 `stageRuntimeMap` candidate；
  wrapper 不再二次 `loadVerified`。
- PCD、manifest/hash、静态索引、height field、关键帧 scan context、NDT 和
  空工作缓冲均在安装前准备。
- persistent static suspension 在首个地图指针替换前执行；失败立即返回。
- 安装阶段使用 `installPreparedSnapshot` 和
  `installPreparedKeyFrameDatabase`，不读取磁盘、不解析 PCD/YAML/hash。
- 静态 revision 按 manifest 原值安装，不增加到 N+1。
- `UNVERIFIED_LOADED_CLEAN` 由强类型 gate 禁止正式 origin、独立厚度、
  官方静态 17/18 和 14；仅保留诊断高度场。
- Pending 默认仍返回正式 30/33；显式开启时只能正向升级 17/18，永远不能
  产生 14，`cargo_valid` 与正式货物删除授权始终为 false。

## 未运行

- `NOT_RUN_REQUIRES_UBUNTU`：ROS Noetic/catkin 编译、全部 C++ gtest、
  ROS 节点/服务/话题验证。
- `NOT_RUN_REQUIRES_REAL_BAG`：真实 Bag 场景矩阵。
- `NOT_RUN_REQUIRES_SERVER`：生产地图、双雷达、长时间运行和故障注入。
