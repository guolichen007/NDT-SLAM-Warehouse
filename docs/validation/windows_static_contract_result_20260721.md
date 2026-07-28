# Windows 静态合同验证结果（2026-07-21）

## 验证边界

执行环境为 Windows。本报告只确认 Python、YAML、Git 和源码结构合同。
ROS Noetic 编译、C++ gtest、Bag 回放、服务器运行和真实吊装均没有在
Windows 上伪造为通过。

代码提交基线：

- base/master：`10eba695faee0775ecdbfd806a7813fb38f69fd4`
- 第一次修复前 HEAD：`dc129a921cdfcbd53b461cbc58da74b653199490`
- 第二轮修复前 HEAD：`3be49103b65ab2efc30456b018cda9bf5cbae12b`
- `src/ndt_omp`：`5495fd9214945afcb4b35d5a1da385e405c52bf9`

## 两轮 CI 失败对比

### 第一轮（run 29806291286, 修复 `dc129a9`）

```text
fatal: detected dubious ownership in repository
FAIL: cannot enumerate tracked files
```
根因：ROS job container 内 Git 不信任 bind-mounted `$GITHUB_WORKSPACE`。
修复：在 CI 中登记安全目录并统一调用 `run_static_contracts.py`。

### 第二轮（run 29810283921, 修复 `3be4910`）

```text
TypeError: 'type' object is not subscriptable
FAIL: unittest discover (test_analyze_map_session.py line 15)
```
根因：Python 3.8（ROS Noetic 容器）不支持运行时解析 `list[tuple[...]]` 注解。
修复：在 `tests/test_analyze_map_session.py` 添加 `from __future__ import annotations`。
完整记录见 `docs/github_ci_failure_3be4910.md`。

## Windows 结果（HEAD 3be4910 + Python 3.8 fix）

| 命令 | 退出码 | 结果 |
|---|---:|---|
| `python scripts/regression/run_static_contracts.py` | 0 | PASS_WINDOWS |
| `python scripts/regression/check_yaml_duplicate_keys.py` | 0 | PASS_WINDOWS |
| `python scripts/regression/check_repository_integrity.py` | 0 | PASS_WINDOWS；897 个已跟踪文件 |
| `python scripts/regression/check_cargo_safety_e2e.py` | 0 | PASS_WINDOWS |
| `python -m compileall -q scripts src/ndt_slam/scripts tests` | 0 | PASS_WINDOWS |
| `python -m unittest discover -v` | 0 | PASS_WINDOWS；25/25 |
| `git diff --check` | 0 | PASS_WINDOWS |

## Python 版本

- 验证环境 Python：3.10.12
- Python 3.8 本地验证：NOT_RUN_LOCAL_PY38
- 最终 Python 3.8 兼容由 GitHub ROS Noetic 容器确认

## 本轮修改

```text
tests/test_analyze_map_session.py  — 添加 from __future__ import annotations
```

其他使用 `list[...]`/`dict[...]`/`tuple[...]` 注解的 Python 文件已在之前提交中
添加了 `from __future__ import annotations`，本轮无需修改：

- `tools/analyze_map_session.py`
- `scripts/regression/check_cargo_safety_e2e.py`
- `scripts/regression/check_repository_integrity.py`
- `scripts/regression/run_static_contracts.py`
- `src/ndt_slam/scripts/ops/server_runtime_monitor.py`

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

- `NOT_RUN_LOCAL_PY38`：本地无 Python 3.8，由 GitHub CI 最终验证。
- `NOT_RUN_REQUIRES_UBUNTU`：ROS Noetic/catkin 编译、全部 C++ gtest、
  ROS 节点/服务/话题验证。
- `NOT_RUN_REQUIRES_REAL_BAG`：真实 Bag 场景矩阵。
- `NOT_RUN_REQUIRES_SERVER`：生产地图、双雷达、长时间运行和故障注入。
