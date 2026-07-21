# Windows 静态合同结果（2026-07-21）

## 边界

执行环境为 Windows。本文只记录真实执行的 Python、YAML、Git 和源码级合同检查；ROS Noetic/catkin、C++ gtest、节点启动、Bag 回放和服务器验收均未在 Windows 伪造为通过。

## 修改前基线

| 命令 | 退出码 | 结果 |
|---|---:|---|
| `python scripts/regression/check_yaml_duplicate_keys.py` | 0 | PASS_WINDOWS |
| `python scripts/regression/check_repository_integrity.py` | 0 | PASS_WINDOWS，890 个已跟踪源码/配置文件 |
| `python scripts/regression/check_cargo_safety_e2e.py` | 0 | PASS_WINDOWS |
| `python -m compileall -q scripts src/ndt_slam/scripts tests` | 0 | PASS_WINDOWS |
| `python -m unittest discover -v` | 0 | PASS_WINDOWS，20/20 |
| `git diff --check` | 0 | PASS_WINDOWS |

因此本轮没有修改附件明确排除的 `runtime_diagnostics_test.cpp`、`RuntimeDiagnosticsConfig::console_period_sec` 或旧 GitHub catkin 漂移。

## 修改后结果

| 命令 | 退出码 | 结果 |
|---|---:|---|
| `python scripts/regression/check_yaml_duplicate_keys.py` | 0 | PASS_WINDOWS |
| `python scripts/regression/check_repository_integrity.py` | 0 | PASS_WINDOWS |
| `python scripts/regression/check_cargo_safety_e2e.py` | 0 | PASS_WINDOWS；覆盖 pending、binder、geometry、两阶段 load、revision 与授权格合同 |
| `python -m compileall -q scripts src/ndt_slam/scripts tests` | 0 | PASS_WINDOWS |
| `python -m unittest discover -v` | 0 | PASS_WINDOWS，20/20 |
| `git diff --check` | 0 | PASS_WINDOWS |

## 修复文件范围

- Pending 包络：`pending_cargo_envelope.hpp/.cpp`、对应 gtest、CMake。
- 运行时融合：`ndt_slam.hpp/.cpp`、`cargo_lift_origin_binder.*`、`cargo_geometry_fusion.*`、生产 YAML。
- 事务与授权：`map_session_snapshot.*`、`static_obstacle_evidence_index.*`、`keyframe_manager.*`、`loop_closure.*` 及测试。
- 合同与说明：`check_cargo_safety_e2e.py`、设计文档和 Ubuntu 验证文档。

## 必须在 Ubuntu/现场复验

- `NOT_RUN_REQUIRES_UBUNTU`：catkin clean/build、全部 C++ gtest、五个生产节点、服务和话题。
- `NOT_RUN_REQUIRES_REAL_BAG`：十个 Cargo/静态避障/会话场景回放。
- `NOT_RUN_REQUIRES_SERVER`：真实双雷达、生产地图、长期运行和故障注入。
