# GitHub CI 失败记录（2026-07-21 · HEAD 3be4910）

## CI 标识

```text
workflow:    repository-and-catkin
run id:      29810283921
run number:  44
job id:      88569540172
job:         noetic
branch:      feature/cargo-static-map-height-fusion-v1
```

## 已通过的步骤

| 步骤 | 结果 |
|------|------|
| repository integrity | PASS |
| cargo safety E2E contract | PASS |
| Python compileall | PASS |

## 失败步骤

```text
python3 -m unittest discover -v
```

## 失败文件

```text
tests/test_analyze_map_session.py
```

## 失败位置

第 15 行附近的函数签名：

```python
def write_binary_pcd(
    path: Path,
    points: list[tuple[float, float, float]],
) -> None:
```

## 原始异常

```text
TypeError: 'type' object is not subscriptable
```

`list[tuple[float, float, float]]` 注解中的 `list[...]` 和 `tuple[...]` 在 Python 3.8 运行时解析失败，导致模块导入异常。

## 根因

GitHub `repository-and-catkin` workflow 使用 ROS Noetic 容器（Ubuntu 20.04），
容器内 Python 版本为 3.8。Python 3.8 不支持在类型注解中直接使用内置泛型语法
`list[...]`、`tuple[...]`、`dict[...]`、`set[...]`，除非文件顶部有
`from __future__ import annotations`。

当 `unittest discover` 尝试导入 `tests/test_analyze_map_session.py` 时，
Python 3.8 在模块顶层解析类型注解并触发 `TypeError`，导致所有 25 个测试
（包括其他文件中的 22 个）都无法运行。

其他三个同样使用 `list[...]` / `dict[...]` 注解的 Python 文件已经有
`from __future__ import annotations` 且不受影响：

- `tools/analyze_map_session.py`
- `scripts/regression/check_cargo_safety_e2e.py`
- `scripts/regression/check_repository_integrity.py`
- `scripts/regression/run_static_contracts.py`
- `src/ndt_slam/scripts/ops/server_runtime_monitor.py`

## 后续影响

由于 `unittest discover` 失败在静态检查阶段，后续步骤完全被跳过：

- Install ROS build dependencies
- Install pinned Sophus and g2o
- Clean catkin build
- C++ unit tests

这意味着本次 run 的 ROS 编译和 C++ 测试状态仍然未知。

## 修复方式

在 `tests/test_analyze_map_session.py` 文件顶部（模块 docstring 之后、
所有 import 之前）添加：

```python
from __future__ import annotations
```

这是 Python 3.7+ 支持的 `__future__` 导入，使所有注解求值延迟到
`typing.get_type_hints()` 调用时，避免了 Python 3.8 的运行时
`TypeError`。

## 预期修复后行为

```text
Static repository contracts    PASS
```

所有后续 ROS/catkin 步骤将正常执行。

## 时间线

```text
2026-07-21  发现并确认根因
2026-07-21  添加 from __future__ import annotations
2026-07-21  Windows 本地验证通过
2026-07-21  等待用户 push 后在 GitHub CI 验证
```
