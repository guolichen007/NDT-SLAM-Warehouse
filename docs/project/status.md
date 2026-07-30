# 项目状态

当前主线状态以本文件所在提交为准。最近 NDT 恢复审查基线：
`2759128e6e08c3fdadebc88f26b39727c79cac38`。

现场验证基线：`8d7d7eed0548321bf0646232f374fe95a29990dd`（`validation-obstacle-avoidance-20260728`）

状态矩阵将“当前实现”和“历史现场证据”分开。`2759128` 及其父提交已做 Windows
静态检查，但没有在 Windows 上冒充 ROS/PCL/Sophus 编译；Ubuntu、Bag 和现场列
只有取得对应提交的实际证据后才会更新。

## 能力状态矩阵

| 能力 | 当前实现 | Windows 静态 | 当前实现 Ubuntu | 当前实现 Bag/现场 | 历史现场证据 |
|---|---|---|---|---|---|
| NDT/EKF/静止保持 | 已实现 | 通过 | `NOT_RUN` | `NOT_RUN` | `8d7d7ee` 长期运行 |
| 长期在线建图/五层地图 | 已实现 | 通过 | `NOT_RUN` | `NOT_RUN` | `8d7d7ee` 现场地图/RViz |
| `objects_clean` 重观测清理 | 已修复（`8b59a0a`） | 通过 | `NOT_RUN` | `NOT_RUN` | 无当前实现证据 |
| 吊物检测、刚体框和显示尺寸 | 已修复（`341e209`、`b5984fa`） | 通过 | `NOT_RUN` | `NOT_RUN` | 无当前实现证据 |
| 障碍追踪与 Code 17/18 | 已实现 | 通过 | `NOT_RUN` | `NOT_RUN` | `8d7d7ee`：235×18、53×17 |
| Pending 静态风险正向告警 | 已实现（`f57d68a`） | 通过 | `NOT_RUN` | `NOT_RUN` | 2026-07-29 数据不能绑定该实现 |
| NDT fitness 自适应熔断 | 已实现（`f57d68a`，恢复链在 `60884c0` 加固） | 通过 | `NOT_RUN` | `NOT_RUN` | 无当前实现证据 |
| 重定位结果多帧确认 | 已实现（`f57d68a`，全局时效在 `60884c0` 加固） | 通过 | `NOT_RUN` | `NOT_RUN` | 无当前实现证据 |
| `objects_clean` 优先全图重定位 | 已实现（`60884c0`） | 通过 | `NOT_RUN` | `NOT_RUN` | 无当前实现证据 |
| NDT 恢复看门狗/systemd 监督 | 已实现（`2759128`） | 通过 | `NOT_RUN` | `NOT_RUN` | 无当前实现证据 |
| 定位地图报告原子写入 | 已实现（`f57d68a`） | 通过 | `NOT_RUN` | `NOT_RUN` | 无当前实现证据 |
| Code 18→主控→S3 | 已实现 | 通过 | `NOT_RUN` | `NOT_RUN` | `8d7d7ee` 分段证据：243×接收、224×S3 |
| Code 17→主控 S3 | 已实现 | 通过 | `NOT_RUN` | `NOT_RUN` | 无 |
| S3 独立闸门路径 | 已实现 | 通过 | `NOT_RUN` | `NOT_RUN` | 无 |

## 验证状态

| 范围 | 状态 |
|---|---|
| 当前实现 Windows 静态合同 | 通过 |
| 当前实现 Ubuntu clean build | `NOT_RUN` |
| 当前实现 Ubuntu gtest | `NOT_RUN` |
| 当前实现 Bag/现场 | `NOT_RUN` |
| 历史治理基线 Ubuntu build | 4/4 packages 通过 |
| 历史治理基线 gtest | 944 总计，930 通过，14 失败 |
| 历史治理基线状态 | `FAIL_KNOWN_BASELINE`、`NO_NEW_FAILURES` |

## 版本状态

| 维度 | 状态 |
|---|---|
| **仓库工程状态** | `MASTER_ENTERPRISE_ENGINEERED` |
| **避障基线状态** | `FIELD_VALIDATED_RC` |
| **当前实现质量门禁** | `BLOCKED_NOT_RUN` |
| **历史治理基线质量门禁** | `FAIL_KNOWN_BASELINE` |
| **Production Release 状态** | `BLOCKED` |

`FIELD_VALIDATED_RC` 仅表示避障算法基线（`8d7d7ee`）已取得现场验证证据，不表示
当前实现已达到正式发布准入条件。2026-07-29/30 的补充数据属于
[候选证据审查](../validation/obstacle_avoidance_runtime_evidence_review_20260729_20260730.md)，
不会替换正式验证 Tag。
