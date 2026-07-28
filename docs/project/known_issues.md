# 已知问题

## KI-001：CargoObstacleTracker 测试失败

| 字段 | 内容 |
|---|---|
| **模块** | `CargoObstacleTracker` |
| **现象** | 10 个 gtest 失败 |
| **影响** | 障碍物追踪的远场历史/静态 provenace 边界条件判断与测试预期不一致 |
| **安全影响** | 不影响正向 17/18 输出。失败集中在边界条件：near_field_track_missing_far_history、static_provenance_unavailable、static duration check |
| **复现** | `catkin run_tests --no-status` 后 `catkin_test_results --verbose` |
| **当前状态** | 已知基线失败，非回归 |
| **临时措施** | 持续监控测试输出。现场运行时 tracker 行为已在避障场景中验证 |
| **目标修复** | 对齐测试期望与当前 tracker 逻辑，或修复 tracker 边界条件 |
| **验证要求** | 修复后所有 44 个 CargoObstacleTracker 测试通过 |
| **关联 SHA** | `8d7d7ee`（基线） |

## KI-002：CargoSwingMonitor 测试失败

| 字段 | 内容 |
|---|---|
| **模块** | `CargoSwingMonitor` |
| **现象** | 2 个 gtest 失败：ConfiguredRopeIsNotAngleAuthoritative、ImmediateAlarmTransitionsThroughSettling |
| **影响** | 吊绳长度配置与角度授权关系的测试期望与当前实现不一致 |
| **安全影响** | 不影响实际摆动检测与 skew-pull 告警。ConfiguredRope 相关失败是 HOIST_MISSING 诊断边界 |
| **复现** | `catkin run_tests` |
| **当前状态** | 已知基线失败 |
| **临时措施** | 生产配置使用真实 hoist 数据源，不依赖 configured rope |
| **目标修复** | 审查 rope length 授权链路，统一测试与实现 |
| **验证要求** | 修复后 84 个 CargoSwingMonitor 测试全部通过 |
| **关联 SHA** | `8d7d7ee` |

## KI-003：CargoComponentFusion 测试失败

| 字段 | 内容 |
|---|---|
| **模块** | `CargoComponentFusion` |
| **现象** | 2 个 gtest 失败 |
| **影响** | 货物组件融合的边界条件测试与当前行为不一致 |
| **安全影响** | 低。货物检测的主路径（OBB→Locked→Geometry→Safety）不受影响 |
| **复现** | `catkin run_tests` |
| **当前状态** | 已知基线失败 |
| **目标修复** | 对齐 component fusion 测试期望 |
| **关联 SHA** | `8d7d7ee` |
