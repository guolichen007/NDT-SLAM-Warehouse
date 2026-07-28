# GitHub 仓库治理

## 分支保护

当前 master 分支保护状态：**待审查**（gtest 14 个活跃失败未清零前不建议强制要求 full CI）

建议目标配置：
- 禁止 force push
- 禁止删除分支
- PR 合并（不允许直接 push）
- 要求对话解决后合并
- 要求 CI 通过（gtest 全绿后启用）

## 分支命名规范

| 前缀 | 用途 |
|---|---|
| `fix/*` | Bug 修复、安全补丁 |
| `feature/*` | 新功能 |
| `chore/*` | 工程整理、文档、CI |
| `docs/*` | 纯文档变更 |
| `refactor/*` | 代码重构（不变更行为） |
| `experiment/*` | 探索性工作，不合并 |

## Code Owners

见 `.github/CODEOWNERS`。

当前 owner：`@guolichen007`

## CI 策略

- 每次 push/PR 触发
- 静态合同阶段先执行（快速失败）
- 文档合同在静态合同阶段执行
- ROS Noetic 编译与测试在独立阶段执行
- `catkin_test_results` 非零必须使 CI 失败

## Dependabot

监控 GitHub Actions 依赖，每周检查。

## Issue 模板

- 缺陷报告（中文）
- 安全问题（中文，要求 SHA/安全码/复现/日志/现场状态）
- 功能请求（中文）
