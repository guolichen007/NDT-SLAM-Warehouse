# GitHub 仓库治理

## 分支保护

当前事实：

```text
FULL_GTEST = 1000 / 0
PYTHON     = 107 / 0
master protected = false
```

目标配置（测试已全绿，可启用）：

```text
master protected = true
force push disabled
branch deletion disabled
PR required
required checks enabled
CODEOWNERS review required（按团队需求）
```

本轮仅记录治理目标，不改动 GitHub branch protection 设置，除非另行授权。

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
