# 贡献指南

## 分支规范

| 前缀 | 用途 | 合并目标 |
|---|---|---|
| `fix/*` | Bug 修复、安全补丁 | `master` |
| `feature/*` | 新功能 | `master` |
| `chore/*` | 工程整理、文档、CI | `master` |
| `docs/*` | 纯文档变更 | `master` |
| `refactor/*` | 代码重构（不变更行为） | `master` |
| `experiment/*` | 探索性工作 | 不合并 |

### 历史改写策略

- **本地未发布分支**：允许 rebase。
- **已 push 或多人共享分支**：默认不改写历史。
- **服务器已验证的 branch/SHA**：严禁 rebase 或 force push。
- **现场验证 SHA / validation tag**：严禁改写。
- **master**：禁止 force push。只允许 Fast-Forward 合并。

## 提交风格

使用常规提交前缀，按子系统区分：

```
fix(cargo):
fix(safety):
feat(mapping):
feat(monitor):
chore(repo):
docs:
test:
ci:
```

## 安全合同 PR 要求

任何涉及吊物安全行为的 PR 必须包含：

```
INPUT_SHA:
OUTPUT_SHA:

安全合同影响：
- [ ] Code 14（CLEAR）
- [ ] Code 17（NEAR_3M）
- [ ] Code 18（NEAR_5M）
- [ ] Code 30-35（FAULT/INVALID）
- [ ] 货物点从 registration 剔除
- [ ] 静态地图排除
- [ ] MapCommit 排除
- [ ] 定位
- [ ] 消息 Schema

是否改变运行行为：是 / 否
回滚 SHA：
```

## 验证清单

提交 review 前：

- [ ] `git diff --check`
- [ ] `python3 scripts/regression/run_static_contracts.py`
- [ ] `python3 scripts/regression/check_repository_integrity.py`
- [ ] `python3 scripts/regression/check_cargo_safety_e2e.py`
- [ ] `python3 scripts/regression/check_docs_contract.py`
- [ ] `python3 -m compileall scripts tests tools`
- [ ] `python3 -m unittest discover`
- [ ] Ubuntu clean catkin build
- [ ] Ubuntu gtest（`catkin run_tests && catkin_test_results`）
- [ ] Bag 验收（运行时行为变更时必须）
- [ ] 服务器运行（运行时行为变更时必须）
- [ ] 现场验证（安全合同变更时必须）

无法执行的环境相关检查必须标记为 **NOT_RUN**，不得标记为已通过。仅 Windows 检查不足以替代 ROS 编译、测试、bag 或现场验证。

## 服务器验证证据

服务器验证运行必须保留：
- `run_manifest.json`
- `reports/final_summary.json`
- `reports/final_report.md`

附上精确 SHA。不得将未执行项目标记为已通过。生成的地图、bag 和 `server_runs/` 不属于 Git。
