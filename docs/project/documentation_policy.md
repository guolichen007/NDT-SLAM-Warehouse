# 文档保留策略

## 哪些文档应进入 Git 并长期保留

| 类型 | 示例 | 位置 |
|---|---|---|
| 当前技术合同 | 架构、API、安全协议、配置说明 | `src/ndt_slam/doc/` |
| 项目管理 | 状态、路线图、已知问题、发布流程 | `docs/project/` |
| 验证证据 | 与正式 Tag 绑定的现场验证报告 | `docs/validation/` |
| 设计决策 | 重要设计文档 | `docs/design/` |
| 事故复盘 | 有长期安全价值的事故分析 | `docs/incidents/` |

## 哪些不应进入 Git 或应定期清理

| 类型 | 原因 |
|---|---|
| 临时 branch status | Git 历史本身就是记录 |
| PR body 草稿 | 提炼到 design doc 后删除原文 |
| 一次性 TODO / 聊天总结 | 不构成正式工程文档 |
| 普通 CI failure note | 提炼规则到 CONTRIBUTING 后删除 |
| 运行日志 / 大 JSON / bag / PCD / server_runs | `.gitignore` 已排除 |
| 自动生成报告 | 放入 Actions artifact 或 release asset |

## 技术文档要求

`src/ndt_slam/doc/` 下的每一篇文档必须：
- 描述当前 master 的真实行为
- 不引用已删除的类、文件、Topic、参数
- 不含待办事项或道路规划
- 含"最后更新"或对应版本标注

## 禁止

- 在 README 中列出已知问题、待完成项、测试失败
- 在技术文档中引用不存在的头文件或类
- 用"看起来合理"代替与代码的逐项核对
