# 文档保留策略

## 原则

当前源码树只保留「当前工程事实」：技术方案、架构、接口、配置、部署、运维、
安全、测试准入、当前版本状态。历史过程数据由 Git 历史 / Tag / Release / CI
artifact 承担，不在当前树继续堆积。

## 长期保留

| 类型 | 位置 |
|---|---|
| 当前技术合同（架构、API、安全协议、配置） | `src/ndt_slam/doc/` |
| 项目管理（状态、路线图、发布流程、治理） | `docs/project/` |
| 当前版本基线 | `docs/release/current_baseline.md` |

## 不进入 Git 当前树

以下内容由 Git history / Tag / Release / CI artifacts / server_runs 承担：

- 历史审计过程（audits）
- incident 长报告（incidents）
- branch handoff（investigations）
- runtime metrics dump
- Bag replay report
- CI generated report
- 自动验收输出（metrics.json / final_report.md / CSV）
- 历史 validation 报告

## 技术文档要求

`src/ndt_slam/doc/` 下每篇文档必须：

- 描述当前 master 的真实行为
- 不引用已删除的类、文件、Topic、参数
- 不含待办事项或道路规划
