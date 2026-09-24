# 文档中心

本文件是仓库文档的唯一导航入口。

## 当前项目状态

- [project/status.md](project/status.md) — 当前实现状态、验证矩阵、剩余 blocker
- [project/document_inventory.md](project/document_inventory.md) — 文档资产清单
- [project/github_governance.md](project/github_governance.md) — GitHub 治理

## 当前技术参考

- [src/ndt_slam/doc/](../src/ndt_slam/doc/) — 当前 master 技术事实
  - [api.md](../src/ndt_slam/doc/api.md)
  - [architecture.md](../src/ndt_slam/doc/architecture.md)
  - [cargo_tracking_and_safety.md](../src/ndt_slam/doc/cargo_tracking_and_safety.md)
  - [configuration.md](../src/ndt_slam/doc/configuration.md)
  - [deployment.md](../src/ndt_slam/doc/deployment.md)
  - [localization_runtime.md](../src/ndt_slam/doc/localization_runtime.md)
  - [operations.md](../src/ndt_slam/doc/operations.md)

## 发布与冻结

- [release/](release/) — 封板、限制、发布矩阵

## 正式验证证据

- [validation/](validation/) — 不可变验收证据（绑定 SHA，不代表当前实现）

## 设计合同

- [design/](design/) — 长期设计 / 坐标约定合同

## 历史归档

- [archive/](archive/) — 历史取证，不得作为生产配置依据
  - [archive/audits/](archive/audits/)
  - [archive/incidents/](archive/incidents/)
  - [archive/legacy/](archive/legacy/)

## 目录职责

```text
src/ndt_slam/doc/   = 当前 master 技术事实
docs/validation/    = 不可变证据，不代表当前实现
docs/release/       = 封板与限制
docs/design/        = 长期设计/合同
docs/archive/       = 历史取证，不得作为生产配置依据
```
