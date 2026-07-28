# 发布流程

## 版本标识

| 标识 | 含义 | 何时使用 |
|---|---|---|
| `development SHA` | 开发中的提交 | 日常开发 |
| `validation SHA` | 已经过现场验证的提交 | 现场运行取得正向证据后 |
| `validation-obstacle-avoidance-YYYYMMDD` | 验证 Tag | 现场验证完成后打 annotated tag |
| `RC` | 候选发布版本 | 所有质量门禁通过、Bag 验收通过 |
| `vX.Y.Z` | 正式 production release | RC + 所有现场验收通过 |

## validation tag 不等于 production release

`validation-obstacle-avoidance-20260728` 表示 `8d7d7ee` 这个版本在现场实际运行并取得了避障正向证据。

它不表示：
- 系统所有能力都已验收
- 质量门禁全部通过
- 可以安全部署到任何现场

## 打 validation tag 的条件

- 该 SHA 已在现场服务器实际运行
- 取得了预期行为的正向证据（日志、截图、统计数据）
- 记录运行环境、日期、证据类型

## 打 RC tag 的条件

- 静态合同全部通过
- Ubuntu clean build 通过
- gtest 0 active failure
- Bag 验收通过
- Server soak 通过

## 打 production release tag 的条件

- RC 条件全部满足
- 所有 P0/P1 现场验收通过
- 完整的端到端同步日志验证

## 回滚

若怀疑安全行为退化，回滚到最近验证 Tag：

```bash
git checkout validation-obstacle-avoidance-20260728
```

重新编译、部署、验证。
