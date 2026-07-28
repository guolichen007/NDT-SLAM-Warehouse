## 修改目的

## 基线与输出

INPUT_SHA:
OUTPUT_SHA:

## 是否改变运行行为

- [ ] 否
- [ ] 是（说明）：

## 安全合同影响

- [ ] Code 14（CLEAR）
- [ ] Code 17（NEAR_3M）
- [ ] Code 18（NEAR_5M）
- [ ] Code 30–35（FAULT/INVALID）
- [ ] 货物点从 registration 剔除
- [ ] 静态地图排除
- [ ] MapCommit 排除
- [ ] 定位
- [ ] 消息 Schema
- [ ] 几何授权规则

## 配置变化

- [ ] 无
- [ ] 如下：

## 验证

- [ ] `git diff --check`
- [ ] `python3 scripts/regression/run_static_contracts.py`
- [ ] `python3 scripts/regression/check_repository_integrity.py`
- [ ] `python3 scripts/regression/check_cargo_safety_e2e.py`
- [ ] `python3 scripts/regression/check_docs_contract.py`
- [ ] `python3 -m compileall scripts tests tools`
- [ ] `python3 -m unittest discover`
- [ ] Ubuntu clean catkin build
- [ ] Ubuntu gtest
- [ ] Bag 验收
- [ ] 服务器运行
- [ ] 现场验证

## 未执行项目

（列出不适用或无法执行的项目）

## 回滚 SHA
