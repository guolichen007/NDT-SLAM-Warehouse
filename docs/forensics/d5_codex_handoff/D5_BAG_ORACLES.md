# D5 Bag Oracles — 物理真值（仅分析用，绝不写入产品代码）

来源：已从 shadow_integrated 提取归档的 oracle JSON。这些 window/真值只用于 bag 取证分类，禁止进入产品算法。

## 大件（调运大件.bag，本 handoff 唯一 replay 对象）

- **oracle 类型**：positive_control（单候选 96.8%）
- **window**：1778303489.58 ~ 1778303538.98（约 49s，起升段）
- **真实 cargo**：size 1.97 × 1.56（厚大件），单候选场景
- **ground truth**：cargo 从静止 → 吊起 → 悬空 → 放下，135s 完整

## 无.bag（safe-over 负样本，本 handoff 未 replay）

- **oracle 类型**：negative_safe_over
- **window**：1787034958.42 ~ 1787035076.80
- **true cargo lift**：0.29 → 1.75（从障碍上方通过）
- **wrong static**：z ≈ 0.23

## 有.bag（positive_collision 正样本，本 handoff 未 replay）

- **oracle 类型**：positive_collision
- **window**：1787034797.41 ~ 1787034872.82
- **true cargo lift**：0.27 → 0.95（碰到障碍悬停）
- **wrong static**：z ≈ 0.28

## 长件（调运长件.bag，本 handoff 未 replay）

- **oracle 类型**：long_geometry
- **window**：1778217252.99 ~ 1778217338.29
- **true cargo**：Y（长轴 Y），wrong X（418/419 帧选错）
- **lift 不区分方向**（放下方向场景）

## ORACLE_HIGH_SURFACE 标记说明

`z>=1.3m` 仅在本 forensic 里作为 `oracle_high_surface` 标记（d5_weak_fragments.csv 一列），用于区分「cargo 顶面碎片」与「贴地静态结构」。**这不是产品算法标准**，只用于离线分类 F1-F4。
