# D5 Forbidden Changes — 禁止改动清单

> 这些改动只能把 fail-closed 变成错误合并风险。Codex 的产品修复不得触碰以下任何一项。

## 聚类参数（D5 直接相关）

```
component_cluster_tolerance_m: 0.20  （不得改成 0.25/0.30/0.35）
weak_min_points / min_cluster_size: 10  （不得改成 5/3/1）
voxel leaf: 0.05  （不得改）
max_cluster_size: 8000  （不得改）
扩大 Cargo ROI / 修改 HAG 过滤
```

## 绝对阈值 / 启发式

```
禁止用 z>1.3m 判断 Cargo（仅可作 bag oracle 标记）
禁止用 extent≈3m 判断 Cargo
禁止 prefer-high（偏好高点）
禁止沿用 previous component / 历史 OBB 合并
禁止用 predicted Cargo 强行归属
```

## IdentityAuthority（frozen，正确 fail-closed）

```
maximum_xy_step_m=0.30
Z gate / EXTENT gate / gap / lift confirm_frames=4
```

## 其它产品逻辑（frozen 文件，见 D5_CODE_LOCATIONS.md）

```
cargo_physical_identity_authority.cpp/.hpp
cargo_bottom_fusion.cpp / cargo_vertical_evidence 链
cargo_obstacle_tracker.cpp / obstacle_cluster_tolerance_m=0.25 / far-history
```

## D1 ROI / G11.2 Obstacle（本轮明确不修）

```
D1_STATUS = POSSIBLE_D5_DOWNSTREAM（ROI clipping 28% 是无.bag 特有，D5 下游）
G11_2_STATUS = OUT_OF_SCOPE_FOR_D5（obstacle fragmentation 是 later risk）
```

## 产品修复的硬语义约束（如果 Codex 要做 topology recovery）

```
PRIMARY_TO_PRIMARY_MERGE = FORBIDDEN
UNIQUE_TOPOLOGY_OWNER = 一个 weak island 只能有一个 primary root
AMBIGUOUS_BRIDGE = 连接 >=2 primary 的 weak set 整组拒绝
CURRENT_FRAME_ONLY = 禁止消费历史/prediction
绝对 Z 禁止进入产品算法
```

## 本轮角色边界

```
Ubuntu ClaudeCLI = ROOT-CAUSE / FORENSIC / BAG VALIDATION OWNER
产品算法实现 = 暂停，交 Codex
Ubuntu 不得 commit / push 任何产品修复
```
