# Phase 0C Counterfactual — 未评估

## 结论

```text
D5_COUNTERFACTUAL_GATE_IMPROVEMENT = NOT_EVALUATED
```

## 原因

任务书第 24 节要求「如果找到可用 signal，重新计算 identity gate（XY_GATE / EXTENT_X / EXTENT_Y / EXTENT_Z / Z_GATE）BEFORE/AFTER」。

但本轮结论是：

```text
TEMPORAL_COMPOSITE_SIGNAL = PROVEN_OFFLINE_ONLY
```

temporal signal 在**离线**层面能区分 cargo/static（map 位移 14m vs 0m），但「实时使用」需要「跨帧关联」机制，而：

1. 跨帧关联（identity association）正是当前 blocker（representative center 抖动 → history 寿命 1~2 帧）。
2. mature static 不可用（temporally_mature=0）。

因此「可用的实时 signal」尚未确立，counterfactual identity gate 无法有意义地计算 BEFORE/AFTER。

## 下一步依赖

若要推进到 counterfactual，需先确立一个「实时可用」的 signal，至少满足：
- 离线 temporal signal 的实时化（跨帧关联方案）
- 或 mature static 的可用性（非 empty_map 冷启动采集）

在此之前，counterfactual identity gate 保持 NOT_EVALUATED。
