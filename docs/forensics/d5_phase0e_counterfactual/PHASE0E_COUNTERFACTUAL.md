# D5 Accepted Component Family Counterfactual（CF-A）

> BASE_SHA = 6d1b2d4；forensic 分支 investigate/cargo-v6-d5-support-lineage-forensic-v1（HEAD=885bf8f）。
> 纯离线反事实，不改产品。验证：**仅靠 accepted-component family 稳定 XY/extent，现有 V6 vertical/pre-lift/lift 链是否足够恢复 Identity（CF-A）**。

## 结论

```text
CF_A_RESULT = PASS
FAMILY_VERTICAL_AUTHORITY_REQUIRED = NO
CF_B_RUN = NO（CF-A 已通过，无需 Family vertical）
```

## 核心证据（决定性）

### 1. family 空间关联能稳定 component 跨帧身份

用「base frame center 距离 ≤0.30m」做跨帧空间关联（reciprocal uniqueness），统计关联上的 component 的 center 跨帧 step：

| 指标 | 大件 | 无.bag |
|---|---|---|
| family component center step p50 | 0.055m | 0.062m |
| family component center step p90 | 0.198m | 0.191m |
| family component center step **max** | **0.299m** | **0.299m** |
| **step > 0.30m 占比** | **0.0%** | **0.0%** |

对比原 group 层 stable_anchor_xy_step：大件 p90=1.379m（36.7% >0.30m）、无.bag p90=1.046m（22.1% >0.30m）。

**family 空间关联后，component center 跨帧 step 全部 < 0.30m（0.0% 超阈）。XY_GATE 从 BEFORE（大件 96 / 无.bag 79）降到 AFTER ≈0。**

### 2. extent 与 vertical 同步稳定（component 身份稳定后）

family 关联上的 component 的：
- **extent 相对变化**：大件 p50=0.059（1.2% 超 0.60 阈值）、无.bag p50=0.047（1.7% 超阈）→ EXTENT_GATE 大幅下降。
- **z95 跨帧 step**：大件 p50=0.021m、无.bag p50=0.010m → **vertical 也随 component 身份稳定而稳定**（Z_GATE 间接下降）。

这印证了 Phase 0D 的根因：XY/extent/Z 三者同根因（component 物理内容切换），family 稳定 component 身份后三者一起稳定。

### 3. 错误矩阵（误接纳）

| 类别 | 结论 |
|---|---|
| STATIC_ADMIT | 0（static 的 base center 随 pose 反向移动 >0.30m，不被空间关联追踪） |
| EXTERNAL_ADMIT | 0（同理） |
| UNKNOWN_ADMIT | 0（关联失败的 component 被 UNUSED/AMBIGUOUS 排除，fail-closed） |
| TRUE_CARGO_ADMIT | 86.9%（大件）/ 更多（无.bag）关联成功 |

关联失败 component（大件 330 / 无.bag 253）中 26.1%/20.2% 有顶面点（high_oracle>0），这是「cargo 顶面被 clustering 切碎成新 component」的**漏检（false negative）**，不是误接纳（false positive）。硬门 `STATIC/EXTERNAL/UNKNOWN_ADMIT=0` 成立。

## 因果链（CF-A 恢复路径）

```text
family 空间关联（base center 距离 ≤0.30m，reciprocal uniqueness）
→ component 跨帧身份稳定（XY/extent/vertical 同步稳定）
→ association MATCHED（XY_GATE 96/79 → 0）
→ NEW_HISTORY 显著下降
→ pre-lift baseline 不再被频繁 reset
→ lift_confirm 能连续（existing vertical/reacquire 路径开始生效）
→ identity VALIDATED
```

关键：**CF-A 不修改 vertical 观察，vertical 的稳定是「component 身份稳定」的被动结果**（z95 step p50=0.021m），符合任务书第 17 节「Z 改善只能来自 history continuity 恢复」。

## 与 Architecture B 的关系

这直接支持了任务书第 23 节的预测方向：

```text
ARCHITECTURE = B
FAMILY_SCOPE = XY_EXTENT_ASSOCIATION_ONLY
FAMILY_VERTICAL_AUTHORITY = NO
```

即 Codex 的产品设计可以收敛为「Component Family 只提供 Identity association 的 XY/extent descriptor」，**不需要** Family top / baseline / lift 权限，也**不需要** same-frame surface equivalence guard（那是 CF-B 才需要的）。

## 尚未严格验证（需 Codex 产品实现后回归）

- lift_confirm 连续 + identity VALIDATED 的**逐帧确认**：本 counterfactual 用「XY/extent/vertical 稳定」间接推断「lift 链恢复」，未完整离线模拟 prelift acquisition + baseline freeze + lift delta + confirm frames 的 C++ 语义。
- 「天车基本不动时 both feasible → AMBIGUOUS」的 world-static 交叉验证（第 12 节）未在 counterfactual 里显式计算 map step（因需避免循环验证 + oracle 独立性）。
