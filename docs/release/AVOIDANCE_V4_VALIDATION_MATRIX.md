# Avoidance V4 Final Validation Matrix

All acceptance runs use one candidate SHA, binary, configuration, oracle and
analyzer. Codex does not replay Bags. Ubuntu claims the acceptance ledger
before the first Bag and does not modify artifacts between scenarios.

## Pre-Bag code gates

| Gate | Requirement |
| --- | --- |
| Clean build | PASS |
| Full GTest and `catkin_test_results` | zero errors/failures/skips |
| Avoidance V4 firewall | PASS |
| Cargo Safety E2E | PASS |
| Integration contracts | PASS |
| `git diff --check` | PASS |

## Fixed four-Bag matrix

| Scenario | Minimum freeze requirement |
| --- | --- |
| 大件 | Identity reaches stable `VALIDATED`; maximum lift confirmation is at least 4; lineage contributes where required; wrong object, false exact ownership and thickness-as-lift are zero |
| 无 | Applicable safe-over window has no false 17/18/29, no persistent Code30 that prevents normal work, no environment object promoted as Cargo, and every CLEAR has valid clearance authority |
| 有 | The positive-collision window produces an evidence-backed 17, 18 or 29; `FALSE_CLEAR=0` |
| 长件 | Normal operation is preferred; conservative UNKNOWN/Code30 is acceptable, but wrong object, false CLEAR, crash and unauthorized map deletion are zero |

If a P0 failure appears in 大件, acceptance stops. Otherwise the same SHA runs
`无 -> 有 -> 长件` without code or configuration changes.

## One-hour closure soak

After the four Bags, run one hour of loop replay with the same candidate.
Required results:

- process crashes, fatal exceptions and node restarts: zero;
- thread count, queues and histories: no sustained growth;
- RSS: no sustained monotonic growth;
- no new NDT/EKF regression;
- existing runtime performance acceptance remains satisfied.

The one-hour closure soak freezes this field version. It is not a substitute
for later production reliability certification.
