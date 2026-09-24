# 文档资产清单

> 生成于 2026-09-24，BASE_SHA=`18619e7`。本清单记录当前文档资产的分类与处置，
> 用于仓库文档治理（历史资料降级为 archive、当前事实重建为单一可信入口）。

## 分类定义

- `CURRENT`：当前 master 技术事实，保留并保持更新。
- `FORMAL_EVIDENCE`：正式不可变验收证据，绑定 SHA，不改写。
- `RELEASE_RECORD`：封板/限制/发布矩阵，保留。
- `HISTORICAL_FORENSIC`：历史取证，降级到 `docs/archive/`。
- `SUPERSEDED`：被正式文档取代，归档并加 archived header。
- `DELETE_CANDIDATE`：唯一信息已覆盖且无安全取证价值，删除（git 历史可恢复）。

## 根级

| 文件 | 分类 | action |
|---|---|---|
| README.md | CURRENT | 重写为入口文档，修正 baseline 为 `baseline-field-legacy-20260923` |
| CHANGELOG.md | RELEASE_RECORD | 重构，[Unreleased] 清空，补 `baseline-field-legacy-20260923` |
| CONTRIBUTING.md | CURRENT | 保留 |
| SAFETY.md | CURRENT | 解耦历史 17/18 证据与当前 baseline |
| SECURITY.md | CURRENT | 保留 |

## src/ndt_slam/

| 文件 | 分类 | action |
|---|---|---|
| src/ndt_slam/README.md | CURRENT | 保留（已是 ROS1 入口） |
| src/ndt_slam/README_cargo_warning_v1.md | SUPERSEDED | 归档到 `docs/archive/legacy/cargo_warning_v1.md` |
| src/ndt_slam/config/README.md | SUPERSEDED | 重写为 ROS1 Noetic 配置索引 |
| src/ndt_slam/doc/*.md | CURRENT | 保留（当前 master 技术说明） |

## docs/

| 目录/文件 | 分类 | action |
|---|---|---|
| docs/README.md | CURRENT | 重写为唯一文档导航 |
| docs/project/ | CURRENT | 重写 status/roadmap/known_issues/governance；保留 documentation_policy/release_process |
| docs/design/ | CURRENT | 保留（长期设计/合同） |
| docs/release/ | RELEASE_RECORD | 保留，新增 README.md |
| docs/validation/ | FORMAL_EVIDENCE | 保留正式证据；yaw_map_frame_convention.md 移 design |
| docs/audits/ | HISTORICAL_FORENSIC | `git mv` → docs/archive/audits/ |
| docs/incidents/ | HISTORICAL_FORENSIC | `git mv` → docs/archive/incidents/ |
| docs/investigations/ | HISTORICAL_FORENSIC | handoff.yaml 归档或删除 |
| docs/decisions/ | HISTORICAL_FORENSIC | 归档到 archive/decisions/（历史决策） |

## 待判定

- `docs/investigations/integrated_cargo_identity_shadow_handoff.yaml`：
  若唯一信息已覆盖 → DELETE，否则归档。
- `docs/decisions/`：检查是否为当前设计决策（保留）还是历史（归档）。
