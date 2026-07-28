# Contributing

## Branch Discipline

| Prefix | Purpose | Merge target |
|---|---|---|
| `fix/*` | Bug fixes, safety patches | `master` |
| `feature/*` | New capabilities | `master` |
| `chore/*` | Engineering, docs, CI | `master` |
| `docs/*` | Documentation-only changes | `master` |
| `refactor/*` | Code restructuring (no behaviour change) | `master` |
| `experiment/*` | Exploratory work | Do not merge |

Keep history linear. Branch from and rebase onto the latest `master`.
Never force-push `master`.

## Commit Style

Use conventional commit prefixes scoped to the subsystem:

```
fix(cargo):
fix(safety):
feat(mapping):
feat(monitor):
chore(repo):
docs:
test:
ci:
```

## Safety-contract PR Requirements

Any PR that touches cargo safety behaviour must include:

```
INPUT_SHA:
OUTPUT_SHA:

Safety contract impact:
- [ ] Code 14 (CLEAR)
- [ ] Code 17 (NEAR_3M)
- [ ] Code 18 (NEAR_5M)
- [ ] Code 30-35 (FAULT/INVALID)
- [ ] Cargo point removal from registration
- [ ] Static map exclusion
- [ ] MapCommit exclusion
- [ ] Localization
- [ ] Message schema

Runtime behaviour changed: YES / NO
Rollback SHA:
```

## Verification Checklist

Before requesting review:

- [ ] `git diff --check`
- [ ] `python3 scripts/regression/run_static_contracts.py`
- [ ] `python3 scripts/regression/check_repository_integrity.py`
- [ ] `python3 scripts/regression/check_cargo_safety_e2e.py`
- [ ] `python3 -m compileall scripts tests tools`
- [ ] `python3 -m unittest discover`
- [ ] Ubuntu clean catkin build
- [ ] Ubuntu gtest (`catkin run_tests && catkin_test_results`)
- [ ] Bag acceptance (if runtime behaviour changed)
- [ ] Server runtime (if runtime behaviour changed)
- [ ] Field case (if safety contract changed)

Unavailable environment-specific checks must be marked **NOT_RUN**, never
reported as passed. Windows-only checks are not sufficient for ROS builds,
tests, bag, or field verification.

## Server Validation Evidence

Server validation runs must preserve:
- `run_manifest.json`
- `reports/final_summary.json`
- `reports/final_report.md`

Attach the exact SHA. Never label an unexecuted item as passed.
Generated maps, bags, and `server_runs/` do not belong in Git.
