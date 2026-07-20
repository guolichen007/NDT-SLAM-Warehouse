# Contributing

This repository carries a typed cargo-safety contract. Changes to warning
codes, the 3 m / 5 m distance bands, the 0.8 m clearance, message schema, or
source-validation fail-safe require an explicit safety-contract review.

Use a topic branch and keep history linear. Before requesting review, record
the exact target SHA and run the repository checks:

```bash
git diff --check
python3 scripts/regression/check_yaml_duplicate_keys.py
python3 scripts/regression/check_repository_integrity.py
python3 scripts/regression/check_cargo_safety_e2e.py
```

ROS Noetic builds/tests and Bag/server results must be reported separately;
do not mark them passed from a Windows-only checkout. Never force-push master.
Release candidates are reviewed by PR and then integrated with
`git merge --ff-only` after required checks pass.

Server validation evidence must include `run_manifest.json`,
`reports/final_summary.json`, and `reports/final_report.md`. Attach the exact
SHA and preserve PASS/FAIL/NOT_RUN values; never label an unexecuted Ubuntu,
Bag, or soak item as passed. Generated maps, bags, and `server_runs/` do not
belong in Git.
