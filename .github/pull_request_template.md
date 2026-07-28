## Purpose

## Base / Head

INPUT_SHA:
OUTPUT_SHA:

## Runtime behaviour changed

- [ ] No
- [ ] Yes (describe):

## Safety contract impact

- [ ] Code 14 (CLEAR)
- [ ] Code 17 (NEAR_3M)
- [ ] Code 18 (NEAR_5M)
- [ ] Code 30–35 (FAULT/INVALID)
- [ ] Cargo point removal from registration
- [ ] Static map exclusion
- [ ] MapCommit exclusion
- [ ] Localization
- [ ] Message schema
- [ ] Geometry authority rules

## Configuration changes

- [ ] None
- [ ] Listed below:

## Verification

- [ ] `git diff --check`
- [ ] `python3 scripts/regression/run_static_contracts.py`
- [ ] `python3 scripts/regression/check_repository_integrity.py`
- [ ] `python3 scripts/regression/check_cargo_safety_e2e.py`
- [ ] `python3 -m compileall scripts tests tools`
- [ ] `python3 -m unittest discover`
- [ ] Ubuntu clean catkin build
- [ ] Ubuntu gtest
- [ ] Bag acceptance
- [ ] Server runtime
- [ ] Field case

## NOT RUN

(List any checks that are not applicable or could not be executed.)

## Rollback SHA
