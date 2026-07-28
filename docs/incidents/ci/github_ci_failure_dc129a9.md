# GitHub CI failure evidence for dc129a9

Status: `FAIL_GITHUB_CI_REPRODUCED`

- Repository: `guolichen007/NDT-SLAM-Warehouse`
- PR: `#2`
- Workflow: `repository-and-catkin`
- Run ID: `29806291286`
- Job: `noetic`
- Job ID: `88557420965`
- Failed step: `Static repository contracts`
- Head SHA: `dc129a921cdfcbd53b461cbc58da74b653199490`
- Merge SHA checked out by Actions: `f904f41dfe81fc164f0fdcb2f8cf08f94570f77e`

## Original failure lines

The following lines are copied verbatim from the decoded GitHub Actions job
log (timestamps retained):

```text
2026-07-21T06:12:51.2892337Z fatal: detected dubious ownership in repository at '/__w/NDT-SLAM-Warehouse/NDT-SLAM-Warehouse'
2026-07-21T06:12:51.2893446Z To add an exception for this directory, call:
2026-07-21T06:12:51.2893805Z
2026-07-21T06:12:51.2894258Z \tgit config --global --add safe.directory /__w/NDT-SLAM-Warehouse/NDT-SLAM-Warehouse
2026-07-21T06:12:51.2896861Z FAIL: cannot enumerate tracked files: Command '['git', 'ls-files', '-z']' returned non-zero exit status 128.
2026-07-21T06:12:51.2981020Z ##[error]Process completed with exit code 2.
```

## Root cause and bounded fix

`actions/checkout` registered the workspace as safe outside the ROS job
container, while the static-contract Python process invoked the container's
Git against a bind-mounted repository owned by a different UID. Git therefore
rejected `git ls-files` before the cargo contract checker ran.

The workflow now registers the exact `$GITHUB_WORKSPACE` path as a global safe
directory inside the job container and invokes the platform-neutral
`scripts/regression/run_static_contracts.py` entry point. No server monitoring
workflow, `scripts/ops/`, systemd file, or server monitoring code was changed.

Ubuntu catkin build and C++ gtest remain `NOT_RUN_REQUIRES_UBUNTU` until a new
workflow run passes the static step and reaches those jobs.
