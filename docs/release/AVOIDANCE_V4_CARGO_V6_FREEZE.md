# Avoidance V4 / Cargo V6 Freeze

## Release identity

- Final code base: `b42add34ded6de9e4fe7387c2cb7957c75bb12b2`
- Final candidate: the commit containing this document; its exact SHA is
  recorded in the delivery report and the Ubuntu acceptance ledger.
- Release branch: `final/avoidance-v4-cargo-v6-freeze`
- Root cause: `C2_COMPONENT_SPLIT_NOT_GROUPED`
- Root-cause investigation: closed

## Final architecture

The accepted-component lineage introduced by `b42add3` remains the only D5
repair. It stores one previous source frame of compact component descriptors
and owns no Cargo identity, point cloud, vertical evidence, validation state,
Safety authority or map authority. `CargoPhysicalIdentityAuthority` remains
the sole owner of histories, pre-lift reference, lift confirmation and
`VALIDATED`.

This final change only closes the low-ego observability gap:

- observable ego motion keeps the original base/map world-static test;
- low ego plus a fresh `LOADED` state permits continuation-only lineage;
- low ego plus `EMPTY` remains exact-path only;
- invalid, stale, `UNKNOWN` or `INHIBIT` load state fails closed.

Load is motion observability only. It cannot select a component, create or
choose a Cargo history, break a history tie, provide vertical evidence, alter
baseline/lift, or grant ownership, Formal, Bottom, Safety, self-removal or map
authority.

## Frozen safety contracts

- Exact association has absolute priority.
- Any exact-feasible competitor vetoes lineage rescue.
- Lineage cannot create history or validate by itself.
- All vertical evidence remains current exact-product-group evidence plus the
  existing `CONTINUITY_ONLY` RAW-ROI reacquisition path.
- Original product candidates/groups and canonical exact ownership are
  unchanged.
- Self-removal, map mutation, Bottom, Safety thresholds, Obstacle/G11.2,
  NDT, EKF, Rail/Yaw and relocalization are unchanged.
- No new parameter, thread, timer, worker, queue or point-cloud history is
  introduced.

## Freeze policy

After Ubuntu acceptance, Avoidance V4 / Cargo V6 is frozen. Reopening requires
one of: wrong object authority, false CLEAR, missed positive-collision warning,
false stop in normal safe-over, crash, resource leak or map corruption.
Non-safety diagnostic imperfections and rare conservative gates are recorded
as known limitations instead of triggering further algorithm work.
