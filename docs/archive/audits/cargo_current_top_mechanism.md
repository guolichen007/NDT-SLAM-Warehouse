# Cargo current-top mechanism audit

Audit baseline: `fcb90ebadf927cbc4809106ee0bea97be29242d7`
Product parent: `df45c397cf02dbd3dfbf4d7f7549929f58482366`

This audit covers the product path before Cargo Vertical Evidence V2. The
forensic commit adds CSV diagnostics only and does not feed data back into a
decision.

## Mechanism

`detectCargoAroundOdomAnchor()` receives the unvoxelized, finite, range-filtered
cloud after the near-field filter. It crops the fixed/adaptive hook ROI to
`search_z_min..search_z_max`, optionally applies external-ground HAG filtering,
voxelizes at 5 cm, extracts Euclidean components, builds component-fusion
hypotheses, and selects one hypothesis with the existing Cargo identity ranker.

The selected hypothesis becomes `HookCargoDetection::core_points_base`. The
same point set is used to estimate the current footprint, `z05/z50/z95`, top
support, and top coverage. `z95` is copied to
`CargoBottomObservation::current_top_z_base`; the entire selected point set is
copied to `CargoBottomObservation::points_base`.

## Required answers

```text
GROUND_REFERENCE_SOURCE=
spatially distributed per-cell robust-low samples in a rectangular ring
outside the payload ROI, evaluated in base_link

GROUND_REFERENCE_INVALID_REASON=
NOT_DISAMBIGUATED_BY THE PAIRED B0 TELEMETRY. A reference is invalid when the ring
has too few populated cells, lacks three quadrants/opposite sides, has more
than ground_max_range_m vertical spread, or violates the configured expected
base_link ground height. Paired evidence records valid=0 for every frame but
does not record which gate rejected it. B1 adds cells/points/quadrants,
opposite-side coverage, range, and a deterministic rejection reason to the
forensic trace so Ubuntu replay can resolve the gate without changing HAG.

GROUND_REFERENCE_REQUIRED_FOR_HAG=YES

WHEN_GROUND_INVALID_CURRENT_BEHAVIOR=
the detector copies the unfiltered ROI crop into filtered_cloud and continues
component extraction ("bypass HAG prefilter")

DET_Z95_INPUT_POINT_SET=
the full identity-selected (possibly fused) voxel component after the optional
HAG stage

TOP_SUPPORT_INPUT_POINT_SET=
the same full selected component; points in the configured band below z95 are
counted and binned in XY

LOW_POINT_REJECTION_CURRENTLY=
only search_z_min and HAG when an external ground reference is valid; there is
no independent supported-upper-surface or locked-footprint vertical slab

LOCKED_FOOTPRINT_USED_FOR_VERTICAL_FILTER=NO
```

The existing locked/current footprint participates in identity association and
later pose support, but it is not used to clean the vertical evidence copied to
BottomFusion. With ground invalid, a component containing predominantly low
returns can therefore have low `z95`; the same contamination simultaneously
affects the POINTS source and current-top source.

## B1 boundary

Cargo Vertical Evidence V2 is a stateless SHADOW extractor downstream of the
existing identity selection. It does not alter selector scores, tracking,
lifecycle, BottomFusion source priority, warning thresholds, obstacle input,
or product geometry. It requires a supported upper band with both point and XY
coverage. A current-lifecycle frozen thickness may bound a slab only after that
surface exists; it cannot create a top. Invalid ground is never replaced by
zero. The B1 build rejects `shadow_only: false`.
