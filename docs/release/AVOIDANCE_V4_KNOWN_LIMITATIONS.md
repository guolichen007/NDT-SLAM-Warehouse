# Avoidance V4 Known Limitations

The following behaviors are accepted for the frozen field version and do not
automatically reopen development:

- a rare isolated association or lineage gate;
- a minor missed lineage frame that does not break the core working window;
- conservative UNKNOWN/Code30 for complex started-loaded or long-Cargo cases;
- non-safety diagnostic or telemetry imperfections;
- minor runtime metric fluctuations within the existing acceptance contract;
- further theoretical improvements to Cargo, D1 ROI or Obstacle/G11.2 that are
  not required by the fixed acceptance matrix.

These limitations never authorize wrong ownership, false CLEAR, a missed
positive-collision warning, false stop in normal safe-over, map corruption,
crash or unbounded resource growth. Any such outcome is a P0/P1 release
blocker and requires a hotfix from the freeze tag.
