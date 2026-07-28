# NDT-SLAM Technical Documentation

Current-master reference. For historical evidence and design decisions, see
[docs/](../../../docs/).

## System

- [Architecture](architecture.md) — component graph, data flow, thread model
- [API Reference](api.md) — topics, services, parameters

## Subsystems

- [Localization Runtime](localization_runtime.md) — NDT scan matching, EKF, MotionGate
- [Cargo Tracking & Safety](cargo_tracking_and_safety.md) — detection, lifecycle, avoidance, safety contract
- [Map Lifecycle](map_lifecycle.md) — layers, sessions, static evidence, MapCommit
- [Long-term Mapping](longterm_mapping.md) — keyframes, tile persistence, sessions
- [Dynamic Filtering](dynamic_filtering.md) — human, payload-channel filters
- [Memory Guard](memory_guard.md) — memory pressure levels and actions
- [Map Postprocess](map_postprocess.md) — clean-map pipeline

## Operations

- [Configuration](configuration.md) — YAML reference
- [Deployment](deployment.md) — install, systemd, launch
- [Operations](operations.md) — runtime commands, monitoring
- [Server Monitoring](server_monitoring.md) — monitorctl, diagnostics, CSV reports
- [Server Validation Runbook](server_validation_runbook.md) — SHA-gated acceptance
- [Engineering Mapping Guide](engineering_mapping_guide.md) — site mapping workflow

## Quality

- [Testing & Acceptance](testing_and_acceptance.md) — gtest, bag, static contracts
- [Troubleshooting](troubleshooting.md) — common failure modes and recovery
- [Roadmap](roadmap.md) — planned work
