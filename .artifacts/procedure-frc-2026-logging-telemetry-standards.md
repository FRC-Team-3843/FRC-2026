---
id: frc-2026-logging-telemetry-standards
title: FRC-2026 logging + telemetry standards (DataLog, NT naming)
schema_version: 2
artifact_kind: memory
memory_class: procedural
created: 2026-06-23T00:54:31Z
updated: 2026-06-23T00:54:31Z
valid_until: null
author: claude
session: acc-monolith-decomp-pilot-20260623
model: claude-opus-4-8
model_basis: confirmed
tags: [frc, standards, logging, telemetry, networktables]
aliases: [frc datalog, nt naming convention, subsystem/category/value, telemetry standards]
status: active
supersedes: null
confidence: 58
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
load_profile: scope_entry
load_priority: 130
source_rel: FRC-2026\.standards.md
enforceability: preferred
---

# FRC-2026 logging + telemetry standards (DataLog, NT naming)

> Start DataLog in `Robot.java` behind an enable flag; name NetworkTables values with structured hierarchical paths (`Subsystem/Category/Value`).

## Context
Decomposed from the `.standards.md` "Logging & Telemetry Standards" section during the ACC monolith-decomposition pilot. The Elastic-Dashboard / live-tuning portions of that section are covered by `elastic-dashboard` and `frc-2026-elastic-layout-editing`; this note carries the DataLog + NT-naming standard.

## Observations
- [decision] Initialize logging in the `Robot.java` constructor, gated by a flag: `if (Constants.LoggingConstants.ENABLE_LOGGING) { DataLogManager.start(); DriverStation.startDataLog(DataLogManager.getLog()); }` #logging. Source: `STANDARDS.md` DataLog Usage.
- [decision] Enable flags live in `Constants.LoggingConstants` (`ENABLE_LOGGING`, `ENABLE_DRIVE_TELEMETRY`) #logging. Source: `STANDARDS.md` Enable Flags in Constants.
- [decision] NetworkTables values use structured hierarchical paths: pattern `"Subsystem/Category/Value"`, e.g. `SmartDashboard.putNumber("Drive/Velocity/X", vx)`, `SmartDashboard.putBoolean("Vision/HasTarget", hasTarget)` #telemetry. Source: `STANDARDS.md` NetworkTables Naming Convention.
- [decision] Elastic Dashboard is the default/recommended driver-station dashboard (distributed with WPILib 2026; NT4 + Shuffleboard-widget compatible). The SmartDashboard *application* is deprecated (removal planned 2027) but the `SmartDashboard` *API class* remains valid for NT publishing #dashboard. Source: `STANDARDS.md` Dashboard Standards.

## Notes for Future Sessions
Elastic launch path: VS Code → `Ctrl+Shift+P` → `WPILib: Start Tool` → `Elastic`. For the layout-JSON editing standard (widget types, NT-path table, ARGB colors, agent editing rules), see `frc-2026-elastic-layout-editing`.

## Relations
- relates-to [[frc-2026]] (telemetry standard for the 2026 robot)
- relates-to [[elastic-dashboard]] (the default dashboard + live tuning)
- relates-to [[frc-2026-elastic-layout-editing]] (the layout-JSON editing standard)
- relates-to [[advantagescope]] (the `.wpilog` viewer that consumes DataLog output)
