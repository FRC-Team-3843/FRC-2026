---
id: frc-2026-common-build-errors
title: FRC-2026 common build errors (2026 migration lookup)
schema_version: 2
artifact_kind: reference
created: 2026-06-23T00:54:31Z
updated: 2026-06-23T00:54:31Z
valid_until: null
author: claude
session: acc-monolith-decomp-pilot-20260623
model: claude-opus-4-8
model_basis: confirmed
tags: [frc, reference, build, errors, migration]
aliases: [frc build errors, cansparkmax error, controlmode error, 2026 migration errors]
source: compiled from FRC-2026 2026Robot build + 2026 REBUILT migration experience (decomposed from .standards.md)
status: active
supersedes: null
confidence: 60
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
load_profile: scope_entry
load_priority: 195
source_rel: FRC-2026\.standards.md
scope: FRC-2026
---

# FRC-2026 common build errors (2026 migration lookup)

> Quick lookup for the recurring 2026-migration build errors and their fixes.

## Context
Decomposed from the `.standards.md` "Common Build Errors (2026 Migration)" section during the ACC monolith-decomposition pilot. The underlying API rules live in `frc-2026-api-standards`, `revlib-sparkmax`, and `phoenix5-6-coexistence`; this is the fast error→fix table.

## Lookup table

| Error | Cause | Fix |
|-------|-------|-----|
| `cannot find symbol: CANSparkMax` | Using old API | Change to `SparkMax` |
| `cannot find symbol: method setInverted(boolean)` | Direct setter removed | Use `SparkMaxConfig` |
| `incompatible types: ControlMode` | Phoenix5 vs Phoenix6 | Use Phoenix6 control request objects |

## Notes for Future Sessions
Forbidden legacy patterns (do NOT copy from FRC-2024/2025): global static subsystem access (`Robot.driveSubsystem.drive(...)`), monolithic `Robot.java` with hardware in it, old REVLib (`CANSparkMax`, `setInverted`, `setIdleMode`). Correct 2026 idioms: dependency injection into commands, `SparkMax` + `SparkMaxConfig`, command-based `Robot.java` that delegates to `CommandScheduler`. Full forbidden/correct examples were in `STANDARDS.md` "Legacy Code Warnings" — the rule itself is captured in `frc-2026-api-standards`.

## Relations
- relates-to [[frc-2026]] (build troubleshooting for the 2026 robot)
- relates-to [[frc-2026-api-standards]] (the standards behind these errors)
- relates-to [[revlib-sparkmax]] (the CANSparkMax→SparkMax migration)
- relates-to [[phoenix5-6-coexistence]] (the Phoenix5/6 namespace error)
