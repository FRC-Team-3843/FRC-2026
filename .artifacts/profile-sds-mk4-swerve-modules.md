---
id: sds-mk4-swerve-modules
title: SDS MK4 swerve modules (L1 stock)
schema_version: 2
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, swerve, hardware]
aliases: [mk4, sds mk4, swerve modules, mk4 l1, swerve drive specialties]
status: active
supersedes: null
confidence: 60
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
model: claude-opus-4-8
model_basis: confirmed
provenance:
  harvest: deterministic
  recall-extract: claude-sonnet-4-6
  find-missing: claude-sonnet-4-6
  precision-judge: claude-opus-4-8
lifecycle: active
scope: FRC-2026
load_profile: on_demand
---

# SDS MK4 swerve modules (L1 stock)

> The physical swerve modules — Swerve Drive Specialties MK4, L1 ratio, stock orientation; the "inversion plate" variant was planned but never built.

## Context

The drivetrain uses Swerve Drive Specialties MK4 modules in the **L1** configuration, stock orientation: drive 8.14:1, angle 12.8:1, 4" wheel. Each module has a NEO drive motor, a NEO angle motor, and a CTRE CANcoder for absolute steering position. The gearing question is settled in the scope: repo docs (L1, 8.14:1/12.8:1) win. The 5.8267:1 + custom inversion-plate setup was the *planned* inversion that a subteam failed to complete; the team improvised with stock modules and never flipped them. Full resolution lives in `frc-2026-shooter-swerve-hardware`.

## Observations

- [registry] SDS MK4, L1 stock: drive 8.14:1, angle 12.8:1, 4" wheel. #swerve
- [registry] Per module: NEO drive (REVLib), NEO angle (REVLib), CTRE CANcoder (Phoenix6). #hardware
- [registry] CAN allocation: swerve drive+angle 11-18, CANcoders 19-22. #can
- [registry] The L1-vs-inversion-plate question is closed — repo docs win (see shooter-swerve-hardware note). #swerve

## Open Questions

None — the historical gearing divergence is resolved in `frc-2026-shooter-swerve-hardware`.

## Relations

- relates-to [[frc-2026]] (the swerve modules of the 2026 robot)
- relates-to [[yagsl]] (these modules are configured/driven by YAGSL)
- relates-to [[frc-2026-shooter-swerve-hardware]] (gearing resolution)
