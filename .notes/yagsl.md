---
id: yagsl
title: YAGSL (Yet Another Generic Swerve Library)
schema_version: 2
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, swerve, vendordep]
aliases: [yagsl, yet another generic swerve library, yagsl-2026.1.20]
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
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
---

# YAGSL (Yet Another Generic Swerve Library)

> The swerve-drive library the 2026 robot is built on; it owns the JSON swerve config under `deploy/swerve/`, not the Java constants.

## Context

YAGSL ("Yet Another Generic Swerve Library") is the primary drivetrain library for the FRC-2026 robot, pulled in via the `yagsl-2026.1.20` vendordep. It drives `SwerveSubsystem.java`. Unlike most subsystems, the swerve configuration is data-driven: module zeroing, IMU type, drive-inversion flags, and PID wiring live in JSON under `2026Robot/src/main/deploy/swerve/` (e.g. `modules/physicalproperties.json`), which YAGSL parses at boot. Editing the Java alone will not change drivetrain behavior that the JSON owns.

Docs: https://broncbotz3481.github.io/YAGSL-Example/

## Observations

- [registry] Vendordep `yagsl-2026.1.20.json` is present in `2026Robot/vendordeps/`. #frc
- [registry] YAGSL reads swerve config from `deploy/swerve/*.json` (module zeroing, IMU type, drive inversion, PID) — these JSON files are the source of truth, not `Constants.java`. #swerve
- [registry] Drivetrain hardware per repo docs: SDS MK4 L1 (drive 8.14:1, angle 12.8:1, 4" wheel), NEO drive + angle motors, CTRE CANcoders. #swerve
- [registry] `physicalproperties.json` is one of the most-churned files in git history (4 commits) — competition tuning happens here. #swerve

## Open Questions

None from the corpus — the L1/stock-orientation question (vs the planned 5.8267:1 inversion-plate variant) is already settled in `frc-2026-shooter-swerve-hardware` (repo docs win).

## Relations

- relates-to [[frc-2026]] (the swerve drivetrain of the 2026 robot)
- relates-to [[navx-gyro]] (YAGSL field-centric control uses the NavX IMU)
- relates-to [[photonvision]] (pose fusion feeds the YAGSL odometry on the RIO)
