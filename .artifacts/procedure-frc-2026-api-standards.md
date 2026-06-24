---
id: frc-2026-api-standards
model: unattributed
model_basis: unattributed
title: FRC-2026 coding standards — 2026 motor APIs, no Phoenix Pro, no warning suppression
schema_version: 2
artifact_kind: memory
memory_class: procedural
created: 2026-06-12T06:30:00Z
updated: 2026-06-12T06:30:00Z
valid_until: null
author: claude
session: phase7-onboarding-20260612
tags: [frc, standards, architecture, config]
aliases: [2026 strict api, phoenix pro forbidden, sparkmax not cansparkmax, command-based di]
status: active
supersedes: null
confidence: 57
source_basis: conversation
human_edited: false
sensitivity: normal
decisions: []
scope: FRC-2026
---

# FRC-2026 coding standards — 2026 motor APIs, no Phoenix Pro, no warning suppression

> The repo enforces a strict 2026 standard set: command-based + dependency injection only, current-2026 motor APIs (`SparkMax`/`SparkMaxConfig`, Phoenix6 control requests), an explicit ban on Phoenix Pro features, and zero warning suppression. `STANDARDS.md` is the team's single source of truth and the "read first" file.

## Context
Distilled from `STANDARDS.md` (and reinforced by `README.md`/`.project-context.md`) during ACC Phase 7 onboarding. These are durable team rules, not implementation detail.

## Observations
- [decision] Strict command-based architecture with dependency injection; NO global static subsystem access (e.g. `Robot.driveSubsystem`); `RobotContainer` is the ONLY place subsystems/commands are instantiated and bound #architecture (testability + clean separation). Source: `STANDARDS.md` Architecture Standards; `README.md` Architecture.
- [constraint] Use current-2026 REVLib: `SparkMax` + `SparkMaxConfig` builder; `CANSparkMax` and direct setters (`setInverted`, `setIdleMode`) are REMOVED and will not compile #standards. Source: `STANDARDS.md` 2026 Motor API Reference §1.
- [constraint] Phoenix6 `TalonFX` via control-request objects (`MotionMagicVoltage`, `PositionVoltage`); the team does NOT use Phoenix Pro — avoid Pro-only APIs (`TorqueCurrentFOC`, Fused/Sync feedback sources, MotionMagic Expo) #standards (no Pro license). Source: `STANDARDS.md` §3 Phoenix6; changelog 2026-01-25 CODEX entry.
- [constraint] NO warning suppression (`@SuppressWarnings`) — fix the root cause or leave the warning visible so tech debt stays tracked #standards. Source: `STANDARDS.md` Code Quality & Warnings.
- [constraint] Do NOT copy FRC-2024/2025 patterns without refactoring to 2026 idioms #standards (old REVLib/static-access patterns are forbidden). Source: `STANDARDS.md` Legacy Code Warnings; `.project-context.md` Code Standards.
- [decision] All motors require current limits; mechanisms require soft limits; mechanism commands require timeouts + clean interrupt handling #safety. Source: `STANDARDS.md` Safety Standards.

## Notes for Future Sessions
Read `STANDARDS.md` before any code change in this repo — it is the team's stated precondition and is more current than the stale `.project-context.md`. Elastic Dashboard is the primary dashboard; layout is a JSON in `2026Robot\src\main\deploy\elastic-layout.json` served on port 5800, with bidirectional tuning via `/Tuning/` NT paths gated by `TelemetryConstants.TUNING_MODE`.

## Relations
[[frc-2026]] [[frc-team-3843]]
