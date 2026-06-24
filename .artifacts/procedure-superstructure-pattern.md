---
id: superstructure-pattern
title: Superstructure coordinates the feed chain — don't bypass it
schema_version: 2
artifact_kind: memory
memory_class: procedural
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, architecture, command-based]
aliases: [superstructure, superstructure pattern, feed chain coordinator]
status: active
supersedes: null
confidence: 57
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
scope: FRC-2026
---

# Superstructure coordinates the feed chain — don't bypass it

> `Superstructure.java` is a command-factory that combines intake + conveyor + hopper + feeder + shooter + turret into coordinated multi-subsystem commands; wiring those subsystems individually around it invites requirement conflicts.

## Context

The 2026 robot has a `Superstructure` that owns multi-subsystem actions. Important nuance verified in code: it is NOT a `SubsystemBase` — it's a plain coordinator class that produces command factories combining the six mechanism subsystems (intake, conveyor, hopper, feeder, shooter, turret). RobotContainer binds these factory commands (e.g. `intakeCommand()`, `setPositionCommand(preset)`) to controls. The point of the pattern: the feed chain and shooting sequence are coordinated in one place so command requirements are owned coherently. If you bind raw per-subsystem commands that overlap what Superstructure already coordinates, you get command-requirement conflicts (one command interrupting another's subsystem mid-action).

## Observations

- [registry] `Superstructure.java` is a coordinator (NOT a SubsystemBase); it returns command factories. #architecture
- [registry] Combines intake + conveyor + hopper + feeder + shooter + turret into one coordinated command surface. #architecture
- [registry] RobotContainer binds its factory commands to driver/operator controls. #architecture
- [registry] Bypassing it with overlapping per-subsystem commands causes requirement conflicts. #gotcha

## Open Questions

None — class structure verified at seed time.

## Relations

- relates-to [[frc-2026]] (the command architecture of the 2026 robot)
- relates-to [[frc-2026-api-standards]] (strict command-based + DI; RobotContainer is the only binding site)
