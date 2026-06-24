---
id: frc-2026-live-todo
title: FRC-2026 live competition TODO + recent decisions
schema_version: 2
artifact_kind: task
created: 2026-06-23T00:54:31Z
updated: 2026-06-23T01:30:00Z
valid_until: null
author: claude
session: acc-monolith-decomp-pilot-20260623
model: claude-opus-4-8
model_basis: confirmed
tags: [frc, todo, decisions, status]
aliases: [frc live todo, competition todo, frc-2026 open tasks, frc recent decisions]
status: open
supersedes: null
confidence: 60
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
load_profile: scope_entry
load_priority: 10
source_rel: FRC-2026\.context.md
scope: FRC-2026
---

# FRC-2026 live competition TODO + recent decisions

> The live, orientation-critical task list and the standing design decisions for the 2026 REBUILT robot. The authoritative live task source is `2026Robot\TODO-COMPETITION.md`; this mirrors the open items so they surface on repo entry.

## Context
Decomposed from the `.context.md` monolith's `@always` block (TODO + Recent Decisions) during the ACC monolith-decomposition pilot. The robot is self-contained and must stay usable without `C:\GitHub` present. Tuned values and competition status are point-in-time (competition was March 2026); treat the live list in `2026Robot\TODO-COMPETITION.md` as the source of truth and reconcile this on entry.

## Observations
- [question] Fix main-shooter TalonFX CAN 38 firmware error — needs a Phoenix6 flash #robotics. Source: `2026Robot\TODO-COMPETITION.md`.
- [question] SysId swerve drive (needs space) + update pidf properties #robotics. Source: `TODO-COMPETITION.md`.
- [question] SysId shooter + preshooter for velocity PID tuning #robotics. Source: `TODO-COMPETITION.md`.
- [question] Vision bring-up: mount/wire/calibrate right camera, then set `VisionConstants.ENABLE_VISION = true` #vision. Source: `TODO-COMPETITION.md`, `README.md`.
- [decision] 2026-02-15 — AprilTag field set to `AprilTagFields.k2026RebuiltAndymark` (lowercase 'm'); compiles, done #vision. Source: `ROBOT NOTES.md` §3.3.
- [decision] 2026-03 — Mirrored double-turret shooter is the committed mechanism design #architecture. Source: `README.md`, `ROBOT NOTES.md`.
- [decision] 2026-01-25 — Phoenix Pro features explicitly NOT used; Phoenix6 non-Pro control requests only #standards. Source: `STANDARDS.md` §3, changelog.
- [decision] 2026-01-23 — `STANDARDS.md` is the single source of truth; strict command-based + DI mandatory #standards. Source: `STANDARDS.md`.

## Notes for Future Sessions
The legacy `.project-context.md` template was stale (2026-01-23 pre-season) and has been retired — `README.md` / `ROBOT NOTES.md` / `2026Robot\TODO-COMPETITION.md` are the live truth. The swerve-ratio and design decisions are detailed in `frc-2026-shooter-swerve-hardware`.

## Relations
- relates-to [[frc-2026]] (live status of the 2026 robot)
- relates-to [[frc-2026-shooter-swerve-hardware]] (the design decisions referenced here)
- relates-to [[frc-2026-can-bus-map]] (CAN 38 main-shooter device)
