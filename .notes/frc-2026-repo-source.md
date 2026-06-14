---
id: frc-2026-repo-source
title: FRC-2026 Robot Repository
type: reference
schema_version: 1
created: 2026-06-12T06:30:00Z
updated: 2026-06-12T06:30:00Z
path: C:\GitHub\FRC-2026
status: active
account: personal
reachable_via: local
model: unattributed                              # Phase-7 note predates RC-PROV-1; producing model not in the runtime record (§3.9)
model_basis: unattributed
tags: [frc, robotics, source, build]
aliases: [2026 robot, rebuilt, 2026Robot, team 3843 robot]
---

# FRC-2026 Robot Repository

> FRC Team 3843's active 2026-season ("REBUILT") robot codebase; Java/WPILib command-based swerve robot at `C:\GitHub\FRC-2026`, with robot code under `2026Robot\`.

## Context
Competition robot code for Team 3843's 2026 REBUILT season (Smokey Mountain Regional, March 2026). WPILib 2026 command-based Java with YAGSL swerve, PhotonVision localization (on a Beelink coprocessor), PathPlanner autos, and a mirrored double-turret shooter. The repo is self-contained: per its own `.protocol.md` it must stay usable without `C:\GitHub` present.

Authoritative docs live at the repo root and in `2026Robot\`: `README.md` (overview), `NOTES.md` (setup/tuning/game/CAN-bus/troubleshooting), `STANDARDS.md` (coding+API rules, the team's "read first" file), `2026Robot\TODO-COMPETITION.md` (live competition task list + tuned values), and `.changelog.md` (append-only repo log). Note: `.project-context.md` is stale (dated 2026-01-23, describes a pre-season template) and is superseded in practice by README/NOTES/TODO-COMPETITION.

Build env gotcha is load-bearing: system `java` is 8, builds require the WPILib JDK17 — see `wpilib-build-env` entity and this repo's `.protocol.md` critical rule.

## Mentioned by
