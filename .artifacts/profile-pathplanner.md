---
id: pathplanner
title: PathPlanner (autonomous path library)
schema_version: 2
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, autonomous, vendordep]
aliases: [pathplanner, pathplannerlib, pathplannerlib-2026.1.2]
status: active
supersedes: null
confidence: 58
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

# PathPlanner (autonomous path library)

> The autonomous-routine library; its settings + named commands are data-driven under `deploy/pathplanner/`.

## Context

PathPlanner (`PathplannerLib-2026.1.2` vendordep) generates and follows autonomous trajectories for the 2026 robot. Robot kinematic settings for the planner — width, length, mass, coefficient of friction — live in `2026Robot/src/main/deploy/pathplanner/settings.json` (a frequently-tuned file, 4 commits). Autonomous PID lives in `AutonConstants`. The robot registers `NamedCommands` such as `autoShoot` and `stopShooter` that paths can call, plus a `navgrid` for pathfinding. A separate trajectory vendordep (ChoreoLib) is also present in the project.

## Observations

- [registry] Vendordep `PathplannerLib-2026.1.2.json` present in `2026Robot/vendordeps/`. #autonomous
- [registry] `deploy/pathplanner/settings.json` holds robot width/length/mass/COF for path planning (heavily tuned — 4 commits). #autonomous
- [registry] NamedCommands registered for autos include `autoShoot` and `stopShooter`. #autonomous
- [registry] A PathPlanner `navgrid` was added for pathfinding (commit "Update docs for competition state, add PathPlanner navgrid"). #autonomous
- [constraint] Git-hygiene for PathPlanner: gitignore `**/generatedJSON/` (regenerated on every build/deploy) but KEEP `.path`/`.auto` files, `navgrid.json`, and `settings.json` tracked in git so paths sync across team computers — confirmed via FRC-2025 cleanup session (2026-01-20, web-claude-web-20d9f262-c217-4f58-b237-08ace8a48da8): `git rm -r --cached **/generatedJSON/` + matching `.gitignore` entry removed 54,012 lines of regenerable trajectory JSON from FRC-2025 tracking without affecting path sync. #git-hygiene <!-- @claude 2026-07-07T00:00:00Z -->

## Open Questions

PathPlanner and ChoreoLib are both present — whether Choreo is actively used or vestigial is unconfirmed (see `choreolib` note).

## Relations

- relates-to [[frc-2026]] (autonomous path library for the 2026 robot)
- relates-to [[choreolib]] (alternative trajectory lib also present in the project)
- relates-to [[yagsl]] (paths drive the YAGSL swerve subsystem)
