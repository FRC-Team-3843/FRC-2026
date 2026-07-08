---
id: choreolib
title: ChoreoLib (trajectory vendordep)
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
aliases: [choreolib, choreo, choreolib2026]
status: active
supersedes: null
confidence: 50
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

# ChoreoLib (trajectory vendordep)

> A trajectory/path vendordep present in the project alongside PathPlanner; whether it's actively used or vestigial is unconfirmed.

## Context

ChoreoLib (`ChoreoLib2026.json` vendordep) is a trajectory-generation library present in `2026Robot/vendordeps/`. It coexists with PathPlanner, which is the library the autonomous routines visibly use (NamedCommands, navgrid, `deploy/pathplanner/`). No Choreo `.traj` assets or Choreo-specific code surfaced in the harvest, so this may be an installed-but-unused dependency. Kept as a low-confidence stub so a future session knows it's on the classpath.

## Observations

- [registry] Vendordep `ChoreoLib2026.json` present in `2026Robot/vendordeps/`. #autonomous
- [registry] Coexists with PathPlanner; PathPlanner is the library with visible auto wiring. #autonomous
- [correction] Choreo is NOT distributed via the Microsoft Store — install from GitHub releases instead #frc-2026 <!-- @claude 2026-07-07T00:00:00Z, source: session-frc-laptop-setup-doc-review -->

## Open Questions

Is Choreo actually used, or a vestigial dependency? No Choreo trajectory assets or code surfaced in the corpus — confirm before relying on it.

## Relations

- relates-to [[frc-2026]] (trajectory library present in the 2026 robot project)
- relates-to [[pathplanner]] (the auto library actually wired up; Choreo coexists with it)
- relates_to [[frc-laptop-setup-doc-review]] (Profile's 'Choreo distributed via GitHub releases, not the Microsoft Store' correction is cited inline as sourced from this session, but neither Relat) <!-- @dreaming 2026-07-08 -->
