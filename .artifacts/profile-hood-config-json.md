---
id: hood-config-json
title: Hood angles live in deploy/hood-config.json, not Constants
schema_version: 2
artifact_kind: memory
memory_class: semantic
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, config, tuning, gotcha]
aliases: [hood-config.json, hood config, hood angles]
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
enforceability: preferred
scope: FRC-2026
---

# Hood angles live in deploy/hood-config.json, not Constants

> The shooter hood near/far angles are runtime-loaded from a deployed JSON and tuned live from the dashboard — hardcoding them in Java breaks the "Save Hood" tuning workflow.

## Context

A tuning gotcha: the shooter hood's near and far angles are NOT in `Constants.java`. They live in `2026Robot/src/main/deploy/hood-config.json`, which the robot loads at runtime and which the Elastic dashboard's "Save Hood" button rewrites during tuning. If you "fix" hood angles by editing Java constants, you'll be overwritten by the JSON on the next deploy and you'll break the drivers' on-the-fly tuning loop. Verified — the file exists with keys `leftNearAngle` / `leftFarAngle` / `rightNearAngle` / `rightFarAngle` (left/right because the shooter is a mirrored double turret).

## Observations

- [registry] Hood angles live in `deploy/hood-config.json` (keys: `leftNearAngle`, `leftFarAngle`, `rightNearAngle`, `rightFarAngle`), runtime-loaded. #config
- [registry] Elastic dashboard "Save Hood" persists tuned values back to this JSON. #tuning
- [registry] Do NOT hardcode hood angles in `Constants.java` — it breaks the tuning workflow and gets overwritten. #gotcha

## Open Questions

None — file verified at seed time.

## Relations

- relates-to [[frc-2026]] (a tuning gotcha in the 2026 robot)
- relates-to [[elastic-dashboard]] (the "Save Hood" tuning surface)
