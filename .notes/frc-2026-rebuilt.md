---
id: frc-2026-rebuilt
title: FRC 2026 REBUILT (game)
schema_version: 2
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, game, rules]
aliases: [rebuilt, 2026 game, rebuilt game, hub shifting, frc rebuilt, frc 2026 rebuilt]
status: active
supersedes: null
confidence: 55
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

# REBUILT (2026 FRC game)

> The 2026 FRC game the robot is built for; its defining mechanic is Hub Shifting, and the alliance color comes from the game-specific message at runtime.

## Context

REBUILT (presented by Haas) is the 2026 FRC game. Alliances score Fuel (5.91" balls) into their Hub, traverse field obstacles (Bumps ~6.5" ramps, Trenches 22.25" clearance), and climb the Tower in endgame. The signature mechanic is **Hub Shifting**: alliance Hubs alternate between active and inactive during teleop based on autonomous performance. The robot reads its alliance from `DriverStation.getGameSpecificMessage()` → `'R'` or `'B'`. Full scoring and Ranking Point breakdown is in NOTES §0. Match length is 2:40 (160 s). For max RP an alliance needs ~360 of the 504 field fuel scored.

## Observations

- [registry] Game: REBUILT (Haas), 2026 season; match length 2:40 (160 s). #game
- [registry] Hub Shifting: alliance Hubs alternate active/inactive in teleop, driven by auto performance. #game
- [registry] Alliance color is read at runtime via `DriverStation.getGameSpecificMessage()` → 'R'/'B'. #game
- [registry] Fuel is 5.91" diameter; obstacles are Bumps (~6.5" ramps) and Trenches (22.25" clearance); Tower climb in endgame. #game

## Open Questions

None from the corpus (full rules in NOTES §0 and the linked 2026 Game Manual).

## Relations

- relates-to [[frc-2026]] (the game the 2026 robot is designed for)
