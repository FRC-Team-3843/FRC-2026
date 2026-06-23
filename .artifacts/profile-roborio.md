---
id: roborio
title: RoboRIO (FRC robot controller / deploy target)
schema_version: 2
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, hardware, deploy-target]
aliases: [roborio, rio, roborio-3843-frc.local]
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
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
---

# RoboRIO (FRC robot controller / deploy target)

> The robot's main controller and the `gradlew deploy` target; runs the robot code, fuses vision pose, and logs SysId data locally.

## Context

The RoboRIO is the FRC standard robot controller (platform `linuxathena`) and the deploy target for the 2026 robot code. It is addressable as `roboRIO-3843-frc.local` / `10.38.43.2`. Team number 3843 is recorded in `.wpilib/wpilib_preferences.json`. The RIO runs the command-based robot program, performs vision pose fusion (PhotonVision results come from the coprocessor), and writes SysId `.wpilog` files to `/home/lvuser/` for later analysis in AdvantageScope.

## Observations

- [registry] Platform `linuxathena`; deploy target for `./gradlew deploy`. #deploy
- [registry] Addresses: `roboRIO-3843-frc.local` / `10.38.43.2`. #network
- [registry] Team number 3843 set in `.wpilib/wpilib_preferences.json`. #config
- [registry] SysId logs land in `/home/lvuser/` on the RIO; pulled for AdvantageScope review. #telemetry

## Open Questions

None from the corpus.

## Relations

- relates-to [[frc-2026]] (the robot controller / deploy target)
- relates-to [[photonvision-coprocessor]] (RIO consumes its vision results)
- relates-to [[advantagescope]] (SysId logs from the RIO are viewed here)
- relates-to [[wpilib-gradlerio]] (GradleRIO deploys the code to this target)
