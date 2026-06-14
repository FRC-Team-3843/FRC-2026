---
id: photonvision
title: PhotonVision (vision library / photonlib)
type: reference
schema_version: 1
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, vision, vendordep]
aliases: [photonvision, photonlib, photon vision]
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
---

# PhotonVision (vision library / photonlib)

> The vision software/library side of the 2026 robot's localization — AprilTag detection on the coprocessor, pose fusion on the RIO.

## Context

PhotonVision is the vision stack the robot uses for AprilTag-based field localization, pulled in on the robot code side via the `photonlib` vendordep. Heavy detection runs on the PhotonVision coprocessor; the RoboRIO consumes the results and fuses pose estimates into the YAGSL odometry (`Vision.java` in `subsystems/swervedrive/`). Cameras are named `right_cam` / `left_cam`.

Docs: https://docs.photonvision.org/

## Observations

- [registry] Vendordep `photonlib.json` present in `2026Robot/vendordeps/`. #vision
- [registry] Camera naming convention: `right_cam` / `left_cam`; pose fusion happens on the RIO, not the coprocessor. #vision
- [registry] `Vision.java` (in `subsystems/swervedrive/`) is among the more-churned source files (4 commits). #vision
- [registry] AprilTag field set is `k2026RebuiltAndymark` (see `frc-2026-shooter-swerve-hardware`). #vision

## Open Questions

None from the corpus.

## Relations

- relates-to [[frc-2026]] (vision/localization library for the 2026 robot)
- relates-to [[photonvision-coprocessor]] (the software that runs on that box)
- relates-to [[yagsl]] (vision pose fuses into the YAGSL swerve odometry)
