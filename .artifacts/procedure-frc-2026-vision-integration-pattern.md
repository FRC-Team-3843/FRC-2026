---
id: frc-2026-vision-integration-pattern
title: FRC-2026 vision integration pattern (Cameras enum, ENABLE_VISION, graceful degradation)
schema_version: 2
artifact_kind: memory
memory_class: procedural
created: 2026-06-23T00:54:31Z
updated: 2026-06-23T00:54:31Z
valid_until: null
author: claude
session: acc-monolith-decomp-pilot-20260623
model: claude-opus-4-8
model_basis: confirmed
tags: [frc, standards, vision, photonvision]
aliases: [frc vision pattern, cameras enum, enable_vision, vision graceful degradation]
status: active
supersedes: null
confidence: 57
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
load_profile: scope_entry
load_priority: 160
source_rel: FRC-2026\.standards.md
enforceability: preferred
---

# FRC-2026 vision integration pattern (Cameras enum, ENABLE_VISION, graceful degradation)

> The standard structure for wiring vision: a `Vision` class at `frc.robot.vision.Vision`, an `ENABLE_VISION` flag, a `Cameras` enum carrying per-camera transforms + std-devs, and graceful degradation so the robot runs without vision.

## Context
Decomposed from the `.standards.md` "Vision Integration Standards" section during the ACC monolith-decomposition pilot. The live deployment facts (coprocessor at `10.38.43.11:5800`, `ENABLE_VISION=false` pending camera mount, camera names) are covered by `photonvision-coprocessor` and `photonvision`; this note carries the reusable integration *pattern*.

## Observations
- [decision] Vision class location is `frc.robot.vision.Vision`; the enable flag is `Constants.VisionConstants.ENABLE_VISION`; the AprilTag layout file is `src/main/deploy/apriltag_layout.json` #vision. Source: `STANDARDS.md` Vision Integration / Standard Pattern.
- [decision] Per-camera configuration is a `Cameras` enum where each entry carries a name, `Rotation3d` rotation, `Translation3d` position, single-tag std-devs (`VecBuilder.fill(4, 4, 8)`) and multi-tag std-devs (`VecBuilder.fill(0.5, 0.5, 1)`) #vision. Source: `STANDARDS.md` Vision / Camera Configuration Enum.
- [constraint] Vision MUST be optional — the robot must function without it; gate construction on the flag: `if (Constants.VisionConstants.ENABLE_VISION) { m_vision = new Vision(m_drivebase::getPose, m_drivebase.getSwerveDrive().field); }` #vision. Source: `STANDARDS.md` Vision / Graceful Degradation.

## Notes for Future Sessions
Live state: `ENABLE_VISION` stays false until the right camera is mounted + calibrated (see `frc-2026-live-todo` and `photonvision-coprocessor`); the coded camera transforms are estimates, not surveyed values.

## Relations
- relates-to [[frc-2026]] (vision pattern for the 2026 robot)
- relates-to [[photonvision]] (the photonlib wiring + camera names)
- relates-to [[photonvision-coprocessor]] (the coprocessor + ENABLE_VISION live state)
- relates-to [[frc-2026-shooter-swerve-hardware]] (the vision compute-split decision)
