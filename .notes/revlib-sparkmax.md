---
id: revlib-sparkmax
title: REVLib (SparkMax / NEO library)
schema_version: 2
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, motor-controller, vendordep, rev]
aliases: [revlib, rev lib, sparkmax, spark max, neo, neo550, rev robotics]
status: active
supersedes: null
confidence: 60
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

# REVLib (SparkMax / NEO library)

> The REV library for all NEO and NEO 550 motors on the robot; team standard is the new `SparkMax`/`SparkMaxConfig` API, never the deprecated `CANSparkMax`.

## Context

REVLib (`REVLib.json` vendordep) drives every NEO and NEO 550 motor on the 2026 robot through SparkMax controllers. Team STANDARDS mandate the 2026 API — `SparkMax` / `SparkMaxConfig` — and explicitly prohibit the older `CANSparkMax`/`CANSparkMaxConfig` classes (verified in `frc-2026-api-standards`). The SparkMax CAN-frame latency is also why `Constants.LOOP_TIME` is set high — see `sparkmax-loop-time`.

## Observations

- [registry] Vendordep `REVLib.json` present in `2026Robot/vendordeps/`. #rev
- [registry] Drives NEO/NEO550 devices: swerve drive+angle CAN 11-18, floor conveyor 31, feeder 33, both turrets 34-35. #can
- [registry] Team API standard: `SparkMax` / `SparkMaxConfig` ONLY — `CANSparkMax` is forbidden. #standards
- [registry] SparkMax CAN delay (~110 ms) drives the unusually large `LOOP_TIME = 0.13` — see `sparkmax-loop-time`. #control

## Open Questions

None from the corpus.

## Relations

- relates-to [[frc-2026]] (motor-controller library for the 2026 robot)
- relates-to [[frc-2026-api-standards]] (the SparkMax-not-CANSparkMax standard)
- relates-to [[rev-hardware-client]] (the config/flash tool for SparkMax)
