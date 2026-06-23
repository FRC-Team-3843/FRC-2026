---
id: sparkmax-loop-time
title: LOOP_TIME = 0.13 accounts for SparkMax CAN delay
schema_version: 2
artifact_kind: memory
memory_class: semantic
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, control, tuning, gotcha]
aliases: [loop_time, loop time, sparkmax delay, odometry loop time]
status: active
supersedes: null
confidence: 57
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
---

# LOOP_TIME = 0.13 accounts for SparkMax CAN delay

> `Constants.LOOP_TIME = 0.13` is intentionally large (not a typo): it folds in the ~110 ms SparkMax CAN latency on top of the 20 ms WPILib loop. Drop it to ~0.04 only if the drive motors move to TalonFX.

## Context

If you see `Constants.LOOP_TIME = 0.13` and think it should be `0.02`, don't change it. The value deliberately accounts for the 20 ms WPILib periodic loop PLUS the roughly 110 ms CAN reporting delay of the SparkMax controllers driving the swerve. It feeds odometry and feedforward timing; lowering it naively will desync those. The in-code comment (verified at `Constants.java` line ~33-36) says exactly this and notes: reduce to ~0.04 if the drive motors are switched to TalonFX (Krakens), which report far faster than SparkMax.

## Observations

- [registry] `Constants.LOOP_TIME = 0.13` = 20 ms WPILib loop + ~110 ms SparkMax CAN delay. #control
- [registry] Used in odometry / feedforward timing — naive reduction desyncs them. #control
- [registry] Comment says reduce to ~0.04 if drive motors switch to TalonFX. #control

## Open Questions

None — value + rationale verified in `Constants.java` at seed time.

## Relations

- relates-to [[frc-2026]] (a control-timing gotcha in the 2026 robot)
- relates-to [[revlib-sparkmax]] (the SparkMax CAN latency this compensates for)
