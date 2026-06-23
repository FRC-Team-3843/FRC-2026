---
id: rev-hardware-client
title: REV Hardware Client (SparkMax config tool)
schema_version: 2
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, tool, rev, pit]
aliases: [rev hardware client, rev client, sparkmax config tool]
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
lifecycle: active
---

# REV Hardware Client (SparkMax config tool)

> REV's desktop tool for assigning CAN IDs and flashing firmware to SparkMax controllers; the REV-side counterpart to Phoenix Tuner X in the pit.

## Context

The REV Hardware Client is used to configure CAN IDs and flash firmware on the robot's SparkMax controllers (the NEO/NEO 550 devices: swerve drive+angle, conveyor, feeder, turrets). It is the REV-side counterpart to Phoenix Tuner X (which handles CTRE devices). Together they are the two mandatory pit tools for the 2026 robot's mixed CTRE+REV CAN bus.

## Observations

- [registry] Used to flash CAN IDs + firmware on SparkMax controllers (NEO / NEO 550 devices). #tool
- [registry] REV-side counterpart to Phoenix Tuner X; the two cover the full mixed CAN bus. #pit

## Open Questions

None from the corpus.

## Relations

- relates-to [[frc-2026]] (config tool for the 2026 robot's REV devices)
- relates-to [[revlib-sparkmax]] (configures the SparkMax controllers REVLib drives)
- relates-to [[phoenix-tuner-x]] (the paired CTRE-side tool)
