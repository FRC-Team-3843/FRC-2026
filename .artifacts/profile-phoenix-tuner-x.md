---
id: phoenix-tuner-x
title: Phoenix Tuner X (CTRE config tool)
schema_version: 2
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, tool, ctre, pit]
aliases: [phoenix tuner x, tuner x, phoenix tuner]
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
scope: FRC-2026
load_profile: on_demand
---

# Phoenix Tuner X (CTRE config tool)

> CTRE's desktop tool for flashing CAN IDs and firmware onto TalonFX/TalonSRX/CANcoder devices; one of the two mandatory pit tools (competition setup Step 1).

## Context

Phoenix Tuner X is the CTRE configuration utility used to assign CAN IDs and flash firmware to all CTRE devices on the robot — TalonFX (Kraken), TalonSRX (Bag), and CANcoders. NOTES §0B lists it as competition-setup Step 1. It pairs with the REV Hardware Client (for SparkMax) as the two tools every pit session needs. Relevant now because a main-shooter TalonFX (CAN 38) has a firmware error needing a Phoenix6 flash via this tool.

## Observations

- [registry] Used to flash CAN IDs + firmware on CTRE TalonFX / TalonSRX / CANcoder. #tool
- [registry] Competition-setup Step 1 per NOTES §0B. #pit
- [registry] Paired with REV Hardware Client as the two mandatory pit config tools. #pit
- [gotcha] Legacy Phoenix Tuner (v1), if present, should be removed to avoid confusion/conflict with Phoenix Tuner X #frc-2026 <!-- @claude 2026-07-07T00:00:00Z, source: session-frc-laptop-setup-doc-review -->
- [registry] Firmware-update UX gotcha: after selecting Phoenix version + year and clicking "Update Firmware", the actual firmware-file picker is a blank/unlabeled dropdown you must click to reveal options — miss it and the tool appears to just dump every firmware file ever made with no compatibility filtering. #gotcha (source: session-cancoder-firmware-vk-vh-selection, 2026-01-17) <!-- @claude 2026-07-07T00:00:00Z -->

## Open Questions

None from the corpus.

## Relations

- relates-to [[frc-2026]] (config tool for the 2026 robot's CTRE devices)
- relates-to [[phoenix6-ctre]] (configures Phoenix6 TalonFX/CANcoder devices)
- relates-to [[phoenix5-ctre]] (also flashes the Phoenix5 TalonSRX)
- relates-to [[rev-hardware-client]] (the paired tool for REV/SparkMax devices)
