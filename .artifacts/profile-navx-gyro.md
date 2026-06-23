---
id: navx-gyro
title: NavX IMU (gyro on RoboRIO SPI)
schema_version: 2
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, sensor, imu, hardware]
aliases: [navx, navx-gyro, navx imu, studica]
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
---

# NavX IMU (gyro on RoboRIO SPI)

> The robot's heading sensor — a NavX on the RIO's SPI bus (not CAN); its alliance-aware zeroing must happen before enabling field-relative drive.

## Context

The NavX IMU (Studica vendordep, `Studica.json`) provides heading for field-centric swerve and heading lock. It connects over the RoboRIO's **SPI** bus (config key `navx_spi`) — it is NOT a CAN device, so it never appears in the CAN map. Operationally important: gyro zeroing is alliance-aware (blue → 0°, red → 180°) and must be done BEFORE enabling field-relative control, or the field frame is wrong. (The related pitfall — never combine alliance-relative control with an alliance-aware gyro zero — is captured cross-scope in the PersonalContext "Alliance control pitfall" feedback note.)

## Observations

- [registry] Vendordep `Studica.json` present; the NavX is on RoboRIO SPI (`navx_spi`), no CAN ID. #sensor
- [registry] Provides heading for field-centric drive and heading lock in the swerve subsystem. #swerve
- [registry] Zero-gyro is alliance-aware: blue 0° / red 180°; do it before enabling field-relative. #operations

## Open Questions

None from the corpus.

## Relations

- relates-to [[frc-2026]] (heading sensor for the 2026 robot)
- relates-to [[yagsl]] (NavX feeds YAGSL field-centric control)
- relates-to [[roborio]] (connected via the RIO SPI bus)
