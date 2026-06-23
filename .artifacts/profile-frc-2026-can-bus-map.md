---
id: frc-2026-can-bus-map
title: FRC-2026 CAN bus map (device → controller → motor)
schema_version: 2
artifact_kind: memory
memory_class: semantic
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, can, hardware, reference]
aliases: [can map, can bus map, can id table, device map]
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

# FRC-2026 CAN bus map (device → controller → motor)

> The full CAN device table for the 2026 robot — which CAN ID is which motor, on which controller, via which library — so you don't have to reconstruct it from scattered Constants.

## Context

The 2026 robot mixes three motor-controller ecosystems (Phoenix6, Phoenix5, REVLib) on one CAN bus, plus PWM and SPI devices off the bus. The authoritative table lives in `2026Robot/TODO-COMPETITION.md` and `NOTES.md` §0B; the per-device facts below are corroborated by the CAN-ID comments in `Constants.java`. Keep this as a single reference because the mapping is spread across multiple Constants inner-classes and two docs.

## Observations

- [registry] Swerve drive + angle: CAN **11-18** (NEO / SparkMax / REVLib). #can
- [registry] Swerve CANcoders: CAN **19-22** (CTRE / Phoenix6). #can
- [registry] Intake: CAN **30** — Kraken X44 / TalonFX / Phoenix6. #can
- [registry] Floor conveyor: CAN **31** — NEO 550 / SparkMax / REVLib. #can
- [registry] Hopper: CAN **32** — Bag motor / TalonSRX / **Phoenix5** (the lone Phoenix5 device). #can
- [registry] Feeder: CAN **33** — NEO 550 / SparkMax / REVLib. #can
- [registry] Turrets (L/R): CAN **34-35** — NEO 550 / SparkMax / REVLib. #can
- [registry] Preshooters: CAN **36-37** — Kraken X44 / TalonFX / Phoenix6. #can
- [registry] Main shooters: CAN **38-39** — Kraken X60 / TalonFX / Phoenix6. #can
- [registry] Off-bus: hood servos on **PWM 0/1**; NavX IMU on RoboRIO **SPI** (no CAN ID). #hardware

## Open Questions

Main-shooter TalonFX on CAN 38 has a firmware error (needs a Phoenix6 flash) — open TODO in `.context.md`.

## Relations

- relates-to [[frc-2026]] (the CAN bus of the 2026 robot)
- relates-to [[phoenix6-ctre]] (TalonFX + CANcoder devices)
- relates-to [[phoenix5-ctre]] (the lone TalonSRX on CAN 32)
- relates-to [[revlib-sparkmax]] (the SparkMax/NEO devices)
- relates-to [[navx-gyro]] (the off-bus SPI IMU)
