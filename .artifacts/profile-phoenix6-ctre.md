---
id: phoenix6-ctre
title: CTRE Phoenix6 (TalonFX / CANcoder library)
schema_version: 2
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, motor-controller, vendordep, ctre]
aliases: [phoenix6, phoenix 6, ctre phoenix6, phoenix6-replay-26.1.0]
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
---

# CTRE Phoenix6 (TalonFX / CANcoder library)

> The CTRE library for all Kraken (TalonFX) motors and CANcoders on the robot; non-Pro control requests only — Pro features are forbidden by team standard.

## Context

Phoenix6 (`Phoenix6-replay-26.1.0` vendordep, `com.ctre.phoenix6.*`) drives every Kraken motor on the 2026 robot via `TalonFX`, plus the swerve `CANcoder`s. Team STANDARDS forbid Phoenix Pro features — only non-Pro control requests are allowed (`VelocityVoltage`, `VoltageOut`, `DutyCycleOut`). This is verified in `frc-2026-api-standards`. Note the project ALSO uses legacy Phoenix5 for one device (the hopper Bag motor) — see `phoenix5-6-coexistence`.

## Observations

- [registry] Vendordep `Phoenix6-replay-26.1.0.json` present; imports are `com.ctre.phoenix6.*`. #ctre
- [registry] Drives Kraken TalonFX motors: intake CAN 30 (X44), preshooters 36-37 (X44), main shooters 38-39 (X60); plus swerve CANcoders 19-22. #can
- [registry] Pro features forbidden — only `VelocityVoltage` / `VoltageOut` / `DutyCycleOut` control requests appear in subsystem code (verified in `ShooterSubsystem.java`, `IntakeSubsystem.java`). #standards
- [registry] Coexists with Phoenix5 in the same project (hopper TalonSRX) — do not assume one CTRE API everywhere. #ctre

## Open Questions

Main-shooter TalonFX on CAN 38 has a firmware error needing a Phoenix6 flash (open TODO in `.context.md`).

## Relations

- relates-to [[frc-2026]] (motor-controller library for the 2026 robot)
- relates-to [[phoenix5-ctre]] (legacy CTRE API also present, for the hopper)
- relates-to [[frc-2026-api-standards]] (the no-Phoenix-Pro standard)
- relates-to [[phoenix-tuner-x]] (the config/flash tool for these devices)
