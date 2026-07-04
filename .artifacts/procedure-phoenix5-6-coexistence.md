---
id: phoenix5-6-coexistence
title: Phoenix5 + Phoenix6 coexist — hopper is the Phoenix5 exception
schema_version: 2
artifact_kind: memory
memory_class: procedural
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, ctre, gotcha, build]
aliases: [phoenix5-6 coexistence, dual phoenix api, phoenix import gotcha]
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
enforceability: preferred
scope: FRC-2026
load_profile: on_demand
---

# Phoenix5 + Phoenix6 coexist — hopper is the Phoenix5 exception

> This project deliberately uses BOTH CTRE APIs: Phoenix6 for everything except the hopper Bag motor, which is Phoenix5. Import from the wrong namespace and the build breaks.

## Context

A non-obvious trap for anyone touching motor code in this repo: there is no single CTRE API. The hopper's Bag motor (TalonSRX, CAN 32) is driven by legacy **Phoenix5** (`com.ctre.phoenix.*`). Every other CTRE device — all the Krakens (TalonFX) and the swerve CANcoders — uses **Phoenix6** (`com.ctre.phoenix6.*`). Both vendordeps (`Phoenix5-replay-5.36.0`, `Phoenix6-replay-26.1.0`) are installed. If you copy a TalonFX import or pattern into `HopperSubsystem`, or vice-versa, it won't compile, because the class hierarchies are entirely separate.

Verified in code: `HopperSubsystem.java` imports `com.ctre.phoenix.motorcontrol.can.TalonSRX` / `TalonSRXControlMode` / `SupplyCurrentLimitConfiguration`; `Constants.java` line 306 labels CAN 32 "Bag motor / TalonSRX — Phoenix5"; all other subsystems import `com.ctre.phoenix6.*`.

## Observations

- [registry] Hopper Bag motor (CAN 32) = Phoenix5 `com.ctre.phoenix.*`; the only Phoenix5 device. #ctre
- [registry] All Krakens (TalonFX) + swerve CANcoders = Phoenix6 `com.ctre.phoenix6.*`. #ctre
- [registry] Both vendordeps installed: `Phoenix5-replay-5.36.0.json` and `Phoenix6-replay-26.1.0.json`. #build
- [registry] Wrong-namespace import → build break; the two class hierarchies don't interoperate. #gotcha

## Open Questions

None — verified directly in the source at seed time.

## Relations

- relates-to [[frc-2026]] (a build gotcha in the 2026 robot code)
- relates-to [[phoenix5-ctre]] (the legacy API, hopper only)
- relates-to [[phoenix6-ctre]] (the primary API, everything else)
- relates-to [[frc-2026-can-bus-map]] (which device is on which library)
