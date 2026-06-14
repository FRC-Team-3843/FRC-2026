---
id: phoenix5-ctre
title: CTRE Phoenix5 (legacy TalonSRX library)
type: reference
schema_version: 1
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, motor-controller, vendordep, ctre]
aliases: [phoenix5, phoenix 5, ctre phoenix5, phoenix5-replay-5.36.0]
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
---

# CTRE Phoenix5 (legacy TalonSRX library)

> The legacy CTRE library kept in the project for exactly one device — the hopper Bag motor on a TalonSRX — alongside Phoenix6 for everything else.

## Context

Phoenix5 (`Phoenix5-replay-5.36.0` vendordep, `com.ctre.phoenix.*` — note: no `6`) is retained solely to drive the hopper's Bag motor through a `TalonSRX` on CAN ID 32. Every other CTRE device on the robot uses Phoenix6. This dual-API coexistence is deliberate and easy to get wrong: importing from the wrong namespace breaks the build. Verified in code — `HopperSubsystem.java` imports `com.ctre.phoenix.motorcontrol.can.TalonSRX`, and `Constants.java` line 306 comments the CAN-32 device as "Bag motor / TalonSRX — Phoenix5".

## Observations

- [registry] Vendordep `Phoenix5-replay-5.36.0.json` present; imports are `com.ctre.phoenix.*` (NOT `phoenix6`). #ctre
- [registry] Sole consumer is `HopperSubsystem.java` — `TalonSRX` Bag motor on CAN 32. #can
- [registry] Coexists with Phoenix6 in the same project; the import namespace distinguishes them. #ctre

## Open Questions

None from the corpus.

## Relations

- relates-to [[frc-2026]] (legacy motor-controller library for the 2026 robot)
- relates-to [[phoenix6-ctre]] (the primary CTRE API used for everything else)
- relates-to [[phoenix5-6-coexistence]] (the gotcha note about the dual-API split)
