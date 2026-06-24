---
id: frc-2026-motor-safety-standards
title: FRC-2026 motor safety standards (current/soft limits, timeouts, failsafe)
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
tags: [frc, standards, safety, motors]
aliases: [frc motor safety, current limits, soft limits, command timeouts, failsafe behavior]
status: active
supersedes: null
confidence: 60
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
load_profile: scope_entry
load_priority: 150
source_rel: FRC-2026\.standards.md
enforceability: mandatory
trigger: configuring any motor or authoring any mechanism command
preconditions: working in FRC-2026 robot code
verification: every motor has a current limit; mechanisms have soft limits; mechanism commands have timeouts + clean interrupt handling
scope: FRC-2026
---

# FRC-2026 motor safety standards (current/soft limits, timeouts, failsafe)

> Required safety configuration for every motor and mechanism command: current limits on all motors, soft limits on mechanisms, timeouts and clean-up on mechanism commands.

## Context
Decomposed from the `.standards.md` "Safety Standards" section during the ACC monolith-decomposition pilot. These are mandatory and prevent the most common physical-damage and stuck-state failures.

## Observations
- [constraint] All motors MUST have current limits configured. SparkMax: `config.smartCurrentLimit(40)` (Amps). TalonFX: `config.CurrentLimits.StatorCurrentLimit = 40; config.CurrentLimits.StatorCurrentLimitEnable = true;` #safety. Source: `STANDARDS.md` Safety / Motor Current Limits.
- [constraint] Mechanisms MUST have soft limits. SparkMax: `config.softLimit.forwardSoftLimit(MAX)/forwardSoftLimitEnabled(true)` and reverse equivalents. TalonFX: `config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX; ForwardSoftLimitEnable = true;` #safety. Source: `STANDARDS.md` Safety / Soft Limits for Mechanisms.
- [constraint] Mechanism commands MUST have timeouts to prevent stuck states, e.g. `.run(...).until(...).withTimeout(3.0).finallyDo(interrupted -> stop())` #safety. Source: `STANDARDS.md` Safety / Command Timeouts.
- [constraint] Commands MUST clean up on interrupt — use `runEnd(runAction, endAction)` so the end action always runs (e.g. `() -> m_motor.set(0.0)`) #safety. Source: `STANDARDS.md` Safety / Failsafe Behavior.

## Relations
- relates-to [[frc-2026]] (motor safety for the 2026 robot)
- relates-to [[frc-2026-api-standards]] (the broader standard set, which also references these safety rules)
- relates-to [[revlib-sparkmax]] (SparkMax current/soft-limit config)
- relates-to [[phoenix6-ctre]] (TalonFX current/soft-limit config)
