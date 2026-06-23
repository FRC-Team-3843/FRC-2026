---
id: frc-2026-pre-change-gate
title: FRC-2026 pre-change gate — read STANDARDS, set JDK17, verify CAN IDs
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
tags: [frc, procedure, gate, build, standards]
aliases: [frc pre-change gate, read standards first, verify can before deploy, frc critical rules]
status: active
supersedes: null
confidence: 62
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
load_profile: scope_entry
load_priority: 20
source_rel: FRC-2026\.protocol.md
enforceability: mandatory
trigger: before any code change or hardware deploy in FRC-2026
preconditions: working in the FRC-2026 repo
verification: JAVA_HOME points at the WPILib JDK17; STANDARDS read this session; CAN IDs checked against the map
---

# FRC-2026 pre-change gate — read STANDARDS, set JDK17, verify CAN IDs

> The mandatory preconditions before touching FRC-2026 code or deploying to hardware: build under the WPILib JDK17, read the coding standards first, and verify CAN IDs before a hardware deploy.

## Context
Decomposed from the `.protocol.md` monolith's `@always` "Critical rules (FRC-2026)" block during the ACC monolith-decomposition pilot. These are the load-bearing gates that catch the two most common failures (wrong Java version; un-refactored legacy patterns) and the most dangerous one (wrong CAN ID on deploy). The repo must remain usable without `C:\GitHub`.

## Observations
- [constraint] Build with the WPILib JDK17 — system `java` is 8, so an unset `JAVA_HOME` fails the build with a Java-version error. From the `2026Robot` dir: `JAVA_HOME="C:/Users/Public/wpilib/2026/jdk" ./gradlew build` (likewise `deploy` / `test` / `simulateJava`); run from the project dir, not the repo root #build. Source: `.protocol.md` Critical rules. (Full build-env detail in `frc-2026-build-env-java17`.)
- [constraint] Read the coding standards before any code change — strict command-based + DI (RobotContainer is the only instantiation/binding site), 2026 motor APIs only, no Phoenix Pro, no warning suppression, no un-refactored FRC-2024/2025 patterns #standards. Source: `.protocol.md` Critical rules. (Full standard set in `frc-2026-api-standards`.)
- [constraint] Verify CAN IDs before deploying to hardware (swerve 11-22, mechanisms 30-39) #robotics. Source: `.protocol.md` Critical rules. (Full device table in `frc-2026-can-bus-map`.)
- [decision] The legacy `.project-context.md` template was stale (2026-01-23 pre-season); `README.md` / `NOTES.md` / `2026Robot\TODO-COMPETITION.md` are the live truth #status. Source: `.protocol.md` Critical rules.

## Notes for Future Sessions
This consolidates the FRC-2026 critical-rules gate; the underlying standards and tables live in `frc-2026-api-standards`, `frc-2026-build-env-java17`, and `frc-2026-can-bus-map`. The `.protocol.md` Read-Order / During-Work / Logging / Working-Files sections were dropped in the decomposition because the workspace-level protocol and the new ACC read-on-entry rule supersede them.

## Relations
- relates-to [[frc-2026]] (the change-gate for the 2026 robot)
- relates-to [[frc-2026-api-standards]] (the standards this gate enforces reading)
- relates-to [[frc-2026-build-env-java17]] (the JDK17 build precondition)
- relates-to [[frc-2026-can-bus-map]] (the CAN IDs this gate verifies)
