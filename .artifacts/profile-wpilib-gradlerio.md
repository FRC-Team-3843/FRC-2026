---
id: wpilib-gradlerio
title: WPILib GradleRIO (build/deploy plugin)
schema_version: 2
artifact_kind: memory
memory_class: semantic
semantic_kind: entity_profile
created: 2026-06-13T21:55:41Z
updated: 2026-06-13T21:55:41Z
valid_until: null
author: claude
session: null
tags: [frc, build, tooling]
aliases: [gradlerio, gradle rio, wpilib build]
status: active
supersedes: null
confidence: 55
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

# WPILib GradleRIO (build/deploy plugin)

> The Gradle plugin that builds, deploys, tests, and simulates the robot code; the JDK17 build-env requirement is documented separately — do not duplicate it here.

## Context

GradleRIO (2026.2.1) is the WPILib Gradle plugin that drives the robot build lifecycle: `./gradlew build`, `deploy`, `test`, `simulateJava`. It is invoked from the `2026Robot/` project directory, not the repo root. The critical build-environment requirement — that builds must use the WPILib JDK17 because the system `java` is Java 8 — is fully captured in `frc-2026-build-env-java17`; this note intentionally cross-links rather than restating it.

## Observations

- [registry] GradleRIO 2026.2.1; tasks `build` / `deploy` / `test` / `simulateJava`. #build
- [registry] Run from the `2026Robot/` project dir, not the repo root. #build
- [registry] JDK17/JAVA_HOME requirement lives in `frc-2026-build-env-java17` — see that note, not duplicated here. #build
- [fact] WPILib installs per-year to separate folders — updating to a new season's release does not require uninstalling the prior year; re-running the new year's installer in place is sufficient, and VS Code offers to upgrade an existing robot project on next open. Running the FULL installer (not just opening VS Code) is required to also pick up that year's updated desktop tools (Elastic, AdvantageScope) #frc-2026 <!-- @claude 2026-07-07T00:00:00Z, source: session-frc-laptop-setup-doc-review -->

## Open Questions

None from the corpus.

## Relations

- relates-to [[frc-2026]] (build/deploy tooling for the 2026 robot)
- relates-to [[frc-2026-build-env-java17]] (the JDK17 build-env rule)
- relates-to [[roborio]] (the deploy target)
- relates_to [[frc-laptop-setup-doc-review]] (Profile's per-year-install/no-uninstall-needed fact is cited inline as sourced from this session, but neither Relations section links them.) <!-- @dreaming 2026-07-08 -->
