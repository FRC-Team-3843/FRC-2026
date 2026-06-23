---
id: frc-2026-build-env-java17
model: unattributed
model_basis: unattributed
title: FRC-2026 build requires WPILib JDK17 (system java is 8)
schema_version: 2
artifact_kind: memory
memory_class: procedural
created: 2026-06-12T06:30:00Z
updated: 2026-06-12T06:30:00Z
valid_until: null
author: claude
session: phase7-onboarding-20260612
tags: [frc, build, config]
aliases: [java_home rule, frc build env, gradlew jdk17]
status: active
supersedes: null
confidence: 58
source_basis: conversation
human_edited: false
sensitivity: normal
decisions: []
---

# FRC-2026 build requires WPILib JDK17 (system java is 8)

> Gradle builds of `2026Robot` must run under the WPILib JDK17 at `C:\Users\Public\wpilib\2026\jdk`; the machine's default `java` is 8, so an unset `JAVA_HOME` fails the build with a Java-version error.

## Context
Surfaced during ACC Phase 7 onboarding of the FRC-2026 repo. Documented in the repo corpus, not from a live build this session.

## Observations
- [constraint] Set `JAVA_HOME` to `C:\Users\Public\wpilib\2026\jdk` before any gradle invocation #build (system default java is 8; WPILib targets Java 17). Source: `NOTES.md` Section 1 ("IMPORTANT: Use the WPILib JDK for builds").
- [decision] Concrete build command, run from the `2026Robot` project dir #build (matches workspace memory + repo NOTES): `JAVA_HOME="C:/Users/Public/wpilib/2026/jdk" ./gradlew build`. Deploy/test/sim are `./gradlew deploy | test | simulateJava` from the same dir. Source: `NOTES.md` Section 1, `STANDARDS.md` Build Commands, `README.md` Building and Deploying.
- [constraint] Build/deploy commands are project-dir-scoped (`cd 2026Robot` first), not repo-root #build. Source: `STANDARDS.md` ("Run from the specific robot project directory").

## Notes for Future Sessions
This is the single most common FRC build trap in this workspace; it is also captured as the `wpilib-build-env` personal entity and as a critical rule in this repo's `.protocol.md`. If a gradle build errors on Java version, this is the first thing to check.

## Relations
[[frc-2026]] [[wpilib-build-env]] [[frc-team-3843]]
