---
id: frc-2026-testing-requirements
title: FRC-2026 testing requirements (JUnit 5 per subsystem)
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
tags: [frc, standards, testing, junit]
aliases: [frc testing requirements, junit 5 frc, subsystem tests, gradlew test]
status: active
supersedes: null
confidence: 58
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
load_profile: scope_entry
load_priority: 110
source_rel: FRC-2026\.standards.md
artifact_kind: memory
memory_class: procedural
enforceability: preferred
---

# FRC-2026 testing requirements (JUnit 5 per subsystem)

> Every subsystem should have JUnit 5 tests; run them with `./gradlew test`.

## Context
Decomposed from the `.standards.md` "Testing Requirements" section during the ACC monolith-decomposition pilot. The `test` task is part of the GradleRIO lifecycle (see `wpilib-gradlerio`); this note carries the standard that tests are expected.

## Observations
- [decision] Every subsystem should have tests under `src/test/java/frc/robot/subsystems/`, using JUnit 5 (`org.junit.jupiter.api.Test`, `org.junit.jupiter.api.Assertions.*`) #testing. Source: `STANDARDS.md` Testing Requirements.
- [decision] Standard tests cover creation (`assertNotNull(subsystem)`) and command factories (assert non-null + `command.getRequirements().contains(subsystem)`) #testing. Source: `STANDARDS.md` Testing Requirements (JUnit 5 example).
- [constraint] Run tests with `./gradlew test` (under the WPILib JDK17 — see `frc-2026-build-env-java17`) #testing. Source: `STANDARDS.md` Testing Requirements + Build Commands.

```java
class ExampleSubsystemTest {
    @Test
    void testCreation() {
        ExampleSubsystem subsystem = new ExampleSubsystem();
        assertNotNull(subsystem);
    }

    @Test
    void testCommandFactory() {
        ExampleSubsystem subsystem = new ExampleSubsystem();
        Command command = subsystem.exampleCommand();
        assertNotNull(command);
        assertTrue(command.getRequirements().contains(subsystem));
    }
}
```

## Relations
- relates-to [[frc-2026]] (testing standard for the 2026 robot)
- relates-to [[wpilib-gradlerio]] (the `test` gradle task)
- relates-to [[frc-2026-api-standards]] (the broader standard set)
