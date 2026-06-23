---
id: frc-2026-naming-conventions
title: FRC-2026 Java naming conventions + Constants.java structure
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
tags: [frc, standards, naming, style]
aliases: [frc naming conventions, m_ prefix, constants nested classes, java style frc]
status: active
supersedes: null
confidence: 60
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
load_profile: scope_entry
load_priority: 60
source_rel: FRC-2026\.standards.md
enforceability: preferred
---

# FRC-2026 Java naming conventions + Constants.java structure

> The team's Java naming rules and the nested-class layout for `Constants.java`, so new code matches the existing style.

## Context
Decomposed from the `.standards.md` "Naming Conventions" section during the ACC monolith-decomposition pilot. These are style rules, enforced by convention rather than tooling.

## Observations
- [decision] Classes `PascalCase` (`DriveSubsystem`, `Superstructure`); member variables `m_` + camelCase (`m_driveMotor`); local variables camelCase, no prefix; methods camelCase (`getPose()`); constants `UPPER_SNAKE_CASE` (`MAX_VELOCITY_MPS`, `DRIVE_MOTOR_ID`) #style. Source: `STANDARDS.md` Naming Conventions / General.
- [decision] WPILib specifics: `Subsystem` suffix optional but encouraged; command factory methods are verb phrases (`intakeNote()`, `aimAtSpeaker()`); command classes are noun/verb phrases (`IntakeCommand`, `DriveToPose`); include units in variable names when not using the Units library (`targetPosMeters`, `durationSeconds`) #style. Source: `STANDARDS.md` Naming Conventions / WPILib Specifics.
- [decision] `Constants.java` uses nested `public static final class` blocks per subsystem for organization #style. Source: `STANDARDS.md` Constants.java Structure.

```java
public final class Constants {
    public static final class DriveConstants {
        public static final int FL_DRIVE_ID = 1;
        public static final double MAX_SPEED_MPS = 4.5;
    }
    public static final class ArmConstants {
        public static final int MOTOR_ID = 20;
        public static final double MAX_ANGLE_DEG = 120.0;
    }
}
```

## Notes for Future Sessions
Note the live exception: shooter hood angles do NOT live in `Constants.java` — they are runtime-loaded from `deploy/hood-config.json` (see `hood-config-json`).

## Relations
- relates-to [[frc-2026]] (code style for the 2026 robot)
- relates-to [[frc-2026-api-standards]] (the broader standard set)
- relates-to [[hood-config-json]] (the Constants.java exception)
