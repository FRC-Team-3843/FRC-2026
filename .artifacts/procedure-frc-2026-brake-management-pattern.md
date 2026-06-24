---
id: frc-2026-brake-management-pattern
title: FRC-2026 brake management pattern (disabled-mode brake delay)
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
tags: [frc, reference, pattern, brake]
aliases: [brake management, disabled brake delay, wheel_lock_time, motor brake release]
status: active
supersedes: null
confidence: 58
source_basis: document
human_edited: false
sensitivity: normal
decisions: []
load_profile: scope_entry
load_priority: 155
source_rel: FRC-2026\.standards.md
scope: FRC-2026
---

# FRC-2026 brake management pattern (disabled-mode brake delay)

> The standard `Robot.java` pattern for releasing motor brakes after a delay once disabled (so the robot can be repositioned), and re-engaging brakes on auto/teleop init.

## Context
Decomposed from the `.standards.md` "Brake Management Pattern" section during the ACC monolith-decomposition pilot. Reference code pattern for the disabled-mode brake delay.

## Pattern

```java
// Robot.java
private final Timer m_disabledTimer = new Timer();

@Override
public void disabledInit() {
  m_disabledTimer.reset();
  m_disabledTimer.start();
}

@Override
public void disabledPeriodic() {
  if (m_disabledTimer.hasElapsed(Constants.DriveConstants.WHEEL_LOCK_TIME)) {
    m_robotContainer.setMotorBrake(false);
    m_disabledTimer.stop();
  }
}

@Override
public void autonomousInit() {
  m_robotContainer.setMotorBrake(true);
  // ... rest of auto init
}

@Override
public void teleopInit() {
  m_robotContainer.setMotorBrake(true);
  // ... rest of teleop init
}
```

### Constant

```java
public static final class DriveConstants {
  public static final double WHEEL_LOCK_TIME = 10.0;  // Seconds before releasing brakes
}
```

## Relations
- relates-to [[frc-2026]] (a Robot.java pattern for the 2026 robot)
- relates-to [[frc-2026-api-standards]] (the broader standard set)
