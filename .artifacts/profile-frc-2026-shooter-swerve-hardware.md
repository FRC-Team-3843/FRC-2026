---
id: frc-2026-shooter-swerve-hardware
model: unattributed
model_basis: unattributed
title: FRC-2026 hardware — double-turret shooter, swerve config, vision split
schema_version: 2
artifact_kind: memory
memory_class: semantic
created: 2026-06-12T06:30:00Z
updated: 2026-06-12T12:05:00Z
valid_until: null
author: claude
session: phase7-onboarding-20260612
tags: [frc, architecture, robotics, vision]
aliases: [double turret shooter, mirrored turrets, mk4 swerve, photonvision coprocessor, rebuilt robot]
status: active
supersedes: null
confidence: 55
source_basis: conversation
human_edited: false
sensitivity: normal
decisions: []
artifact_kind: memory
memory_class: semantic
semantic_kind: state
---

# FRC-2026 hardware — double-turret shooter, swerve config, vision split

> The REBUILT robot is a mirrored double-turret shooter on a YAGSL swerve base with PhotonVision localization offloaded to a Beelink coprocessor. Captures the durable hardware decisions plus the live, hardware-tuned gotchas, and flags a swerve-ratio discrepancy between the repo docs and external memory.

## Context
Distilled during ACC Phase 7 onboarding from `README.md`, `NOTES.md`, and `2026Robot\TODO-COMPETITION.md`. Numbers below are document-sourced (competition was March 2026); treat tuned values as point-in-time hardware state, not eternal truths.

## Observations
- [decision] Mirrored double-turret shooter: two NEO 550 turrets (52.987:1, 11t>68t / 14t>120t, +/-110 deg, both locked to operator left stick, field-centric aim) #architecture. Source: `README.md` Mechanisms; `NOTES.md` Mechanism Hardware Summary.
- [decision] Per side: Kraken X44 preshooter (0.625 overdrive) + Kraken X60 main shooter (0.917 overdrive, 4" wheel, max ~5500 RPM), velocity PID #architecture. Source: `NOTES.md` Mechanism Hardware Summary + CAN table; `TODO-COMPETITION.md` Tuned Values.
- [decision] Intake chain: Kraken X44 ground roller (CAN 30) -> NEO 550 floor conveyor (31) -> Bag-motor hopper on TalonSRX/Phoenix5 (32) -> NEO 550 feeder (33); orchestrated by `Superstructure` #architecture. Source: `README.md`; `NOTES.md` CAN Bus Assignments.
- [decision] Vision compute split: heavy AprilTag detection + pose estimation on a Beelink coprocessor running PhotonVision headless; RoboRIO only fuses time-stamped poses + does turret aim #vision (keeps RIO real-time control loops free). Source: `README.md` Vision; `NOTES.md` Section 8.1.
- [decision] AprilTag field layout = `AprilTagFields.k2026RebuiltAndymark` (lowercase 'm'), set 2026-02-15, confirmed compiles #vision. Source: `NOTES.md` Section 3.3 (DONE); `README.md`.
- [constraint] `VisionConstants.ENABLE_VISION` stays false until the right camera is physically mounted + calibrated; the coded camera transforms (`right_cam`/`left_cam`) are estimates, not surveyed values #vision. Source: `README.md` Vision status; `NOTES.md` Section 3.2; `TODO-COMPETITION.md`.
- [constraint] Bevel-gear orientation per swerve module governs `inverted.drive` in each module JSON; mixed bevels make wheels fight (some forward, some back) #robotics. Source: `NOTES.md` Section 2.2 + Section 7.
- [constraint] Turret PID is asymmetric by hardware: left kP=0.15 @ 70% output limit (high friction), right kP=0.10 @ 50% — do not "normalize" them #robotics. Source: `TODO-COMPETITION.md` Tuned Values.
- [question] Main-shooter TalonFX CAN 38 has an open firmware error needing a Phoenix6 flash; swerve-drive SysId and shooter/preshooter SysId still owed #robotics. Source: `TODO-COMPETITION.md` STILL NEEDED.
- [decision] Swerve stays STOCK MK4 L1 orientation (8.14:1 drive / 12.8:1 angle), NO inversion plate — the planned module inversion (which required a ratio change; the externally remembered 5.8267:1 figure belonged to that plan) was abandoned when that subteam failed the task with no time to correct; the team improvised with the stock config #robotics (user-confirmed 2026-06-12, resolving the doc-vs-memory conflict below). Source: user statement 2026-06-12; `NOTES.md` lines ~208-212.

## Open Questions
- [x] RESOLVED 2026-06-12 (user): swerve gearing conflict closed — repo docs are correct (MK4 L1, 8.14:1 / 12.8:1, stock orientation, max ~3.71 m/s). The conflicting external-memory values (5.8267:1 / 12:1 + "custom inversion plate") described a PLANNED inversion that was never executed: the subteam failed the task with no time to correct, and the team improvised with stock modules. Transition captured as a [decision] in Observations above.

## Notes for Future Sessions
CAN map: swerve 11-22, mechanisms 30-39, hood servos on PWM 0/1, NavX on SPI (no CAN ID). Coprocessor reachable on the robot network at `http://10.38.43.11:5800` (PhotonVision), team number 3843, NT in client mode. Full controller bindings + CAN tables live in `2026Robot\TODO-COMPETITION.md` and `NOTES.md` Section 0B — cite those rather than recopying.

## Relations
[[frc-2026]] [[frc-team-3843]] [[beelink]]
