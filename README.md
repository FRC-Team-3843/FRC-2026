# FRC-2026 - REBUILT

FRC Team 3843 - 2026 Season

> **Documentation Guide:**
> - **This file (README):** Project overview and quick start
> - **NOTES.md:** Setup procedures, tuning values, troubleshooting, game details
> - **STANDARDS.md:** Coding standards and architecture rules

## Overview
Team 3843's robot for the 2026 REBUILT season. Features a modern swerve drive system powered by YAGSL with vision-assisted localization via PhotonVision. Built using the latest WPILib 2026 APIs with strict command-based architecture and dependency injection patterns.

## 2026 Game: REBUILT

In REBUILT (presented by Haas), alliances score **Fuel** into their **Hub**, navigate field obstacles, and climb the **Tower** before time runs out. The unique mechanic is **Hub shifting** -- alliance hubs alternate between active and inactive during teleop based on autonomous performance.

**Key Robot Capabilities Needed:**
- **Fuel intake & shooting** -- collect 5.91" diameter balls and score into the Hub
- **Field traversal** -- cross Bumps (~6.5" ramps) or drive through Trenches (22.25" clearance)
- **Tower climbing** -- hang on rungs at up to 3 levels for endgame points (up to 30 pts each)
- **High cycle rate** -- 504 fuel on field, alliance needs 360 scored for max Ranking Points

*See NOTES.md Section 0 for full game details and scoring breakdown.*

## Hardware

### Drivetrain
- **Type:** Swerve Drive (4-wheel independent steering)
- **Library:** YAGSL (Yet Another Generic Swerve Library)
- **Wheelbase:** 27" (13.5" half-width configuration)
- **Control:** Field-centric with NavX gyro and vision fusion

### Vision System
- **Library:** PhotonVision
- **AprilTag Detection:** Field localization and pose estimation
- **Coprocessor:** Intel N100 + Arducam (planned dual camera setup)
- **Strategy:** Heavy vision processing on coprocessor, pose fusion on RoboRIO

### Mechanisms (REBUILT Game)
*[Design in progress -- see NOTES.md Section 0 for game requirements]*

**Required for REBUILT:**
- Fuel intake (collect 5.91" balls from depot, neutral zone, human player)
- Fuel shooter/scorer (deposit into Hub -- top opening of 47" x 47" structure)
- Tower climber (3 rung levels, rungs are 1-1/4" Sch 40 pipe, 18" apart)
- Field traversal (Bumps at ~6.5" or Trench at 22.25" clearance)

*See NOTES.md for CAN bus assignments*

## Software Stack
- WPILib 2026.1.1+
- YAGSL (Swerve Drive Library)
- PathPlanner (Autonomous pathfinding)
- PhotonVision (AprilTag vision)
- REVLib 2025+ (SparkMax with config objects)
- Phoenix6 v25+ (TalonFX with control requests, no Phoenix Pro features)
- Command-Based Framework (strict dependency injection)

## Architecture

### Command-Based Framework
This codebase strictly follows WPILib's command-based architecture:
- **Subsystems** handle hardware interaction and low-level control
- **Command Factories** in subsystems for simple operations
- **RobotContainer** is the ONLY place for instantiation and bindings
- **Strict dependency injection** - no global static subsystem access

### Modern Patterns (2026)
- SparkMax configuration uses `SparkMaxConfig` builder pattern
- TalonFX control uses request objects (`MotionMagicVoltage`, `PositionVoltage`)
- Command factories preferred over separate Command classes for simple logic
- Hardware abstraction layer for testability

## Building and Deploying
```bash
cd 2026Robot
./gradlew build
./gradlew deploy

# Run tests
./gradlew test

# Simulation
./gradlew simulateJava
```

## Vision & Localization
- **Pose Estimator:** Fuses vision measurements with odometry
- **AprilTag Field:** Currently set to 2025 Reefscape (**UPDATE NEEDED** to 2026 REBUILT layout)
- **Trust Settings:** Configurable standard deviations for vision vs odometry
- **Camera Transform:** Robot-to-camera measurements in Vision.java

## Autonomous
PathPlanner-based autonomous with:
- Pre-planned trajectories for reliable execution
- On-the-fly pathfinding for adaptive navigation
- Auto PID tuning via SysId for optimal performance

## Competitive Advantage Strategy

### Targeting Pipeline (Planned)
1. **Perception Layer:** Robot pose + target pose from vision
2. **Ballistics Layer:** Distance -> shooter settings (external JSON lookup table)
3. **Aim/Lead Layer:** Turret angles with optional motion compensation

### Driver Assist Modes (Planned)
- Auto path to pose (PathPlanner pathfind)
- Auto align heading to target
- Micro adjust (rate-limited or position-nudge)

### Telemetry Strategy
- Full data logging to DataLog for AdvantageScope replay
- Minimal live NetworkTables for low bandwidth
- Tuning mode for rich drivetrain diagnostics

## Additional Documentation
*For detailed setup and configuration, see NOTES.md*

## Development Status
- **Phase:** Active development (game: REBUILT)
- **Focus:** Swerve drive foundation complete, mechanism design in progress
- **Next Steps:**
  - Design and implement fuel intake/shooter mechanisms
  - Implement tower climbing mechanism
  - Update Vision.java to 2026 REBUILT AprilTag field layout
  - Mount cameras and calibrate PhotonVision
  - Implement driver assist commands for Hub scoring

## Resources
- [WPILib Docs](https://docs.wpilib.org/en/stable/)
- [YAGSL Documentation](https://broncbotz3481.github.io/YAGSL-Example/)
- [PathPlanner](https://pathplanner.dev/pplib-getting-started.html)
- [PhotonVision](https://docs.photonvision.org/)
