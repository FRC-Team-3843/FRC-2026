# FRC-2026 - [Game TBA]

FRC Team 3843 - 2026 Season

## Overview
Team 3843's robot for the 2026 season. Features a modern swerve drive system powered by YAGSL with vision-assisted localization via PhotonVision. Built using the latest WPILib 2026 APIs with strict command-based architecture and dependency injection patterns.

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

### Mechanisms
*[To be determined based on 2026 game reveal]*

## Software Stack
- WPILib 2026.1.1+
- YAGSL (Swerve Drive Library)
- PathPlanner (Autonomous pathfinding)
- PhotonVision (AprilTag vision)
- REVLib 2025+ (SparkMax with config objects)
- Phoenix6 v25+ (TalonFX with control requests)
- Command-Based Framework (strict dependency injection)

## CAN ID Assignments

### Swerve Drive System
| Device | CAN ID | Description |
|--------|--------|-------------|
| FL Drive Motor | 1 | Front left drive |
| FR Drive Motor | 2 | Front right drive |
| BL Drive Motor | 3 | Back left drive |
| BR Drive Motor | 4 | Back right drive |
| FL Steer Motor | 5 | Front left steering |
| FR Steer Motor | 6 | Front right steering |
| BL Steer Motor | 7 | Back left steering |
| BR Steer Motor | 8 | Back right steering |
| FL CANCoder | 9 | Front left absolute encoder |
| FR CANCoder | 10 | Front right absolute encoder |
| BL CANCoder | 11 | Back left absolute encoder |
| BR CANCoder | 12 | Back right absolute encoder |

### Mechanisms
*[CAN IDs 20-99 reserved for mechanism motors - assignments TBD]*

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
- **AprilTag Field:** Currently set to 2025 Reefscape (update when 2026 field released)
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
2. **Ballistics Layer:** Distance → shooter settings (external JSON lookup table)
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
- **Phase:** Pre-season development (game not announced)
- **Focus:** Robust swerve drive foundation with vision integration
- **Next Steps:** Mechanism design pending game reveal

## Resources
- [WPILib Docs](https://docs.wpilib.org/en/stable/)
- [YAGSL Documentation](https://broncbotz3481.github.io/YAGSL-Example/)
- [PathPlanner](https://pathplanner.dev/pplib-getting-started.html)
- [PhotonVision](https://docs.photonvision.org/)
