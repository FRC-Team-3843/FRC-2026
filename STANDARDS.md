# FRC-2026 Technical Standards

**SINGLE SOURCE OF TRUTH** for all FRC-2026 coding standards.

All agents (Claude, Cursor, Gemini) working in FRC-2026 projects must follow these standards. Project-level configs include this content for IDE agent accessibility.

**Team:** FRC 3843
**Season:** 2026
**Java Version:** 17
**Framework:** WPILib Command-Based (strict dependency injection)

---

## Architecture Standards

### Command-Based Framework (MANDATORY)

All 2026 code **MUST** adhere to the Command-Based Framework with strict dependency injection.

**Core Principles:**
1. **Strict Dependency Injection** - Subsystems are passed into Commands via constructors. NO global static access (e.g., `Robot.driveSubsystem`).
2. **Subsystems** - Handle hardware interaction and low-level control loops.
3. **Commands** - Handle state logic and sequencing.
4. **RobotContainer** - The ONLY place where Subsystems and Commands are instantiated and bound to triggers.
5. **Robot.java** - Minimal entry point, delegates to CommandScheduler.

**Architecture Hierarchy:**
```
Robot.java (minimal entry point)
    └─> RobotContainer.java (instantiation & bindings ONLY)
            ├─> Subsystems (hardware control)
            │       └─> Command Factories (preferred for simple logic)
            └─> Commands (complex state logic)
```

### Modern Command Factories (2026 Preferred)

Prefer inline command factories defined in subsystems over creating separate `Command` classes for simple actions.

**Subsystem Pattern:**
```java
// ExampleSubsystem.java
public class Intake extends SubsystemBase {
    private final SparkMax motor = new SparkMax(1, MotorType.kBrushless);

    // COMMAND FACTORY: Simple "Run Until" command
    public Command runIntakeCommand() {
        return this.runEnd(
            () -> motor.set(0.5),   // Execute loop
            () -> motor.set(0.0)    // End action
        ).withName("RunIntake");
    }

    // COMMAND FACTORY: Instant action
    public Command stopCommand() {
        return this.runOnce(() -> motor.set(0.0));
    }

    // COMMAND FACTORY: Complex sequence with lambda
    public Command scoreCommand() {
        return this.runOnce(() -> motor.set(1.0))
                   .withTimeout(1.0)
                   .andThen(stopCommand());
    }
}
```

**RobotContainer Binding:**
```java
// RobotContainer.java
driverController.a().whileTrue(intake.runIntakeCommand());
driverController.b().onTrue(intake.stopCommand());
```

**Only create separate Command classes for:**
- Complex multi-subsystem coordination
- State machines with multiple steps
- Commands needing instance variables

---

## 2026 Motor API Reference (BREAKING CHANGES)

### 1. REVLib (SparkMax) - Version 2025+

**CRITICAL:** `CANSparkMax` is REMOVED. Use `SparkMax`.
**Configuration:** Direct setters (e.g., `motor.setIdleMode()`) are REMOVED. Use `SparkMaxConfig`.

**Required Imports:**
```java
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
```

**Class Changes:**
| Old (2024) | New (2026) |
|------------|------------|
| `CANSparkMax` | `SparkMax` |
| `SparkPIDController` | `SparkClosedLoopController` |
| Direct setter methods | `SparkMaxConfig` builder pattern |

**Configuration Pattern:**
```java
SparkMax motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
SparkClosedLoopController closedLoop = motor.getClosedLoopController();
RelativeEncoder encoder = motor.getEncoder();
SparkMaxConfig config = new SparkMaxConfig();

// Basic configuration (builder pattern)
config.inverted(true)
      .smartCurrentLimit(40)
      .idleMode(SparkBaseConfig.IdleMode.kBrake);

// Encoder conversion
config.encoder.positionConversionFactor(1.0)
              .velocityConversionFactor(1.0);

// Closed-loop control
config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(0.1).i(0).d(0.01)
    .outputRange(-1, 1);

// MAXMotion (optional)
config.closedLoop.maxMotion
    .allowedClosedLoopError(0.8)
    .maxVelocity(7000)
    .maxAcceleration(28000);

// Apply configuration
motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
```

**Control:**
```java
// Percent output
motor.set(0.5);

// Position control (MAXMotion)
closedLoop.setReference(targetPos, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);

// Velocity control
closedLoop.setReference(targetVel, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
```

### 2. Phoenix5 (TalonSRX) - Version 5.35+

**Note:** Unchanged from 2024, but Phoenix6 is preferred for new code.

**Imports:**
```java
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
```

**Pattern:**
```java
TalonSRX motor = new TalonSRX(MOTOR_ID);
motor.setInverted(true);
motor.set(TalonSRXControlMode.PercentOutput, 0.5);
motor.set(ControlMode.MotionMagic, position);
motor.set(ControlMode.Position, position);
```

### 3. Phoenix6 (TalonFX) - Version 25+

**Standard:** Use `TalonFX` with control request objects (`MotionMagicVoltage`, `PositionVoltage`).
**Phoenix Pro:** Team does NOT use Phoenix Pro. Avoid Pro-only APIs (e.g., `TorqueCurrentFOC`, Fused/Sync feedback sources, MotionMagic Expo variants). Use voltage/duty-cycle/position/velocity requests only.

**Required Imports:**
```java
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
```

**Configuration Pattern:**
```java
TalonFX motor = new TalonFX(MOTOR_ID, "rio"); // or CANBus name
TalonFXConfiguration config = new TalonFXConfiguration();
MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

// Motor output config
config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

// PID config
config.Slot0.kP = 5;
config.Slot0.kI = 0;
config.Slot0.kD = 0;

// Motion Magic config
config.MotionMagic.MotionMagicAcceleration = 100;
config.MotionMagic.MotionMagicCruiseVelocity = 500;

// Apply configuration
motor.getConfigurator().apply(config);
```

**Control:**
```java
// Motion Magic control
motor.setControl(motionMagic.withPosition(targetPos));

// Percent output
motor.set(0.5);

// Read position
double pos = motor.getPosition().getValue().magnitude();
```

### 4. WPILib Command-Based (2026)

**Standard:** Use `CommandXboxController` with modern trigger API.

**Imports:**
```java
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
```

**Controller Pattern:**
```java
// In RobotContainer
CommandXboxController driver = new CommandXboxController(0);

// Trigger bindings
driver.a().onTrue(command);                           // Press once
driver.b().whileTrue(continuousCommand);              // Hold
driver.x().onFalse(releaseCommand);                   // On release
driver.y().toggleOnTrue(toggleCommand);               // Toggle
driver.rightBumper().and(driver.a()).onTrue(combo);   // Compound
```

---

## Naming Conventions

### General

- **Classes:** `PascalCase` (e.g., `DriveSubsystem`, `Superstructure`)
- **Methods/Variables:** `camelCase` (e.g., `getPose()`, `targetVelocity`)
- **Constants:** `UPPER_SNAKE_CASE` (e.g., `MAX_VELOCITY_MPS`, `DRIVE_MOTOR_ID`)

### WPILib Specifics

- **Subsystems:** Suffix with `Subsystem` is optional but encouraged (e.g., `Drive` vs `DriveSubsystem`)
- **Commands:**
  - Factory methods: Verb phrases (e.g., `intakeNote()`, `aimAtSpeaker()`)
  - Command Classes: Noun/Verb phrases (e.g., `IntakeCommand`, `DriveToPose`)
- **Units:** Include units in variable names if not using Units library (e.g., `targetPosMeters`, `durationSeconds`)

### Constants.java Structure

Use nested classes for organization:

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

---

## CAN Bus Base Assignments

**Standard Ranges:**
```
0       - Power Distribution Panel
1-4     - Drive Motors (FL, FR, RL, RR)
5-8     - Steer Motors (FL, FR, RL, RR)
9-12    - Pod Angle Sensors (FL, FR, RL, RR)
20-99   - Mechanism motors (project-specific)
```

**Project-specific assignments:** See project-level CLAUDE.md for exact motor IDs.

---

## Build Commands

Run from the specific robot project directory (e.g., `FRC-2026/2026Robot/`).

```bash
# Build the project
./gradlew build

# Deploy to RoboRIO (requires network connection to robot)
./gradlew deploy

# Run tests
./gradlew test

# Run simulation with GUI
./gradlew simulateJava

# Clean build artifacts
./gradlew clean
```

**Team number** is configured in `.wpilib/wpilib_preferences.json` (Team 3843).

---

## Key Vendor Libraries

- **Phoenix6** - CTRE TalonFX motor controllers (preferred)
- **Phoenix5** - CTRE TalonSRX motor controllers (legacy)
- **REVLib** - REV SparkMax/NEO motors
- **YAGSL** - Swerve drive library
- **PathplannerLib** - Autonomous path planning
- **PhotonLib** - Vision processing

Vendor dependencies are JSON files in `vendordeps/`.

---

## Hardware Patterns

### Standard Patterns

- TalonFX motors use Phoenix6 API with MotionMagic for position control
- Arm/elevator positions defined as encoder rotations in Constants.java
- Servos use WPILib Servo class with angle control (0-200 degrees)

### Subsystem Standard Pattern

```java
public class ExampleSubsystem extends SubsystemBase {
    // 1. Hardware (private final)
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    // 2. Constructor (configure hardware here)
    public ExampleSubsystem() {
        motor = new SparkMax(Constants.ExampleConstants.MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false).smartCurrentLimit(40);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // 3. Command factories (public, return Command)
    public Command exampleCommand() {
        return this.runEnd(
            () -> motor.set(0.5),
            () -> motor.set(0)
        );
    }

    // 4. Periodic (minimal - prefer commands)
    @Override
    public void periodic() {
        // Only for: logging, dashboard updates, continuous monitoring
    }
}
```

---

## Testing Requirements

### JUnit 5

Every subsystem should have tests:

```java
// src/test/java/frc/robot/subsystems/ExampleSubsystemTest.java
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

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

**Run tests:** `./gradlew test`

---

## Legacy Code Warnings

### DO NOT Copy from FRC-2024 or FRC-2025

Previous seasons contain deprecated patterns:

**FORBIDDEN Patterns:**
```java
// Global static subsystem access
Robot.driveSubsystem.drive(x, y, rotation);

// Monolithic Robot.java with hardware
public class Robot extends TimedRobot {
    CANSparkMax motor = new CANSparkMax(1, MotorType.kBrushless);
    public void teleopPeriodic() {
        motor.set(joystick.getY());  // NO!
    }
}

// Old REVLib API (removed in 2026)
CANSparkMax motor = new CANSparkMax(ID, MotorType.kBrushless);  // Class doesn't exist!
motor.setInverted(true);  // Method removed!
motor.setIdleMode(IdleMode.kBrake);  // Method removed!
```

**CORRECT (2026):**
```java
// Dependency injection
public class DriveCommand extends Command {
    private final DriveSubsystem drive;
    public DriveCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }
}

// Command-based with proper subsystems
public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }
}

// Config objects (2026 REVLib)
SparkMax motor = new SparkMax(ID, MotorType.kBrushless);
SparkMaxConfig config = new SparkMaxConfig();
config.inverted(true).idleMode(SparkBaseConfig.IdleMode.kBrake);
motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
```

---

## Common Build Errors (2026 Migration)

| Error | Cause | Fix |
|-------|-------|-----|
| `cannot find symbol: CANSparkMax` | Using old API | Change to `SparkMax` |
| `cannot find symbol: method setInverted(boolean)` | Direct setter removed | Use `SparkMaxConfig` |
| `incompatible types: ControlMode` | Phoenix5 vs Phoenix6 | Use Phoenix6 control request objects |

---

## Resources

- **WPILib Docs:** https://docs.wpilib.org/en/stable/
- **REVLib API:** https://codedocs.revrobotics.com/java/
- **Phoenix6 API:** https://api.ctr-electronics.com/phoenix6/release/java/
- **YAGSL Docs:** https://yagsl.gitbook.io/yagsl
- **PathPlanner:** https://pathplanner.dev/home.html
- **Command-Based Guide:** https://docs.wpilib.org/en/stable/docs/software/commandbased/

---

**Last Updated:** 2026-01-23
**Maintained By:** All agents (Claude, Cursor, Gemini)
