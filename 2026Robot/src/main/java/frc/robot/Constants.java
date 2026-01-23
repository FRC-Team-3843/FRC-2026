package frc.robot;

/**
 * Centralized configuration constants for the 2026 robot.
 *
 * <p>All hardware IDs, physical measurements, and tuning values belong here.
 * When you need to change a value (speed, PID, CAN ID, etc.), look here FIRST
 * rather than modifying subsystem code directly.
 *
 * <p>Naming convention: constants use UPPER_SNAKE_CASE.
 * Inner classes group related constants (e.g., DriveProfiles, AutonConstants).
 *
 * <p>See also: Notes.txt Section 4 for JSON config file documentation.
 */
public final class Constants {

  // ──────────────────────────────────────────────────────────────────────────
  // ROBOT PHYSICAL PROPERTIES
  // ──────────────────────────────────────────────────────────────────────────

  /**
   * Maximum chassis speed in meters per second.
   *
   * <p>Calculated from hardware specs:
   * <ul>
   *   <li>NEO free speed: 5676 RPM</li>
   *   <li>L1 drive gear ratio: 8.14:1</li>
   *   <li>Wheel diameter: 4 inches (0.1016m)</li>
   *   <li>Formula: (5676 / 60) / 8.14 * (0.1016 * pi) = 3.71 m/s</li>
   * </ul>
   *
   * <p>This is the THEORETICAL maximum. Real-world speed will be slightly lower
   * due to friction, battery sag under load, and current limits.
   */
  public static final double MAX_SPEED = 3.71; // meters per second

  /**
   * Robot mass in kilograms including battery and bumpers.
   * Approximately 105 lbs total.
   *
   * <p>Used by PathPlanner for feedforward calculations.
   * Update this value after weighing the competition-ready robot.
   */
  public static final double ROBOT_MASS = 47.6; // kg

  /**
   * Effective control loop time in seconds.
   *
   * <p>This accounts for the 20ms robot periodic loop PLUS the ~110ms measurement
   * delay inherent to SparkMax encoder readings over CAN. YAGSL uses this value
   * internally for velocity calculations and angular velocity compensation.
   *
   * <p>If you switch to TalonFX motors (which have faster CAN updates), reduce this
   * to approximately 0.04 seconds.
   */
  public static final double LOOP_TIME = 0.13; // seconds

  // ──────────────────────────────────────────────────────────────────────────
  // DRIVEBASE BEHAVIOR
  // ──────────────────────────────────────────────────────────────────────────

  public static final class DrivebaseConstants {
    /**
     * Time in seconds to hold motor brake mode after the robot is disabled,
     * before switching to coast mode.
     *
     * <p>Purpose: After a match ends or robot is disabled, brake mode prevents
     * the robot from rolling off the field or a ramp. After this timeout, motors
     * switch to coast mode so the robot can be pushed freely in the pits.
     *
     * <p>This timer is managed in Robot.java's disabledPeriodic() method.
     * Increase if your robot tends to roll after matches (e.g., on a ramp).
     */
    public static final double WHEEL_LOCK_TIME = 10.0; // seconds
  }

  // ──────────────────────────────────────────────────────────────────────────
  // OPERATOR INTERFACE
  // ──────────────────────────────────────────────────────────────────────────

  public static final class OperatorConstants {
    /**
     * Joystick deadband applied to all drive axes.
     *
     * <p>Any stick input below this threshold is treated as zero.
     * This prevents drift from slightly off-center joysticks.
     *
     * <p>Typical values: 0.05 (tight) to 0.2 (loose).
     * If the robot creeps when sticks are released, increase this value.
     * If the robot feels unresponsive at low speeds, decrease this value.
     */
    public static final double DEADBAND = 0.15;

    /**
     * Driver controller USB port number (set in Driver Station USB tab).
     * The driver controls robot translation and rotation.
     */
    public static final int DRIVER_CONTROLLER_PORT = 0;

    /**
     * Operator controller USB port number.
     * The operator controls mechanisms (arm, intake, etc.) once added.
     */
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // DRIVER ASSIST & CONTROL MODES
  // ─────────────────────────────────────────────────────────────────────────────

  /**
   * Driver assist and control mode configuration.
   *
   * <p>Keep these as simple toggles so we can A/B test without touching command code.
   * Only one drive control mode should be active at a time.
   */
  public static final class AssistConstants {
    public enum DriveControlMode {
      STANDARD,            // Direct YAGSL drive (lowest CPU usage)
      SETPOINT_GENERATOR   // Smoother acceleration using setpoint generator
    }

    /** Select the default drive control mode for teleop. */
    public static final DriveControlMode DEFAULT_DRIVE_MODE = DriveControlMode.STANDARD;

    public enum DriverAssistMode {
      OFF,
      AUTO_ALIGN,
      PATH_TO_POSE,
      SNAP_HEADING,
      MICRO_ADJUST_RATE,
      MICRO_ADJUST_NUDGE
    }

    /** Select the default assist mode (only one active at a time). */
    public static final DriverAssistMode DEFAULT_ASSIST_MODE = DriverAssistMode.OFF;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // DRIVE PROFILES
  // ──────────────────────────────────────────────────────────────────────────

  /**
   * Configurable drive profiles for speed scaling.
   *
   * <p>Profiles allow the driver to switch between different speed modes during a match:
   * <ul>
   *   <li><b>Full speed:</b> Default mode for open-field traversal and defense.</li>
   *   <li><b>Slow mode:</b> Activated by holding left trigger. Used for precision
   *       alignment with game pieces, scoring positions, or close-quarters maneuvering.</li>
   * </ul>
   *
   * <p>SPEED_SCALE is a multiplier on MAX_SPEED (1.0 = 100%, 0.5 = 50%).
   * SPEED_RAMP is the time in seconds for the motor to go from 0 to full output
   * (set in physicalproperties.json, documented here for reference).
   */
  public static final class DriveProfiles {
    /** Full speed multiplier: 100% of MAX_SPEED. Used during normal driving. */
    public static final double FULL_SPEED_SCALE = 1.0;
    /** Full speed ramp rate in seconds. Matches physicalproperties.json rampRate.drive. */
    public static final double FULL_SPEED_RAMP = 0.25;

    /** Slow mode multiplier: 50% of MAX_SPEED. Activated by holding left trigger. */
    public static final double SLOW_SPEED_SCALE = 0.5;
    /** Slow mode ramp rate. Slightly faster response for precision control. */
    public static final double SLOW_SPEED_RAMP = 0.15;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // AUTONOMOUS
  // ──────────────────────────────────────────────────────────────────────────

  /**
   * PID constants for PathPlanner's holonomic drive controller during autonomous.
   *
   * <p>These control how accurately the robot follows generated paths:
   * <ul>
   *   <li><b>Translation PID:</b> Controls position error in X and Y (meters).
   *       Higher P = snappier path following, but can oscillate.</li>
   *   <li><b>Rotation PID:</b> Controls heading error (radians).
   *       Higher P = faster heading correction, but can overshoot.</li>
   * </ul>
   *
   * <p>Starting values of 5.0/0/0 work for most robots. Fine-tune by:
   * 1. Running a simple straight-line path
   * 2. Watching path-following error in PathPlanner telemetry
   * 3. Increasing P if the robot is sluggish, decreasing if it oscillates
   */
  public static final class AutonConstants {
    public static final double TRANSLATION_P = 5.0;
    public static final double TRANSLATION_I = 0.0;
    public static final double TRANSLATION_D = 0.0;

    public static final double ROTATION_P = 5.0;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // VISION
  // ──────────────────────────────────────────────────────────────────────────

  /**
   * Vision configuration placeholders.
   *
   * <p>Update these values once cameras are physically mounted on the robot.
   * The camera name MUST match the name configured in the PhotonVision UI
   * (accessible via the coprocessor's web interface at photonvision.local:5800).
   *
   * <p>When adding cameras:
   * 1. Mount camera rigidly to the robot frame
   * 2. Measure the transform from robot center to camera lens (X, Y, Z in meters)
   * 3. Measure camera angle (pitch, yaw, roll in degrees)
   * 4. Add a new entry to the Cameras enum in Vision.java
   * 5. Update CAMERA_NAME here to match the PhotonVision UI name
   */
  public static final class VisionConstants {
    /** PhotonVision camera name - must match the name configured in the PV UI. */
    public static final String CAMERA_NAME = "YOUR_CAMERA_NAME";

    /**
     * Global vision enable.
     *
     * <p>Keep false until cameras + coprocessor are ready. The robot should run
     * normally with vision disabled (failsafe).
     */
    public static final boolean ENABLE_VISION = false;

    /**
     * Maximum acceptable vision latency in seconds before rejecting measurements.
     * Adjust after initial testing.
     */
    public static final double MAX_LATENCY_SECONDS = 0.25;
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // SHOOTER MODEL (FUTURE)
  // ─────────────────────────────────────────────────────────────────────────────

  public static final class ShooterConstants {
    /** Enable loading shooter lookup table JSON (kept false until shooter is ready). */
    public static final boolean ENABLE_SHOT_TABLE = false;

    /** Path relative to deploy directory for shooter table JSON. */
    public static final String SHOT_TABLE_PATH = "shooter/shooter_table.json";
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // TELEMETRY
  // ─────────────────────────────────────────────────────────────────────────────

  public static final class TelemetryConstants {
    /** Global telemetry enable (keep false while tuning drivetrain). */
    public static final boolean ENABLE_TELEMETRY = false;

    /** Enable higher-detail telemetry for tuning sessions. */
    public static final boolean TUNING_MODE = false;

    /** Publish rate in Hz for normal NetworkTables telemetry. */
    public static final double PUBLISH_HZ = 10.0;

    /** Publish rate in Hz for tuning telemetry (keep modest to protect RoboRIO 1). */
    public static final double TUNING_PUBLISH_HZ = 20.0;
  }
}
