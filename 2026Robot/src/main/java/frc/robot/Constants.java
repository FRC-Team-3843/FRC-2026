package frc.robot;

/**
 * Centralized configuration constants for the 2026 robot.
 *
 * <p>All hardware IDs, physical measurements, and tuning values belong here.
 * When you need to change a value (speed, PID, CAN ID, etc.), look here FIRST.
 *
 * <p>INVERSION FLAGS: All motor inversions default to false. Set to true after
 * verifying direction on real hardware. Wrong inversion = mechanism runs backward.
 */
public final class Constants {

  // ──────────────────────────────────────────────────────────────────────────
  // ROBOT PHYSICAL PROPERTIES
  // ──────────────────────────────────────────────────────────────────────────

  /**
   * Maximum chassis speed in meters per second.
   * MK4 L1 standard (no changes): NEO 5676 RPM / 8.14 ratio, 4" wheel.
   * Formula: (5676/60) / 8.14 * (0.1016 * pi) = 3.71 m/s
   */
  public static final double MAX_SPEED = 3.71;

  /** Robot mass in kg including battery and bumpers (~105 lbs). */
  public static final double ROBOT_MASS = 47.6;

  /**
   * Effective control loop time. Accounts for 20ms loop + ~110ms SparkMax CAN delay.
   * Reduce to ~0.04 if switching drive motors to TalonFX.
   */
  public static final double LOOP_TIME = 0.13;

  // ──────────────────────────────────────────────────────────────────────────
  // DRIVEBASE BEHAVIOR
  // ──────────────────────────────────────────────────────────────────────────

  public static final class DrivebaseConstants {
    /** Seconds to hold brake after disable before switching to coast. */
    public static final double WHEEL_LOCK_TIME = 10.0;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // OPERATOR INTERFACE
  // ──────────────────────────────────────────────────────────────────────────

  public static final class OperatorConstants {
    public static final double DEADBAND = 0.15;
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // DRIVER ASSIST & CONTROL MODES
  // ──────────────────────────────────────────────────────────────────────────

  public static final class AssistConstants {
    public enum DriveControlMode {
      STANDARD,
      SETPOINT_GENERATOR
    }

    public static final DriveControlMode DEFAULT_DRIVE_MODE = DriveControlMode.STANDARD;

    public enum DriverAssistMode {
      OFF,
      AUTO_ALIGN,
      PATH_TO_POSE,
      SNAP_HEADING,
      MICRO_ADJUST_RATE,
      MICRO_ADJUST_NUDGE
    }

    public static final DriverAssistMode DEFAULT_ASSIST_MODE = DriverAssistMode.OFF;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // DRIVE PROFILES
  // ──────────────────────────────────────────────────────────────────────────

  public static final class DriveProfiles {
    public static final double FULL_SPEED_SCALE = 1.0;
    public static final double FULL_SPEED_RAMP  = 0.25;
    public static final double SLOW_SPEED_SCALE = 0.5;
    public static final double SLOW_SPEED_RAMP  = 0.15;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // AUTONOMOUS
  // ──────────────────────────────────────────────────────────────────────────

  public static final class AutonConstants {
    public static final double TRANSLATION_P = 5.0;
    public static final double TRANSLATION_I = 0.0;
    public static final double TRANSLATION_D = 0.0;
    public static final double ROTATION_P    = 5.0;
    public static final double ROTATION_I    = 0.0;
    public static final double ROTATION_D    = 0.0;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // VISION
  // ──────────────────────────────────────────────────────────────────────────

  public static final class VisionConstants {
    /**
     * PhotonVision camera names — must match names configured in the PV UI.
     * Set ENABLE_VISION = true once cameras are mounted, calibrated, and network tested.
     */
    public static final String LEFT_CAMERA_NAME  = "left_cam";
    public static final String RIGHT_CAMERA_NAME = "right_cam";
    public static final boolean ENABLE_VISION         = false;
    public static final double  MAX_LATENCY_SECONDS   = 0.25;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // TURRET MECHANISM
  // CAN IDs 34–35 (NEO 550 / SparkMax)
  // ──────────────────────────────────────────────────────────────────────────

  public static final class TurretConstants {
    public static final int LEFT_MOTOR_ID  = 34;
    public static final int RIGHT_MOTOR_ID = 35;

    /**
     * Two-stage turret reduction: 11t→68t (6.182:1) then 14t→120t (8.571:1).
     * Total: (68*120)/(11*14) = 8160/154 = 52.987:1
     */
    public static final double GEAR_RATIO = (68.0 * 120.0) / (11.0 * 14.0); // 52.987:1

    /** Soft limits in motor rotations. 16.2 rot ≈ 110° turret travel. */
    public static final double FORWARD_LIMIT_ROTATIONS = 16.2;
    public static final double REVERSE_LIMIT_ROTATIONS = -16.2;

    /** NEO 550 current limit (amps). */
    public static final int CURRENT_LIMIT_AMPS = 20;

    // Per-motor PID — left turret has 2x friction (kS=1.03V vs 0.54V)
    public static final double LEFT_KP  = 0.15;
    public static final double LEFT_KI  = 0.0;
    public static final double LEFT_KD  = 0.01;
    public static final double LEFT_OUTPUT_LIMIT = 0.7; // higher to overcome friction

    public static final double RIGHT_KP  = 0.10;
    public static final double RIGHT_KI  = 0.0;
    public static final double RIGHT_KD  = 0.008;
    public static final double RIGHT_OUTPUT_LIMIT = 0.5;

    /**
     * Motor inversion flags — set after verifying on hardware.
     * Right turret is likely inverted (mirrored mounting).
     */
    public static final boolean LEFT_INVERTED  = false;
    public static final boolean RIGHT_INVERTED = false; // VERIFY ON HARDWARE

    /**
     * Joystick magnitude deadband for manual turret control.
     * Below this threshold the turret holds position and shooter idles.
     */
    public static final double JOYSTICK_DEADBAND = 0.15;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // SHOOTER MECHANISM
  // CAN IDs 36–39 (Kraken X44/X60 / TalonFX)
  // ──────────────────────────────────────────────────────────────────────────

  public static final class ShooterConstants {
    // CAN IDs
    public static final int LEFT_PRESHOOTER_ID    = 36;
    public static final int RIGHT_PRESHOOTER_ID   = 37;
    public static final int LEFT_MAIN_SHOOTER_ID  = 38;
    public static final int RIGHT_MAIN_SHOOTER_ID = 39;

    /** Main shooter wheel diameter in inches. */
    public static final double SHOOTER_WHEEL_DIAMETER_IN = 4.0;

    /**
     * Gear ratios (motor rotations per output rotation, <1 means overdrive).
     * Preshooter: 0.625 (5:8 overdrive, X44).
     * Main shooter: 66/72 = 0.91667 (overdrive, X60).
     * Motor RPS = (wheel_rpm / 60) * gear_ratio
     */
    public static final double PRESHOOTER_GEAR_RATIO    = 0.625;       // 5:8 overdrive
    public static final double MAIN_SHOOTER_GEAR_RATIO  = 66.0 / 72.0; // 0.91667 overdrive

    /** Preshooter runs at this fraction of main shooter wheel RPM. */
    public static final double PRESHOOTER_SPEED_RATIO = 0.667;

    // Stator current limits
    public static final double PRESHOOTER_CURRENT_LIMIT_AMPS    = 40.0;
    public static final double MAIN_SHOOTER_CURRENT_LIMIT_AMPS  = 60.0;

    /**
     * Maximum wheel RPM at full joystick deflection (magnitude = 1.0).
     * X60 free speed 6000 RPM / 0.917 ratio = 6545 RPM max wheel.
     * At low battery (10.5V) max achievable = ~5727 RPM.
     * 5500 gives consistent performance even at end-of-match battery sag.
     */
    public static final double MAX_WHEEL_RPM = 5500.0;

    /** Velocity PID for TalonFX — tune kP first, add kV for feedforward. */
    public static final double KP = 0.1;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KV = 0.12; // volts per RPS, rough starting estimate

    /** Robot is "at speed" when within this many RPM of target. */
    public static final double AT_SPEED_TOLERANCE_RPM = 150.0;

    /** Default target RPMs used by DashboardManager tunable widgets. */
    public static final double PRESHOOTER_TARGET_RPM   = 3000.0;
    public static final double MAIN_SHOOTER_TARGET_RPM = 4500.0;

    /**
     * Motor inversion flags — set after verifying on hardware.
     * Mirrored mounting means one side is likely inverted.
     */
    public static final boolean LEFT_MAIN_INVERTED  = false;
    public static final boolean RIGHT_MAIN_INVERTED = false; // VERIFY ON HARDWARE
    public static final boolean LEFT_PRE_INVERTED   = false;
    public static final boolean RIGHT_PRE_INVERTED  = false; // VERIFY ON HARDWARE
  }

  // ──────────────────────────────────────────────────────────────────────────
  // HOOD SERVOS
  // PWM channels 0 (left) and 1 (right)
  // ──────────────────────────────────────────────────────────────────────────

  public static final class HoodConstants {
    public static final int LEFT_PWM_CHANNEL  = 0;
    public static final int RIGHT_PWM_CHANNEL = 1;

    /**
     * Servo angles in degrees (WPILib Servo: 0–180).
     * NEAR = close shot, FAR = distance shot.
     * TUNE THESE ON HARDWARE — start at 45/90 and adjust.
     */
    public static final double NEAR_ANGLE = 45.0;
    public static final double FAR_ANGLE  = 90.0;

    /**
     * Joystick magnitude at which hood automatically switches near→far.
     * Only applies in manual mode. Set to 1.1 to disable auto-hood.
     */
    public static final double AUTO_HOOD_THRESHOLD = 0.6;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // INTAKE MECHANISM
  // CAN ID 30 (Kraken X44 / TalonFX)
  // ──────────────────────────────────────────────────────────────────────────

  public static final class IntakeConstants {
    public static final int    MOTOR_ID          = 30;
    public static final double INTAKE_SPEED      = 0.8; // duty cycle, 0–1
    public static final int    CURRENT_LIMIT_AMPS = 40;
    public static final boolean MOTOR_INVERTED   = true;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // FLOOR CONVEYOR
  // CAN ID 31 (NEO 550 / SparkMax)
  // ──────────────────────────────────────────────────────────────────────────

  public static final class ConveyorConstants {
    public static final int    MOTOR_ID          = 31;
    public static final double CONVEYOR_SPEED    = 0.7;
    public static final int    CURRENT_LIMIT_AMPS = 30;
    public static final boolean MOTOR_INVERTED   = false; // VERIFY ON HARDWARE
  }

  // ──────────────────────────────────────────────────────────────────────────
  // HOPPER
  // CAN ID 32 (Bag motor / TalonSRX — Phoenix5)
  // ──────────────────────────────────────────────────────────────────────────

  public static final class HopperConstants {
    public static final int    MOTOR_ID       = 32;
    public static final double HOPPER_SPEED   = 0.6;
    public static final boolean MOTOR_INVERTED = false; // VERIFY ON HARDWARE
  }

  // ──────────────────────────────────────────────────────────────────────────
  // FEEDER (pre-shooter, feeds both sides)
  // CAN ID 33 (NEO 550 / SparkMax)
  // ──────────────────────────────────────────────────────────────────────────

  public static final class FeederConstants {
    public static final boolean ENABLED           = true;
    public static final int    MOTOR_ID          = 33;
    public static final double FEEDER_SPEED      = 0.8;
    public static final int    CURRENT_LIMIT_AMPS = 30;
    public static final boolean MOTOR_INVERTED   = true;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // SHOOTING POSITION PRESETS
  // D-pad selects a preset; turret angles and RPMs are configured here.
  // ALL ANGLES AND RPMS ARE PLACEHOLDERS — TUNE ON HARDWARE.
  // Turret angles: 0° = straight ahead, +90° = right of robot, -90° = left.
  // ──────────────────────────────────────────────────────────────────────────

  public static final class ShootingPositions {

    /**
     * Immutable preset bundling turret angles, RPMs, and hood position.
     * leftTurretDeg / rightTurretDeg: angle from robot forward, degrees.
     * mainRpm / preshooterRpm: target wheel RPM for each motor group.
     * hoodFar: true = FAR_ANGLE, false = NEAR_ANGLE.
     */
    public record ShootingPreset(
        double leftTurretDeg,
        double rightTurretDeg,
        double mainRpm,
        double preshooterRpm,
        boolean hoodFar
    ) {}

    /** D-pad UP: close range (~2m), straight ahead at hub. Hood near. */
    public static final ShootingPreset CLOSE = new ShootingPreset(
        0.0, 0.0, 2500.0, 1668.0, false
    );

    /** D-pad DOWN: far range (~5m), straight ahead at hub. Hood far. */
    public static final ShootingPreset FAR = new ShootingPreset(
        0.0, 0.0, 4000.0, 2668.0, true
    );

    /** D-pad LEFT: back left corner of field, shooting to the right (~7m). Hood far. */
    public static final ShootingPreset ANGLE_LEFT = new ShootingPreset(
        45.0, 45.0, 5000.0, 3335.0, true
    );

    /** D-pad RIGHT: back right corner of field (~7m). Hood far. */
    public static final ShootingPreset ANGLE_RIGHT = new ShootingPreset(
        -45.0, -45.0, 5000.0, 3335.0, true
    );
  }

  // ──────────────────────────────────────────────────────────────────────────
  // TELEMETRY
  // ──────────────────────────────────────────────────────────────────────────

  public static final class TelemetryConstants {
    public static final boolean ENABLE_TELEMETRY  = false;
    public static final boolean TUNING_MODE       = false;
    public static final double  PUBLISH_HZ        = 10.0;
    public static final double  TUNING_PUBLISH_HZ = 20.0;
  }
}
