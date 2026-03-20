package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TurretConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Double turret rotation — 2x NEO 550 on SparkMax (CAN 34/35).
 * FIELD-CENTRIC: joystick forward = always downfield, regardless of robot heading.
 *
 * <p>Seeds at zero on each robot boot — MANUALLY ALIGN TURRETS TO FORWARD BEFORE ENABLING.
 * Soft limits ±13.25 motor rotations (±90°). Targets outside ±90° are clamped and
 * flagged unreachable — feeder is blocked to prevent wild shots.
 *
 * <p>FIELD-CENTRIC MATH:
 * fieldAngle = atan2(stickX, -stickY)  → 0° = downfield, +90° = field right
 * robotAngle = fieldAngle + robotHeading  (heading is CCW+ from WPILib)
 * Normalized to ±180°, clamped to ±90° turret limit.
 *
 * <p>When the joystick is released, the turret CONTINUES tracking its last field
 * direction as the robot rotates. This keeps the turret aimed at the target.
 */
public class TurretSubsystem extends SubsystemBase {

  // --- Hardware ---
  private final SparkMax m_leftMotor  = new SparkMax(TurretConstants.LEFT_MOTOR_ID,
      MotorType.kBrushless);
  private final SparkMax m_rightMotor = new SparkMax(TurretConstants.RIGHT_MOTOR_ID,
      MotorType.kBrushless);

  private final RelativeEncoder          m_leftEncoder  = m_leftMotor.getEncoder();
  private final RelativeEncoder          m_rightEncoder = m_rightMotor.getEncoder();
  private final SparkClosedLoopController m_leftCL      = m_leftMotor.getClosedLoopController();
  private final SparkClosedLoopController m_rightCL     = m_rightMotor.getClosedLoopController();

  // --- Field-centric ---
  private final DoubleSupplier m_headingSupplier;

  // Conversion: degrees → motor rotations
  private static final double DEG_TO_MOTOR_ROT = TurretConstants.GEAR_RATIO / 360.0;

  /** Max turret angle in degrees, derived from soft limit. */
  private static final double LIMIT_DEG =
      TurretConstants.FORWARD_LIMIT_ROTATIONS / DEG_TO_MOTOR_ROT;

  // Last commanded field angles (turret continues tracking when stick released)
  private double m_lastLeftFieldAngle  = 0.0;
  private double m_lastRightFieldAngle = 0.0;

  // Reachability — false if target angle exceeds ±90° turret limit
  private boolean m_leftReachable  = true;
  private boolean m_rightReachable = true;

  // SysId — 2 V/s ramp, 6V step (53:1 gearbox needs more voltage for usable data)
  private final SysIdRoutine m_turretSysId = new SysIdRoutine(
      new SysIdRoutine.Config(
          Volts.of(2).per(Second),
          Volts.of(6),
          Seconds.of(5)
      ),
      new SysIdRoutine.Mechanism(
          volts -> {
            double v = volts.in(Volts);
            m_leftMotor.setVoltage(v);
            m_rightMotor.setVoltage(v);
          },
          log -> {
            log.motor("turret-left")
                .voltage(Volts.of(m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage()))
                .angularPosition(Rotations.of(m_leftEncoder.getPosition()))
                .angularVelocity(RotationsPerSecond.of(m_leftEncoder.getVelocity() / 60.0));
            log.motor("turret-right")
                .voltage(Volts.of(m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage()))
                .angularPosition(Rotations.of(m_rightEncoder.getPosition()))
                .angularVelocity(RotationsPerSecond.of(m_rightEncoder.getVelocity() / 60.0));
          },
          this
      )
  );

  /**
   * @param headingSupplier robot heading in degrees (CCW-positive, 0° = downfield).
   *                        Typically: () -> drivebase.getPose().getRotation().getDegrees()
   */
  public TurretSubsystem(DoubleSupplier headingSupplier) {
    m_headingSupplier = headingSupplier;
    configureMotor(m_leftMotor, TurretConstants.LEFT_INVERTED,
        TurretConstants.LEFT_KP, TurretConstants.LEFT_KI, TurretConstants.LEFT_KD,
        TurretConstants.LEFT_OUTPUT_LIMIT);
    configureMotor(m_rightMotor, TurretConstants.RIGHT_INVERTED,
        TurretConstants.RIGHT_KP, TurretConstants.RIGHT_KI, TurretConstants.RIGHT_KD,
        TurretConstants.RIGHT_OUTPUT_LIMIT);
  }

  // ── Hardware Configuration ────────────────────────────────────────────────

  private void configureMotor(SparkMax motor, boolean inverted,
      double kP, double kI, double kD, double outputLimit) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.inverted(inverted)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(TurretConstants.CURRENT_LIMIT_AMPS);

    config.encoder
          .positionConversionFactor(1.0)
          .velocityConversionFactor(1.0);

    config.closedLoop
          .p(kP)
          .i(kI)
          .d(kD)
          .outputRange(-outputLimit, outputLimit);

    config.softLimit
          .forwardSoftLimit((float) TurretConstants.FORWARD_LIMIT_ROTATIONS)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimit((float) TurretConstants.REVERSE_LIMIT_ROTATIONS)
          .reverseSoftLimitEnabled(true);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ── Field-Centric Conversion ──────────────────────────────────────────────

  /**
   * Convert a field-relative angle to a robot-relative turret angle.
   * Accounts for current robot heading. Result normalized to ±180°.
   */
  private double fieldToRobotDeg(double fieldAngleDeg) {
    double heading = m_headingSupplier.getAsDouble();
    return MathUtil.inputModulus(fieldAngleDeg + heading, -180.0, 180.0);
  }

  /**
   * Clamp a robot-relative angle to the turret's reachable range and
   * return whether the original target was within limits.
   */
  private double clampToLimit(double robotAngleDeg) {
    return MathUtil.clamp(robotAngleDeg, -LIMIT_DEG, LIMIT_DEG);
  }

  // ── Internal Helpers ──────────────────────────────────────────────────────

  private double degreesToMotorRotations(double degrees) {
    return degrees * DEG_TO_MOTOR_ROT;
  }

  private void setLeftDeg(double robotDegrees) {
    m_leftCL.setReference(
        degreesToMotorRotations(robotDegrees), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  private void setRightDeg(double robotDegrees) {
    m_rightCL.setReference(
        degreesToMotorRotations(robotDegrees), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /**
   * Command a turret to a field-relative angle. Converts to robot-relative,
   * clamps to limits, and updates reachability flag.
   */
  private void commandLeftFieldAngle(double fieldDeg) {
    m_lastLeftFieldAngle = fieldDeg;
    double robotDeg = fieldToRobotDeg(fieldDeg);
    m_leftReachable = Math.abs(robotDeg) <= LIMIT_DEG;
    setLeftDeg(clampToLimit(robotDeg));
  }

  private void commandRightFieldAngle(double fieldDeg) {
    m_lastRightFieldAngle = fieldDeg;
    double robotDeg = fieldToRobotDeg(fieldDeg);
    m_rightReachable = Math.abs(robotDeg) <= LIMIT_DEG;
    setRightDeg(clampToLimit(robotDeg));
  }

  // ── Status ────────────────────────────────────────────────────────────────

  public double getLeftDeg() {
    return m_leftEncoder.getPosition() / DEG_TO_MOTOR_ROT;
  }

  public double getRightDeg() {
    return m_rightEncoder.getPosition() / DEG_TO_MOTOR_ROT;
  }

  /**
   * True when BOTH turrets can reach their target field angles.
   * Use this to gate the feeder — don't fire if turrets can't aim properly.
   */
  public boolean isTargetReachable() {
    return m_leftReachable && m_rightReachable;
  }

  // ── Command Factories ─────────────────────────────────────────────────────

  /**
   * Field-centric joystick control. Push stick toward a field direction and the
   * turret tracks that direction regardless of robot heading.
   *
   * <p>When the stick is released, the turret CONTINUES tracking the last commanded
   * field direction (compensates as robot rotates).
   *
   * @param leftX/leftY   left stick axes (WPILib: forward Y = negative)
   * @param rightX/rightY right stick axes
   * @param locked        when true, left stick controls both turrets
   */
  public Command joystickControlCommand(
      DoubleSupplier leftX, DoubleSupplier leftY,
      DoubleSupplier rightX, DoubleSupplier rightY,
      BooleanSupplier locked) {
    return this.run(() -> {
      double lx = leftX.getAsDouble();
      double ly = leftY.getAsDouble();
      double rx = rightX.getAsDouble();
      double ry = rightY.getAsDouble();
      boolean isLocked = locked.getAsBoolean();

      double leftMag  = Math.min(1.0, Math.hypot(lx, ly));
      double rightMag = Math.min(1.0, Math.hypot(rx, ry));

      // Left stick → update field target if above deadband, else hold last field target
      if (leftMag > TurretConstants.JOYSTICK_DEADBAND) {
        m_lastLeftFieldAngle = Math.toDegrees(Math.atan2(lx, -ly));
      }
      commandLeftFieldAngle(m_lastLeftFieldAngle);

      if (isLocked) {
        // Both turrets track left stick's field angle
        m_lastRightFieldAngle = m_lastLeftFieldAngle;
        commandRightFieldAngle(m_lastRightFieldAngle);
      } else {
        // Right stick → update field target if above deadband
        if (rightMag > TurretConstants.JOYSTICK_DEADBAND) {
          m_lastRightFieldAngle = Math.toDegrees(Math.atan2(rx, -ry));
        }
        commandRightFieldAngle(m_lastRightFieldAngle);
      }
    }).withName("TurretFieldCentric");
  }

  /**
   * Hold both turrets at fixed FIELD-RELATIVE angles.
   * Continuously compensates for robot rotation.
   * Used by D-pad presets. Angles: 0° = downfield, +45° = field-right.
   */
  public Command holdPositionCommand(double leftFieldDeg, double rightFieldDeg) {
    return this.run(() -> {
      commandLeftFieldAngle(leftFieldDeg);
      commandRightFieldAngle(rightFieldDeg);
    }).withName("TurretHoldFieldPosition");
  }

  /** Reset both encoder positions to zero. Align turrets to forward first! */
  public Command zeroEncodersCommand() {
    return this.runOnce(() -> {
      m_leftEncoder.setPosition(0.0);
      m_rightEncoder.setPosition(0.0);
      m_lastLeftFieldAngle  = 0.0;
      m_lastRightFieldAngle = 0.0;
    }).withName("TurretZeroEncoders");
  }

  // ── SysId Commands ──────────────────────────────────────────────────────

  public Command sysIdTurretCommand() {
    return Commands.sequence(
        m_turretSysId.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5.0),
        Commands.waitSeconds(1.0),
        m_turretSysId.dynamic(SysIdRoutine.Direction.kForward).withTimeout(4.0),
        Commands.waitSeconds(1.0),
        m_turretSysId.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5.0),
        Commands.waitSeconds(1.0),
        m_turretSysId.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(4.0)
    ).withName("SysIdTurret");
  }

  // ── Periodic ─────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/LeftDeg",       getLeftDeg());
    SmartDashboard.putNumber("Turret/RightDeg",      getRightDeg());
    SmartDashboard.putBoolean("Turret/Reachable",    isTargetReachable());
    SmartDashboard.putNumber("Turret/LeftFieldTarget",  m_lastLeftFieldAngle);
    SmartDashboard.putNumber("Turret/RightFieldTarget", m_lastRightFieldAngle);
  }
}
