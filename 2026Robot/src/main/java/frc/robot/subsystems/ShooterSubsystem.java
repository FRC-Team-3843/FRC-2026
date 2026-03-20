package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

/**
 * Double shooter — 2x Kraken X44 preshooters (CAN 36/37) +
 * 2x Kraken X60 main shooters (CAN 38/39) + 2x hood servos (PWM 0/1).
 *
 * <p>Velocity is specified as WHEEL RPM and converted to motor RPS internally.
 * Both sides always fire together.
 *
 * <p>INVERSION: Set LEFT/RIGHT_MAIN_INVERTED and LEFT/RIGHT_PRE_INVERTED in
 * ShooterConstants after verifying direction on hardware. Mirrored mounting
 * means one side of each pair will need inversion.
 *
 * <p>PID TUNING: Start with kP=0.1, kV=0.12. Increase kP if slow to reach
 * target. If oscillation occurs, reduce kP. Add kD if overshoot is severe.
 */
public class ShooterSubsystem extends SubsystemBase {

  // --- Main shooters (Kraken X60) ---
  private final TalonFX m_leftMain  = new TalonFX(ShooterConstants.LEFT_MAIN_SHOOTER_ID);
  private final TalonFX m_rightMain = new TalonFX(ShooterConstants.RIGHT_MAIN_SHOOTER_ID);

  // --- Preshooters (Kraken X44) ---
  private final TalonFX m_leftPre   = new TalonFX(ShooterConstants.LEFT_PRESHOOTER_ID);
  private final TalonFX m_rightPre  = new TalonFX(ShooterConstants.RIGHT_PRESHOOTER_ID);

  // --- Hood servos ---
  private final Servo m_leftHood  = new Servo(HoodConstants.LEFT_PWM_CHANNEL);
  private final Servo m_rightHood = new Servo(HoodConstants.RIGHT_PWM_CHANNEL);

  // Separate control request per motor — Phoenix6 holds references, shared objects cause glitches
  private final VelocityVoltage m_leftMainReq  = new VelocityVoltage(0).withEnableFOC(false);
  private final VelocityVoltage m_rightMainReq = new VelocityVoltage(0).withEnableFOC(false);
  private final VelocityVoltage m_leftPreReq   = new VelocityVoltage(0).withEnableFOC(false);
  private final VelocityVoltage m_rightPreReq  = new VelocityVoltage(0).withEnableFOC(false);

  // SysId voltage requests (separate from velocity control)
  private final VoltageOut m_sysIdLeftMain  = new VoltageOut(0);
  private final VoltageOut m_sysIdRightMain = new VoltageOut(0);
  private final VoltageOut m_sysIdLeftPre   = new VoltageOut(0);
  private final VoltageOut m_sysIdRightPre  = new VoltageOut(0);

  // SysId routines for characterizing main shooters and preshooters separately
  private final SysIdRoutine m_mainShooterSysId = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          volts -> {
            double v = volts.in(Volts);
            m_leftMain.setControl(m_sysIdLeftMain.withOutput(v));
            m_rightMain.setControl(m_sysIdRightMain.withOutput(v));
          },
          log -> {
            log.motor("main-shooter-left")
                .voltage(Volts.of(m_leftMain.getMotorVoltage().getValueAsDouble()))
                .angularPosition(Rotations.of(m_leftMain.getPosition().getValueAsDouble()))
                .angularVelocity(RotationsPerSecond.of(m_leftMain.getVelocity().getValueAsDouble()));
            log.motor("main-shooter-right")
                .voltage(Volts.of(m_rightMain.getMotorVoltage().getValueAsDouble()))
                .angularPosition(Rotations.of(m_rightMain.getPosition().getValueAsDouble()))
                .angularVelocity(RotationsPerSecond.of(m_rightMain.getVelocity().getValueAsDouble()));
          },
          this
      )
  );

  private final SysIdRoutine m_preshooterSysId = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          volts -> {
            double v = volts.in(Volts);
            m_leftPre.setControl(m_sysIdLeftPre.withOutput(v));
            m_rightPre.setControl(m_sysIdRightPre.withOutput(v));
          },
          log -> {
            log.motor("preshooter-left")
                .voltage(Volts.of(m_leftPre.getMotorVoltage().getValueAsDouble()))
                .angularPosition(Rotations.of(m_leftPre.getPosition().getValueAsDouble()))
                .angularVelocity(RotationsPerSecond.of(m_leftPre.getVelocity().getValueAsDouble()));
            log.motor("preshooter-right")
                .voltage(Volts.of(m_rightPre.getMotorVoltage().getValueAsDouble()))
                .angularPosition(Rotations.of(m_rightPre.getPosition().getValueAsDouble()))
                .angularVelocity(RotationsPerSecond.of(m_rightPre.getVelocity().getValueAsDouble()));
          },
          this
      )
  );

  private double m_targetMainRpm  = 0.0;
  private double m_targetPreRpm   = 0.0;

  public ShooterSubsystem() {
    configureMotor(m_leftMain,  ShooterConstants.LEFT_MAIN_INVERTED,
        ShooterConstants.MAIN_SHOOTER_CURRENT_LIMIT_AMPS);
    configureMotor(m_rightMain, ShooterConstants.RIGHT_MAIN_INVERTED,
        ShooterConstants.MAIN_SHOOTER_CURRENT_LIMIT_AMPS);
    configureMotor(m_leftPre,   ShooterConstants.LEFT_PRE_INVERTED,
        ShooterConstants.PRESHOOTER_CURRENT_LIMIT_AMPS);
    configureMotor(m_rightPre,  ShooterConstants.RIGHT_PRE_INVERTED,
        ShooterConstants.PRESHOOTER_CURRENT_LIMIT_AMPS);

    // Start hood in near position
    setHoodFar(false);
  }

  // ── Hardware Configuration ────────────────────────────────────────────────

  private void configureMotor(TalonFX motor, boolean inverted, double currentLimit) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted    = inverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.StatorCurrentLimit       = currentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = ShooterConstants.KP;
    slot0.kI = ShooterConstants.KI;
    slot0.kD = ShooterConstants.KD;
    slot0.kV = ShooterConstants.KV;

    motor.getConfigurator().apply(config);
  }

  // ── Internal Helpers ──────────────────────────────────────────────────────

  /**
   * Convert wheel RPM to motor RPS for TalonFX setpoint.
   * gear_ratio < 1 means output is faster than motor.
   * motor_rps = (wheel_rpm / 60) * gear_ratio
   */
  private double wheelRpmToMotorRps(double wheelRpm, double gearRatio) {
    return (wheelRpm / 60.0) * gearRatio;
  }

  /** Apply velocity setpoints to all four motors (same RPM both sides). */
  private void applyRpm(double mainRpm, double preRpm) {
    applyLeftRpm(mainRpm, preRpm);
    applyRightRpm(mainRpm, preRpm);
    m_targetMainRpm = mainRpm;
    m_targetPreRpm  = preRpm;
  }

  /** Apply velocity setpoints to left side only. */
  private void applyLeftRpm(double mainRpm, double preRpm) {
    double mainRps = wheelRpmToMotorRps(mainRpm, ShooterConstants.MAIN_SHOOTER_GEAR_RATIO);
    double preRps  = wheelRpmToMotorRps(preRpm,  ShooterConstants.PRESHOOTER_GEAR_RATIO);
    m_leftMain.setControl(m_leftMainReq.withVelocity(mainRps));
    m_leftPre.setControl(m_leftPreReq.withVelocity(preRps));
  }

  /** Apply velocity setpoints to right side only. */
  private void applyRightRpm(double mainRpm, double preRpm) {
    double mainRps = wheelRpmToMotorRps(mainRpm, ShooterConstants.MAIN_SHOOTER_GEAR_RATIO);
    double preRps  = wheelRpmToMotorRps(preRpm,  ShooterConstants.PRESHOOTER_GEAR_RATIO);
    m_rightMain.setControl(m_rightMainReq.withVelocity(mainRps));
    m_rightPre.setControl(m_rightPreReq.withVelocity(preRps));
  }

  /** Stop all shooter motors. */
  private void stopAll() {
    m_leftMain.set(0.0);
    m_rightMain.set(0.0);
    m_leftPre.set(0.0);
    m_rightPre.set(0.0);
    m_targetMainRpm = 0.0;
    m_targetPreRpm  = 0.0;
  }

  /** Set both hood servos to near or far position. */
  private void setHoodFar(boolean far) {
    double angle = far ? HoodConstants.FAR_ANGLE : HoodConstants.NEAR_ANGLE;
    m_leftHood.setAngle(angle);
    m_rightHood.setAngle(angle);
  }

  // ── Status ────────────────────────────────────────────────────────────────

  /**
   * Returns true when BOTH main shooters are within tolerance of their target.
   * Use this to gate feeder activation.
   */
  public boolean atSpeed() {
    if (m_targetMainRpm < 100.0) return false; // not spinning up
    double leftRpm  = m_leftMain.getVelocity().getValueAsDouble()  * 60.0
        / ShooterConstants.MAIN_SHOOTER_GEAR_RATIO;
    double rightRpm = m_rightMain.getVelocity().getValueAsDouble() * 60.0
        / ShooterConstants.MAIN_SHOOTER_GEAR_RATIO;
    return Math.abs(leftRpm  - m_targetMainRpm) < ShooterConstants.AT_SPEED_TOLERANCE_RPM
        && Math.abs(rightRpm - m_targetMainRpm) < ShooterConstants.AT_SPEED_TOLERANCE_RPM;
  }

  // ── Command Factories ─────────────────────────────────────────────────────

  /**
   * Manual speed command — left stick magnitude drives left shooter,
   * right stick magnitude drives right shooter independently.
   * Hood auto-switches near/far based on max of both magnitudes.
   * Call this as the ShooterSubsystem default command in RobotContainer.
   */
  public Command spinFromDualMagnitudeCommand(
      DoubleSupplier leftMagnitude, DoubleSupplier rightMagnitude) {
    return this.run(() -> {
      double leftMag  = leftMagnitude.getAsDouble();
      double rightMag = rightMagnitude.getAsDouble();
      double deadband = Constants.TurretConstants.JOYSTICK_DEADBAND;

      // Left side
      if (leftMag > deadband) {
        double mainRpm = leftMag * ShooterConstants.MAX_WHEEL_RPM;
        double preRpm  = mainRpm * ShooterConstants.PRESHOOTER_SPEED_RATIO;
        applyLeftRpm(mainRpm, preRpm);
      } else {
        m_leftMain.set(0.0);
        m_leftPre.set(0.0);
      }

      // Right side
      if (rightMag > deadband) {
        double mainRpm = rightMag * ShooterConstants.MAX_WHEEL_RPM;
        double preRpm  = mainRpm * ShooterConstants.PRESHOOTER_SPEED_RATIO;
        applyRightRpm(mainRpm, preRpm);
      } else {
        m_rightMain.set(0.0);
        m_rightPre.set(0.0);
      }

      // Track target for atSpeed() — use max of both sides
      double maxMag = Math.max(leftMag, rightMag);
      m_targetMainRpm = maxMag > deadband ? maxMag * ShooterConstants.MAX_WHEEL_RPM : 0.0;
      m_targetPreRpm  = m_targetMainRpm * ShooterConstants.PRESHOOTER_SPEED_RATIO;

      // Hood auto-position based on max magnitude
      setHoodFar(maxMag >= HoodConstants.AUTO_HOOD_THRESHOLD);
    }).withName("ShooterDualManual")
      .finallyDo(interrupted -> stopAll());
  }

  /**
   * Hold a fixed RPM — used for D-pad preset positions.
   * Runs continuously until cancelled (Y button or new D-pad press).
   */
  public Command holdRpmCommand(double mainRpm, double preRpm, boolean hoodFar) {
    return this.run(() -> {
      applyRpm(mainRpm, preRpm);
      setHoodFar(hoodFar);
    }).withName("ShooterHoldRpm")
      .finallyDo(interrupted -> stopAll());
  }

  /**
   * Set hood to near position (for close shots).
   * Uses Commands.runOnce() (no subsystem requirement) so it does NOT
   * interrupt the active shooter velocity command.
   */
  public Command hoodNearCommand() {
    return Commands.runOnce(() -> setHoodFar(false)).withName("HoodNear");
  }

  /**
   * Set hood to far position (for distance shots).
   * Uses Commands.runOnce() (no subsystem requirement) so it does NOT
   * interrupt the active shooter velocity command.
   */
  public Command hoodFarCommand() {
    return Commands.runOnce(() -> setHoodFar(true)).withName("HoodFar");
  }

  /** Immediately stop all shooter motors. */
  public Command stopCommand() {
    return this.runOnce(this::stopAll).withName("ShooterStop");
  }

  // ── SysId Commands ──────────────────────────────────────────────────────

  /** Full SysId characterization for main shooter motors. ~20s total. */
  public Command sysIdMainShooterCommand() {
    return Commands.sequence(
        m_mainShooterSysId.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5.0),
        Commands.waitSeconds(1.0),
        m_mainShooterSysId.dynamic(SysIdRoutine.Direction.kForward).withTimeout(3.0),
        Commands.waitSeconds(1.0),
        m_mainShooterSysId.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5.0),
        Commands.waitSeconds(1.0),
        m_mainShooterSysId.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(3.0)
    ).withName("SysIdMainShooter");
  }

  /** Full SysId characterization for preshooter motors. ~20s total. */
  public Command sysIdPreshooterCommand() {
    return Commands.sequence(
        m_preshooterSysId.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5.0),
        Commands.waitSeconds(1.0),
        m_preshooterSysId.dynamic(SysIdRoutine.Direction.kForward).withTimeout(3.0),
        Commands.waitSeconds(1.0),
        m_preshooterSysId.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5.0),
        Commands.waitSeconds(1.0),
        m_preshooterSysId.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(3.0)
    ).withName("SysIdPreshooter");
  }

  // ── Periodic ─────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/TargetMainRPM",  m_targetMainRpm);
    SmartDashboard.putBoolean("Shooter/AtSpeed",       atSpeed());
    SmartDashboard.putNumber("Shooter/LeftMainRPM",
        m_leftMain.getVelocity().getValueAsDouble() * 60.0
            / ShooterConstants.MAIN_SHOOTER_GEAR_RATIO);
    SmartDashboard.putNumber("Shooter/RightMainRPM",
        m_rightMain.getVelocity().getValueAsDouble() * 60.0
            / ShooterConstants.MAIN_SHOOTER_GEAR_RATIO);
  }
}
