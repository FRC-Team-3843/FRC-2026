package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

/**
 * Feeder — NEO 550 on SparkMax (CAN 33).
 * Feeds fuel from the hopper into both shooters simultaneously.
 *
 * DISABLED: Encoder cable broken. Set FeederConstants.ENABLED = true once repaired.
 * When disabled, no SparkMax is constructed (avoids CAN fault spam from broken encoder).
 * All commands become no-ops so the rest of the robot works normally.
 */
public class FeederSubsystem extends SubsystemBase {

  private SparkMax m_motor;

  public FeederSubsystem() {
    if (FeederConstants.ENABLED) {
      m_motor = new SparkMax(FeederConstants.MOTOR_ID, MotorType.kBrushless);
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(FeederConstants.MOTOR_INVERTED)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(FeederConstants.CURRENT_LIMIT_AMPS);
      m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  /** Run feeder (fire). No-op when disabled. */
  public Command runCommand() {
    if (!FeederConstants.ENABLED) {
      return this.run(() -> {}).withName("FeederDisabled");
    }
    return this.runEnd(
        () -> m_motor.set(FeederConstants.FEEDER_SPEED),
        () -> m_motor.set(0.0)
    ).withName("FeederRun");
  }

  /** Run feeder in reverse (unjam). No-op when disabled. */
  public Command reverseCommand() {
    if (!FeederConstants.ENABLED) {
      return this.run(() -> {}).withName("FeederDisabled");
    }
    return this.runEnd(
        () -> m_motor.set(-FeederConstants.FEEDER_SPEED),
        () -> m_motor.set(0.0)
    ).withName("FeederReverse");
  }

  public Command stopCommand() {
    if (!FeederConstants.ENABLED) {
      return this.runOnce(() -> {}).withName("FeederDisabled");
    }
    return this.runOnce(() -> m_motor.set(0.0)).withName("FeederStop");
  }
}
