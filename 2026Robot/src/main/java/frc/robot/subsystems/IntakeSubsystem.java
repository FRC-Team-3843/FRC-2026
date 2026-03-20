package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Ground intake — Kraken X44 (TalonFX, CAN 30).
 * Flops out passively; no driven rotation mechanism.
 * Call runCommand() while held to intake, reverseCommand() to unjam.
 *
 * INVERSION: Set IntakeConstants.MOTOR_INVERTED after verifying direction on hardware.
 */
public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX m_motor = new TalonFX(IntakeConstants.MOTOR_ID);
  private final DutyCycleOut m_output = new DutyCycleOut(0.0).withEnableFOC(false);

  public IntakeSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = IntakeConstants.MOTOR_INVERTED
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.StatorCurrentLimit       = IntakeConstants.CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    m_motor.getConfigurator().apply(config);
  }

  /** Run intake forward (ingesting fuel). Stops motor on command end. */
  public Command runCommand() {
    return this.runEnd(
        () -> m_motor.setControl(m_output.withOutput(IntakeConstants.INTAKE_SPEED)),
        () -> m_motor.setControl(m_output.withOutput(0.0))
    ).withName("IntakeRun");
  }

  /** Run intake in reverse (unjam / eject). Stops motor on command end. */
  public Command reverseCommand() {
    return this.runEnd(
        () -> m_motor.setControl(m_output.withOutput(-IntakeConstants.INTAKE_SPEED)),
        () -> m_motor.setControl(m_output.withOutput(0.0))
    ).withName("IntakeReverse");
  }

  /** Immediately stop the intake motor. */
  public Command stopCommand() {
    return this.runOnce(() -> m_motor.setControl(m_output.withOutput(0.0)))
        .withName("IntakeStop");
  }
}
