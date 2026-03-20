package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

/**
 * Hopper — Bag motor on TalonSRX (CAN 32), Phoenix5 API.
 * Holds and agitates fuel above the floor conveyor, feeding into the feeder.
 *
 * INVERSION: Set HopperConstants.MOTOR_INVERTED after verifying direction on hardware.
 */
public class HopperSubsystem extends SubsystemBase {

  private final TalonSRX m_motor = new TalonSRX(HopperConstants.MOTOR_ID);

  public HopperSubsystem() {
    m_motor.configFactoryDefault();
    m_motor.setInverted(HopperConstants.MOTOR_INVERTED);
    m_motor.setNeutralMode(NeutralMode.Coast);

    // Current limiting disabled — breaker is the only protection
    m_motor.enableCurrentLimit(false);
  }

  /** Set motor speed directly. Used by auto-shoot command. */
  public void setSpeed(double speed) {
    m_motor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  /** Run hopper forward (agitate toward feeder). Stops on command end. */
  public Command runCommand() {
    return this.runEnd(
        () -> m_motor.set(TalonSRXControlMode.PercentOutput, HopperConstants.HOPPER_SPEED),
        () -> m_motor.set(TalonSRXControlMode.PercentOutput, 0.0)
    ).withName("HopperRun");
  }

  /** Run hopper in reverse (unjam). Stops on command end. */
  public Command reverseCommand() {
    return this.runEnd(
        () -> m_motor.set(TalonSRXControlMode.PercentOutput, -HopperConstants.HOPPER_SPEED),
        () -> m_motor.set(TalonSRXControlMode.PercentOutput, 0.0)
    ).withName("HopperReverse");
  }

  public Command stopCommand() {
    return this.runOnce(() -> m_motor.set(TalonSRXControlMode.PercentOutput, 0.0))
        .withName("HopperStop");
  }
}
