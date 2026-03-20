package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

/**
 * Floor conveyor — NEO 550 on SparkMax (CAN 31).
 * Transports fuel from intake up to the hopper.
 *
 * INVERSION: Set ConveyorConstants.MOTOR_INVERTED after verifying direction on hardware.
 */
public class ConveyorSubsystem extends SubsystemBase {

  private final SparkMax m_motor = new SparkMax(ConveyorConstants.MOTOR_ID, MotorType.kBrushless);

  public ConveyorSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(ConveyorConstants.MOTOR_INVERTED)
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(ConveyorConstants.CURRENT_LIMIT_AMPS);
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Run conveyor forward (toward hopper). Stops on command end. */
  public Command runCommand() {
    return this.runEnd(
        () -> m_motor.set(ConveyorConstants.CONVEYOR_SPEED),
        () -> m_motor.set(0.0)
    ).withName("ConveyorRun");
  }

  /** Run conveyor in reverse (eject). Stops on command end. */
  public Command reverseCommand() {
    return this.runEnd(
        () -> m_motor.set(-ConveyorConstants.CONVEYOR_SPEED),
        () -> m_motor.set(0.0)
    ).withName("ConveyorReverse");
  }

  public Command stopCommand() {
    return this.runOnce(() -> m_motor.set(0.0)).withName("ConveyorStop");
  }
}
