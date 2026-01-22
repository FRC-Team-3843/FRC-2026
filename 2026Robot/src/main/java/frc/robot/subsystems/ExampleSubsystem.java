package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

/**
 * TEMPLATE SUBSYSTEM: Use this as a reference for creating new subsystems (Intake, Shooter, etc.).
 *
 * <p><b>2026 Standards:</b>
 * <ul>
 *   <li><b>Dependency Injection:</b> Pass hardware via constructor (or create internally if simple).</li>
 *   <li><b>Command Factories:</b> Define methods that return `Command` objects for actions.</li>
 *   <li><b>No Public Hardware:</b> Keep motors and sensors private. Expose only Commands or boolean state.</li>
 * </ul>
 */
public class ExampleSubsystem extends SubsystemBase {

  // Hardware (Private)
  // private final SparkMax motor;
  // private final DigitalInput limitSwitch;

  /**
   * Create a new ExampleSubsystem.
   *
   * @param canId The CAN ID for the motor (passed from Constants).
   */
  public ExampleSubsystem(int canId) {
    // motor = new SparkMax(canId, MotorType.kBrushless);
    // configureMotor();
  }

  // --- Command Factories (The 2026 Way) ---

  /**
   * Example: Run the motor at a set speed until interrupted.
   *
   * <p>Usage in RobotContainer:
   * {@code operatorController.a().whileTrue(subsystem.runCommand(0.5));}
   */
  public Command runCommand(double speed) {
    // runEnd runs the first runnable repeatedly, then the second runnable once when interrupted
    return this.runEnd(
        () -> System.out.println("Running motor at " + speed), // motor.set(speed)
        () -> System.out.println("Stopping motor")             // motor.set(0)
    );
  }

  /**
   * Example: Run using a supplier (e.g., from a joystick axis).
   *
   * <p>Usage in RobotContainer:
   * {@code subsystem.setDefaultCommand(subsystem.runJoysticks(operator::getLeftY));}
   */
  public Command runJoysticks(DoubleSupplier speedSupplier) {
    return this.run(() -> {
      double speed = speedSupplier.getAsDouble();
      // motor.set(speed);
    });
  }

  /**
   * Example: A complex sequence (e.g., "Score Game Piece").
   *
   * <p>Combines multiple actions into one atomic command factory.
   */
  public Command scoreSequenceCommand() {
    return this.runOnce(() -> System.out.println("Prepare to score"))
        .andThen(runCommand(1.0).withTimeout(0.5)) // Run for 0.5s
        .andThen(this.runOnce(() -> System.out.println("Score complete")));
  }

  // --- Configuration (Private) ---

  private void configureMotor() {
    // SparkMaxConfig config = new SparkMaxConfig();
    // config.idleMode(IdleMode.kBrake);
    // motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // Updates called every 20ms (e.g., updating dashboard values)
  }
}
