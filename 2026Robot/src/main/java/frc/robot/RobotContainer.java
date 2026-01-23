package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * Subsystem instantiation, controller bindings, and autonomous selection.
 */
public class RobotContainer {

  private final CommandXboxController driverXbox =
      new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorXbox =
      new CommandXboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private SendableChooser<Command> autoChooser;

  /**
   * Primary drive stream: field-relative with angular velocity rotation control.
   * Left stick = translation, right stick X = rotation rate.
   */
  private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
          drivebase.getSwerveDrive(),
          () -> -driverXbox.getLeftY(),
          () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(Constants.OperatorConstants.DEADBAND)
      .scaleTranslation(Constants.DriveProfiles.FULL_SPEED_SCALE)
      .allianceRelativeControl(true);

  /**
   * Direct angle drive: right stick controls heading direction instead of rotation rate.
   * Better for simulation testing with keyboard.
   */
  private final SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Robot-oriented drive stream (not field-relative).
   */
  private final SwerveInputStream driveRobotOriented = driveAngularVelocity.copy()
      .robotRelative(true)
      .allianceRelativeControl(false);

  /**
   * Slow mode drive stream: reduced speed for precision maneuvers.
   * Activated by holding left trigger.
   */
  private final SwerveInputStream driveAngularVelocitySlow = SwerveInputStream.of(
          drivebase.getSwerveDrive(),
          () -> -driverXbox.getLeftY(),
          () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(Constants.OperatorConstants.DEADBAND)
      .scaleTranslation(Constants.DriveProfiles.SLOW_SPEED_SCALE)
      .allianceRelativeControl(true);

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Build the auto chooser from PathPlanner paths
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // --- Default drive command ---
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedSetpoint =
        drivebase.driveWithSetpointGeneratorFieldRelative(driveAngularVelocity::get);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    } else {
      if (Constants.AssistConstants.DEFAULT_DRIVE_MODE
          == Constants.AssistConstants.DriveControlMode.SETPOINT_GENERATOR) {
        drivebase.setDefaultCommand(driveFieldOrientedSetpoint);
      } else {
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
      }
    }

    // --- Driver controls ---

    // Start: zero gyro (alliance-aware - faces 0 on blue, 180 on red)
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

    // POV Up: lock wheels in X pattern to resist defense
    driverXbox.povUp().whileTrue(drivebase.lockCommand());

    // Back: center all modules to 0 degrees (alignment check)
    driverXbox.back().whileTrue(drivebase.centerModulesCommand());

    // Left trigger held: slow mode for precision alignment
    driverXbox.leftTrigger(0.5).whileTrue(drivebase.driveFieldOriented(driveAngularVelocitySlow));

    // --- Driver assist modes (placeholder bindings, OFF by default) ---
    Command assistCommand = getDriverAssistCommand();
    driverXbox.rightBumper().whileTrue(assistCommand);

    // --- Test mode bindings (SysId characterization) ---
    if (DriverStation.isTest()) {
      driverXbox.x().whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.y().whileTrue(drivebase.sysIdAngleMotorCommand());
    }

    // --- Operator controls ---
    // Add mechanism bindings here as subsystems are added
  }

  private Command getDriverAssistCommand() {
    switch (Constants.AssistConstants.DEFAULT_ASSIST_MODE) {
      case AUTO_ALIGN:
      case PATH_TO_POSE:
      case SNAP_HEADING:
      case MICRO_ADJUST_RATE:
      case MICRO_ADJUST_NUDGE:
        // TODO: implement actual assist commands when mechanisms/targets are defined.
        return Commands.none();
      case OFF:
      default:
        return Commands.none();
    }
  }

  /**
   * Get the selected autonomous command from SmartDashboard chooser.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Set motor brake/coast mode on the drivebase.
   */
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
