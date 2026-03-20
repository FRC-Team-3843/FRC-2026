package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShootingPositions.ShootingPreset;

/**
 * Superstructure — coordinates multi-subsystem actions.
 *
 * <p>Not a SubsystemBase. Provides command factories that combine
 * intake, conveyor, hopper, feeder, shooter, and turret.
 *
 * <p>Usage in RobotContainer:
 * <ul>
 *   <li>{@link #intakeCommand()} — bind to driver right bumper (whileTrue)</li>
 *   <li>{@link #reverseIntakeCommand()} — bind to driver left bumper (whileTrue)</li>
 *   <li>{@link #setPositionCommand(ShootingPreset)} — bind to operator D-pad (onTrue)</li>
 *   <li>{@link #clearPositionCommand()} — bind to operator Y button (onTrue)</li>
 * </ul>
 */
public class Superstructure {

  private final IntakeSubsystem   m_intake;
  private final ConveyorSubsystem m_conveyor;
  private final HopperSubsystem   m_hopper;
  private final FeederSubsystem   m_feeder;
  private final ShooterSubsystem  m_shooter;
  private final TurretSubsystem   m_turret;

  public Superstructure(
      IntakeSubsystem intake,
      ConveyorSubsystem conveyor,
      HopperSubsystem hopper,
      FeederSubsystem feeder,
      ShooterSubsystem shooter,
      TurretSubsystem turret) {
    m_intake   = intake;
    m_conveyor = conveyor;
    m_hopper   = hopper;
    m_feeder   = feeder;
    m_shooter  = shooter;
    m_turret   = turret;
  }

  // ── Intake Sequences ─────────────────────────────────────────────────────

  /**
   * Run the full intake chain: intake roller + floor conveyor + hopper.
   * Bind to driver right bumper (whileTrue).
   */
  public Command intakeCommand() {
    return Commands.parallel(
        m_intake.runCommand(),
        m_conveyor.runCommand(),
        m_hopper.runCommand()
    ).withName("IntakeAll");
  }

  /**
   * Reverse the full intake chain to eject or unjam.
   * Bind to driver left bumper (whileTrue).
   */
  public Command reverseIntakeCommand() {
    return Commands.parallel(
        m_intake.reverseCommand(),
        m_conveyor.reverseCommand(),
        m_hopper.reverseCommand()
    ).withName("ReverseAll");
  }

  // ── Preset Shooting Positions ─────────────────────────────────────────────

  /**
   * Activate a D-pad shooting preset.
   * Simultaneously holds turrets at preset angles AND spins shooters at preset RPM.
   * Requires both TurretSubsystem and ShooterSubsystem, so it overrides their
   * default commands until cancelled.
   *
   * <p>Cancel with {@link #clearPositionCommand()} (Y button).
   *
   * @param preset one of ShootingPositions.CLOSE / FAR / ANGLE_LEFT / ANGLE_RIGHT
   */
  public Command setPositionCommand(ShootingPreset preset) {
    return Commands.parallel(
        m_turret.holdPositionCommand(preset.leftTurretDeg(), preset.rightTurretDeg()),
        m_shooter.holdRpmCommand(preset.mainRpm(), preset.preshooterRpm(), preset.hoodFar())
    ).withName("ShootPosition");
  }

  /**
   * Cancel any active position preset and return turret + shooter to their default commands.
   * Default: turret follows joystick, shooter follows joystick magnitude.
   * Bind to operator Y button (onTrue).
   */
  public Command clearPositionCommand() {
    // Scheduling a runOnce that requires turret and shooter causes the position command
    // (which also requires them) to be interrupted, falling back to defaults.
    return Commands.runOnce(() -> {}, m_turret, m_shooter)
        .withName("ClearPosition");
  }
}
