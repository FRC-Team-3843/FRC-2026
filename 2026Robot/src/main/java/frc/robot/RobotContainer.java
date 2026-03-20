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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShootingPositions;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.DashboardManager;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

/**
 * Subsystem instantiation, controller bindings, and autonomous selection.
 *
 * <p>DRIVER (port 0):
 *   Left stick        — translate (field-relative)
 *   Right stick X     — rotate
 *   Left trigger held — slow mode
 *   Right bumper held — intake (intake + conveyor + hopper)
 *   Left bumper held  — reverse intake (eject/unjam)
 *   Start             — zero gyro (alliance-aware)
 *   POV Up            — lock wheels (X-pattern defense)
 *   Back              — center all modules to 0°
 *   X (test mode)     — SysId drive motors
 *   Y (test mode)     — SysId angle motors
 *
 * <p>OPERATOR (port 1):
 *   Left stick        — aim left turret (manual mode); locks both when turret locked
 *   Right stick       — aim right turret (manual mode)
 *   Left stick button — toggle turret lock (left stick controls both)
 *   D-pad Up          — preset: CLOSE (low RPM, hood near)
 *   D-pad Down        — preset: FAR (high RPM, hood far)
 *   D-pad Left        — preset: ANGLE_LEFT
 *   D-pad Right       — preset: ANGLE_RIGHT
 *   Y                 — clear preset, return to manual control
 *   A (held)          — fire (run feeder)
 *   B                 — toggle constant fire
 *   Left bumper       — hood near (manual override)
 *   Right bumper      — hood far (manual override)
 *   Start             — zero turret encoders (align turrets to forward first!)
 */
public class RobotContainer {

  // ── Controllers ───────────────────────────────────────────────────────────

  private final CommandXboxController driverXbox =
      new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorXbox =
      new CommandXboxController(Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // ── Subsystems ────────────────────────────────────────────────────────────

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final IntakeSubsystem   m_intake   = new IntakeSubsystem();
  private final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
  private final HopperSubsystem   m_hopper   = new HopperSubsystem();
  private final FeederSubsystem   m_feeder   = new FeederSubsystem();
  private final ShooterSubsystem  m_shooter  = new ShooterSubsystem();
  private final TurretSubsystem   m_turret   = new TurretSubsystem(
      () -> drivebase.getPose().getRotation().getDegrees());

  private final Superstructure m_superstructure = new Superstructure(
      m_intake, m_conveyor, m_hopper, m_feeder, m_shooter, m_turret);

  private final DashboardManager dashboardManager = new DashboardManager();

  // ── Auto Chooser ──────────────────────────────────────────────────────────

  private SendableChooser<Command> autoChooser;

  // ── Toggle State ──────────────────────────────────────────────────────────

  /** True when left stick button is held/toggled to lock both turrets to left stick. */
  private boolean m_turretLocked = false;

  /** True when constant fire mode is toggled on (B button). */
  private boolean m_constantFire = false;

  // ── Drive Streams ─────────────────────────────────────────────────────────

  private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
          drivebase.getSwerveDrive(),
          () -> -driverXbox.getLeftY(),
          () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(Constants.OperatorConstants.DEADBAND)
      .scaleTranslation(Constants.DriveProfiles.FULL_SPEED_SCALE)
      .allianceRelativeControl(true);

  private final SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
      .headingWhile(true);

  private final SwerveInputStream driveAngularVelocitySlow = SwerveInputStream.of(
          drivebase.getSwerveDrive(),
          () -> -driverXbox.getLeftY(),
          () -> driverXbox.getLeftX())
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(Constants.OperatorConstants.DEADBAND)
      .scaleTranslation(Constants.DriveProfiles.SLOW_SPEED_SCALE)
      .allianceRelativeControl(true);

  // ── Dashboard Choosers ────────────────────────────────────────────────────

  private final SendableChooser<String> driveModeChooser  = new SendableChooser<>();
  private final SendableChooser<String> assistModeChooser = new SendableChooser<>();

  // ── Constructor ───────────────────────────────────────────────────────────

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    driveModeChooser.setDefaultOption("Standard", "STANDARD");
    driveModeChooser.addOption("Setpoint Generator", "SETPOINT_GENERATOR");
    SmartDashboard.putData("Drive Mode", driveModeChooser);

    assistModeChooser.setDefaultOption("Off", "OFF");
    SmartDashboard.putData("Assist Mode", assistModeChooser);
  }

  // ── Bindings ──────────────────────────────────────────────────────────────

  private void configureBindings() {

    // ── Default Commands ────────────────────────────────────────────────────

    // Drive
    Command driveFieldOriented = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    } else {
      drivebase.setDefaultCommand(driveFieldOriented);
    }

    // Turret: both turrets follow left joystick (field-centric, ±110° range)
    m_turret.setDefaultCommand(m_turret.joystickControlCommand(
        operatorXbox::getLeftX,
        operatorXbox::getLeftY,
        operatorXbox::getLeftX,
        operatorXbox::getLeftY,
        () -> true // always locked — both turrets track left stick
    ));

    // Shooter default: both sides follow left stick magnitude (turrets are locked to left stick)
    DoubleSupplier shooterMagnitude =
        () -> Math.min(1.0, Math.hypot(operatorXbox.getLeftX(), operatorXbox.getLeftY()));
    m_shooter.setDefaultCommand(m_shooter.spinFromDualMagnitudeCommand(
        shooterMagnitude, shooterMagnitude
    ));

    // ── Driver Bindings ─────────────────────────────────────────────────────

    // Zero gyro (faces 0° on blue alliance, 180° on red)
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

    // X-pattern wheel lock (defense)
    driverXbox.povUp().whileTrue(drivebase.lockCommand());

    // Center all modules to 0°
    driverXbox.back().whileTrue(drivebase.centerModulesCommand());

    // Slow mode
    driverXbox.leftTrigger(0.5).whileTrue(drivebase.driveFieldOriented(driveAngularVelocitySlow));

    // Driver intake: intake roller + floor conveyor only
    driverXbox.rightBumper().and(() -> !DriverStation.isTest())
        .whileTrue(Commands.parallel(
            m_intake.runCommand(),
            m_conveyor.runCommand()
        ));

    // Driver reverse: intake roller + floor conveyor reversed
    driverXbox.leftBumper().and(() -> !DriverStation.isTest())
        .whileTrue(Commands.parallel(
            m_intake.reverseCommand(),
            m_conveyor.reverseCommand()
        ));

    // Test mode: SysId characterization (runtime check, not init-time)
    // Driver X/Y = swerve, Driver A/B/LB = mechanisms
    driverXbox.x().and(() -> DriverStation.isTest())
        .whileTrue(drivebase.sysIdDriveMotorCommand());
    driverXbox.y().and(() -> DriverStation.isTest())
        .whileTrue(drivebase.sysIdAngleMotorCommand());
    driverXbox.a().and(() -> DriverStation.isTest())
        .whileTrue(m_shooter.sysIdMainShooterCommand());
    driverXbox.b().and(() -> DriverStation.isTest())
        .whileTrue(m_shooter.sysIdPreshooterCommand());
    driverXbox.leftBumper().and(() -> DriverStation.isTest())
        .whileTrue(m_turret.sysIdTurretCommand());

    // ── Operator Bindings ───────────────────────────────────────────────────

    // --- Turret lock toggle (left stick button) ---
    operatorXbox.leftStick().onTrue(Commands.runOnce(() -> {
      m_turretLocked = !m_turretLocked;
      SmartDashboard.putBoolean("Turret/Locked", m_turretLocked);
    }));

    // --- D-pad preset positions ---
    operatorXbox.povUp().onTrue(
        m_superstructure.setPositionCommand(ShootingPositions.CLOSE));
    operatorXbox.povDown().onTrue(
        m_superstructure.setPositionCommand(ShootingPositions.FAR));
    operatorXbox.povLeft().onTrue(
        m_superstructure.setPositionCommand(ShootingPositions.ANGLE_LEFT));
    operatorXbox.povRight().onTrue(
        m_superstructure.setPositionCommand(ShootingPositions.ANGLE_RIGHT));

    // --- Y: clear position preset, return to manual joystick ---
    operatorXbox.y().onTrue(m_superstructure.clearPositionCommand());

    // --- A: feed chain (floor + hopper + feeder) ---
    operatorXbox.a().whileTrue(Commands.parallel(
        m_conveyor.runCommand(),
        m_hopper.runCommand(),
        m_feeder.runCommand()
    ));

    // --- B: reverse feed chain (floor + hopper + feeder reversed) ---
    operatorXbox.b().whileTrue(Commands.parallel(
        m_conveyor.reverseCommand(),
        m_hopper.reverseCommand(),
        m_feeder.reverseCommand()
    ));

    // --- Hood manual overrides ---
    operatorXbox.leftBumper().onTrue(m_shooter.hoodNearCommand());
    operatorXbox.rightBumper().onTrue(m_shooter.hoodFarCommand());

    // --- Start: zero turret encoders (align to forward before pressing!) ---
    operatorXbox.start().onTrue(m_turret.zeroEncodersCommand());
  }

  // ── Public API ────────────────────────────────────────────────────────────

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public DashboardManager getDashboardManager() {
    return dashboardManager;
  }
}
