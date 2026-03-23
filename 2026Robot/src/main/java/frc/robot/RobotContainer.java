package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoShootConstants;
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
import frc.robot.util.ShooterCalculator;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveInputStream;

/**
 * Subsystem instantiation, controller bindings, and autonomous selection.
 *
 * <p>DRIVER (port 0):
 *   Left stick        — translate (field-relative)
 *   Right stick X     — rotate
 *   Left trigger      — progressive slow mode (80% → 20%)
 *   Right trigger     — intake + conveyor (progressive 0% → 100%)
 *   Right bumper held — intake + conveyor (full speed)
 *   Left bumper held  — reverse intake (eject/unjam)
 *   Start             — zero gyro (alliance-aware)
 *   POV Up            — lock wheels (X-pattern defense)
 *   POV Down          — center all modules to 0°
 *   X (test mode)     — SysId drive motors
 *   Y (test mode)     — SysId angle motors
 *   A                 — toggle heading lock (on by default)
 *   RB (test mode)    — drive distance test (3m, for wheel diameter calibration)
 *
 * <p>OPERATOR (port 1):
 *   Left stick        — aim both turrets (field-centric direction)
 *   Right trigger     — shooter speed (0 = stop, full pull = max RPM)
 *   Left stick button — toggle turret lock (reserved)
 *   D-pad Up          — preset: CLOSE (low RPM, hood near)
 *   D-pad Down        — preset: FAR (high RPM, hood far)
 *   D-pad Left        — preset: ANGLE_LEFT
 *   D-pad Right       — preset: ANGLE_RIGHT
 *   Y                 — clear preset, return to manual control
 *   X (held)          — auto-calculated shot (turret+RPM from ShooterCalculator, fires when ready)
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
  private final ShooterCalculator m_shotCalc = new ShooterCalculator();

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
          () -> -driverXbox.getLeftY() * (0.8 - 0.6 * driverXbox.getLeftTriggerAxis()),
          () -> -driverXbox.getLeftX() * (0.8 - 0.6 * driverXbox.getLeftTriggerAxis()))
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(Constants.OperatorConstants.DEADBAND)
      .scaleTranslation(Constants.DriveProfiles.FULL_SPEED_SCALE)
      .allianceRelativeControl(false);

  private final SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
      .headingWhile(true);

  // ── Dashboard Choosers ────────────────────────────────────────────────────

  private final SendableChooser<String> driveModeChooser  = new SendableChooser<>();
  private final SendableChooser<String> assistModeChooser = new SendableChooser<>();

  // ── Constructor ───────────────────────────────────────────────────────────

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Register named commands for PathPlanner event markers
    NamedCommands.registerCommand("autoShoot", buildAutoShootCommand().withTimeout(20.0));
    NamedCommands.registerCommand("stopShooter", m_shooter.stopCommand());

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

    // Shooter default: right trigger controls raw voltage (0 = stop, full pull = full power)
    m_shooter.setDefaultCommand(m_shooter.spinDutyCycleCommand(
        () -> operatorXbox.getRightTriggerAxis()
    ));

    // ── Driver Bindings ─────────────────────────────────────────────────────

    // Zero gyro (faces 0° on blue alliance, 180° on red)
    driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

    // X-pattern wheel lock (defense)
    driverXbox.povUp().whileTrue(drivebase.lockCommand());

    // Center all modules to 0°
    driverXbox.povDown().whileTrue(drivebase.centerModulesCommand());

    // Toggle heading lock (A button, normal operation only)
    driverXbox.a().and(() -> !DriverStation.isTest())
        .onTrue(Commands.runOnce(() -> drivebase.toggleHeadingCorrection()));

    // Driver intake: right trigger scales intake roller + floor conveyor (0% → 100%)
    new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.05)
        .and(() -> !DriverStation.isTest())
        .whileTrue(Commands.run(() -> {
            double speed = driverXbox.getRightTriggerAxis();
            m_intake.setSpeed(speed);
            m_conveyor.setSpeed(speed);
        }, m_intake, m_conveyor)
        .finallyDo(interrupted -> {
            m_intake.setSpeed(0);
            m_conveyor.setSpeed(0);
        })
        .withName("TriggerIntake"));

    // Driver intake: right bumper full speed intake + conveyor
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
    // RB (test mode): Drive distance test — drives 3m forward, release to stop early
    driverXbox.rightBumper().and(() -> DriverStation.isTest())
        .whileTrue(drivebase.driveDistanceTestCommand(3.0));

    // ── Operator Bindings ───────────────────────────────────────────────────

    // --- Turret lock toggle (left stick button) ---
    operatorXbox.leftStick().onTrue(Commands.runOnce(() -> {
      m_turretLocked = !m_turretLocked;
      SmartDashboard.putBoolean("Turret/Locked", m_turretLocked);
    }));

    // --- D-pad preset speeds (RPM + hood only, turret stays on joystick) ---
    operatorXbox.povUp().onTrue(m_shooter.holdRpmCommand(
        ShootingPositions.CLOSE.mainRpm(), ShootingPositions.CLOSE.preshooterRpm()));
    operatorXbox.povDown().onTrue(m_shooter.holdRpmCommand(
        ShootingPositions.FAR.mainRpm(), ShootingPositions.FAR.preshooterRpm()));
    operatorXbox.povLeft().onTrue(m_shooter.holdRpmCommand(
        ShootingPositions.ANGLE_LEFT.mainRpm(), ShootingPositions.ANGLE_LEFT.preshooterRpm()));
    operatorXbox.povRight().onTrue(m_shooter.holdRpmCommand(
        ShootingPositions.ANGLE_RIGHT.mainRpm(), ShootingPositions.ANGLE_RIGHT.preshooterRpm()));

    // --- Y: clear RPM preset, return to trigger-controlled speed ---
    operatorXbox.y().onTrue(m_shooter.stopCommand());

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

    // --- X: start auto-calculated shot, Y cancels (via stopCommand interrupting shooter) ---
    operatorXbox.x().onTrue(buildAutoShootCommand());

    // --- Hood manual overrides ---
    operatorXbox.leftBumper().onTrue(m_shooter.hoodNearCommand());
    operatorXbox.rightBumper().onTrue(m_shooter.hoodFarCommand());

    // --- Start: zero turret encoders (align to forward before pressing!) ---
    operatorXbox.start().onTrue(m_turret.zeroEncodersCommand());
  }

  // ── Auto-Shoot Command ───────────────────────────────────────────────────

  /**
   * Build the auto-calculated shooting command.
   *
   * <p>Each 20ms cycle: recalculates shot solution from current robot pose and velocity,
   * aims turret at the hub, spins shooter to calculated RPM, and sets hood. When the
   * shooter is at speed AND turret is aimed (both with generous tolerances), runs
   * the full feed chain (conveyor + hopper + feeder) continuously.
   *
   * <p>Requires turret, shooter, conveyor, hopper, and feeder subsystems — takes over
   * from their default commands while active.
   *
   * <p>Used by operator X button (whileTrue) and PathPlanner "autoShoot" named command.
   */
  private Command buildAutoShootCommand() {
    return Commands.run(() -> {
      // 1. Calculate shot solution from current state
      ShooterCalculator.ShotSolution shot = m_shotCalc.calculate(
          drivebase.getPose(), drivebase.getFieldVelocity());

      // 2. Aim turret (both turrets to same field angle)
      m_turret.setFieldAngle(shot.turretFieldAngleDeg());

      // 3. Spin up shooter + set hood
      m_shooter.setRpmAndHood(shot.mainRpm(), shot.preshooterRpm(), shot.hoodFar());

      // 4. Gate feeder on readiness (generous tolerances until tuned)
      boolean ready = shot.viable()
          && m_shooter.atSpeed(AutoShootConstants.AT_SPEED_TOLERANCE_RPM)
          && m_turret.isAtFieldAngle(shot.turretFieldAngleDeg(),
              AutoShootConstants.TURRET_TOLERANCE_DEG);

      SmartDashboard.putBoolean("AutoShoot/Ready", ready);

      // 5. Run or stop feed chain
      if (ready) {
        m_conveyor.setSpeed(Constants.ConveyorConstants.CONVEYOR_SPEED);
        m_hopper.setSpeed(Constants.HopperConstants.HOPPER_SPEED);
        m_feeder.setSpeed(Constants.FeederConstants.FEEDER_SPEED);
      } else {
        m_conveyor.setSpeed(0);
        m_hopper.setSpeed(0);
        m_feeder.setSpeed(0);
      }
    }, m_turret, m_shooter, m_conveyor, m_hopper, m_feeder)
    .finallyDo(interrupted -> {
      // Stop everything when command ends
      m_shooter.setRpmAndHood(0, 0, false);
      m_conveyor.setSpeed(0);
      m_hopper.setSpeed(0);
      m_feeder.setSpeed(0);
      SmartDashboard.putBoolean("AutoShoot/Ready", false);
    })
    .withName("AutoShoot");
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
