package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import java.io.File;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Swerve drive subsystem using YAGSL.
 * Provides field-oriented driving, PathPlanner autonomous, and SysId characterization.
 */
public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  private final Vision vision;

  /**
   * Initialize the swerve subsystem from JSON configuration files.
   *
   * @param directory Directory containing YAGSL swerve config (deploy/swerve/).
   */
  public SwerveSubsystem(File directory) {
    // Set telemetry verbosity BEFORE creating SwerveDrive for AdvantageScope compatibility
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(
          Constants.MAX_SPEED,
          new Pose2d(new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Heading correction should only be used while controlling via direct angle
    swerveDrive.setHeadingCorrection(false);
    // Reduces wheel scrub when modules change direction
    swerveDrive.setCosineCompensator(true);
    // Corrects translational skew that worsens with angular velocity
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    // Disable auto-sync (enable if encoders drift during match)
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

    this.vision = new Vision(this::getPose, swerveDrive.field);
    setupPathPlanner();
  }

  /**
   * Construct the swerve drive from pre-built configurations.
   *
   * <p>Alternative constructor for programmatic configuration (useful for testing or
   * non-standard configurations). Most robots should use the JSON-based constructor instead.
   *
   * @param driveCfg Swerve drive configuration (module positions, motor types, etc.).
   * @param controllerCfg Swerve controller configuration (PID values, constraints, etc.).
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(
        driveCfg, controllerCfg, Constants.MAX_SPEED,
        new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)), Rotation2d.fromDegrees(0)));
    this.vision = new Vision(this::getPose, swerveDrive.field);
  }

  /**
   * Called periodically by the scheduler (every 20ms).
   * Odometry updates are handled by YAGSL's dedicated thread.
   */
  @Override
  public void periodic() {
    // Odometry is updated automatically by YAGSL's odometry thread
    vision.updatePoseEstimation(swerveDrive);
  }

  /**
   * Configure PathPlanner's AutoBuilder for autonomous path following.
   *
   * <p>Sets up the holonomic drive controller with PID constants from Constants.java,
   * configures alliance flipping for red alliance paths, and preloads pathfinding
   * to eliminate first-use delay during matches.
   */
  private void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            swerveDrive.drive(
                speedsRobotRelative,
                swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                moduleFeedForwards.linearForces());
          },
          new PPHolonomicDriveController(
              new PIDConstants(Constants.AutonConstants.TRANSLATION_P,
                               Constants.AutonConstants.TRANSLATION_I,
                               Constants.AutonConstants.TRANSLATION_D),
              new PIDConstants(Constants.AutonConstants.ROTATION_P,
                               Constants.AutonConstants.ROTATION_I,
                               Constants.AutonConstants.ROTATION_D)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Preload pathfinding to avoid first-use delay
    PathfindingCommand.warmupCommand().schedule();
  }

  // --- Commands ---

  /**
   * Lock the swerve modules into an X pattern to resist pushing.
   *
   * <p>Rotates modules to form an X (FL and RR at +45°, FR and RL at -45°). This makes
   * the robot extremely difficult to push, useful for defense or holding position.
   * The command runs continuously until interrupted.
   *
   * @return Command that locks modules in X formation.
   */
  public Command lockCommand() {
    return run(() -> swerveDrive.lockPose());
  }

  /**
   * Point all modules straight forward (0 degrees).
   *
   * <p>Useful for alignment checks and verifying module calibration. All modules will
   * continuously rotate to face forward (same direction as robot front). The command
   * runs until interrupted.
   *
   * @return Command that centers all modules to 0 degrees.
   */
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                          .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Zero the gyro with alliance awareness (command version).
   *
   * <p>This is a one-shot command that zeroes the gyro and immediately finishes.
   * Bound to the driver's Start button in RobotContainer for match initialization.
   *
   * <p>Alliance handling: Blue = 0°, Red = 180° (both facing away from driver station).
   *
   * @return Command that zeroes the gyro once.
   */
  public Command zeroGyroCommand() {
    return Commands.runOnce(() -> zeroGyroWithAlliance());
  }

  /**
   * Drive the robot field-oriented using a ChassisSpeeds supplier.
   *
   * <p>This command continuously reads velocity from the supplier and converts it to
   * field-relative motion (forward is always away from the driver's station,
   * regardless of robot rotation). The command runs until interrupted.
   *
   * @param velocity Supplier providing desired ChassisSpeeds (typically from controller input).
   * @return Command that drives the robot field-oriented.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  /**
   * Use PathPlanner pathfinding to drive to a target pose.
   *
   * <p>This command dynamically generates a path from the current robot pose to the
   * target, avoiding obstacles if configured in PathPlanner. Useful for autonomous
   * repositioning or driver-assist features like "snap to scoring position."
   *
   * <p>The command finishes when the robot reaches the target within tolerance.
   *
   * @param pose Target field-relative pose (X, Y, rotation).
   * @return Command that pathfinds to the pose and finishes when reached.
   */
  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
    return AutoBuilder.pathfindToPose(
        pose, constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0));
  }

  /**
   * Drive with Team 254's setpoint generator (via PathPlanner) for field-relative input.
   *
   * <p>The setpoint generator optimizes module trajectories to reduce wheel scrub and
   * improve acceleration by considering individual module limits. This results in
   * smoother, more predictable motion compared to naive inverse kinematics, especially
   * during aggressive maneuvers.
   *
   * <p>Use this for teleop driving if you want smoother acceleration curves and better
   * module coordination. The downside is slightly increased computational overhead.
   *
   * @param fieldRelativeSpeeds Supplier providing desired field-relative speeds.
   * @return Command using the setpoint generator, or Commands.none() on configuration error.
   */
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(
          () -> ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading()));
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
  }

  /**
   * Internal implementation of setpoint generator drive with robot-relative speeds.
   *
   * <p>Maintains previous setpoint state between loop iterations to ensure smooth
   * transitions. Uses FPGA timestamp for accurate dt calculation.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed) throws Exception {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
        RobotConfig.fromGUISettings(),
        swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
        new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                           swerveDrive.getStates(),
                           DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(
        () -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(
              prevSetpoint.get(),
              robotRelativeChassisSpeed.get(),
              newTime - previousTime.get());
          swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                            newSetpoint.moduleStates(),
                            newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);
        });
  }

  /**
   * SysId routine for characterizing drive motors.
   *
   * <p>Runs a quasistatic + dynamic test to determine kS, kV, kA for the drive motors.
   * This data is used to improve feedforward control and autonomous path following.
   *
   * <p><b>Usage:</b> Enable test mode, bind this command to a controller button, then
   * run the SysId analyzer tool on the generated log. Update the PID values in
   * swerve JSON configs based on the results.
   *
   * @return SysId command for drive characterization (3s quasistatic, 5s dynamic, 3s quasistatic).
   */
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * SysId routine for characterizing angle (steer) motors.
   *
   * <p>Runs a quasistatic + dynamic test to determine kS, kV, kA for the angle motors.
   * This improves module rotation accuracy and responsiveness.
   *
   * <p><b>Usage:</b> Enable test mode, bind this command to a controller button, then
   * run the SysId analyzer tool on the generated log. Update PID values in
   * swerve JSON configs based on the results.
   *
   * @return SysId command for angle characterization (3s quasistatic, 5s dynamic, 3s quasistatic).
   */
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  // --- Drive Methods ---

  /**
   * Drive field-oriented with a ChassisSpeeds value directly (non-command method).
   *
   * <p>Use this method when you need to set drive speeds from outside a command context,
   * such as from within another subsystem's periodic method or a complex command's execute().
   *
   * @param velocity Field-relative chassis speeds (vx, vy, omega).
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive robot-oriented with a ChassisSpeeds value (non-command method).
   *
   * <p>Robot-relative means forward is always in the direction the robot is facing,
   * not relative to the field. Useful for autonomous routines that need precise
   * robot-frame control.
   *
   * @param velocity Robot-relative chassis speeds (vx, vy, omega).
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  /**
   * Drive with translation vector and rotation rate.
   *
   * <p>Lower-level drive method that accepts translation as a 2D vector instead of
   * separate X/Y components. The fourth parameter (isOpenLoop) is hardcoded to false
   * in this wrapper to always use closed-loop velocity control.
   *
   * @param translation Translation vector in meters per second.
   * @param rotation Rotation rate in radians per second.
   * @param fieldRelative If true, translation is field-relative; if false, robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  // --- Getters ---

  /**
   * Get the current robot pose from odometry.
   *
   * @return Current field-relative pose (X, Y, rotation).
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Reset odometry to a specific pose.
   *
   * <p>Use this at the start of autonomous to set the robot's known starting position.
   * Can also be called during teleop to correct for vision measurements or manual repositioning.
   *
   * @param pose New field-relative pose.
   */
  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  /**
   * Get the robot's current velocity in robot-relative coordinates.
   *
   * @return Robot-relative chassis speeds (vx forward, vy left, omega CCW).
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the robot's current velocity in field-relative coordinates.
   *
   * @return Field-relative chassis speeds.
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * <p>Useful for custom path following or trajectory generation that needs to
   * convert between chassis speeds and individual module states.
   *
   * @return SwerveDriveKinematics configured with module positions.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Get the robot's current heading (rotation component of pose).
   *
   * @return Current heading as a Rotation2d.
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the underlying YAGSL SwerveDrive object.
   *
   * <p>Provides direct access to YAGSL methods not exposed through this subsystem.
   * Use with caution - prefer using SwerveSubsystem's wrapped methods when possible.
   *
   * @return The YAGSL SwerveDrive instance.
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  // --- Utility ---

  /**
   * Set motor brake or coast mode for all swerve modules.
   *
   * <p>Brake mode holds position when motors are idle (used during enabled periods).
   * Coast mode allows free movement (used after disable timeout for easy pit pushing).
   *
   * @param brake True for brake mode, false for coast mode.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Zero the gyro to the current heading.
   *
   * <p>After calling this, the robot's current facing direction becomes the new 0 degrees.
   * This does NOT account for alliance - use zeroGyroWithAlliance() for match use.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Check if the robot is on the red alliance.
   *
   * @return True if red alliance, false if blue or alliance not available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /**
   * Zero the gyro with alliance awareness.
   *
   * <p>Blue alliance: 0 degrees faces away from the driver station (toward opponent wall).
   * Red alliance: 0 degrees also faces away from the driver station, but that's 180 degrees
   * in field coordinates since red drivers face the opposite direction.
   *
   * <p>This method should be bound to a driver button (usually Start) and pressed at the
   * start of each match after aligning the robot to face away from the driver station.
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Lock the swerve modules in an X pattern to resist pushing.
   *
   * <p>Non-command version of lockCommand(). Use this when you need to lock wheels
   * from within another command's execute() method.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  /**
   * Post a trajectory to the SmartDashboard field widget for visualization.
   *
   * <p>Useful for debugging autonomous paths. The trajectory will appear on the
   * Field2d widget in Glass/Shuffleboard.
   *
   * @param trajectory The trajectory to visualize.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }
}
