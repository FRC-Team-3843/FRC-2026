package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

/**
 * Calculates turret angle, hood position, and RPMs to hit the hub from any field position.
 *
 * <p>Features:
 * <ul>
 *   <li>Numerical trajectory simulation with air drag</li>
 *   <li>Shoot-on-the-move compensation (leads the target based on robot velocity)</li>
 *   <li>Alliance-aware hub position flipping</li>
 *   <li>Steepest landing angle optimization (pos1 first, fall back to pos2)</li>
 *   <li>Dashboard-tunable slip factors per hood position</li>
 * </ul>
 */
public class ShooterCalculator {

  // ── Field & Goal Constants ──────────────────────────────────────────────

  /** Field dimensions in meters. */
  private static final double FIELD_LENGTH_M = 16.54;
  private static final double FIELD_WIDTH_M  = 8.07;

  /**
   * Blue alliance hub center in field coordinates (meters).
   * ESTIMATE — update with measured value from PathPlanner field view.
   */
  private static final Translation2d BLUE_HUB = new Translation2d(4.5, 3.17);

  /** Height of the center of the hub opening, in meters. 56 inches confirmed at event. */
  private static final double HUB_HEIGHT_M = Units.inchesToMeters(56.0);

  /** Shooter exit height above floor. 30 inches (max robot height). */
  private static final double SHOOTER_HEIGHT_M = Units.inchesToMeters(30.0);

  /** Shooter exit offset forward of robot center. 10" from front bumper edge = 6.5" from center. */
  private static final double SHOOTER_FORWARD_OFFSET_M = Units.inchesToMeters(6.5);

  /** Height difference the ball must climb. */
  private static final double DELTA_H = HUB_HEIGHT_M - SHOOTER_HEIGHT_M;

  // ── Ball Physics ────────────────────────────────────────────────────────

  private static final double BALL_MASS_KG       = 0.227;  // 0.5 lbs
  private static final double BALL_DIAMETER_M    = 0.1524; // 6 inches
  private static final double BALL_CROSS_SECTION = Math.PI * Math.pow(BALL_DIAMETER_M / 2, 2);
  private static final double CD                 = 0.47;   // smooth sphere
  private static final double AIR_DENSITY        = 1.225;  // kg/m³ sea level

  /** Drag constant: F_drag = DRAG_K * v². Deceleration = DRAG_K * v² / BALL_MASS_KG. */
  private static final double DRAG_K = 0.5 * CD * AIR_DENSITY * BALL_CROSS_SECTION;

  // ── Shooter Geometry ────────────────────────────────────────────────────

  private static final double WHEEL_CIRCUMFERENCE_M =
      Math.PI * Units.inchesToMeters(ShooterConstants.SHOOTER_WHEEL_DIAMETER_IN);

  /** Hood position 1: steep angle for close range (70° from horizontal). */
  private static final double POS1_LAUNCH_RAD = Math.toRadians(70.0);
  /** Hood position 2: flatter angle for far range (45° from horizontal). */
  private static final double POS2_LAUNCH_RAD = Math.toRadians(45.0);

  // ── Simulation Parameters ──────────────────────────────────────────────

  private static final double SIM_DT       = 0.001; // 1ms time step
  private static final double SIM_MAX_TIME = 3.0;   // max 3 seconds of flight
  private static final int    SEARCH_ITERATIONS = 30; // binary search precision

  // ── Tunable Slip Factors (read from SmartDashboard) ─────────────────────

  private double m_pos1SlipMain = 0.75;
  private double m_pos1SlipPre  = 0.70;
  private double m_pos2SlipMain = 0.75;
  private double m_pos2SlipPre  = 0.70;

  // ── Result Record ──────────────────────────────────────────────────────

  /** Immutable result of a shot calculation. */
  public record ShotSolution(
      double turretFieldAngleDeg,
      double mainRpm,
      double preshooterRpm,
      boolean hoodFar,
      double distanceM,
      double landingAngleDeg,
      double ballExitVelocity,
      boolean viable
  ) {}

  // ── Constructor ─────────────────────────────────────────────────────────

  public ShooterCalculator() {
    // Publish initial slip factors to dashboard for tuning
    SmartDashboard.putNumber("ShotCalc/Pos1SlipMain", m_pos1SlipMain);
    SmartDashboard.putNumber("ShotCalc/Pos1SlipPre",  m_pos1SlipPre);
    SmartDashboard.putNumber("ShotCalc/Pos2SlipMain", m_pos2SlipMain);
    SmartDashboard.putNumber("ShotCalc/Pos2SlipPre",  m_pos2SlipPre);
    SmartDashboard.putNumber("ShotCalc/HubX_blue", BLUE_HUB.getX());
    SmartDashboard.putNumber("ShotCalc/HubY_blue", BLUE_HUB.getY());
  }

  // ── Main Calculation ───────────────────────────────────────────────────

  /**
   * Calculate the optimal shot solution for the current robot state.
   *
   * @param robotPose    current field-relative pose
   * @param robotVelocity current field-relative velocity (for shoot-on-move)
   * @return ShotSolution with turret angle, RPMs, hood position, and viability
   */
  public ShotSolution calculate(Pose2d robotPose, ChassisSpeeds robotVelocity) {
    // Read tunable slip factors from dashboard
    m_pos1SlipMain = SmartDashboard.getNumber("ShotCalc/Pos1SlipMain", m_pos1SlipMain);
    m_pos1SlipPre  = SmartDashboard.getNumber("ShotCalc/Pos1SlipPre",  m_pos1SlipPre);
    m_pos2SlipMain = SmartDashboard.getNumber("ShotCalc/Pos2SlipMain", m_pos2SlipMain);
    m_pos2SlipPre  = SmartDashboard.getNumber("ShotCalc/Pos2SlipPre",  m_pos2SlipPre);

    // Read tunable hub position
    double hubXBlue = SmartDashboard.getNumber("ShotCalc/HubX_blue", BLUE_HUB.getX());
    double hubYBlue = SmartDashboard.getNumber("ShotCalc/HubY_blue", BLUE_HUB.getY());

    // Alliance-aware hub position
    Translation2d hub = getHubPosition(hubXBlue, hubYBlue);

    // Shooter exit position in field coords
    double heading = robotPose.getRotation().getRadians();
    double shooterX = robotPose.getX() + SHOOTER_FORWARD_OFFSET_M * Math.cos(heading);
    double shooterY = robotPose.getY() + SHOOTER_FORWARD_OFFSET_M * Math.sin(heading);

    // Vector from shooter to hub
    double dx = hub.getX() - shooterX;
    double dy = hub.getY() - shooterY;
    double horizontalDistance = Math.hypot(dx, dy);

    // Turret field angle (turret convention: 0°=+X downfield, +90°=field right=-Y)
    double turretFieldAngleDeg = Math.toDegrees(Math.atan2(-dy, dx));

    // Shoot-on-move compensation: lead the target
    // Subtract robot velocity from required ball velocity by shifting the virtual target
    double fieldVx = robotVelocity.vxMetersPerSecond;
    double fieldVy = robotVelocity.vyMetersPerSecond;
    double robotSpeed = Math.hypot(fieldVx, fieldVy);

    if (robotSpeed > 0.1) {
      // Estimate flight time from a no-compensation guess, then adjust
      double roughFlightTime = horizontalDistance / 8.0; // assume ~8 m/s ball speed
      double virtualDx = dx - fieldVx * roughFlightTime;
      double virtualDy = dy - fieldVy * roughFlightTime;
      horizontalDistance = Math.hypot(virtualDx, virtualDy);
      turretFieldAngleDeg = Math.toDegrees(Math.atan2(-virtualDy, virtualDx));
    }

    // Try pos1 (steep, 70°) first for best landing angle
    double exitVel1 = solveExitVelocity(horizontalDistance, DELTA_H, POS1_LAUNCH_RAD);
    if (exitVel1 > 0) {
      double mainRpm = exitVelocityToWheelRpm(exitVel1, m_pos1SlipMain);
      if (mainRpm <= ShooterConstants.MAX_WHEEL_RPM) {
        double preRpm = exitVelocityToWheelRpm(
            exitVel1 * ShooterConstants.PRESHOOTER_SPEED_RATIO, m_pos1SlipPre);
        double landingAngle = calcLandingAngle(exitVel1, POS1_LAUNCH_RAD, horizontalDistance);
        publishSolution(turretFieldAngleDeg, mainRpm, preRpm, false, horizontalDistance,
            landingAngle, exitVel1);
        return new ShotSolution(turretFieldAngleDeg, mainRpm, preRpm, false,
            horizontalDistance, landingAngle, exitVel1, true);
      }
    }

    // Fall back to pos2 (45°) for longer range
    double exitVel2 = solveExitVelocity(horizontalDistance, DELTA_H, POS2_LAUNCH_RAD);
    if (exitVel2 > 0) {
      double mainRpm = exitVelocityToWheelRpm(exitVel2, m_pos2SlipMain);
      if (mainRpm <= ShooterConstants.MAX_WHEEL_RPM) {
        double preRpm = exitVelocityToWheelRpm(
            exitVel2 * ShooterConstants.PRESHOOTER_SPEED_RATIO, m_pos2SlipPre);
        double landingAngle = calcLandingAngle(exitVel2, POS2_LAUNCH_RAD, horizontalDistance);
        publishSolution(turretFieldAngleDeg, mainRpm, preRpm, true, horizontalDistance,
            landingAngle, exitVel2);
        return new ShotSolution(turretFieldAngleDeg, mainRpm, preRpm, true,
            horizontalDistance, landingAngle, exitVel2, true);
      }
    }

    // Out of range — return max RPM with best guess
    double maxRpm = ShooterConstants.MAX_WHEEL_RPM;
    publishSolution(turretFieldAngleDeg, maxRpm, maxRpm * ShooterConstants.PRESHOOTER_SPEED_RATIO,
        true, horizontalDistance, 0, 0);
    return new ShotSolution(turretFieldAngleDeg, maxRpm,
        maxRpm * ShooterConstants.PRESHOOTER_SPEED_RATIO, true,
        horizontalDistance, 0, 0, false);
  }

  // ── Alliance Flip ──────────────────────────────────────────────────────

  private Translation2d getHubPosition(double blueX, double blueY) {
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    if (isRed) {
      // Standard FRC mirror symmetry: flip X only (mirror across field centerline)
      return new Translation2d(FIELD_LENGTH_M - blueX, blueY);
    }
    return new Translation2d(blueX, blueY);
  }

  // ── Trajectory Solver ──────────────────────────────────────────────────

  /**
   * Binary search for the exit velocity that makes the ball hit (targetX, targetY)
   * with the given launch angle, accounting for air drag.
   *
   * @return exit velocity in m/s, or -1 if no solution found
   */
  private double solveExitVelocity(double targetX, double targetY, double launchAngleRad) {
    double vLow = 1.0;
    double vHigh = 40.0;

    // Check if high velocity overshoots (sanity check)
    double yAtHigh = simulateYAtX(vHigh, launchAngleRad, targetX);
    if (Double.isNaN(yAtHigh)) return -1;

    double yAtLow = simulateYAtX(vLow, launchAngleRad, targetX);

    // If even max velocity can't reach targetY, no solution
    if (yAtHigh < targetY && yAtLow < targetY) return -1;
    // If even min velocity overshoots, use min (unlikely)
    if (yAtLow > targetY) return vLow;

    // Binary search
    for (int i = 0; i < SEARCH_ITERATIONS; i++) {
      double vMid = (vLow + vHigh) / 2.0;
      double yAtMid = simulateYAtX(vMid, launchAngleRad, targetX);
      if (Double.isNaN(yAtMid) || yAtMid < targetY) {
        vLow = vMid;
      } else {
        vHigh = vMid;
      }
    }
    return (vLow + vHigh) / 2.0;
  }

  /**
   * Simulate a ball trajectory and return the Y (height) when it reaches horizontal distance X.
   * Uses Euler integration with air drag.
   *
   * @return height at targetX, or NaN if ball never reaches that distance
   */
  private double simulateYAtX(double exitVelocity, double launchAngleRad, double targetX) {
    double vx = exitVelocity * Math.cos(launchAngleRad);
    double vy = exitVelocity * Math.sin(launchAngleRad);
    double x = 0;
    double y = 0;

    for (double t = 0; t < SIM_MAX_TIME; t += SIM_DT) {
      double speed = Math.hypot(vx, vy);
      double dragAccel = DRAG_K * speed / BALL_MASS_KG; // drag decel magnitude

      // Drag acts opposite to velocity
      double ax = -dragAccel * vx; // vx/speed * drag * speed = drag * vx
      double ay = -9.81 - dragAccel * vy;

      vx += ax * SIM_DT;
      vy += ay * SIM_DT;
      x += vx * SIM_DT;
      y += vy * SIM_DT;

      if (x >= targetX) {
        // Interpolate to exact targetX
        double frac = (targetX - (x - vx * SIM_DT)) / (vx * SIM_DT);
        return (y - vy * SIM_DT) + vy * SIM_DT * frac;
      }

      // Ball falling below launch height and past apex — no solution at this velocity
      if (y < -1.0) return Double.NaN;
    }
    return Double.NaN; // never reached targetX
  }

  /**
   * Calculate the landing angle (angle from horizontal, positive = steep descent)
   * at the target distance.
   */
  private double calcLandingAngle(double exitVelocity, double launchAngleRad, double targetX) {
    double vx = exitVelocity * Math.cos(launchAngleRad);
    double vy = exitVelocity * Math.sin(launchAngleRad);
    double x = 0;

    for (double t = 0; t < SIM_MAX_TIME; t += SIM_DT) {
      double speed = Math.hypot(vx, vy);
      double dragAccel = DRAG_K * speed / BALL_MASS_KG;

      double ax = -dragAccel * vx;
      double ay = -9.81 - dragAccel * vy;

      vx += ax * SIM_DT;
      vy += ay * SIM_DT;
      x += vx * SIM_DT;

      if (x >= targetX) {
        // Landing angle = angle of velocity vector below horizontal
        return Math.toDegrees(Math.atan2(-vy, vx));
      }
    }
    return 0;
  }

  // ── Unit Conversion ────────────────────────────────────────────────────

  private double exitVelocityToWheelRpm(double exitVelocity, double slipFactor) {
    double wheelSurfaceSpeed = exitVelocity / slipFactor;
    return (wheelSurfaceSpeed / WHEEL_CIRCUMFERENCE_M) * 60.0;
  }

  // ── Dashboard Publishing ───────────────────────────────────────────────

  private void publishSolution(double turretDeg, double mainRpm, double preRpm,
      boolean hoodFar, double distance, double landingAngle, double exitVel) {
    SmartDashboard.putNumber("ShotCalc/TurretAngleDeg", turretDeg);
    SmartDashboard.putNumber("ShotCalc/MainRPM", mainRpm);
    SmartDashboard.putNumber("ShotCalc/PreRPM", preRpm);
    SmartDashboard.putBoolean("ShotCalc/HoodFar", hoodFar);
    SmartDashboard.putNumber("ShotCalc/DistanceM", distance);
    SmartDashboard.putNumber("ShotCalc/LandingAngleDeg", landingAngle);
    SmartDashboard.putNumber("ShotCalc/ExitVelocity", exitVel);
  }
}
