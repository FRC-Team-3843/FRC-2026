package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TelemetryConstants;
import frc.robot.Constants.TurretConstants;

/**
 * Central dashboard wiring class that publishes all tunable parameters to NetworkTables.
 *
 * <p>Creates TunableNumbers for every tunable parameter, grouped by subsystem with
 * structured NT paths. Also publishes read-only sensor data and system health.
 *
 * <p>Call {@link #periodic()} from Robot.robotPeriodic() to update sensor readings.
 */
public class DashboardManager {

  // --- Swerve PID Tunables ---
  private final TunableNumber m_swerveDriveP;
  private final TunableNumber m_swerveDriveI;
  private final TunableNumber m_swerveDriveD;
  private final TunableNumber m_swerveAngleP;
  private final TunableNumber m_swerveAngleI;
  private final TunableNumber m_swerveAngleD;

  // --- Turret PID Tunables ---
  private final TunableNumber m_turretLeftP;
  private final TunableNumber m_turretLeftI;
  private final TunableNumber m_turretLeftD;
  private final TunableNumber m_turretRightP;
  private final TunableNumber m_turretRightI;
  private final TunableNumber m_turretRightD;

  // --- Shooter Tunables ---
  private final TunableNumber m_preshooterP;
  private final TunableNumber m_preshooterI;
  private final TunableNumber m_preshooterD;
  private final TunableNumber m_preshooterTargetRPM;
  private final TunableNumber m_mainShooterP;
  private final TunableNumber m_mainShooterI;
  private final TunableNumber m_mainShooterD;
  private final TunableNumber m_mainShooterTargetRPM;

  // --- Auton PID Tunables ---
  private final TunableNumber m_autonTranslationP;
  private final TunableNumber m_autonTranslationI;
  private final TunableNumber m_autonTranslationD;
  private final TunableNumber m_autonRotationP;
  private final TunableNumber m_autonRotationI;
  private final TunableNumber m_autonRotationD;

  // --- Enable/Disable Flags ---
  private final TunableBoolean m_turretLeftEnabled;
  private final TunableBoolean m_turretRightEnabled;
  private final TunableBoolean m_shooterLeftEnabled;
  private final TunableBoolean m_shooterRightEnabled;
  private final TunableBoolean m_visionEnabled;

  // --- Sendable PID Controllers (for Elastic PIDController widgets) ---
  private final PIDController m_swerveDrivePID;
  private final PIDController m_swerveAnglePID;
  private final PIDController m_turretPID;
  private final PIDController m_shooterPID;
  private final PIDController m_autonTranslationPID;
  private final PIDController m_autonRotationPID;

  // --- Sensor publishing ---
  private final NetworkTable m_sensorsTable;
  private final NetworkTable m_systemTable;
  private final double m_periodSeconds;
  private double m_lastPublishTime;

  public DashboardManager() {
    // Swerve Drive PID (placeholder defaults — actual values come from YAGSL JSON)
    m_swerveDriveP = new TunableNumber("Swerve/Drive", "P", 0.002);
    m_swerveDriveI = new TunableNumber("Swerve/Drive", "I", 0.0);
    m_swerveDriveD = new TunableNumber("Swerve/Drive", "D", 0.0);
    m_swerveAngleP = new TunableNumber("Swerve/Angle", "P", 0.01);
    m_swerveAngleI = new TunableNumber("Swerve/Angle", "I", 0.0);
    m_swerveAngleD = new TunableNumber("Swerve/Angle", "D", 0.0);

    // Turret PID
    m_turretLeftP = new TunableNumber("Turret/Left", "P", TurretConstants.LEFT_KP);
    m_turretLeftI = new TunableNumber("Turret/Left", "I", TurretConstants.LEFT_KI);
    m_turretLeftD = new TunableNumber("Turret/Left", "D", TurretConstants.LEFT_KD);
    m_turretRightP = new TunableNumber("Turret/Right", "P", TurretConstants.RIGHT_KP);
    m_turretRightI = new TunableNumber("Turret/Right", "I", TurretConstants.RIGHT_KI);
    m_turretRightD = new TunableNumber("Turret/Right", "D", TurretConstants.RIGHT_KD);

    // Shooter PID + targets
    m_preshooterP = new TunableNumber("Shooter/Preshooter", "P", ShooterConstants.KP);
    m_preshooterI = new TunableNumber("Shooter/Preshooter", "I", ShooterConstants.KI);
    m_preshooterD = new TunableNumber("Shooter/Preshooter", "D", ShooterConstants.KD);
    m_preshooterTargetRPM = new TunableNumber("Shooter/Preshooter", "TargetRPM",
        ShooterConstants.PRESHOOTER_TARGET_RPM);
    m_mainShooterP = new TunableNumber("Shooter/MainShooter", "P", ShooterConstants.KP);
    m_mainShooterI = new TunableNumber("Shooter/MainShooter", "I", ShooterConstants.KI);
    m_mainShooterD = new TunableNumber("Shooter/MainShooter", "D", ShooterConstants.KD);
    m_mainShooterTargetRPM = new TunableNumber("Shooter/MainShooter", "TargetRPM",
        ShooterConstants.MAIN_SHOOTER_TARGET_RPM);

    // Auton PID
    m_autonTranslationP = new TunableNumber("Auton/Translation", "P", AutonConstants.TRANSLATION_P);
    m_autonTranslationI = new TunableNumber("Auton/Translation", "I", AutonConstants.TRANSLATION_I);
    m_autonTranslationD = new TunableNumber("Auton/Translation", "D", AutonConstants.TRANSLATION_D);
    m_autonRotationP = new TunableNumber("Auton/Rotation", "P", AutonConstants.ROTATION_P);
    m_autonRotationI = new TunableNumber("Auton/Rotation", "I", AutonConstants.ROTATION_I);
    m_autonRotationD = new TunableNumber("Auton/Rotation", "D", AutonConstants.ROTATION_D);

    // Enable/disable flags
    m_turretLeftEnabled = new TunableBoolean("Control", "Turret/Left", "Enabled", true);
    m_turretRightEnabled = new TunableBoolean("Control", "Turret/Right", "Enabled", true);
    m_shooterLeftEnabled = new TunableBoolean("Control", "Shooter/Left", "Enabled", true);
    m_shooterRightEnabled = new TunableBoolean("Control", "Shooter/Right", "Enabled", true);
    m_visionEnabled = new TunableBoolean("Control", "Vision", "Enabled",
        Constants.VisionConstants.ENABLE_VISION);

    // Sendable PID controllers for Elastic PIDController widgets
    m_swerveDrivePID = new PIDController(m_swerveDriveP.get(), m_swerveDriveI.get(), m_swerveDriveD.get());
    m_swerveAnglePID = new PIDController(m_swerveAngleP.get(), m_swerveAngleI.get(), m_swerveAngleD.get());
    m_turretPID = new PIDController(m_turretLeftP.get(), m_turretLeftI.get(), m_turretLeftD.get());
    m_shooterPID = new PIDController(m_preshooterP.get(), m_preshooterI.get(), m_preshooterD.get());
    m_autonTranslationPID = new PIDController(
        m_autonTranslationP.get(), m_autonTranslationI.get(), m_autonTranslationD.get());
    m_autonRotationPID = new PIDController(
        m_autonRotationP.get(), m_autonRotationI.get(), m_autonRotationD.get());

    SmartDashboard.putData("PID/Swerve Drive", m_swerveDrivePID);
    SmartDashboard.putData("PID/Swerve Angle", m_swerveAnglePID);
    SmartDashboard.putData("PID/Turret", m_turretPID);
    SmartDashboard.putData("PID/Shooter", m_shooterPID);
    SmartDashboard.putData("PID/Auton Translation", m_autonTranslationPID);
    SmartDashboard.putData("PID/Auton Rotation", m_autonRotationPID);

    // Sensor and system health tables
    m_sensorsTable = NetworkTableInstance.getDefault().getTable("Sensors");
    m_systemTable = NetworkTableInstance.getDefault().getTable("System");
    m_periodSeconds = TelemetryConstants.PUBLISH_HZ <= 0.0
        ? 0.1 : 1.0 / TelemetryConstants.PUBLISH_HZ;
    m_lastPublishTime = 0.0;
  }

  /**
   * Call from Robot.robotPeriodic() to publish sensor data and system health.
   * Throttled to the configured publish rate.
   */
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    if (now - m_lastPublishTime < m_periodSeconds) {
      return;
    }
    m_lastPublishTime = now;

    // System health
    m_systemTable.getEntry("batteryVoltage").setDouble(RobotController.getBatteryVoltage());
    m_systemTable.getEntry("canUtilization").setDouble(
        RobotController.getCANStatus().percentBusUtilization * 100.0);

    // Sync Sendable PID controllers if tuning values changed
    if (TelemetryConstants.TUNING_MODE) {
      syncPIDControllers();
    }
  }

  /**
   * Check all TunableNumbers for changes and update the Sendable PID controllers.
   * This keeps the Elastic PIDController widgets in sync with the TunableNumber sliders.
   */
  private void syncPIDControllers() {
    if (m_swerveDriveP.hasChanged() || m_swerveDriveI.hasChanged() || m_swerveDriveD.hasChanged()) {
      m_swerveDrivePID.setPID(m_swerveDriveP.get(), m_swerveDriveI.get(), m_swerveDriveD.get());
    }
    if (m_swerveAngleP.hasChanged() || m_swerveAngleI.hasChanged() || m_swerveAngleD.hasChanged()) {
      m_swerveAnglePID.setPID(m_swerveAngleP.get(), m_swerveAngleI.get(), m_swerveAngleD.get());
    }
    if (m_turretLeftP.hasChanged() || m_turretLeftI.hasChanged() || m_turretLeftD.hasChanged()) {
      m_turretPID.setPID(m_turretLeftP.get(), m_turretLeftI.get(), m_turretLeftD.get());
    }
    if (m_preshooterP.hasChanged() || m_preshooterI.hasChanged() || m_preshooterD.hasChanged()) {
      m_shooterPID.setPID(m_preshooterP.get(), m_preshooterI.get(), m_preshooterD.get());
    }
    if (m_autonTranslationP.hasChanged() || m_autonTranslationI.hasChanged()
        || m_autonTranslationD.hasChanged()) {
      m_autonTranslationPID.setPID(
          m_autonTranslationP.get(), m_autonTranslationI.get(), m_autonTranslationD.get());
    }
    if (m_autonRotationP.hasChanged() || m_autonRotationI.hasChanged()
        || m_autonRotationD.hasChanged()) {
      m_autonRotationPID.setPID(
          m_autonRotationP.get(), m_autonRotationI.get(), m_autonRotationD.get());
    }
  }

  // --- Getters for subsystems to read tuned values ---

  public double getSwerveDriveP() { return m_swerveDriveP.get(); }
  public double getSwerveDriveI() { return m_swerveDriveI.get(); }
  public double getSwerveDriveD() { return m_swerveDriveD.get(); }
  public double getSwerveAngleP() { return m_swerveAngleP.get(); }
  public double getSwerveAngleI() { return m_swerveAngleI.get(); }
  public double getSwerveAngleD() { return m_swerveAngleD.get(); }

  public double getTurretLeftP() { return m_turretLeftP.get(); }
  public double getTurretLeftI() { return m_turretLeftI.get(); }
  public double getTurretLeftD() { return m_turretLeftD.get(); }
  public double getTurretRightP() { return m_turretRightP.get(); }
  public double getTurretRightI() { return m_turretRightI.get(); }
  public double getTurretRightD() { return m_turretRightD.get(); }

  public double getPreshooterP() { return m_preshooterP.get(); }
  public double getPreshooterI() { return m_preshooterI.get(); }
  public double getPreshooterD() { return m_preshooterD.get(); }
  public double getPreshooterTargetRPM() { return m_preshooterTargetRPM.get(); }
  public double getMainShooterP() { return m_mainShooterP.get(); }
  public double getMainShooterI() { return m_mainShooterI.get(); }
  public double getMainShooterD() { return m_mainShooterD.get(); }
  public double getMainShooterTargetRPM() { return m_mainShooterTargetRPM.get(); }

  public double getAutonTranslationP() { return m_autonTranslationP.get(); }
  public double getAutonTranslationI() { return m_autonTranslationI.get(); }
  public double getAutonTranslationD() { return m_autonTranslationD.get(); }
  public double getAutonRotationP() { return m_autonRotationP.get(); }
  public double getAutonRotationI() { return m_autonRotationI.get(); }
  public double getAutonRotationD() { return m_autonRotationD.get(); }

  public boolean isTurretLeftEnabled() { return m_turretLeftEnabled.get(); }
  public boolean isTurretRightEnabled() { return m_turretRightEnabled.get(); }
  public boolean isShooterLeftEnabled() { return m_shooterLeftEnabled.get(); }
  public boolean isShooterRightEnabled() { return m_shooterRightEnabled.get(); }
  public boolean isVisionEnabled() { return m_visionEnabled.get(); }
}
