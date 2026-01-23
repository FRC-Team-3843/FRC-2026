package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/**
 * Minimal NetworkTables telemetry publisher with throttling.
 *
 * <p>Keep the publish rate low to protect RoboRIO 1 CPU and NT bandwidth.
 */
public final class TelemetryPublisher {
  private final NetworkTable table;
  private final double periodSeconds;
  private double lastPublishTime;

  public TelemetryPublisher(String tableName, double publishHz) {
    table = NetworkTableInstance.getDefault().getTable(tableName);
    periodSeconds = publishHz <= 0.0 ? 0.1 : 1.0 / publishHz;
    lastPublishTime = 0.0;
  }

  public void publishDrivebase(
      Pose2d pose,
      ChassisSpeeds speeds,
      String driveMode,
      String assistMode,
      boolean visionEnabled) {
    double now = Timer.getFPGATimestamp();
    if (now - lastPublishTime < periodSeconds) {
      return;
    }
    lastPublishTime = now;

    table.getEntry("pose").setDoubleArray(new double[] {
        pose.getX(), pose.getY(), pose.getRotation().getRadians()
    });
    table.getEntry("speeds").setDoubleArray(new double[] {
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond
    });
    table.getEntry("driveMode").setString(driveMode);
    table.getEntry("assistMode").setString(assistMode);
    table.getEntry("visionEnabled").setBoolean(visionEnabled);
  }

  public void publishDrivebaseTuning(
      Pose2d pose,
      ChassisSpeeds speeds,
      SwerveModuleState[] moduleStates,
      ModuleTelemetryData moduleTelemetry,
      String driveMode,
      String assistMode,
      boolean visionEnabled) {
    publishDrivebase(pose, speeds, driveMode, assistMode, visionEnabled);
    if (moduleStates == null) {
      return;
    }

    // Keep arrays flat and numeric for easier log parsing and AI analysis.
    double[] moduleAngles = new double[moduleStates.length];
    double[] moduleSpeeds = new double[moduleStates.length];
    for (int i = 0; i < moduleStates.length; i++) {
      moduleAngles[i] = moduleStates[i].angle.getRadians();
      moduleSpeeds[i] = moduleStates[i].speedMetersPerSecond;
    }
    table.getEntry("moduleAnglesRad").setDoubleArray(moduleAngles);
    table.getEntry("moduleSpeedsMps").setDoubleArray(moduleSpeeds);
    table.getEntry("tuningMode").setBoolean(true);

    if (moduleTelemetry != null) {
      if (moduleTelemetry.driveCurrentA != null) {
        table.getEntry("driveCurrentA").setDoubleArray(moduleTelemetry.driveCurrentA);
      }
      if (moduleTelemetry.angleCurrentA != null) {
        table.getEntry("angleCurrentA").setDoubleArray(moduleTelemetry.angleCurrentA);
      }
      if (moduleTelemetry.driveTempC != null) {
        table.getEntry("driveTempC").setDoubleArray(moduleTelemetry.driveTempC);
      }
      if (moduleTelemetry.angleTempC != null) {
        table.getEntry("angleTempC").setDoubleArray(moduleTelemetry.angleTempC);
      }
      if (moduleTelemetry.driveVoltageV != null) {
        table.getEntry("driveVoltageV").setDoubleArray(moduleTelemetry.driveVoltageV);
      }
      if (moduleTelemetry.angleVoltageV != null) {
        table.getEntry("angleVoltageV").setDoubleArray(moduleTelemetry.angleVoltageV);
      }
      if (moduleTelemetry.driveAppliedOutput != null) {
        table.getEntry("driveAppliedOutput").setDoubleArray(moduleTelemetry.driveAppliedOutput);
      }
      if (moduleTelemetry.angleAppliedOutput != null) {
        table.getEntry("angleAppliedOutput").setDoubleArray(moduleTelemetry.angleAppliedOutput);
      }
    }
  }
}
