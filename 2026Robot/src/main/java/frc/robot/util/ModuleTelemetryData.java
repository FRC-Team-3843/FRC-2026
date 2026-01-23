package frc.robot.util;

/**
 * Container for per-module telemetry arrays.
 *
 * <p>Arrays are indexed by module order (FL, FR, BL, BR).
 */
public final class ModuleTelemetryData {
  public final double[] driveCurrentA;
  public final double[] angleCurrentA;
  public final double[] driveTempC;
  public final double[] angleTempC;
  public final double[] driveVoltageV;
  public final double[] angleVoltageV;
  public final double[] driveAppliedOutput;
  public final double[] angleAppliedOutput;

  public ModuleTelemetryData(
      double[] driveCurrentA,
      double[] angleCurrentA,
      double[] driveTempC,
      double[] angleTempC,
      double[] driveVoltageV,
      double[] angleVoltageV,
      double[] driveAppliedOutput,
      double[] angleAppliedOutput) {
    this.driveCurrentA = driveCurrentA;
    this.angleCurrentA = angleCurrentA;
    this.driveTempC = driveTempC;
    this.angleTempC = angleTempC;
    this.driveVoltageV = driveVoltageV;
    this.angleVoltageV = angleVoltageV;
    this.driveAppliedOutput = driveAppliedOutput;
    this.angleAppliedOutput = angleAppliedOutput;
  }
}
