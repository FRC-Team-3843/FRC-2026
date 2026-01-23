package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import swervelib.SwerveModule;

/**
 * Helper to read per-module motor telemetry across vendor APIs.
 *
 * <p>Uses NaN when a value is not available for the given motor type.
 */
public final class ModuleTelemetryReader {

  private ModuleTelemetryReader() {
  }

  public static ModuleTelemetryData read(SwerveModule[] modules) {
    int count = modules.length;
    double[] driveCurrentA = initArray(count);
    double[] angleCurrentA = initArray(count);
    double[] driveTempC = initArray(count);
    double[] angleTempC = initArray(count);
    double[] driveVoltageV = initArray(count);
    double[] angleVoltageV = initArray(count);
    double[] driveAppliedOutput = initArray(count);
    double[] angleAppliedOutput = initArray(count);

    for (int i = 0; i < count; i++) {
      var module = modules[i];
      fillMotorTelemetry(
          module.getDriveMotor().getMotor(),
          driveCurrentA,
          driveTempC,
          driveVoltageV,
          driveAppliedOutput,
          i);
      fillMotorTelemetry(
          module.getAngleMotor().getMotor(),
          angleCurrentA,
          angleTempC,
          angleVoltageV,
          angleAppliedOutput,
          i);
    }

    return new ModuleTelemetryData(
        driveCurrentA,
        angleCurrentA,
        driveTempC,
        angleTempC,
        driveVoltageV,
        angleVoltageV,
        driveAppliedOutput,
        angleAppliedOutput);
  }

  private static void fillMotorTelemetry(
      Object motor,
      double[] currentA,
      double[] tempC,
      double[] voltageV,
      double[] appliedOutput,
      int index) {
    if (motor instanceof SparkMax sparkMax) {
      currentA[index] = sparkMax.getOutputCurrent();
      tempC[index] = sparkMax.getMotorTemperature();
      voltageV[index] = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
      appliedOutput[index] = sparkMax.getAppliedOutput();
    } else if (motor instanceof TalonFX talonFX) {
      currentA[index] = talonFX.getSupplyCurrent()
          .getValue()
          .in(edu.wpi.first.units.Units.Amps);
      tempC[index] = talonFX.getDeviceTemp()
          .getValue()
          .in(edu.wpi.first.units.Units.Celsius);
      voltageV[index] = talonFX.getMotorVoltage()
          .getValue()
          .in(edu.wpi.first.units.Units.Volts);
      appliedOutput[index] = talonFX.getDutyCycle().getValue();
    }
  }

  private static double[] initArray(int count) {
    double[] values = new double[count];
    for (int i = 0; i < count; i++) {
      values[i] = Double.NaN;
    }
    return values;
  }
}
