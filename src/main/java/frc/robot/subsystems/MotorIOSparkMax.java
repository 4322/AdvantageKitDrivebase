package frc.robot.subsystems;

import frc.util.CleanSparkMaxValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.util.Units;

public class MotorIOSparkMax implements MotorIO {

  /*
   * IO specific to the SparkMax. We'll need a different one of these for any
   * other piece of hardware we plan to use (e.g. another motor controller or a
   * simulated motor)
   */

  private final CANSparkMax sparkMax;
  private final RelativeEncoder encoder;

  public MotorIOSparkMax(int id) {
    sparkMax = new CANSparkMax(id, MotorType.kBrushless);
    encoder = sparkMax.getEncoder();
  }

  public void updateInputs(MotorIOInputs inputs) {
    inputs.drivePositionRad =
        CleanSparkMaxValue.cleanSparkMaxValue(
          inputs.drivePositionRad,
          Units.rotationsToRadians(encoder.getPosition())
        );
    inputs.driveVelocityRadPerSec =
        CleanSparkMaxValue.cleanSparkMaxValue(
          inputs.driveVelocityRadPerSec,
          Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity())
        );
    inputs.driveAppliedVolts = sparkMax.getAppliedOutput() * sparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {sparkMax.getOutputCurrent()};
    inputs.driveTempCelcius = new double[] {sparkMax.getMotorTemperature()};
  }

  public void setMotorSpeed(double percent) {
    sparkMax.set(percent);
  }
}
