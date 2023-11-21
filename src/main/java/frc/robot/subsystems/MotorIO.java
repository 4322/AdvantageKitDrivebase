package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

  /*
   * Generic IO class to log data between the hardware (in this case the motor
   * controller) and the software. A similar IO will be written for any hardware
   * interface we use.
   */

  @AutoLog
  public static class MotorIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};
    public double[] driveTempCelcius = new double[] {};
  }

  public default void updateInputs(MotorIOInputs inputs) {}

  public default void setMotorSpeed(double percent) {}
}
