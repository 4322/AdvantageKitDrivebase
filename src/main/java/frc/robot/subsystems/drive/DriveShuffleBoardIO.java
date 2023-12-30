package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveShuffleBoardIO {
  @AutoLog
  public static class DriveShuffleBoardIOInputs {
    public boolean psuedoAutoRotateEnabled;
    public String inputScaling;
    public String driveControllerType; 
    public double maxManualRotatePower;
    public double slowMovingAutoRotatePower;
    public double fastMovingAutoRotatePower;
    public double fastMovingFtPerSec;
    public double accelerationRampRate;
    public double stoppedRampRate;
    public double[] voltsAtMaxSpeed;
    public double[] feedForwardRPSThresholdEntry;
  }

  public default void updateInputs(DriveShuffleBoardIOInputs inputs) {}
}
