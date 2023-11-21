package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        public double drive1PositionRad = 0.0;
        public double drive1VelocityRadPerSec = 0.0;
        public double drive1AppliedVolts = 0.0;
        public double[] drive1CurrentAmps = new double[] {};
        public double[] drive1TempCelcius = new double[] {};

        public double drive2PositionRad = 0.0;
        public double drive2VelocityRadPerSec = 0.0;
        public double drive2AppliedVolts = 0.0;
        public double[] drive2CurrentAmps = new double[] {};
        public double[] drive2TempCelcius = new double[] {};

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};
        public double[] turnTempCelcius = new double[] {};
    }
    
    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void setDriveVoltage(double volts) {}
    
    public default void setTurnVoltage(double volts) {}
    
    public default void setDriveBrakeMode(boolean enable) {}

    public default void setTurnBrakeMode(boolean enable) {}



}
