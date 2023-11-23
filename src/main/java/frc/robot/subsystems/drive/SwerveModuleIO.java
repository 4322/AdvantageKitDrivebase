package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.CANSparkMax.ControlType;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        public double drive1Position = 0.0;
        public double drive1VelocityRadPerSec = 0.0;
        public double drive1AppliedVolts = 0.0;
        public double drive1CurrentAmps = 0.0;

        public double drive2Position = 0.0;
        public double drive2VelocityRadPerSec = 0.0;
        public double drive2AppliedVolts = 0.0;
        public double drive2CurrentAmps = 0.0;

        public double turnVelocityDegPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};
        public double turnRotations = 0.0;
    }
    
    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void setDrivePIDTargetVel(VelocityVoltage request) {}

    public default void setTurnPIDReference(double value, ControlType ctrl) {}

    public default void setBrakeMode() {}

    public default void setCoastMode() {}

    public default void stopMotor() {}

}
