package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.CANSparkMax.ControlType;

public interface SwerveModuleIO {
    @AutoLog
    public class SwerveModuleIOInputs {
        public double driveRotations = 0.0;
        public double driveRotationsPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public double rotateRotations = 0.0;
        public double rotateRotationsPerSec = 0.0;
        public double rotateAppliedVolts = 0.0;
        public double rotateCurrentAmps = 0.0;
    }
    
    public default void updateInputs(SwerveModuleIOInputs inputs) {}

    public default void setDrivePIDTargetVel(VelocityVoltage request) {}

    public default void setTurnPIDReference(double value, ControlType ctrl) {}

    public default void setBrakeMode() {}

    public default void setCoastMode() {}

    public default void setClosedRampRate(double period) {}

    public default void setOpenRampRate(double period) {}

    public default void stopMotor() {}

}
